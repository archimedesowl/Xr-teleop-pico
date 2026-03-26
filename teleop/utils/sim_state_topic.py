# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Simple sim state subscriber class.

Subscribe to the ``rt/sim_state_cmd`` topic via the Unitree SDK DDS
channel and write received JSON payloads to POSIX shared memory so that
other processes (e.g. the main teleop loop) can read the latest
simulation state without DDS coupling.
"""

import threading
import time
import json
from multiprocessing import shared_memory
from typing import Any, Dict, Optional

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

import logging_mp
logger_mp = logging_mp.getLogger(__name__)


class SharedMemoryManager:
    """Manages a fixed-size POSIX shared memory segment for JSON data.

    The segment layout is:

    * **Bytes 0-3** -- 32-bit little-endian UNIX timestamp (seconds).
    * **Bytes 4-7** -- 32-bit little-endian data length in bytes.
    * **Bytes 8+** -- UTF-8 encoded JSON payload.

    A reentrant lock serialises concurrent reads and writes within the
    same process.

    Attributes:
        size: Total size of the shared memory segment in bytes.
        lock: Reentrant thread lock protecting buffer access.
        shm: Underlying ``shared_memory.SharedMemory`` object.
        shm_name: Name of the shared memory segment.
        created: ``True`` if this instance created the segment (and is
            therefore responsible for unlinking it).
    """

    def __init__(self, name: Optional[str] = None, size: int = 512) -> None:
        """Initialize shared memory manager.

        Args:
            name: Shared memory name.  If ``None``, a new anonymous
                segment is created.  If provided and no segment with
                that name exists, a new one is created.
            size: Shared memory size in bytes.
        """
        self.size: int = size
        self.lock: threading.RLock = threading.RLock()  # reentrant lock

        if name:
            try:
                self.shm: shared_memory.SharedMemory = shared_memory.SharedMemory(name=name)
                self.shm_name: str = name
                self.created: bool = False
            except FileNotFoundError:
                # Segment does not exist yet -- create it
                self.shm = shared_memory.SharedMemory(create=True, size=size)
                self.shm_name = self.shm.name
                self.created = True
        else:
            self.shm = shared_memory.SharedMemory(create=True, size=size)
            self.shm_name = self.shm.name
            self.created = True

    def write_data(self, data: Dict[str, Any]) -> bool:
        """Write a JSON-serialisable dictionary to shared memory.

        The data is prefixed with a 4-byte timestamp and a 4-byte
        length header.

        Args:
            data: Dictionary to serialise and store.

        Returns:
            ``True`` on success, ``False`` if the serialised data
            exceeds the available buffer space.
        """
        try:
            with self.lock:
                json_str: str = json.dumps(data)
                json_bytes: bytes = json_str.encode('utf-8')

                if len(json_bytes) > self.size - 8:  # reserve 8 bytes for length and timestamp
                    logger_mp.warning(f"Data too large for shared memory ({len(json_bytes)} > {self.size - 8})")
                    return False

                # write timestamp (4 bytes) and data length (4 bytes)
                timestamp: int = int(time.time()) & 0xFFFFFFFF  # 32-bit timestamp, use bitmask to ensure in range
                self.shm.buf[0:4] = timestamp.to_bytes(4, 'little')
                self.shm.buf[4:8] = len(json_bytes).to_bytes(4, 'little')

                # write data
                self.shm.buf[8:8+len(json_bytes)] = json_bytes
                return True

        except Exception as e:
            logger_mp.error(f"Error writing to shared memory: {e}")
            return False

    def read_data(self) -> Optional[Dict[str, Any]]:
        """Read the latest JSON dictionary from shared memory.

        Returns:
            Parsed dictionary with an additional ``_timestamp`` key, or
            ``None`` if no data has been written yet or an error occurs.
        """
        try:
            with self.lock:
                # read timestamp and data length
                timestamp: int = int.from_bytes(self.shm.buf[0:4], 'little')
                data_len: int = int.from_bytes(self.shm.buf[4:8], 'little')

                if data_len == 0:
                    return None

                # read data
                json_bytes: bytes = bytes(self.shm.buf[8:8+data_len])
                data: Dict[str, Any] = json.loads(json_bytes.decode('utf-8'))
                data['_timestamp'] = timestamp  # add timestamp information
                return data

        except Exception as e:
            logger_mp.error(f"Error reading from shared memory: {e}")
            return None

    def get_name(self) -> str:
        """Get the shared memory segment name.

        Returns:
            POSIX shared memory name string.
        """
        return self.shm_name

    def cleanup(self) -> None:
        """Close and optionally unlink the shared memory segment.

        Unlinking is only performed if this instance created the
        segment (``self.created is True``).
        """
        if hasattr(self, 'shm') and self.shm:
            self.shm.close()
            if self.created:
                try:
                    self.shm.unlink()
                except FileNotFoundError:
                    pass

    def __del__(self) -> None:
        """Destructor -- delegates to ``cleanup``."""
        self.cleanup()


class SimStateSubscriber:
    """Subscribes to the ``rt/sim_state`` DDS topic and mirrors data to shared memory.

    The subscriber runs a background daemon thread that polls the DDS
    channel at ~500 Hz and writes each received JSON payload into a
    ``SharedMemoryManager`` segment.

    Attributes:
        shm_name: Name of the shared memory segment.
        shm_size: Size of the shared memory segment in bytes.
        running: ``True`` while the subscribe loop is active.
        subscriber: Unitree ``ChannelSubscriber`` instance.
        subscribe_thread: Background daemon thread running the
            subscribe loop.
        shared_memory: ``SharedMemoryManager`` used for cross-process
            data sharing.
    """

    def __init__(self, shm_name: str = "sim_state_cmd_data", shm_size: int = 4096) -> None:
        """Initialize the subscriber.

        Args:
            shm_name: Name for the POSIX shared memory segment.
            shm_size: Size of the shared memory segment in bytes.
        """
        self.shm_name: str = shm_name
        self.shm_size: int = shm_size
        self.running: bool = False
        self.subscriber: Optional[ChannelSubscriber] = None
        self.subscribe_thread: Optional[threading.Thread] = None
        self.shared_memory: Optional[SharedMemoryManager] = None

        # initialize shared memory
        self._setup_shared_memory()

        logger_mp.debug(f"[SimStateSubscriber] Initialized with shared memory: {shm_name}")

    def _setup_shared_memory(self) -> None:
        """Create the ``SharedMemoryManager`` instance.

        Logs an error and leaves ``self.shared_memory`` as ``None`` if
        creation fails.
        """
        try:
            self.shared_memory = SharedMemoryManager(self.shm_name, self.shm_size)
            logger_mp.debug(f"[SimStateSubscriber] Shared memory setup successfully")
        except Exception as e:
            logger_mp.error(f"[SimStateSubscriber] Failed to setup shared memory: {e}")

    def start_subscribe(self) -> None:
        """Start the DDS subscriber and background polling thread.

        Does nothing if the subscriber is already running.
        """
        if self.running:
            logger_mp.warning(f"[SimStateSubscriber] Already running")
            return

        try:
            self.subscriber = ChannelSubscriber("rt/sim_state", String_)
            self.subscriber.Init()
            self.running = True

            self.subscribe_thread = threading.Thread(target=self._subscribe_sim_state, daemon=True)
            self.subscribe_thread.start()

            logger_mp.info(f"[SimStateSubscriber] Started subscribing to rt/sim_state")

        except Exception as e:
            logger_mp.error(f"[SimStateSubscriber] Failed to start subscribing: {e}")
            self.running = False

    def _subscribe_sim_state(self) -> None:
        """Subscribe loop thread that reads DDS messages and writes to shared memory.

        Polls the DDS channel every 2 ms.  Received JSON strings are
        parsed and forwarded to the shared memory manager.
        """
        logger_mp.debug(f"[SimStateSubscriber] Subscribe thread started")

        while self.running:
            try:
                if self.subscriber:
                    msg: Optional[String_] = self.subscriber.Read()
                    if msg:
                        data: Dict[str, Any] = json.loads(msg.data)
                        if self.shared_memory and data:
                            self.shared_memory.write_data(data)
                    else:
                        logger_mp.warning("[SimStateSubscriber] Received None message")
                else:
                    logger_mp.error("[SimStateSubscriber] Subscriber is not initialized")
                time.sleep(0.002)
            except Exception as e:
                logger_mp.error(f"[SimStateSubscriber] Error in subscribe loop: {e}")
                time.sleep(0.01)

    def stop_subscribe(self) -> None:
        """Stop the subscriber and clean up shared memory.

        Joins the background thread (with a 1-second timeout) and
        releases the shared memory segment.
        """
        if not self.running:
            logger_mp.warning(f"[SimStateSubscriber] Already stopped or not running")
            return

        self.running = False
        # wait for thread to finish
        if self.subscribe_thread:
            self.subscribe_thread.join(timeout=1.0)

        if self.shared_memory:
            self.shared_memory.cleanup()
        logger_mp.info(f"[SimStateSubscriber] Subscriber stopped")

    def read_data(self) -> Optional[Dict[str, Any]]:
        """Read the latest sim state from shared memory.

        Returns:
            Parsed dictionary with a ``_timestamp`` key, or ``None`` if
            no data is available or an error occurs.
        """
        try:
            if self.shared_memory:
                return self.shared_memory.read_data()
            return None
        except Exception as e:
            logger_mp.error(f"[SimStateSubscriber] Error reading data: {e}")
            return None

    def is_running(self) -> bool:
        """Check whether the subscribe loop is active.

        Returns:
            ``True`` if the subscriber is currently running.
        """
        return self.running


def start_sim_state_subscribe(shm_name: str = "sim_state_cmd_data", shm_size: int = 4096) -> SimStateSubscriber:
    """Convenience factory: create and start a ``SimStateSubscriber``.

    Args:
        shm_name: POSIX shared memory segment name.
        shm_size: Shared memory size in bytes.

    Returns:
        A ``SimStateSubscriber`` instance that is already subscribing.
    """
    subscriber: SimStateSubscriber = SimStateSubscriber(shm_name, shm_size)
    subscriber.start_subscribe()
    return subscriber


# if __name__ == "__main__":
#     # example usage
#     logger_mp.info("Starting sim state subscriber...")
#     ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation
#     # create and start subscriber
#     subscriber = start_sim_state_subscribe()

#     try:
#         # keep running and check for data
#         while True:
#             data = subscriber.read_data()
#             if data:
#                 logger_mp.info(f"Read data: {data}")
#             time.sleep(1)

#     except KeyboardInterrupt:
#         logger_mp.warning("\nInterrupted by user")
#     finally:
#         subscriber.stop_subscribe()
#         logger_mp.info("Subscriber stopped")
