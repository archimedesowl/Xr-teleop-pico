"""ZMQ-based inter-process communication for teleoperation.

Provides REQ/REP command channel and PUB/SUB heartbeat monitoring
between the teleop process and external clients (e.g. a web UI or
keyboard controller).

Protocol overview
-----------------

**Client -> Server (Request)**::

    1) launch   {"reqid": <uuid>, "cmd": "CMD_START"}
    2) exit     {"reqid": <uuid>, "cmd": "CMD_STOP"}
    3) toggle   {"reqid": <uuid>, "cmd": "CMD_RECORD_TOGGLE"}

**Server -> Client (Reply)**::

    OK:    {"repid": <reqid>, "status": "ok",    "msg": "ok"}
    Error: {"repid": <reqid|0|1>, "status": "error", "msg": "..."}

**Heartbeat (PUB)**::

    {"START": bool, "STOP": bool,
     "RECORD_RUNNING": bool, "RECORD_READY": bool}
"""

import os
import zmq
import time
import threading
from typing import Any, Callable, Dict, Optional

import logging_mp
logger_mp = logging_mp.getLogger(__name__)


class IPC_Server:
    """Inter-Process Communication server.

    Runs two daemon threads:

    * **Data loop** -- listens on a ``REP`` socket for JSON command
      messages, dispatches them via the *on_press* callback, and sends
      back a reply.
    * **Heartbeat loop** -- periodically publishes the current state
      (obtained via *get_state*) on a ``PUB`` socket so that clients
      can detect liveness.

    Attributes:
        cmd_map: Mapping from protocol command strings to the
            single-character keys expected by the *on_press* callback.
        on_press: Callback invoked with a single-character key string
            whenever a valid command is received.
        get_state: Callback returning the current heartbeat state
            dictionary.
    """

    # Mapping table for on_press keys
    cmd_map: Dict[str, str] = {
        "CMD_START": "r",          # launch
        "CMD_STOP": "q",           # exit
        "CMD_RECORD_TOGGLE": "s",  # start & stop (toggle record)
    }

    def __init__(
        self,
        on_press: Optional[Callable[[str], None]] = None,
        get_state: Optional[Callable[[], Dict[str, Any]]] = None,
        hb_fps: float = 10.0,
    ) -> None:
        """Initialise the IPC server.

        Args:
            on_press: Callback ``(cmd: str) -> None`` invoked for every
                valid command received on the REP socket.  The *cmd*
                argument is the single-character key from ``cmd_map``.
            get_state: Callback ``() -> dict`` that returns the current
                heartbeat state dictionary to publish.
            hb_fps: Heartbeat publish frequency in Hz.

        Raises:
            ValueError: If *on_press* or *get_state* is not callable.
        """
        if callable(on_press):
            self.on_press: Callable[[str], None] = on_press
        else:
            raise ValueError("[IPC_Server] on_press callback function must be provided")

        if callable(get_state):
            self.get_state: Callable[[], Dict[str, Any]] = get_state
        else:
            raise ValueError("[IPC_Server] get_state callback function must be provided")
        self._hb_interval: float = 1.0 / float(hb_fps)
        self._running: bool = True
        self._data_loop_thread: Optional[threading.Thread] = None
        self._hb_loop_thread: Optional[threading.Thread] = None

        # Data IPC (REQ/REP)
        self.ctx: zmq.Context = zmq.Context.instance()
        self.rep_socket: zmq.Socket = self.ctx.socket(zmq.REP)
        self.rep_socket.bind("ipc://@xr_teleoperate_data.ipc")
        logger_mp.info("[IPC_Server] Listening to Data at ipc://@xr_teleoperate_data.ipc")

        # heartbeat IPC (PUB/SUB)
        self.pub_socket: zmq.Socket = self.ctx.socket(zmq.PUB)
        self.pub_socket.bind("ipc://@xr_teleoperate_hb.ipc")
        logger_mp.info("[IPC_Server] Publishing HeartBeat at ipc://@xr_teleoperate_hb.ipc")

    def _data_loop(self) -> None:
        """Listen for REQ/REP commands and optional info.

        Polls the ``REP`` socket with a 20 ms timeout and dispatches
        incoming JSON messages through ``_handle_message``.  Runs until
        ``_running`` is set to ``False`` or the ZMQ context is
        terminated.
        """
        poller: zmq.Poller = zmq.Poller()
        poller.register(self.rep_socket, zmq.POLLIN)
        while self._running:
            try:
                socks: Dict[zmq.Socket, int] = dict(poller.poll(20))
                if self.rep_socket in socks:
                    msg: Dict[str, Any] = self.rep_socket.recv_json()
                    reply: Dict[str, Any] = self._handle_message(msg)
                    try:
                        self.rep_socket.send_json(reply)
                    except Exception as e:
                        logger_mp.error(f"[IPC_Server] Failed to send reply: {e}")
                    finally:
                        logger_mp.debug(f"[IPC_Server] DATA recv: {msg} -> rep: {reply}")
            except zmq.error.ContextTerminated:
                break
            except Exception as e:
                logger_mp.error(f"[IPC_Server] Data loop exception: {e}")

    def _hb_loop(self) -> None:
        """Publish heartbeat periodically.

        Calls ``get_state`` to obtain the current status dictionary
        and publishes it as JSON on the ``PUB`` socket.  Sleeps for the
        remainder of each heartbeat interval to maintain the target
        frequency.
        """
        while self._running:
            start_time: float = time.monotonic()
            try:
                state: Dict[str, Any] = dict(self.get_state() or {})
                self.pub_socket.send_json(state)
                logger_mp.debug(f"[IPC_Server] HB pub: {state}")
            except Exception as e:
                logger_mp.error(f"[IPC_Server] HeartBeat loop exception: {e}")
            elapsed: float = time.monotonic() - start_time
            if elapsed < self._hb_interval:
                time.sleep(self._hb_interval - elapsed)

    def _handle_message(self, msg: dict) -> dict:
        """Process an incoming command message and return a reply.

        Validates the ``reqid`` and ``cmd`` fields, checks that the
        command is in ``cmd_map``, and invokes the ``on_press`` callback
        with the mapped key character.

        Args:
            msg: Incoming JSON message dictionary.

        Returns:
            Reply dictionary with ``repid``, ``status``, and ``msg``
            fields.
        """
        try:
            # validate reqid
            reqid: Optional[str] = msg.get("reqid", None)
            if not reqid:
                return {"repid": 0, "status": "error", "msg": "reqid not provided"}

            # validate cmd
            cmd: Optional[str] = msg.get("cmd", None)
            if not cmd:
                return {"repid": reqid, "status": "error", "msg": "cmd not provided"}

            # unsupported cmd
            if cmd not in self.cmd_map:
                return {"repid": reqid, "status": "error", "msg": f"cmd not supported: {cmd}"}

            # supported cmd path
            self.on_press(self.cmd_map[cmd])
            return {"repid": reqid, "status": "ok", "msg": "ok"}

        except Exception as e:
            return {"repid": 1, "status": "error", "msg": str(e)}

    # ---------------------------
    # Public API
    # ---------------------------
    def start(self) -> None:
        """Start both the data loop and heartbeat loop daemon threads."""
        self._data_loop_thread = threading.Thread(target=self._data_loop, daemon=True)
        self._data_loop_thread.start()
        self._hb_loop_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_loop_thread.start()

    def stop(self) -> None:
        """Stop the server and release all ZMQ resources.

        Sets the running flag to ``False``, joins both daemon threads,
        and closes the REP and PUB sockets before terminating the ZMQ
        context.
        """
        self._running = False
        if self._data_loop_thread:
            self._data_loop_thread.join(timeout=1.0)
        if self._hb_loop_thread:
            self._hb_loop_thread.join(timeout=1.0)
        try:
            self.rep_socket.setsockopt(zmq.LINGER, 0)
            self.rep_socket.close()
        except Exception:
            pass
        try:
            self.pub_socket.setsockopt(zmq.LINGER, 0)
            self.pub_socket.close()
        except Exception:
            pass
        try:
            self.ctx.term()
        except Exception:
            pass


class IPC_Client:
    """Inter-Process Communication client.

    Connects to an ``IPC_Server`` via two channels:

    * **Command channel (REQ)** -- sends JSON commands and waits for
      the server's reply.
    * **Heartbeat channel (SUB)** -- subscribes to the server's
      periodic PUB messages to track liveness.

    The heartbeat subscriber runs in a background daemon thread.  The
    server is considered *online* after three consecutive heartbeat
    messages are received and *offline* after a configurable timeout
    with no messages.

    Attributes:
        ctx: Shared ZMQ context.
        req_socket: ``REQ`` socket for sending commands.
        sub_socket: ``SUB`` socket for receiving heartbeats.
    """

    def __init__(self, hb_fps: float = 10.0) -> None:
        """Initialise the IPC client.

        Args:
            hb_fps: Expected heartbeat frequency in Hz.  Used to
                calculate the poll interval and offline timeout.
        """
        self.ctx: zmq.Context = zmq.Context.instance()

        # heartbeat IPC (PUB/SUB)
        self._hb_running: bool = True
        self._hb_last_time: float = 0           # timestamp of last heartbeat received
        self._hb_latest_state: Dict[str, Any] = {}       # latest heartbeat state
        self._hb_online: bool = False          # whether heartbeat is online
        self._hb_interval: float = 1.0 / float(hb_fps)     # expected heartbeat interval
        self._hb_lock: threading.Lock = threading.Lock()            # lock for heartbeat state
        self._hb_timeout: float = 5.0 * self._hb_interval  # timeout to consider offline

        self.sub_socket: zmq.Socket = self.ctx.socket(zmq.SUB)
        self.sub_socket.setsockopt(zmq.RCVHWM, 1)
        self.sub_socket.connect("ipc://@xr_teleoperate_hb.ipc")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        logger_mp.info("[IPC_Client] Subscribed to HeartBeat at ipc://@xr_teleoperate_hb.ipc")

        self._hb_thread: threading.Thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()

        # data IPC (REQ/REP)
        self.req_socket: zmq.Socket = self.ctx.socket(zmq.REQ)
        self.req_socket.connect("ipc://@xr_teleoperate_data.ipc")
        logger_mp.info("[IPC_Client] Connected to Data at ipc://@xr_teleoperate_data.ipc")

    def _make_reqid(self) -> str:
        """Generate a unique request identifier.

        Returns:
            UUID4 string.
        """
        import uuid
        return str(uuid.uuid4())

    # ---------------------------
    # Heartbeat handling
    # ---------------------------
    def _hb_loop(self) -> None:
        """Background heartbeat subscriber loop.

        Non-blocking reads from the SUB socket.  After three
        consecutive successful receives the server is marked *online*.
        If no heartbeat is received within ``_hb_timeout`` seconds the
        server is marked *offline* and the state is cleared.
        """
        consecutive: int = 0
        while self._hb_running:
            start_time: float = time.monotonic()
            try:
                msg: Dict[str, Any] = self.sub_socket.recv_json(flags=zmq.NOBLOCK)
                with self._hb_lock:
                    self._hb_latest_state = msg
                    self._hb_last_time = time.monotonic()
                    consecutive += 1
                    if consecutive >= 3:  # require 3 consecutive heartbeats to be considered online
                        self._hb_online = True
            except zmq.Again:
                with self._hb_lock:
                    if self._hb_last_time > 0:
                        if self._hb_online and (time.monotonic() - self._hb_last_time > self._hb_timeout):
                            self._hb_latest_state = {}
                            self._hb_last_time = 0
                            self._hb_online = False
                            consecutive = 0
                            logger_mp.warning("[IPC_Client] HeartBeat timeout -> OFFLINE")
            except Exception as e:
                logger_mp.error(f"[IPC_Client] HB loop exception: {e}")
            elapsed: float = time.monotonic() - start_time
            if elapsed < self._hb_interval:
                time.sleep(self._hb_interval - elapsed)

    # ---------------------------
    # Public API
    # ---------------------------
    def send_data(self, cmd: str) -> dict:
        """Send a command to the server and wait for the reply.

        Checks that the server is online (via heartbeat) before
        attempting to send.  Waits up to 1 second for the server's
        reply.

        Args:
            cmd: Command string (e.g. ``"CMD_START"``,
                ``"CMD_STOP"``, ``"CMD_RECORD_TOGGLE"``).

        Returns:
            Reply dictionary with ``repid``, ``status``, and ``msg``
            keys.  On failure, ``status`` is ``"error"``.
        """
        reqid: str = self._make_reqid()
        if not self.is_online():
            logger_mp.warning(f"[IPC_Client] Cannot send {cmd}, server offline (no heartbeat)")
            return {"repid": reqid, "status": "error", "msg": "server offline (no heartbeat)"}

        msg: Dict[str, str] = {"reqid": reqid, "cmd": cmd}
        try:
            self.req_socket.send_json(msg)
            # wait up to 1s for reply
            if self.req_socket.poll(1000):
                reply: Dict[str, Any] = self.req_socket.recv_json()
            else:
                return {"repid": reqid, "status": "error", "msg": "timeout waiting for server reply"}
        except Exception as e:
            logger_mp.error(f"[IPC_Client] send_data failed: {e}")
            return {"repid": reqid, "status": "error", "msg": str(e)}

        if reply.get("status") != "ok":
            return reply
        if reply.get("repid") != reqid:
            return {"repid": reqid, "status": "error", "msg": f"reply id mismatch: expected {reqid}, got {reply.get('repid')}"}
        return reply

    def is_online(self) -> bool:
        """Check whether the server is reachable via heartbeat.

        Returns:
            ``True`` if at least three consecutive heartbeats have been
            received and no timeout has occurred since.
        """
        with self._hb_lock:
            return self._hb_online

    def latest_state(self) -> dict:
        """Return the most recently received heartbeat state.

        Returns:
            Copy of the last heartbeat dictionary, or an empty dict
            if no heartbeat has been received.
        """
        with self._hb_lock:
            return dict(self._hb_latest_state)

    def stop(self) -> None:
        """Stop the client and release all ZMQ resources.

        Joins the heartbeat thread and closes REQ and SUB sockets
        before terminating the ZMQ context.
        """
        self._hb_running = False
        if self._hb_thread:
            self._hb_thread.join(timeout=1.0)
        try:
            self.req_socket.setsockopt(zmq.LINGER, 0)
            self.req_socket.close()
        except Exception:
            pass
        try:
            self.sub_socket.setsockopt(zmq.LINGER, 0)
            self.sub_socket.close()
        except Exception:
            pass
        try:
            self.ctx.term()
        except Exception:
            pass


# ---------------------------
# Client Example usage
# ---------------------------
if __name__ == "__main__":
    from sshkeyboard import listen_keyboard, stop_listening
    client = None

    def on_press(key: str):
        global client
        if client is None:
            logger_mp.warning("⚠️ Client not initialized, ignoring key press")
            return

        if key == "r":
            logger_mp.info("▶️ Sending launch command...")
            rep = client.send_data("CMD_START")
            logger_mp.info("Reply: %s", rep)

        elif key == "s":
            logger_mp.info("⏺️ Sending record toggle command...")
            rep = client.send_data("CMD_RECORD_TOGGLE")
            logger_mp.info("Reply: %s", rep)


        elif key == "q":
            logger_mp.info("⏹️ Sending exit command...")
            rep = client.send_data("CMD_STOP")
            logger_mp.info("Reply: %s", rep)

        elif key == "b":
            if client.is_online():
                state = client.latest_state()
                logger_mp.info(f"[HEARTBEAT] Current heartbeat: {state}")
            else:
                logger_mp.warning("[HEARTBEAT] No heartbeat received (OFFLINE)")

        else:
            logger_mp.warning(f"⚠️ Undefined key: {key}")

    # Initialize client
    client = IPC_Client(hb_fps=10.0)

    # Start keyboard listening thread
    listen_keyboard_thread = threading.Thread(target=listen_keyboard, kwargs={"on_press": on_press, "until": None, "sequential": False}, daemon=True)
    listen_keyboard_thread.start()

    logger_mp.info("✅ Client started, waiting for keyboard input:\n [r] launch, [s] start/stop record, [b] heartbeat, [q] exit")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logger_mp.info("⏹️ User interrupt, preparing to exit...")
    finally:
        stop_listening()
        client.stop()
        logger_mp.info("✅ Client exited")
