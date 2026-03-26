"""Episode data recording for teleoperation.

Records camera images, robot states, actions, audio, and sim state to
structured JSON+image directories.  Each episode is stored under a
numbered sub-directory containing colour images, depth images, audio
numpy files, and a ``data.json`` manifest.  A background worker thread
handles all file I/O so that ``add_item`` calls never block the
real-time control loop.
"""

import os
import cv2
import json
import datetime
import numpy as np
import time
from .rerun_visualizer import RerunLogger
from queue import Queue, Empty
from threading import Thread
from typing import Any, Dict, List, Optional

import logging_mp
logger_mp = logging_mp.getLogger(__name__)


class EpisodeWriter():
    """Manages the lifecycle of episode recording: create, add items, save.

    An episode is a time-ordered sequence of data frames (items).  Each
    item may contain colour images, depth images, robot joint states,
    actions, tactile readings, audio samples, and simulation state.

    The writer uses a background ``Thread`` with a ``Queue`` so that
    image encoding and disk writes happen asynchronously.  The public
    API follows the pattern::

        writer.create_episode()
        for frame in frames:
            writer.add_item(colors=..., states=..., actions=...)
        writer.save_episode()

    Attributes:
        task_dir: Root directory under which episode sub-directories are
            created.
        text: Dictionary of task-level text annotations (goal, desc,
            steps).
        frequency: Target recording frequency in Hz.
        image_size: ``[width, height]`` of recorded images.
        rerun_log: Whether to mirror data to a Rerun viewer.
        rerun_logger: ``RerunLogger`` instance used for live
            visualisation (created when *rerun_log* is True).
        item_id: Zero-based index of the current item within the
            episode; ``-1`` before the first item.
        episode_id: Running episode counter across the task directory.
        is_available: ``True`` when the writer is ready to create a new
            episode (i.e. no episode is currently being recorded).
    """

    def __init__(
        self,
        task_dir: str,
        task_goal: Optional[str] = None,
        task_desc: Optional[str] = None,
        task_steps: Optional[str] = None,
        frequency: int = 30,
        image_size: List[int] = [640, 480],
        rerun_log: bool = True,
    ) -> None:
        """Initialise the episode writer.

        Args:
            task_dir: Filesystem path for storing episodes.  Created if
                it does not exist.
            task_goal: Short natural-language goal for the task.
            task_desc: Longer description of the task.
            task_steps: Semi-colon-separated steps describing the task.
            frequency: Target recording frame rate in Hz.
            image_size: ``[width, height]`` of images to record.
            rerun_log: If ``True``, initialise a ``RerunLogger`` and
                mirror each recorded item to it.
        """
        logger_mp.info("==> EpisodeWriter initializing...\n")
        self.task_dir: str = task_dir
        self.text: Dict[str, str] = {
            "goal": "Pick up the red cup on the table.",
            "desc": "task description",
            "steps":"step1: do this; step2: do that; ...",
        }
        if task_goal is not None:
            self.text['goal'] = task_goal
        if task_desc is not None:
            self.text['desc'] = task_desc
        if task_steps is not None:
            self.text['steps'] = task_steps

        self.frequency: int = frequency
        self.image_size: List[int] = image_size

        self.rerun_log: bool = rerun_log
        if self.rerun_log:
            logger_mp.info("==> RerunLogger initializing...\n")
            self.rerun_logger: RerunLogger = RerunLogger(prefix="online/", IdxRangeBoundary = 60, memory_limit = "300MB")
            logger_mp.info("==> RerunLogger initializing ok.\n")

        self.item_id: int = -1
        self.episode_id: int = -1
        if os.path.exists(self.task_dir):
            # Scan existing episode directories to determine the next episode ID
            episode_dirs = [episode_dir for episode_dir in os.listdir(self.task_dir) if 'episode_' in episode_dir and not episode_dir.endswith('.zip')]
            episode_last = sorted(episode_dirs)[-1] if len(episode_dirs) > 0 else None
            self.episode_id = 0 if episode_last is None else int(episode_last.split('_')[-1])
            logger_mp.info(f"==> task_dir directory already exist, now self.episode_id is:{self.episode_id}\n")
        else:
            os.makedirs(self.task_dir)
            logger_mp.info(f"==> episode directory does not exist, now create one.\n")
        self.data_info()

        self.is_available: bool = True  # Indicates whether the class is available for new operations
        # Initialize the queue and worker thread
        self.item_data_queue: Queue[Dict[str, Any]] = Queue(-1)
        self.stop_worker: bool = False
        self.need_save: bool = False  # Flag to indicate when save_episode is triggered
        self.worker_thread: Thread = Thread(target=self.process_queue)
        self.worker_thread.start()

        logger_mp.info("==> EpisodeWriter initialized successfully.\n")

    def is_ready(self) -> bool:
        """Check whether the writer is ready to create a new episode.

        Returns:
            ``True`` if no episode is currently being recorded or saved.
        """
        return self.is_available

    def data_info(
        self,
        version: Optional[str] = '1.0.0',
        date: Optional[str] = None,
        author: Optional[str] = None,
    ) -> None:
        """Populate the episode-level metadata dictionary.

        This dictionary is written at the top of every episode's
        ``data.json`` file and describes image/depth/audio formats as
        well as joint and tactile name lists.

        Args:
            version: Schema version string.
            date: Recording date (``YYYY-MM-DD``).  Defaults to today.
            author: Author/operator name.  Defaults to ``"unitree"``.
        """
        self.info: Dict[str, Any] = {
                "version": "1.0.0" if version is None else version,
                "date": datetime.date.today().strftime('%Y-%m-%d') if date is None else date,
                "author": "unitree" if author is None else author,
                "image": {"width":self.image_size[0], "height":self.image_size[1], "fps":self.frequency},
                "depth": {"width":self.image_size[0], "height":self.image_size[1], "fps":self.frequency},
                "audio": {"sample_rate": 16000, "channels": 1, "format":"PCM", "bits":16},    # PCM_S16
                "joint_names":{
                    "left_arm":   [],
                    "left_ee":  [],
                    "right_arm":  [],
                    "right_ee": [],
                    "body":       [],
                },

                "tactile_names": {
                    "left_ee": [],
                    "right_ee": [],
                },
                "sim_state": ""
            }


    def create_episode(self) -> bool:
        """Create a new episode directory and initialise its JSON file.

        Sets up colour, depth, and audio sub-directories, writes the
        ``info`` and ``text`` headers into ``data.json``, and marks the
        writer as unavailable until ``save_episode`` completes.

        Returns:
            ``True`` if the episode was successfully created, ``False``
            if the writer is still busy with a previous episode.

        Note:
            Once successfully created, this function will only be
            available again after ``save_episode`` completes its save
            task.
        """
        if not self.is_available:
            logger_mp.info("==> The class is currently unavailable for new operations. Please wait until ongoing tasks are completed.")
            return False  # Return False if the class is unavailable

        # Reset episode-related data and create necessary directories
        self.item_id = -1
        self.episode_id = self.episode_id + 1

        self.episode_dir: str = os.path.join(self.task_dir, f"episode_{str(self.episode_id).zfill(4)}")
        self.color_dir: str = os.path.join(self.episode_dir, 'colors')
        self.depth_dir: str = os.path.join(self.episode_dir, 'depths')
        self.audio_dir: str = os.path.join(self.episode_dir, 'audios')
        self.json_path: str = os.path.join(self.episode_dir, 'data.json')
        os.makedirs(self.episode_dir, exist_ok=True)
        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        os.makedirs(self.audio_dir, exist_ok=True)
        # Write JSON header (info + text), leaving the data array open
        with open(self.json_path, "w", encoding="utf-8") as f:
            f.write('{\n')
            f.write('"info": ' + json.dumps(self.info, ensure_ascii=False, indent=4) + ',\n')
            f.write('"text": ' + json.dumps(self.text, ensure_ascii=False, indent=4) + ',\n')
            f.write('"data": [\n')
        self.first_item: bool = True   # Flag to handle commas in JSON array

        if self.rerun_log:
            self.online_logger: RerunLogger = RerunLogger(prefix="online/", IdxRangeBoundary = 60, memory_limit="300MB")

        self.is_available = False  # After the episode is created, the class is marked as unavailable until the episode is successfully saved
        logger_mp.info(f"==> New episode created: {self.episode_dir}")
        return True  # Return True if the episode is successfully created

    def add_item(
        self,
        colors: Optional[Dict[str, Any]] = None,
        depths: Optional[Dict[str, Any]] = None,
        states: Optional[Dict[str, Any]] = None,
        actions: Optional[Dict[str, Any]] = None,
        tactiles: Optional[Dict[str, Any]] = None,
        audios: Optional[Dict[str, Any]] = None,
        sim_state: Optional[Any] = None,
    ) -> None:
        """Enqueue a single data frame for background writing.

        The item is placed on the internal queue and will be processed
        asynchronously by the worker thread.  Image encoding, file
        writes, and optional Rerun logging all happen off the calling
        thread.

        Args:
            colors: Mapping of camera name to BGR ``np.ndarray`` image.
            depths: Mapping of camera name to depth ``np.ndarray``.
            states: Robot joint state dictionaries keyed by body part.
            actions: Robot action dictionaries keyed by body part.
            tactiles: Tactile sensor data keyed by end-effector name.
            audios: Mapping of microphone name to audio ``np.ndarray``.
            sim_state: Arbitrary simulation state payload.
        """
        # Increment the item ID
        self.item_id += 1
        # Create the item data dictionary
        item_data: Dict[str, Any] = {
            'idx': self.item_id,
            'colors': colors,
            'depths': depths,
            'states': states,
            'actions': actions,
            'tactiles': tactiles,
            'audios': audios,
            'sim_state': sim_state,
        }
        # Enqueue the item data
        self.item_data_queue.put(item_data)

    def process_queue(self) -> None:
        """Background worker loop that drains the item queue.

        Runs in a dedicated ``Thread``.  Processes queued items via
        ``_process_item_data`` and triggers ``_save_episode`` when the
        ``need_save`` flag is set and the queue is empty.
        """
        while not self.stop_worker or not self.item_data_queue.empty():
            # Process items in the queue
            try:
                item_data: Dict[str, Any] = self.item_data_queue.get(timeout=1)
                try:
                    self._process_item_data(item_data)
                except Exception as e:
                    logger_mp.info(f"Error processing item_data (idx={item_data['idx']}): {e}")
                self.item_data_queue.task_done()
            except Empty:
                pass

            # Check if save_episode was triggered
            if self.need_save and self.item_data_queue.empty():
                self._save_episode()

    def _process_item_data(self, item_data: Dict[str, Any]) -> None:
        """Write a single item's images, audio, and JSON to disk.

        Colour and depth images are JPEG-encoded.  Audio arrays are
        saved as 16-bit PCM ``.npy`` files.  The item dictionary (with
        file paths replacing raw arrays) is appended to ``data.json``.

        Args:
            item_data: Dictionary with keys ``idx``, ``colors``,
                ``depths``, ``states``, ``actions``, ``tactiles``,
                ``audios``, and ``sim_state``.
        """
        idx: int = item_data['idx']
        colors: Dict[str, Any] = item_data.get('colors', {})
        depths: Dict[str, Any] = item_data.get('depths', {})
        audios: Dict[str, Any] = item_data.get('audios', {})

        # Save images
        if colors:
            for idx_color, (color_key, color) in enumerate(colors.items()):
                color_name: str = f'{str(idx).zfill(6)}_{color_key}.jpg'
                if not cv2.imwrite(os.path.join(self.color_dir, color_name), color):
                    logger_mp.info(f"Failed to save color image.")
                item_data['colors'][color_key] = os.path.join('colors', color_name)

        # Save depths
        if depths:
            for idx_depth, (depth_key, depth) in enumerate(depths.items()):
                depth_name: str = f'{str(idx).zfill(6)}_{depth_key}.jpg'
                if not cv2.imwrite(os.path.join(self.depth_dir, depth_name), depth):
                    logger_mp.info(f"Failed to save depth image.")
                item_data['depths'][depth_key] = os.path.join('depths', depth_name)

        # Save audios
        if audios:
            for mic, audio in audios.items():
                audio_name: str = f'audio_{str(idx).zfill(6)}_{mic}.npy'
                np.save(os.path.join(self.audio_dir, audio_name), audio.astype(np.int16))
                item_data['audios'][mic] = os.path.join('audios', audio_name)

        # Update episode data — append item JSON to the open data array
        with open(self.json_path, "a", encoding="utf-8") as f:
            if not self.first_item:
                f.write(",\n")
            f.write(json.dumps(item_data, ensure_ascii=False, indent=4))
            self.first_item = False

        # Log data if necessary
        if self.rerun_log:
            curent_record_time: float = time.time()
            logger_mp.info(f"==> episode_id:{self.episode_id}  item_id:{idx}  current_time:{curent_record_time}")
            self.rerun_logger.log_item_data(item_data)

    def save_episode(self) -> None:
        """Trigger the save operation.

        This sets an internal flag; the background worker thread will
        finish draining the queue and then finalise the JSON file.  The
        writer becomes available for a new episode once saving is done.
        """
        self.need_save = True  # Set the save flag
        logger_mp.info(f"==> Episode saved start...")

    def _save_episode(self) -> None:
        """Finalise the episode JSON file (called by the worker thread).

        Closes the JSON array and object, resets the save flag, and
        marks the writer as available for new episodes.
        """
        with open(self.json_path, "a", encoding="utf-8") as f:
            f.write("\n]\n}")      # Close the JSON array and object

        self.need_save = False     # Reset the save flag
        self.is_available = True   # Mark the class as available after saving
        logger_mp.info(f"==> Episode saved successfully to {self.json_path}.")

    def close(self) -> None:
        """Shut down the writer, flushing any pending data.

        Waits for the item queue to drain, triggers a save if an
        episode is still open, and joins the background worker thread.
        Must be called before the process exits to avoid data loss.
        """
        self.item_data_queue.join()
        if not self.is_available:  # If self.is_available is False, it means there is still data not saved.
            self.save_episode()
        while not self.is_available:
            time.sleep(0.01)
        self.stop_worker = True
        self.worker_thread.join()
