"""Controllers for Inspire DFX and FTP dexterous hands.

DFX uses single DDS topic for both hands. FTP uses separate left/right DDS
topics with inspire_sdkpy.
"""

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_                           # idl
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
from enum import IntEnum
import threading
import time
from multiprocessing import Process, Array, Lock

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

# Number of motors per hand: pinky, ring, middle, index, thumb-bend, thumb-rotation
Inspire_Num_Motors = 6

# DDS topic names for Inspire DFX (single topic carries both hands)
kTopicInspireDFXCommand = "rt/inspire/cmd"
kTopicInspireDFXState = "rt/inspire/state"

class Inspire_Controller_DFX:
    """6-DoF per hand controller for the Inspire DFX dexterous hand.

    Motors per hand: pinky, ring, middle, index, thumb-bend, thumb-rotation.
    Normalizes retargeted radian angles to [0, 1] range per motor using
    inverted mapping (``(max - val) / range``), where 0.0 = fully closed
    and 1.0 = fully open.

    Uses a single DDS topic for both hands. The controller spawns:
        - A daemon thread for subscribing to hand state DDS topics.
        - A daemon process for the retargeting control loop.

    Attributes:
        fps: Control loop frequency in Hz.
        Unit_Test: Whether unit-test retargeting config is used.
        simulation_mode: If True, skip real-robot safety guards.
        hand_retargeting: Retargeting engine mapping XR skeleton to motor space.
    """

    def __init__(
        self,
        left_hand_array: Array,
        right_hand_array: Array,
        dual_hand_data_lock: Lock | None = None,
        dual_hand_state_array: Array | None = None,
        dual_hand_action_array: Array | None = None,
        fps: float = 100.0,
        Unit_Test: bool = False,
        simulation_mode: bool = False,
    ) -> None:
        """Initializes the Inspire DFX hand controller.

        Sets up DDS publisher/subscriber for the single combined topic,
        starts the state subscription thread, waits for initial hand state,
        and spawns the control process.

        Args:
            left_hand_array: Shared array of left hand skeleton data
                (25 joints x 3 = 75 floats) from the XR device.
            right_hand_array: Shared array of right hand skeleton data
                (25 joints x 3 = 75 floats) from the XR device.
            dual_hand_data_lock: Synchronization lock protecting
                ``dual_hand_state_array`` and ``dual_hand_action_array``.
            dual_hand_state_array: Output shared array receiving
                left (6) + right (6) motor state positions.
            dual_hand_action_array: Output shared array receiving
                left (6) + right (6) motor action targets.
            fps: Control loop frequency in Hz.
            Unit_Test: Whether to load the unit-test retargeting configuration.
            simulation_mode: Whether to use simulation mode (default False
                means real robot).
        """
        logger_mp.info("Initialize Inspire_Controller_DFX...")
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.HandCmb_publisher = ChannelPublisher(kTopicInspireDFXCommand, MotorCmds_)
        self.HandCmb_publisher.Init()

        self.HandState_subscriber = ChannelSubscriber(kTopicInspireDFXState, MotorStates_)
        self.HandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Block until at least the right hand reports non-zero state
        while True:
            if any(self.right_hand_state_array): # any(self.left_hand_state_array) and
                break
            time.sleep(0.01)
            logger_mp.warning("[Inspire_Controller_DFX] Waiting to subscribe dds...")
        logger_mp.info("[Inspire_Controller_DFX] Subscribe dds ok.")

        # Spawn the retargeting control process as a daemon
        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_DFX OK!")

    def _subscribe_hand_state(self) -> None:
        """Continuously reads hand state from the single DDS topic and updates shared arrays.

        Runs in a daemon thread. Polls the combined hand state subscriber at
        ~500 Hz and copies each motor's position (``q``) into the left and
        right shared array slots using the respective joint-index enums.
        """
        while True:
            hand_msg  = self.HandState_subscriber.Read()
            if hand_msg is not None:
                for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
                    self.left_hand_state_array[idx] = hand_msg.states[id].q
                for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
                    self.right_hand_state_array[idx] = hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target: np.ndarray, right_q_target: np.ndarray) -> None:
        """Publishes normalized [0,1] position targets for both hands via a single DDS topic.

        Args:
            left_q_target: Array of 6 normalized target positions for the
                left hand (0.0 = closed, 1.0 = open).
            right_q_target: Array of 6 normalized target positions for the
                right hand (0.0 = closed, 1.0 = open).
        """
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = left_q_target[idx]
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = right_q_target[idx]

        self.HandCmb_publisher.Write(self.hand_msg)
        # logger_mp.debug("hand ctrl publish ok.")

    def control_process(
        self,
        left_hand_array: Array,
        right_hand_array: Array,
        left_hand_state_array: Array,
        right_hand_state_array: Array,
        dual_hand_data_lock: Lock | None = None,
        dual_hand_state_array: Array | None = None,
        dual_hand_action_array: Array | None = None,
    ) -> None:
        """Multiprocessing target that runs the retargeting loop at configured fps.

        Reads XR hand skeleton data from shared arrays, computes retargeted
        motor positions via dex-retargeting, normalizes radian values to [0,1]
        per motor, optionally writes state/action data to output shared arrays,
        and publishes motor commands.

        Args:
            left_hand_array: Shared array with left hand skeleton
                (25 joints x 3 = 75 floats).
            right_hand_array: Shared array with right hand skeleton
                (25 joints x 3 = 75 floats).
            left_hand_state_array: Shared array of 6 current left motor states.
            right_hand_state_array: Shared array of 6 current right motor states.
            dual_hand_data_lock: Lock protecting the output shared arrays.
            dual_hand_state_array: Output array for left+right motor states (12).
            dual_hand_action_array: Output array for left+right motor actions (12).
        """
        self.running = True

        # Default targets: fully open (1.0)
        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        # initialize inspire hand's cmd msg
        self.hand_msg  = MotorCmds_()
        self.hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Inspire_Right_Hand_JointIndex) + len(Inspire_Left_Hand_JointIndex))]

        # Set initial positions to fully open (1.0) for all motors
        for idx, id in enumerate(Inspire_Left_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0
        for idx, id in enumerate(Inspire_Right_Hand_JointIndex):
            self.hand_msg.cmds[id].q = 1.0

        def normalize(val: float, min_val: float, max_val: float) -> float:
            """Normalizes a radian value to [0, 1] with inverted mapping.

            Args:
                val: Raw retargeted joint angle in radians.
                min_val: Minimum expected radian value.
                max_val: Maximum expected radian value.

            Returns:
                Normalized value clipped to [0.0, 1.0], where
                max_val maps to 0.0 and min_val maps to 1.0.
            """
            return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                # Skip retargeting if hand data has not been initialized yet
                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    # Compute reference vectors (bone directions) for retargeting
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    # Retarget XR skeleton to motor joint positions (radians)
                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range

                    # Normalize each motor's radian value to [0, 1] using its specific range
                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            # Finger motors (pinky, ring, middle, index): range [0, 1.7] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            # Thumb bend motor: range [0, 0.5] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            # Thumb rotation motor: range [-0.1, 1.3] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)

                # Maintain target loop rate
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_DFX has been closed.")



# DDS topic names for Inspire FTP (separate topics per hand)
kTopicInspireFTPLeftCommand   = "rt/inspire_hand/ctrl/l"
kTopicInspireFTPRightCommand  = "rt/inspire_hand/ctrl/r"
kTopicInspireFTPLeftState  = "rt/inspire_hand/state/l"
kTopicInspireFTPRightState = "rt/inspire_hand/state/r"

class Inspire_Controller_FTP:
    """6-DoF per hand controller for the Inspire FTP dexterous hand.

    Same motor layout as DFX (pinky, ring, middle, index, thumb-bend,
    thumb-rotation) but uses the FTP protocol with separate left/right DDS
    topics via ``inspire_sdkpy``. Commands are scaled to [0-1000] integer
    range before publishing.

    The controller spawns:
        - A daemon thread for subscribing to hand state DDS topics.
        - A daemon process for the retargeting control loop.

    Attributes:
        fps: Control loop frequency in Hz.
        Unit_Test: Whether unit-test retargeting config is used.
        simulation_mode: If True, skip real-robot safety guards.
        hand_retargeting: Retargeting engine mapping XR skeleton to motor space.
    """

    def __init__(
        self,
        left_hand_array: Array,
        right_hand_array: Array,
        dual_hand_data_lock: Lock | None = None,
        dual_hand_state_array: Array | None = None,
        dual_hand_action_array: Array | None = None,
        fps: float = 100.0,
        Unit_Test: bool = False,
        simulation_mode: bool = False,
    ) -> None:
        """Initializes the Inspire FTP hand controller.

        Lazily imports ``inspire_sdkpy``, sets up separate left/right DDS
        publishers/subscribers, starts the state subscription thread, waits
        for initial hand state (with timeout), and spawns the control process.

        Args:
            left_hand_array: Shared array of left hand skeleton data
                (25 joints x 3 = 75 floats) from the XR device.
            right_hand_array: Shared array of right hand skeleton data
                (25 joints x 3 = 75 floats) from the XR device.
            dual_hand_data_lock: Synchronization lock protecting
                ``dual_hand_state_array`` and ``dual_hand_action_array``.
            dual_hand_state_array: Output shared array receiving
                left (6) + right (6) motor state positions.
            dual_hand_action_array: Output shared array receiving
                left (6) + right (6) motor action targets.
            fps: Control loop frequency in Hz.
            Unit_Test: Whether to load the unit-test retargeting configuration.
            simulation_mode: Whether to use simulation mode (default False
                means real robot).
        """
        logger_mp.info("Initialize Inspire_Controller_FTP...")
        from inspire_sdkpy import inspire_dds  # lazy import
        import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default
        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND_Unit_Test)


        # Initialize hand command publishers
        self.LeftHandCmd_publisher = ChannelPublisher(kTopicInspireFTPLeftCommand, inspire_dds.inspire_hand_ctrl)
        self.LeftHandCmd_publisher.Init()
        self.RightHandCmd_publisher = ChannelPublisher(kTopicInspireFTPRightCommand, inspire_dds.inspire_hand_ctrl)
        self.RightHandCmd_publisher.Init()

        # Initialize hand state subscribers
        self.LeftHandState_subscriber = ChannelSubscriber(kTopicInspireFTPLeftState, inspire_dds.inspire_hand_state)
        self.LeftHandState_subscriber.Init() # Consider using callback if preferred: Init(callback_func, period_ms)
        self.RightHandState_subscriber = ChannelSubscriber(kTopicInspireFTPRightState, inspire_dds.inspire_hand_state)
        self.RightHandState_subscriber.Init()

        # Shared Arrays for hand states ([0,1] normalized values)
        self.left_hand_state_array  = Array('d', Inspire_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Inspire_Num_Motors, lock=True)

        # Initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Wait for initial DDS messages (optional, but good for ensuring connection)
        wait_count = 0
        while not (any(self.left_hand_state_array) or any(self.right_hand_state_array)):
            if wait_count % 100 == 0: # Print every second
                logger_mp.info(f"[Inspire_Controller_FTP] Waiting to subscribe to hand states from DDS (L: {any(self.left_hand_state_array)}, R: {any(self.right_hand_state_array)})...")
            time.sleep(0.01)
            wait_count += 1
            if wait_count > 500: # Timeout after 5 seconds
                logger_mp.warning("[Inspire_Controller_FTP] Warning: Timeout waiting for initial hand states. Proceeding anyway.")
                break
        logger_mp.info("[Inspire_Controller_FTP] Initial hand states received or timeout.")

        # Spawn the retargeting control process as a daemon
        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array, self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Inspire_Controller_FTP OK!\n")

    def _subscribe_hand_state(self) -> None:
        """Continuously reads left/right hand state from separate FTP DDS topics.

        Runs in a daemon thread. Polls both hand state subscribers at ~500 Hz.
        Reads ``angle_act`` from each message (list of 6 integer values in
        [0, 1000]) and divides by 1000.0 to store normalized [0, 1] values
        in the shared arrays. Logs warnings if the message format is unexpected.
        """
        logger_mp.info("[Inspire_Controller_FTP] Subscribe thread started.")
        while True:
            # Left Hand
            left_state_msg = self.LeftHandState_subscriber.Read()
            if left_state_msg is not None:
                if hasattr(left_state_msg, 'angle_act') and len(left_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.left_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            # Convert [0, 1000] integer range to [0.0, 1.0] float
                            self.left_hand_state_array[i] = left_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received left_state_msg but attributes are missing or incorrect. Type: {type(left_state_msg)}, Content: {str(left_state_msg)[:100]}")
            # Right Hand
            right_state_msg = self.RightHandState_subscriber.Read()
            if right_state_msg is not None:
                if hasattr(right_state_msg, 'angle_act') and len(right_state_msg.angle_act) == Inspire_Num_Motors:
                    with self.right_hand_state_array.get_lock():
                        for i in range(Inspire_Num_Motors):
                            # Convert [0, 1000] integer range to [0.0, 1.0] float
                            self.right_hand_state_array[i] = right_state_msg.angle_act[i] / 1000.0
                else:
                    logger_mp.warning(f"[Inspire_Controller_FTP] Received right_state_msg but attributes are missing or incorrect. Type: {type(right_state_msg)}, Content: {str(right_state_msg)[:100]}")

            time.sleep(0.002)

    def _send_hand_command(self, left_angle_cmd_scaled: list[int], right_angle_cmd_scaled: list[int]) -> None:
        """Sends scaled angle commands [0-1000] to both hands via FTP protocol.

        Constructs command messages using ``inspire_hand_default``, sets
        angle mode (mode bit 0), and publishes to the left and right DDS
        topics. Logs the first 50 commands for debugging.

        Args:
            left_angle_cmd_scaled: List of 6 integer motor positions in
                [0, 1000] for the left hand.
            right_angle_cmd_scaled: List of 6 integer motor positions in
                [0, 1000] for the right hand.
        """
        from inspire_sdkpy import inspire_dds  # lazy import
        import inspire_sdkpy.inspire_hand_defaut as inspire_hand_default

        # Left Hand Command
        left_cmd_msg = inspire_hand_default.get_inspire_hand_ctrl()
        left_cmd_msg.angle_set = left_angle_cmd_scaled
        left_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.LeftHandCmd_publisher.Write(left_cmd_msg)

        # Right Hand Command
        right_cmd_msg = inspire_hand_default.get_inspire_hand_ctrl()
        right_cmd_msg.angle_set = right_angle_cmd_scaled
        right_cmd_msg.mode = 0b0001 # Mode 1: Angle control
        self.RightHandCmd_publisher.Write(right_cmd_msg)

        # Temporarily log the first N commands for debugging
        if not hasattr(self, "_debug_count"):
            self._debug_count = 0
        if self._debug_count < 50:
            logger_mp.info(f"[Inspire_Controller_FTP] Publish cmd L={left_angle_cmd_scaled} R={right_angle_cmd_scaled} ")
            self._debug_count += 1


    def control_process(
        self,
        left_hand_array: Array,
        right_hand_array: Array,
        left_hand_state_array: Array,
        right_hand_state_array: Array,
        dual_hand_data_lock: Lock | None = None,
        dual_hand_state_array: Array | None = None,
        dual_hand_action_array: Array | None = None,
    ) -> None:
        """Multiprocessing target that runs the FTP retargeting loop at configured fps.

        Reads XR hand skeleton data from shared arrays, computes retargeted
        motor positions via dex-retargeting, normalizes radian values to [0,1]
        per motor, scales to [0-1000] integers for the FTP protocol, optionally
        writes state/action data to output shared arrays, and publishes
        commands via ``_send_hand_command``.

        Args:
            left_hand_array: Shared array with left hand skeleton
                (25 joints x 3 = 75 floats).
            right_hand_array: Shared array with right hand skeleton
                (25 joints x 3 = 75 floats).
            left_hand_state_array: Shared array of 6 current left motor states.
            right_hand_state_array: Shared array of 6 current right motor states.
            dual_hand_data_lock: Lock protecting the output shared arrays.
            dual_hand_state_array: Output array for left+right motor states (12).
            dual_hand_action_array: Output array for left+right motor actions (12).
        """
        logger_mp.info("[Inspire_Controller_FTP] Control process started.")
        self.running = True

        # Default targets: fully open (1.0)
        left_q_target  = np.full(Inspire_Num_Motors, 1.0)
        right_q_target = np.full(Inspire_Num_Motors, 1.0)

        def normalize(val: float, min_val: float, max_val: float) -> float:
            """Normalizes a radian value to [0, 1] with inverted mapping.

            Args:
                val: Raw retargeted joint angle in radians.
                min_val: Minimum expected radian value.
                max_val: Maximum expected radian value.

            Returns:
                Normalized value clipped to [0.0, 1.0], where
                max_val maps to 0.0 and min_val maps to 1.0.
            """
            return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array.get_lock():
                    left_hand_data  = np.array(left_hand_array[:]).reshape(25, 3).copy()
                with right_hand_array.get_lock():
                    right_hand_data = np.array(right_hand_array[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                # Skip retargeting if hand data has not been initialized yet
                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    # Compute reference vectors (bone directions) for retargeting
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    # Retarget XR skeleton to motor joint positions (radians)
                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # Normalize each motor's radian value to [0, 1] using its specific range
                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            # Finger motors (pinky, ring, middle, index): range [0, 1.7] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            # Thumb bend motor: range [0, 0.5] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            # Thumb rotation motor: range [-0.1, 1.3] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)

                # Scale normalized [0, 1] float to [0, 1000] integer for FTP protocol
                scaled_left_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in left_q_target]
                scaled_right_cmd = [int(np.clip(val * 1000, 0, 1000)) for val in right_q_target]

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data

                self._send_hand_command(scaled_left_cmd, scaled_right_cmd)

                # Maintain target loop rate
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Inspire_Controller_FTP has been closed.")

# Update hand state, according to the official documentation:
# 1. https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand
# 2. https://support.unitree.com/home/en/G1_developer/inspire_ftp_dexterity_hand
# the state sequence is as shown in the table below
# ┌──────┬───────┬──────┬────────┬────────┬────────────┬────────────────┬───────┬──────┬────────┬────────┬────────────┬────────────────┐
# │ Id   │   0   │  1   │   2    │   3    │     4      │       5        │   6   │  7   │   8    │   9    │    10      │       11       │
# ├──────┼───────┼──────┼────────┼────────┼────────────┼────────────────┼───────┼──────┼────────┼────────┼────────────┼────────────────┤
# │      │                    Right Hand                                │                   Left Hand                                  │
# │Joint │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │ pinky │ ring │ middle │ index  │ thumb-bend │ thumb-rotation │
# └──────┴───────┴──────┴────────┴────────┴────────────┴────────────────┴───────┴──────┴────────┴────────┴────────────┴────────────────┘
class Inspire_Right_Hand_JointIndex(IntEnum):
    """Motor ID enum for the Inspire right hand following Unitree documentation order.

    IDs 0-5 correspond to the right hand motors in the combined DDS message.
    Order: pinky, ring, middle, index, thumb-bend, thumb-rotation.

    Members:
        kRightHandPinky: Pinky finger motor (id 0).
        kRightHandRing: Ring finger motor (id 1).
        kRightHandMiddle: Middle finger motor (id 2).
        kRightHandIndex: Index finger motor (id 3).
        kRightHandThumbBend: Thumb bend motor (id 4).
        kRightHandThumbRotation: Thumb rotation motor (id 5).
    """
    kRightHandPinky = 0
    kRightHandRing = 1
    kRightHandMiddle = 2
    kRightHandIndex = 3
    kRightHandThumbBend = 4
    kRightHandThumbRotation = 5

class Inspire_Left_Hand_JointIndex(IntEnum):
    """Motor ID enum for the Inspire left hand following Unitree documentation order.

    IDs 6-11 correspond to the left hand motors in the combined DDS message.
    Order: pinky, ring, middle, index, thumb-bend, thumb-rotation.

    Members:
        kLeftHandPinky: Pinky finger motor (id 6).
        kLeftHandRing: Ring finger motor (id 7).
        kLeftHandMiddle: Middle finger motor (id 8).
        kLeftHandIndex: Index finger motor (id 9).
        kLeftHandThumbBend: Thumb bend motor (id 10).
        kLeftHandThumbRotation: Thumb rotation motor (id 11).
    """
    kLeftHandPinky = 6
    kLeftHandRing = 7
    kLeftHandMiddle = 8
    kLeftHandIndex = 9
    kLeftHandThumbBend = 10
    kLeftHandThumbRotation = 11
