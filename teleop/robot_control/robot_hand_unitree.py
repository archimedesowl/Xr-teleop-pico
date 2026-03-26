"""DDS-based controllers for Unitree Dex3-1 dexterous hand and Dex1-1 gripper.

Handles hand state subscription, retargeting from XR hand tracking data,
and motor command publishing.
"""

# DDS imports (shared by Dex3-1 and Dex1-1)
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
# Dex3-1 IDL
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
# Dex1-1 gripper IDL
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

import numpy as np
from enum import IntEnum
import time
import os
import sys
import threading
from multiprocessing import Process, Array, Value, Lock

parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
from teleop.utils.weighted_moving_filter import WeightedMovingFilter

import logging_mp
logger_mp = logging_mp.getLogger(__name__)


# Number of motors in each Dex3-1 hand (3 thumb joints + 2 index + 2 middle)
Dex3_Num_Motors = 7

# DDS topic names for Dex3-1 hand command and state channels
kTopicDex3LeftCommand = "rt/dex3/left/cmd"
kTopicDex3RightCommand = "rt/dex3/right/cmd"
kTopicDex3LeftState = "rt/dex3/left/state"
kTopicDex3RightState = "rt/dex3/right/state"


class Dex3_1_Controller:
    """7-DoF per hand dexterous hand controller for the Unitree Dex3-1.

    Takes XR hand skeleton data (25 joints x 3D positions), retargets to
    7 motor positions per hand via dex-retargeting, and publishes motor
    commands over DDS.

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
        left_hand_array_in: Array,
        right_hand_array_in: Array,
        dual_hand_data_lock: Lock | None = None,
        dual_hand_state_array_out: Array | None = None,
        dual_hand_action_array_out: Array | None = None,
        fps: float = 100.0,
        Unit_Test: bool = False,
        simulation_mode: bool = False,
    ) -> None:
        """Initializes the Dex3-1 hand controller.

        Sets up DDS publishers/subscribers, starts the state subscription
        thread, waits for initial hand state, and spawns the control process.

        Note:
            A ``*_array`` type parameter requires using a multiprocessing
            Array, because it needs to be passed to the internal child process.

        Args:
            left_hand_array_in: Shared array of left hand skeleton data
                (25 joints x 3 = 75 floats) from the XR device.
            right_hand_array_in: Shared array of right hand skeleton data
                (25 joints x 3 = 75 floats) from the XR device.
            dual_hand_data_lock: Synchronization lock protecting
                ``dual_hand_state_array_out`` and ``dual_hand_action_array_out``.
            dual_hand_state_array_out: Output shared array receiving
                left (7) + right (7) motor state positions.
            dual_hand_action_array_out: Output shared array receiving
                left (7) + right (7) motor action targets.
            fps: Control loop frequency in Hz.
            Unit_Test: Whether to load the unit-test retargeting configuration.
            simulation_mode: Whether to use simulation mode (default False
                means real robot).
        """
        logger_mp.info("Initialize Dex3_1_Controller...")

        self.fps = fps
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode
        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3)
        else:
            self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3_Unit_Test)

        # initialize handcmd publisher and handstate subscriber
        self.LeftHandCmb_publisher = ChannelPublisher(kTopicDex3LeftCommand, HandCmd_)
        self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicDex3RightCommand, HandCmd_)
        self.RightHandCmb_publisher.Init()

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicDex3LeftState, HandState_)
        self.LeftHandState_subscriber.Init()
        self.RightHandState_subscriber = ChannelSubscriber(kTopicDex3RightState, HandState_)
        self.RightHandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', Dex3_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', Dex3_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Block until both hands have reported non-zero state
        while True:
            if any(self.left_hand_state_array) and any(self.right_hand_state_array):
                break
            time.sleep(0.01)
            logger_mp.warning("[Dex3_1_Controller] Waiting to subscribe dds...")
        logger_mp.info("[Dex3_1_Controller] Subscribe dds ok.")

        # Spawn the retargeting control process as a daemon
        hand_control_process = Process(target=self.control_process, args=(left_hand_array_in, right_hand_array_in,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array_out, dual_hand_action_array_out))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize Dex3_1_Controller OK!")

    def _subscribe_hand_state(self) -> None:
        """Continuously reads left/right hand state from DDS and updates shared arrays.

        Runs in a daemon thread. Polls both hand state subscribers at ~500 Hz
        and copies each motor's position (``q``) into the corresponding shared
        array slot using the joint-index enums.
        """
        while True:
            left_hand_msg  = self.LeftHandState_subscriber.Read()
            right_hand_msg = self.RightHandState_subscriber.Read()
            if left_hand_msg is not None and right_hand_msg is not None:
                # Update left hand state
                for idx, id in enumerate(Dex3_1_Left_JointIndex):
                    self.left_hand_state_array[idx] = left_hand_msg.motor_state[id].q
                # Update right hand state
                for idx, id in enumerate(Dex3_1_Right_JointIndex):
                    self.right_hand_state_array[idx] = right_hand_msg.motor_state[id].q
            time.sleep(0.002)

    class _RIS_Mode:
        """Encodes motor mode byte combining motor id, status, and timeout fields.

        The mode byte layout (8 bits) is:
            - Bits [0:3]: motor id (4 bits, 0-15)
            - Bits [4:6]: status code (3 bits, 0-7)
            - Bit  [7]:   timeout flag (1 bit)

        Attributes:
            motor_mode: The assembled 8-bit mode value.
            id: Motor identifier (0-15).
            status: Motor status code (0-7), default 0x01 (enabled).
            timeout: Timeout flag (0 or 1).
        """

        def __init__(self, id: int = 0, status: int = 0x01, timeout: int = 0) -> None:
            """Initializes the RIS mode encoder.

            Args:
                id: Motor identifier, masked to 4 bits.
                status: Motor status code, masked to 3 bits.
                timeout: Timeout flag, masked to 1 bit.
            """
            self.motor_mode = 0
            self.id = id & 0x0F  # 4 bits for id
            self.status = status & 0x07  # 3 bits for status
            self.timeout = timeout & 0x01  # 1 bit for timeout

        def _mode_to_uint8(self) -> int:
            """Packs id, status, and timeout into a single uint8 mode byte.

            Returns:
                The assembled 8-bit motor mode value.
            """
            self.motor_mode |= (self.id & 0x0F)
            self.motor_mode |= (self.status & 0x07) << 4
            self.motor_mode |= (self.timeout & 0x01) << 7
            return self.motor_mode

    def ctrl_dual_hand(self, left_q_target: np.ndarray, right_q_target: np.ndarray) -> None:
        """Publishes position targets to both hands over DDS.

        Iterates over joint-index enums for each hand, sets the target
        position (``q``) on the pre-allocated command messages, and writes
        them to the respective DDS publishers.

        Args:
            left_q_target: Array of 7 target joint positions (radians) for the
                left hand.
            right_q_target: Array of 7 target joint positions (radians) for the
                right hand.
        """
        for idx, id in enumerate(Dex3_1_Left_JointIndex):
            self.left_msg.motor_cmd[id].q = left_q_target[idx]
        for idx, id in enumerate(Dex3_1_Right_JointIndex):
            self.right_msg.motor_cmd[id].q = right_q_target[idx]

        self.LeftHandCmb_publisher.Write(self.left_msg)
        self.RightHandCmb_publisher.Write(self.right_msg)
        # logger_mp.debug("hand ctrl publish ok.")

    def control_process(
        self,
        left_hand_array_in: Array,
        right_hand_array_in: Array,
        left_hand_state_array: Array,
        right_hand_state_array: Array,
        dual_hand_data_lock: Lock | None = None,
        dual_hand_state_array_out: Array | None = None,
        dual_hand_action_array_out: Array | None = None,
    ) -> None:
        """Multiprocessing target that runs the retargeting loop at configured fps.

        Reads XR hand skeleton data from shared arrays, computes retargeted
        motor positions via dex-retargeting, optionally writes state/action
        data to output shared arrays, and publishes motor commands.

        Args:
            left_hand_array_in: Shared array with left hand skeleton
                (25 joints x 3 = 75 floats).
            right_hand_array_in: Shared array with right hand skeleton
                (25 joints x 3 = 75 floats).
            left_hand_state_array: Shared array of 7 current left motor states.
            right_hand_state_array: Shared array of 7 current right motor states.
            dual_hand_data_lock: Lock protecting the output shared arrays.
            dual_hand_state_array_out: Output array for left+right motor states (14).
            dual_hand_action_array_out: Output array for left+right motor actions (14).
        """
        self.running = True

        left_q_target  = np.full(Dex3_Num_Motors, 0)
        right_q_target = np.full(Dex3_Num_Motors, 0)

        # PD control gains for motor commands
        q = 0.0
        dq = 0.0
        tau = 0.0
        kp = 1.5
        kd = 0.2

        # initialize dex3-1's left hand cmd msg
        self.left_msg  = unitree_hg_msg_dds__HandCmd_()
        for id in Dex3_1_Left_JointIndex:
            ris_mode = self._RIS_Mode(id = id, status = 0x01)
            motor_mode = ris_mode._mode_to_uint8()
            self.left_msg.motor_cmd[id].mode = motor_mode
            self.left_msg.motor_cmd[id].q    = q
            self.left_msg.motor_cmd[id].dq   = dq
            self.left_msg.motor_cmd[id].tau  = tau
            self.left_msg.motor_cmd[id].kp   = kp
            self.left_msg.motor_cmd[id].kd   = kd

        # initialize dex3-1's right hand cmd msg
        self.right_msg = unitree_hg_msg_dds__HandCmd_()
        for id in Dex3_1_Right_JointIndex:
            ris_mode = self._RIS_Mode(id = id, status = 0x01)
            motor_mode = ris_mode._mode_to_uint8()
            self.right_msg.motor_cmd[id].mode = motor_mode
            self.right_msg.motor_cmd[id].q    = q
            self.right_msg.motor_cmd[id].dq   = dq
            self.right_msg.motor_cmd[id].tau  = tau
            self.right_msg.motor_cmd[id].kp   = kp
            self.right_msg.motor_cmd[id].kd   = kd

        try:
            while self.running:
                start_time = time.time()
                # get dual hand state
                with left_hand_array_in.get_lock():
                    left_hand_data  = np.array(left_hand_array_in[:]).reshape(25, 3).copy()
                with right_hand_array_in.get_lock():
                    right_hand_data = np.array(right_hand_array_in[:]).reshape(25, 3).copy()

                # Read left and right q_state from shared arrays
                state_data = np.concatenate((np.array(left_hand_state_array[:]), np.array(right_hand_state_array[:])))

                # Skip retargeting if hand data has not been initialized yet
                if not np.all(right_hand_data == 0.0) and not np.all(left_hand_data[4] == np.array([-1.13, 0.3, 0.15])): # if hand data has been initialized.
                    # Compute reference vectors (bone directions) for retargeting
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    # Retarget XR skeleton to motor joint positions
                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array_out and dual_hand_action_array_out:
                    with dual_hand_data_lock:
                        dual_hand_state_array_out[:] = state_data
                        dual_hand_action_array_out[:] = action_data

                self.ctrl_dual_hand(left_q_target, right_q_target)

                # Maintain target loop rate
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Dex3_1_Controller has been closed.")

class Dex3_1_Left_JointIndex(IntEnum):
    """Motor ID enum for the Dex3-1 left hand.

    Maps logical joint names to motor indices used in ``HandCmd_`` and
    ``HandState_`` message arrays for the left hand.

    Members:
        kLeftHandThumb0: Thumb proximal joint (id 0).
        kLeftHandThumb1: Thumb middle joint (id 1).
        kLeftHandThumb2: Thumb distal joint (id 2).
        kLeftHandMiddle0: Middle finger proximal joint (id 3).
        kLeftHandMiddle1: Middle finger distal joint (id 4).
        kLeftHandIndex0: Index finger proximal joint (id 5).
        kLeftHandIndex1: Index finger distal joint (id 6).
    """
    kLeftHandThumb0 = 0
    kLeftHandThumb1 = 1
    kLeftHandThumb2 = 2
    kLeftHandMiddle0 = 3
    kLeftHandMiddle1 = 4
    kLeftHandIndex0 = 5
    kLeftHandIndex1 = 6

class Dex3_1_Right_JointIndex(IntEnum):
    """Motor ID enum for the Dex3-1 right hand.

    Maps logical joint names to motor indices used in ``HandCmd_`` and
    ``HandState_`` message arrays for the right hand.

    Members:
        kRightHandThumb0: Thumb proximal joint (id 0).
        kRightHandThumb1: Thumb middle joint (id 1).
        kRightHandThumb2: Thumb distal joint (id 2).
        kRightHandIndex0: Index finger proximal joint (id 3).
        kRightHandIndex1: Index finger distal joint (id 4).
        kRightHandMiddle0: Middle finger proximal joint (id 5).
        kRightHandMiddle1: Middle finger distal joint (id 6).
    """
    kRightHandThumb0 = 0
    kRightHandThumb1 = 1
    kRightHandThumb2 = 2
    kRightHandIndex0 = 3
    kRightHandIndex1 = 4
    kRightHandMiddle0 = 5
    kRightHandMiddle1 = 6


# DDS topic names for Dex1-1 gripper command and state channels
kTopicGripperLeftCommand = "rt/dex1/left/cmd"
kTopicGripperLeftState = "rt/dex1/left/state"
kTopicGripperRightCommand = "rt/dex1/right/cmd"
kTopicGripperRightState = "rt/dex1/right/state"

class Dex1_1_Gripper_Controller:
    """Simple 1-DoF per hand gripper controller for the Unitree Dex1-1.

    Maps a pinch/trigger scalar value from the XR device to a gripper motor
    position, with velocity limiting (delta clamp) and optional weighted
    moving average smoothing filter.

    The controller uses a threading-based control loop (not multiprocessing)
    because only one motor per hand needs to be driven.

    Attributes:
        fps: Control loop frequency in Hz.
        Unit_Test: Whether unit-test mode is active.
        gripper_sub_ready: Set True once the DDS subscriber has read at least once.
        simulation_mode: If True, skip velocity-limiting clamp.
        smooth_filter: Optional weighted moving average filter for smoothing
            gripper commands, or None if disabled.
    """

    def __init__(
        self,
        left_gripper_value_in: Value,
        right_gripper_value_in: Value,
        dual_gripper_data_lock: Lock | None = None,
        dual_gripper_state_out: Array | None = None,
        dual_gripper_action_out: Array | None = None,
        filter: bool = True,
        fps: float = 200.0,
        Unit_Test: bool = False,
        simulation_mode: bool = False,
    ) -> None:
        """Initializes the Dex1-1 gripper controller.

        Sets up DDS publishers/subscribers for both grippers, starts the
        state subscription thread, waits for initial data, and spawns the
        control thread.

        Note:
            A ``*_array`` type parameter requires using a multiprocessing
            Array, because it needs to be passed to the internal child process.

        Args:
            left_gripper_value_in: Shared Value with left gripper input
                (pinch distance or trigger value) from the XR device.
            right_gripper_value_in: Shared Value with right gripper input
                (pinch distance or trigger value) from the XR device.
            dual_gripper_data_lock: Synchronization lock protecting
                ``dual_gripper_state_out`` and ``dual_gripper_action_out``.
            dual_gripper_state_out: Output shared array receiving
                left (1) + right (1) gripper motor states.
            dual_gripper_action_out: Output shared array receiving
                left (1) + right (1) gripper motor actions.
            filter: Whether to enable the weighted moving average smoothing
                filter on gripper commands.
            fps: Control loop frequency in Hz.
            Unit_Test: Whether to enable unit testing mode.
            simulation_mode: Whether to use simulation mode (default False
                means real robot).
        """

        logger_mp.info("Initialize Dex1_1_Gripper_Controller...")

        self.fps = fps
        self.Unit_Test = Unit_Test
        self.gripper_sub_ready = False
        self.simulation_mode = simulation_mode

        # Set up optional smoothing filter (disabled in simulation to avoid lag)
        if filter and not self.simulation_mode:
            self.smooth_filter = WeightedMovingFilter(np.array([0.5, 0.3, 0.2]), 2)
        else:
            self.smooth_filter = None

        # initialize handcmd publisher and handstate subscriber
        self.LeftGripperCmb_publisher = ChannelPublisher(kTopicGripperLeftCommand, MotorCmds_)
        self.LeftGripperCmb_publisher.Init()
        self.RightGripperCmb_publisher = ChannelPublisher(kTopicGripperRightCommand, MotorCmds_)
        self.RightGripperCmb_publisher.Init()

        self.LeftGripperState_subscriber = ChannelSubscriber(kTopicGripperLeftState, MotorStates_)
        self.LeftGripperState_subscriber.Init()
        self.RightGripperState_subscriber = ChannelSubscriber(kTopicGripperRightState, MotorStates_)
        self.RightGripperState_subscriber.Init()

        # Shared Arrays for gripper states
        self.left_gripper_state_value = Value('d', 0.0, lock=True)
        self.right_gripper_state_value = Value('d', 0.0, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_gripper_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Block until the subscriber has performed at least one read cycle
        while not self.gripper_sub_ready:
            time.sleep(0.01)
            logger_mp.warning("[Dex1_1_Gripper_Controller] Waiting to subscribe dds...")
        logger_mp.info("[Dex1_1_Gripper_Controller] Subscribe dds ok.")

        self.gripper_control_thread = threading.Thread(target=self.control_thread, args=(left_gripper_value_in, right_gripper_value_in, self.left_gripper_state_value, self.right_gripper_state_value,
                                                                                         dual_gripper_data_lock, dual_gripper_state_out, dual_gripper_action_out))
        self.gripper_control_thread.daemon = True
        self.gripper_control_thread.start()

        logger_mp.info("Initialize Dex1_1_Gripper_Controller OK!")

    def _subscribe_gripper_state(self) -> None:
        """Continuously reads left/right gripper state from DDS.

        Runs in a daemon thread. Polls both gripper state subscribers at
        ~500 Hz and copies each gripper motor position (``q``) into the
        corresponding shared Value. Sets ``gripper_sub_ready`` on the
        first read attempt regardless of whether messages were received.
        """
        while True:
            left_gripper_msg  = self.LeftGripperState_subscriber.Read()
            right_gripper_msg  = self.RightGripperState_subscriber.Read()
            self.gripper_sub_ready = True
            if left_gripper_msg is not None and right_gripper_msg is not None:
                self.left_gripper_state_value.value = left_gripper_msg.states[0].q
                self.right_gripper_state_value.value = right_gripper_msg.states[0].q
            time.sleep(0.002)

    def ctrl_dual_gripper(self, dual_gripper_action: np.ndarray) -> None:
        """Publishes gripper position targets to both grippers over DDS.

        Args:
            dual_gripper_action: Array of shape (2,) containing
                [left_gripper_q, right_gripper_q] target positions in radians.
        """
        self.left_gripper_msg.cmds[0].q  = dual_gripper_action[0]
        self.right_gripper_msg.cmds[0].q = dual_gripper_action[1]

        self.LeftGripperCmb_publisher.Write(self.left_gripper_msg)
        self.RightGripperCmb_publisher.Write(self.right_gripper_msg)
        # logger_mp.debug("gripper ctrl publish ok.")

    def control_thread(
        self,
        left_gripper_value_in: Value,
        right_gripper_value_in: Value,
        left_gripper_state_value: Value,
        right_gripper_state_value: Value,
        dual_hand_data_lock: Lock | None = None,
        dual_gripper_state_out: Array | None = None,
        dual_gripper_action_out: Array | None = None,
    ) -> None:
        """Threading target for the gripper control loop.

        Maps XR pinch/trigger values to gripper motor positions via linear
        interpolation, applies velocity-limiting delta clamp (unless in
        simulation mode), optionally smooths with a weighted moving filter,
        and publishes commands at the configured fps.

        Args:
            left_gripper_value_in: Shared Value with left gripper input
                (pinch distance or trigger value).
            right_gripper_value_in: Shared Value with right gripper input
                (pinch distance or trigger value).
            left_gripper_state_value: Shared Value with current left gripper
                motor position.
            right_gripper_state_value: Shared Value with current right gripper
                motor position.
            dual_hand_data_lock: Lock protecting the output shared arrays.
            dual_gripper_state_out: Output array for left+right gripper states (2).
            dual_gripper_action_out: Output array for left+right gripper actions (2).
        """
        self.running = True
        DELTA_GRIPPER_CMD = 0.18     # The motor rotates 5.4 radians, the clamping jaw slide open 9 cm, so 0.6 rad <==> 1 cm, 0.18 rad <==> 3 mm
        THUMB_INDEX_DISTANCE_MIN = 5.0
        THUMB_INDEX_DISTANCE_MAX = 7.0
        LEFT_MAPPED_MIN  = 0.0           # The minimum initial motor position when the gripper closes at startup.
        RIGHT_MAPPED_MIN = 0.0           # The minimum initial motor position when the gripper closes at startup.
        # The maximum initial motor position when the gripper closes before calibration (with the rail stroke calculated as 0.6 cm/rad * 9 rad = 5.4 cm).
        LEFT_MAPPED_MAX = LEFT_MAPPED_MIN + 5.40
        RIGHT_MAPPED_MAX = RIGHT_MAPPED_MIN + 5.40
        # Start at midpoint of the gripper range
        left_target_action  = (LEFT_MAPPED_MAX - LEFT_MAPPED_MIN) / 2.0
        right_target_action = (RIGHT_MAPPED_MAX - RIGHT_MAPPED_MIN) / 2.0

        # PD control gains for gripper motor commands
        dq = 0.0
        tau = 0.0
        kp = 5.00
        kd = 0.05
        # initialize gripper cmd msg
        self.left_gripper_msg  = MotorCmds_()
        self.left_gripper_msg.cmds = [unitree_go_msg_dds__MotorCmd_()]
        self.right_gripper_msg = MotorCmds_()
        self.right_gripper_msg.cmds = [unitree_go_msg_dds__MotorCmd_()]

        self.left_gripper_msg.cmds[0].dq  = dq
        self.left_gripper_msg.cmds[0].tau = tau
        self.left_gripper_msg.cmds[0].kp  = kp
        self.left_gripper_msg.cmds[0].kd  = kd

        self.right_gripper_msg.cmds[0].dq  = dq
        self.right_gripper_msg.cmds[0].tau = tau
        self.right_gripper_msg.cmds[0].kp  = kp
        self.right_gripper_msg.cmds[0].kd  = kd
        try:
            while self.running:
                start_time = time.time()
                # get dual hand skeletal point state from XR device
                with left_gripper_value_in.get_lock():
                    left_gripper_value  = left_gripper_value_in.value
                with right_gripper_value_in.get_lock():
                    right_gripper_value = right_gripper_value_in.value
                # get current dual gripper motor state
                dual_gripper_state = np.array([left_gripper_state_value.value, right_gripper_state_value.value])

                # Skip mapping if input data has not been initialized yet
                if left_gripper_value != 0.0 or right_gripper_value != 0.0: # if input data has been initialized.
                    # Linear mapping from [0, THUMB_INDEX_DISTANCE_MAX] to gripper action range
                    left_target_action  = np.interp(left_gripper_value, [THUMB_INDEX_DISTANCE_MIN, THUMB_INDEX_DISTANCE_MAX], [LEFT_MAPPED_MIN, LEFT_MAPPED_MAX])
                    right_target_action = np.interp(right_gripper_value, [THUMB_INDEX_DISTANCE_MIN, THUMB_INDEX_DISTANCE_MAX], [RIGHT_MAPPED_MIN, RIGHT_MAPPED_MAX])
                # clip dual gripper action to avoid overflow
                if not self.simulation_mode:
                    # Clamp to DELTA_GRIPPER_CMD around current state for velocity limiting
                    left_actual_action  = np.clip(left_target_action,  dual_gripper_state[0] - DELTA_GRIPPER_CMD, dual_gripper_state[0] + DELTA_GRIPPER_CMD)
                    right_actual_action = np.clip(right_target_action, dual_gripper_state[1] - DELTA_GRIPPER_CMD, dual_gripper_state[1] + DELTA_GRIPPER_CMD)
                else:
                    left_actual_action  = left_target_action
                    right_actual_action = right_target_action
                dual_gripper_action = np.array([left_actual_action, right_actual_action])

                # Apply optional smoothing filter
                if self.smooth_filter:
                    self.smooth_filter.add_data(dual_gripper_action)
                    dual_gripper_action = self.smooth_filter.filtered_data

                if dual_gripper_state_out and dual_gripper_action_out:
                    with dual_hand_data_lock:
                        # Subtract minimum offset so output is zero-referenced
                        dual_gripper_state_out[:] = dual_gripper_state - np.array([LEFT_MAPPED_MIN, RIGHT_MAPPED_MIN])
                        dual_gripper_action_out[:] = dual_gripper_action - np.array([LEFT_MAPPED_MIN, RIGHT_MAPPED_MIN])

                self.ctrl_dual_gripper(dual_gripper_action)

                # Maintain target loop rate
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Dex1_1_Gripper_Controller has been closed.")

class Gripper_JointIndex(IntEnum):
    """Motor ID enum for the Dex1-1 single-motor gripper.

    Members:
        kGripper: The sole gripper motor (id 0).
    """
    kGripper = 0


if __name__ == "__main__":
    import argparse
    from televuer import TeleVuerWrapper
    from teleimager import ImageClient

    parser = argparse.ArgumentParser()
    parser.add_argument('--xr-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device tracking source')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire1', 'brainco'], help='Select end effector controller')
    args = parser.parse_args()
    logger_mp.info(f"args:{args}\n")

    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation

    # image client
    img_client = ImageClient(host='127.0.0.1') #host='192.168.123.164'
    if not img_client.has_head_cam():
        logger_mp.error("Head camera is required. Please enable head camera on the image server side.")
    head_img_shape = img_client.get_head_shape()
    tv_binocular = img_client.head_is_binocular()

    # television: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
    tv_wrapper = TeleVuerWrapper(binocular=tv_binocular, use_hand_tracking=args.xr_mode == "hand", img_shape=head_img_shape, return_hand_rot_data = False)

# end-effector
    if args.ee == "dex3":
        left_hand_pos_array = Array('d', 75, lock = True)      # [input]
        right_hand_pos_array = Array('d', 75, lock = True)     # [input]
        dual_hand_data_lock = Lock()
        dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
        dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
        hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array)
    elif args.ee == "dex1":
        left_gripper_value = Value('d', 0.0, lock=True)        # [input]
        right_gripper_value = Value('d', 0.0, lock=True)       # [input]
        dual_gripper_data_lock = Lock()
        dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
        dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
        gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array)

    user_input = input("Please enter the start signal (enter 's' to start the subsequent program):\n")
    if user_input.lower() == 's':
        while True:
            head_img, head_img_fps = img_client.get_head_frame()
            tv_wrapper.set_display_image(head_img)
            tele_data = tv_wrapper.get_tele_data()
            if args.ee == "dex3" and args.xr_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.xr_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.xr_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass

            # with dual_hand_data_lock:
            #     logger_mp.info(f"state : {list(dual_hand_state_array)} \naction: {list(dual_hand_action_array)} \n")
            time.sleep(0.01)
