"""Controller for BrainCo revolimb dexterous hands via DDS.

Uses separate left/right DDS topics for command publishing and state
subscription. Normalizes retargeted radian values to [0, 1] with inverted
mapping (0 = open, 1 = closed).
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

# Number of motors per hand: thumb, thumb-aux, index, middle, ring, pinky
brainco_Num_Motors = 6

# DDS topic names for BrainCo hand command and state channels (separate per hand)
kTopicbraincoLeftCommand = "rt/brainco/left/cmd"
kTopicbraincoLeftState = "rt/brainco/left/state"
kTopicbraincoRightCommand = "rt/brainco/right/cmd"
kTopicbraincoRightState = "rt/brainco/right/state"

class Brainco_Controller:
    """6-DoF per hand controller for the BrainCo revolimb dexterous hand.

    Motors per hand: thumb, thumb-aux, index, middle, ring, pinky. Uses
    separate left/right DDS topics for command publishing and state
    subscription.

    Normalizes retargeted radian angles with inverted mapping where
    0.0 = fully open and 1.0 = fully closed (opposite to Inspire convention).
    The normalization formula is ``1.0 - clip((max - val) / range, 0, 1)``.

    The controller spawns:
        - A daemon thread for subscribing to hand state DDS topics.
        - A daemon process for the retargeting control loop.

    Attributes:
        fps: Control loop frequency in Hz.
        hand_sub_ready: Set True once the DDS subscriber has read at least once.
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
        """Initializes the BrainCo hand controller.

        Sets up separate left/right DDS publishers/subscribers, starts
        the state subscription thread, waits for initial hand state, and
        spawns the control process.

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
        logger_mp.info("Initialize Brainco_Controller...")
        self.fps = fps
        self.hand_sub_ready = False
        self.Unit_Test = Unit_Test
        self.simulation_mode = simulation_mode

        if not self.Unit_Test:
            self.hand_retargeting = HandRetargeting(HandType.BRAINCO_HAND)
        else:
            self.hand_retargeting = HandRetargeting(HandType.BRAINCO_HAND_Unit_Test)


        # initialize handcmd publisher and handstate subscriber
        self.LeftHandCmb_publisher = ChannelPublisher(kTopicbraincoLeftCommand, MotorCmds_)
        self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicbraincoRightCommand, MotorCmds_)
        self.RightHandCmb_publisher.Init()

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicbraincoLeftState, MotorStates_)
        self.LeftHandState_subscriber.Init()
        self.RightHandState_subscriber = ChannelSubscriber(kTopicbraincoRightState, MotorStates_)
        self.RightHandState_subscriber.Init()

        # Shared Arrays for hand states
        self.left_hand_state_array  = Array('d', brainco_Num_Motors, lock=True)
        self.right_hand_state_array = Array('d', brainco_Num_Motors, lock=True)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        # Block until the subscriber has performed at least one read cycle
        while not self.hand_sub_ready:
            time.sleep(0.1)
            logger_mp.warning("[brainco_Controller] Waiting to subscribe dds...")
        logger_mp.info("[brainco_Controller] Subscribe dds ok.")

        # Spawn the retargeting control process as a daemon
        hand_control_process = Process(target=self.control_process, args=(left_hand_array, right_hand_array,  self.left_hand_state_array, self.right_hand_state_array,
                                                                          dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array))
        hand_control_process.daemon = True
        hand_control_process.start()

        logger_mp.info("Initialize brainco_Controller OK!")

    def _subscribe_hand_state(self) -> None:
        """Continuously reads left/right hand state from separate DDS topics.

        Runs in a daemon thread. Polls both hand state subscribers at ~500 Hz
        and copies each motor's position (``q``) into the corresponding shared
        array slot using the joint-index enums. Sets ``hand_sub_ready`` on
        the first read attempt regardless of whether messages were received.
        """
        while True:
            left_hand_msg  = self.LeftHandState_subscriber.Read()
            right_hand_msg = self.RightHandState_subscriber.Read()
            self.hand_sub_ready = True
            if left_hand_msg is not None and right_hand_msg is not None:
                # Update left hand state
                for idx, id in enumerate(Brainco_Left_Hand_JointIndex):
                    self.left_hand_state_array[idx] = left_hand_msg.states[id].q
                # Update right hand state
                for idx, id in enumerate(Brainco_Right_Hand_JointIndex):
                    self.right_hand_state_array[idx] = right_hand_msg.states[id].q
            time.sleep(0.002)

    def ctrl_dual_hand(self, left_q_target: np.ndarray, right_q_target: np.ndarray) -> None:
        """Publishes normalized [0,1] position targets to both hands via separate DDS topics.

        Args:
            left_q_target: Array of 6 normalized target positions for the
                left hand (0.0 = open, 1.0 = closed).
            right_q_target: Array of 6 normalized target positions for the
                right hand (0.0 = open, 1.0 = closed).
        """
        for idx, id in enumerate(Brainco_Left_Hand_JointIndex):
            self.left_hand_msg.cmds[id].q = left_q_target[idx]
        for idx, id in enumerate(Brainco_Right_Hand_JointIndex):
            self.right_hand_msg.cmds[id].q = right_q_target[idx]

        self.LeftHandCmb_publisher.Write(self.left_hand_msg)
        self.RightHandCmb_publisher.Write(self.right_hand_msg)
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
        with inverted mapping (0 = open, 1 = closed), optionally writes
        state/action data to output shared arrays, and publishes motor commands.

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

        # Default targets: fully open (0.0 for BrainCo convention)
        left_q_target  = np.full(brainco_Num_Motors, 0)
        right_q_target = np.full(brainco_Num_Motors, 0)

        # initialize brainco hand's cmd msg
        self.left_hand_msg  = MotorCmds_()
        self.left_hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Brainco_Left_Hand_JointIndex))]
        self.right_hand_msg = MotorCmds_()
        self.right_hand_msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(len(Brainco_Right_Hand_JointIndex))]

        # Set initial positions to open (q=0.0) with velocity mode enabled (dq=1.0)
        for idx, id in enumerate(Brainco_Left_Hand_JointIndex):
            self.left_hand_msg.cmds[id].q = 0.0
            self.left_hand_msg.cmds[id].dq = 1.0
        for idx, id in enumerate(Brainco_Right_Hand_JointIndex):
            self.right_hand_msg.cmds[id].q = 0.0
            self.right_hand_msg.cmds[id].dq = 1.0

        def normalize(val: float, min_val: float, max_val: float) -> float:
            """Normalizes a radian value to [0, 1] with inverted-then-flipped mapping.

            For BrainCo, 0.0 = open and 1.0 = closed, so we apply
            ``1.0 - clip((max - val) / range, 0, 1)``.

            Args:
                val: Raw retargeted joint angle in radians.
                min_val: Minimum expected radian value.
                max_val: Maximum expected radian value.

            Returns:
                Normalized value clipped to [0.0, 1.0], where
                min_val maps to 0.0 (open) and max_val maps to
                1.0 (closed).
            """
            return 1.0 - np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

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

                    # In the official document, the angles are in the range [0, 1] ==> 0.0: fully open  1.0: fully closed
                    # The q_target now is in radians, ranges:
                    #     - idx 0:   0~1.52
                    #     - idx 1:   0~1.05
                    #     - idx 2~5: 0~1.47
                    # We normalize them using (max - value) / range

                    # Normalize each motor's radian value to [0, 1] using its specific range
                    for idx in range(brainco_Num_Motors):
                        if idx == 0:
                            # Thumb motor: range [0, 1.52] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.52)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.52)
                        elif idx == 1:
                            # Thumb-aux motor: range [0, 1.05] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.05)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.05)
                        elif idx >= 2:
                            # Finger motors (index, middle, ring, pinky): range [0, 1.47] rad
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.47)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.47)

                # get dual hand action
                action_data = np.concatenate((left_q_target, right_q_target))
                if dual_hand_state_array and dual_hand_action_array:
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state_data
                        dual_hand_action_array[:] = action_data
                # logger_mp.info(f"left_q_target:{left_q_target}")
                self.ctrl_dual_hand(left_q_target, right_q_target)

                # Maintain target loop rate
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("brainco_Controller has been closed.")

# according to the official documentation, https://www.brainco-hz.com/docs/revolimb-hand/product/parameters.html
# the motor sequence is as shown in the table below
# ┌──────┬───────┬────────────┬────────┬────────┬────────┬────────┐
# │ Id   │   0   │     1      │   2    │   3    │   4    │   5    │
# ├──────┼───────┼────────────┼────────┼────────┼────────┼────────┤
# │Joint │ thumb │ thumb-aux  |  index │ middle │  ring  │  pinky │
# └──────┴───────┴────────────┴────────┴────────┴────────┴────────┘
class Brainco_Right_Hand_JointIndex(IntEnum):
    """Motor ID enum for the BrainCo right hand.

    Motor order follows the official BrainCo revolimb documentation:
    thumb, thumb-aux, index, middle, ring, pinky.

    Members:
        kRightHandThumb: Thumb primary motor (id 0).
        kRightHandThumbAux: Thumb auxiliary motor (id 1).
        kRightHandIndex: Index finger motor (id 2).
        kRightHandMiddle: Middle finger motor (id 3).
        kRightHandRing: Ring finger motor (id 4).
        kRightHandPinky: Pinky finger motor (id 5).
    """
    kRightHandThumb = 0
    kRightHandThumbAux = 1
    kRightHandIndex = 2
    kRightHandMiddle = 3
    kRightHandRing = 4
    kRightHandPinky = 5

class Brainco_Left_Hand_JointIndex(IntEnum):
    """Motor ID enum for the BrainCo left hand.

    Motor order follows the official BrainCo revolimb documentation:
    thumb, thumb-aux, index, middle, ring, pinky.

    Members:
        kLeftHandThumb: Thumb primary motor (id 0).
        kLeftHandThumbAux: Thumb auxiliary motor (id 1).
        kLeftHandIndex: Index finger motor (id 2).
        kLeftHandMiddle: Middle finger motor (id 3).
        kLeftHandRing: Ring finger motor (id 4).
        kLeftHandPinky: Pinky finger motor (id 5).
    """
    kLeftHandThumb = 0
    kLeftHandThumbAux = 1
    kLeftHandIndex = 2
    kLeftHandMiddle = 3
    kLeftHandRing = 4
    kLeftHandPinky = 5
