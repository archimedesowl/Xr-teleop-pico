"""DDS-based arm controllers for Unitree G1/H1 humanoid robots.

Handles motor state subscription, joint locking, velocity-limited arm control
via CRC-protected DDS messages. Provides controllers for four robot
configurations:

- G1 29-DoF (14 arm joints: 7 left + 7 right, including wrist)
- G1 23-DoF (10 arm joints: 5 left + 5 right, no wrist pitch/yaw)
- H1_2 (14 arm joints: 7 left + 7 right, including elbow roll and wrist)
- H1 (8 arm joints: 4 left + 4 right, shoulder + elbow only)

Each controller runs two daemon threads:
    1. A subscriber thread polling DDS low-state messages at ~500 Hz.
    2. A publisher thread sending CRC-protected motor commands at 250 Hz.

Non-arm joints are locked at their current positions on startup so that
only the arm joints respond to control inputs.
"""

import os
import numpy as np
import threading
import time
from enum import IntEnum
from typing import Any, List, Optional

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import ( LowCmd_  as hg_LowCmd, LowState_ as hg_LowState) # idl for g1, h1_2
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from unitree_sdk2py.idl.unitree_go.msg.dds_ import ( LowCmd_  as go_LowCmd, LowState_ as go_LowState)  # idl for h1
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_

import logging_mp
logger_mp = logging_mp.getLogger(__name__)

# DDS topic names for publishing motor commands and subscribing to state
kTopicLowCommand_Debug  = "rt/lowcmd"
kTopicLowCommand_Motion = "rt/arm_sdk"
kTopicLowState = "rt/lowstate"

# Total motor counts for each robot configuration (includes legs, waist, arms, and unused slots)
G1_29_Num_Motors = 35
G1_23_Num_Motors = 35
H1_2_Num_Motors = 35
H1_Num_Motors = 20


class MotorState:
    """Container for a single motor's state.

    Attributes:
        q: Joint position in radians. None until first DDS update.
        dq: Joint velocity in radians per second. None until first DDS update.
    """

    def __init__(self) -> None:
        self.q: Optional[float] = None
        self.dq: Optional[float] = None


class G1_29_LowState:
    """Low-level state container for the G1 29-DoF robot.

    Stores motor states for all 35 motor slots of the G1 29-DoF
    configuration, covering legs, waist, arms, and unused joints.

    Attributes:
        motor_state: List of MotorState instances, one per motor slot.
    """

    def __init__(self) -> None:
        self.motor_state: List[MotorState] = [MotorState() for _ in range(G1_29_Num_Motors)]


class G1_23_LowState:
    """Low-level state container for the G1 23-DoF robot.

    Stores motor states for all 35 motor slots of the G1 23-DoF
    configuration. Structurally identical to G1_29_LowState but used
    with G1_23 joint index mappings.

    Attributes:
        motor_state: List of MotorState instances, one per motor slot.
    """

    def __init__(self) -> None:
        self.motor_state: List[MotorState] = [MotorState() for _ in range(G1_23_Num_Motors)]


class H1_2_LowState:
    """Low-level state container for the H1_2 robot.

    Stores motor states for all 35 motor slots of the H1_2
    configuration.

    Attributes:
        motor_state: List of MotorState instances, one per motor slot.
    """

    def __init__(self) -> None:
        self.motor_state: List[MotorState] = [MotorState() for _ in range(H1_2_Num_Motors)]


class H1_LowState:
    """Low-level state container for the H1 robot.

    Stores motor states for all 20 motor slots of the H1 configuration.

    Attributes:
        motor_state: List of MotorState instances, one per motor slot.
    """

    def __init__(self) -> None:
        self.motor_state: List[MotorState] = [MotorState() for _ in range(H1_Num_Motors)]


class DataBuffer:
    """Thread-safe data buffer with a reentrant lock.

    Provides synchronized get/set access to an arbitrary data payload,
    used to pass low-state snapshots between the DDS subscriber thread
    and the main control logic.

    Attributes:
        data: The stored data payload. None until first set.
        lock: Threading lock guarding concurrent access to data.
    """

    def __init__(self) -> None:
        self.data: Any = None
        self.lock: threading.Lock = threading.Lock()

    def GetData(self) -> Any:
        """Retrieve the current data payload in a thread-safe manner.

        Returns:
            The stored data, or None if no data has been set yet.
        """
        with self.lock:
            return self.data

    def SetData(self, data: Any) -> None:
        """Store a new data payload in a thread-safe manner.

        Args:
            data: The data to store, typically a robot low-state object.
        """
        with self.lock:
            self.data = data


class G1_29_ArmController:
    """Arm controller for the Unitree G1 29-DoF robot.

    Controls 14 arm joints (7 left + 7 right) including shoulder, elbow,
    and wrist degrees of freedom. Non-arm joints (legs, waist, unused) are
    locked at their startup positions.

    The controller runs two background daemon threads:
        - A subscriber thread that reads motor states from DDS at ~500 Hz.
        - A publisher thread that sends CRC-protected motor commands at 250 Hz.

    Velocity limiting is applied to arm joint targets for safety, preventing
    sudden large motions. The velocity limit can be gradually or instantly
    increased after startup.

    Attributes:
        q_target: Current target joint positions for 14 arm joints (rad).
        tauff_target: Current target feedforward torques for 14 arm joints (Nm).
        motion_mode: If True, publish to the motion SDK topic; otherwise debug topic.
        simulation_mode: If True, skip velocity clipping for simulation use.
        kp_high: Proportional gain for strong (non-arm, non-weak) motors.
        kd_high: Derivative gain for strong motors.
        kp_low: Proportional gain for weak motors (ankles, shoulders, elbows).
        kd_low: Derivative gain for weak motors.
        kp_wrist: Proportional gain for wrist motors.
        kd_wrist: Derivative gain for wrist motors.
        all_motor_q: Cached array of all motor positions from startup.
        arm_velocity_limit: Current velocity limit for arm joint targets (rad/s).
        control_dt: Control loop period in seconds (1/250 Hz).
    """

    def __init__(self, motion_mode: bool = False, simulation_mode: bool = False) -> None:
        """Initialize the G1 29-DoF arm controller.

        Sets up DDS publishers and subscribers, waits for the first motor
        state message, locks all non-arm joints at their current positions,
        and starts background subscriber and publisher threads.

        Args:
            motion_mode: If True, publish commands to the motion SDK topic
                (``rt/arm_sdk``). If False, use the debug topic (``rt/lowcmd``).
            simulation_mode: If True, disable velocity clipping on arm
                targets, allowing instantaneous position jumps suitable
                for simulation environments.
        """
        logger_mp.info("Initialize G1_29_ArmController...")
        self.q_target: np.ndarray = np.zeros(14)
        self.tauff_target: np.ndarray = np.zeros(14)
        self.motion_mode: bool = motion_mode
        self.simulation_mode: bool = simulation_mode
        self.kp_high: float = 300.0
        self.kd_high: float = 3.0
        self.kp_low: float = 80.0
        self.kd_low: float = 3.0
        self.kp_wrist: float = 40.0
        self.kd_wrist: float = 1.5

        self.all_motor_q: Optional[np.ndarray] = None
        self.arm_velocity_limit: float = 20.0
        self.control_dt: float = 1.0 / 250.0

        # Gradual speed ramp state
        self._speed_gradual_max: bool = False
        self._gradual_start_time: Optional[float] = None
        self._gradual_time: Optional[float] = None

        # Set up DDS publisher on the appropriate topic
        if self.motion_mode:
            self.lowcmd_publisher: ChannelPublisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber: ChannelSubscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer: DataBuffer = DataBuffer()

        # initialize subscribe thread
        self.subscribe_thread: threading.Thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # Block until the first DDS state message arrives, with configurable timeout
        _dds_timeout = float(os.environ.get("DDS_SUBSCRIBE_TIMEOUT", "15"))
        _dds_start = time.time()
        while not self.lowstate_buffer.GetData():
            if time.time() - _dds_start > _dds_timeout:
                logger_mp.error(
                    f"[G1_29_ArmController] DDS subscription timed out after {_dds_timeout}s.\n"
                    "  Diagnostics:\n"
                    f"    - Listening on topic: {kTopicLowState}\n"
                    f"    - Motion mode: {self.motion_mode}, Sim mode: {self.simulation_mode}\n"
                    "  Possible causes:\n"
                    "    - Robot/simulator is not running or not publishing on the expected DDS domain\n"
                    "    - Wrong --network-interface (check with: ip -4 addr show)\n"
                    "    - DDS channel mismatch (sim uses channel 1, real uses channel 0)\n"
                    "  Try: ping 192.168.123.161 to verify robot connectivity"
                )
                raise TimeoutError(f"[G1_29_ArmController] DDS subscription timed out after {_dds_timeout}s")
            time.sleep(0.1)
            logger_mp.warning("[G1_29_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[G1_29_ArmController] Subscribe dds ok.")

        # initialize hg's lowcmd msg
        self.crc: CRC = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.debug(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.debug(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        # Lock every joint at its current position; assign PD gains based on motor type
        arm_indices = set(member.value for member in G1_29_JointArmIndex)
        for id in G1_29_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                # Arm joints use lower gains; wrist joints get even lower gains
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                # Non-arm joints: weak motors get lower gains, others get high gains
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread: threading.Thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock: threading.Lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize G1_29_ArmController OK!")

    def _subscribe_motor_state(self) -> None:
        """Background thread that continuously reads motor states from DDS.

        Polls the DDS low-state subscriber at ~500 Hz (2 ms sleep) and
        stores each received snapshot in the thread-safe lowstate_buffer.
        Runs as a daemon thread and never returns.
        """
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_29_LowState()
                for id in range(G1_29_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q: np.ndarray, velocity_limit: float) -> np.ndarray:
        """Clip arm joint targets to enforce a velocity limit for safety.

        Computes the delta between the target and current joint positions,
        then scales the delta down if the maximum per-joint velocity would
        exceed the given limit within one control timestep.

        Args:
            target_q: Desired arm joint positions, shape (14,), in radians.
            velocity_limit: Maximum allowed joint velocity in rad/s.

        Returns:
            Clipped arm joint target positions as a numpy array of shape (14,).
        """
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        # Scale factor: how much the max joint motion exceeds the allowed per-step limit
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        # Only scale down (never scale up) by clamping the divisor to at least 1.0
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self) -> None:
        """Background thread that publishes motor commands at 250 Hz.

        Reads the latest arm target positions and torques under a lock,
        applies velocity clipping (unless in simulation mode), writes
        the CRC-protected command message to DDS, and optionally ramps
        the velocity limit if gradual speed increase is active. Runs as
        a daemon thread and never returns.
        """
        # In motion mode, signal the motion SDK via the unused joint slot
        if self.motion_mode:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0;

        while True:
            start_time = time.time()

            # Read current arm targets under lock
            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target

            # Apply velocity clipping unless in simulation mode
            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit = self.arm_velocity_limit)

            # Write clipped targets into the DDS command message for each arm joint
            for idx, id in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            # Compute CRC and publish
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            # Gradually increase velocity limit if ramp is active
            if self._speed_gradual_max is True:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / self._gradual_time))

            # Sleep for the remainder of the control period to maintain 250 Hz
            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)
            # logger_mp.debug(f"arm_velocity_limit:{self.arm_velocity_limit}")
            # logger_mp.debug(f"sleep_time:{sleep_time}")

    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None:
        """Set control target values q and tau of the left and right arm motors.

        Thread-safe setter that updates arm joint position and feedforward
        torque targets. The publisher thread picks up these values on its
        next 250 Hz cycle.

        Args:
            q_target: Target joint positions for 14 arm joints (rad),
                shape (14,). Order: 7 left arm joints then 7 right arm joints.
            tauff_target: Target feedforward torques for 14 arm joints (Nm),
                shape (14,).
        """
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self) -> int:
        """Return the current DDS mode machine value.

        Reads the latest low-state message directly from the DDS subscriber
        (not the buffered copy) to obtain the mode_machine field.

        Returns:
            The mode_machine integer from the robot's low-state message.
        """
        return self.lowstate_subscriber.Read().mode_machine

    def get_current_motor_q(self) -> np.ndarray:
        """Return current joint positions of all body motors.

        Reads from the buffered low-state and extracts positions for every
        joint in the G1_29_JointIndex enumeration.

        Returns:
            Numpy array of joint positions (rad) for all body motors.
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointIndex])

    def get_current_dual_arm_q(self) -> np.ndarray:
        """Return current joint positions of the left and right arm motors.

        Reads from the buffered low-state and extracts positions for the
        14 arm joints defined in G1_29_JointArmIndex.

        Returns:
            Numpy array of shape (14,) with arm joint positions (rad).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_29_JointArmIndex])

    def get_current_dual_arm_dq(self) -> np.ndarray:
        """Return current joint velocities of the left and right arm motors.

        Reads from the buffered low-state and extracts velocities for the
        14 arm joints defined in G1_29_JointArmIndex.

        Returns:
            Numpy array of shape (14,) with arm joint velocities (rad/s).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_29_JointArmIndex])

    def ctrl_dual_arm_go_home(self) -> None:
        """Move both arms to the home (zero) position.

        Sets all 14 arm joint targets to zero and polls until joint
        positions converge within a tolerance of 0.05 rad, or until
        a maximum of 100 attempts (5 seconds) is reached. In motion
        mode, gradually ramps down the unused joint weight signal
        after reaching home.
        """
        logger_mp.info("[G1_29_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
            # self.tauff_target = np.zeros(14)
        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                # In motion mode, smoothly ramp down the SDK weight signal
                if self.motion_mode:
                    for weight in np.linspace(1, 0, num=101):
                        self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = weight;
                        time.sleep(0.02)
                logger_mp.info("[G1_29_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def speed_gradual_max(self, t: float = 5.0) -> None:
        """Gradually increase the arm velocity limit to its maximum over t seconds.

        Activates a linear ramp from the default velocity limit (20.0 rad/s)
        to the maximum (30.0 rad/s) over the specified duration. The ramp
        is applied in the publisher thread's control loop.

        Args:
            t: Duration in seconds for the velocity ramp. Defaults to 5.0.
        """
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self) -> None:
        """Set the arm velocity limit to its maximum value immediately.

        Bypasses the gradual ramp and sets the velocity limit to 30.0 rad/s
        in a single step.
        """
        self.arm_velocity_limit = 30.0

    def _Is_weak_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a weak motor.

        Weak motors (ankles and arm shoulder/elbow joints) receive lower
        PD gains to prevent excessive torque on mechanically lighter
        actuators.

        Args:
            motor_index: A G1_29_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a weak motor, False otherwise.
        """
        weak_motors = [
            G1_29_JointIndex.kLeftAnklePitch.value,
            G1_29_JointIndex.kRightAnklePitch.value,
            # Left arm
            G1_29_JointIndex.kLeftShoulderPitch.value,
            G1_29_JointIndex.kLeftShoulderRoll.value,
            G1_29_JointIndex.kLeftShoulderYaw.value,
            G1_29_JointIndex.kLeftElbow.value,
            # Right arm
            G1_29_JointIndex.kRightShoulderPitch.value,
            G1_29_JointIndex.kRightShoulderRoll.value,
            G1_29_JointIndex.kRightShoulderYaw.value,
            G1_29_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a wrist motor.

        Wrist motors receive the lowest PD gains due to their small
        actuator size and delicate mechanical linkage.

        Args:
            motor_index: A G1_29_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a wrist motor, False otherwise.
        """
        wrist_motors = [
            G1_29_JointIndex.kLeftWristRoll.value,
            G1_29_JointIndex.kLeftWristPitch.value,
            G1_29_JointIndex.kLeftWristyaw.value,
            G1_29_JointIndex.kRightWristRoll.value,
            G1_29_JointIndex.kRightWristPitch.value,
            G1_29_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors


class G1_29_JointArmIndex(IntEnum):
    """Motor index mapping for the 14 arm joints of the G1 29-DoF robot.

    Maps human-readable joint names to DDS motor IDs for the left and
    right arms. Each arm has 7 joints: shoulder pitch/roll/yaw, elbow,
    and wrist roll/pitch/yaw.
    """

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


class G1_29_JointIndex(IntEnum):
    """Motor index mapping for all 35 joints of the G1 29-DoF robot.

    Maps human-readable joint names to DDS motor IDs for the entire body:
    legs (0-11), waist (12-14), left arm (15-21), right arm (22-28),
    and unused placeholder slots (29-34).
    """

    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12
    kWaistRoll = 13
    kWaistPitch = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28

    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34


class G1_23_ArmController:
    """Arm controller for the Unitree G1 23-DoF robot.

    Controls 10 arm joints (5 left + 5 right) covering shoulder pitch/roll/yaw,
    elbow, and wrist roll. Unlike the 29-DoF variant, this configuration omits
    wrist pitch and yaw joints. Non-arm joints are locked at startup.

    The controller architecture (dual daemon threads, velocity clipping,
    CRC-protected DDS messages) is identical to G1_29_ArmController.

    Attributes:
        q_target: Current target joint positions for 10 arm joints (rad).
        tauff_target: Current target feedforward torques for 10 arm joints (Nm).
        motion_mode: If True, publish to the motion SDK topic; otherwise debug topic.
        simulation_mode: If True, skip velocity clipping for simulation use.
        kp_high: Proportional gain for strong (non-arm, non-weak) motors.
        kd_high: Derivative gain for strong motors.
        kp_low: Proportional gain for weak motors (ankles, shoulders, elbows).
        kd_low: Derivative gain for weak motors.
        kp_wrist: Proportional gain for wrist motors.
        kd_wrist: Derivative gain for wrist motors.
        all_motor_q: Cached array of all motor positions from startup.
        arm_velocity_limit: Current velocity limit for arm joint targets (rad/s).
        control_dt: Control loop period in seconds (1/250 Hz).
    """

    def __init__(self, motion_mode: bool = False, simulation_mode: bool = False) -> None:
        """Initialize the G1 23-DoF arm controller.

        Sets up DDS publishers and subscribers, waits for the first motor
        state message, locks all non-arm joints at their current positions,
        and starts background subscriber and publisher threads.

        Args:
            motion_mode: If True, publish commands to the motion SDK topic
                (``rt/arm_sdk``). If False, use the debug topic (``rt/lowcmd``).
            simulation_mode: If True, disable velocity clipping on arm
                targets for simulation environments.
        """
        self.simulation_mode: bool = simulation_mode
        self.motion_mode: bool = motion_mode

        logger_mp.info("Initialize G1_23_ArmController...")
        self.q_target: np.ndarray = np.zeros(10)
        self.tauff_target: np.ndarray = np.zeros(10)

        self.kp_high: float = 300.0
        self.kd_high: float = 3.0
        self.kp_low: float = 80.0
        self.kd_low: float = 3.0
        self.kp_wrist: float = 40.0
        self.kd_wrist: float = 1.5

        self.all_motor_q: Optional[np.ndarray] = None
        self.arm_velocity_limit: float = 20.0
        self.control_dt: float = 1.0 / 250.0

        # Gradual speed ramp state
        self._speed_gradual_max: bool = False
        self._gradual_start_time: Optional[float] = None
        self._gradual_time: Optional[float] = None

        # Set up DDS publisher on the appropriate topic
        if self.motion_mode:
            self.lowcmd_publisher: ChannelPublisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber: ChannelSubscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer: DataBuffer = DataBuffer()

        # initialize subscribe thread
        self.subscribe_thread: threading.Thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # Block until the first DDS state message arrives, with configurable timeout
        _dds_timeout = float(os.environ.get("DDS_SUBSCRIBE_TIMEOUT", "15"))
        _dds_start = time.time()
        while not self.lowstate_buffer.GetData():
            if time.time() - _dds_start > _dds_timeout:
                logger_mp.error(
                    f"[G1_23_ArmController] DDS subscription timed out after {_dds_timeout}s.\n"
                    "  Diagnostics:\n"
                    f"    - Listening on topic: {kTopicLowState}\n"
                    f"    - Motion mode: {self.motion_mode}, Sim mode: {self.simulation_mode}\n"
                    "  Possible causes:\n"
                    "    - Robot/simulator is not running or not publishing on the expected DDS domain\n"
                    "    - Wrong --network-interface (check with: ip -4 addr show)\n"
                    "    - DDS channel mismatch (sim uses channel 1, real uses channel 0)\n"
                    "  Try: ping 192.168.123.161 to verify robot connectivity"
                )
                raise TimeoutError(f"[G1_23_ArmController] DDS subscription timed out after {_dds_timeout}s")
            time.sleep(0.1)
            logger_mp.warning("[G1_23_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[G1_23_ArmController] Subscribe dds ok.")

        # initialize hg's lowcmd msg
        self.crc: CRC = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.info(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.info(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        # Lock every joint at its current position; assign PD gains based on motor type
        arm_indices = set(member.value for member in G1_23_JointArmIndex)
        for id in G1_23_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                # Arm joints use lower gains; wrist joints get even lower gains
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                # Non-arm joints: weak motors get lower gains, others get high gains
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread: threading.Thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock: threading.Lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize G1_23_ArmController OK!")

    def _subscribe_motor_state(self) -> None:
        """Background thread that continuously reads motor states from DDS.

        Polls the DDS low-state subscriber at ~500 Hz (2 ms sleep) and
        stores each received snapshot in the thread-safe lowstate_buffer.
        Runs as a daemon thread and never returns.
        """
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1_23_LowState()
                for id in range(G1_23_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q: np.ndarray, velocity_limit: float) -> np.ndarray:
        """Clip arm joint targets to enforce a velocity limit for safety.

        Computes the delta between the target and current joint positions,
        then scales the delta down if the maximum per-joint velocity would
        exceed the given limit within one control timestep.

        Args:
            target_q: Desired arm joint positions, shape (10,), in radians.
            velocity_limit: Maximum allowed joint velocity in rad/s.

        Returns:
            Clipped arm joint target positions as a numpy array of shape (10,).
        """
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        # Scale factor: how much the max joint motion exceeds the allowed per-step limit
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        # Only scale down (never scale up) by clamping the divisor to at least 1.0
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self) -> None:
        """Background thread that publishes motor commands at 250 Hz.

        Reads the latest arm target positions and torques under a lock,
        applies velocity clipping (unless in simulation mode), writes
        the CRC-protected command message to DDS, and optionally ramps
        the velocity limit if gradual speed increase is active. Runs as
        a daemon thread and never returns.
        """
        # In motion mode, signal the motion SDK via the unused joint slot
        if self.motion_mode:
            self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = 1.0;

        while True:
            start_time = time.time()

            # Read current arm targets under lock
            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target

            # Apply velocity clipping unless in simulation mode
            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit = self.arm_velocity_limit)

            # Write clipped targets into the DDS command message for each arm joint
            for idx, id in enumerate(G1_23_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            # Compute CRC and publish
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            # Gradually increase velocity limit if ramp is active
            if self._speed_gradual_max is True:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / self._gradual_time))

            # Sleep for the remainder of the control period to maintain 250 Hz
            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)
            # logger_mp.debug(f"arm_velocity_limit:{self.arm_velocity_limit}")
            # logger_mp.debug(f"sleep_time:{sleep_time}")

    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None:
        """Set control target values q and tau of the left and right arm motors.

        Thread-safe setter that updates arm joint position and feedforward
        torque targets. The publisher thread picks up these values on its
        next 250 Hz cycle.

        Args:
            q_target: Target joint positions for 10 arm joints (rad),
                shape (10,). Order: 5 left arm joints then 5 right arm joints.
            tauff_target: Target feedforward torques for 10 arm joints (Nm),
                shape (10,).
        """
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self) -> int:
        """Return the current DDS mode machine value.

        Reads the latest low-state message directly from the DDS subscriber
        (not the buffered copy) to obtain the mode_machine field.

        Returns:
            The mode_machine integer from the robot's low-state message.
        """
        return self.lowstate_subscriber.Read().mode_machine

    def get_current_motor_q(self) -> np.ndarray:
        """Return current joint positions of all body motors.

        Reads from the buffered low-state and extracts positions for every
        joint in the G1_23_JointIndex enumeration.

        Returns:
            Numpy array of joint positions (rad) for all body motors.
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_23_JointIndex])

    def get_current_dual_arm_q(self) -> np.ndarray:
        """Return current joint positions of the left and right arm motors.

        Reads from the buffered low-state and extracts positions for the
        10 arm joints defined in G1_23_JointArmIndex.

        Returns:
            Numpy array of shape (10,) with arm joint positions (rad).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in G1_23_JointArmIndex])

    def get_current_dual_arm_dq(self) -> np.ndarray:
        """Return current joint velocities of the left and right arm motors.

        Reads from the buffered low-state and extracts velocities for the
        10 arm joints defined in G1_23_JointArmIndex.

        Returns:
            Numpy array of shape (10,) with arm joint velocities (rad/s).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in G1_23_JointArmIndex])

    def ctrl_dual_arm_go_home(self) -> None:
        """Move both arms to the home (zero) position.

        Sets all 10 arm joint targets to zero and polls until joint
        positions converge within a tolerance of 0.05 rad, or until
        a maximum of 100 attempts (5 seconds) is reached. In motion
        mode, gradually ramps down the unused joint weight signal
        after reaching home.
        """
        logger_mp.info("[G1_23_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(10)
            # self.tauff_target = np.zeros(10)
        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                # In motion mode, smoothly ramp down the SDK weight signal
                if self.motion_mode:
                    for weight in np.linspace(1, 0, num=101):
                        self.msg.motor_cmd[G1_23_JointIndex.kNotUsedJoint0].q = weight;
                        time.sleep(0.02)
                logger_mp.info("[G1_23_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def speed_gradual_max(self, t: float = 5.0) -> None:
        """Gradually increase the arm velocity limit to its maximum over t seconds.

        Activates a linear ramp from the default velocity limit (20.0 rad/s)
        to the maximum (30.0 rad/s) over the specified duration. The ramp
        is applied in the publisher thread's control loop.

        Args:
            t: Duration in seconds for the velocity ramp. Defaults to 5.0.
        """
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self) -> None:
        """Set the arm velocity limit to its maximum value immediately.

        Bypasses the gradual ramp and sets the velocity limit to 30.0 rad/s
        in a single step.
        """
        self.arm_velocity_limit = 30.0

    def _Is_weak_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a weak motor.

        Weak motors (ankles and arm shoulder/elbow joints) receive lower
        PD gains to prevent excessive torque on mechanically lighter
        actuators.

        Args:
            motor_index: A G1_23_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a weak motor, False otherwise.
        """
        weak_motors = [
            G1_23_JointIndex.kLeftAnklePitch.value,
            G1_23_JointIndex.kRightAnklePitch.value,
            # Left arm
            G1_23_JointIndex.kLeftShoulderPitch.value,
            G1_23_JointIndex.kLeftShoulderRoll.value,
            G1_23_JointIndex.kLeftShoulderYaw.value,
            G1_23_JointIndex.kLeftElbow.value,
            # Right arm
            G1_23_JointIndex.kRightShoulderPitch.value,
            G1_23_JointIndex.kRightShoulderRoll.value,
            G1_23_JointIndex.kRightShoulderYaw.value,
            G1_23_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a wrist motor.

        Wrist motors receive the lowest PD gains. In the G1 23-DoF
        configuration, only the wrist roll joints are active (pitch and
        yaw are not used).

        Args:
            motor_index: A G1_23_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a wrist motor, False otherwise.
        """
        wrist_motors = [
            G1_23_JointIndex.kLeftWristRoll.value,
            G1_23_JointIndex.kRightWristRoll.value,
        ]
        return motor_index.value in wrist_motors


class G1_23_JointArmIndex(IntEnum):
    """Motor index mapping for the 10 arm joints of the G1 23-DoF robot.

    Maps human-readable joint names to DDS motor IDs for the left and
    right arms. Each arm has 5 joints: shoulder pitch/roll/yaw, elbow,
    and wrist roll (no wrist pitch/yaw in the 23-DoF configuration).
    """

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26


class G1_23_JointIndex(IntEnum):
    """Motor index mapping for all 35 joints of the G1 23-DoF robot.

    Maps human-readable joint names to DDS motor IDs for the entire body:
    legs (0-11), waist yaw (12, with roll/pitch not used), left arm (15-21,
    with wrist pitch/yaw not used), right arm (22-28, with wrist pitch/yaw
    not used), and unused placeholder slots (29-34).
    """

    # Left leg
    kLeftHipPitch = 0
    kLeftHipRoll = 1
    kLeftHipYaw = 2
    kLeftKnee = 3
    kLeftAnklePitch = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipPitch = 6
    kRightHipRoll = 7
    kRightHipYaw = 8
    kRightKnee = 9
    kRightAnklePitch = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12
    kWaistRollNotUsed = 13
    kWaistPitchNotUsed = 14

    # Left arm
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitchNotUsed = 20
    kLeftWristyawNotUsed = 21

    # Right arm
    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitchNotUsed = 27
    kRightWristYawNotUsed = 28

    # not used
    kNotUsedJoint0 = 29
    kNotUsedJoint1 = 30
    kNotUsedJoint2 = 31
    kNotUsedJoint3 = 32
    kNotUsedJoint4 = 33
    kNotUsedJoint5 = 34


class H1_2_ArmController:
    """Arm controller for the Unitree H1_2 robot.

    Controls 14 arm joints (7 left + 7 right) including shoulder
    pitch/roll/yaw, elbow pitch/roll, and wrist pitch/yaw. Non-arm
    joints (legs, waist, unused) are locked at their startup positions.

    The controller architecture (dual daemon threads, velocity clipping,
    CRC-protected DDS messages) is identical to G1_29_ArmController,
    but uses H1_2-specific joint indices and PD gains.

    Attributes:
        q_target: Current target joint positions for 14 arm joints (rad).
        tauff_target: Current target feedforward torques for 14 arm joints (Nm).
        motion_mode: If True, publish to the motion SDK topic; otherwise debug topic.
        simulation_mode: If True, skip velocity clipping for simulation use.
        kp_high: Proportional gain for strong (non-arm, non-weak) motors.
        kd_high: Derivative gain for strong motors.
        kp_low: Proportional gain for weak motors (ankles, shoulders, elbows).
        kd_low: Derivative gain for weak motors.
        kp_wrist: Proportional gain for wrist motors.
        kd_wrist: Derivative gain for wrist motors.
        all_motor_q: Cached array of all motor positions from startup.
        arm_velocity_limit: Current velocity limit for arm joint targets (rad/s).
        control_dt: Control loop period in seconds (1/250 Hz).
    """

    def __init__(self, motion_mode: bool = False, simulation_mode: bool = False) -> None:
        """Initialize the H1_2 arm controller.

        Sets up DDS publishers and subscribers, waits for the first motor
        state message, locks all non-arm joints at their current positions,
        and starts background subscriber and publisher threads.

        Args:
            motion_mode: If True, publish commands to the motion SDK topic
                (``rt/arm_sdk``). If False, use the debug topic (``rt/lowcmd``).
            simulation_mode: If True, disable velocity clipping on arm
                targets for simulation environments.
        """
        self.simulation_mode: bool = simulation_mode
        self.motion_mode: bool = motion_mode

        logger_mp.info("Initialize H1_2_ArmController...")
        self.q_target: np.ndarray = np.zeros(14)
        self.tauff_target: np.ndarray = np.zeros(14)

        self.kp_high: float = 300.0
        self.kd_high: float = 5.0
        self.kp_low: float = 140.0
        self.kd_low: float = 3.0
        self.kp_wrist: float = 50.0
        self.kd_wrist: float = 2.0

        self.all_motor_q: Optional[np.ndarray] = None
        self.arm_velocity_limit: float = 20.0
        self.control_dt: float = 1.0 / 250.0

        # Gradual speed ramp state
        self._speed_gradual_max: bool = False
        self._gradual_start_time: Optional[float] = None
        self._gradual_time: Optional[float] = None


        # Set up DDS publisher on the appropriate topic
        if self.motion_mode:
            self.lowcmd_publisher: ChannelPublisher = ChannelPublisher(kTopicLowCommand_Motion, hg_LowCmd)
        else:
            self.lowcmd_publisher = ChannelPublisher(kTopicLowCommand_Debug, hg_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber: ChannelSubscriber = ChannelSubscriber(kTopicLowState, hg_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer: DataBuffer = DataBuffer()

        # initialize subscribe thread
        self.subscribe_thread: threading.Thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # Block until the first DDS state message arrives, with configurable timeout
        _dds_timeout = float(os.environ.get("DDS_SUBSCRIBE_TIMEOUT", "15"))
        _dds_start = time.time()
        while not self.lowstate_buffer.GetData():
            if time.time() - _dds_start > _dds_timeout:
                logger_mp.error(
                    f"[H1_2_ArmController] DDS subscription timed out after {_dds_timeout}s.\n"
                    "  Diagnostics:\n"
                    f"    - Listening on topic: {kTopicLowState}\n"
                    f"    - Motion mode: {self.motion_mode}, Sim mode: {self.simulation_mode}\n"
                    "  Possible causes:\n"
                    "    - Robot/simulator is not running or not publishing on the expected DDS domain\n"
                    "    - Wrong --network-interface (check with: ip -4 addr show)\n"
                    "    - DDS channel mismatch (sim uses channel 1, real uses channel 0)\n"
                    "  Try: ping 192.168.123.161 to verify robot connectivity"
                )
                raise TimeoutError(f"[H1_2_ArmController] DDS subscription timed out after {_dds_timeout}s")
            time.sleep(0.1)
            logger_mp.warning("[H1_2_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[H1_2_ArmController] Subscribe dds ok.")

        # initialize hg's lowcmd msg
        self.crc: CRC = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.info(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.info(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        # Lock every joint at its current position; assign PD gains based on motor type
        arm_indices = set(member.value for member in H1_2_JointArmIndex)
        for id in H1_2_JointIndex:
            self.msg.motor_cmd[id].mode = 1
            if id.value in arm_indices:
                # Arm joints use lower gains; wrist joints get even lower gains
                if self._Is_wrist_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_wrist
                    self.msg.motor_cmd[id].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
            else:
                # Non-arm joints: weak motors get lower gains, others get high gains
                if self._Is_weak_motor(id):
                    self.msg.motor_cmd[id].kp = self.kp_low
                    self.msg.motor_cmd[id].kd = self.kd_low
                else:
                    self.msg.motor_cmd[id].kp = self.kp_high
                    self.msg.motor_cmd[id].kd = self.kd_high
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread: threading.Thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock: threading.Lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize H1_2_ArmController OK!")

    def _subscribe_motor_state(self) -> None:
        """Background thread that continuously reads motor states from DDS.

        Polls the DDS low-state subscriber at ~500 Hz (2 ms sleep) and
        stores each received snapshot in the thread-safe lowstate_buffer.
        Runs as a daemon thread and never returns.
        """
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = H1_2_LowState()
                for id in range(H1_2_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q: np.ndarray, velocity_limit: float) -> np.ndarray:
        """Clip arm joint targets to enforce a velocity limit for safety.

        Computes the delta between the target and current joint positions,
        then scales the delta down if the maximum per-joint velocity would
        exceed the given limit within one control timestep.

        Args:
            target_q: Desired arm joint positions, shape (14,), in radians.
            velocity_limit: Maximum allowed joint velocity in rad/s.

        Returns:
            Clipped arm joint target positions as a numpy array of shape (14,).
        """
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        # Scale factor: how much the max joint motion exceeds the allowed per-step limit
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        # Only scale down (never scale up) by clamping the divisor to at least 1.0
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self) -> None:
        """Background thread that publishes motor commands at 250 Hz.

        Reads the latest arm target positions and torques under a lock,
        applies velocity clipping (unless in simulation mode), writes
        the CRC-protected command message to DDS, and optionally ramps
        the velocity limit if gradual speed increase is active. Runs as
        a daemon thread and never returns.
        """
        # In motion mode, signal the motion SDK via the unused joint slot
        if self.motion_mode:
            self.msg.motor_cmd[H1_2_JointIndex.kNotUsedJoint0].q = 1.0;

        while True:
            start_time = time.time()

            # Read current arm targets under lock
            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target

            # Apply velocity clipping unless in simulation mode
            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit = self.arm_velocity_limit)

            # Write clipped targets into the DDS command message for each arm joint
            for idx, id in enumerate(H1_2_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            # Compute CRC and publish
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            # Gradually increase velocity limit if ramp is active
            if self._speed_gradual_max is True:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / self._gradual_time))

            # Sleep for the remainder of the control period to maintain 250 Hz
            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)
            # logger_mp.debug(f"arm_velocity_limit:{self.arm_velocity_limit}")
            # logger_mp.debug(f"sleep_time:{sleep_time}")

    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None:
        """Set control target values q and tau of the left and right arm motors.

        Thread-safe setter that updates arm joint position and feedforward
        torque targets. The publisher thread picks up these values on its
        next 250 Hz cycle.

        Args:
            q_target: Target joint positions for 14 arm joints (rad),
                shape (14,). Order: 7 left arm joints then 7 right arm joints.
            tauff_target: Target feedforward torques for 14 arm joints (Nm),
                shape (14,).
        """
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_mode_machine(self) -> int:
        """Return the current DDS mode machine value.

        Reads the latest low-state message directly from the DDS subscriber
        (not the buffered copy) to obtain the mode_machine field.

        Returns:
            The mode_machine integer from the robot's low-state message.
        """
        return self.lowstate_subscriber.Read().mode_machine

    def get_current_motor_q(self) -> np.ndarray:
        """Return current joint positions of all body motors.

        Reads from the buffered low-state and extracts positions for every
        joint in the H1_2_JointIndex enumeration.

        Returns:
            Numpy array of joint positions (rad) for all body motors.
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_2_JointIndex])

    def get_current_dual_arm_q(self) -> np.ndarray:
        """Return current joint positions of the left and right arm motors.

        Reads from the buffered low-state and extracts positions for the
        14 arm joints defined in H1_2_JointArmIndex.

        Returns:
            Numpy array of shape (14,) with arm joint positions (rad).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_2_JointArmIndex])

    def get_current_dual_arm_dq(self) -> np.ndarray:
        """Return current joint velocities of the left and right arm motors.

        Reads from the buffered low-state and extracts velocities for the
        14 arm joints defined in H1_2_JointArmIndex.

        Returns:
            Numpy array of shape (14,) with arm joint velocities (rad/s).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in H1_2_JointArmIndex])

    def ctrl_dual_arm_go_home(self) -> None:
        """Move both arms to the home (zero) position.

        Sets all 14 arm joint targets to zero and polls until joint
        positions converge within a tolerance of 0.05 rad, or until
        a maximum of 100 attempts (5 seconds) is reached. In motion
        mode, gradually ramps down the unused joint weight signal
        after reaching home.
        """
        logger_mp.info("[H1_2_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
            # self.tauff_target = np.zeros(14)
        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                # In motion mode, smoothly ramp down the SDK weight signal
                if self.motion_mode:
                    for weight in np.linspace(1, 0, num=101):
                        self.msg.motor_cmd[H1_2_JointIndex.kNotUsedJoint0].q = weight;
                        time.sleep(0.02)
                logger_mp.info("[H1_2_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def speed_gradual_max(self, t: float = 5.0) -> None:
        """Gradually increase the arm velocity limit to its maximum over t seconds.

        Activates a linear ramp from the default velocity limit (20.0 rad/s)
        to the maximum (30.0 rad/s) over the specified duration. The ramp
        is applied in the publisher thread's control loop.

        Args:
            t: Duration in seconds for the velocity ramp. Defaults to 5.0.
        """
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self) -> None:
        """Set the arm velocity limit to its maximum value immediately.

        Bypasses the gradual ramp and sets the velocity limit to 30.0 rad/s
        in a single step.
        """
        self.arm_velocity_limit = 30.0

    def _Is_weak_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a weak motor.

        Weak motors (ankles and arm shoulder/elbow pitch joints) receive
        lower PD gains to prevent excessive torque on mechanically lighter
        actuators.

        Args:
            motor_index: An H1_2_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a weak motor, False otherwise.
        """
        weak_motors = [
            H1_2_JointIndex.kLeftAnkle.value,
            H1_2_JointIndex.kRightAnkle.value,
            # Left arm
            H1_2_JointIndex.kLeftShoulderPitch.value,
            H1_2_JointIndex.kLeftShoulderRoll.value,
            H1_2_JointIndex.kLeftShoulderYaw.value,
            H1_2_JointIndex.kLeftElbowPitch.value,
            # Right arm
            H1_2_JointIndex.kRightShoulderPitch.value,
            H1_2_JointIndex.kRightShoulderRoll.value,
            H1_2_JointIndex.kRightShoulderYaw.value,
            H1_2_JointIndex.kRightElbowPitch.value,
        ]
        return motor_index.value in weak_motors

    def _Is_wrist_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a wrist motor.

        Wrist motors (elbow roll and wrist pitch/yaw) receive the lowest
        PD gains due to their smaller actuator size.

        Args:
            motor_index: An H1_2_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a wrist motor, False otherwise.
        """
        wrist_motors = [
            H1_2_JointIndex.kLeftElbowRoll.value,
            H1_2_JointIndex.kLeftWristPitch.value,
            H1_2_JointIndex.kLeftWristyaw.value,
            H1_2_JointIndex.kRightElbowRoll.value,
            H1_2_JointIndex.kRightWristPitch.value,
            H1_2_JointIndex.kRightWristYaw.value,
        ]
        return motor_index.value in wrist_motors


class H1_2_JointArmIndex(IntEnum):
    """Motor index mapping for the 14 arm joints of the H1_2 robot.

    Maps human-readable joint names to DDS motor IDs for the left and
    right arms. Each arm has 7 joints: shoulder pitch/roll/yaw, elbow
    pitch/roll, and wrist pitch/yaw.
    """

    # Left arm
    kLeftShoulderPitch = 13
    kLeftShoulderRoll = 14
    kLeftShoulderYaw = 15
    kLeftElbowPitch = 16
    kLeftElbowRoll = 17
    kLeftWristPitch = 18
    kLeftWristyaw = 19

    # Right arm
    kRightShoulderPitch = 20
    kRightShoulderRoll = 21
    kRightShoulderYaw = 22
    kRightElbowPitch = 23
    kRightElbowRoll = 24
    kRightWristPitch = 25
    kRightWristYaw = 26


class H1_2_JointIndex(IntEnum):
    """Motor index mapping for all 35 joints of the H1_2 robot.

    Maps human-readable joint names to DDS motor IDs for the entire body:
    left leg (0-5), right leg (6-11), waist yaw (12), left arm (13-19),
    right arm (20-26), and unused placeholder slots (27-34).
    """

    # Left leg
    kLeftHipYaw = 0
    kLeftHipRoll = 1
    kLeftHipPitch = 2
    kLeftKnee = 3
    kLeftAnkle = 4
    kLeftAnkleRoll = 5

    # Right leg
    kRightHipYaw = 6
    kRightHipRoll = 7
    kRightHipPitch = 8
    kRightKnee = 9
    kRightAnkle = 10
    kRightAnkleRoll = 11

    kWaistYaw = 12

    # Left arm
    kLeftShoulderPitch = 13
    kLeftShoulderRoll = 14
    kLeftShoulderYaw = 15
    kLeftElbowPitch = 16
    kLeftElbowRoll = 17
    kLeftWristPitch = 18
    kLeftWristyaw = 19

    # Right arm
    kRightShoulderPitch = 20
    kRightShoulderRoll = 21
    kRightShoulderYaw = 22
    kRightElbowPitch = 23
    kRightElbowRoll = 24
    kRightWristPitch = 25
    kRightWristYaw = 26

    kNotUsedJoint0 = 27
    kNotUsedJoint1 = 28
    kNotUsedJoint2 = 29
    kNotUsedJoint3 = 30
    kNotUsedJoint4 = 31
    kNotUsedJoint5 = 32
    kNotUsedJoint6 = 33
    kNotUsedJoint7 = 34


class H1_ArmController:
    """Arm controller for the Unitree H1 robot.

    Controls 8 arm joints (4 left + 4 right) covering shoulder
    pitch/roll/yaw and elbow only (no wrist joints). Non-arm joints
    (legs, waist, unused) are locked at their startup positions.

    Unlike the G1 and H1_2 controllers, the H1 uses the ``go`` IDL
    (``go_LowCmd``/``go_LowState``) instead of the ``hg`` IDL, and
    does not support motion_mode (always publishes to the debug topic).

    Note:
        The DDS motor ordering for H1 places the right arm before the
        left arm, which is the reverse of G1 and H1_2. The JointArmIndex
        enum reorders them to left-first for consistency.

    Attributes:
        q_target: Current target joint positions for 8 arm joints (rad).
        tauff_target: Current target feedforward torques for 8 arm joints (Nm).
        simulation_mode: If True, skip velocity clipping for simulation use.
        kp_high: Proportional gain for strong (non-arm, non-weak) motors.
        kd_high: Derivative gain for strong motors.
        kp_low: Proportional gain for weak motors (ankles, shoulders, elbows).
        kd_low: Derivative gain for weak motors.
        all_motor_q: Cached array of all motor positions from startup.
        arm_velocity_limit: Current velocity limit for arm joint targets (rad/s).
        control_dt: Control loop period in seconds (1/250 Hz).
    """

    def __init__(self, simulation_mode: bool = False) -> None:
        """Initialize the H1 arm controller.

        Sets up DDS publishers and subscribers using the ``go`` IDL,
        waits for the first motor state message, locks all non-arm joints
        at their current positions, and starts background subscriber and
        publisher threads. Always uses the debug topic (no motion_mode).

        Args:
            simulation_mode: If True, disable velocity clipping on arm
                targets for simulation environments.
        """
        self.simulation_mode: bool = simulation_mode

        logger_mp.info("Initialize H1_ArmController...")
        self.q_target: np.ndarray = np.zeros(8)
        self.tauff_target: np.ndarray = np.zeros(8)

        self.kp_high: float = 300.0
        self.kd_high: float = 5.0
        self.kp_low: float = 140.0
        self.kd_low: float = 3.0

        self.all_motor_q: Optional[np.ndarray] = None
        self.arm_velocity_limit: float = 20.0
        self.control_dt: float = 1.0 / 250.0

        # Gradual speed ramp state
        self._speed_gradual_max: bool = False
        self._gradual_start_time: Optional[float] = None
        self._gradual_time: Optional[float] = None

        # H1 always uses the debug topic with the go IDL
        self.lowcmd_publisher: ChannelPublisher = ChannelPublisher(kTopicLowCommand_Debug, go_LowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber: ChannelSubscriber = ChannelSubscriber(kTopicLowState, go_LowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer: DataBuffer = DataBuffer()

        # initialize subscribe thread
        self.subscribe_thread: threading.Thread = threading.Thread(target=self._subscribe_motor_state)
        self.subscribe_thread.daemon = True
        self.subscribe_thread.start()

        # Block until the first DDS state message arrives, with configurable timeout
        _dds_timeout = float(os.environ.get("DDS_SUBSCRIBE_TIMEOUT", "15"))
        _dds_start = time.time()
        while not self.lowstate_buffer.GetData():
            if time.time() - _dds_start > _dds_timeout:
                logger_mp.error(
                    f"[H1_ArmController] DDS subscription timed out after {_dds_timeout}s.\n"
                    "  Diagnostics:\n"
                    f"    - Listening on topic: {kTopicLowState}\n"
                    f"    - Sim mode: {self.simulation_mode}\n"
                    "  Possible causes:\n"
                    "    - Robot/simulator is not running or not publishing on the expected DDS domain\n"
                    "    - Wrong --network-interface (check with: ip -4 addr show)\n"
                    "    - DDS channel mismatch (sim uses channel 1, real uses channel 0)\n"
                    "  Try: ping 192.168.123.161 to verify robot connectivity"
                )
                raise TimeoutError(f"[H1_ArmController] DDS subscription timed out after {_dds_timeout}s")
            time.sleep(0.1)
            logger_mp.warning("[H1_ArmController] Waiting to subscribe dds...")
        logger_mp.info("[H1_ArmController] Subscribe dds ok.")

        # initialize h1's lowcmd msg
        self.crc: CRC = CRC()
        self.msg = unitree_go_msg_dds__LowCmd_()
        self.msg.head[0] = 0xFE
        self.msg.head[1] = 0xEF
        self.msg.level_flag = 0xFF
        self.msg.gpio = 0

        self.all_motor_q = self.get_current_motor_q()
        logger_mp.info(f"Current all body motor state q:\n{self.all_motor_q} \n")
        logger_mp.info(f"Current two arms motor state q:\n{self.get_current_dual_arm_q()}\n")
        logger_mp.info("Lock all joints except two arms...")

        # Lock every joint at its current position; assign PD gains and mode based on motor type
        for id in H1_JointIndex:
            if self._Is_weak_motor(id):
                self.msg.motor_cmd[id].kp = self.kp_low
                self.msg.motor_cmd[id].kd = self.kd_low
                self.msg.motor_cmd[id].mode = 0x01
            else:
                self.msg.motor_cmd[id].kp = self.kp_high
                self.msg.motor_cmd[id].kd = self.kd_high
                self.msg.motor_cmd[id].mode = 0x0A
            self.msg.motor_cmd[id].q  = self.all_motor_q[id]
        logger_mp.info("Lock OK!")

        # initialize publish thread
        self.publish_thread: threading.Thread = threading.Thread(target=self._ctrl_motor_state)
        self.ctrl_lock: threading.Lock = threading.Lock()
        self.publish_thread.daemon = True
        self.publish_thread.start()

        logger_mp.info("Initialize H1_ArmController OK!")

    def _subscribe_motor_state(self) -> None:
        """Background thread that continuously reads motor states from DDS.

        Polls the DDS low-state subscriber at ~500 Hz (2 ms sleep) and
        stores each received snapshot in the thread-safe lowstate_buffer.
        Runs as a daemon thread and never returns.
        """
        while True:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = H1_LowState()
                for id in range(H1_Num_Motors):
                    lowstate.motor_state[id].q  = msg.motor_state[id].q
                    lowstate.motor_state[id].dq = msg.motor_state[id].dq
                self.lowstate_buffer.SetData(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q: np.ndarray, velocity_limit: float) -> np.ndarray:
        """Clip arm joint targets to enforce a velocity limit for safety.

        Computes the delta between the target and current joint positions,
        then scales the delta down if the maximum per-joint velocity would
        exceed the given limit within one control timestep.

        Args:
            target_q: Desired arm joint positions, shape (8,), in radians.
            velocity_limit: Maximum allowed joint velocity in rad/s.

        Returns:
            Clipped arm joint target positions as a numpy array of shape (8,).
        """
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        # Scale factor: how much the max joint motion exceeds the allowed per-step limit
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        # Only scale down (never scale up) by clamping the divisor to at least 1.0
        cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
        return cliped_arm_q_target

    def _ctrl_motor_state(self) -> None:
        """Background thread that publishes motor commands at 250 Hz.

        Reads the latest arm target positions and torques under a lock,
        applies velocity clipping (unless in simulation mode), writes
        the CRC-protected command message to DDS, and optionally ramps
        the velocity limit if gradual speed increase is active. Runs as
        a daemon thread and never returns.
        """
        while True:
            start_time = time.time()

            # Read current arm targets under lock
            with self.ctrl_lock:
                arm_q_target     = self.q_target
                arm_tauff_target = self.tauff_target

            # Apply velocity clipping unless in simulation mode
            if self.simulation_mode:
                cliped_arm_q_target = arm_q_target
            else:
                cliped_arm_q_target = self.clip_arm_q_target(arm_q_target, velocity_limit = self.arm_velocity_limit)

            # Write clipped targets into the DDS command message for each arm joint
            for idx, id in enumerate(H1_JointArmIndex):
                self.msg.motor_cmd[id].q = cliped_arm_q_target[idx]
                self.msg.motor_cmd[id].dq = 0
                self.msg.motor_cmd[id].tau = arm_tauff_target[idx]

            # Compute CRC and publish
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            # Gradually increase velocity limit if ramp is active
            if self._speed_gradual_max is True:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / self._gradual_time))

            # Sleep for the remainder of the control period to maintain 250 Hz
            current_time = time.time()
            all_t_elapsed = current_time - start_time
            sleep_time = max(0, (self.control_dt - all_t_elapsed))
            time.sleep(sleep_time)
            # logger_mp.debug(f"arm_velocity_limit:{self.arm_velocity_limit}")
            # logger_mp.debug(f"sleep_time:{sleep_time}")

    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None:
        """Set control target values q and tau of the left and right arm motors.

        Thread-safe setter that updates arm joint position and feedforward
        torque targets. The publisher thread picks up these values on its
        next 250 Hz cycle.

        Args:
            q_target: Target joint positions for 8 arm joints (rad),
                shape (8,). Order: 4 left arm joints then 4 right arm joints.
            tauff_target: Target feedforward torques for 8 arm joints (Nm),
                shape (8,).
        """
        with self.ctrl_lock:
            self.q_target = q_target
            self.tauff_target = tauff_target

    def get_current_motor_q(self) -> np.ndarray:
        """Return current joint positions of all body motors.

        Reads from the buffered low-state and extracts positions for every
        joint in the H1_JointIndex enumeration.

        Returns:
            Numpy array of joint positions (rad) for all body motors.
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_JointIndex])

    def get_current_dual_arm_q(self) -> np.ndarray:
        """Return current joint positions of the left and right arm motors.

        Reads from the buffered low-state and extracts positions for the
        8 arm joints defined in H1_JointArmIndex.

        Returns:
            Numpy array of shape (8,) with arm joint positions (rad).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].q for id in H1_JointArmIndex])

    def get_current_dual_arm_dq(self) -> np.ndarray:
        """Return current joint velocities of the left and right arm motors.

        Reads from the buffered low-state and extracts velocities for the
        8 arm joints defined in H1_JointArmIndex.

        Returns:
            Numpy array of shape (8,) with arm joint velocities (rad/s).
        """
        return np.array([self.lowstate_buffer.GetData().motor_state[id].dq for id in H1_JointArmIndex])

    def ctrl_dual_arm_go_home(self) -> None:
        """Move both arms to the home (zero) position.

        Sets all 8 arm joint targets to zero and polls until joint
        positions converge within a tolerance of 0.05 rad, or until
        a maximum of 100 attempts (5 seconds) is reached.
        """
        logger_mp.info("[H1_ArmController] ctrl_dual_arm_go_home start...")
        max_attempts = 100
        current_attempts = 0
        with self.ctrl_lock:
            self.q_target = np.zeros(8)
            # self.tauff_target = np.zeros(8)
        tolerance = 0.05  # Tolerance threshold for joint angles to determine "close to zero", can be adjusted based on your motor's precision requirements
        while current_attempts < max_attempts:
            current_q = self.get_current_dual_arm_q()
            if np.all(np.abs(current_q) < tolerance):
                logger_mp.info("[H1_ArmController] both arms have reached the home position.")
                break
            current_attempts += 1
            time.sleep(0.05)

    def speed_gradual_max(self, t: float = 5.0) -> None:
        """Gradually increase the arm velocity limit to its maximum over t seconds.

        Activates a linear ramp from the default velocity limit (20.0 rad/s)
        to the maximum (30.0 rad/s) over the specified duration. The ramp
        is applied in the publisher thread's control loop.

        Args:
            t: Duration in seconds for the velocity ramp. Defaults to 5.0.
        """
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self) -> None:
        """Set the arm velocity limit to its maximum value immediately.

        Bypasses the gradual ramp and sets the velocity limit to 30.0 rad/s
        in a single step.
        """
        self.arm_velocity_limit = 30.0

    def _Is_weak_motor(self, motor_index: IntEnum) -> bool:
        """Check whether a motor is classified as a weak motor.

        Weak motors (ankles and arm shoulder/elbow joints) receive lower
        PD gains to prevent excessive torque on mechanically lighter
        actuators.

        Args:
            motor_index: An H1_JointIndex enum member identifying the motor.

        Returns:
            True if the motor is a weak motor, False otherwise.
        """
        weak_motors = [
            H1_JointIndex.kLeftAnkle.value,
            H1_JointIndex.kRightAnkle.value,
            # Left arm
            H1_JointIndex.kLeftShoulderPitch.value,
            H1_JointIndex.kLeftShoulderRoll.value,
            H1_JointIndex.kLeftShoulderYaw.value,
            H1_JointIndex.kLeftElbow.value,
            # Right arm
            H1_JointIndex.kRightShoulderPitch.value,
            H1_JointIndex.kRightShoulderRoll.value,
            H1_JointIndex.kRightShoulderYaw.value,
            H1_JointIndex.kRightElbow.value,
        ]
        return motor_index.value in weak_motors


class H1_JointArmIndex(IntEnum):
    """Motor index mapping for the 8 arm joints of the H1 robot.

    Maps human-readable joint names to DDS motor IDs for the left and
    right arms. Each arm has 4 joints: shoulder pitch/roll/yaw and elbow.

    Note:
        Unlike G1 and H1_2, the arm order in DDS messages for H1 is right
        then left. Therefore, the purpose of switching the order here is to
        maintain consistency with G1 and H1_2.
    """

    # Unlike G1 and H1_2, the arm order in DDS messages for H1 is right then left.
    # Therefore, the purpose of switching the order here is to maintain consistency with G1 and H1_2.
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19
    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15


class H1_JointIndex(IntEnum):
    """Motor index mapping for all 20 joints of the H1 robot.

    Maps human-readable joint names to DDS motor IDs for the entire body:
    right leg (0-2), left leg (3-5), waist (6), hip yaw (7-8), unused (9),
    ankles (10-11), right arm (12-15), and left arm (16-19).

    Note:
        The H1 motor ordering differs from G1/H1_2: legs interleave
        right/left, and the right arm comes before the left arm.
    """

    kRightHipRoll = 0
    kRightHipPitch = 1
    kRightKnee = 2
    kLeftHipRoll = 3
    kLeftHipPitch = 4
    kLeftKnee = 5
    kWaistYaw = 6
    kLeftHipYaw = 7
    kRightHipYaw = 8
    kNotUsedJoint = 9
    kLeftAnkle = 10
    kRightAnkle = 11
    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15
    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19


if __name__ == "__main__":
    from robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK
    import pinocchio as pin

    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation

    arm_ik = G1_29_ArmIK(Unit_Test = True, Visualization = False)
    arm = G1_29_ArmController(simulation_mode=True)
    # arm_ik = G1_23_ArmIK(Unit_Test = True, Visualization = False)
    # arm = G1_23_ArmController()
    # arm_ik = H1_2_ArmIK(Unit_Test = True, Visualization = False)
    # arm = H1_2_ArmController()
    # arm_ik = H1_ArmIK(Unit_Test = True, Visualization = True)
    # arm = H1_ArmController()

    # initial positon
    L_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, +0.25, 0.1]),
    )

    R_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, -0.25, 0.1]),
    )

    rotation_speed = 0.005  # Rotation speed in radians per iteration

    user_input = input("Please enter the start signal (enter 's' to start the subsequent program): \n")
    if user_input.lower() == 's':
        step = 0
        arm.speed_gradual_max()
        while True:
            if step <= 120:
                angle = rotation_speed * step
                L_quat = pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)  # y axis
                R_quat = pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))  # z axis

                L_tf_target.translation += np.array([0.001,  0.001, 0.001])
                R_tf_target.translation += np.array([0.001, -0.001, 0.001])
            else:
                angle = rotation_speed * (240 - step)
                L_quat = pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)  # y axis
                R_quat = pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))  # z axis

                L_tf_target.translation -= np.array([0.001,  0.001, 0.001])
                R_tf_target.translation -= np.array([0.001, -0.001, 0.001])

            L_tf_target.rotation = L_quat.toRotationMatrix()
            R_tf_target.rotation = R_quat.toRotationMatrix()

            current_lr_arm_q  = arm.get_current_dual_arm_q()
            current_lr_arm_dq = arm.get_current_dual_arm_dq()

            sol_q, sol_tauff = arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous, current_lr_arm_q, current_lr_arm_dq)

            arm.ctrl_dual_arm(sol_q, sol_tauff)

            step += 1
            if step > 240:
                step = 0
            time.sleep(0.01)
