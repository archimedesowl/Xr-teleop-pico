# Codebase Index

## Architecture Overview

XR-based teleoperation system for Unitree humanoid robots (G1, H1). An XR headset
(PICO, Quest, or Apple Vision Pro) streams hand/controller tracking data to the host
machine, which solves inverse kinematics and publishes joint commands to the robot
over DDS (Data Distribution Service).

```
┌─────────────┐    WebXR/WSS     ┌──────────────────┐     DDS/UDP      ┌────────────┐
│  XR Device   │ ──────────────► │   Host Machine    │ ───────────────► │  G1 Robot  │
│ (PICO/Quest) │ ◄────────────── │                   │ ◄─────────────── │  (29 DoF)  │
│              │   camera feed   │                   │   motor states   │            │
└─────────────┘                  └──────────────────┘                   └────────────┘
                                         │
                            ┌────────────┴────────────┐
                            │                         │
                  ┌─────────▼─────────┐    ┌──────────▼──────────┐
                  │  televuer (WebXR)  │    │  teleimager (camera) │
                  │  - serves VR app   │    │  - image_server (bot)│
                  │  - hand tracking   │    │  - image_client (host│
                  │  - controller I/O  │    │  - ZMQ/WebRTC        │
                  └───────────────────┘    └─────────────────────┘

Data Flow (per frame, ~30 Hz):
  1. televuer → TeleVuerWrapper.get_tele_data() → wrist poses + hand skeleton
  2. wrist poses → ArmIK.solve_ik() → joint angles (q) + torques (tau)
  3. q, tau → ArmController.ctrl_dual_arm() → DDS publish → robot motors
  4. hand skeleton → HandRetargeting → HandController → DDS → robot fingers
  5. teleimager → ImageClient.get_head_frame() → televuer → XR display
```

## Directory Structure

```
Xr-teleop-pico/
├── teleop/
│   ├── teleop_hand_and_arm.py          # Main entry point — orchestrates XR input, IK, and robot control
│   ├── robot_control/
│   │   ├── robot_arm.py                # DDS-based arm controllers (G1_29, G1_23, H1_2, H1)
│   │   ├── robot_arm_ik.py             # CasADi+Pinocchio IK solvers for each robot variant
│   │   ├── robot_hand_unitree.py       # Unitree Dex3-1 hand and Dex1-1 gripper controllers
│   │   ├── robot_hand_inspire.py       # Inspire DFX and FTP dexterous hand controllers
│   │   ├── robot_hand_brainco.py       # BrainCo revolimb hand controller
│   │   ├── hand_retargeting.py         # Maps XR hand tracking to robot hand joints via dex-retargeting
│   │   └── dex-retargeting/            # [submodule] Third-party hand retargeting library
│   ├── teleimager/                     # [submodule] Camera streaming (ZMQ/WebRTC image server+client)
│   ├── televuer/                       # [submodule] WebXR frontend (vuer-based, served to XR headset)
│   └── utils/
│       ├── episode_writer.py           # Records teleoperation episodes (images, states, actions) to disk
│       ├── motion_switcher.py          # Switches robot between debug mode and AI/locomotion modes
│       ├── weighted_moving_filter.py   # Weighted moving average filter for smoothing signals
│       ├── rerun_visualizer.py         # Rerun-based visualization for online/offline episode data
│       ├── ipc.py                      # ZMQ-based IPC (REQ/REP commands + PUB/SUB heartbeat)
│       └── sim_state_topic.py          # DDS subscriber for simulation state (shared memory bridge)
├── assets/
│   ├── g1/                             # G1 URDF models (g1_body29_hand14.urdf, g1_body23.urdf)
│   ├── h1/                             # H1 URDF model (h1_with_hand.urdf)
│   ├── h1_2/                           # H1_2 URDF model (h1_2.urdf)
│   ├── unitree_hand/                   # Dex3 hand retargeting config (unitree_dex3.yml)
│   ├── inspire_hand/                   # Inspire hand retargeting config (inspire_hand.yml)
│   └── brainco_hand/                   # BrainCo hand retargeting config (brainco.yml)
├── scripts/
│   ├── generate_certs.sh               # SSL certificate generator for XR device connections
│   └── generate_certs.py               # Python alternative for SSL cert generation
├── img/                                # Documentation images
├── CODEBASE_INDEX.md                   # This file
├── README.md                           # Project README
├── CHANGELOG.md                        # Version history
├── requirements.txt                    # Python dependencies
└── LICENSE                             # Apache 2.0
```

## File-by-File Reference

### teleop/teleop_hand_and_arm.py
Main entry point. Parses CLI args, initializes all subsystems, runs the main control loop.

| Function/Block | Description |
|---|---|
| `publish_reset_category(category, publisher)` | Publishes scene reset signal to simulation via DDS |
| `on_press(key)` | Keyboard handler: `r`=start, `q`=stop, `s`=toggle recording |
| `get_state()` | Returns current heartbeat state dict (START, STOP, READY, RECORD_RUNNING) |
| `if __name__ == '__main__':` | Main block: CLI parsing, DDS init, subsystem setup, control loop |

**CLI Arguments:**
- `--arm`: Robot type (`G1_29`, `G1_23`, `H1_2`, `H1`)
- `--ee`: End-effector (`dex1`, `dex3`, `inspire_ftp`, `inspire_dfx`, `brainco`)
- `--sim`: Enable Isaac Sim mode (DDS channel 1)
- `--motion`: Enable locomotion mode (vs debug/direct joint mode)
- `--record`: Enable episode recording
- `--headless`: Disable visualization
- `--ipc`: Use ZMQ IPC instead of sshkeyboard for input
- `--frequency`: Control loop frequency (default 30 Hz)
- `--input-mode`: `hand` (hand tracking) or `controller` (VR controller)
- `--display-mode`: `immersive`, `ego`, or `pass-through`

---

### teleop/robot_control/robot_arm.py
DDS-based arm controllers. Each controller variant follows the same pattern.

| Class | DoF | Arm Joints | Description |
|---|---|---|---|
| `G1_29_ArmController` | 29 body | 14 arm (7L+7R) | G1 EDU with wrist joints |
| `G1_23_ArmController` | 23 body | 10 arm (5L+5R) | G1 without wrist pitch/yaw |
| `H1_2_ArmController` | 27 body | 14 arm (7L+7R) | H1 v2 with wrist joints |
| `H1_ArmController` | 20 body | 8 arm (4L+4R) | H1 v1 (no wrist) |

**Common Methods (all controllers):**

| Method | Description |
|---|---|
| `__init__(motion_mode, simulation_mode)` | Sets up DDS pub/sub, locks non-arm joints at current position, starts control thread at 250 Hz |
| `_subscribe_motor_state()` | Background thread: reads motor states from DDS at 500 Hz |
| `clip_arm_q_target(target_q, velocity_limit)` | Clamps joint velocity to `velocity_limit` rad/s per control step |
| `_ctrl_motor_state()` | Background thread: publishes CRC-protected motor commands at 250 Hz |
| `ctrl_dual_arm(q_target, tauff_target)` | Thread-safe: sets target joint angles and feedforward torques |
| `get_mode_machine()` | Returns current DDS mode machine state |
| `get_current_motor_q()` | Returns all body joint positions as numpy array |
| `get_current_dual_arm_q()` | Returns left+right arm joint positions (concatenated) |
| `get_current_dual_arm_dq()` | Returns left+right arm joint velocities (concatenated) |
| `ctrl_dual_arm_go_home()` | Moves arms to zero position, waits for convergence |
| `speed_gradual_max(t)` | Ramps velocity limit from 20→30 rad/s over `t` seconds |
| `speed_instant_max()` | Sets velocity limit to 30 rad/s immediately |

**Helper Classes:**

| Class | Description |
|---|---|
| `MotorState` | Container: position `q` and velocity `dq` |
| `DataBuffer` | Thread-safe buffer with `GetData()`/`SetData()` |
| `G1_29_JointIndex` | Enum mapping motor IDs 0-34 to joint names (legs, waist, arms) |
| `G1_29_JointArmIndex` | Enum for arm-only motor IDs (15-28) |
| `G1_23_JointIndex` | G1 23-DoF joint mapping |
| `G1_23_JointArmIndex` | G1 23-DoF arm-only mapping |
| `H1_2_JointIndex` | H1_2 joint mapping |
| `H1_2_JointArmIndex` | H1_2 arm-only mapping |
| `H1_JointIndex` | H1 joint mapping (note: DDS order is right-then-left) |
| `H1_JointArmIndex` | H1 arm-only mapping (reordered to left-then-right) |

---

### teleop/robot_control/robot_arm_ik.py
CasADi+Pinocchio inverse kinematics solvers.

| Class | Robot | Arm Joints | Notes |
|---|---|---|---|
| `G1_29_ArmIK` | G1 29-DoF | 14 | Includes wrist pitch/yaw |
| `G1_23_ArmIK` | G1 23-DoF | 10 | Wrist roll only |
| `H1_2_ArmIK` | H1_2 | 14 | Includes wrist pitch/yaw |
| `H1_ArmIK` | H1 | 8 | Elbow only (no wrist) |

**Common Methods (all solvers):**

| Method | Description |
|---|---|
| `__init__(Unit_Test, Visualization)` | Loads URDF, builds reduced model (locks non-arm joints), creates CasADi optimization problem |
| `save_cache()` | Pickles robot models for faster startup |
| `load_cache()` | Loads pickled robot models |
| `scale_arms(human_left, human_right, ...)` | Scales human arm poses to robot proportions |
| `solve_ik(left_wrist, right_wrist, current_q, current_dq)` | Solves IK for both arms. Returns `(joint_angles, feedforward_torques)`. Uses IPOPT with warm-starting, max 30 iterations. Falls back to current state on failure. |

**Optimization Details:**
- Cost: `50 × translation_error² + rotation_error² + 0.02 × regularization² + 0.1 × smoothing²`
- Subject to: joint position limits from URDF
- Solver: IPOPT with warm-starting, 30 max iterations, tolerance 1e-4
- Post-processing: `WeightedMovingFilter` for temporal smoothing, `pin.rnea()` for gravity compensation torques

---

### teleop/robot_control/robot_hand_unitree.py
Unitree dexterous hand and gripper controllers.

| Class | Type | DoF/hand | Description |
|---|---|---|---|
| `Dex3_1_Controller` | Dexterous hand | 7 | Full hand with thumb (3), middle (2), index (2) |
| `Dex1_1_Gripper_Controller` | Gripper | 1 | Simple open/close gripper |

| Class/Enum | Description |
|---|---|
| `Dex3_1_Left_JointIndex` | Motor IDs for left Dex3-1 hand (0-6) |
| `Dex3_1_Right_JointIndex` | Motor IDs for right Dex3-1 hand (0-6) |
| `Gripper_JointIndex` | Motor ID for gripper (0) |
| `_RIS_Mode` | Helper to encode motor mode byte (4-bit id + 3-bit status + 1-bit timeout) |

---

### teleop/robot_control/robot_hand_inspire.py
Inspire dexterous hand controllers.

| Class | Protocol | DoF/hand | Description |
|---|---|---|---|
| `Inspire_Controller_DFX` | DDS (MotorCmds) | 6 | Single DDS topic, both hands |
| `Inspire_Controller_FTP` | DDS (inspire_sdkpy) | 6 | Separate L/R topics, [0-1000] int commands |

| Enum | Description |
|---|---|
| `Inspire_Right_Hand_JointIndex` | Right hand motor IDs 0-5 (pinky→thumb-rotation) |
| `Inspire_Left_Hand_JointIndex` | Left hand motor IDs 6-11 (pinky→thumb-rotation) |

---

### teleop/robot_control/robot_hand_brainco.py
BrainCo revolimb hand controller.

| Class | DoF/hand | Description |
|---|---|---|
| `Brainco_Controller` | 6 | Separate L/R DDS topics, inverted normalization (0=open, 1=closed) |

| Enum | Description |
|---|---|
| `Brainco_Right_Hand_JointIndex` | Right hand: thumb, thumb-aux, index, middle, ring, pinky (0-5) |
| `Brainco_Left_Hand_JointIndex` | Left hand: same order (0-5) |

---

### teleop/robot_control/hand_retargeting.py
Hand retargeting configuration loader.

| Class | Description |
|---|---|
| `HandType` | Enum of supported hand types with paths to YAML configs |
| `HandRetargeting` | Loads retargeting config from YAML, builds left/right retargeters, maps joint names to hardware motor indices |

---

### teleop/utils/episode_writer.py
Episode data recording.

| Class | Description |
|---|---|
| `EpisodeWriter` | Records teleoperation episodes to structured directories |

| Method | Description |
|---|---|
| `__init__(task_dir, task_goal, ...)` | Initializes writer, creates task directory, starts background worker |
| `create_episode()` | Creates new episode directory and JSON file. Returns `True` on success |
| `add_item(colors, depths, states, actions, ...)` | Enqueues one timestep of data for background writing |
| `save_episode()` | Triggers episode finalization (closes JSON, marks available) |
| `close()` | Flushes queue, saves any unsaved episode, stops worker thread |
| `is_ready()` | Returns whether writer is available for new episodes |

**Episode Directory Structure:**
```
task_dir/
└── episode_0001/
    ├── data.json        # Metadata + per-frame states/actions
    ├── colors/          # Camera images (JPEG)
    ├── depths/          # Depth images (JPEG)
    └── audios/          # Audio data (NPY)
```

---

### teleop/utils/motion_switcher.py
Motion mode control.

| Class | Description |
|---|---|
| `MotionSwitcher` | Enters/exits debug mode (direct joint control) via MotionSwitcherClient |
| `LocoClientWrapper` | Locomotion commands: `Move(vx, vy, vyaw)`, `Enter_Damp_Mode()` |

---

### teleop/utils/weighted_moving_filter.py
Signal smoothing filter.

| Class | Description |
|---|---|
| `WeightedMovingFilter` | Weighted moving average with configurable window size and weights |

| Method | Description |
|---|---|
| `__init__(weights, data_size)` | Weights must sum to 1.0. `data_size` = number of channels |
| `add_data(new_data)` | Adds sample, applies convolution filter. Skips duplicates |
| `filtered_data` | Property returning latest filtered output |

---

### teleop/utils/rerun_visualizer.py
Rerun-based visualization.

| Class | Description |
|---|---|
| `RerunEpisodeReader` | Loads recorded episodes from disk (images, states, actions) |
| `RerunLogger` | Logs data to Rerun viewer with time series blueprint |

| Method | Description |
|---|---|
| `RerunEpisodeReader.return_episode_data(episode_idx)` | Returns list of item_data dicts for given episode |
| `RerunLogger.log_item_data(item_data)` | Logs one timestep (states/actions as scalars) |
| `RerunLogger.log_episode_data(episode_data)` | Logs full episode sequentially |

---

### teleop/utils/ipc.py
ZMQ-based inter-process communication.

| Class | Description |
|---|---|
| `IPC_Server` | REQ/REP command handler + PUB heartbeat publisher |
| `IPC_Client` | REQ command sender + SUB heartbeat subscriber |

**Protocol:**
- Commands: `CMD_START` (→key `r`), `CMD_STOP` (→key `q`), `CMD_RECORD_TOGGLE` (→key `s`)
- Heartbeat: `{START, STOP, READY, RECORD_RUNNING}` published at 10 Hz
- Transport: Unix domain sockets (`ipc://@xr_teleoperate_data.ipc`, `ipc://@xr_teleoperate_hb.ipc`)

---

### teleop/utils/sim_state_topic.py
Simulation state bridge.

| Class | Description |
|---|---|
| `SharedMemoryManager` | Manages POSIX shared memory for cross-process data exchange |
| `SimStateSubscriber` | Subscribes to `rt/sim_state` DDS topic, writes to shared memory |

| Function | Description |
|---|---|
| `start_sim_state_subscribe(shm_name, shm_size)` | Convenience: creates and starts a SimStateSubscriber |

---

## Key Abstractions and Design Patterns

### Controller Pattern
All arm controllers (`G1_29_ArmController`, etc.) and hand controllers (`Dex3_1_Controller`, etc.) follow the same architecture:
1. **DDS Subscribe Thread**: Reads motor states at high frequency (~500 Hz)
2. **DDS Publish Thread/Process**: Writes motor commands at control frequency (250 Hz for arms, 100-200 Hz for hands)
3. **Thread-safe Interface**: `ctrl_dual_arm()` / `ctrl_dual_hand()` sets targets behind a lock

### IK Solver Pattern
All IK solvers (`G1_29_ArmIK`, etc.) follow the same architecture:
1. Load URDF → build reduced robot model (lock non-arm joints)
2. Create CasADi symbolic optimization problem (translation + rotation + regularization + smoothing)
3. `solve_ik()`: set parameters → IPOPT solve → smooth filter → RNEA for torques

### Recording Pattern
`EpisodeWriter` uses a producer-consumer pattern:
1. Main thread calls `add_item()` → enqueues data
2. Background worker thread dequeues → writes images to disk, appends JSON
3. `save_episode()` sets a flag → worker finalizes JSON when queue is empty

### State Machine
The main teleop loop uses a simple state machine controlled by global flags:
```
[Idle] --r--> [Running] --s--> [Recording] --s--> [Saving] --auto--> [Running]
  ^                                                                       |
  └──────────────────────────q─────────────────────────────────────────────┘
```

### DDS Communication
- **Channel 0**: Real robot communication
- **Channel 1**: Simulation (Isaac Sim) communication
- **Topics**: `rt/lowcmd` (debug mode), `rt/arm_sdk` (motion mode), `rt/lowstate` (state feedback)
- **CRC**: All motor commands include CRC checksum for integrity
