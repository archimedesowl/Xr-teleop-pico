# AI Agent Developer Guide

This guide is optimized for AI coding agents (Claude Code, Cursor, Copilot, etc.) and developers who want to modify this codebase without deep robotics expertise.

## How This Codebase Works

This is an XR teleoperation system. An XR headset (PICO, Quest, or Apple Vision Pro) streams hand/controller tracking data over WebSocket to a host machine. The host solves inverse kinematics to convert wrist poses into joint angles, then publishes motor commands to a Unitree humanoid robot (G1 or H1) over DDS (Data Distribution Service).

The system runs a ~30 Hz control loop in `teleop_hand_and_arm.py`. Each frame:

1. **televuer** captures XR data → `TeleVuerWrapper.get_tele_data()` returns wrist poses + hand skeleton
2. **robot_arm_ik.py** solves IK → `ArmIK.solve_ik()` returns joint angles + feedforward torques
3. **robot_arm.py** sends commands → `ArmController.ctrl_dual_arm()` publishes via DDS at 250 Hz
4. **hand_retargeting.py** maps hand tracking → `HandController.ctrl_dual_hand()` drives fingers
5. **teleimager** streams camera from robot → displayed in XR headset

Three submodules are external (do NOT modify): `teleop/televuer/`, `teleop/teleimager/`, `teleop/robot_control/dex-retargeting/`.

Key dependencies: Pinocchio (FK/IK), CasADi (optimization), unitree_sdk2_python (DDS), ZMQ (IPC), vuer (WebXR).

## Entry Points

| File | What It Does |
|---|---|
| `teleop/teleop_hand_and_arm.py` | Main entry point. Parses CLI args, initializes subsystems, runs control loop. Start here. |
| `teleop/robot_control/robot_arm.py` | DDS-based arm controllers. Manages motor state pub/sub at 250-500 Hz. |
| `teleop/robot_control/robot_arm_ik.py` | CasADi+Pinocchio IK solvers. Converts wrist poses → joint angles. |
| `teleop/robot_control/robot_hand_unitree.py` | Unitree Dex3-1 hand and Dex1-1 gripper controllers. |
| `teleop/robot_control/robot_hand_inspire.py` | Inspire DFX and FTP dexterous hand controllers. |
| `teleop/robot_control/robot_hand_brainco.py` | BrainCo revolimb hand controller. |
| `teleop/robot_control/hand_retargeting.py` | Maps XR hand tracking to robot hand joints via dex-retargeting. |
| `teleop/utils/episode_writer.py` | Records teleoperation episodes (images + states + actions) to disk. |
| `teleop/utils/motion_switcher.py` | Switches between debug mode and locomotion mode. |
| `teleop/utils/ipc.py` | ZMQ-based IPC for controlling teleop from external programs. |

## Common Modification Patterns

### "I want to change the IK solver"
Modify `teleop/robot_control/robot_arm_ik.py`. Each robot variant has its own class (e.g., `G1_29_ArmIK`). The `solve_ik()` method is the core — it sets CasADi parameters, calls IPOPT, applies smoothing, and computes gravity compensation torques via `pin.rnea()`. Optimization weights are in `__init__()`: translation=50, rotation=1, regularization=0.02, smoothing=0.1.

### "I want to add a new hand type"
1. Create `teleop/robot_control/robot_hand_X.py` following `robot_hand_unitree.py` as a template
2. Define joint index enums and a controller class with `ctrl_dual_hand(left_angles, right_angles)` method
3. Add a retargeting YAML config in `assets/X_hand/`
4. Register the new hand type in `teleop/robot_control/hand_retargeting.py` `HandType` enum
5. Add the `--ee` option in `teleop/teleop_hand_and_arm.py` CLI args and initialization logic

### "I want to change XR input handling"
The XR frontend is in the `teleop/televuer/` submodule. The key file is `src/televuer/tv_wrapper.py` which post-processes captured data from the WebXR frontend. **Do not modify submodule files directly** — fork the submodule repo and update `.gitmodules`.

### "I want to add a new robot model"
1. Add a URDF to `assets/new_robot/`
2. Create a new `ArmController` class in `robot_arm.py` following `G1_29_ArmController` pattern (define joint index enums, set DoF counts, configure DDS topics)
3. Create a new `ArmIK` class in `robot_arm_ik.py` following `G1_29_ArmIK` pattern (load URDF, define locked joints, build CasADi problem)
4. Add the `--arm` option in `teleop_hand_and_arm.py`

### "I want to change the control frequency"
Pass `--frequency <hz>` on the CLI. Default is 30 Hz. This affects the main control loop in `teleop_hand_and_arm.py`. The arm controller's internal publish rate (250 Hz) and subscribe rate (500 Hz) are separate and hardcoded in `robot_arm.py`.

### "I want to record training data"
Pass `--record` on the CLI. Press `r` to start teleop, then `s` to start/stop recording. Data is written by `teleop/utils/episode_writer.py` to `teleop/utils/data/` with structure: `episode_NNNN/{data.json, colors/, depths/, audios/}`. Configure with `--task-dir`, `--task-name`, `--task-goal`, `--task-desc`, `--task-steps`.

### "I want to run in simulation"
Pass `--sim` on the CLI. This uses DDS channel 1 instead of channel 0. Requires [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) running separately. Example: `python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim`

### "I want to add walking/locomotion"
Pass `--motion` on the CLI. This uses `teleop/utils/motion_switcher.py` to switch the robot from debug mode to motion control mode. Locomotion commands go through `LocoClientWrapper.Move(vx, vy, vyaw)`. IPC commands (`teleop/utils/ipc.py`) can also control locomotion from external programs.

## Key Abstractions

| Class | File | Interface |
|---|---|---|
| `G1_29_ArmController` | `robot_arm.py` | `ctrl_dual_arm(q_target, tauff_target)`, `get_current_dual_arm_q()`, `ctrl_dual_arm_go_home()` |
| `G1_29_ArmIK` | `robot_arm_ik.py` | `solve_ik(left_wrist, right_wrist, current_q, current_dq)` → `(q, tau)` |
| `Dex3_1_Controller` | `robot_hand_unitree.py` | `ctrl_dual_hand(left_angles, right_angles)` |
| `HandRetargeting` | `hand_retargeting.py` | Loads YAML config, `retarget(hand_skeleton)` → motor angles |
| `EpisodeWriter` | `episode_writer.py` | `create_episode()`, `add_item(...)`, `save_episode()`, `close()` |
| `TeleVuerWrapper` | `televuer/tv_wrapper.py` | `get_tele_data()` → wrist poses + hand skeleton |
| `IPC_Server` / `IPC_Client` | `ipc.py` | `CMD_START`, `CMD_STOP`, `CMD_RECORD_TOGGLE` over ZMQ |
| `MotionSwitcher` | `motion_switcher.py` | Enter/exit debug mode; `LocoClientWrapper.Move(vx, vy, vyaw)` |

## Data Flow — One Frame of the Control Loop

```
teleop_hand_and_arm.py main loop iteration:
│
├─ tv_wrapper.get_tele_data()
│   └─ Returns: left_wrist_mat4, right_wrist_mat4, left_hand_skeleton, right_hand_skeleton
│
├─ arm_ik.solve_ik(left_wrist, right_wrist, current_q, current_dq)
│   ├─ CasADi IPOPT optimization (max 30 iterations, warm-started)
│   ├─ WeightedMovingFilter for temporal smoothing
│   ├─ pin.rnea() for gravity compensation torques
│   └─ Returns: q_target (joint angles), tau_ff (feedforward torques)
│
├─ arm_controller.ctrl_dual_arm(q_target, tau_ff)
│   └─ Sets targets behind a lock; 250 Hz publish thread sends via DDS
│
├─ hand_retargeting.retarget(hand_skeleton)
│   └─ dex-retargeting library maps hand joints to motor angles
│
├─ hand_controller.ctrl_dual_hand(left_angles, right_angles)
│   └─ Publishes hand motor commands via DDS
│
└─ episode_writer.add_item(...)  [if recording]
    └─ Enqueues data for background disk writing
```

## Configuration

**CLI Arguments:** See the table in [README.md](README.md#cli-arguments).

**Environment Variables:**
- `XR_TELEOP_CERT` — Path to SSL certificate file (cert.pem)
- `XR_TELEOP_KEY` — Path to SSL key file (key.pem)
- `CYCLONEDDS_URI` — CycloneDDS configuration XML (auto-set by `--network-interface`)

**YAML Configs:**
- `assets/unitree_hand/unitree_dex3.yml` — Dex3 hand retargeting mapping
- `assets/inspire_hand/inspire_hand.yml` — Inspire hand retargeting mapping
- `assets/brainco_hand/brainco.yml` — BrainCo hand retargeting mapping

**URDF Models:** `assets/{g1,h1,h1_2}/` — Robot descriptions loaded by IK solvers.

## Testing Your Changes

**Simulation mode (safest):**
```bash
# Start Isaac Sim first (separate terminal)
python teleop_hand_and_arm.py --arm=G1_29 --ee=dex3 --sim
```

**Headless mode (no XR device needed for backend testing):**
```bash
python teleop_hand_and_arm.py --arm=G1_29 --headless --sim
```

**IPC mode (programmatic control for testing):**
```bash
# Terminal 1: start teleop with IPC
python teleop_hand_and_arm.py --arm=G1_29 --sim --ipc

# Terminal 2: send commands
python -c "from utils.ipc import IPC_Client; c = IPC_Client(); c.send_cmd('CMD_START')"
```

**Unit-test IK solver:**
```python
from robot_control.robot_arm_ik import G1_29_ArmIK
ik = G1_29_ArmIK(Unit_Test=True, Visualization=True)
# Opens meshcat visualizer — verify FK/IK visually
```

## Common Pitfalls

1. **Modifying submodules directly** — `teleop/televuer/`, `teleop/teleimager/`, and `teleop/robot_control/dex-retargeting/` are git submodules. Do not edit files in these directories. Fork them separately if needed.

2. **Forgetting DDS channel for sim** — Physical robot uses DDS channel 0, simulation uses channel 1. The `--sim` flag handles this automatically, but if you initialize DDS manually, use `ChannelFactoryInitialize(1)` for sim.

3. **Joint ordering mismatch** — H1's DDS motor order is right-then-left, but the IK solver expects left-then-right. `H1_JointArmIndex` handles the reordering. If adding a new robot, check the DDS topic joint ordering carefully.

4. **IK solver cache invalidation** — `robot_arm_ik.py` caches the Pinocchio model as a pickle. If you change a URDF, delete the cache file or the solver will use stale data.

5. **Control frequency vs publish frequency** — The main loop runs at `--frequency` Hz (default 30). The arm controller's DDS publish thread runs at 250 Hz, interpolating between targets. Changing `--frequency` does NOT change the DDS rate.

6. **Thread safety** — `ctrl_dual_arm()` and `ctrl_dual_hand()` are thread-safe (use locks). Reading motor state (`get_current_dual_arm_q()`) is also thread-safe. Do not access internal buffers directly.

7. **Network interface required for physical robot** — DDS communication with a physical robot requires specifying the correct network interface (the one on the 192.168.123.x subnet). Auto-detection handles this, but verify with `ip addr show` if DDS fails to connect.

8. **CRC checksums** — All motor commands include CRC for integrity. If you modify the motor command structure in `robot_arm.py`, you must update the CRC calculation or commands will be rejected by the robot.
