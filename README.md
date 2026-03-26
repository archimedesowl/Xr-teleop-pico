# Xr-teleop-pico — Agentic-Friendly XR Teleoperation for Unitree G1

![Python 3.10](https://img.shields.io/badge/python-3.10-blue)
![Ubuntu 20.04 / 22.04](https://img.shields.io/badge/ubuntu-20.04%20%7C%2022.04-orange)
![License Apache 2.0](https://img.shields.io/badge/license-Apache%202.0-green)

A fork of [unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate), enhanced with comprehensive docstrings, type hints, auto-detection of network interfaces and DDS channels, and developer tooling designed for AI coding agents and vibe coders. Optimized for the **PICO + Unitree G1 (29 DoF)** workflow while maintaining full support for Quest, Apple Vision Pro, H1, and H1_2.

## What's Different in This Fork

- **Comprehensive docstrings** — Google-style docstrings on every function and class in all core files
- **Full type hints** — Type annotations on all function signatures for IDE and agent tooling support
- **CODEBASE_INDEX.md** — Complete file-by-file reference with class/function tables, architecture diagrams, and design pattern documentation
- **AGENTS.md** — AI agent developer guide with modification patterns, data flow descriptions, and common pitfalls
- **One-command setup** — `./setup.sh --device pico` handles conda, deps, submodules, and SSL certs
- **Auto-detect network interface** — Finds the NIC on the 192.168.123.x subnet automatically (PATCH-002)
- **DDS timeout diagnostics** — 15-second timeout with actionable error messages instead of hanging forever (PATCH-004)
- **Sim-mode DDS channel fix** — Automatically uses DDS channel 1 in `--sim` mode (PATCH-001)
- **Debug mode error handling** — Clear error messages when debug mode entry fails (PATCH-005)
- **SSL cert generator** — `scripts/generate_certs.sh pico` for one-command certificate setup (PATCH-003)

## Quick Start

```bash
# 1. Clone with submodules
git clone --recurse-submodules https://github.com/archimedesowl/Xr-teleop-pico.git
cd Xr-teleop-pico

# 2. Run setup (installs conda env, deps, certs)
./setup.sh --device pico

# 3. Launch teleoperation
conda activate tv
cd teleop
python teleop_hand_and_arm.py --arm=G1_29
```

## Architecture

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

## Supported Hardware

| Component | Options | Status |
|---|---|---|
| **Robot** | G1 (29 DoF), G1 (23 DoF), H1_2 (7-DoF arm), H1 (4-DoF arm) | All supported |
| **XR Device** | PICO 4 Ultra Enterprise, Meta Quest 3, Apple Vision Pro | All supported |
| **End-Effector** | Unitree Dex3-1, Unitree Dex1-1 gripper, Inspire DFX, Inspire FTP, BrainCo | All supported |
| **Simulation** | Isaac Sim via [unitree_sim_isaaclab](https://github.com/unitreerobotics/unitree_sim_isaaclab) | Supported |

## CLI Arguments

| Argument | Description | Default |
|---|---|---|
| `--arm` | Robot type: `G1_29`, `G1_23`, `H1_2`, `H1` | `G1_29` |
| `--ee` | End-effector: `dex1`, `dex3`, `inspire_ftp`, `inspire_dfx`, `brainco` | None |
| `--sim` | Enable Isaac Sim mode | Off |
| `--motion` | Enable locomotion mode (vs debug/direct joint) | Off |
| `--record` | Enable episode recording | Off |
| `--headless` | Run without display | Off |
| `--frequency` | Control loop frequency in Hz | 30.0 |
| `--input-mode` | `hand` (hand tracking) or `controller` (VR controller) | `hand` |
| `--display-mode` | `immersive`, `ego`, or `pass-through` | `immersive` |
| `--network-interface` | Network interface for DDS (auto-detected if omitted) | Auto |
| `--ipc` | Use ZMQ IPC instead of sshkeyboard | Off |

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Waiting to subscribe dds...` hangs forever | DDS channel mismatch or wrong network interface | In `--sim` mode, channel 1 is now auto-selected. For physical robot, ensure `--network-interface` points to the 192.168.123.x NIC. The patched code now times out after 15s with diagnostics. |
| Black camera feed in XR headset | teleimager not running or WebRTC cert not trusted | Start `teleimager-server` on PC2. Open `https://192.168.123.164:60001` in headset browser first to trust the cert. |
| `[ClientStub] send request error` / `Enter debug mode: Failed` | Robot not reachable or not in zero-torque state | Power on the G1, ensure it is in zero-torque state. Run `ping 192.168.123.161` to verify connectivity. Check that your host is on the 192.168.123.x subnet. |
| SSL certificate errors in headset browser | Certs not generated or not matching device type | Run `./scripts/generate_certs.sh pico` (or `quest`/`avp`). For Apple Vision Pro, AirDrop `rootCA.pem` to the device and install it. |
| `No network interface found on 192.168.123.x subnet` | Host not connected to robot network | Connect an Ethernet cable from host to the robot's switch. Verify with `ip addr show` that you have a 192.168.123.x address. |
| Import errors for `pinocchio` or `casadi` | Wrong conda environment or missing deps | Run `conda activate tv`. If env doesn't exist, run `./setup.sh --device pico`. |

## Documentation

- **[CODEBASE_INDEX.md](CODEBASE_INDEX.md)** — Complete file-by-file reference with every class and function documented
- **[AGENTS.md](AGENTS.md)** — AI agent developer guide: architecture, modification patterns, data flow, pitfalls
- **[CONTRIBUTING.md](CONTRIBUTING.md)** — How to contribute: code style, branching, PR process
- **[CHANGELOG.md](CHANGELOG.md)** — Version history

## Credits

This is a fork of [unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate) by [Unitree Robotics](https://www.unitree.com/).

Built on top of: [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision), [dex-retargeting](https://github.com/dexsuite/dex-retargeting), [vuer](https://github.com/vuer-ai/vuer), [Pinocchio](https://github.com/stack-of-tasks/pinocchio), [CasADi](https://github.com/casadi/casadi), [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python).

## License

Apache 2.0 — see [LICENSE](LICENSE).
