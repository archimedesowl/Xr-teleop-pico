# Contributing to Xr-teleop-pico

## Dev Environment Setup

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/archimedesowl/Xr-teleop-pico.git
cd Xr-teleop-pico

# Option A: automated setup
./setup.sh --device pico

# Option B: manual setup
conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate tv
git submodule update --init --depth 1
cd teleop/teleimager && pip install -e . --no-deps && cd ../..
cd teleop/televuer && pip install -e . && cd ../..
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/unitree_sdk2_python
pip install -e /tmp/unitree_sdk2_python
pip install -r requirements.txt
```

## Code Style

- **Docstrings:** Google-style on all public functions and classes
- **Type hints:** Required on all function signatures
- **Naming:** `snake_case` for functions and variables, `PascalCase` for classes
- **Line length:** 120 characters max
- **Imports:** Standard library first, then third-party, then local. Absolute imports preferred.

Example:
```python
def solve_ik(
    self,
    target_pose: np.ndarray,
    arm_side: str = "left",
) -> np.ndarray:
    """Solve inverse kinematics for a target end-effector pose.

    Args:
        target_pose: 4x4 homogeneous transformation matrix.
        arm_side: Which arm to solve for ("left" or "right").

    Returns:
        Joint angles array of shape (N,).

    Raises:
        ValueError: If arm_side is not "left" or "right".
    """
```

## Branch Naming

- `feature/short-description` — New features
- `fix/short-description` — Bug fixes
- `docs/short-description` — Documentation changes

## Commit Messages

Use the format: `[type] short description`

Types: `[feature]`, `[fix]`, `[docs]`, `[refactor]`, `[upgrade]`, `[recover]`

Examples:
```
[feature] add BrainCo hand controller
[fix] DDS timeout with diagnostics
[docs] add AGENTS.md for AI agent developers
[upgrade] televuer submodule to v1.5
```

## Pull Request Process

1. Create a branch from `main` following the naming convention above
2. Make your changes, ensuring all functions have docstrings and type hints
3. Test in simulation mode (`--sim`) if modifying control code
4. Fill out the PR template below
5. Request review

### PR Template

```markdown
## Summary

What this PR does and why.

## Changes

- Bullet list of specific changes

## Testing

How you verified the changes work:
- [ ] Tested in simulation mode
- [ ] Tested with physical robot (if applicable)
- [ ] Verified no regressions in existing CLI args

## Notes

Any additional context, trade-offs, or follow-up items.
```

## Issue Template

```markdown
## Description

What happened vs what you expected.

## Environment

- OS: (e.g., Ubuntu 22.04)
- XR Device: (e.g., PICO 4 Ultra Enterprise)
- Robot: (e.g., G1 29 DoF)
- Branch/commit: (e.g., main @ abc1234)

## Steps to Reproduce

1. ...
2. ...
3. ...

## Logs

Paste any relevant terminal output.
```

## Submodules

The following directories are git submodules — **do not modify files in them directly**:

- `teleop/televuer/` — WebXR frontend
- `teleop/teleimager/` — Camera streaming
- `teleop/robot_control/dex-retargeting/` — Hand retargeting library

If you need to change a submodule, fork the upstream repo, make changes there, and update the submodule reference in `.gitmodules`.

## Key Files

Before making changes, read these files for context:

- [CODEBASE_INDEX.md](CODEBASE_INDEX.md) — Complete file-by-file reference
- [AGENTS.md](AGENTS.md) — Architecture overview and modification patterns
- [CHANGELOG.md](CHANGELOG.md) — Version history and recent changes
