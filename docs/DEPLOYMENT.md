# Deployment

This document describes a clean Ubuntu 22.04 / ROS 2 Humble deployment. The installer prepares software and workspaces; it does not start ROS, enable the Piper arm, or send motion commands.

## Clone and install

```bash
git clone https://github.com/Zhang-GTIIT/stability-gated-eye-in-hand-grasping.git
cd stability-gated-eye-in-hand-grasping
chmod +x install.sh scripts/*.sh
./install.sh
./scripts/validate_project.sh
```

The dependency manifests in `dependencies/` are imported into separate workspaces. Do not copy `build/`, `install/`, `log/`, or a Python virtual environment between machines.

## Runtime resources

The repository intentionally excludes the trained model and rig-specific hand-eye calibration. `config/runtime-release.env` records the currently verified legacy release source and checksums. The source must not be changed to a new repository until matching release assets have been published and tested.

The model, calibration, raw dataset, LabelMe annotations, generated masks, training runs, and rosbags are not ordinary Git repository contents. See the README and the runtime/data README files for their intended release channels.

## First hardware run

1. Check the emergency stop and clear the workspace.
2. Launch the plan-only or preview nodes.
3. Verify RGB-D topics, transforms, MoveIt IK, collision checks, and gripper feedback.
4. Confirm the preflight-derived workspace and joint envelope for the actual rig.
5. Only then run a physical grasp. The 5 s stability gate authorizes `HOLD/readiness`; an operator confirmation and execution command are still required before `GRASP -> LIFT`.
