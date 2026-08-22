# Stability-Gated Closed-Loop Eye-in-Hand Tracking and Grasping

ROS 2 research code for semantic RGB-D perception, continuous target tracking, and safety-gated Piper grasping of a manually moved target.

This repository accompanies an **IEEE EPIC 2026 short-paper submission**. It is research software, not a certified industrial safety controller. Test in plan-only mode first, keep the workspace clear, provide an emergency stop, and verify every motion on the actual robot before enabling actuation.

## Paper

**Stability-Gated Closed-Loop Eye-in-Hand Tracking and Grasping of a Manually Moved Target**

Zhang Yue and Xinlei Lin, IEEE EPIC 2026 short-paper submission.

The paper is not described as accepted or published. Citation information will be updated after the conference decision.

## Project provenance

This project was jointly developed by Zhang Yue and Wei Liu. An earlier version of the shared codebase, including contributions from both collaborators, was published under Wei Liu's GitHub account at [`WillLiu322/foam-grasp-ros2`](https://github.com/WillLiu322/foam-grasp-ros2). The present repository contains the complete codebase for the subsequent closed-loop tracking and grasping work and is released and maintained by Zhang Yue with Wei Liu's permission.

The continuous target-tracking and stability-gated tracking-to-grasp control scheme presented in the current work was proposed by Zhang Yue. Wei Liu provided the Piper manipulator and RGB-D experimental platform and collaborated on the development and validation of the system.

The provenance statement above describes collaboration history and does not add Wei Liu to the paper author list.

## System scope

The implementation includes:

- RGB-D fusion and semantic segmentation of cube, cylinder, and sphere classes;
- eye-in-hand camera-to-base-frame localization;
- continuous selected-class target following with filtered polar displacement;
- bounded joint-rate servoing with preflight workspace and MoveIt IK/collision checks;
- a continuous 5 s stability/readiness gate, stable-sample relatching, and path validation;
- operator safety authorization and an execution token before physical `GRASP -> LIFT`.

The 5 s gate only authorizes `HOLD/readiness`; it does not automatically start a physical grasp. The operator must confirm that the workspace is clear and issue the execution command.

The current evidence does not establish motion prediction, same-class distractor rejection, repeated-trial grasp success, or statistical superiority over a baseline.

## Repository layout

```text
config/                         machine-level project and runtime settings
data/                           dataset instructions only; raw data is excluded
dependencies/                   vcs manifests for ROS 2 dependencies
docs/                           deployment, operation, and reproducibility notes
patches/                        minimal upstream patches
runtime/                        model/calibration instructions and model card
scripts/                        installation, validation, and release helpers
training/                       capture, labeling, training, and evaluation scripts
workspaces/app_ws/              the foam_grasp and foam_grasp_sim ROS 2 packages
analysis/                       read-only campaign summaries and plots
```

Raw images, LabelMe annotations, generated masks, training runs, rosbag files, model checkpoints, and hardware calibration JSON are not committed to ordinary Git history. See `data/README.md` and `runtime/models/README.md` or `runtime/calibration/README.md` for the intended release channels.

The Gazebo benchmark runner and simulated eye-in-hand RGB-D composition are
documented in [`docs/SIMULATION_BENCHMARK.md`](docs/SIMULATION_BENCHMARK.md).
Analysis tools consume campaign artifacts only and write to a separate
`<campaign_id>-analysis/` directory.

## Requirements

- Ubuntu 22.04 LTS
- ROS 2 Humble
- NVIDIA driver and a compatible CUDA/PyTorch environment for segmentation
- Piper manipulator and gripper
- Orbbec DaBai DC1 RGB-D camera
- MoveIt 2 and the dependency workspaces listed in `dependencies/*.repos`

## Installation and validation

```bash
git clone https://github.com/Zhang-GTIIT/stability-gated-eye-in-hand-grasping.git
cd stability-gated-eye-in-hand-grasping
chmod +x install.sh scripts/*.sh
cp config/project.env.example config/project.env
# Edit config/project.env for the target rig before a supervised run.
./install.sh
./scripts/validate_project.sh
./scripts/check_github_ready.sh
```

The installer does not start ROS, enable the robot, or send motion commands. Start with the plan-only launch files and verify the camera, transforms, IK, collision checks, and gripper feedback before any physical execution.

## Runtime assets

The trained model and rig-specific hand-eye calibration are intentionally distributed as versioned release assets rather than committed to normal Git history. The current `config/runtime-release.env` retains the verified legacy release source for those assets because the model and calibration have not been migrated into this repository's releases. Do not substitute a new URL until matching assets and checksums have been published and tested.

Calibration is valid only for the camera, mount, and Piper rig used to create it. A different rig requires a new calibration profile and a separate release asset.

## License and third-party software

The project is released under the Apache License 2.0. Third-party components retain their respective upstream licenses; see [`THIRD_PARTY.md`](THIRD_PARTY.md) and the dependency manifests.

## Security and responsible use

Do not commit API tokens, passwords, private keys, `.env` secrets, machine inventories, or local absolute paths. Use the release checklist and security guidance before publishing a new tag or runtime asset.
