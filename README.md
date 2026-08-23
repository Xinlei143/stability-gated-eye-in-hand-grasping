# Stability-Gated Closed-Loop Eye-in-Hand Tracking and Grasping

ROS 2 research code for stability-gated closed-loop eye-in-hand tracking and grasping, combining semantic RGB-D perception, Piper manipulation, Gazebo/MoveIt simulation, and reproducible benchmark evaluation under target motion and perception disturbances.

This repository accompanies an **IEEE EPIC 2026 short-paper submission**. It is research software, not a certified industrial safety controller. Test in plan-only mode first, keep the workspace clear, provide an emergency stop, and verify every motion on the actual robot before enabling actuation.

## Paper

**Stability-Gated Closed-Loop Eye-in-Hand Tracking and Grasping of a Manually Moved Target**

Zhang Yue and Xinlei Lin, IEEE EPIC 2026 short-paper submission.

The paper is not described as accepted or published. Citation information will be updated after the conference decision.

## Project provenance

The original project was developed through collaboration between Zhang Yue and Wei Liu. An earlier version of the shared codebase, including contributions from both collaborators, was published under Wei Liu's GitHub account at [`WillLiu322/foam-grasp-ros2`](https://github.com/WillLiu322/foam-grasp-ros2). That repository predates the present repository and already contained major components of the robotic grasping system, including semantic RGB-D perception, camera-to-base target localization, MoveIt-based motion planning and validation, target latching, and the Piper grasp-execution pipeline. The codebase was subsequently provided for the present study with Wei Liu's permission.

The continuous target-tracking and stability-gated tracking-to-grasp study direction presented in the associated manuscript was proposed by Zhang Yue, who leads the manuscript preparation and serves as first author. Wei Liu provided the Piper manipulator and RGB-D experimental platform and collaborated on the development and validation of the physical system.

Building on the pre-existing grasping pipeline, Xinlei Lin led the subsequent software implementation and evaluation development in this repository, including the real/simulation execution abstraction, Gazebo and MoveIt simulation infrastructure, moving-target and simulated-perception models, snapshot/tracking/gated method-policy integration, benchmark orchestration, latency/noise/dropout evaluation, metrics and offline statistical analysis, simulated eye-in-hand RGB-D integration, and end-to-end simulation grasp validation. These contributions are documented in the repository history, primarily through PRs #1–#10.

The present repository therefore combines the pre-existing robotic grasping system, the stability-gated tracking study direction, and the subsequent simulation, benchmarking, integration, and evaluation software developed for the current work.

The provenance statement above describes collaboration and software-development history and does not determine the authorship of the associated manuscript.

## System scope

### Physical perception and grasping pipeline

The physical-system pipeline includes:

- semantic RGB-D perception for cube, cylinder, and sphere classes;
- depth fusion and eye-in-hand camera-to-base target localization;
- MoveIt-based IK, collision checking, and grasp-path validation;
- continuous selected-class target following with bounded joint-rate servoing;
- stability-gated target commitment and grasp readiness;
- operator-authorized physical `GRASP -> LIFT` execution with explicit safety checks.

The 5 s gate only authorizes `HOLD/readiness`; it does not automatically start a physical grasp. The operator must confirm that the workspace is clear and issue the execution command.

### Simulation and evaluation framework

The repository also includes a reproducible Gazebo/MoveIt simulation and evaluation framework for closed-loop target tracking and grasping:

- `snapshot`, `tracking`, and `gated` method policies under a common evaluation interface;
- moving-target and simulated-perception models with configurable latency, noise, and dropout;
- simulated eye-in-hand RGB-D perception using the same perception/localization pipeline as the physical system;
- deterministic benchmark campaigns with paired seeds and configurations;
- baseline comparison, latency sweep, noise sweep, dropout sweep, and gate-ablation suites;
- run-level telemetry and metrics for tracking error, readiness error, grasp-initiation error, time-to-ready, gate resets, planning success, and task success;
- offline paired analysis, deterministic bootstrap confidence intervals, result tables, and publication-oriented plots.

### Gazebo grasp stabilization backend

The formal simulator baseline uses `grasp_stabilization_mode:=gazebo_grasp_fix`.
This selects the pinned `gazebo_grasp_fix` plugin and the no-attachment world;
the legacy contact-confirmed attach service remains off. The plugin only
stabilizes an already commanded grasp in Gazebo. It does not replace
perception, target tracking, readiness gating, target commitment, or the grasp
trigger. Mechanical task success is accepted only from the physical lift/hold
fields in `metrics.json`, never from attachment or motion completion alone.

Install the pinned third-party plugin once, then source the project runtime:

```bash
bash scripts/setup_gazebo_grasp_plugin.sh
source scripts/source_env.sh
```

The setup script records the exact upstream revision in
`dependencies/gazebo_grasp_plugin.repos` and installs the shared library under
`.external/gazebo-grasp-install/lib/`. See `THIRD_PARTY.md` for license and
patch provenance.

## Reproducible simulation and benchmarking

The simulation framework provides controlled evaluation of three target-selection and tracking policies:

- `snapshot`: commits the first valid target observation;
- `tracking`: continuously follows the latest target estimate with bounded replanning;
- `gated`: commits a target only after the configured stability criteria are satisfied.

Packaged benchmark suites cover baseline comparison, latency, perception noise, observation dropout, and stability-gate ablations. Trials use deterministic configuration hashes and paired seeds so methods can be compared under matched target trajectories and disturbance conditions.

Each run records target ground truth, observations, selected and committed targets, TCP state, joint state, readiness, and execution events. Offline analysis produces run-level metrics, grouped summaries, paired method differences, deterministic bootstrap confidence intervals, and publication-oriented plots.

See [`docs/SIMULATION_BENCHMARK.md`](docs/SIMULATION_BENCHMARK.md) for the complete benchmark contract and reproduction commands.

## Current limitations

The current repository provides the infrastructure for controlled baseline comparisons and repeated simulation campaigns, but no claim of statistical superiority is made here unless supported by completed experimental results. The current system also does not establish motion prediction, same-class distractor rejection, or repeated-trial physical grasp success.

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
workspaces/app_ws/              physical foam_grasp and Gazebo/benchmark foam_grasp_sim ROS 2 packages
analysis/                       offline metrics, paired comparisons, bootstrap CIs, tables, and plots
```

Raw images, LabelMe annotations, generated masks, training runs, rosbag files, model checkpoints, and hardware calibration JSON are not committed to ordinary Git history. See `data/README.md` and `runtime/models/README.md` or `runtime/calibration/README.md` for the intended release channels.

Analysis tools consume campaign artifacts only and write to a separate
`<campaign_id>-analysis/` directory.

## Requirements

### Simulation and benchmark workflow

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo and MoveIt 2
- Python dependencies listed in the repository
- NVIDIA CUDA/PyTorch environment only when running the semantic-segmentation RGB-D pipeline

### Physical-system workflow

In addition to the software requirements above:

- Piper manipulator and gripper
- Orbbec DaBai DC1 RGB-D camera
- verified hand-eye calibration and rig-specific runtime assets

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
