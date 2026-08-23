# Simulation Readiness and Trial Semantics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gate target/pipeline startup on deterministic controller and action readiness, and preserve the distinction between infrastructure failure and a completed trial whose physical task failed.

**Architecture:** Add a wall-clock `simulation_readiness_node` that observes `/controller_manager/list_controllers`, `/joint_states`, and the three FollowJointTrajectory action servers, then exits 0 only after all required conditions are simultaneously available. `sim_bringup.launch.py` starts Piper and MoveIt first, launches this node, and dynamically starts the target scene only after a successful exit; any nonzero readiness exit shuts the launch down. Refactor runner artifact handling so an execute `TRIAL_FINISHED` is `finished/true`, with `metrics.json.task_success` as the authoritative task result, while missing artifacts and infrastructure termination remain `failed/false`.

**Tech Stack:** ROS 2 Humble Python (`rclpy`, `rclpy.action`, `controller_manager_msgs`, `control_msgs`, `sensor_msgs`), ROS 2 launch event handlers, Python `unittest`/`pytest`, colcon.

---

### Task 1: Define readiness and runner regression tests

**Files:**
- Create: `workspaces/app_ws/src/foam_grasp_sim/test/test_simulation_readiness.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/test/test_experiment_runner.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/test/test_scene_assets.py`

- [ ] **Step 1: Write failing readiness helper tests**

  Test that a readiness snapshot is incomplete when any controller is not active, when `/joint_states` lacks one of `joint1` through `joint8`, or when any action server is not ready; test that the complete snapshot has no missing conditions and that timeout formatting names every missing condition.

- [ ] **Step 2: Run the readiness tests and confirm the expected missing-symbol failure**

  Run:
  ```bash
  source scripts/source_env.sh
  export PYTHONPATH="$PWD/workspaces/app_ws/src/foam_grasp_sim:$PWD/workspaces/app_ws/src/foam_grasp"
  /usr/bin/python3 -m pytest -q workspaces/app_ws/src/foam_grasp_sim/test/test_simulation_readiness.py
  ```
  Expected: collection or assertion failure because `simulation_readiness.py` does not yet exist.

- [ ] **Step 3: Add the three required runner semantic tests**

  Add tests for `TRIAL_FINISHED` with execute metrics `task_success=false` and `task_success=true`, asserting respectively `finished/true/false` and `finished/true/true`; add a missing `metrics.json` test asserting `failed/false/false`. Keep `physical_grasp_success` false in the first case to prove it does not turn a completed trial into infrastructure failure.

- [ ] **Step 4: Add the launch ordering regression test**

  Assert that `sim_bringup.launch.py` contains the readiness executable, registers an `OnProcessExit` handler for it, and does not start `target_spawns` or the grasp sequence as unconditional top-level actions. Assert the failure path contains `Shutdown` and the source has no new fixed readiness delay timer.

- [ ] **Step 5: Run the new runner and launch tests to observe the old semantic failures**

  Run the focused test files and confirm the old execute status logic reports `failed/false/false` for a physical task failure and the old launch starts the scene before readiness.

### Task 2: Implement the wall-clock readiness process

**Files:**
- Create: `workspaces/app_ws/src/foam_grasp_sim/foam_grasp_sim/simulation_readiness.py`
- Create: `workspaces/app_ws/src/foam_grasp_sim/foam_grasp_sim/simulation_readiness_node.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/setup.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/package.xml`

- [ ] **Step 1: Implement pure readiness state evaluation**

  Define constants for the required controllers, joints, and action names. Implement a small state object/helper that records service availability, active controller names, observed joint names, and action-server readiness, returning a deterministic list of missing condition strings.

- [ ] **Step 2: Implement the ROS node with wall-clock timeout**

  Subscribe to `/joint_states`, create a `ListControllers` client for `/controller_manager/list_controllers`, create `ActionClient` instances for `/arm_controller/follow_joint_trajectory`, `/gripper_controller/follow_joint_trajectory`, and `/gripper8_controller/follow_joint_trajectory`, and poll with `time.monotonic()` until all conditions are ready. Log missing conditions on timeout and return nonzero; return zero immediately after the complete condition is observed.

- [ ] **Step 3: Add the executable wrapper**

  Implement `simulation_readiness_node.py:main()` to parse `--timeout-s`, initialize rclpy, spin the readiness node, destroy it, shut down rclpy, and return its result. Register `simulation_readiness = foam_grasp_sim.simulation_readiness_node:main` in `setup.py`.

- [ ] **Step 4: Add the ROS dependency**

  Add `<exec_depend>controller_manager_msgs</exec_depend>` to `package.xml`; retain existing `control_msgs`, `sensor_msgs`, and `rclpy` dependencies.

- [ ] **Step 5: Run focused readiness tests and then the complete Python test suite**

  The readiness tests must pass, followed by the repository's existing `foam_grasp_sim` and `foam_grasp` tests.

### Task 3: Gate sim_bringup scene and pipeline startup

**Files:**
- Modify: `workspaces/app_ws/src/foam_grasp_sim/launch/sim_bringup.launch.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/test/test_scene_assets.py`

- [ ] **Step 1: Start readiness after the existing Piper and MoveIt includes**

  Add a `Node(package="foam_grasp_sim", executable="simulation_readiness", arguments=["--timeout-s", LaunchConfiguration("simulation_readiness_timeout_s")])` and a launch argument with default `30.0`.

- [ ] **Step 2: Move scene startup behind readiness success**

  Replace the unconditional target `TimerAction` with an `OnProcessExit` callback for readiness: on return code 0 return the selected target spawn actions; on any other return code return `Shutdown` with the missing-condition exit reason. Keep the existing target-spawn exit handler as the only path that starts motion, perception, logging, and grasp sequence nodes.

- [ ] **Step 3: Remove fixed pipeline-delay timers**

  Start the grasp pipeline actions directly after the successful target spawn exit; do not add or retain a fixed timer as a substitute for readiness.

- [ ] **Step 4: Run launch-source tests and a dry launch parse**

  Verify the launch tests pass and use `ros2 launch ... --show-args`/the project launch parser to ensure the new argument and executable resolve.

### Task 4: Correct execute trial/task semantics

**Files:**
- Modify: `workspaces/app_ws/src/foam_grasp_sim/foam_grasp_sim/experiment_runner.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/test/test_experiment_runner.py`

- [ ] **Step 1: Make `TRIAL_FINISHED` mean a completed trial**

  Change `status_for_terminal()` so every `TRIAL_FINISHED` returns `status="finished"` and `trial_success=true`; plan-only remains `task_success=false`, while execute task success is filled from `metrics.json`.

- [ ] **Step 2: Make execute metrics authoritative for task success**

  Change `status_for_artifacts()` so an execute run with a readable metrics file returns `finished/true/<bool(metrics["task_success"])>`, without consulting `physical_grasp_success` to change status. A missing/unreadable metrics file returns `failed/false/false` with an infrastructure/artifact error. Preserve plan-only handling and contact artifact checks.

- [ ] **Step 3: Preserve infrastructure failures as failed**

  Keep process timeout, process exit without a terminal event, `TRIAL_FAILED`, and incomplete artifact cases as `failed/false/false`; do not reinterpret a normal terminal event with `task_success=false` as a crash.

- [ ] **Step 4: Run focused runner tests and the full suite**

  Confirm all four requested semantic cases and all existing tests pass.

### Task 5: Build and run one bounded execute smoke

**Files:**
- No source changes expected beyond the files above.

- [ ] **Step 1: Run the complete test suite**

  ```bash
  source scripts/source_env.sh
  export PYTHONPATH="$PWD/workspaces/app_ws/src/foam_grasp_sim:$PWD/workspaces/app_ws/src/foam_grasp:${PYTHONPATH}"
  /usr/bin/python3 -m pytest -q workspaces/app_ws/src/foam_grasp_sim/test workspaces/app_ws/src/foam_grasp/test
  ```

- [ ] **Step 2: Build the ROS workspace**

  ```bash
  bash scripts/build_all.sh
  ```

- [ ] **Step 3: Run exactly one `gazebo_grasp_fix` + Kp30 + seed42 execute smoke**

  Use the existing formal launch/runner entry point with `grasp_stabilization_mode:=gazebo_grasp_fix`, `seed:=42`, and the existing PID configuration that provides Kp=30. Use a unique temporary results directory and a bounded timeout; do not run the 120-trial campaign. Confirm the readiness node reports success before target spawn and inspect `metrics.json` and terminal status.

- [ ] **Step 4: Report validation boundaries**

  Report test/build/smoke outcomes separately. If Gazebo still fails after readiness, classify the result as infrastructure failure and preserve its log/artifacts; do not alter grasp-fix parameters or claim task success from launch completion.

