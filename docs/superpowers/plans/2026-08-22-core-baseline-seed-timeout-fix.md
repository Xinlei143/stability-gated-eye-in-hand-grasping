# Core Baseline Seed and Timeout Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make each benchmark grasp-sequence terminal event carry the suite trial seed and set the formal Core Baseline per-trial timeout to 90 seconds.

**Architecture:** Preserve the existing sequential runner and paired trial matrix. Pass the launch-level `seed` into the `object_grasp_sequence` node alongside the existing execution parameters, then validate the wiring with a source-level regression test and validate the formal suite expansion without starting Gazebo.

**Tech Stack:** ROS 2 Humble launch Python, Python `unittest`, YAML benchmark suites, `colcon`.

---

### Task 1: Stop the invalid campaign and preserve evidence

**Files:**
- Read/retain: `results/core_baseline_formal-20260822T135213Z-71d5afc6/campaign.json`
- Read/retain: `results/core_baseline_formal-20260822T135213Z-71d5afc6/trials.csv`

- [ ] Stop only the known invalid runner process group and verify no Gazebo child remains.
- [ ] Do not delete the campaign, logs, or partial artifacts.

### Task 2: Add the failing regression test

**Files:**
- Modify: `workspaces/app_ws/src/foam_grasp_sim/test/test_scene_assets.py`

- [ ] Add one test that requires `sequence_parameters` to contain the launch-level seed parameter before the grasp-sequence nodes are created:

```python
    def test_grasp_sequence_receives_trial_seed(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn(
            'sequence_parameters["seed"] = _parameter("seed", int)',
            bringup,
        )
```

- [ ] Run only this test and confirm it fails because the production launch file does not yet contain the seed assignment.

### Task 3: Implement the minimal fix and formal timeout change

**Files:**
- Modify: `workspaces/app_ws/src/foam_grasp_sim/launch/sim_bringup.launch.py`
- Modify: `workspaces/app_ws/src/foam_grasp_sim/config/benchmark_suites/core_baseline_formal.yaml`

- [ ] Immediately after `sequence_parameters = dict(execution)`, add:

```python
    sequence_parameters["seed"] = _parameter("seed", int)
```

- [ ] Change only the formal suite default from `timeout_s: 300.0` to `timeout_s: 90.0`.

### Task 4: Verify code, build, and suite expansion

**Files:**
- Validate: `workspaces/app_ws/src/foam_grasp_sim/test/test_scene_assets.py`
- Validate: `workspaces/app_ws/src/foam_grasp_sim/config/benchmark_suites/core_baseline_formal.yaml`

- [ ] Run the focused regression test and the benchmark-suite tests; all must pass.
- [ ] Build the application workspace so the installed launch file contains the fix.
- [ ] Run the formal suite dry-run with writable `ROS_LOG_DIR` and verify 120 launch commands, `timeout_s:=90.0` in `condition_json`, and seed-specific sequence parameters in the installed launch path.

### Task 5: Start the corrected formal campaign

**Files:**
- Create: a new timestamped campaign under `results/`

- [ ] Start the corrected formal campaign with `ROS_LOG_DIR` under `/tmp` and the same YAML path.
- [ ] Confirm the first non-42 trial emits a terminal event with its own seed and is not held for the 300-second timeout.
- [ ] Keep the runner sequential and retain all logs/artifacts.
