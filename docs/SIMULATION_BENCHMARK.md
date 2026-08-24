# Gazebo benchmark and RGB-D contract (Stage 4/5/6/7)

`sim_bringup.launch.py` is the single entry point for the controlled Gazebo
trial.  The method layer is now the only target gate in simulation:

- `snapshot` selects the first valid observation and keeps `READY` even if the
  stream later times out.
- `gated` requires the configured duration, spread and sample count.
- `tracking` continuously publishes the latest observation.  The sequence
  follows live PREGRASP with bounded receding-horizon plans and commits only
  after the observation is fresh, the PREGRASP drift is at most 5 mm, and the
  joint error is at most 0.03 rad.

The method node exposes `/foam_grasp/commit_method_target` and
`/foam_grasp/reset_method` (`std_srvs/Trigger`).  Live target output is
`/foam_grasp/<class>_method_point_base`; the committed compatibility output is
`/foam_grasp/target_point_base_latched` and the corresponding class topic is
`/foam_grasp/latched_target_class`.  The legacy `target_latch_node` remains
available for real perception workflows but is not started by the simulation
bringup, so the target is not gated twice.

## Reproducible runs

The defaults preserve the static, ideal Stage-3 scene.  A moving tracking run
can be started with:

```bash
ros2 launch foam_grasp_sim sim_bringup.launch.py \
  method:=tracking trajectory:=constant_velocity scenario:=constant_velocity \
  record_benchmark:=true results_root:=results run_id:=tracking-static-seed42
```

`record_benchmark` is off by default.  When enabled, the logger creates a new
directory (it never overwrites an existing run):

```text
results/<run_id>/
  metadata.json
  states.csv
  events.csv
  metrics.json
```

`seed`, `scenario`, `method`, target class, frame names and the 0.1358 m local
`+Z` tool offset are recorded in `metadata.json`.  `states.csv` contains the
ground-truth, observed, selected and committed target positions, TCP position,
six arm joints, gripper, method state, readiness and observation ages.  RGB-D
runs additionally record the fresh-observation flag and the latest depth-fusion
diagnostic (mask/component/valid-depth counts, synchronization delta, camera
frame point and output rate). TCP is computed from `base_link -> link6` TF plus
the configured local `+Z` offset.

`events.csv` stores edge-triggered JSON events from the common
`/foam_grasp/benchmark_event` topic.  Every event has:

```json
{
  "schema_version": 1,
  "event": "READY",
  "sim_time_ns": 123,
  "method": "gated",
  "scenario": "static",
  "seed": 42,
  "details": {}
}
```

The logger derives RMS tracking error `|p_ground_truth - p_TCP|` (from first
observation to `GRASP_STARTED`; plan-only runs use the selected-target proxy),
error at `READY`, grasp-initiation error, time-to-ready, gate reset count,
planning/task success and the `false_ready` flag.  `false_ready`
is raised when the ground-truth target moved more than 2 mm or exceeded 3 mm/s
in the 0.5 s preceding `READY`.

For a plan-only validation use `execute_motion:=false`; tracking validates the
latest PREGRASP plan and still commits the selected target, but it does not
claim mechanical tracking.  Physical or simulated motion requires
`execute_motion:=true` and the existing explicit execution safeguards.

## Automated reproducible campaigns (Stage 6)

`sim_bringup.launch.py` remains the only single-trial Gazebo entry point. The
outer runner expands a validated YAML suite and starts that launch once per
trial; it does not duplicate the simulation pipeline.

Packaged suites are in `config/benchmark_suites/`:

```text
smoke, baseline_comparison, latency_sweep, noise_sweep,
dropout_sweep, gate_ablation
```

Run a plan-only smoke campaign after sourcing the ROS workspace overlay:

```bash
ros2 run foam_grasp_sim run_sim_benchmark smoke \
  --results-root results --max-trials 1
```

Inspect the exact resolved command without starting ROS or Gazebo:

```bash
ros2 run foam_grasp_sim run_sim_benchmark smoke --dry-run --max-trials 1
```

Each suite uses this schema. `methods`, `trajectories`, and `seeds` are
explicit expansion dimensions. Every `sweeps` item has exactly one parameter;
items are concatenated as independent alternatives, never multiplied. A
multi-axis mapping or nested Cartesian list is rejected.

```yaml
schema_version: 1
name: example
defaults: {target_model: cube, execute_motion: false}
methods: [snapshot, tracking, gated]
trajectories: [static]
seeds: [42]
sweeps:
  - parameter: latency_ms
    values: [0.0, 50.0, 100.0]
```

Resolved conditions are canonicalized as sorted-key JSON. `config_hash` is its
SHA-256. `pair_id` hashes the same condition with `method` removed, so paired
methods sharing condition and seed use the same ID. `run_id` is a deterministic
readable prefix plus the config-hash prefix. Paired methods must have identical
motion, trajectory, seed, target, execution mode, and perception/measurement
disturbances.

Campaign output is:

```text
results/<campaign_id>/
  campaign.json
  trials.csv
  logs/<run_id>.log
  runs/<run_id>/
    metadata.json
    states.csv
    events.csv
    metrics.json
```

`campaign.json` and `trials.csv` are updated atomically. Failed, timed-out,
interrupted, and incomplete trials retain logs and partial result files.
`--resume --campaign-id <id>` skips only finished trials with all four run
artifacts. `--rerun-failed --campaign-id <id>` uses deterministic
`__attempt-002`-style names and never overwrites an earlier attempt.
`--max-trials` truncates the already deterministic queue.

Terminal events are distinct from task events:

- `TRIAL_FINISHED` means the program trial completed with a valid result. In
  plan-only mode it follows `PLAN_SUCCEEDED` while `task_success` remains
  false. Expected readiness, planning, grasp, and safety refusals also emit
  `TRIAL_FINISHED` with `outcome=task_failure`, so they remain in the
  statistical denominator.
- `TRIAL_FAILED` is reserved for infrastructure failures such as missing
  services, process termination, timeout, interruption, or incomplete/corrupt
  artifacts. These trials are excluded and may be rerun.
- `TASK_FINISHED` still means the simulated/real mechanical grasp and lift
  succeeded; it is never synthesized from plan-only success.

Each launch is started in its own process group. On terminal event, timeout, or
interruption the runner signals only that group in order: SIGINT, SIGTERM, then
SIGKILL after bounded grace periods. It never calls global `pkill ros2`,
`pkill gazebo`, or kills processes it did not start.

Stage 6 stops at reproducible orchestration and raw run artifacts. RGB-D
integration, offline analysis, plotting, and the complete paper benchmark
matrix remain outside this workflow.

## Stage 7 simulated eye-in-hand RGB-D pipeline

`full_pipeline.launch.py` composes the existing trial with the real RGB-D
nodes. It always includes `sim_bringup.launch.py`; it does not clone the
Gazebo scene, target-motion node, method policy, or grasp sequence.

```bash
ros2 launch foam_grasp_sim full_pipeline.launch.py \
  checkpoint:=/absolute/path/to/best_model.pth \
  require_cuda:=true target_model:=cube method:=gated \
  trajectory:=static execute_motion:=false use_rviz:=false
```

The eye-in-hand Xacro is selected by `robot_xacro` and adds a fixed
`camera_link` near the link6 tool axis plus color/depth optical frames and a
640x480, 30 Hz Gazebo depth sensor matched to the project-validated DaBai DC1
profile. The plugin publishes
`/camera/color/image_raw`, `/camera/depth/image_raw`, and
`/camera/depth/camera_info`. `foam_camera_to_base_node` uses
`transform_source:=tf` in this launch and looks up the stamped source frame in
the robot TF tree. The real `system.launch.py` remains calibration mode.

The fixed `link6 -> camera_color_optical_frame` transform is derived from the
repository's eye-in-hand hand-eye asset; the color/depth optical frames remain
coincident with that calibrated optical center. Before claiming end-to-end success, verify RGB,
depth, CameraInfo, TF, mask, camera-frame points, base-frame points, method
`READY`, `PLAN_SUCCEEDED`, and the terminal event in that order. A plan-only
trial ends with `TRIAL_FINISHED` and `task_success=false`; only an executed
grasp may emit `TASK_FINISHED`.

The depth-fusion diagnostic topic is `/foam_grasp/depth_fusion_diagnostics`.
It emits one JSON record per depth frame with explicit failure reasons such as
`component_too_small`, `insufficient_eroded_pixels`, `insufficient_valid_depth`,
`depth_outlier_rejection`, and `stale_mask`. The RGB-D qualification uses a
freshness threshold of 0.20 s for `observation_fresh`; this is separate from
the 0.15 s mask/depth synchronization limit.

For raw localization qualification, compare the observed base-frame point
directly with ground truth:

```bash
PYTHONPATH=. python3 analysis/semantic_perception_quality.py \
  results/<run_id>/states.csv
```

This reports valid fresh-observation fraction, per-axis absolute errors,
planar error, and 3D error. It deliberately does not use method-selected or
latched target metrics as perception accuracy.

## Offline Stage 7 analysis

Analysis tools read standard campaign artifacts and never rewrite the raw
campaign. By default they write to a sibling directory `<campaign_id>-analysis/`
(override with `--output-dir`):

```bash
PYTHONPATH=. python3 -m analysis.summarize results/smoke-...
PYTHONPATH=. python3 -m analysis.plot_latency_sweep results/latency-...
PYTHONPATH=. python3 -m analysis.plot_noise_sweep results/noise-...
PYTHONPATH=. python3 -m analysis.plot_gate_ablation results/gate-...
PYTHONPATH=. python3 -m analysis.make_result_table results/baseline-...
```

Outputs include `run_metrics.csv`, `group_summary.csv`,
`paired_differences.csv`, `excluded_runs.csv`, deterministic bootstrap
confidence intervals (seed 2026), and PNG/PDF/CSV plot data where relevant.
Malformed metrics and incomplete/failed runs are listed in `excluded_runs.csv`,
never silently treated as successes. Paired results are reported as
`gated - snapshot` and `gated - tracking` by `pair_id`.

Stage 7 does not add RGB-D domain adaptation, retraining, ground-truth
shortcuts, statistical claims, or the complete paper benchmark matrix.

## Gazebo grasp stabilization qualification

The simulator has one explicit stabilization selector:
`grasp_stabilization_mode` is `off`, `gazebo_grasp_fix`, or
`legacy_contact_confirmed`. The formal core baseline compares
snapshot/tracking/gated under `gazebo_grasp_fix`. In that mode
`sim_bringup.launch.py`
selects `piper_eye_in_hand_grasp_fix.xacro` and
`grasp_table_no_attachment.world`; the old model-attachment plugin and the
legacy `/gazebo/attach` service are not started. Selecting the legacy mode is
allowed only for compatibility diagnostics, never together with
`gazebo_grasp_fix`.

The pinned plugin is built locally with:

```bash
bash scripts/setup_gazebo_grasp_plugin.sh
source scripts/source_env.sh
```

The plugin configuration is shared across methods: palm `link6`, gripper links
`link7`/`link8`, force-angle tolerance `100`, update rate `10 Hz`, grip
threshold `2`, maximum grip count `3`, release tolerance `5 mm`, and collision
disabling disabled. These are simulator-level stabilization parameters, not
perception or method-policy parameters. A successful attach or completed
trajectory is not task success; the runner requires readable `metrics.json`
with physical lift and hold checks passing.

Plugin-load smoke test (no target or grasp pipeline is needed):

```bash
source scripts/source_env.sh
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
export GAZEBO_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1
ros2 launch foam_grasp_sim piper_sim.launch.py \
  robot_xacro:="$PWD/workspaces/app_ws/install/foam_grasp_sim/share/foam_grasp_sim/urdf/piper_eye_in_hand_grasp_fix.xacro" \
  world:="$PWD/workspaces/app_ws/install/foam_grasp_sim/share/foam_grasp_sim/worlds/grasp_table_no_attachment.world" \
  gazebo_executable:=gzserver
```

The log must contain `Loading grasp-fix plugin`, the configured link names and
contact subscription, with no `Failed to load plugin` or missing-link error.
