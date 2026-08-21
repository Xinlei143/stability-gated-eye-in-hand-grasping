# Gazebo benchmark contract (Stage 4/5)

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
six arm joints, gripper, method state, readiness and observation ages.  TCP is
computed from `base_link -> link6` TF plus the configured local `+Z` offset.

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
