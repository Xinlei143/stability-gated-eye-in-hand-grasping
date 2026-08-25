import csv
import json
from pathlib import Path

import pytest

from analysis.final_experiment_summary import (
    _stats,
    audit_trial,
    compute_tracking_rms,
    load_canonical_trials,
    planning_semantics,
)


def _write_run(root: Path, run_id: str, *, task_success=False, physical=False):
    run = root / "runs" / run_id
    run.mkdir(parents=True)
    (run / "metadata.json").write_text(json.dumps({"run_id": run_id}))
    events = [
        {"event": "TARGET_OBSERVED", "sim_time_ns": "100"},
        {"event": "PLAN_SUCCEEDED", "sim_time_ns": "200"},
        {
            "event": "TRIAL_FINISHED",
            "sim_time_ns": "300",
            "details": json.dumps({"task_success": task_success, "outcome": "success" if task_success else "task_failure"}),
        },
    ]
    with (run / "events.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["event", "sim_time_ns", "details"])
        writer.writeheader()
        writer.writerows(events)
    (run / "states.csv").write_text("sim_time_ns,target_ground_truth_x,target_ground_truth_y,target_ground_truth_z,tcp_x,tcp_y,tcp_z\n100,0,0,0,1,0,0\n200,0,0,0,0,2,0\n")
    (run / "metrics.json").write_text(json.dumps({
        "task_success": task_success,
        "physical_grasp_success": physical,
        "planning_success": True,
        "trial_status": "finished",
        "outcome": "success" if task_success else "task_failure",
        "failure_stage": "planning" if not task_success else "",
        "failure_reason": "fresh, low-drift PREGRASP commitment failed" if not task_success else "",
    }))
    return run


def test_load_canonical_trials_rejects_duplicate_condition_seed(tmp_path):
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    run = _write_run(campaign, "r1")
    with (campaign / "trials.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["run_id", "method", "scenario", "seed", "status", "attempt", "result_path"])
        writer.writeheader()
        writer.writerow({"run_id": "r1", "method": "tracking", "scenario": "static", "seed": 42, "status": "finished", "attempt": 1, "result_path": str(run)})
        writer.writerow({"run_id": "r2", "method": "tracking", "scenario": "static", "seed": 42, "status": "finished", "attempt": 2, "result_path": str(run)})
    with pytest.raises(ValueError, match="duplicate canonical condition"):
        load_canonical_trials(campaign)


def test_tracking_rms_uses_tcp_to_ground_truth_between_observation_and_grasp():
    states = [
        {"sim_time_ns": "50", "target_ground_truth_x": "0", "target_ground_truth_y": "0", "target_ground_truth_z": "0", "tcp_x": "10", "tcp_y": "0", "tcp_z": "0"},
        {"sim_time_ns": "100", "target_ground_truth_x": "0", "target_ground_truth_y": "0", "target_ground_truth_z": "0", "tcp_x": "1", "tcp_y": "0", "tcp_z": "0"},
        {"sim_time_ns": "200", "target_ground_truth_x": "0", "target_ground_truth_y": "0", "target_ground_truth_z": "0", "tcp_x": "0", "tcp_y": "2", "tcp_z": "0"},
        {"sim_time_ns": "300", "target_ground_truth_x": "0", "target_ground_truth_y": "0", "target_ground_truth_z": "0", "tcp_x": "100", "tcp_y": "0", "tcp_z": "0"},
    ]
    events = [
        {"event": "TARGET_OBSERVED", "sim_time_ns": "100"},
        {"event": "GRASP_STARTED", "sim_time_ns": "200"},
    ]
    assert compute_tracking_rms(states, events) == pytest.approx((2.5) ** 0.5)


def test_planning_success_is_distinct_from_terminal_planning_failure():
    metrics = {"planning_success": True, "failure_stage": "planning", "failure_reason": "fresh, low-drift PREGRASP commitment failed"}
    semantics = planning_semantics(metrics)
    assert semantics["initial_plan_succeeded"] is True
    assert semantics["terminal_task_phase"] == "planning"
    assert semantics["pregrasp_commitment_failure"] is True


def test_audit_trial_reports_terminal_metrics_mismatch(tmp_path):
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    run = _write_run(campaign, "r1", task_success=False, physical=False)
    metrics_path = run / "metrics.json"
    metrics = json.loads(metrics_path.read_text())
    metrics["task_success"] = True
    metrics_path.write_text(json.dumps(metrics))
    row = {
        "run_id": "r1",
        "pair_id": "p1",
        "method": "tracking",
        "scenario": "static",
        "seed": "42",
        "attempt": "1",
        "status": "finished",
        "trial_success": "true",
        "_resolved_result_path": str(run),
    }
    audited = audit_trial(row, "controlled")
    assert "terminal_vs_metrics_task_success" in audited["inconsistencies"]


def test_stats_preserves_missing_values():
    summary = _stats([1.0, None, 3.0, None])
    assert summary["n"] == 2
    assert summary["missing_n"] == 2
    assert summary["median"] == 2.0
