import csv
import json
import tempfile
import unittest
from pathlib import Path

from analysis.benchmark_io import (
    bootstrap_mean_ci,
    load_campaign_runs,
    paired_differences,
)
from analysis.summarize import summarize_campaign


def _write_campaign(root: Path):
    (root / "runs" / "run-a").mkdir(parents=True)
    (root / "runs" / "run-b").mkdir(parents=True)
    for name, content in {
        "metadata.json": "{}",
        "states.csv": "sim_time_ns\n1\n",
        "events.csv": "event\nTRIAL_FINISHED\n",
        "metrics.json": "bad",
    }.items():
        (root / "runs" / "run-b" / name).write_text(content, encoding="utf-8")
    (root / "campaign.json").write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
    fields = ["run_id", "pair_id", "method", "trajectory", "seed", "status", "artifacts_complete"]
    with (root / "trials.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow({"run_id": "run-a", "pair_id": "pair-1", "method": "gated", "trajectory": "static", "seed": "42", "status": "finished", "artifacts_complete": "true"})
        writer.writerow({"run_id": "run-b", "pair_id": "pair-2", "method": "snapshot", "trajectory": "static", "seed": "42", "status": "finished", "artifacts_complete": "true"})
    (root / "runs" / "run-a" / "metadata.json").write_text(json.dumps({"run_id": "run-a", "pair_id": "pair-1", "condition_json": json.dumps({"latency_ms": 10})}), encoding="utf-8")
    (root / "runs" / "run-a" / "states.csv").write_text("sim_time_ns\n1\n", encoding="utf-8")
    (root / "runs" / "run-a" / "events.csv").write_text("event\nTRIAL_FINISHED\n", encoding="utf-8")
    (root / "runs" / "run-a" / "metrics.json").write_text(json.dumps({"trial_success": True, "task_success": False, "outcome": "task_failure", "failure_stage": "planning", "failure_reason": "no candidate", "tracking_rms_error_m": 0.01, "time_to_ready_s": 2.0}), encoding="utf-8")


class BenchmarkIoTest(unittest.TestCase):
    def test_malformed_metrics_are_excluded_with_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_campaign(root)
            records, exclusions = load_campaign_runs(root)
            self.assertEqual([record.run_id for record in records], ["run-a"])
            self.assertEqual(records[0].condition["latency_ms"], 10)
            self.assertEqual(exclusions[0]["run_id"], "run-b")
            self.assertIn("metrics", exclusions[0]["reason"])

    def test_bootstrap_ci_is_deterministic(self):
        first = bootstrap_mean_ci([1.0, 2.0, 3.0], seed=2026, samples=100)
        second = bootstrap_mean_ci([1.0, 2.0, 3.0], seed=2026, samples=100)
        self.assertEqual(first, second)
        self.assertEqual(first[0], 2.0)

    def test_paired_differences_are_method_order_independent(self):
        records = [
            {"pair_id": "p", "method": "snapshot", "tracking_rms_error_m": 0.5},
            {"pair_id": "p", "method": "gated", "tracking_rms_error_m": 0.2},
        ]
        differences = paired_differences(records, metric="tracking_rms_error_m")
        self.assertEqual(differences, [{"pair_id": "p", "gated_minus_snapshot": -0.3}])

    def test_summary_writes_to_sibling_without_touching_campaign(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "campaign"
            root.mkdir()
            _write_campaign(root)
            before = (root / "campaign.json").read_bytes()
            output = summarize_campaign(root)
            self.assertEqual(output, root.parent / "campaign-analysis")
            self.assertTrue((output / "run_metrics.csv").is_file())
            self.assertTrue((output / "excluded_runs.csv").is_file())
            self.assertEqual((root / "campaign.json").read_bytes(), before)

    def test_summary_preserves_task_failure_classification(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "campaign"
            root.mkdir()
            _write_campaign(root)
            output = summarize_campaign(root)
            with (output / "run_metrics.csv").open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["outcome"], "task_failure")
            self.assertEqual(rows[0]["failure_stage"], "planning")
            self.assertEqual(rows[0]["failure_reason"], "no candidate")


if __name__ == "__main__":
    unittest.main()
