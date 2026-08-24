import json
import unittest

from foam_grasp.benchmark_events import make_event
from foam_grasp_sim.metrics_model import MetricsAccumulator


class MetricsOutcomeTest(unittest.TestCase):
    def test_expected_task_failure_has_finished_trial_status(self):
        metrics = MetricsAccumulator()
        metrics.record_event(json.loads(make_event(
            "PLAN_FAILED",
            sim_time_ns=1,
            details={"failure_stage": "planning", "reason": "no candidate"},
        )))
        metrics.record_event(json.loads(make_event(
            "TRIAL_FINISHED",
            sim_time_ns=2,
            details={
                "execution_mode": "execute",
                "outcome": "task_failure",
                "failure_stage": "planning",
                "task_success": False,
            },
        )))
        result = metrics.finalize()
        self.assertEqual(result["trial_status"], "finished")
        self.assertTrue(result["trial_success"])
        self.assertFalse(result["task_success"])
        self.assertEqual(result["outcome"], "task_failure")
        self.assertEqual(result["failure_stage"], "planning")

    def test_physical_failure_overrides_success_terminal_label(self):
        metrics = MetricsAccumulator()
        metrics.record_event({"event": "EXECUTION_FINISHED", "sim_time_ns": 1})
        metrics.record_event(json.loads(make_event(
            "TRIAL_FINISHED",
            sim_time_ns=2,
            details={"execution_mode": "execute", "task_success": True, "outcome": "success"},
        )))
        result = metrics.finalize()
        self.assertFalse(result["task_success"])
        self.assertEqual(result["outcome"], "task_failure")


if __name__ == "__main__":
    unittest.main()
