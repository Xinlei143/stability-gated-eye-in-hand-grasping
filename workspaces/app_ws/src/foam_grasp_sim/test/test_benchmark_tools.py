import json
import unittest

from foam_grasp.benchmark_events import make_event
from foam_grasp_sim.metrics_model import MetricsAccumulator


class BenchmarkToolsTest(unittest.TestCase):
    def test_event_schema_is_stable_and_json_serializable(self):
        payload = make_event(
            "READY",
            sim_time_ns=123,
            method="gated",
            scenario="static",
            seed=42,
            details={"samples": 25},
        )
        event = json.loads(payload)
        self.assertEqual(event["schema_version"], 1)
        self.assertEqual(event["event"], "READY")
        self.assertEqual(event["sim_time_ns"], 123)
        self.assertEqual(event["details"], {"samples": 25})

    def test_metrics_model_computes_ready_tracking_and_success_fields(self):
        metrics = MetricsAccumulator()
        metrics.record_state(
            {
                "sim_time_ns": 0,
                "target_ground_truth_x": 0.400,
                "target_ground_truth_y": 0.0,
                "target_ground_truth_z": 0.03,
                "target_selected_x": 0.402,
                "target_selected_y": 0.0,
                "target_selected_z": 0.03,
                "tcp_x": 0.500,
                "tcp_y": 0.0,
                "tcp_z": 0.03,
                "valid": True,
            }
        )
        metrics.record_event(
            json.loads(
                make_event(
                    "TARGET_OBSERVED",
                    sim_time_ns=0,
                    method="tracking",
                    scenario="static",
                    seed=42,
                )
            )
        )
        metrics.record_state(
            {
                "sim_time_ns": 1_000_000_000,
                "target_ground_truth_x": 0.400,
                "target_ground_truth_y": 0.0,
                "target_ground_truth_z": 0.03,
                "target_selected_x": 0.401,
                "target_selected_y": 0.0,
                "target_selected_z": 0.03,
                "tcp_x": 0.500,
                "tcp_y": 0.0,
                "tcp_z": 0.03,
                "valid": True,
            }
        )
        metrics.record_event(
            json.loads(
                make_event(
                    "READY",
                    sim_time_ns=1_000_000_000,
                    method="tracking",
                    scenario="static",
                    seed=42,
                )
            )
        )
        metrics.record_event(
            json.loads(
                make_event(
                    "PLAN_SUCCEEDED",
                    sim_time_ns=2_000_000_000,
                    method="tracking",
                    scenario="static",
                    seed=42,
                )
            )
        )
        result = metrics.finalize()
        self.assertAlmostEqual(result["time_to_ready_s"], 1.0)
        self.assertAlmostEqual(result["target_error_at_ready_m"], 0.001)
        self.assertAlmostEqual(result["tracking_rms_error_m"], 0.1)
        self.assertTrue(result["planning_success"])


if __name__ == "__main__":
    unittest.main()
