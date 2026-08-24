import json
import unittest

from foam_grasp.benchmark_events import make_event
from foam_grasp_sim.metrics_logger_node import _joint_state_fields
from foam_grasp_sim.metrics_model import MetricsAccumulator


class BenchmarkToolsTest(unittest.TestCase):
    def test_joint_state_fields_match_states_csv_schema(self):
        fields = _joint_state_fields({"joint7": 0.02, "joint8": -0.02})
        self.assertEqual(fields, {"gripper": 0.02, "gripper8": -0.02})

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

    def test_plan_only_trial_finished_is_not_task_success(self):
        metrics = MetricsAccumulator()
        metrics.record_event(json.loads(make_event("PLAN_SUCCEEDED", sim_time_ns=1)))
        metrics.record_event(json.loads(make_event(
            "TRIAL_FINISHED", sim_time_ns=2, details={"execution_mode": "plan_only"}
        )))
        result = metrics.finalize()
        self.assertEqual(result["trial_status"], "finished")
        self.assertTrue(result["trial_success"])
        self.assertFalse(result["task_success"])

    def test_finished_event_without_lifted_target_is_not_physical_success(self):
        metrics = MetricsAccumulator()
        metrics.record_event({"event": "TARGET_OBSERVED", "sim_time_ns": 0})
        metrics.record_event({"event": "GRIPPER_CLOSED", "sim_time_ns": 1_000_000_000})
        metrics.record_event({"event": "LIFT_STARTED", "sim_time_ns": 2_000_000_000})
        metrics.record_event({"event": "TASK_FINISHED", "sim_time_ns": 3_000_000_000})
        metrics.record_event({"event": "TRIAL_FINISHED", "sim_time_ns": 3_000_000_000})
        for time_ns in range(1_000_000_000, 4_000_000_001, 100_000_000):
            metrics.record_state({
                "sim_time_ns": time_ns,
                "target_ground_truth_x": 0.4,
                "target_ground_truth_y": 0.0,
                "target_ground_truth_z": 0.026,
                "tcp_x": 0.4,
                "tcp_y": 0.0,
                "tcp_z": 0.081,
            })

        result = metrics.finalize()

        self.assertFalse(result["physical_grasp_success"])
        self.assertFalse(result["task_success"])

    def test_execution_finished_without_physical_lift_is_not_task_success(self):
        metrics = MetricsAccumulator()
        metrics.record_event({"event": "GRIPPER_CLOSED", "sim_time_ns": 1_000_000_000})
        metrics.record_event({"event": "LIFT_STARTED", "sim_time_ns": 2_000_000_000})
        metrics.record_event({"event": "EXECUTION_FINISHED", "sim_time_ns": 3_000_000_000})
        metrics.record_event({"event": "TRIAL_FINISHED", "sim_time_ns": 3_000_000_000,
                              "details": {"execution_mode": "execute"}})
        for time_ns in range(1_000_000_000, 4_000_000_001, 100_000_000):
            metrics.record_state({
                "sim_time_ns": time_ns,
                "target_ground_truth_x": 0.4,
                "target_ground_truth_y": 0.0,
                "target_ground_truth_z": 0.026,
                "tcp_x": 0.4,
                "tcp_y": 0.0,
                "tcp_z": 0.081,
            })
        result = metrics.finalize()
        self.assertFalse(result["physical_grasp_success"])
        self.assertFalse(result["task_success"])

    def test_finished_event_with_lifted_target_is_physical_success(self):
        metrics = MetricsAccumulator()
        metrics.record_event({"event": "TARGET_OBSERVED", "sim_time_ns": 0})
        metrics.record_event({"event": "GRIPPER_CLOSED", "sim_time_ns": 1_000_000_000})
        metrics.record_event({"event": "LIFT_STARTED", "sim_time_ns": 2_000_000_000})
        metrics.record_event({"event": "TASK_FINISHED", "sim_time_ns": 3_000_000_000})
        metrics.record_event({"event": "TRIAL_FINISHED", "sim_time_ns": 3_000_000_000})
        for time_ns in range(1_000_000_000, 4_000_000_001, 100_000_000):
            lifted = time_ns >= 2_500_000_000
            z = 0.081 if lifted else 0.026
            metrics.record_state({
                "sim_time_ns": time_ns,
                "target_ground_truth_x": 0.4,
                "target_ground_truth_y": 0.0,
                "target_ground_truth_z": z,
                "tcp_x": 0.4,
                "tcp_y": 0.0,
                "tcp_z": z,
            })

        result = metrics.finalize()

        self.assertTrue(result["physical_grasp_success"])
        self.assertTrue(result["task_success"])

    def test_failed_trial_has_failure_status(self):
        metrics = MetricsAccumulator()
        metrics.record_event(json.loads(make_event(
            "TRIAL_FAILED", sim_time_ns=2, details={"reason": "timeout"}
        )))
        result = metrics.finalize()
        self.assertEqual(result["trial_status"], "failed")
        self.assertFalse(result["trial_success"])

if __name__ == "__main__":
    unittest.main()
