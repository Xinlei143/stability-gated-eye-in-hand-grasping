import unittest

from foam_grasp_sim.control_qualification import (
    summarize_arm_run,
    summarize_loaded_gripper_run,
    summarize_gripper_run,
)


class ControlQualificationTest(unittest.TestCase):
    def test_arm_summary_requires_action_success_error_and_settled_spread(self):
        target = [0.0, 1.76, -1.18, 0.0, 0.73, 0.0]
        samples = [
            [0.001, 1.761, -1.181, 0.001, 0.731, 0.001],
            [0.002, 1.762, -1.182, 0.002, 0.732, 0.002],
        ]

        summary = summarize_arm_run(target, samples, error_code=0)

        self.assertTrue(summary["passed"])
        self.assertLess(summary["max_final_error_rad"], 0.02)
        self.assertLess(summary["settled_max_spread_rad"], 0.01)

    def test_arm_summary_rejects_large_joint6_error_even_with_successful_action(self):
        target = [0.0, 1.76, -1.18, 0.0, 0.73, 0.0]
        samples = [
            [0.0, 1.76, -1.18, 0.0, 0.73, 0.075],
            [0.0, 1.76, -1.18, 0.0, 0.73, 0.075],
        ]

        summary = summarize_arm_run(target, samples, error_code=0)

        self.assertFalse(summary["passed"])
        self.assertGreater(summary["joint6_final_error_rad"], 0.02)

    def test_gripper_summary_requires_symmetry_completion_and_finite_effort(self):
        samples = [
            {"joint7": 0.0200, "joint8": -0.0200, "effort7": 1.0, "effort8": 1.1},
            {"joint7": 0.0201, "joint8": -0.0201, "effort7": 1.2, "effort8": 1.3},
        ]

        summary = summarize_gripper_run(
            samples,
            target_joint7=0.0200,
            target_joint8=-0.0200,
        )

        self.assertTrue(summary["passed"])
        self.assertLessEqual(summary["symmetry_error_mm"], 1.0)
        self.assertTrue(summary["effort_finite"])

    def test_gripper_summary_rejects_asymmetric_feedback(self):
        samples = [
            {"joint7": 0.0200, "joint8": -0.0160, "effort7": 1.0, "effort8": 1.1},
        ]

        summary = summarize_gripper_run(
            samples,
            target_joint7=0.0200,
            target_joint8=-0.0200,
        )

        self.assertFalse(summary["passed"])
        self.assertGreater(summary["symmetry_error_mm"], 1.0)

    def test_loaded_gripper_summary_requires_bilateral_force(self):
        samples = [
            {
                "joint7": 0.0200,
                "joint8": -0.0200,
                "effort7": 1.0,
                "effort8": 1.1,
                "left_force_N": 1.0,
                "right_force_N": 0.9,
                "sim_time_ns": index * 1_000_000_000 // 19,
            }
            for index in range(20)
        ]
        summary = summarize_loaded_gripper_run(
            samples,
            target_joint7=0.0150,
            target_joint8=-0.0150,
            minimum_force_N=0.8,
        )

        self.assertTrue(summary["passed"])
        self.assertGreaterEqual(summary["bilateral_stable_fraction"], 0.8)
        self.assertAlmostEqual(summary["left_median_force_N"], 1.0)
        self.assertAlmostEqual(summary["right_median_force_N"], 0.9)

    def test_loaded_gripper_summary_rejects_single_side_force(self):
        samples = [
            {
                "joint7": 0.0200,
                "joint8": -0.0200,
                "effort7": 1.0,
                "effort8": 1.1,
                "left_force_N": 1.0,
                "right_force_N": 0.0,
                "sim_time_ns": index * 1_000_000_000 // 19,
            }
            for index in range(20)
        ]
        summary = summarize_loaded_gripper_run(
            samples,
            target_joint7=0.0200,
            target_joint8=-0.0200,
            minimum_force_N=0.8,
        )

        self.assertFalse(summary["passed"])
        self.assertIn("missing_bilateral_force", summary["failure_reasons"])

    def test_loaded_gripper_summary_allows_small_transient_below_fraction_threshold(self):
        samples = [
            {
                "joint7": 0.0200,
                "joint8": -0.0200,
                "effort7": 1.0,
                "effort8": 1.1,
                "left_force_N": 0.0 if index == 0 else 1.0,
                "right_force_N": 0.0 if index == 0 else 0.9,
                "sim_time_ns": index * 1_000_000_000 // 19,
            }
            for index in range(20)
        ]
        summary = summarize_loaded_gripper_run(
            samples,
            target_joint7=0.0200,
            target_joint8=-0.0200,
            minimum_force_N=0.8,
        )

        self.assertTrue(summary["passed"])
        self.assertEqual(summary["bilateral_stable_fraction"], 19 / 20)

    def test_loaded_gripper_summary_rejects_lost_force_in_hold_second_half(self):
        samples = [
            {
                "sim_time_ns": index * 1_000_000_000 // 19,
                "joint7": 0.0200,
                "joint8": -0.0200,
                "effort7": 1.0,
                "effort8": 1.1,
                "left_force_N": 1.0 if index < 10 else 0.0,
                "right_force_N": 0.9 if index < 10 else 0.0,
            }
            for index in range(20)
        ]
        summary = summarize_loaded_gripper_run(
            samples,
            target_joint7=0.0200,
            target_joint8=-0.0200,
            minimum_force_N=0.8,
        )

        self.assertFalse(summary["passed"])
        self.assertIn("low_left_second_half_median_force", summary["failure_reasons"])
        self.assertIn("bilateral_contact_too_short", summary["failure_reasons"])

    def test_loaded_gripper_summary_rejects_short_actual_hold(self):
        samples = [
            {
                "sim_time_ns": index * 900_000_000 // 19,
                "joint7": 0.0200,
                "joint8": -0.0200,
                "effort7": 1.0,
                "effort8": 1.1,
                "left_force_N": 1.0,
                "right_force_N": 0.9,
            }
            for index in range(20)
        ]

        summary = summarize_loaded_gripper_run(
            samples,
            target_joint7=0.0200,
            target_joint8=-0.0200,
            minimum_force_N=0.8,
            hold_s=1.0,
        )

        self.assertFalse(summary["passed"])
        self.assertIn("hold_window_short", summary["failure_reasons"])


if __name__ == "__main__":
    unittest.main()
