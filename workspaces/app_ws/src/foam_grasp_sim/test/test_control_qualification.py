import unittest

from foam_grasp_sim.control_qualification import (
    summarize_arm_run,
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


if __name__ == "__main__":
    unittest.main()
