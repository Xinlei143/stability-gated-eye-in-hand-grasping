import unittest

from foam_grasp.benchmark_events import TrialTaskFailure, task_failure_details


class TrialOutcomeTest(unittest.TestCase):
    def test_task_failure_keeps_stage_and_reason_for_terminal_event(self):
        error = TrialTaskFailure("planning", "no collision-free candidate")
        self.assertEqual(
            task_failure_details(error),
            {
                "outcome": "task_failure",
                "failure_stage": "planning",
                "reason": "no collision-free candidate",
            },
        )


if __name__ == "__main__":
    unittest.main()
