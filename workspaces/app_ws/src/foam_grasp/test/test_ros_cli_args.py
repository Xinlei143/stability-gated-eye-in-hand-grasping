import unittest
from pathlib import Path
from tempfile import NamedTemporaryFile

from foam_grasp.foam_cube_grasp_sequence import parse_args as parse_sequence_args
from foam_grasp.foam_move_to_observe import parse_args as parse_observe_args


class RosLaunchArgumentTest(unittest.TestCase):
    def test_grasp_sequence_accepts_launch_ros_arguments_after_app_arguments(self):
        with NamedTemporaryFile("w", suffix=".yaml") as params:
            args = parse_sequence_args(
                [
                    "--execution-backend",
                    "simulation",
                    "--target-class",
                    "cube",
                    "--auto-latch",
                    "--ros-args",
                    "-r",
                    "__node:=foam_static_grasp_sequence",
                    "--params-file",
                    str(Path(params.name)),
                ]
            )
        self.assertEqual(args.execution_backend, "simulation")
        self.assertTrue(args.auto_latch)

    def test_observe_command_accepts_launch_ros_arguments(self):
        args = parse_observe_args(
            [
                "--execution-backend",
                "simulation",
                "--execute",
                "--confirm",
                "AUTO_MOVE_TO_OBSERVE",
                "--ros-args",
                "-r",
                "__node:=foam_move_to_observe",
            ]
        )
        self.assertEqual(args.execution_backend, "simulation")
        self.assertTrue(args.execute)

    def test_observe_refreshes_feedback_before_execution_after_planning(self):
        source = (Path(__file__).parents[1] / "foam_grasp" / "foam_move_to_observe.py").read_text()
        self.assertIn(
            "node.spin_for(0.8)\n        refreshed = node.validate_robot_state()",
            source,
        )

    def test_terminal_trial_is_not_reclassified_as_failure_during_cleanup(self):
        source = (Path(__file__).parents[1] / "foam_grasp" / "foam_cube_grasp_sequence.py").read_text()
        self.assertIn("terminal_emitted = False", source)
        self.assertIn("if terminal_emitted:\n            return 0", source)

    def test_contact_confirmed_grasp_assist_is_prepared_before_lift(self):
        source = (Path(__file__).parents[1] / "foam_grasp" / "foam_cube_grasp_sequence.py").read_text()
        self.assertIn('declare_parameter("grasp_assist_service", "")', source)
        prepare = source.index("prepare_grasp_assist")
        lift = source.index('node.operator_gate(\n            f"确认{args.target_class}已被夹住')
        self.assertLess(prepare, lift)

    def test_gripper_margin_is_reported_as_jaw_blocking_evidence(self):
        source = (Path(__file__).parents[1] / "foam_grasp" / "foam_cube_grasp_sequence.py").read_text()
        self.assertIn("jaw_blocked_margin_mm", source)
        self.assertIn('"JAW_BLOCKED"', source)
        self.assertNotIn('"夹持确认："', source)

    def test_sequence_accepts_a_post_close_hold_duration(self):
        args = parse_sequence_args(
            [
                "--execution-backend", "simulation",
                "--target-class", "cube",
                "--post-close-hold-s", "1.0",
            ]
        )
        self.assertAlmostEqual(args.post_close_hold_s, 1.0)


if __name__ == "__main__":
    unittest.main()
