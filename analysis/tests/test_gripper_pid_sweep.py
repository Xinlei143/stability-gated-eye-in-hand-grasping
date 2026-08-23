import unittest

from scripts.gripper_pid_sweep import (
    _aggregate_kp_details,
    build_launch_command,
    build_launch_environment,
    select_minimum_passing_kp,
)


class GripperPidSweepTest(unittest.TestCase):
    def test_build_launch_command_is_loaded_gripper_only_and_uses_overrides(self):
        command = build_launch_command(
            config_path="/tmp/pid.yaml",
            robot_xacro="/tmp/loaded.xacro",
            output_dir="/tmp/run",
            qualification_config="/tmp/qualification.yaml",
        )

        self.assertEqual(command[:3], ["ros2", "launch", "foam_grasp_sim"])
        self.assertIn("mode:=loaded_gripper", command)
        self.assertIn("robot_xacro:=/tmp/loaded.xacro", command)
        self.assertIn("physics_pid_config:=/tmp/pid.yaml", command)
        self.assertIn("output_dir:=/tmp/run", command)

    def test_select_minimum_passing_kp_requires_every_repeat(self):
        results = {
            50.0: [False, True, True],
            100.0: [True, True, True],
            200.0: [True, True, False],
        }
        self.assertEqual(select_minimum_passing_kp(results), 100.0)

    def test_build_launch_environment_uses_an_isolated_gazebo_master_port(self):
        environment = build_launch_environment(11642)
        self.assertEqual(environment["GAZEBO_MASTER_URI"], "http://127.0.0.1:11642")
        self.assertEqual(environment["ROS_DOMAIN_ID"], "192")

    def test_sweep_requires_three_repeats_for_a_candidate(self):
        with self.assertRaisesRegex(ValueError, "repeats"):
            from scripts.gripper_pid_sweep import run_sweep
            run_sweep(
                source_pid_config="/tmp/missing.yaml",
                robot_xacro="/tmp/missing.xacro",
                qualification_config="/tmp/missing.yaml",
                output_root="/tmp/gripper-sweep-test",
                kps=(200.0,),
                repeats=1,
                dry_run=True,
            )

    def test_kp_aggregate_reports_pass_rate_and_quality_metrics(self):
        aggregate = _aggregate_kp_details([
            {
                "passed": True,
                "summary": {
                    "runs": [{
                        "left_median_force_N": 1.0,
                        "right_median_force_N": 0.9,
                        "left_p95_force_N": 1.2,
                        "right_p95_force_N": 1.1,
                        "longest_contiguous_bilateral_contact_s": 1.0,
                        "symmetry_error_mm": 0.2,
                        "settled_oscillation_mm": 0.1,
                    }],
                },
            },
            {
                "passed": False,
                "summary": {
                    "runs": [{
                        "left_median_force_N": 0.5,
                        "right_median_force_N": 0.4,
                        "left_p95_force_N": 0.7,
                        "right_p95_force_N": 0.6,
                        "longest_contiguous_bilateral_contact_s": 0.4,
                        "symmetry_error_mm": 0.8,
                        "settled_oscillation_mm": 0.6,
                    }],
                },
            },
        ])

        self.assertEqual(aggregate["repeat_count"], 2)
        self.assertEqual(aggregate["pass_count"], 1)
        self.assertEqual(aggregate["pass_rate"], 0.5)
        self.assertEqual(aggregate["metrics"]["left_median_force_N"]["sample_count"], 2)


if __name__ == "__main__":
    unittest.main()
