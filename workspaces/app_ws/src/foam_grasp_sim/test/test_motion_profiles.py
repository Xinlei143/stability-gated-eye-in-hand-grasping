import unittest

from foam_grasp_sim.motion_profiles import (
    sample_motion,
    validate_motion_parameters,
)


START = (0.40, 0.00, 0.026)
VELOCITY = (0.01, -0.02, 0.00)


class MotionProfileTest(unittest.TestCase):
    def sample(self, profile, elapsed):
        return sample_motion(profile, START, VELOCITY, 4.0, 6.0, elapsed)

    def assert_point_equal(self, actual, expected):
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value)

    def test_static_never_commands_motion(self):
        sample = self.sample("static", 100.0)
        self.assert_point_equal(sample.position, START)
        self.assert_point_equal(sample.velocity, (0.0, 0.0, 0.0))
        self.assertFalse(sample.complete)

    def test_constant_velocity_is_unbounded(self):
        sample = self.sample("constant_velocity", 5.0)
        self.assert_point_equal(sample.position, (0.45, -0.10, 0.026))
        self.assert_point_equal(sample.velocity, VELOCITY)
        self.assertFalse(sample.complete)

    def test_move_stop_releases_after_first_segment(self):
        moving = self.sample("move_stop", 3.0)
        stopped = self.sample("move_stop", 4.0)
        self.assert_point_equal(moving.position, (0.43, -0.06, 0.026))
        self.assert_point_equal(moving.velocity, VELOCITY)
        self.assertFalse(moving.complete)
        self.assert_point_equal(stopped.position, (0.44, -0.08, 0.026))
        self.assert_point_equal(stopped.velocity, (0.0, 0.0, 0.0))
        self.assertTrue(stopped.complete)

    def test_move_stop_move_has_exact_stop_window_and_final_release(self):
        stopped = self.sample("move_stop_move", 7.0)
        second_move = self.sample("move_stop_move", 11.0)
        finished = self.sample("move_stop_move", 14.0)
        self.assert_point_equal(stopped.position, (0.44, -0.08, 0.026))
        self.assert_point_equal(stopped.velocity, (0.0, 0.0, 0.0))
        self.assertFalse(stopped.complete)
        self.assert_point_equal(second_move.position, (0.45, -0.10, 0.026))
        self.assert_point_equal(second_move.velocity, VELOCITY)
        self.assertFalse(second_move.complete)
        self.assert_point_equal(finished.position, (0.48, -0.16, 0.026))
        self.assertTrue(finished.complete)

    def test_invalid_motion_parameters_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "table plane"):
            validate_motion_parameters(
                "static", START, (0.0, 0.0, 0.01), 1.0, 0.0
            )
        with self.assertRaisesRegex(ValueError, "trajectory"):
            validate_motion_parameters("circle", START, VELOCITY, 1.0, 0.0)
        with self.assertRaisesRegex(ValueError, "move_duration"):
            validate_motion_parameters("static", START, VELOCITY, 0.0, 0.0)
