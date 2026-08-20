import unittest

from foam_grasp_sim.perception_model import (
    DelayedPointBuffer,
    DisturbanceModel,
    effective_latency_seconds,
    validate_perception_parameters,
)


class PerceptionModelTest(unittest.TestCase):
    def test_delayed_buffer_selects_latest_available_sample(self):
        buffer = DelayedPointBuffer(10.0)
        buffer.append(1.0, (0.1, 0.2, 0.3))
        buffer.append(2.0, (0.2, 0.2, 0.3))
        buffer.append(3.0, (0.3, 0.2, 0.3))
        self.assertEqual(buffer.latest_at_or_before(2.4).point, (0.2, 0.2, 0.3))
        self.assertEqual(buffer.latest_at_or_before(3.0).point, (0.3, 0.2, 0.3))
        self.assertIsNone(buffer.latest_at_or_before(0.9))

    def test_zero_disturbance_is_identity(self):
        model = DisturbanceModel(42, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(model.apply((0.4, 0.0, 0.03)), (0.4, 0.0, 0.03))

    def test_ground_truth_mode_ignores_configured_latency(self):
        self.assertEqual(effective_latency_seconds("ground_truth", 0.1), 0.0)
        self.assertEqual(effective_latency_seconds("disturbed", 0.1), 0.1)

    def test_seed_reproduces_disturbed_sequence(self):
        first = DisturbanceModel(42, 0.005, 0.1, 0.2, 0.05)
        second = DisturbanceModel(42, 0.005, 0.1, 0.2, 0.05)
        points = [(0.4 + 0.01 * index, 0.0, 0.03) for index in range(12)]
        self.assertEqual(
            [first.apply(point) for point in points],
            [second.apply(point) for point in points],
        )

    def test_full_dropout_emits_no_observation(self):
        model = DisturbanceModel(42, 0.0, 1.0, 0.0, 0.0)
        self.assertIsNone(model.apply((0.4, 0.0, 0.03)))

    def test_outliers_stay_within_configured_range(self):
        source = (0.4, 0.0, 0.03)
        model = DisturbanceModel(7, 0.0, 0.0, 1.0, 0.05)
        observed = model.apply(source)
        for value, source_value in zip(observed, source):
            self.assertLessEqual(abs(value - source_value), 0.05)

    def test_invalid_perception_parameters_are_rejected(self):
        arguments = ["disturbed", 10.0, 100.0, 5.0, 0.1, 0.01, 50.0, 10.0, 42]
        with self.assertRaisesRegex(ValueError, "dropout_probability"):
            validate_perception_parameters(
                *arguments[:4], 1.1, *arguments[5:]
            )
        with self.assertRaisesRegex(ValueError, "history_duration"):
            validate_perception_parameters(
                *arguments[:7], 0.05, arguments[8]
            )
        with self.assertRaisesRegex(ValueError, "seed"):
            validate_perception_parameters(*arguments[:-1], -1)
