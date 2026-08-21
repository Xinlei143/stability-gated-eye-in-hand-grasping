import unittest

from foam_grasp_sim.method_policy import (
    METHODS,
    MethodConfig,
    MethodPolicy,
)


class MethodPolicyTest(unittest.TestCase):
    def test_snapshot_latches_first_valid_observation(self):
        policy = MethodPolicy(MethodConfig(method="snapshot"))

        first = policy.update((0.40, 0.00, 0.03), 1.0)
        later = policy.update((0.50, 0.00, 0.03), 2.0)

        self.assertTrue(first.ready)
        self.assertEqual(first.point, (0.40, 0.00, 0.03))
        self.assertEqual(later.point, first.point)
        self.assertEqual(policy.latched_point, first.point)

    def test_tracking_uses_latest_valid_observation_without_gate(self):
        policy = MethodPolicy(MethodConfig(method="tracking"))

        first = policy.update((0.40, 0.00, 0.03), 1.0)
        later = policy.update((0.50, 0.00, 0.03), 2.0)

        self.assertTrue(first.ready)
        self.assertTrue(later.ready)
        self.assertEqual(later.point, (0.50, 0.00, 0.03))

    def test_gated_requires_duration_and_minimum_samples(self):
        config = MethodConfig(
            method="gated",
            stability_duration_s=2.0,
            minimum_stable_samples=3,
            position_spread_threshold_m=0.006,
        )
        policy = MethodPolicy(config)

        self.assertFalse(policy.update((0.40, 0.00, 0.03), 0.0).ready)
        self.assertFalse(policy.update((0.401, 0.00, 0.03), 1.0).ready)
        final = policy.update((0.4005, 0.00, 0.03), 2.0)

        self.assertTrue(final.ready)
        self.assertAlmostEqual(final.point[0], 0.4005)

    def test_gated_resets_on_spread_violation_and_restarts_window(self):
        config = MethodConfig(
            method="gated",
            stability_duration_s=2.0,
            minimum_stable_samples=3,
            position_spread_threshold_m=0.006,
        )
        policy = MethodPolicy(config)
        policy.update((0.40, 0.00, 0.03), 0.0)
        policy.update((0.401, 0.00, 0.03), 1.0)
        reset = policy.update((0.42, 0.00, 0.03), 1.5)
        self.assertFalse(reset.ready)
        self.assertEqual(policy.stable_sample_count, 1)

        policy.update((0.4205, 0.00, 0.03), 2.5)
        still_waiting = policy.update((0.4202, 0.00, 0.03), 3.4)
        self.assertFalse(still_waiting.ready)
        ready = policy.update((0.4201, 0.00, 0.03), 3.5)
        self.assertTrue(ready.ready)

    def test_tracking_expires_when_observation_is_stale(self):
        policy = MethodPolicy(
            MethodConfig(method="tracking", observation_timeout_s=0.5)
        )
        policy.update((0.40, 0.00, 0.03), 1.0)
        expired = policy.expire(1.51)
        self.assertFalse(expired.ready)
        self.assertIsNone(expired.point)

    def test_invalid_method_and_gate_parameters_are_rejected(self):
        self.assertEqual(METHODS, ("snapshot", "tracking", "gated"))
        with self.assertRaises(ValueError):
            MethodConfig(method="unknown")
        with self.assertRaises(ValueError):
            MethodConfig(method="gated", stability_duration_s=0.0)
        with self.assertRaises(ValueError):
            MethodConfig(method="gated", minimum_stable_samples=1)


if __name__ == "__main__":
    unittest.main()
