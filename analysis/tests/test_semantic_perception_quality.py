import math
import unittest

from analysis.semantic_perception_quality import summarize_rows


def _row(**values):
    row = {
        "target_ground_truth_x": "0.400",
        "target_ground_truth_y": "0.000",
        "target_ground_truth_z": "0.030",
        "target_observed_x": "0.410",
        "target_observed_y": "-0.020",
        "target_observed_z": "0.020",
    }
    row.update(values)
    return row


class SemanticPerceptionQualityTest(unittest.TestCase):
    def test_compares_raw_observation_with_ground_truth(self):
        summary = summarize_rows(
            [
                _row(),
                _row(
                    target_observed_x="",
                    target_observed_y="",
                    target_observed_z="",
                ),
            ]
        )

        self.assertEqual(summary["row_count"], 2)
        self.assertEqual(summary["ground_truth_valid_count"], 2)
        self.assertEqual(summary["valid_observation_count"], 1)
        self.assertAlmostEqual(summary["valid_observation_fraction"], 0.5)
        self.assertAlmostEqual(summary["median_abs_ex_m"], 0.01)
        self.assertAlmostEqual(summary["median_abs_ey_m"], 0.02)
        self.assertAlmostEqual(summary["median_abs_ez_m"], 0.01)
        self.assertAlmostEqual(summary["median_planar_error_m"], math.sqrt(0.0005))
        self.assertAlmostEqual(summary["median_3d_error_m"], math.sqrt(0.0006))
        self.assertAlmostEqual(summary["max_3d_error_m"], math.sqrt(0.0006))

    def test_stale_latched_observation_does_not_count_as_fresh(self):
        summary = summarize_rows(
            [
                _row(observation_age_s="0.05", observation_fresh="1"),
                _row(observation_age_s="0.50", observation_fresh="0"),
            ]
        )

        self.assertEqual(summary["valid_observation_count"], 1)
        self.assertAlmostEqual(summary["valid_observation_fraction"], 0.5)

    def test_missing_required_column_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "target_observed_z"):
            summarize_rows([{"target_ground_truth_x": "0.4"}])


if __name__ == "__main__":
    unittest.main()
