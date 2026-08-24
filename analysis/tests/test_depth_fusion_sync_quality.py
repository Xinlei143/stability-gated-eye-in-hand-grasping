import unittest

from analysis.depth_fusion_sync_quality import summarize_records


class DepthFusionSyncQualityTest(unittest.TestCase):
    def test_summary_reports_signed_and_absolute_age_statistics(self):
        records = [
            {"mask_age_s": 0.04, "classes": {"cube": {"status": "valid"}}},
            {"mask_age_s": 0.10, "classes": {"cube": {"status": "valid"}}},
            {"mask_age_s": -0.02, "classes": {"cube": {"status": "valid"}}},
        ]

        summary = summarize_records(records, target_class="cube")

        self.assertEqual(summary["record_count"], 3)
        self.assertEqual(summary["paired_stamp_count"], 3)
        self.assertAlmostEqual(summary["median_mask_age_s"], 0.04)
        self.assertAlmostEqual(summary["median_abs_mask_age_s"], 0.04)
        self.assertEqual(summary["over_threshold_count"], 0)
        self.assertTrue(summary["sync_pass"])

    def test_summary_fails_when_any_absolute_age_exceeds_threshold(self):
        records = [
            {"mask_age_s": 0.04, "classes": {"cube": {"status": "valid"}}},
            {"mask_age_s": 0.16, "classes": {"cube": {"status": "stale_mask"}}},
            {"mask_age_s": None, "classes": {"cube": {"status": "no_mask"}}},
        ]

        summary = summarize_records(records, target_class="cube")

        self.assertEqual(summary["paired_stamp_count"], 2)
        self.assertEqual(summary["over_threshold_count"], 1)
        self.assertAlmostEqual(summary["over_threshold_fraction"], 0.5)
        self.assertEqual(summary["stale_mask_count"], 1)
        self.assertEqual(summary["no_mask_count"], 1)
        self.assertFalse(summary["sync_pass"])


if __name__ == "__main__":
    unittest.main()
