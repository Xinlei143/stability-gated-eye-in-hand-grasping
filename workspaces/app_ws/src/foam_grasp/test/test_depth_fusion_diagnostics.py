import unittest
from pathlib import Path

from foam_grasp.depth_fusion_diagnostics import (
    build_diagnostic,
    select_closest_mask,
)


class DepthFusionDiagnosticsTest(unittest.TestCase):
    def test_record_contains_stage_counts_and_camera_point(self):
        record = build_diagnostic(
            depth_stamp=12.0,
            mask_stamp=11.98,
            frame_id="camera_color_optical_frame",
            frame_count=30,
            valid_output_count=28,
            output_rate_hz=29.5,
            classes={
                "cube": {
                    "status": "valid",
                    "mask_pixels": 1000,
                    "component_pixels": 900,
                    "eroded_pixels": 700,
                    "valid_depth_pixels": 650,
                    "point_camera": [0.01, -0.02, 0.40],
                }
            },
        )

        self.assertEqual(record["schema_version"], 1)
        self.assertAlmostEqual(record["mask_depth_delta_s"], 0.02)
        self.assertEqual(record["classes"]["cube"]["valid_depth_pixels"], 650)
        self.assertEqual(record["classes"]["cube"]["point_camera"], [0.01, -0.02, 0.40])
        self.assertEqual(record["frame_count"], 30)
        self.assertEqual(record["valid_output_count"], 28)

    def test_node_keeps_dc1_fusion_thresholds_and_publishes_failure_reasons(self):
        source = (Path(__file__).parents[1] / "foam_grasp" / "foam_depth_fusion_node.py").read_text(encoding="utf-8")
        self.assertIn("MAX_TIME_DIFFERENCE_SECONDS = 0.15", source)
        self.assertIn("MIN_COMPONENT_AREA_PIXELS = 150", source)
        self.assertIn("MIN_VALID_DEPTH_PIXELS = 50", source)
        self.assertIn("MIN_DEPTH_METERS = 0.15", source)
        self.assertIn("MAX_DEPTH_METERS = 1.50", source)
        for status in (
            "component_too_small",
            "insufficient_eroded_pixels",
            "insufficient_valid_depth",
            "depth_outlier_rejection",
            "stale_mask",
        ):
            self.assertIn(status, source)
        self.assertIn("DIAGNOSTIC_TOPIC", source)

    def test_closest_mask_selection_avoids_latest_frame_lag(self):
        selected = select_closest_mask(
            [(10.00, "old"), (10.06, "near"), (10.20, "future")],
            depth_stamp=10.08,
        )
        self.assertEqual(selected, (10.06, "near"))

        self.assertIsNone(select_closest_mask([], depth_stamp=10.0))


if __name__ == "__main__":
    unittest.main()
