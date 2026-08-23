import csv
import json
import tempfile
import unittest
from pathlib import Path

import yaml

from foam_grasp_sim.static_grasp_diagnosis import (
    build_diagnostics_rows,
    classify_diagnosis,
    summarize_static_hold,
    write_sweep_summary_csv,
)


class StaticGraspDiagnosisTest(unittest.TestCase):
    def test_static_diagnosis_uses_the_fixed_grasp_pose_not_pregrasp_pose(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "static_grasp_hold_diagnosis.yaml"
        with config_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config["arm_pose"], [0.0, 1.85, -1.07, 0.0, 0.525, 0.0])

    def test_static_diagnosis_spawns_free_cube_after_arm_settle(self):
        launch_path = Path(__file__).resolve().parents[1] / "launch" / "static_grasp_hold_diagnosis.launch.py"
        source = launch_path.read_text(encoding="utf-8")
        node_path = Path(__file__).resolve().parents[1] / "foam_grasp_sim" / "static_grasp_hold_diagnosis_node.py"
        node_source = node_path.read_text(encoding="utf-8")
        self.assertIn("SpawnEntity", node_source)
        self.assertIn("_spawn_free_cube", node_source)
        self.assertNotIn("spawn_free_cube_for_static_diagnosis", source)

    def test_static_diagnosis_opens_gripper_before_spawning_cube(self):
        node_path = Path(__file__).resolve().parents[1] / "foam_grasp_sim" / "static_grasp_hold_diagnosis_node.py"
        source = node_path.read_text(encoding="utf-8")
        open_call = source.index("self._send_pair(open_half, -open_half)")
        spawn_call = source.index("self._spawn_free_cube()")
        close_call = source.index("self._send_pair(close_half, -close_half)")
        self.assertLess(open_call, spawn_call)
        self.assertLess(spawn_call, close_call)

    def test_resampling_uses_sim_time_grid_and_zero_fills_missing_contacts(self):
        joints = [
            {
                "sim_time_ns": 0,
                "joint7_position_m": 0.035,
                "joint8_position_m": -0.035,
                "joint7_velocity_m_s": 0.0,
                "joint8_velocity_m_s": 0.0,
                "joint7_effort_N": 1.0,
                "joint8_effort_N": -1.0,
                "commanded_joint7_position_m": 0.020,
                "commanded_joint8_position_m": -0.020,
                "stage": "hold",
            },
            {
                "sim_time_ns": 20_000_000,
                "joint7_position_m": 0.035,
                "joint8_position_m": -0.035,
                "joint7_velocity_m_s": 0.0,
                "joint8_velocity_m_s": 0.0,
                "joint7_effort_N": 1.0,
                "joint8_effort_N": -1.0,
                "commanded_joint7_position_m": 0.020,
                "commanded_joint8_position_m": -0.020,
                "stage": "hold",
            },
        ]
        contacts = [
            {"sim_time_ns": 0, "side": "left", "normal_force_N": 1.0},
            {"sim_time_ns": 0, "side": "right", "normal_force_N": 1.0},
        ]
        cube = [
            {"sim_time_ns": 0, "x_m": 0.4, "y_m": 0.0, "z_m": 0.026},
            {"sim_time_ns": 20_000_000, "x_m": 0.4, "y_m": 0.0, "z_m": 0.026},
        ]

        rows = build_diagnostics_rows(
            joints, contacts, cube, start_ns=0, end_ns=20_000_000, grid_hz=100.0
        )

        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["timestamp_sim_time_ns"], 0)
        self.assertEqual(rows[1]["timestamp_sim_time_ns"], 10_000_000)
        self.assertEqual(rows[2]["timestamp_sim_time_ns"], 20_000_000)
        self.assertEqual(rows[1]["left_contact_normal_force_N"], 0.0)
        self.assertEqual(rows[1]["right_contact_normal_force_N"], 0.0)
        self.assertAlmostEqual(rows[1]["joint7_effort_N"], 1.0)

    def test_summary_reports_force_duration_error_correlation_and_cube_displacement(self):
        rows = []
        for index in range(201):
            force = 1.0 if index < 151 else 0.0
            rows.append(
                {
                    "timestamp_sim_time_ns": index * 10_000_000,
                    "time_s": index / 100.0,
                    "joint7_position_m": 0.030,
                    "joint8_position_m": -0.030,
                    "joint7_velocity_m_s": 0.0,
                    "joint8_velocity_m_s": 0.0,
                    "joint7_effort_N": 2.0,
                    "joint8_effort_N": -2.0,
                    "commanded_joint7_position_m": 0.020,
                    "commanded_joint8_position_m": -0.020,
                    "gripper_total_opening_m": 0.060,
                    "left_contact_normal_force_N": force,
                    "right_contact_normal_force_N": force,
                    "cube_x_m": 0.4 + index * 1e-5,
                    "cube_y_m": 0.0,
                    "cube_z_m": 0.026,
                }
            )
        summary = summarize_static_hold(rows, hold_s=2.0, force_threshold_N=0.8)

        self.assertAlmostEqual(summary["median_joint7_effort_N"], 2.0)
        self.assertAlmostEqual(summary["median_joint8_effort_N"], -2.0)
        self.assertAlmostEqual(summary["median_left_contact_normal_force_N"], 1.0)
        self.assertAlmostEqual(summary["bilateral_nonzero_fraction"], 151 / 201)
        self.assertAlmostEqual(summary["bilateral_force_qualified_fraction"], 151 / 201)
        self.assertAlmostEqual(summary["longest_bilateral_force_qualified_duration_s"], 1.5)
        self.assertGreater(summary["cube_displacement_during_hold_m"], 0.001)
        self.assertEqual(summary["static_hold_passed"], False)

    def test_classifier_distinguishes_low_effort_from_solver_dropouts(self):
        base = {
            "position_error_sustained": True,
            "effort_sustained": True,
            "contact_persistent": False,
            "contact_force_low": False,
        }
        self.assertEqual(classify_diagnosis({**base, "effort_sustained": False}), "Case B")
        self.assertEqual(classify_diagnosis(base), "Case A")
        self.assertEqual(
            classify_diagnosis(
                {
                    **base,
                    "contact_persistent": True,
                    "contact_force_low": True,
                }
            ),
            "Case C",
        )
        self.assertEqual(
            classify_diagnosis(
                {
                    **base,
                    "contact_persistent": True,
                    "contact_force_low": False,
                }
            ),
            "Case D",
        )

    def test_summary_treats_small_but_continuous_effort_as_case_c(self):
        rows = []
        for index in range(201):
            rows.append({
                "timestamp_sim_time_ns": index * 10_000_000,
                "time_s": index / 100.0,
                "joint7_position_m": 0.025,
                "joint8_position_m": -0.025,
                "joint7_effort_N": -0.025,
                "joint8_effort_N": 0.025,
                "commanded_joint7_position_m": 0.020,
                "commanded_joint8_position_m": -0.020,
                "left_contact_normal_force_N": 0.24,
                "right_contact_normal_force_N": 0.24,
                "cube_x_m": 0.4,
                "cube_y_m": 0.0,
                "cube_z_m": 0.026,
            })
        summary = summarize_static_hold(rows, hold_s=2.0)
        self.assertEqual(summary["diagnosis_case"], "Case C")

    def test_sweep_summary_csv_contains_requested_comparison_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sweep_summary.csv"
            write_sweep_summary_csv(
                path,
                [
                    {
                        "kp": 300.0,
                        "summary": {
                            "median_joint7_effort_N": 1.2,
                            "median_joint8_effort_N": -1.1,
                            "median_left_contact_normal_force_N": 0.9,
                            "median_right_contact_normal_force_N": 1.0,
                            "bilateral_nonzero_fraction": 0.8,
                            "longest_bilateral_nonzero_duration_s": 0.7,
                            "p95_contact_force_N": 1.4,
                            "static_hold_passed": True,
                        },
                    }
                ],
            )
            with path.open(newline="", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["Kp"], "300.0")
            self.assertEqual(row["static_grasp_qualification"], "true")
            self.assertIn("median_contact_force_N", row)


if __name__ == "__main__":
    unittest.main()
