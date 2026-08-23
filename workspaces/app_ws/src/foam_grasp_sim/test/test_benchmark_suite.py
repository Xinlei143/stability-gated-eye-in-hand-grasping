import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import yaml

from foam_grasp_sim.benchmark_suite import (
    SuiteValidationError,
    TrialSpec,
    expand_suite,
    load_suite,
)


def _suite(**overrides):
    value = {
        "schema_version": 1,
        "name": "unit",
        "defaults": {
            "target_model": "cube",
            "execute_motion": False,
            "trajectory": "static",
            "latency_ms": 10.0,
            "noise_std_mm": 2.0,
            "dropout_probability": 0.1,
            "outlier_probability": 0.0,
            "outlier_range_mm": 50.0,
            "velocity_x": 0.01,
            "velocity_y": 0.0,
            "velocity_z": 0.0,
            "move_duration": 4.0,
            "stop_duration": 6.0,
            "stability_duration": 5.0,
            "position_spread_threshold": 0.006,
            "minimum_stable_samples": 25,
            "observation_timeout": 1.0,
        },
        "methods": ["snapshot", "tracking", "gated"],
        "trajectories": ["static"],
        "seeds": [42],
        "sweeps": [],
    }
    value.update(overrides)
    return value


class BenchmarkSuiteTest(unittest.TestCase):
    def test_expands_one_axis_and_pairs_methods(self):
        suite = _suite(sweeps=[{"parameter": "latency_ms", "values": [0, 50]}])
        trials = expand_suite(suite)
        self.assertEqual(len(trials), 6)
        self.assertIsInstance(trials[0], TrialSpec)
        self.assertEqual(trials[0].method, "snapshot")
        self.assertEqual(trials[0].latency_ms, 0.0)
        self.assertEqual(
            {trial.pair_id for trial in trials if trial.latency_ms == 0.0},
            {trials[0].pair_id},
        )
        self.assertEqual(
            {trial.method for trial in trials if trial.pair_id == trials[0].pair_id},
            {"snapshot", "tracking", "gated"},
        )

    def test_optional_grasp_assist_settings_are_forwarded_to_launch(self):
        suite = _suite(
            defaults={
                **_suite()["defaults"],
                "grasp_assist_mode": "contact_confirmed",
                "grasp_assist_service": "/foam_grasp_sim/prepare_grasp_assist",
            }
        )
        trial = expand_suite(suite)[0]
        self.assertIn("grasp_assist_mode:=contact_confirmed", trial.launch_args)
        self.assertIn(
            "grasp_assist_service:=/foam_grasp_sim/prepare_grasp_assist",
            trial.launch_args,
        )

    def test_empty_grasp_assist_service_is_not_emitted_as_malformed_launch_arg(self):
        suite = _suite(
            defaults={
                **_suite()["defaults"],
                "grasp_assist_mode": "off",
                "grasp_assist_service": "",
            }
        )
        trial = expand_suite(suite)[0]
        self.assertIn("grasp_assist_mode:=off", trial.launch_args)
        self.assertNotIn("grasp_assist_service:=", trial.launch_args)

    def test_ids_are_stable_and_config_hash_is_canonical(self):
        first = expand_suite(_suite())[0]
        reordered = _suite()
        reordered["defaults"] = dict(reversed(list(reordered["defaults"].items())))
        second = expand_suite(reordered)[0]
        self.assertEqual(first.config_hash, second.config_hash)
        self.assertEqual(first.run_id, second.run_id)
        self.assertEqual(first.pair_id, second.pair_id)
        expected = hashlib.sha256(first.canonical_config.encode()).hexdigest()
        self.assertEqual(first.config_hash, expected)

    def test_rejects_cartesian_sweep_and_unknown_keys(self):
        with self.assertRaises(SuiteValidationError):
            expand_suite(_suite(sweeps=[
                {"parameter": "latency_ms", "values": [0, 10], "noise_std_mm": [0, 1]}
            ]))
        bad = _suite(extra=True)
        with self.assertRaises(SuiteValidationError):
            expand_suite(bad)

    def test_rejects_paired_condition_mismatch(self):
        suite = _suite(methods=["snapshot", "tracking"])
        suite["method_overrides"] = {"tracking": {"latency_ms": 20}}
        with self.assertRaises(SuiteValidationError):
            expand_suite(suite)

    def test_load_suite_reports_yaml_path(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(_suite(sweeps=[{"parameter": "latency_ms", "values": []}])))
            with self.assertRaisesRegex(SuiteValidationError, "sweeps"):
                load_suite(path)

    def test_standard_suites_are_valid_and_have_expected_shape(self):
        root = Path(__file__).resolve().parents[1] / "config" / "benchmark_suites"
        expected = {
            "smoke", "baseline_comparison", "latency_sweep", "noise_sweep",
            "dropout_sweep", "gate_ablation", "core_baseline_formal",
            "grasp_physics_qualification",
        }
        self.assertEqual({path.stem for path in root.glob("*.yaml")}, expected)
        for name in sorted(expected):
            suite = load_suite(root / f"{name}.yaml")
            trials = expand_suite(suite)
            self.assertTrue(trials, name)
            if name == "smoke":
                self.assertEqual(len(trials), 1)
                self.assertFalse(trials[0].execute_motion)
                self.assertEqual(trials[0].resolved["perception_source"], "ground_truth")
            if name == "baseline_comparison":
                self.assertEqual({trial.method for trial in trials}, {"snapshot", "tracking", "gated"})
            if name == "core_baseline_formal":
                self.assertEqual(len(trials), 120)
                self.assertTrue(all(trial.timeout_s == 90.0 for trial in trials))
                self.assertTrue(
                    all(trial.resolved["grasp_assist_mode"] == "off" for trial in trials)
                )
            if name == "grasp_physics_qualification":
                self.assertEqual(len(trials), 1)
                self.assertEqual(trials[0].method, "snapshot")
                self.assertEqual(trials[0].trajectory, "static")
                self.assertEqual(trials[0].seed, 42)
                self.assertTrue(trials[0].execute_motion)
                self.assertEqual(trials[0].resolved["move_duration"], 4.0)
                self.assertEqual(trials[0].resolved["stop_duration"], 6.0)
                self.assertEqual(trials[0].resolved["post_close_hold_s"], 1.0)
                self.assertEqual(trials[0].resolved["auto_pause_s"], 0.5)
                self.assertEqual(trials[0].resolved["countdown_seconds"], 0)
                self.assertTrue(trials[0].resolved["record_contact_diagnostics"])
                self.assertIn("record_contact_diagnostics:=true", trials[0].launch_args)
                self.assertIn("post_close_hold_s:=1.0", trials[0].launch_args)
                self.assertIn("auto_pause_s:=0.5", trials[0].launch_args)
                self.assertIn("countdown_seconds:=0", trials[0].launch_args)

    def test_standard_suites_have_unique_deterministic_run_ids(self):
        root = Path(__file__).resolve().parents[1] / "config" / "benchmark_suites"
        for path in sorted(root.glob("*.yaml")):
            trials = expand_suite(load_suite(path))
            run_ids = [trial.run_id for trial in trials]
            self.assertEqual(len(run_ids), len(set(run_ids)), path.name)


if __name__ == "__main__":
    unittest.main()
