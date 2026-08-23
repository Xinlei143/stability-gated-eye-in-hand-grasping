"""Validated, deterministic expansion of Stage 6 benchmark suites.

This module intentionally has no ROS imports.  A suite describes one or more
invocations of the existing ``sim_bringup.launch.py`` entry point; it does not
describe another simulation pipeline.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


class SuiteValidationError(ValueError):
    """Raised when a suite cannot be expanded without ambiguity."""


_TOP_LEVEL = {
    "schema_version", "name", "defaults", "methods", "trajectories", "seeds",
    "sweeps", "method_overrides",
}
_METHODS = {"snapshot", "tracking", "gated"}
_TRAJECTORIES = {"static", "constant_velocity", "move_stop", "move_stop_move"}
_TARGETS = {"cube", "cylinder", "sphere"}
_SWEEP_PARAMETERS = {
    "latency_ms", "noise_std_mm", "dropout_probability", "outlier_probability",
    "outlier_range_mm", "stability_duration", "position_spread_threshold",
    "minimum_stable_samples", "observation_timeout", "velocity_x", "velocity_y",
    "velocity_z", "move_duration", "stop_duration", "trajectory",
}
_KNOWN_DEFAULTS = {
    "target_model", "execute_motion", "trajectory", "method", "latency_ms",
    "noise_std_mm", "dropout_probability", "outlier_probability", "outlier_range_mm",
    "velocity_x", "velocity_y", "velocity_z", "move_duration", "stop_duration",
    "motion_control_rate", "ground_truth_rate", "perception_sampling_rate",
    "history_duration", "stability_duration", "position_spread_threshold",
    "minimum_stable_samples", "observation_timeout", "tracking_commit_timeout",
    "tracking_replan_threshold", "tracking_commit_tolerance", "tracking_max_updates",
    "metrics_rate", "record_benchmark", "run_grasp_pipeline", "use_rviz",
    "perception_source", "outlier_range_mm", "scenario", "target_timeout",
    "timeout_s", "grasp_assist_mode", "grasp_assist_service",
    "grasp_stabilization_mode",
    "record_contact_diagnostics", "post_close_hold_s",
    "auto_pause_s",
    "countdown_seconds",
    "target_spawn_delay_s",
}
_FLOAT_RANGES = {
    "latency_ms": (0.0, 60_000.0),
    "noise_std_mm": (0.0, 1_000.0),
    "dropout_probability": (0.0, 1.0),
    "outlier_probability": (0.0, 1.0),
    "outlier_range_mm": (0.0, 10_000.0),
    "stability_duration": (0.0, 3_600.0),
    "position_spread_threshold": (0.0, 10.0),
    "observation_timeout": (0.001, 3_600.0),
    "velocity_x": (-10.0, 10.0),
    "velocity_y": (-10.0, 10.0),
    "velocity_z": (-10.0, 10.0),
    "move_duration": (0.0, 3_600.0),
    "stop_duration": (0.0, 3_600.0),
    "auto_pause_s": (0.5, 3.0),
    "target_spawn_delay_s": (1.0, 30.0),
}
_INT_PARAMETERS = {
    "minimum_stable_samples", "tracking_max_updates", "countdown_seconds"
}
_METHOD_ONLY = {"method"}


def _error(path: str, message: str) -> SuiteValidationError:
    return SuiteValidationError(f"{path}: {message}")


def _number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error(path, "must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise _error(path, "must be finite")
    return result


def _normalise(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _normalise(value[key]) for key in sorted(value)}
    if isinstance(value, (list, tuple)):
        return [_normalise(item) for item in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("non-finite value")
        return float(format(value, ".15g"))
    return value


def _canonical(value: Mapping[str, Any]) -> str:
    return json.dumps(_normalise(value), sort_keys=True, separators=(",", ":"), ensure_ascii=True)


@dataclass(frozen=True)
class TrialSpec:
    suite_name: str
    method: str
    trajectory: str
    seed: int
    target_model: str
    execute_motion: bool
    resolved: Mapping[str, Any]
    canonical_config: str
    config_hash: str
    pair_id: str
    run_id: str
    launch_args: tuple[str, ...]
    timeout_s: float

    @property
    def scenario(self) -> str:
        return str(self.resolved.get("scenario", self.trajectory))

    @property
    def latency_ms(self) -> float:
        return float(self.resolved.get("latency_ms", 0.0))

    @property
    def noise_std_mm(self) -> float:
        return float(self.resolved.get("noise_std_mm", 0.0))

    @property
    def dropout_probability(self) -> float:
        return float(self.resolved.get("dropout_probability", 0.0))


def _validate_mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise _error(path, "must be a mapping")
    return value


def _validate_suite(raw: Mapping[str, Any]) -> dict[str, Any]:
    unknown = set(raw) - _TOP_LEVEL
    if unknown:
        raise _error("suite", "unknown key(s): " + ", ".join(sorted(unknown)))
    required = {"schema_version", "name", "defaults", "methods", "trajectories", "seeds", "sweeps"}
    missing = required - set(raw)
    if missing:
        raise _error("suite", "missing key(s): " + ", ".join(sorted(missing)))
    if raw["schema_version"] != 1:
        raise _error("schema_version", "must be 1")
    if not isinstance(raw["name"], str) or not raw["name"].strip():
        raise _error("name", "must be a non-empty string")
    defaults = dict(_validate_mapping(raw["defaults"], "defaults"))
    unknown_defaults = set(defaults) - _KNOWN_DEFAULTS
    if unknown_defaults:
        raise _error("defaults", "unknown key(s): " + ", ".join(sorted(unknown_defaults)))
    stabilization_mode = str(defaults.get("grasp_stabilization_mode", "off")).strip().lower()
    if stabilization_mode not in {"off", "gazebo_grasp_fix", "legacy_contact_confirmed"}:
        raise _error("defaults.grasp_stabilization_mode", "is not a supported stabilization mode")
    assist_mode = str(defaults.get("grasp_assist_mode", "off")).strip().lower()
    assist_service = str(defaults.get("grasp_assist_service", "")).strip()
    if stabilization_mode == "gazebo_grasp_fix" and (assist_mode != "off" or assist_service):
        raise _error("defaults", "gazebo_grasp_fix and legacy grasp assist are mutually exclusive")
    methods = raw["methods"]
    trajectories = raw["trajectories"]
    seeds = raw["seeds"]
    for name, values in (("methods", methods), ("trajectories", trajectories), ("seeds", seeds)):
        if not isinstance(values, list) or not values:
            raise _error(name, "must be a non-empty list")
        if len(values) != len({json.dumps(item, sort_keys=True) for item in values}):
            raise _error(name, "must not contain duplicates")
    for index, method in enumerate(methods):
        if method not in _METHODS:
            raise _error(f"methods[{index}]", "must be snapshot, tracking, or gated")
    for index, trajectory in enumerate(trajectories):
        if trajectory not in _TRAJECTORIES:
            raise _error(f"trajectories[{index}]", "is not a supported trajectory")
    for index, seed in enumerate(seeds):
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise _error(f"seeds[{index}]", "must be an integer")
    sweeps = raw["sweeps"]
    if not isinstance(sweeps, list):
        raise _error("sweeps", "must be a list")
    for index, sweep in enumerate(sweeps):
        mapping = _validate_mapping(sweep, f"sweeps[{index}]")
        if set(mapping) != {"parameter", "values"}:
            raise _error(f"sweeps[{index}]", "must contain only parameter and values; multi-axis Cartesian sweeps are forbidden")
        parameter = mapping["parameter"]
        if parameter not in _SWEEP_PARAMETERS:
            raise _error(f"sweeps[{index}].parameter", "is not sweepable")
        values = mapping["values"]
        if not isinstance(values, list) or not values:
            raise _error(f"sweeps[{index}].values", "must be a non-empty list")
        if len(values) != len({json.dumps(item, sort_keys=True) for item in values}):
            raise _error(f"sweeps[{index}].values", "must not contain duplicates")
        for item_index, value in enumerate(values):
            if parameter in _INT_PARAMETERS:
                if isinstance(value, bool) or not isinstance(value, int):
                    raise _error(f"sweeps[{index}].values[{item_index}]", "must be an integer")
            elif parameter == "trajectory":
                if value not in _TRAJECTORIES:
                    raise _error(f"sweeps[{index}].values[{item_index}]", "is not a supported trajectory")
            else:
                number = _number(value, f"sweeps[{index}].values[{item_index}]")
                if parameter in _FLOAT_RANGES:
                    low, high = _FLOAT_RANGES[parameter]
                    if not low <= number <= high:
                        raise _error(f"sweeps[{index}].values[{item_index}]", f"must be within {low}..{high}")
    overrides = raw.get("method_overrides", {})
    if not isinstance(overrides, Mapping):
        raise _error("method_overrides", "must be a mapping")
    for method, values in overrides.items():
        if method not in methods:
            raise _error(f"method_overrides.{method}", "method is not selected")
        values = _validate_mapping(values, f"method_overrides.{method}")
        unknown_values = set(values) - set(defaults)
        if unknown_values:
            raise _error(f"method_overrides.{method}", "unknown key(s): " + ", ".join(sorted(unknown_values)))
    return {
        "schema_version": 1,
        "name": raw["name"].strip(),
        "defaults": defaults,
        "methods": list(methods),
        "trajectories": list(trajectories),
        "seeds": list(seeds),
        "sweeps": [dict(item) for item in sweeps],
        "method_overrides": {str(key): dict(value) for key, value in overrides.items()},
    }


def load_suite(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    try:
        with path.open(encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except OSError as error:
        raise SuiteValidationError(f"suite file: cannot read {path}: {error}") from error
    if not isinstance(raw, Mapping):
        raise _error("suite", "YAML document must be a mapping")
    return _validate_suite(raw)


def _launch_args(resolved: Mapping[str, Any], run_id: str, results_root: str) -> tuple[str, ...]:
    # Explicit mapping keeps all values shell-safe and avoids shell=True.
    names = (
        "use_rviz", "target_model", "run_grasp_pipeline", "execute_motion", "method",
        "scenario", "record_benchmark", "results_root", "run_id", "trajectory",
        "velocity_x", "velocity_y", "velocity_z", "move_duration", "stop_duration",
        "motion_control_rate", "ground_truth_rate", "perception_source",
        "perception_sampling_rate", "latency_ms", "noise_std_mm", "dropout_probability",
        "outlier_probability", "outlier_range_mm", "history_duration", "seed",
        "stability_duration", "position_spread_threshold", "minimum_stable_samples",
        "observation_timeout", "metrics_rate", "tracking_commit_timeout",
        "tracking_replan_threshold", "tracking_commit_tolerance", "tracking_max_updates",
        "grasp_assist_mode", "grasp_assist_service",
        "grasp_stabilization_mode",
        "record_contact_diagnostics", "post_close_hold_s",
        "auto_pause_s",
        "countdown_seconds",
        "target_spawn_delay_s",
    )
    args = ["ros2", "launch", "foam_grasp_sim", "sim_bringup.launch.py"]
    for name in names:
        if name == "results_root":
            value = results_root
        elif name == "run_id":
            value = run_id
        elif name not in resolved:
            continue
        else:
            value = resolved[name]
        if name == "grasp_assist_service" and value in (None, ""):
            # ros2 launch rejects the bare ``name:=`` form.  An empty service
            # means that the launch file's default (assist disabled) should
            # remain in effect, so omit the argument entirely.
            continue
        if isinstance(value, bool):
            value = "true" if value else "false"
        args.append(f"{name}:={value}")
    return tuple(args)


def expand_suite(raw_or_path: Mapping[str, Any] | str | Path) -> list[TrialSpec]:
    suite = load_suite(raw_or_path) if isinstance(raw_or_path, (str, Path)) else _validate_suite(raw_or_path)
    base_defaults = dict(suite["defaults"])
    sweep_variants: list[dict[str, Any]] = [{}]
    # Each declaration is an independent axis.  Consecutive declarations are
    # concatenated, never multiplied, so an accidental Cartesian product is
    # impossible.  A suite with sweeps contains exactly those sweep values;
    # defaults are used for all other parameters.
    if suite["sweeps"]:
        sweep_variants = [
            {sweep["parameter"]: value}
            for sweep in suite["sweeps"]
            for value in sweep["values"]
        ]

    trials: list[TrialSpec] = []
    for variant in sweep_variants:
        for trajectory in suite["trajectories"]:
            condition = dict(base_defaults)
            condition.update(variant)
            condition["trajectory"] = trajectory
            condition["scenario"] = condition.get("scenario", trajectory)
            for seed in suite["seeds"]:
                condition_with_seed = dict(condition, seed=seed)
                pair_specs = []
                for method in suite["methods"]:
                    resolved = dict(condition_with_seed)
                    resolved["method"] = method
                    resolved.update(suite["method_overrides"].get(method, {}))
                    if resolved.get("trajectory") != trajectory:
                        raise _error("method_overrides", "cannot change trajectory in a paired comparison")
                    canonical = _canonical(resolved)
                    config_hash = hashlib.sha256(canonical.encode()).hexdigest()
                    pair_material = dict(condition_with_seed)
                    pair_id = "pair-" + hashlib.sha256(_canonical(pair_material).encode()).hexdigest()[:16]
                    run_id = f"{suite['name']}-{method}-{trajectory}-s{seed}-{config_hash[:12]}"
                    timeout_s = float(resolved.get("timeout_s", 300.0))
                    pair_specs.append((method, resolved, canonical, config_hash, pair_id, run_id, timeout_s))
                # All selected methods must share every condition except the
                # method label itself.  This catches accidental method-only
                # overrides before any process is launched.
                comparable = [{key: value for key, value in item[1].items() if key != "method"} for item in pair_specs]
                if any(item != comparable[0] for item in comparable[1:]):
                    raise _error("methods", "paired methods must have identical non-method configuration")
                for method, resolved, canonical, config_hash, pair_id, run_id, timeout_s in pair_specs:
                    trials.append(TrialSpec(
                        suite_name=suite["name"], method=method,
                        trajectory=trajectory, seed=seed,
                        target_model=str(resolved.get("target_model", "cube")),
                        execute_motion=bool(resolved.get("execute_motion", False)),
                        resolved=_normalise(resolved), canonical_config=canonical,
                        config_hash=config_hash, pair_id=pair_id, run_id=run_id,
                        launch_args=_launch_args(resolved, run_id, "results"),
                        timeout_s=timeout_s,
                    ))
    return trials
