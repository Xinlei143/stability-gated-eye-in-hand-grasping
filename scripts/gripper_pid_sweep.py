#!/usr/bin/env python3
"""Run isolated loaded-gripper qualification trials across PID gains."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import signal
from statistics import median
import subprocess
from pathlib import Path

import yaml

from foam_grasp_sim.static_grasp_diagnosis import write_sweep_summary_csv


DEFAULT_KPS = (50.0, 100.0, 150.0, 200.0, 250.0, 300.0)
STATIC_HOLD_DEFAULT_KPS = (300.0, 350.0, 400.0, 450.0, 500.0)


def _percentile(values, fraction):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def _aggregate_kp_details(details):
    summaries = [
        entry.get("summary", {})
        for entry in details
        if isinstance(entry.get("summary", {}), dict)
    ]
    runs = [
        run
        for summary in summaries
        for run in summary.get("runs", [])
        if isinstance(run, dict)
    ]
    metric_names = (
        "left_median_force_N",
        "right_median_force_N",
        "left_p95_force_N",
        "right_p95_force_N",
        "longest_contiguous_bilateral_contact_s",
        "symmetry_error_mm",
        "settled_oscillation_mm",
    )
    metrics = {}
    for name in metric_names:
        values = [
            float(run[name])
            for run in runs
            if name in run and math.isfinite(float(run[name]))
        ]
        metrics[name] = {
            "median": float(median(values)) if values else None,
            "p95": _percentile(values, 0.95),
            "sample_count": len(values),
        }
    pass_count = sum(bool(entry.get("passed")) for entry in details)
    return {
        "repeat_count": len(details),
        "pass_count": pass_count,
        "pass_rate": pass_count / len(details) if details else 0.0,
        "metrics": metrics,
    }


def build_launch_command(*, config_path, robot_xacro, output_dir, qualification_config):
    return [
        "ros2",
        "launch",
        "foam_grasp_sim",
        "control_physics_qualification.launch.py",
        "mode:=loaded_gripper",
        f"config:={qualification_config}",
        f"output_dir:={output_dir}",
        f"robot_xacro:={robot_xacro}",
        f"physics_pid_config:={config_path}",
    ]


def build_static_hold_launch_command(*, config_path, robot_xacro, output_dir,
                                     diagnosis_config):
    return [
        "ros2",
        "launch",
        "foam_grasp_sim",
        "static_grasp_hold_diagnosis.launch.py",
        f"config:={diagnosis_config}",
        f"output_dir:={output_dir}",
        f"robot_xacro:={robot_xacro}",
        f"physics_pid_config:={config_path}",
    ]


def select_minimum_passing_kp(results):
    for kp in sorted(results):
        repeats = list(results[kp])
        if repeats and all(repeats):
            return float(kp)
    return None


def build_launch_environment(gazebo_master_port):
    environment = os.environ.copy()
    port = int(gazebo_master_port)
    environment["GAZEBO_MASTER_URI"] = f"http://127.0.0.1:{port}"
    environment["ROS_DOMAIN_ID"] = str((port - 11450) % 230)
    return environment


def _process_group_exists(process):
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _pid_config(source_path, output_path, kp):
    with Path(source_path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    config = copy.deepcopy(config)
    gains = config["gazebo_ros2_control"]["ros__parameters"]["pid_gains"]["position_pid"]
    for joint in ("joint7", "joint8"):
        gains[joint]["kp"] = float(kp)
        gains[joint]["ki"] = 0.0
        gains[joint]["kd"] = 0.3
    Path(output_path).write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def _stop_process_group(process):
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass


def run_sweep(*, source_pid_config, robot_xacro, qualification_config, output_root,
              kps=DEFAULT_KPS, repeats=3, dry_run=False, trial_timeout_s=90.0):
    if int(repeats) < 3:
        raise ValueError("repeats must be at least 3 for PID candidate selection")
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    pid_config_root = output_root / "pid_configs"
    pid_config_root.mkdir(parents=True, exist_ok=True)
    results = {float(kp): [] for kp in kps}
    details = {float(kp): [] for kp in kps}
    commands = []
    trial_index = 0
    for kp in kps:
        pid_path = pid_config_root / f"kp_{float(kp):g}.yaml"
        _pid_config(source_pid_config, pid_path, kp)
        for repeat in range(1, int(repeats) + 1):
            trial_index += 1
            run_dir = output_root / f"kp_{float(kp):g}" / f"repeat_{repeat}"
            run_dir.mkdir(parents=True, exist_ok=True)
            command = build_launch_command(
                config_path=str(pid_path),
                robot_xacro=robot_xacro,
                output_dir=str(run_dir),
                qualification_config=qualification_config,
            )
            commands.append(command)
            if dry_run:
                continue
            log_path = run_dir / "launch.log"
            with log_path.open("w", encoding="utf-8") as log:
                process = subprocess.Popen(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                    env={
                        **build_launch_environment(11470 + trial_index),
                        "ROS_LOG_DIR": str(output_root / "ros_logs" / f"kp_{kp:g}_repeat_{repeat}"),
                    },
                )
                try:
                    return_code = process.wait(timeout=float(trial_timeout_s))
                except subprocess.TimeoutExpired:
                    _stop_process_group(process)
                    return_code = -signal.SIGTERM
                except KeyboardInterrupt:
                    _stop_process_group(process)
                    raise
                finally:
                    # A ros2 launch leader can exit while Gazebo remains in
                    # the same process group; clean that group on every path.
                    if _process_group_exists(process):
                        _stop_process_group(process)
            summary_path = run_dir / "summary.json"
            passed = False
            summary = {"passed": False, "return_code": int(return_code)}
            if summary_path.is_file():
                try:
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                    passed = bool(summary.get("passed"))
                except (OSError, ValueError, TypeError):
                    passed = False
            results[float(kp)].append(bool(return_code == 0 and passed))
            details[float(kp)].append({
                "repeat": repeat,
                "return_code": int(return_code),
                "passed": bool(return_code == 0 and passed),
                "summary": summary,
            })
    aggregates = {
        float(kp): _aggregate_kp_details(details[float(kp)])
        for kp in details
    }
    return {
        "results": results,
        "details": details,
        "aggregates": aggregates,
        "selected_kp": select_minimum_passing_kp(results),
        "commands": commands,
    }


def run_static_hold_sweep(*, source_pid_config, robot_xacro, diagnosis_config,
                          output_root, kps=STATIC_HOLD_DEFAULT_KPS, repeats=3,
                          dry_run=False, trial_timeout_s=120.0):
    """Run the free-cube static hold diagnosis for every PID candidate."""

    if int(repeats) < 3:
        raise ValueError("repeats must be at least 3 for PID candidate selection")
    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    pid_config_root = output_root / "pid_configs"
    pid_config_root.mkdir(parents=True, exist_ok=True)
    entries = []
    commands = []
    trial_index = 0
    for kp in kps:
        kp = float(kp)
        pid_path = pid_config_root / f"kp_{kp:g}.yaml"
        _pid_config(source_pid_config, pid_path, kp)
        for repeat in range(1, int(repeats) + 1):
            trial_index += 1
            run_dir = output_root / f"kp_{kp:g}" / f"repeat_{repeat}"
            run_dir.mkdir(parents=True, exist_ok=True)
            command = build_static_hold_launch_command(
                config_path=pid_path,
                robot_xacro=robot_xacro,
                output_dir=run_dir,
                diagnosis_config=diagnosis_config,
            )
            commands.append(command)
            if dry_run:
                continue
            log_path = run_dir / "launch.log"
            with log_path.open("w", encoding="utf-8") as log:
                process = subprocess.Popen(
                    command,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    text=True,
                    env={
                        **build_launch_environment(11470 + trial_index),
                        "ROS_LOG_DIR": str(output_root / "ros_logs" / f"kp_{kp:g}_repeat_{repeat}"),
                    },
                )
                try:
                    return_code = process.wait(timeout=float(trial_timeout_s))
                except subprocess.TimeoutExpired:
                    _stop_process_group(process)
                    return_code = -signal.SIGTERM
                except KeyboardInterrupt:
                    _stop_process_group(process)
                    raise
                finally:
                    if _process_group_exists(process):
                        _stop_process_group(process)
            summary_path = run_dir / "summary.json"
            summary = {}
            if summary_path.is_file():
                try:
                    summary = json.loads(summary_path.read_text(encoding="utf-8"))
                except (OSError, ValueError, TypeError):
                    summary = {}
            entries.append({
                "kp": kp,
                "repeat": repeat,
                "return_code": int(return_code),
                "passed": bool(return_code == 0 and summary.get("static_hold_passed", False)),
                "summary": summary,
                "run_dir": str(run_dir),
            })
    if not dry_run:
        write_sweep_summary_csv(output_root / "sweep_summary.csv", entries)
    return {
        "mode": "static_hold",
        "kps": [float(value) for value in kps],
        "repeats": int(repeats),
        "entries": entries,
        "commands": commands,
        "sweep_summary_csv": str(output_root / "sweep_summary.csv"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-pid-config", required=True)
    parser.add_argument("--robot-xacro", required=True)
    parser.add_argument("--qualification-config")
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--kps", nargs="+", type=float, default=None)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--trial-timeout-s", type=float, default=90.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--mode", choices=("loaded_gripper", "static_hold"), default="loaded_gripper",
        help="qualification mode; static_hold uses a free external cube and no lift",
    )
    parser.add_argument(
        "--diagnosis-config",
        help="static_grasp_hold_diagnosis.yaml used when --mode static_hold",
    )
    args = parser.parse_args(argv)
    if args.mode == "static_hold":
        if not args.diagnosis_config:
            parser.error("--diagnosis-config is required with --mode static_hold")
        summary = run_static_hold_sweep(
            source_pid_config=args.source_pid_config,
            robot_xacro=args.robot_xacro,
            diagnosis_config=args.diagnosis_config,
            output_root=args.output_root,
            kps=args.kps or STATIC_HOLD_DEFAULT_KPS,
            repeats=args.repeats,
            dry_run=args.dry_run,
            trial_timeout_s=args.trial_timeout_s,
        )
    else:
        if not args.qualification_config:
            parser.error("--qualification-config is required with --mode loaded_gripper")
        summary = run_sweep(
            source_pid_config=args.source_pid_config,
            robot_xacro=args.robot_xacro,
            qualification_config=args.qualification_config,
            output_root=args.output_root,
            kps=args.kps or DEFAULT_KPS,
            repeats=args.repeats,
            dry_run=args.dry_run,
            trial_timeout_s=args.trial_timeout_s,
        )
    Path(args.output_root, "sweep_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.dry_run:
        return 0
    if args.mode == "static_hold":
        return 0 if any(entry.get("passed") for entry in summary["entries"]) else 1
    return 0 if summary["selected_kp"] is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
