#!/usr/bin/env python3
"""Run the frozen Phase 20 RGB-D repeat matrix.

This keeps the Phase 16 campaign supervisor and artifact semantics, while
selecting ``full_pipeline.launch.py`` so that the real simulated RGB-D chain
(segmentation, depth fusion, and camera-to-base) is present for every trial.
Only infrastructure failures are eligible for rerun via ``--rerun-failed``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from foam_grasp_sim.benchmark_suite import expand_suite
from foam_grasp_sim.experiment_runner import CampaignRunner


REPO_ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = REPO_ROOT / "runtime" / "models" / "best_model.pth"
PHYSICS_XACRO = (
    REPO_ROOT
    / "workspaces"
    / "app_ws"
    / "src"
    / "foam_grasp_sim"
    / "urdf"
    / "piper_eye_in_hand_physics.xacro"
)


FROZEN_SUITE: dict[str, Any] = {
    "schema_version": 1,
    "name": "phase20_rgbd_formal",
    "defaults": {
        "target_model": "cube",
        "execute_motion": True,
        "run_grasp_pipeline": True,
        "record_benchmark": True,
        "use_rviz": False,
        "grasp_assist_mode": "off",
        "grasp_assist_service": "",
        "grasp_stabilization_mode": "gazebo_grasp_fix",
        "perception_source": "rgbd",
        "velocity_x": 0.01,
        "velocity_y": 0.0,
        "velocity_z": 0.0,
        "move_duration": 4.0,
        "stop_duration": 6.0,
        "stability_duration": 5.0,
        "position_spread_threshold": 0.006,
        "minimum_stable_samples": 25,
        "observation_timeout": 1.0,
        "timeout_s": 150.0,
    },
    "methods": ["snapshot", "tracking", "gated"],
    "trajectories": ["static", "move_stop"],
    "seeds": [42, 43, 44, 45, 46],
    "sweeps": [],
}


def _replace_arg(args: list[str], name: str, value: str) -> None:
    prefix = f"{name}:="
    for index, argument in enumerate(args):
        if argument.startswith(prefix):
            args[index] = prefix + value
            return
    args.append(prefix + value)


def _rgbd_specs():
    specs = expand_suite(FROZEN_SUITE)
    if len(specs) != 30:
        raise RuntimeError(f"expected 30 RGB-D specs, got {len(specs)}")
    if any(spec.resolved.get("perception_source") != "rgbd" for spec in specs):
        raise RuntimeError("RGB-D suite lost perception_source=rgbd")
    if any(not spec.execute_motion for spec in specs):
        raise RuntimeError("Phase 20 requires execute_motion=true")

    output = []
    for spec in specs:
        args = list(spec.launch_args)
        if args[:4] != ["ros2", "launch", "foam_grasp_sim", "sim_bringup.launch.py"]:
            raise RuntimeError(f"unexpected base launch command: {args[:4]}")
        args[3] = "full_pipeline.launch.py"
        _replace_arg(args, "checkpoint", str(CHECKPOINT))
        _replace_arg(args, "require_cuda", "true")
        _replace_arg(args, "tf_timeout", "0.2")
        _replace_arg(args, "start_moveit", "true")
        _replace_arg(args, "prepare_observation_pose", "true")
        _replace_arg(args, "robot_xacro", str(PHYSICS_XACRO))
        _replace_arg(args, "gazebo_executable", "gzserver")
        output.append(replace(spec, launch_args=tuple(args)))
    return output


def _suite_hash() -> str:
    canonical = json.dumps(FROZEN_SUITE, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canonical.encode()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-root", type=Path, default=REPO_ROOT / "results")
    parser.add_argument(
        "--campaign-id",
        default="phase20_rgbd_formal-20260825-seeds42-46",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--rerun-failed", action="store_true")
    parser.add_argument("--max-trials", type=int)
    args = parser.parse_args()
    if (args.resume or args.rerun_failed) and not args.campaign_id:
        parser.error("--resume/--rerun-failed require --campaign-id")
    if not CHECKPOINT.is_file():
        raise SystemExit(f"missing frozen segmentation checkpoint: {CHECKPOINT}")
    if not PHYSICS_XACRO.is_file():
        raise SystemExit(f"missing frozen eye-in-hand xacro: {PHYSICS_XACRO}")

    specs = _rgbd_specs()
    campaign_dir = args.results_root / args.campaign_id
    if campaign_dir.exists() and not (args.resume or args.rerun_failed or args.dry_run):
        raise SystemExit(f"campaign exists; use --resume or --rerun-failed: {campaign_dir}")

    print("Phase 20 frozen RGB-D matrix")
    print("  target_model=cube perception_source=rgbd execute_motion=true")
    print("  methods=snapshot,tracking,gated scenarios=static,move_stop seeds=42,43,44,45,46")
    print("  camera=640x480@30Hz segmentation_input=640x360")
    print("  grasp_offset_x=0.015 grasp_offset_y=0.0")
    print("  checkpoint=", CHECKPOINT)
    print("  stabilization=gazebo_grasp_fix")
    print("  trial_count=", len(specs))

    runner = CampaignRunner(
        specs,
        campaign_dir,
        timeout_s=150.0,
        logger_flush_grace_s=2.0,
        ros_domain_id=93,
        gazebo_master_uri="http://127.0.0.1:12193",
    )
    runner.run(
        suite_name=FROZEN_SUITE["name"],
        suite_hash=_suite_hash(),
        dry_run=args.dry_run,
        resume=args.resume,
        rerun_failed=args.rerun_failed,
        max_trials=args.max_trials,
    )
    print("campaign=", campaign_dir)
    print("finished_at=", datetime.now(timezone.utc).isoformat(timespec="seconds"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
