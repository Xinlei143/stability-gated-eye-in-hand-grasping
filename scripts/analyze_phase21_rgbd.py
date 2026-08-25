#!/usr/bin/env python3
"""Analyze the independent Phase 20 RGB-D campaign for Phase 21."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path


METHODS = ("snapshot", "tracking", "gated")
SCENARIOS = ("static", "move_stop")


def _bool(value: object) -> bool:
    return value is True or str(value).lower() == "true"


def _ratio(value: int, total: int) -> str:
    return f"{value}/{total}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
    )
    args = parser.parse_args()
    output_dir = args.output_dir or args.campaign / "analysis"
    output_dir.mkdir(parents=True, exist_ok=True)

    with (args.campaign / "trials.csv").open(newline="") as handle:
        trial_rows = list(csv.DictReader(handle))
    if len(trial_rows) != 30:
        raise SystemExit(f"expected the independent 30-trial RGB-D campaign, got {len(trial_rows)} rows")
    if any(row["status"] != "finished" for row in trial_rows):
        raise SystemExit("canonical RGB-D campaign contains a non-finished row")

    records: list[dict[str, object]] = []
    for row in trial_rows:
        metrics = json.loads((Path(row["result_path"]) / "metrics.json").read_text())
        reason = str(metrics.get("failure_reason", ""))
        fresh_low_drift = (
            row["method"] == "tracking"
            and not _bool(metrics.get("task_success"))
            and "fresh, low-drift PREGRASP" in reason
        )
        records.append(
            {
                "method": row["method"],
                "scenario": row["scenario"],
                "seed": int(row["seed"]),
                "task_success": _bool(metrics.get("task_success")),
                "planning_success": _bool(metrics.get("planning_success")),
                "physical_grasp_success": _bool(metrics.get("physical_grasp_success")),
                "failure_stage": str(metrics.get("failure_stage", "")),
                "failure_reason": reason,
                "tracking_rms_error_m": metrics.get("tracking_rms_error_m"),
                "fresh_low_drift_pregrasp_failure": fresh_low_drift,
            }
        )

    trial_path = output_dir / "phase21_rgbd_trial_level.csv"
    with trial_path.open("w", newline="") as handle:
        fields = [
            "method", "scenario", "seed", "task_success", "planning_success",
            "physical_grasp_success", "failure_stage", "failure_reason",
            "tracking_rms_error_m", "fresh_low_drift_pregrasp_failure",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    groups: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    for record in records:
        groups[(str(record["method"]), str(record["scenario"]))].append(record)

    summary_path = output_dir / "phase21_rgbd_summary.csv"
    with summary_path.open("w", newline="") as handle:
        fields = [
            "method", "scenario", "n", "task_success", "planning_success",
            "physical_grasp_success", "failure_stage",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for method in METHODS:
            for scenario in SCENARIOS:
                group = groups[(method, scenario)]
                failure_stages = Counter(str(item["failure_stage"]) for item in group if not item["task_success"])
                stage_text = "; ".join(
                    f"{stage or 'unspecified'}×{count}" for stage, count in sorted(failure_stages.items())
                ) or "—"
                writer.writerow(
                    {
                        "method": method,
                        "scenario": scenario,
                        "n": len(group),
                        "task_success": _ratio(sum(bool(item["task_success"]) for item in group), len(group)),
                        "planning_success": _ratio(sum(bool(item["planning_success"]) for item in group), len(group)),
                        "physical_grasp_success": _ratio(sum(bool(item["physical_grasp_success"]) for item in group), len(group)),
                        "failure_stage": stage_text,
                    }
                )

    tracking_diagnostics: dict[str, dict[str, object]] = {}
    for scenario in SCENARIOS:
        group = groups[("tracking", scenario)]
        failed = [item for item in group if not item["task_success"]]
        reasons = Counter(str(item["failure_reason"]) for item in failed)
        rms = [float(item["tracking_rms_error_m"]) for item in failed if item["tracking_rms_error_m"] is not None]
        tracking_diagnostics[scenario] = {
            "failed": len(failed),
            "fresh_low_drift": sum(bool(item["fresh_low_drift_pregrasp_failure"]) for item in failed),
            "reason_counts": reasons,
            "rms_median": statistics.median(rms) if rms else None,
            "rms_min": min(rms) if rms else None,
            "rms_max": max(rms) if rms else None,
        }

    report_path = output_dir / "phase21_rgbd_report.md"
    lines = [
        "# Phase 21: RGB-D result analysis",
        "",
        "This analysis uses only the canonical 30-trial Phase 20 RGB-D campaign. It does not include the 120-trial controlled ground-truth benchmark.",
        "",
        "## Task-success matrix",
        "",
        "| Method | Static | Move-stop |",
        "| --- | ---: | ---: |",
    ]
    for method in METHODS:
        cells = []
        for scenario in SCENARIOS:
            group = groups[(method, scenario)]
            cells.append(_ratio(sum(bool(item["task_success"]) for item in group), len(group)))
        lines.append(f"| {method.capitalize()} | {cells[0]} | {cells[1]} |")
    lines.extend([
        "",
        "## Detailed outcome table",
        "",
        "| Method | Scenario | Task success | Planning success | Physical success | Failure stage |",
        "| --- | --- | ---: | ---: | ---: | --- |",
    ])
    for method in METHODS:
        for scenario in SCENARIOS:
            group = groups[(method, scenario)]
            failure_stages = Counter(str(item["failure_stage"]) for item in group if not item["task_success"])
            stage_text = "; ".join(f"{stage or 'unspecified'}×{count}" for stage, count in sorted(failure_stages.items())) or "—"
            lines.append(
                f"| {method.capitalize()} | {scenario} | "
                f"{_ratio(sum(bool(item['task_success']) for item in group), len(group))} | "
                f"{_ratio(sum(bool(item['planning_success']) for item in group), len(group))} | "
                f"{_ratio(sum(bool(item['physical_grasp_success']) for item in group), len(group))} | {stage_text} |"
            )
    lines.extend(["", "## Tracking failure mechanism", ""])
    for scenario in SCENARIOS:
        diagnostic = tracking_diagnostics[scenario]
        lines.append(
            f"- `{scenario}`: {diagnostic['failed']}/5 task failures; "
            f"{diagnostic['fresh_low_drift']}/{diagnostic['failed']} explicitly report "
            "`fresh, low-drift PREGRASP` commitment failure."
        )
        if diagnostic["rms_median"] is not None:
            lines.append(
                f"  - tracking RMS error: median `{diagnostic['rms_median']:.6f} m`, "
                f"range `{diagnostic['rms_min']:.6f}–{diagnostic['rms_max']:.6f} m`."
            )
        for reason, count in diagnostic["reason_counts"].items():
            lines.append(f"  - ×{count}: `{reason}`")
    lines.extend([
        "",
        "## Interpretation",
        "",
        "Tracking failed in all 5 static and all 5 move-stop RGB-D repeats, and every failure carried the same fresh/low-drift PREGRASP reason. This is therefore a repeatable end-to-end RGB-D robustness result in this frozen configuration, not an isolated smoke failure.",
        "The recorded `failure_stage=planning` is a task-level method failure, not an infrastructure failure: the canonical trials finished normally with complete artifacts. The separate `planning_success` metric remains reported exactly as recorded and should not be substituted for task success.",
    ])
    report_path.write_text("\n".join(lines) + "\n")

    print(f"wrote {trial_path}")
    print(f"wrote {summary_path}")
    print(f"wrote {report_path}")
    for method in METHODS:
        print(method, [
            _ratio(sum(bool(item["task_success"]) for item in groups[(method, scenario)]), len(groups[(method, scenario)]))
            for scenario in SCENARIOS
        ])
    for scenario, diagnostic in tracking_diagnostics.items():
        print("tracking", scenario, "fresh_low_drift", diagnostic["fresh_low_drift"], "/", diagnostic["failed"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
