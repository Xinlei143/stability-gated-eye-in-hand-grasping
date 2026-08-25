#!/usr/bin/env python3
"""Audit and summarize the canonical Phase 20 RGB-D campaign."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


REQUIRED_ARTIFACTS = ("metadata.json", "states.csv", "events.csv", "metrics.json")
FROZEN_LOG_MARKERS = {
    "segmentation_input": "Warming up GPU with input size 640x360",
    "depth_profile": "size=640x480",
    "grasp_offset": "dx=0.015 m, dy=0.000 m",
}


def _bool(value: object) -> bool:
    return value is True or str(value).lower() == "true"


def _read_events(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign", type=Path)
    args = parser.parse_args()
    root = args.campaign
    with (root / "trials.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))

    errors: list[str] = []
    audit_rows: list[dict[str, object]] = []
    for row in rows:
        run_id = row["run_id"]
        result_path = Path(row["result_path"])
        log_path = Path(row["log_path"])
        if row["status"] != "finished":
            errors.append(f"{run_id}: canonical status={row['status']}")
            continue
        missing = [name for name in REQUIRED_ARTIFACTS if not (result_path / name).is_file()]
        if missing:
            errors.append(f"{run_id}: missing {','.join(missing)}")
            continue
        try:
            metrics = json.loads((result_path / "metrics.json").read_text())
            events = _read_events(result_path / "events.csv")
            terminal = [event for event in events if event.get("event") in {"TRIAL_FINISHED", "TRIAL_FAILED"}]
            if len(terminal) != 1 or terminal[0].get("event") != "TRIAL_FINISHED":
                errors.append(f"{run_id}: terminal event count/type invalid")
            details = json.loads(terminal[0].get("details", "{}")) if terminal else {}
            task_success = _bool(metrics.get("task_success"))
            if details.get("task_success") is not task_success:
                errors.append(f"{run_id}: events/metrics task_success mismatch")
            if task_success and metrics.get("physical_grasp_success") is not True:
                errors.append(f"{run_id}: task_success without physical_grasp_success")
            if row["task_success"] != str(task_success).lower():
                errors.append(f"{run_id}: trials.csv task_success mismatch")
            if row["terminal_event"] != "TRIAL_FINISHED" or row["artifacts_complete"] != "true":
                errors.append(f"{run_id}: trials.csv terminal/artifact fields invalid")
            log_text = log_path.read_text(errors="replace")
            for marker_name, marker in FROZEN_LOG_MARKERS.items():
                if marker not in log_text:
                    errors.append(f"{run_id}: missing log marker {marker_name}")
            audit_rows.append({
                "method": row["method"],
                "scenario": row["scenario"],
                "seed": row["seed"],
                "task_success": task_success,
                "planning_success": _bool(metrics.get("planning_success")),
                "physical_grasp_success": _bool(metrics.get("physical_grasp_success")),
                "failure_stage": metrics.get("failure_stage", ""),
                "failure_reason": metrics.get("failure_reason", ""),
                "attempt": row["attempt"],
            })
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            errors.append(f"{run_id}: audit exception {error}")

    summary: dict[tuple[str, str], dict[str, int]] = defaultdict(lambda: {"n": 0, "task_success": 0, "planning_success": 0, "physical_grasp_success": 0})
    for row in audit_rows:
        group = summary[(str(row["method"]), str(row["scenario"]))]
        group["n"] += 1
        for field in ("task_success", "planning_success", "physical_grasp_success"):
            group[field] += int(bool(row[field]))

    summary_path = root / "phase20_summary.csv"
    with summary_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["method", "scenario", "n", "task_success", "planning_success", "physical_grasp_success"])
        for (method, scenario), values in sorted(summary.items()):
            writer.writerow([method, scenario, values["n"], values["task_success"], values["planning_success"], values["physical_grasp_success"]])

    failure_counts = Counter(
        (str(row["method"]), str(row["scenario"]), str(row["failure_stage"]), str(row["failure_reason"]))
        for row in audit_rows
        if not row["task_success"]
    )
    reruns = []
    for row in rows:
        if row["attempt"] != "1":
            reruns.append({"run_id": row["run_id"], "attempt": row["attempt"], "result_path": row["result_path"]})

    report_path = root / "phase20_report.md"
    lines = [
        "# Phase 20 formal RGB-D repeat experiment",
        "",
        "## Frozen configuration",
        "",
        "- target: `cube`",
        "- perception: simulated RGB-D chain via `full_pipeline.launch.py`",
        "- methods: `snapshot`, `tracking`, `gated`",
        "- scenarios: `static`, `move_stop`",
        "- seeds: `42–46`",
        "- camera/depth profile: `640×480 @ 30 Hz`; segmentation input: `640×360`",
        "- grasp correction: `grasp_offset_x=0.015 m`, `grasp_offset_y=0.0 m`",
        "- stabilization: `gazebo_grasp_fix`; `execute_motion=true`",
        "",
        "## Campaign and audit",
        "",
        f"- Canonical rows: `{len(rows)}/30`",
        f"- Canonical finished: `{sum(row['status'] == 'finished' for row in rows)}/30`",
        f"- Audit errors: `{len(errors)}`",
        f"- Attempts: `{Counter(row['attempt'] for row in rows)}`",
        "- Task failures remain in the denominator; only infrastructure failures were rerun.",
        "",
        "## Task-level summary",
        "",
        "| method | scenario | n | task success | planning success | physical grasp success |",
        "| --- | --- | ---: | ---: | ---: | ---: |",
    ]
    for (method, scenario), values in sorted(summary.items()):
        lines.append(f"| {method} | {scenario} | {values['n']} | {values['task_success']} | {values['planning_success']} | {values['physical_grasp_success']} |")
    lines.extend(["", "## Failure mechanisms", ""])
    for (method, scenario, stage, reason), count in sorted(failure_counts.items()):
        lines.append(f"- `{method}/{scenario}` × {count}: stage `{stage}`; `{reason}`")
    lines.extend(["", "## Infrastructure reruns", ""])
    for rerun in reruns:
        lines.append(f"- `{rerun['run_id']}`: attempt `{rerun['attempt']}`; canonical artifact `{rerun['result_path']}`; attempt 1 remains under the corresponding `__attempt-001` run/log artifacts.")
    if not reruns:
        lines.append("- None.")
    lines.extend(["", "## Audit errors", ""])
    lines.extend(f"- {error}" for error in errors) if errors else lines.append("- None.")
    report_path.write_text("\n".join(lines) + "\n")

    print(f"rows={len(rows)} finished={sum(row['status'] == 'finished' for row in rows)} audit_errors={len(errors)}")
    for (method, scenario), values in sorted(summary.items()):
        print(method, scenario, values)
    print(f"wrote {summary_path}")
    print(f"wrote {report_path}")
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
