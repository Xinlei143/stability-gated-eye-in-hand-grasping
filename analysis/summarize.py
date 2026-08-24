"""Summarize a Stage 6 campaign into analysis artifacts."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

from .benchmark_io import (
    RunRecord,
    bootstrap_mean_ci,
    load_campaign_runs,
    paired_differences,
    write_csv,
)


METRICS = ("tracking_rms_error_m", "target_error_at_ready_m", "time_to_ready_s", "gate_resets")


def default_output_dir(campaign_dir: str | Path) -> Path:
    root = Path(campaign_dir)
    return root.parent / f"{root.name}-analysis"


def _run_row(record: RunRecord) -> dict[str, Any]:
    row: dict[str, Any] = {
        "run_id": record.run_id,
        "pair_id": record.pair_id,
        "method": record.method,
        "trajectory": record.trajectory,
        "seed": record.seed,
        "trial_success": record.metrics.get("trial_success", False),
        "task_success": record.metrics.get("task_success", False),
        "outcome": record.metrics.get("outcome", ""),
        "failure_class": record.metrics.get("failure_class", ""),
        "failure_stage": record.metrics.get("failure_stage", ""),
        "failure_reason": record.metrics.get("failure_reason", ""),
    }
    for key in sorted(record.condition):
        value = record.condition[key]
        if isinstance(value, (str, int, float, bool)) or value is None:
            row[f"condition.{key}"] = value
    for metric in METRICS:
        row[metric] = record.metrics.get(metric)
    return row


def _group_summary(records: list[RunRecord]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[RunRecord]] = defaultdict(list)
    for record in records:
        groups[(record.method, record.trajectory)].append(record)
    rows = []
    for (method, trajectory), group in sorted(groups.items()):
        row: dict[str, Any] = {
            "method": method,
            "trajectory": trajectory,
            "count": len(group),
            "trial_success_rate": sum(bool(item.metrics.get("trial_success")) for item in group) / len(group),
            "task_success_rate": sum(bool(item.metrics.get("task_success")) for item in group) / len(group),
        }
        for metric in METRICS:
            mean, lower, upper = bootstrap_mean_ci(
                [value for item in group if (value := item.value(metric)) is not None]
            )
            row[f"{metric}.mean"] = mean
            row[f"{metric}.ci_low"] = lower
            row[f"{metric}.ci_high"] = upper
        rows.append(row)
    return rows


def summarize_campaign(campaign_dir: str | Path, output_dir: str | Path | None = None) -> Path:
    root = Path(campaign_dir).resolve()
    output = Path(output_dir).resolve() if output_dir else default_output_dir(root)
    if output == root or root in output.parents:
        raise ValueError("analysis output must be separate from the raw campaign")
    output.mkdir(parents=True, exist_ok=True)
    records, exclusions = load_campaign_runs(root)
    run_rows = [_run_row(record) for record in records]
    run_fields = sorted({key for row in run_rows for key in row}) if run_rows else ["run_id"]
    write_csv(output / "run_metrics.csv", run_rows, run_fields)
    group_rows = _group_summary(records)
    group_fields = sorted({key for row in group_rows for key in row}) if group_rows else ["method", "trajectory", "count"]
    write_csv(output / "group_summary.csv", group_rows, group_fields)
    exclusion_fields = ["run_id", "reason"]
    write_csv(output / "excluded_runs.csv", exclusions, exclusion_fields)

    paired_rows: list[dict[str, Any]] = []
    for metric in METRICS:
        for row in paired_differences(records, metric=metric):
            for key, value in list(row.items()):
                if key != "pair_id":
                    paired_rows.append({"metric": metric, "pair_id": row["pair_id"], "comparison": key, "difference": value})
    write_csv(output / "paired_differences.csv", paired_rows, ["metric", "pair_id", "comparison", "difference"])
    (output / "analysis_manifest.json").write_text(
        json.dumps({"campaign": str(root), "run_count": len(records), "excluded_count": len(exclusions), "seed": 2026}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    print(summarize_campaign(args.campaign, args.output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
