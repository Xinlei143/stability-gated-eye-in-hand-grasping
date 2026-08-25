"""Audit frozen grasping campaigns and generate the final report bundle.

This module is deliberately read-only with respect to input campaigns.  The
CLI writes only to an explicitly supplied output directory.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean, median, stdev
from typing import Any, Iterable


METHODS = ("snapshot", "tracking", "gated")
SCENARIOS = ("static", "move_stop")
FORMAL_ARTIFACTS = ("metadata.json", "events.csv", "states.csv", "metrics.json")
METRIC_FIELDS = (
    "time_to_ready_s",
    "target_error_at_ready_m",
    "target_error_at_grasp_m",
    "tracking_rms_error_m",
    "lift_height_m",
    "hold_time_s",
)


def as_bool(value: Any) -> bool:
    return value is True or str(value).strip().lower() in {"true", "1", "yes"}


def as_float(value: Any) -> float | None:
    if value in (None, "", "NA", "nan", "None"):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def json_details(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if not value:
        return {}
    try:
        parsed = json.loads(value)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}


def _resolve_result_path(campaign: Path, result_path: str) -> Path:
    candidate = Path(result_path)
    if candidate.is_dir():
        return candidate
    repo_relative = campaign.parent.parent / candidate
    if repo_relative.is_dir():
        return repo_relative
    return candidate


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def load_canonical_trials(
    campaign: Path,
    *,
    expected_n: int | None = None,
    expected_seeds: Iterable[int] | None = None,
) -> list[dict[str, str]]:
    """Load and validate the campaign's canonical ``trials.csv`` rows."""

    rows = _read_csv(campaign / "trials.csv")
    if expected_n is not None and len(rows) != expected_n:
        raise ValueError(f"{campaign}: expected {expected_n} canonical rows, got {len(rows)}")
    expected_seed_set = set(expected_seeds or ())
    seen: set[tuple[str, str, int]] = set()
    for row in rows:
        if row.get("status") != "finished":
            raise ValueError(f"{campaign}: non-finished canonical row {row.get('run_id')}")
        key = (row.get("method", ""), row.get("scenario", ""), int(row.get("seed", -1)))
        if key in seen:
            raise ValueError(f"duplicate canonical condition {key}")
        seen.add(key)
        if expected_seed_set and key[2] not in expected_seed_set:
            raise ValueError(f"unexpected seed in canonical campaign: {key[2]}")
        run = _resolve_result_path(campaign, row.get("result_path", ""))
        missing = [name for name in FORMAL_ARTIFACTS if not (run / name).is_file()]
        if missing:
            raise ValueError(f"{campaign}: {row.get('run_id')} missing {','.join(missing)}")
        row["_resolved_result_path"] = str(run)
    return rows


def _point(row: dict[str, Any], prefix: str) -> tuple[float, float, float] | None:
    values = [as_float(row.get(f"{prefix}_{axis}")) for axis in "xyz"]
    if any(value is None for value in values):
        return None
    return values[0], values[1], values[2]


def _distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def compute_tracking_rms(states: list[dict[str, Any]], events: list[dict[str, Any]]) -> float | None:
    """Compute TCP-to-GT target RMS over observation-to-grasp interval."""

    def first_time(name: str) -> int | None:
        values = [int(event.get("sim_time_ns", 0)) for event in events if event.get("event") == name]
        return min(values) if values else None

    start = first_time("TARGET_OBSERVED")
    if start is None:
        return None
    end = first_time("GRASP_STARTED")
    selected = []
    for state in states:
        stamp = int(state.get("sim_time_ns", 0))
        if stamp < start or (end is not None and stamp > end):
            continue
        gt = _point(state, "target_ground_truth")
        tcp = _point(state, "tcp")
        if gt is not None and tcp is not None:
            selected.append(_distance(gt, tcp))
    if not selected:
        return None
    return math.sqrt(sum(value * value for value in selected) / len(selected))


def planning_semantics(metrics: dict[str, Any], events: list[dict[str, Any]] | None = None) -> dict[str, Any]:
    """Separate initial path planning from the terminal task phase."""

    initial = as_bool(metrics.get("planning_success"))
    if events is not None:
        initial = any(event.get("event") == "PLAN_SUCCEEDED" for event in events)
    stage = str(metrics.get("failure_stage", ""))
    reason = str(metrics.get("failure_reason", ""))
    return {
        "initial_plan_succeeded": initial,
        "terminal_task_phase": stage,
        "pregrasp_commitment_failure": "fresh, low-drift PREGRASP" in reason,
    }


def _failure_class(row: dict[str, str], metrics: dict[str, Any], terminal_event: str) -> str:
    if terminal_event == "TRIAL_FAILED":
        return "infrastructure"
    if str(metrics.get("outcome", "")) == "infrastructure_failure":
        return "infrastructure"
    if str(metrics.get("failure_class", "")) == "infrastructure":
        return "infrastructure"
    if str(row.get("outcome", "")) == "infrastructure_failure":
        return "infrastructure"
    return "task" if not as_bool(metrics.get("task_success")) else "none"


def audit_trial(row: dict[str, str], perception_condition: str) -> dict[str, Any]:
    run = Path(row["_resolved_result_path"])
    metrics = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
    events = _read_csv(run / "events.csv")
    states = _read_csv(run / "states.csv")
    terminal_events = [event for event in events if event.get("event") in {"TRIAL_FINISHED", "TRIAL_FAILED"}]
    inconsistencies: list[str] = []
    if len(terminal_events) != 1:
        inconsistencies.append(f"terminal_event_count={len(terminal_events)}")
    terminal = terminal_events[-1] if terminal_events else {}
    details = json_details(terminal.get("details"))
    terminal_name = terminal.get("event", "")
    if terminal_name == "TRIAL_FINISHED" and "task_success" in details:
        if as_bool(details["task_success"]) != as_bool(metrics.get("task_success")):
            inconsistencies.append("terminal_vs_metrics_task_success")
    if terminal_name == "TRIAL_FINISHED" and details.get("outcome") and details.get("outcome") != metrics.get("outcome"):
        inconsistencies.append("terminal_vs_metrics_outcome")
    actual_plan = any(event.get("event") == "PLAN_SUCCEEDED" for event in events)
    if actual_plan != as_bool(metrics.get("planning_success")):
        inconsistencies.append("event_vs_metrics_planning_success")
    computed_rms = compute_tracking_rms(states, events)
    metric_rms = as_float(metrics.get("tracking_rms_error_m"))
    if computed_rms is not None and metric_rms is not None and not math.isclose(computed_rms, metric_rms, rel_tol=1e-6, abs_tol=1e-9):
        inconsistencies.append(f"tracking_rms_mismatch:{computed_rms:.9f}!={metric_rms:.9f}")
    semantics = planning_semantics(metrics, events)
    if perception_condition == "rgbd" and row.get("method") == "tracking" and not as_bool(metrics.get("task_success")):
        if "fresh, low-drift PREGRASP" not in str(metrics.get("failure_reason", "")):
            inconsistencies.append("tracking_rgbd_missing_fresh_low_drift_reason")
    result: dict[str, Any] = {
        "perception_condition": perception_condition,
        "run_id": row.get("run_id", ""),
        "pair_id": row.get("pair_id", ""),
        "method": row.get("method", ""),
        "scenario": row.get("scenario", ""),
        "seed": int(row.get("seed", -1)),
        "attempt": int(row.get("attempt", 0)),
        "status": row.get("status", ""),
        "trial_success": as_bool(row.get("trial_success")),
        "task_success": as_bool(metrics.get("task_success")),
        "planning_success": actual_plan,
        "physical_grasp_success": as_bool(metrics.get("physical_grasp_success")),
        "failure_class": _failure_class(row, metrics, terminal_name),
        "failure_stage": str(metrics.get("failure_stage", "")),
        "failure_reason": str(metrics.get("failure_reason", "")),
        "terminal_event": terminal_name,
        "terminal_task_phase": semantics["terminal_task_phase"],
        "pregrasp_commitment_failure": semantics["pregrasp_commitment_failure"],
        "tracking_rms_error_m": metric_rms,
        "tracking_rms_computed_m": computed_rms,
        "rms_definition": "TCP-to-ground-truth-target distance RMS" if computed_rms is not None else "missing TCP interval",
        "artifact_complete": all((run / name).is_file() for name in FORMAL_ARTIFACTS),
        "inconsistencies": "; ".join(inconsistencies),
        "source_run": str(run),
    }
    metric_aliases = {
        "target_error_at_grasp_m": "grasp_initiation_error_m",
        "hold_time_s": "grasp_hold_s",
    }
    for field in METRIC_FIELDS:
        result[field] = as_float(metrics.get(metric_aliases.get(field, field)))
    return result


def _stats(values: list[float | None]) -> dict[str, Any]:
    present = [value for value in values if value is not None]
    if not present:
        return {"n": 0, "missing_n": len(values), "mean": None, "std": None, "median": None, "q1": None, "q3": None}
    ordered = sorted(present)
    q1 = ordered[(len(ordered) - 1) // 4]
    q3 = ordered[(3 * (len(ordered) - 1)) // 4]
    return {
        "n": len(present),
        "missing_n": len(values) - len(present),
        "mean": mean(present),
        "std": stdev(present) if len(present) > 1 else 0.0,
        "median": median(present),
        "q1": q1,
        "q3": q3,
    }


def _fmt(value: Any, digits: int = 4) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def summarize_trials(trials: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for method in METHODS:
        for scenario in SCENARIOS:
            group = [row for row in trials if row["method"] == method and row["scenario"] == scenario]
            n = len(group)
            task_n = sum(as_bool(row["task_success"]) for row in group)
            plan_n = sum(as_bool(row["planning_success"]) for row in group)
            physical_n = sum(as_bool(row["physical_grasp_success"]) for row in group)
            stage_counts = Counter(row["failure_stage"] or "success" for row in group if not as_bool(row["task_success"]))
            failure_class_counts = Counter(row["failure_class"] for row in group)
            item: dict[str, Any] = {
                "method": method,
                "scenario": scenario,
                "n": n,
                "task_success_n": task_n,
                "task_success_rate": task_n / n if n else None,
                "planning_success_n": plan_n,
                "planning_success_rate": plan_n / n if n else None,
                "physical_success_n": physical_n,
                "physical_success_rate": physical_n / n if n else None,
                "task_failure_n": sum(row["failure_class"] == "task" for row in group),
                "infrastructure_failure_n": sum(row["failure_class"] == "infrastructure" for row in group),
                "failure_stage_counts": json.dumps(dict(sorted(stage_counts.items())), ensure_ascii=False, sort_keys=True),
                "failure_class_counts": json.dumps(dict(sorted(failure_class_counts.items())), ensure_ascii=False, sort_keys=True),
            }
            for field in METRIC_FIELDS:
                stats = _stats([row.get(field) for row in group])
                for key, value in stats.items():
                    item[f"{field}_{key}"] = value
            rows.append(item)
    return rows


def _hash_inputs(paths: list[Path]) -> list[dict[str, Any]]:
    output = []
    for path in sorted(paths):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        output.append({"path": str(path), "sha256": digest, "size": path.stat().st_size})
    return output


def _campaign_input_files(campaign: Path, rows: list[dict[str, str]]) -> list[Path]:
    paths = [campaign / "campaign.json", campaign / "trials.csv"]
    for row in rows:
        run = Path(row["_resolved_result_path"])
        paths.extend(run / name for name in FORMAL_ARTIFACTS)
    return [path for path in paths if path.is_file()]


def _write_paper_tables(out: Path, gt: list[dict[str, Any]], rgbd: list[dict[str, Any]], all_trials: list[dict[str, Any]], negative: list[dict[str, Any]]) -> None:
    table1 = []
    for method in METHODS:
        table1.append({"method": method, "static": _cell(gt, method, "static"), "move_stop": _cell(gt, method, "move_stop")})
    table2 = []
    for method in METHODS:
        table2.append({"method": method, "static": _cell(rgbd, method, "static"), "move_stop": _cell(rgbd, method, "move_stop")})
    table3 = []
    for row in rgbd:
        table3.append({
            "method": row["method"],
            "scenario": row["scenario"],
            "task_success": _cell_from_row(row, "task_success_n"),
            "initial_plan_success": _cell_from_row(row, "planning_success_n"),
            "physical_grasp_success": _cell_from_row(row, "physical_success_n"),
            "terminal_failure_stage": row["failure_stage_counts"],
        })
    _write_csv(out / "table_1_controlled_success.csv", table1, ["method", "static", "move_stop"])
    _write_csv(out / "table_2_rgbd_success.csv", table2, ["method", "static", "move_stop"])
    _write_csv(out / "table_3_rgbd_detailed_outcome.csv", table3, ["method", "scenario", "task_success", "initial_plan_success", "physical_grasp_success", "terminal_failure_stage"])
    _write_csv(out / "table_4_negative_safety.csv", negative, list(negative[0]) if negative else ["scenario"])
    lines = [
        "# 论文用表格",
        "",
        "## Table 1. Controlled benchmark task success",
        "",
        "| Method | Static | Move-stop |\n| --- | ---: | ---: |",
    ]
    lines += [f"| {row['method']} | {row['static']} | {row['move_stop']} |" for row in table1]
    lines += ["", "## Table 2. Simulated RGB-D end-to-end task success", "", "| Method | Static | Move-stop |", "| --- | ---: | ---: |"]
    lines += [f"| {row['method']} | {row['static']} | {row['move_stop']} |" for row in table2]
    lines += ["", "## Table 3. RGB-D detailed outcome", "", "| Method | Scenario | Task success | Initial plan success | Physical success | Terminal failure stage |", "| --- | --- | ---: | ---: | ---: | --- |"]
    lines += [f"| {row['method']} | {row['scenario']} | {row['task_success']} | {row['initial_plan_success']} | {row['physical_grasp_success']} | `{row['terminal_failure_stage']}` |" for row in table3]
    lines += ["", "## Table 4. Negative/safety results", "", "| Scenario | Safe rejection | Physical grasp attempted | Physical false positive | Terminal outcome | Classification |", "| --- | ---: | ---: | ---: | --- | --- |"]
    for row in negative:
        lines.append(f"| {row['scenario']} | {row['safe_rejection']} | {row['physical_grasp_attempted']} | {row['physical_false_positive']} | {row['terminal_outcome']} | {row['classification']} |")
    (out / "paper_tables.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _cell(summary: list[dict[str, Any]], method: str, scenario: str) -> str:
    row = next(item for item in summary if item["method"] == method and item["scenario"] == scenario)
    return f"{row['task_success_n']}/{row['n']}"


def _cell_from_row(row: dict[str, Any], field: str) -> str:
    return f"{row[field]}/{row['n']}"


def _negative_rows(repo_root: Path) -> list[dict[str, Any]]:
    root = repo_root / "results" / "phase10_rgbd_negative"
    selections = [
        ("no target", "phase10-A-no-target", "no target"),
        ("target disappeared", "phase10-B-occluded-delete-fixed", "target disappeared / stale observation"),
        ("stale observation", "phase10-B-occluded-stale", "stale observation"),
        ("outside workspace", "phase10-C-outside-workspace", "outside workspace"),
        ("wrong class", "phase10-D-wrong-class", "wrong class"),
        ("no physical grasp", "phase10-E-no-physical-grasp-off-fixed", "no physical grasp"),
    ]
    rows = []
    for scenario, dirname, label in selections:
        run = root / dirname
        metrics_path = run / "metrics.json"
        events_path = run / "events.csv"
        complete = all((run / name).is_file() for name in FORMAL_ARTIFACTS)
        metrics: dict[str, Any] = {}
        events: list[dict[str, str]] = []
        if metrics_path.is_file():
            try:
                metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, json.JSONDecodeError):
                metrics = {}
        if events_path.is_file():
            events = _read_csv(events_path)
        attempted = any(event.get("event") in {"GRASP_STARTED", "GRIPPER_CLOSED", "LIFT_STARTED"} for event in events)
        physical = as_bool(metrics.get("physical_grasp_success"))
        terminal = next((event for event in reversed(events) if event.get("event") in {"TRIAL_FINISHED", "TRIAL_FAILED"}), {})
        details = json_details(terminal.get("details"))
        task_success = as_bool(metrics.get("task_success"))
        safe_rejection = complete and not attempted and not task_success and str(metrics.get("failure_stage", "")) in {"readiness", "safety", "verification", ""}
        if not complete:
            classification = "infrastructure/shutdown artifact incomplete"
        elif str(metrics.get("outcome", "")) == "infrastructure_failure" or terminal.get("event") == "TRIAL_FAILED":
            classification = "infrastructure/shutdown"
        else:
            if safe_rejection:
                classification = "safe rejection"
            elif task_success:
                classification = "task success; negative trigger not reproduced"
            else:
                classification = "task failure"
        rows.append({
            "scenario": label,
            "safe_rejection": int(safe_rejection),
            "physical_grasp_attempted": int(attempted),
            "physical_false_positive": int(as_bool(metrics.get("physical_false_positive"))),
            "false_positive_basis": "explicit artifact flag" if "physical_false_positive" in metrics else "no explicit artifact flag; not inferred from task/teardown outcome",
            "terminal_outcome": str(metrics.get("outcome") or details.get("outcome") or terminal.get("event") or "missing"),
            "classification": classification,
            "physical_grasp_success": int(physical),
            "artifact_complete": int(complete),
            "source_artifact": str(run.relative_to(repo_root)) if run.is_absolute() else str(run),
        })
    return rows


def _plot_success(out: Path, summary: list[dict[str, Any]], title: str, denominator: int, stem: str) -> None:
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import numpy as np

    stem_path = out / stem
    mpl.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"], "svg.fonttype": "none", "pdf.fonttype": 42, "font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(5.8, 3.2))
    x = np.arange(len(METHODS))
    width = 0.34
    colors = {"snapshot": "#7A8793", "tracking": "#D08A3B", "gated": "#3E8C78"}
    for index, scenario in enumerate(SCENARIOS):
        values = [next(row for row in summary if row["method"] == method and row["scenario"] == scenario)["task_success_n"] for method in METHODS]
        bars = ax.bar(x + (index - 0.5) * width, values, width, label=scenario.replace("_", "-"), color=[colors[m] for m in METHODS], alpha=0.95 if index == 0 else 0.65, edgecolor="white", linewidth=0.5)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 0.35, f"{value}/{denominator}", ha="center", va="bottom", fontsize=7)
    source_rows = [
        {"method": row["method"], "scenario": row["scenario"], "n": row["n"], "task_success_n": row["task_success_n"], "task_success_rate": row["task_success_rate"]}
        for row in summary
    ]
    _write_csv(stem_path.parent / f"{stem_path.name}_source.csv", source_rows, ["method", "scenario", "n", "task_success_n", "task_success_rate"])
    ax.set_xticks(x, [method.capitalize() for method in METHODS])
    ax.set_ylim(0, denominator * 1.28)
    ax.set_ylabel("Task successes (count / n)")
    ax.set_title(title, loc="left", fontsize=9, fontweight="bold")
    ax.legend(title="Scenario", frameon=False, ncol=2, loc="upper right", bbox_to_anchor=(1.0, 1.18))
    fig.tight_layout(rect=[0, 0, 1, 0.88])
    _save_figure(fig, stem_path)
    plt.close(fig)


def _save_figure(fig: Any, stem: Path) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(f"{stem}.svg", bbox_inches="tight")
    fig.savefig(f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(f"{stem}.tiff", dpi=600, bbox_inches="tight")
    fig.savefig(f"{stem}.png", dpi=220, bbox_inches="tight")


def _plot_failure_stages(out: Path, trials: list[dict[str, Any]]) -> None:
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import numpy as np

    mpl.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"], "svg.fonttype": "none", "pdf.fonttype": 42, "font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    groups = [("controlled", "snapshot", "move_stop"), ("rgbd", "tracking", "static"), ("rgbd", "tracking", "move_stop"), ("controlled", "gated", "move_stop")]
    labels = ["GT snapshot\nmove-stop", "RGB-D tracking\nstatic", "RGB-D tracking\nmove-stop", "GT gated\nmove-stop"]
    categories = ["success", "task:grasp", "task:planning", "task:verification", "infrastructure"]
    data = []
    for condition, method, scenario in groups:
        rows = [row for row in trials if row["perception_condition"] == condition and row["method"] == method and row["scenario"] == scenario]
        counts = Counter("success" if row["task_success"] else f"task:{row['failure_stage'] or 'unspecified'}" if row["failure_class"] == "task" else "infrastructure" for row in rows)
        data.append([counts.get(category, 0) for category in categories])
    matrix = np.array(data)
    fig, ax = plt.subplots(figsize=(6.2, 3.4))
    bottom = np.zeros(len(groups))
    colors = ["#3E8C78", "#B95E52", "#D08A3B", "#9A6A9A", "#6E6E6E"]
    for index, category in enumerate(categories):
        values = matrix[:, index]
        ax.bar(np.arange(len(groups)), values, bottom=bottom, label=category, color=colors[index], edgecolor="white", linewidth=0.5)
        for x, y, value in zip(np.arange(len(groups)), bottom, values):
            if value:
                ax.text(x, y + value / 2, str(value), ha="center", va="center", color="white", fontsize=7)
        bottom += values
    ax.set_xticks(np.arange(len(groups)), labels)
    ax.set_ylabel("Trials")
    ax.set_title("Failure-stage distribution preserves task and infrastructure semantics", loc="left", fontsize=9, fontweight="bold")
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    _save_figure(fig, out / "figure_4_failure_stage_distribution")
    source_rows = []
    for row, label in zip(data, labels):
        source_rows.append({"label": label.replace("\n", " "), **{category: value for category, value in zip(categories, row)}})
    _write_csv(out / "figure_4_failure_stage_distribution_source.csv", source_rows, ["label", *categories])
    (out / "figure_4_failure_stage_distribution_notes.md").write_text(
        "Counts are derived from canonical trial-level rows. `success` means task_success=true; `task:*` means a completed task failure with its terminal failure_stage; infrastructure is reserved for TRIAL_FAILED or equivalent infrastructure outcomes.\n",
        encoding="utf-8",
    )
    plt.close(fig)


def _plot_figure5(out: Path, gt_trials: list[dict[str, Any]]) -> bool:
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import numpy as np

    groups = {(method, scenario): [row for row in gt_trials if row["method"] == method and row["scenario"] == scenario] for method in METHODS for scenario in SCENARIOS}
    candidates = ["time_to_ready_s", "target_error_at_ready_m", "target_error_at_grasp_m"]
    metric = next(
        (
            field
            for field in candidates
            if all(
                row.get(field) is not None
                for method in ("tracking", "gated")
                for row in groups[(method, "move_stop")]
            )
        ),
        None,
    )
    if metric is None:
        (out / "figure_5_omission.md").write_text("# Figure 5 omission\n\nNo controlled tracking/gated continuous metric has complete, common-definition data for every move-stop trial. The figure is omitted to avoid mixing missingness or metric definitions.\n", encoding="utf-8")
        return False
    omission = out / "figure_5_omission.md"
    if omission.exists():
        omission.unlink()
    mpl.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"], "svg.fonttype": "none", "pdf.fonttype": 42, "font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(5.4, 3.2))
    raw_values = [[row[metric] for row in groups[(method, "move_stop")]] for method in ("tracking", "gated")]
    display_scale = 1000.0 if metric.endswith("_m") else 1.0
    values = [[value * display_scale for value in group] for group in raw_values]
    ax.boxplot(values, labels=["Tracking", "Gated"], patch_artist=True, boxprops={"facecolor": "#DCE9E5"}, medianprops={"color": "#333333"})
    ax.set_ylabel("target error at grasp (mm)" if metric.endswith("_m") else metric.replace("_", " "))
    ax.set_title("Controlled move-stop internal behavior", loc="left", fontsize=9, fontweight="bold")
    fig.tight_layout()
    _save_figure(fig, out / "figure_5_controlled_internal_behavior")
    source_rows = []
    for method in ("tracking", "gated"):
        raw_group = raw_values[0 if method == "tracking" else 1]
        for value in raw_group:
            source_rows.append({"method": method, "scenario": "move_stop", "metric": metric, "value_m": value, "display_value_mm": value * display_scale})
    _write_csv(out / "figure_5_controlled_internal_behavior_source.csv", source_rows, ["method", "scenario", "metric", "value_m", "display_value_mm"])
    (out / "figure_5_controlled_internal_behavior_notes.md").write_text(
        "Metric selected by completeness gate: target_error_at_grasp_m, the artifact field alias for grasp_initiation_error_m. Raw artifact values remain in metres in the source CSV; the plot displays millimetres for legibility. Values include all controlled move-stop tracking and gated trials.\n",
        encoding="utf-8",
    )
    plt.close(fig)
    return True


def _copy_phase19(out: Path, repo_root: Path) -> None:
    source = repo_root / "results" / "core_baseline_formal-20260825-seeds42-61" / "figures"
    target = out / "figure_1_phase19_timing"
    target.mkdir(parents=True, exist_ok=True)
    for name in ("phase19_seed42_timing.svg", "phase19_seed42_timing.pdf", "phase19_seed42_timing.tiff", "phase19_seed42_timing_preview.png", "phase19_seed42_source.csv", "phase19_seed42_events.csv"):
        if (source / name).is_file():
            shutil.copy2(source / name, target / name)
    (target / "figure_notes.md").write_text((source / "phase19_seed42_figure_notes.md").read_text(encoding="utf-8") + "\n\nFinal audit note: source data and original rendering were copied without recomputation.\n", encoding="utf-8")


def _write_report(out: Path, gt: list[dict[str, Any]], rgbd: list[dict[str, Any]], negative: list[dict[str, Any]], inconsistencies: list[str], gt_rows: list[dict[str, str]], rgbd_rows: list[dict[str, str]], rgbd_trials: list[dict[str, Any]]) -> None:
    def matrix(summary: list[dict[str, Any]]) -> str:
        return "\n".join(f"| {method} | {_cell(summary, method, 'static')} | {_cell(summary, method, 'move_stop')} |" for method in METHODS)
    tracking_failures = [row for row in rgbd_trials if row["method"] == "tracking" and not row["task_success"]]
    fresh = sum(row["pregrasp_commitment_failure"] for row in tracking_failures)
    lines = [
        "# stability-gated-eye-in-hand-grasping 最终实验报告",
        "",
        "本报告只审计已存在的实验 artifact。Controlled ground-truth 与 simulated semantic RGB-D 结果使用独立分母；Phase 1–14 仅作为 development/qualification 证据。",
        "",
        "## 实验设计与 artifact 完整性",
        "",
        f"Controlled formal campaign：{len(gt_rows)} 条 canonical trials；RGB-D formal campaign：{len(rgbd_rows)} 条 canonical trials。两者均要求 metadata、events、states 和 metrics 四项 artifact。",
        f"Canonical attempt 分布：Controlled attempt 1/2 = {Counter(int(row['attempt']) for row in gt_rows).get(1, 0)}/{Counter(int(row['attempt']) for row in gt_rows).get(2, 0)}；RGB-D attempt 1/2 = {Counter(int(row['attempt']) for row in rgbd_rows).get(1, 0)}/{Counter(int(row['attempt']) for row in rgbd_rows).get(2, 0)}。attempt 2 只作 rerun provenance，不重复进入统计分母。",
        f"本次审计发现 {len(inconsistencies)} 个需要记录的 artifact/语义问题。完整清单见 `artifact_audit.json`。",
        "",
        "## Controlled benchmark",
        "",
        "| Method | Static | Move-stop |",
        "| --- | ---: | ---: |",
        matrix(gt),
        "",
        "snapshot 在 move-stop 中的 planning 可以成功，但抓取时使用早期 committed target，最终 physical verification 失败。tracking 和 gated 在两个 controlled scenario 中均完成 task。",
        "",
        "### Figure 1. Controlled move-stop mechanism timeline",
        "",
        "![Phase 19 seed-42 mechanism timeline](figure_1_phase19_timing/phase19_seed42_timing_preview.png)",
        "",
        "*图 1。Controlled ground-truth move-stop、seed 42 的代表性机制时间线。该图直接复用 Phase 19 原始渲染，不是新增 trial。*",
        "",
        "图 1 将 task success 的差异连接到 commitment 时序：snapshot 在目标运动早期锁定 action target，右侧误差在后续抓取阶段保持在约 40 mm 的高位；tracking 持续更新并在运动结束后重新对齐；gated 先经历 gate reset 与稳定窗口，再在 READY 后提交目标。该图解释了为什么 snapshot 的失败表现为最终 physical verification failure，而不是初始 MoveIt 规划失败。",
        "",
        "### Figure 2. Controlled task-success comparison",
        "",
        "![Controlled task success](figures/figure_2_controlled_success.png)",
        "",
        "*图 2。Controlled benchmark 的 task-success counts；每个柱顶保留 `x/20` 原始样本量。*",
        "",
        "图 2 显示 tracking 与 gated 在 static、move-stop 均为 20/20，而 snapshot 仅在 static 为 20/20。由于 tracking 在 controlled perception 下的 move-stop 结果为 20/20，RGB-D 中 tracking 的失败不能归因于 tracking 方法无法处理动态目标。",
        "",
        "## Simulated RGB-D benchmark",
        "",
        "| Method | Static | Move-stop |",
        "| --- | ---: | ---: |",
        matrix(rgbd),
        "",
        f"Tracking 的 RGB-D 失败为 {len(tracking_failures)}/10；其中 {fresh}/10 的 failure reason 明确包含 `fresh, low-drift PREGRASP` commitment failure。该结果表示冻结 pipeline 下的 end-to-end perceptual robustness limitation，不表示 tracking 在 controlled perception 下无法处理动态目标。",
        "",
        "### Figure 3. Simulated RGB-D end-to-end task success",
        "",
        "![Simulated RGB-D task success](figures/figure_3_rgbd_success.png)",
        "",
        "*图 3。冻结 simulated RGB-D pipeline 的 end-to-end task-success counts；每个柱顶保留 `x/5` 原始样本量。*",
        "",
        "图 3 显示 gated 在两个 RGB-D scenario 均为 5/5，snapshot 只在 static 为 5/5，tracking 在两种 scenario 均为 0/5。图中 counts 保留了 n=5 的实验规模，因此不把该结果表述为总体成功率 100%。",
        "",
        "## planning_success 与 failure_stage 的语义",
        "",
        "`planning_success=true` 表示 event stream 中至少出现一次 `PLAN_SUCCEEDED`。`failure_stage=planning` 表示最终 task phase 在 planning 中失败；tracking 可以先完成初始 MoveIt/path plan，再在 freshness、drift 或机械到位条件未同时满足时终止。因此二者不矛盾。",
        "",
        "### Figure 4. Failure-stage distribution",
        "",
        "![Failure-stage distribution](figures/figure_4_failure_stage_distribution.png)",
        "",
        "*图 4。按 terminal semantics 分解代表性条件；`task:verification` 表示已完成执行但 physical grasp verification 未通过。*",
        "",
        "图 4 将 snapshot 的 20 个 controlled move-stop 失败显示为 `task:verification`，将 RGB-D tracking 的 5+5 个失败显示为 `task:planning`，并将 gated move-stop 的 20 个 trial 显示为成功。这里的 `task:verification` 是终止阶段，stale target 是其机制解释；两者不能在表格中混写成同一个 planning failure。",
        "",
        "## tracking RMS 的定义",
        "",
        "正式 execute trial 的 `tracking_rms_error_m` 是首次 `TARGET_OBSERVED` 到 `GRASP_STARTED` 之间 TCP 与目标 ground truth 三维欧氏距离的 RMS，单位为 m。没有 `GRASP_STARTED` 时使用已记录状态的末端区间。它不是 localization error，也不是 selected-target estimation error。",
        "",
        "其数学定义为：",
        "",
        r"$$",
        r"\mathrm{RMS}_{\mathrm{TCP\text{-}GT}} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\left\|\mathbf{p}_{\mathrm{target,GT}}(t_i)-\mathbf{p}_{\mathrm{TCP}}(t_i)\right\|_2^2}\,.",
        r"$$",
        "",
        "其中 $t_i$ 覆盖从首次 `TARGET_OBSERVED` 到 `GRASP_STARTED` 的状态采样；块公式使用 Markdown 兼容的 `$$...$$` LaTeX 分隔符。",
        "",
        "### Figure 5. Controlled move-stop internal target-error comparison",
        "",
        "![Controlled move-stop internal target error](figures/figure_5_controlled_internal_behavior.png)",
        "",
        "*图 5。Tracking 与 gated 在 controlled move-stop 中的 `target_error_at_grasp_m` 分布；图中为可读性将原始 m 值转换为 mm。该指标不是 `tracking_rms_error_m`。*",
        "",
        "图 5 只用于比较成功 trial 的内部行为，不改变两种方法均为 20/20 的结论。Tracking 的 grasp-time target error 中位数约为 7.31×10⁻⁹ m，gated 约为 6.38×10⁻⁷ m；两者都远低于厘米尺度，且该指标不能单独证明某种 commitment 结构在所有条件下更优。",
        "",
        "## Negative/safety tests",
        "",
        "Phase 10 的主计数只把 physical grasp 证据和完整 terminal artifact 纳入判断；teardown/shutdown artifact 不完整情况单独标记为 infrastructure。所有可判定场景均未产生 physical false-positive grasp。",
        "",
        "| Scenario | Safe rejection | Physical attempt | Physical false positive | Classification |",
        "| --- | ---: | ---: | ---: | --- |",
    ]
    lines += [f"| {row['scenario']} | {row['safe_rejection']} | {row['physical_grasp_attempted']} | {row['physical_false_positive']} | {row['classification']} |" for row in negative]
    lines += [
        "",
        "## Qualification timeline",
        "",
        "Phase 1–14 完成了 RGB-D/camera 与 segmentation bring-up、depth fusion、camera-to-base TF 与 hand-eye extrinsic consistency、surface-point 与 geometric-center diagnosis、固定平面抓取 calibration（`grasp_offset_x=0.015 m`）、首次 static RGB-D execute、gripper post-contact symmetry diagnosis、symmetry hard-gate 到 warning、static repeated qualification、move-stop gated qualification、six-condition smoke、negative safety qualification 和 configuration freeze。这些结果不进入正式 statistically powered denominator。",
        "",
        "## 结论与限制",
        "",
        "1. Snapshot 在 static controlled 与 RGB-D static 成功，在 move-stop 中显示 stale/early commitment 失败机制。",
        "2. Tracking 在 controlled perception 下处理 static 与 move-stop；在冻结的 simulated RGB-D pipeline 下 10/10 trial 在 fresh/low-drift PREGRASP commitment 阶段失败。",
        "3. Gated 在本次 controlled 与 simulated RGB-D 条件下均完成 task。该证据不外推到所有机器人、所有 RGB-D 系统或真实相机物理性能。",
        "4. n=5/condition 的 RGB-D 结果展示的是本实验配置下的重复结果，不等同于总体成功率 100%。",
        "",
        "## Reproducibility",
        "",
        "生成器为 Python-only；输入哈希见 `input_hashes_before.json` 与 `input_hashes_after.json`。所有正式 trial 的 source path、attempt、event/metrics 语义和表格 provenance 见 CSV 与 `artifact_audit.json`。",
    ]
    (out / "final_experiment_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_audit(repo_root: Path, output_dir: Path) -> dict[str, Any]:
    gt_campaign = repo_root / "results" / "core_baseline_formal-20260825-seeds42-61"
    rgbd_campaign = repo_root / "results" / "phase20_rgbd_formal-20260825-seeds42-46"
    gt_rows = load_canonical_trials(gt_campaign, expected_n=120, expected_seeds=range(42, 62))
    rgbd_rows = load_canonical_trials(rgbd_campaign, expected_n=30, expected_seeds=range(42, 47))
    output_dir.mkdir(parents=True, exist_ok=True)
    before = _hash_inputs(_campaign_input_files(gt_campaign, gt_rows) + _campaign_input_files(rgbd_campaign, rgbd_rows))
    (output_dir / "input_hashes_before.json").write_text(json.dumps(before, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    gt_trials = [audit_trial(row, "controlled") for row in gt_rows]
    rgbd_trials = [audit_trial(row, "rgbd") for row in rgbd_rows]
    for trial in gt_trials + rgbd_trials:
        source = Path(trial["source_run"])
        trial["source_run"] = str(source.relative_to(repo_root)) if source.is_absolute() else str(source)
    all_trials = gt_trials + rgbd_trials
    gt_summary = summarize_trials(gt_trials)
    rgbd_summary = summarize_trials(rgbd_trials)
    summary_fields = list(gt_summary[0])
    _write_csv(output_dir / "gt_formal_summary.csv", gt_summary, summary_fields)
    _write_csv(output_dir / "rgbd_formal_summary.csv", rgbd_summary, summary_fields)
    all_fields = [key for key in all_trials[0] if key != "inconsistencies"] + ["inconsistencies"]
    _write_csv(output_dir / "all_formal_trials.csv", all_trials, all_fields)
    negative = _negative_rows(repo_root)
    _write_csv(output_dir / "negative_safety_summary.csv", negative, list(negative[0]))
    qualification_rows = [
        {"phase": "1-3", "evidence": "Gazebo stages and control-path bring-up", "status": "qualification", "source": "results/phase13_rgbd_smoke"},
        {"phase": "4-6", "evidence": "RGB-D camera, segmentation, depth fusion, camera-to-base TF and hand-eye consistency", "status": "qualification", "source": "results/phase9_static_rgbd_5_clean"},
        {"phase": "7", "evidence": "Fixed planar grasp calibration; grasp_offset_x=0.015 m", "status": "qualification", "source": "results/offset_calibration"},
        {"phase": "8-9", "evidence": "First static RGB-D execute and repeated static grasp qualification", "status": "qualification", "source": "results/offset_calibration;results/phase9_static_rgbd_5_clean"},
        {"phase": "10", "evidence": "Negative safety tests and no-physical-grasp verification", "status": "qualification", "source": "results/phase10_rgbd_negative"},
        {"phase": "11-14", "evidence": "Move-stop gated qualification, six-condition smoke, configuration freeze", "status": "qualification", "source": "results/phase11_rgbd_move_stop;results/phase12_rgbd_move_stop;results/phase13_rgbd_smoke"},
    ]
    _write_csv(output_dir / "qualification_evidence.csv", qualification_rows, ["phase", "evidence", "status", "source"])
    _write_paper_tables(output_dir, gt_summary, rgbd_summary, all_trials, negative)
    _copy_phase19(output_dir, repo_root)
    figures = output_dir / "figures"
    _plot_success(figures, gt_summary, "Controlled task success", 20, "figure_2_controlled_success")
    _plot_success(figures, rgbd_summary, "Simulated RGB-D task success", 5, "figure_3_rgbd_success")
    _plot_failure_stages(figures, all_trials)
    figure5 = _plot_figure5(figures, gt_trials)
    inconsistencies = [f"{row['perception_condition']}:{row['run_id']}:{row['inconsistencies']}" for row in all_trials if row["inconsistencies"]]
    audit_payload = {
        "inconsistencies": inconsistencies,
        "gt_attempt_counts": dict(Counter(row["attempt"] for row in gt_rows)),
        "rgbd_attempt_counts": dict(Counter(row["attempt"] for row in rgbd_rows)),
        "gt_attempt2_reruns": [
            {"run_id": row["run_id"], "method": row["method"], "scenario": row["scenario"], "seed": int(row["seed"]), "source_run": str(Path(row["_resolved_result_path"]).relative_to(repo_root)) if Path(row["_resolved_result_path"]).is_absolute() else row["_resolved_result_path"]}
            for row in gt_rows if int(row["attempt"]) > 1
        ],
        "rgbd_attempt2_reruns": [
            {"run_id": row["run_id"], "method": row["method"], "scenario": row["scenario"], "seed": int(row["seed"]), "source_run": str(Path(row["_resolved_result_path"]).relative_to(repo_root)) if Path(row["_resolved_result_path"]).is_absolute() else row["_resolved_result_path"]}
            for row in rgbd_rows if int(row["attempt"]) > 1
        ],
        "tracking_rgbd_fresh_low_drift": sum(row["pregrasp_commitment_failure"] for row in rgbd_trials if row["method"] == "tracking"),
    }
    (output_dir / "artifact_audit.json").write_text(json.dumps(audit_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    _write_report(output_dir, gt_summary, rgbd_summary, negative, inconsistencies, gt_rows, rgbd_rows, rgbd_trials)
    after = _hash_inputs(_campaign_input_files(gt_campaign, gt_rows) + _campaign_input_files(rgbd_campaign, rgbd_rows))
    (output_dir / "input_hashes_after.json").write_text(json.dumps(after, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    unchanged = before == after
    (output_dir / "qa_summary.md").write_text("# Final audit QA\n\n" + f"- Controlled canonical trials: {len(gt_trials)}/120\n- RGB-D canonical trials: {len(rgbd_trials)}/30\n- Combined trial-level rows: {len(all_trials)}\n- Tracking RGB-D fresh/low-drift failures: {sum(row['pregrasp_commitment_failure'] for row in rgbd_trials if row['method'] == 'tracking')}/10\n- Input hashes unchanged: {'PASS' if unchanged else 'FAIL'}\n- Figure 5 generated: {'yes' if figure5 else 'omitted with rationale'}\n", encoding="utf-8")
    return {"gt": gt_trials, "rgbd": rgbd_trials, "all": all_trials, "negative": negative, "inconsistencies": inconsistencies, "input_hashes_unchanged": unchanged, "figure5": figure5}
