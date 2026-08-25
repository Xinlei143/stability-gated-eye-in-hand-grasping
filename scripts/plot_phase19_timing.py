#!/usr/bin/env python3
"""Create the Phase 19 seed-42 move-stop mechanism figure.

The figure is deliberately generated from the canonical Phase 16 artifacts,
not from hand-entered values.  It uses the recorded ground-truth, observation,
selected-target, latched-target, and event timelines for each method.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


# Keep text editable in the SVG/PDF outputs and use a portable sans fallback.
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Arial", "DejaVu Sans", "Liberation Sans"]
plt.rcParams["svg.fonttype"] = "none"
plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["axes.linewidth"] = 0.8
plt.rcParams["xtick.major.width"] = 0.8
plt.rcParams["ytick.major.width"] = 0.8


METHODS = ("snapshot", "tracking", "gated")
METHOD_LABELS = {
    "snapshot": "snapshot",
    "tracking": "tracking",
    "gated": "gated",
}
METHOD_COLORS = {
    "snapshot": "#B04443",
    "tracking": "#185A9D",
    "gated": "#2E8B57",
}
EVENT_STYLES = {
    "READY": ("#228B22", "-", 1.0),
    "PLAN_STARTED": ("#386CB0", "--", 0.75),
    "GRASP_STARTED": ("#D97706", "-.", 0.9),
    "LIFT_STARTED": ("#C0392B", "-", 0.95),
    "TARGET_RELATCHED": ("#7B2CBF", ":", 0.9),
    "GATE_RESET": ("#7F8C8D", (0, (2, 2)), 0.55),
}
EVENT_ORDER = (
    "READY",
    "PLAN_STARTED",
    "GRASP_STARTED",
    "LIFT_STARTED",
    "TARGET_RELATCHED",
    "GATE_RESET",
)


def _float_or_nan(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    return float(value)


def _load_states(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"empty state file: {path}")

    # Multiple rows can share a startup timestamp.  Keep the first row for a
    # deterministic, non-self-crossing line plot; startup values are blank.
    times = np.asarray([int(row["sim_time_ns"]) / 1e9 for row in rows])
    _, first_indices = np.unique(times, return_index=True)
    indices = np.sort(first_indices)

    columns = {
        "t_s": times[indices],
        "gt_x_m": np.asarray(
            [_float_or_nan(rows[index].get("target_ground_truth_x")) for index in indices]
        ),
        "observed_x_m": np.asarray(
            [_float_or_nan(rows[index].get("target_observed_x")) for index in indices]
        ),
        "selected_x_m": np.asarray(
            [_float_or_nan(rows[index].get("target_selected_x")) for index in indices]
        ),
        "latched_x_m": np.asarray(
            [_float_or_nan(rows[index].get("target_latched_x")) for index in indices]
        ),
    }
    columns["selected_error_mm"] = np.abs(
        columns["selected_x_m"] - columns["gt_x_m"]
    ) * 1000.0
    return columns


def _load_events(path: Path) -> list[dict[str, str | float]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return [
        {
            "event": row["event"],
            "t_s": int(row["sim_time_ns"]) / 1e9,
            "details": row.get("details", ""),
        }
        for row in rows
    ]


def _motion_interval(data: dict[str, np.ndarray]) -> tuple[float, float]:
    valid = np.isfinite(data["gt_x_m"])
    t = data["t_s"][valid]
    x = data["gt_x_m"][valid]
    if len(t) < 2:
        raise RuntimeError("not enough ground-truth samples to infer motion interval")

    # The move-stop trajectory changes by roughly 1 mm per 100 ms.  The
    # post-stop numerical drift is several orders smaller, so this threshold
    # identifies the recorded motion interval without using method events.
    moving = np.flatnonzero(np.abs(np.diff(x)) > 3e-4)
    if len(moving) == 0:
        raise RuntimeError("could not identify the move-stop motion interval")
    return float(t[moving[0]]), float(t[moving[-1] + 1])


def _write_source_csv(output_dir: Path, records: dict[str, dict[str, np.ndarray]]) -> None:
    source_path = output_dir / "phase19_seed42_source.csv"
    with source_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "method",
                "t_s",
                "target_ground_truth_x_m",
                "target_observed_x_m",
                "target_selected_x_m",
                "target_latched_x_m",
                "selected_target_error_mm",
            ]
        )
        for method in METHODS:
            data = records[method]
            for index in range(len(data["t_s"])):
                writer.writerow(
                    [
                        method,
                        f"{data['t_s'][index]:.6f}",
                        _format_number(data["gt_x_m"][index]),
                        _format_number(data["observed_x_m"][index]),
                        _format_number(data["selected_x_m"][index]),
                        _format_number(data["latched_x_m"][index]),
                        _format_number(data["selected_error_mm"][index]),
                    ]
                )

    events_path = output_dir / "phase19_seed42_events.csv"
    with events_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["method", "event", "t_s", "details"])
        for method in METHODS:
            for event in records[method]["events"]:
                writer.writerow([method, event["event"], f"{event['t_s']:.6f}", event["details"]])


def _format_number(value: float) -> str:
    return "" if not np.isfinite(value) else f"{value:.9f}"


def _event_times(events: list[dict[str, str | float]], event_name: str) -> list[float]:
    return [float(event["t_s"]) for event in events if event["event"] == event_name]


def _plot_event_lines(
    axes: tuple[plt.Axes, plt.Axes],
    events: list[dict[str, str | float]],
    plot_end: float,
) -> None:
    ready_times = _event_times(events, "READY")
    first_ready = min(ready_times) if ready_times else plot_end
    for event_name in EVENT_ORDER:
        color, linestyle, alpha = EVENT_STYLES[event_name]
        for t_s in _event_times(events, event_name):
            # Gate resets after LIFT_STARTED are terminal-state cleanup and
            # are intentionally excluded from the mechanism timeline.
            if t_s > plot_end or (event_name == "GATE_RESET" and t_s > first_ready):
                continue
            for axis in axes:
                axis.axvline(t_s, color=color, linestyle=linestyle, linewidth=0.8, alpha=alpha, zorder=1)


def _plot_row(
    axes: tuple[plt.Axes, plt.Axes],
    method: str,
    data: dict[str, np.ndarray],
    motion_start: float,
    motion_end: float,
    plot_end: float,
    x_min: float,
    x_max: float,
) -> None:
    position_axis, error_axis = axes
    color = METHOD_COLORS[method]
    t = data["t_s"]

    for axis in axes:
        axis.axvspan(motion_start, motion_end, color="#CFE8F3", alpha=0.55, zorder=0)
        axis.axvspan(motion_end, x_max, color="#F2F2F2", alpha=0.45, zorder=0)
        axis.grid(axis="y", color="#D9D9D9", linewidth=0.55, alpha=0.7)
        axis.set_xlim(x_min, x_max)

    visible = t <= plot_end
    def visible_values(values: np.ndarray) -> np.ndarray:
        return np.where(visible, values, np.nan)

    position_axis.plot(t, visible_values(data["gt_x_m"]), color="#222222", linewidth=1.5, linestyle="--", label="GT")
    position_axis.plot(t, visible_values(data["observed_x_m"]), color="#777777", linewidth=1.0, linestyle=":", label="observed")
    position_axis.plot(t, visible_values(data["selected_x_m"]), color=color, linewidth=1.8, label="selected")
    position_axis.plot(
        t,
        visible_values(data["latched_x_m"]),
        color=color,
        linewidth=1.0,
        linestyle=(0, (4, 2)),
        alpha=0.8,
        label="latched/action target",
    )

    error = np.maximum(data["selected_error_mm"], 1e-4)
    error = visible_values(error)
    error_axis.plot(t, error, color=color, linewidth=1.8)
    error_axis.set_yscale("log")
    error_axis.set_ylim(1e-4, 60)
    error_axis.set_yticks([1e-3, 1e-2, 1e-1, 1, 10, 40])
    error_axis.set_yticklabels(["0.001", "0.01", "0.1", "1", "10", "40"])

    _plot_event_lines(axes, data["events"], plot_end)
    position_axis.text(
        -0.12,
        0.50,
        METHOD_LABELS[method],
        transform=position_axis.transAxes,
        ha="right",
        va="center",
        fontsize=10,
        fontweight="bold",
        color=color,
    )
    mechanism_text = {
        "snapshot": "early lock → stale action target",
        "tracking": "continuous update → relatch",
        "gated": "resets → stable READY → commit",
    }[method]
    error_axis.text(
        0.98,
        0.90,
        mechanism_text,
        transform=error_axis.transAxes,
        ha="right",
        va="top",
        fontsize=8,
        color=color,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.75, "pad": 1.5},
    )


def _write_notes(
    output_dir: Path,
    campaign_root: Path,
    trial_paths: dict[str, Path],
    intervals: dict[str, tuple[float, float]],
    plot_ends: dict[str, float],
) -> None:
    notes = output_dir / "phase19_seed42_figure_notes.md"
    lines = [
        "# Phase 19 timing figure provenance",
        "",
        "This figure uses Python/matplotlib and the canonical Phase 16 ground-truth benchmark artifacts.",
        "It is a representative `move_stop`, seed `42` comparison and is not a new trial.",
        "",
        f"- Campaign: `{campaign_root}`",
        "- Methods: snapshot, tracking, gated",
        "- Scenario: move_stop",
        "- Seed: 42",
        "- Left column: recorded GT x position, observed x, selected x, and latched/action target.",
        "- Right column: absolute selected-target error in millimetres on a logarithmic axis.",
        "- Blue background: recorded target motion; grey background: post-motion interval.",
        "- Vertical markers: READY, PLAN_STARTED, GRASP_STARTED, LIFT_STARTED, TARGET_RELATCHED, and GATE_RESET.",
        "- Missing values are kept missing; no interpolation is used.",
        "",
        "## Source trial artifacts",
        "",
    ]
    for method in METHODS:
        start, end = intervals[method]
        lines.append(
            f"- `{method}`: `{trial_paths[method]}`; inferred motion interval `{start:.3f}`–`{end:.3f}` s; plotted through `LIFT_STARTED` at `{plot_ends[method]:.3f}` s."
        )
    notes.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--campaign",
        type=Path,
        default=Path("results/core_baseline_formal-20260825-seeds42-61"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results/core_baseline_formal-20260825-seeds42-61/figures"),
    )
    args = parser.parse_args()

    with (args.campaign / "trials.csv").open(newline="") as handle:
        trials = list(csv.DictReader(handle))

    records: dict[str, dict[str, np.ndarray]] = {}
    trial_paths: dict[str, Path] = {}
    intervals: dict[str, tuple[float, float]] = {}
    plot_ends: dict[str, float] = {}
    for method in METHODS:
        trial = next(
            row
            for row in trials
            if row["method"] == method
            and row["scenario"] == "move_stop"
            and row["seed"] == "42"
            and row["status"] == "finished"
        )
        trial_path = Path(trial["result_path"])
        trial_paths[method] = trial_path
        data = _load_states(trial_path / "states.csv")
        data["events"] = _load_events(trial_path / "events.csv")
        records[method] = data
        intervals[method] = _motion_interval(data)
        lift_times = _event_times(data["events"], "LIFT_STARTED")
        plot_ends[method] = min(lift_times) + 0.15 if lift_times else float(np.nanmax(data["t_s"]))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    _write_source_csv(args.output_dir, records)
    _write_notes(args.output_dir, args.campaign, trial_paths, intervals, plot_ends)

    x_min = min(float(np.nanmin(data["t_s"])) for data in records.values()) - 0.5
    x_max = max(plot_ends.values()) + 0.5

    figure, axes = plt.subplots(
        nrows=3,
        ncols=2,
        figsize=(12.2, 8.4),
        sharex="col",
        gridspec_kw={"width_ratios": [1.7, 1.0], "hspace": 0.16, "wspace": 0.18},
    )
    for row_index, method in enumerate(METHODS):
        _plot_row(
            (axes[row_index, 0], axes[row_index, 1]),
            method,
            records[method],
            intervals[method][0],
            intervals[method][1],
            plot_ends[method],
            x_min,
            x_max,
        )

    axes[0, 0].set_title("Target position and action target", fontsize=11, pad=10)
    axes[0, 1].set_title("Selected-target error", fontsize=11, pad=10)
    axes[0, 0].set_ylabel("x position (m)")
    axes[1, 0].set_ylabel("x position (m)")
    axes[2, 0].set_ylabel("x position (m)")
    axes[0, 1].set_ylabel("error (mm, log scale)")
    axes[1, 1].set_ylabel("error (mm, log scale)")
    axes[2, 1].set_ylabel("error (mm, log scale)")
    axes[2, 0].set_xlabel("simulation time (s)")
    axes[2, 1].set_xlabel("simulation time (s)")

    # The motion/stopped labels apply to each row's independently recorded
    # trial, so place them inside each row instead of forcing a shared time.
    for row_index, method in enumerate(METHODS):
        start, end = intervals[method]
        axes[row_index, 0].text(
            (start + end) / 2,
            1.02,
            "target motion",
            transform=axes[row_index, 0].get_xaxis_transform(),
            ha="center",
            va="bottom",
            fontsize=8,
            color="#356A80",
        )
        axes[row_index, 0].text(
            end + 0.08 * (x_max - x_min),
            1.02,
            "stopped",
            transform=axes[row_index, 0].get_xaxis_transform(),
            ha="left",
            va="bottom",
            fontsize=8,
            color="#666666",
        )

    series_legend = [
        Line2D([0], [0], color="#222222", linestyle="--", linewidth=1.5, label="GT"),
        Line2D([0], [0], color="#777777", linestyle=":", linewidth=1.0, label="observed"),
        Line2D([0], [0], color="#555555", linewidth=1.8, label="selected"),
        Line2D([0], [0], color="#555555", linestyle=(0, (4, 2)), linewidth=1.0, label="latched/action target"),
    ]
    event_legend = [
        Line2D([0], [0], color=EVENT_STYLES[name][0], linestyle=EVENT_STYLES[name][1], linewidth=1.0, label=name)
        for name in ("READY", "PLAN_STARTED", "GRASP_STARTED", "LIFT_STARTED", "GATE_RESET")
    ]
    figure.legend(
        handles=series_legend + event_legend,
        loc="upper center",
        bbox_to_anchor=(0.52, 0.995),
        ncol=5,
        frameon=False,
        fontsize=8.5,
        columnspacing=1.1,
        handlelength=2.3,
    )
    figure.suptitle(
        "Phase 19 | Mechanism timeline: controlled ground-truth move-stop, seed 42",
        fontsize=13,
        fontweight="bold",
        y=1.035,
    )
    figure.text(
        0.01,
        0.005,
        "Events and trajectories are recorded from the canonical Phase 16 artifacts; no values are interpolated.",
        fontsize=7.5,
        color="#666666",
    )

    figure.subplots_adjust(top=0.87, bottom=0.08, left=0.10, right=0.985)
    output_base = args.output_dir / "phase19_seed42_timing"
    figure.savefig(output_base.with_suffix(".svg"), format="svg", bbox_inches="tight")
    figure.savefig(output_base.with_suffix(".pdf"), format="pdf", bbox_inches="tight")
    figure.savefig(output_base.with_suffix(".tiff"), format="tiff", dpi=600, bbox_inches="tight")
    figure.savefig(output_base.with_name(output_base.name + "_preview.png"), format="png", dpi=180, bbox_inches="tight")
    plt.close(figure)

    print(f"wrote figure outputs under {args.output_dir}")
    for method in METHODS:
        print(f"{method}: motion {intervals[method][0]:.3f}-{intervals[method][1]:.3f} s")


if __name__ == "__main__":
    main()
