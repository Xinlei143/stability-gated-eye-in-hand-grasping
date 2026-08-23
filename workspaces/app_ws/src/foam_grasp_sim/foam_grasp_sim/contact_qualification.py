"""Summarize per-finger contact wrench samples for grasp qualification."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median


SETTLE_START_EVENT = "GRIPPER_SETTLE_STARTED"
SETTLE_FINISH_EVENT = "GRIPPER_SETTLE_FINISHED"
DEFAULT_GRID_HZ = 100.0
DEFAULT_MINIMUM_FORCE_N = 0.8
DEFAULT_MAXIMUM_P95_FORCE_N = 3.0
DEFAULT_MINIMUM_BILATERAL_FRACTION = 0.8
DEFAULT_MINIMUM_BILATERAL_DURATION_S = 0.8


def _percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def _legacy_summarize_contact_rows(
    rows,
    *,
    hold_s=1.0,
    minimum_force_N=0.5,
    maximum_p95_force_N=3.0,
):
    """Aggregate contact points by side and timestamp and return pass/fail data."""

    grouped = defaultdict(lambda: defaultdict(float))
    for row in rows:
        side = str(row.get("side", "")).strip()
        if side not in {"left", "right"}:
            continue
        try:
            stamp = int(float(row.get("sim_time_ns", "")))
            force = float(row.get("normal_force_N", ""))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(force):
            continue
        grouped[side][stamp] += max(force, 0.0)

    per_side = {}
    failure_reasons = []
    for side in ("left", "right"):
        samples = grouped.get(side, {})
        stamps = sorted(samples)
        forces = [samples[stamp] for stamp in stamps]
        duration_s = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) >= 2 else 0.0
        median_force = float(median(forces)) if forces else math.nan
        p95_force = _percentile(forces, 0.95)
        stable_fraction = (
            sum(force >= minimum_force_N for force in forces) / len(forces)
            if forces else 0.0
        )
        per_side[side] = {
            "sample_count": len(forces),
            "duration_s": duration_s,
            "median_normal_force_N": median_force,
            "p95_normal_force_N": p95_force,
            "stable_fraction": stable_fraction,
            "passed": bool(
                forces
                and duration_s >= hold_s * 0.8
                and median_force >= minimum_force_N
                and p95_force <= maximum_p95_force_N
                and stable_fraction >= 0.8
            ),
        }
        if not forces:
            failure_reasons.append(f"missing_{side}_contact")
        elif not per_side[side]["passed"]:
            failure_reasons.append(f"unstable_{side}_force")

    sides = sorted(
        side for side, values in per_side.items() if values["sample_count"] > 0
    )
    if len(sides) < 2:
        failure_reasons.append("missing_side")
    return {
        "passed": not failure_reasons,
        "sides": sides,
        "per_side": per_side,
        "hold_s": float(hold_s),
        "minimum_force_N": float(minimum_force_N),
        "maximum_p95_force_N": float(maximum_p95_force_N),
        "failure_reasons": failure_reasons,
    }


def _event_time(event):
    try:
        return int(float(event.get("sim_time_ns", "")))
    except (AttributeError, TypeError, ValueError):
        return None


def _settle_window(events):
    starts = []
    finishes = []
    for event in events or ():
        name = str(event.get("event", "")).strip().upper()
        stamp = _event_time(event)
        if stamp is None:
            continue
        if name == SETTLE_START_EVENT:
            starts.append(stamp)
        elif name == SETTLE_FINISH_EVENT:
            finishes.append(stamp)

    failure_reasons = []
    start_ns = min(starts) if starts else None
    finish_ns = None
    if start_ns is None:
        failure_reasons.append("missing_gripper_settle_started")
    else:
        valid_finishes = [stamp for stamp in finishes if stamp >= start_ns]
        finish_ns = min(valid_finishes) if valid_finishes else None
        if finish_ns is None:
            failure_reasons.append("missing_gripper_settle_finished")
    if start_ns is not None and finishes and finish_ns is None:
        failure_reasons.append("invalid_gripper_settle_window")
    return start_ns, finish_ns, failure_reasons


def _grid_values(rows, start_ns, finish_ns, grid_hz):
    step_ns = 1e9 / float(grid_hz)
    grid_count = int(math.floor((finish_ns - start_ns) / step_ns + 1e-9)) + 1
    grid_count = max(grid_count, 1)
    values = {side: [0.0] * grid_count for side in ("left", "right")}
    records = {side: [[] for _ in range(grid_count)] for side in ("left", "right")}
    for row in rows:
        side = str(row.get("side", "")).strip().lower()
        if side not in values:
            continue
        stamp = _event_time(row)
        try:
            force = float(row.get("normal_force_N", ""))
        except (TypeError, ValueError):
            continue
        if stamp is None or not math.isfinite(force) or stamp < start_ns or stamp > finish_ns:
            continue
        index = int(math.floor((stamp - start_ns) / step_ns + 0.5))
        if 0 <= index < grid_count:
            values[side][index] += max(force, 0.0)
            records[side][index].append(row)
    return values, records, grid_count, step_ns / 1e9


def _longest_contiguous(values, step_s):
    longest = 0
    current = 0
    for value in values:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return max(0.0, (longest - 1) * step_s)


def _event_window_summary(
    rows,
    events,
    *,
    hold_s=1.0,
    minimum_force_N=DEFAULT_MINIMUM_FORCE_N,
    maximum_p95_force_N=DEFAULT_MAXIMUM_P95_FORCE_N,
    grid_hz=DEFAULT_GRID_HZ,
    minimum_bilateral_fraction=DEFAULT_MINIMUM_BILATERAL_FRACTION,
    minimum_bilateral_duration_s=DEFAULT_MINIMUM_BILATERAL_DURATION_S,
):
    start_ns, finish_ns, failure_reasons = _settle_window(events)
    result = {
        "passed": False,
        "sides": [],
        "per_side": {},
        "hold_second_half": {},
        "window_start_ns": start_ns,
        "window_end_ns": finish_ns,
        "window_duration_s": (
            (finish_ns - start_ns) / 1e9
            if start_ns is not None and finish_ns is not None
            else 0.0
        ),
        "grid_hz": float(grid_hz),
        "grid_sample_count": 0,
        "bilateral_stable_fraction": 0.0,
        "longest_contiguous_bilateral_contact_s": 0.0,
        "hold_s": float(hold_s),
        "minimum_force_N": float(minimum_force_N),
        "maximum_p95_force_N": float(maximum_p95_force_N),
        "minimum_bilateral_fraction": float(minimum_bilateral_fraction),
        "minimum_bilateral_duration_s": float(minimum_bilateral_duration_s),
        "failure_reasons": list(failure_reasons),
        "joint_metrics": {},
    }
    if failure_reasons:
        return result
    if finish_ns <= start_ns:
        result["failure_reasons"].append("invalid_gripper_settle_window")
        return result
    if result["window_duration_s"] + 1e-9 < float(hold_s):
        result["failure_reasons"].append("settle_window_short")

    values, records, grid_count, step_s = _grid_values(rows, start_ns, finish_ns, grid_hz)
    result["grid_sample_count"] = grid_count
    midpoint_ns = start_ns + (finish_ns - start_ns) / 2.0
    second_half_index = max(0, int(math.ceil((midpoint_ns - start_ns) / (step_s * 1e9))))
    for side in ("left", "right"):
        side_values = values[side]
        second_half = side_values[second_half_index:]
        side_result = {
            "sample_count": len(side_values),
            "duration_s": result["window_duration_s"],
            "median_normal_force_N": float(median(side_values)),
            "p95_normal_force_N": _percentile(side_values, 0.95),
            "stable_fraction": sum(value >= minimum_force_N for value in side_values) / grid_count,
            "nonzero_sample_count": sum(value > 0.0 for value in side_values),
        }
        hold_result = {
            "sample_count": len(second_half),
            "median_normal_force_N": float(median(second_half)) if second_half else math.nan,
            "p95_normal_force_N": _percentile(second_half, 0.95),
            "stable_fraction": (
                sum(value >= minimum_force_N for value in second_half) / len(second_half)
                if second_half else 0.0
            ),
        }
        side_result["hold_second_half_median_normal_force_N"] = hold_result[
            "median_normal_force_N"
        ]
        side_result["hold_second_half_p95_normal_force_N"] = hold_result[
            "p95_normal_force_N"
        ]
        result["per_side"][side] = side_result
        result["hold_second_half"][side] = hold_result
        if side_result["nonzero_sample_count"] == 0:
            result["failure_reasons"].append(f"missing_{side}_contact")
        if hold_result["median_normal_force_N"] < minimum_force_N:
            result["failure_reasons"].append(f"low_{side}_second_half_median_force")
        if hold_result["p95_normal_force_N"] > maximum_p95_force_N:
            result["failure_reasons"].append(f"high_{side}_second_half_p95_force")

    metric_values = {"effort_abs_N": [], "velocity_abs_m_s": [], "symmetry_error_m": []}
    # Qualification metrics describe the settled hold, so exclude the
    # approach/settling half from effort, velocity, and opening-symmetry
    # summaries just as for the force statistics above.
    for side_records in records.values():
        for cell in side_records[second_half_index:]:
            for row in cell:
                for key, field in (
                    ("effort_abs_N", "joint7_effort_N"),
                    ("effort_abs_N", "joint8_effort_N"),
                    ("velocity_abs_m_s", "joint7_velocity_m_s"),
                    ("velocity_abs_m_s", "joint8_velocity_m_s"),
                    ("symmetry_error_m", "gripper_symmetry_error_m"),
                ):
                    try:
                        value = abs(float(row.get(field, "")))
                    except (TypeError, ValueError):
                        continue
                    if math.isfinite(value):
                        metric_values[key].append(value)
    result["joint_metrics"] = {
        "median_effort_abs_N": float(median(metric_values["effort_abs_N"]))
        if metric_values["effort_abs_N"] else math.nan,
        "p95_effort_abs_N": _percentile(metric_values["effort_abs_N"], 0.95),
        "median_abs_velocity_m_s": float(median(metric_values["velocity_abs_m_s"]))
        if metric_values["velocity_abs_m_s"] else math.nan,
        "p95_abs_velocity_m_s": _percentile(metric_values["velocity_abs_m_s"], 0.95),
        "median_symmetry_error_m": float(median(metric_values["symmetry_error_m"]))
        if metric_values["symmetry_error_m"] else math.nan,
        "p95_symmetry_error_m": _percentile(metric_values["symmetry_error_m"], 0.95),
    }

    result["sides"] = [
        side for side in ("left", "right")
        if result["per_side"][side]["nonzero_sample_count"] > 0
    ]
    bilateral = [
        values["left"][index] >= minimum_force_N
        and values["right"][index] >= minimum_force_N
        for index in range(grid_count)
    ]
    bilateral_count = sum(bilateral)
    result["bilateral_stable_fraction"] = bilateral_count / grid_count
    result["longest_contiguous_bilateral_contact_s"] = _longest_contiguous(
        bilateral, step_s
    )
    if result["bilateral_stable_fraction"] < minimum_bilateral_fraction:
        result["failure_reasons"].append("bilateral_contact_too_sparse")
    if result["longest_contiguous_bilateral_contact_s"] < minimum_bilateral_duration_s:
        result["failure_reasons"].append("bilateral_contact_too_short")
    result["passed"] = not result["failure_reasons"]
    return result


def summarize_contact_rows(
    rows,
    *,
    events=None,
    hold_s=1.0,
    minimum_force_N=None,
    maximum_p95_force_N=DEFAULT_MAXIMUM_P95_FORCE_N,
    grid_hz=DEFAULT_GRID_HZ,
    minimum_bilateral_fraction=DEFAULT_MINIMUM_BILATERAL_FRACTION,
    minimum_bilateral_duration_s=DEFAULT_MINIMUM_BILATERAL_DURATION_S,
):
    """Summarize contacts inside the recorded settle event window."""

    if events is None:
        raise ValueError("events are required for contact qualification")
    return _event_window_summary(
        rows,
        events,
        hold_s=hold_s,
        minimum_force_N=(
            DEFAULT_MINIMUM_FORCE_N if minimum_force_N is None else minimum_force_N
        ),
        maximum_p95_force_N=maximum_p95_force_N,
        grid_hz=grid_hz,
        minimum_bilateral_fraction=minimum_bilateral_fraction,
        minimum_bilateral_duration_s=minimum_bilateral_duration_s,
    )


def _read_csv(path):
    with Path(path).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def summarize_contact_file(input_path, output_path=None, events_path=None, **kwargs):
    if events_path is None:
        raise ValueError("events_path is required for contact qualification")
    input_path = Path(input_path)
    rows = _read_csv(input_path)
    events = None
    if events_path is not None:
        events_path = Path(events_path)
        events = _read_csv(events_path) if events_path.is_file() else []
    summary = summarize_contact_rows(rows, events=events, **kwargs)
    if output_path is not None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv")
    parser.add_argument("--events", required=True, help="events.csv containing settle boundaries")
    parser.add_argument("--output")
    args = parser.parse_args(argv)
    summary = summarize_contact_file(args.input_csv, args.output, events_path=args.events)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
