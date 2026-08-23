"""Static grasp-hold resampling, qualification, classification and reporting."""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path
from statistics import median


DEFAULT_FORCE_THRESHOLD_N = 0.8
DEFAULT_GRID_HZ = 100.0
DEFAULT_POSITION_ERROR_THRESHOLD_M = 0.001
# Effort below this is treated as controller/noise-level activity for Case B;
# values above it can still be mechanically insufficient and therefore map to
# Case C when contact persists but force remains low.
DEFAULT_EFFORT_THRESHOLD_N = 0.005
DEFAULT_FRACTION_THRESHOLD = 0.8
DEFAULT_DURATION_THRESHOLD_S = 0.8


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _number(value, default=0.0):
    return float(value) if _finite(value) else float(default)


def _percentile(values, fraction):
    ordered = sorted(float(value) for value in values if _finite(value))
    if not ordered:
        return 0.0
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])


def _last_sample(samples, timestamp_ns, fields):
    """Return the most recent raw sample at or before ``timestamp_ns``."""

    selected = None
    for sample in samples:
        if int(sample.get("sim_time_ns", -1)) <= int(timestamp_ns):
            selected = sample
        else:
            break
    if selected is None and samples:
        selected = samples[0]
    return selected or {field: 0.0 for field in fields}


def _grid_timestamps(start_ns, end_ns, grid_hz):
    if end_ns < start_ns:
        raise ValueError("end_ns must not precede start_ns")
    step_ns = int(round(1e9 / float(grid_hz)))
    if step_ns <= 0:
        raise ValueError("grid_hz must be positive")
    count = int(math.floor((int(end_ns) - int(start_ns)) / step_ns + 0.5)) + 1
    return [int(start_ns) + index * step_ns for index in range(count)]


def _contact_grid(contacts, start_ns, end_ns, grid_hz):
    timestamps = _grid_timestamps(start_ns, end_ns, grid_hz)
    step_ns = int(round(1e9 / float(grid_hz)))
    values = {timestamp: {"left": 0.0, "right": 0.0} for timestamp in timestamps}
    for row in sorted(contacts, key=lambda item: int(item.get("sim_time_ns", 0))):
        try:
            stamp = int(row["sim_time_ns"])
            side = str(row["side"])
            force = max(0.0, _number(row.get("normal_force_N")))
        except (KeyError, TypeError, ValueError):
            continue
        if side not in ("left", "right") or stamp < start_ns or stamp > end_ns:
            continue
        index = int(math.floor((stamp - int(start_ns)) / step_ns + 0.5))
        if 0 <= index < len(timestamps):
            values[timestamps[index]][side] += force
    return values


def build_diagnostics_rows(joints, contacts, cube_samples, *, start_ns, end_ns,
                           grid_hz=DEFAULT_GRID_HZ):
    """Resample raw asynchronous records onto one simulation-time grid.

    Joint and cube values use the last raw sample at or before each grid point.
    Contact values are independently binned by their original message
    timestamp; a grid cell with no contact message is explicitly zero-filled.
    """

    joint_fields = (
        "joint7_position_m", "joint8_position_m", "joint7_velocity_m_s",
        "joint8_velocity_m_s", "joint7_effort_N", "joint8_effort_N",
        "commanded_joint7_position_m", "commanded_joint8_position_m", "stage",
    )
    cube_fields = ("x_m", "y_m", "z_m")
    joints = sorted(joints, key=lambda item: int(item.get("sim_time_ns", 0)))
    cube_samples = sorted(cube_samples, key=lambda item: int(item.get("sim_time_ns", 0)))
    contacts_by_stamp = _contact_grid(contacts, start_ns, end_ns, grid_hz)
    rows = []
    for timestamp in _grid_timestamps(start_ns, end_ns, grid_hz):
        joint = _last_sample(joints, timestamp, joint_fields)
        cube = _last_sample(cube_samples, timestamp, cube_fields)
        contact = contacts_by_stamp[timestamp]
        joint7 = _number(joint.get("joint7_position_m"))
        joint8 = _number(joint.get("joint8_position_m"))
        rows.append({
            "timestamp_sim_time_ns": timestamp,
            "time_s": (timestamp - int(start_ns)) / 1e9,
            "stage": str(joint.get("stage", "hold")),
            "joint7_position_m": joint7,
            "joint8_position_m": joint8,
            "joint7_velocity_m_s": _number(joint.get("joint7_velocity_m_s")),
            "joint8_velocity_m_s": _number(joint.get("joint8_velocity_m_s")),
            "joint7_effort_N": _number(joint.get("joint7_effort_N")),
            "joint8_effort_N": _number(joint.get("joint8_effort_N")),
            "commanded_joint7_position_m": _number(
                joint.get("commanded_joint7_position_m")
            ),
            "commanded_joint8_position_m": _number(
                joint.get("commanded_joint8_position_m")
            ),
            "gripper_total_opening_m": joint7 - joint8,
            "left_contact_normal_force_N": contact["left"],
            "right_contact_normal_force_N": contact["right"],
            "cube_x_m": _number(cube.get("x_m")),
            "cube_y_m": _number(cube.get("y_m")),
            "cube_z_m": _number(cube.get("z_m")),
        })
    return rows


def _longest_duration(values, step_s):
    longest = current = 0
    for value in values:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return max(0.0, (longest - 1) * float(step_s))


def _correlation(first, second):
    if len(first) != len(second) or len(first) < 2:
        return 0.0
    mean_first = sum(first) / len(first)
    mean_second = sum(second) / len(second)
    numerator = sum((a - mean_first) * (b - mean_second) for a, b in zip(first, second))
    denominator_first = sum((a - mean_first) ** 2 for a in first)
    denominator_second = sum((b - mean_second) ** 2 for b in second)
    denominator = math.sqrt(denominator_first * denominator_second)
    return float(numerator / denominator) if denominator > 0.0 else 0.0


def classify_diagnosis(flags):
    """Classify the effort/contact relationship using the requested cases."""

    if not flags.get("position_error_sustained", False):
        return "insufficient_position_error_evidence"
    if not flags.get("effort_sustained", False):
        return "Case B"
    if not flags.get("contact_persistent", False):
        return "Case A"
    if flags.get("contact_force_low", False):
        return "Case C"
    return "Case D"


def summarize_static_hold(rows, *, hold_s, force_threshold_N=DEFAULT_FORCE_THRESHOLD_N,
                          position_error_threshold_m=DEFAULT_POSITION_ERROR_THRESHOLD_M,
                          effort_threshold_N=DEFAULT_EFFORT_THRESHOLD_N):
    """Compute static-hold metrics and the automatic A/B/C/D classification."""

    if not rows:
        raise ValueError("static hold diagnostics must contain at least one row")
    half_start = len(rows) // 2
    second_half = rows[half_start:]
    left = [_number(row.get("left_contact_normal_force_N")) for row in rows]
    right = [_number(row.get("right_contact_normal_force_N")) for row in rows]
    bilateral_nonzero = [left_value > 0.0 and right_value > 0.0
                         for left_value, right_value in zip(left, right)]
    bilateral_qualified = [left_value >= force_threshold_N and right_value >= force_threshold_N
                           for left_value, right_value in zip(left, right)]
    efforts7 = [_number(row.get("joint7_effort_N")) for row in rows]
    efforts8 = [_number(row.get("joint8_effort_N")) for row in rows]
    abs_efforts = [max(abs(a), abs(b)) for a, b in zip(efforts7, efforts8)]
    errors = [
        (abs(_number(row.get("joint7_position_m"))
             - _number(row.get("commanded_joint7_position_m")))
         + abs(_number(row.get("joint8_position_m"))
               - _number(row.get("commanded_joint8_position_m")))) / 2.0
        for row in rows
    ]
    effort_contact = [
        (abs(a) + abs(b)) / 2.0 for a, b in zip(efforts7, efforts8)
    ]
    contact_mean = [(a + b) / 2.0 for a, b in zip(left, right)]
    step_s = 1.0 / DEFAULT_GRID_HZ
    cube_start = tuple(_number(rows[0].get(field)) for field in ("cube_x_m", "cube_y_m", "cube_z_m"))
    cube_end = tuple(_number(rows[-1].get(field)) for field in ("cube_x_m", "cube_y_m", "cube_z_m"))
    cube_displacement = math.sqrt(sum((end - start) ** 2 for start, end in zip(cube_start, cube_end)))
    position_error_sustained = sum(error >= position_error_threshold_m for error in errors) / len(errors) >= 0.5
    effort_sustained = sum(value >= effort_threshold_N for value in abs_efforts) / len(abs_efforts) >= 0.8
    contact_persistent = sum(bilateral_nonzero) / len(bilateral_nonzero) >= DEFAULT_FRACTION_THRESHOLD
    contact_force_low = (
        contact_persistent
        and median([*left, *right]) < force_threshold_N
    )
    flags = {
        "position_error_sustained": position_error_sustained,
        "effort_sustained": effort_sustained,
        "contact_persistent": contact_persistent,
        "contact_force_low": contact_force_low,
    }
    qualified_fraction = sum(bilateral_qualified) / len(bilateral_qualified)
    nonzero_fraction = sum(bilateral_nonzero) / len(bilateral_nonzero)
    summary = {
        "hold_s": float(hold_s),
        "sample_count": len(rows),
        "hold_duration_s": _number(rows[-1].get("time_s")),
        "median_joint7_effort_N": float(median([row["joint7_effort_N"] for row in second_half])),
        "median_joint8_effort_N": float(median([row["joint8_effort_N"] for row in second_half])),
        "median_abs_joint7_effort_N": float(median([abs(row["joint7_effort_N"]) for row in second_half])),
        "median_abs_joint8_effort_N": float(median([abs(row["joint8_effort_N"]) for row in second_half])),
        "p95_absolute_joint_effort_N": _percentile(abs_efforts, 0.95),
        "median_left_contact_normal_force_N": float(median(left)),
        "median_right_contact_normal_force_N": float(median(right)),
        "p95_contact_force_N": _percentile([*left, *right], 0.95),
        "bilateral_nonzero_fraction": nonzero_fraction,
        "longest_bilateral_nonzero_duration_s": _longest_duration(bilateral_nonzero, step_s),
        "bilateral_force_qualified_fraction": qualified_fraction,
        "longest_bilateral_force_qualified_duration_s": _longest_duration(bilateral_qualified, step_s),
        "force_threshold_N": float(force_threshold_N),
        "mean_finger_position_error_m": sum(errors) / len(errors),
        "p95_finger_position_error_m": _percentile(errors, 0.95),
        "effort_contact_correlation": _correlation(effort_contact, contact_mean),
        "cube_displacement_during_hold_m": cube_displacement,
        "cube_start_xyz_m": list(cube_start),
        "cube_end_xyz_m": list(cube_end),
        "static_hold_passed": (
            qualified_fraction >= DEFAULT_FRACTION_THRESHOLD
            and _longest_duration(bilateral_qualified, step_s) >= DEFAULT_DURATION_THRESHOLD_S
        ),
        "position_error_sustained": position_error_sustained,
        "effort_sustained": effort_sustained,
        "contact_persistent": contact_persistent,
        "contact_force_low": contact_force_low,
    }
    summary["diagnosis_case"] = classify_diagnosis(flags)
    summary["diagnosis_flags"] = flags
    return summary


def diagnostics_fields():
    return [
        "timestamp_sim_time_ns", "time_s", "stage", "joint7_position_m", "joint8_position_m",
        "joint7_velocity_m_s", "joint8_velocity_m_s", "joint7_effort_N", "joint8_effort_N",
        "commanded_joint7_position_m", "commanded_joint8_position_m", "gripper_total_opening_m",
        "left_contact_normal_force_N", "right_contact_normal_force_N", "cube_x_m", "cube_y_m", "cube_z_m",
    ]


def write_diagnostics_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=diagnostics_fields())
        writer.writeheader()
        writer.writerows({field: row.get(field, "") for field in diagnostics_fields()} for row in rows)


def plot_diagnostics(path, rows, *, hold_s, force_threshold_N=DEFAULT_FORCE_THRESHOLD_N,
                     diagnosis_case=""):
    """Render the effort/contact panels without importing matplotlib at module load."""

    import matplotlib.pyplot as plt

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    times = [_number(row.get("time_s")) for row in rows]
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6), constrained_layout=True)
    axes[0].plot(times, [_number(row.get("joint7_effort_N")) for row in rows], label="joint7 effort")
    axes[0].plot(times, [_number(row.get("joint8_effort_N")) for row in rows], label="joint8 effort")
    axes[0].set_ylabel("effort [N]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)
    axes[1].plot(times, [_number(row.get("left_contact_normal_force_N")) for row in rows], label="left contact")
    axes[1].plot(times, [_number(row.get("right_contact_normal_force_N")) for row in rows], label="right contact")
    axes[1].axhline(float(force_threshold_N), color="tab:red", linestyle="--", label=f"threshold {force_threshold_N:g} N")
    axes[1].set_ylabel("normal force [N]")
    axes[1].set_xlabel("simulation time from hold start [s]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)
    for axis in axes:
        axis.axvline(0.0, color="black", linestyle=":", linewidth=0.9)
        axis.axvline(float(hold_s), color="black", linestyle=":", linewidth=0.9)
    axes[0].set_title(f"Static grasp-hold effort/contact diagnosis {diagnosis_case}".strip())
    fig.savefig(path, dpi=150)
    plt.close(fig)


def write_sweep_summary_csv(path, entries):
    """Aggregate repeat summaries by Kp into the requested comparison CSV."""

    grouped = defaultdict(list)
    for entry in entries:
        grouped[float(entry["kp"])].append(entry.get("summary", {}))
    fields = [
        "Kp", "repeat_count", "median_joint_effort_N", "median_joint7_effort_N",
        "median_joint8_effort_N", "median_contact_force_N", "median_left_contact_force_N",
        "median_right_contact_force_N", "bilateral_nonzero_fraction",
        "longest_bilateral_duration_s", "p95_contact_force_N",
        "longest_bilateral_force_qualified_duration_s", "static_grasp_qualification",
        "diagnosis_cases",
    ]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for kp in sorted(grouped):
            summaries = grouped[kp]
            def values(name):
                return [_number(summary.get(name)) for summary in summaries if name in summary]
            def middle(name):
                candidates = values(name)
                return median(candidates) if candidates else 0.0
            left = values("median_left_contact_normal_force_N")
            right = values("median_right_contact_normal_force_N")
            effort7 = values("median_joint7_effort_N")
            effort8 = values("median_joint8_effort_N")
            contact = [sum(pair) / 2.0 for pair in zip(left, right)]
            writer.writerow({
                "Kp": kp,
                "repeat_count": len(summaries),
                "median_joint_effort_N": median([*map(abs, effort7), *map(abs, effort8)]) if effort7 and effort8 else 0.0,
                "median_joint7_effort_N": median(effort7) if effort7 else 0.0,
                "median_joint8_effort_N": median(effort8) if effort8 else 0.0,
                "median_contact_force_N": median(contact) if contact else 0.0,
                "median_left_contact_force_N": median(left) if left else 0.0,
                "median_right_contact_force_N": median(right) if right else 0.0,
                "bilateral_nonzero_fraction": middle("bilateral_nonzero_fraction"),
                "longest_bilateral_duration_s": middle("longest_bilateral_nonzero_duration_s"),
                "p95_contact_force_N": middle("p95_contact_force_N"),
                "longest_bilateral_force_qualified_duration_s": middle("longest_bilateral_force_qualified_duration_s"),
                "static_grasp_qualification": str(all(bool(summary.get("static_hold_passed")) for summary in summaries)).lower(),
                "diagnosis_cases": ";".join(sorted({str(summary.get("diagnosis_case", "")) for summary in summaries})),
            })
