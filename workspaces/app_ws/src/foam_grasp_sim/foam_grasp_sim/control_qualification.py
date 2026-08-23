"""Pure qualification summaries for the arm and gripper physics checks."""

import math
from statistics import median


ARM_FINAL_ERROR_THRESHOLD_RAD = 0.02
ARM_SETTLED_SPREAD_THRESHOLD_RAD = 0.01
GRIPPER_FINAL_ERROR_THRESHOLD_M = 0.001
GRIPPER_SYMMETRY_THRESHOLD_MM = 1.0
GRIPPER_OSCILLATION_THRESHOLD_MM = 1.0
LOADED_GRIPPER_MINIMUM_FORCE_N = 0.8
LOADED_GRIPPER_MAX_P95_FORCE_N = 3.0
LOADED_GRIPPER_MINIMUM_BILATERAL_FRACTION = 0.8
LOADED_GRIPPER_MINIMUM_BILATERAL_DURATION_S = 0.8


def _finite_values(values):
    return all(math.isfinite(float(value)) for value in values)


def summarize_arm_run(target, samples, error_code):
    """Return deterministic pass/fail metrics for one arm trajectory."""

    target = [float(value) for value in target]
    samples = [[float(value) for value in sample] for sample in samples]
    if len(target) != 6 or not samples or any(len(sample) != 6 for sample in samples):
        raise ValueError("arm target and feedback samples must contain six values")
    if not _finite_values(target) or any(not _finite_values(sample) for sample in samples):
        raise ValueError("arm target and feedback samples must be finite")

    final = samples[-1]
    final_errors = [abs(actual - desired) for actual, desired in zip(final, target)]
    settled = samples[-10:]
    spreads = [
        max(sample[index] for sample in settled)
        - min(sample[index] for sample in settled)
        for index in range(6)
    ]
    max_final_error = max(final_errors)
    joint6_error = final_errors[5]
    settled_spread = max(spreads)
    return {
        "passed": (
            int(error_code) == 0
            and max_final_error < ARM_FINAL_ERROR_THRESHOLD_RAD
            and joint6_error < ARM_FINAL_ERROR_THRESHOLD_RAD
            and settled_spread < ARM_SETTLED_SPREAD_THRESHOLD_RAD
        ),
        "error_code": int(error_code),
        "max_final_error_rad": max_final_error,
        "joint6_final_error_rad": joint6_error,
        "settled_max_spread_rad": settled_spread,
    }


def summarize_gripper_run(samples, target_joint7, target_joint8):
    """Return completion, symmetry, oscillation and effort metrics for gripper feedback."""

    if not samples:
        raise ValueError("gripper feedback samples must not be empty")
    target_joint7 = float(target_joint7)
    target_joint8 = float(target_joint8)
    required = ("joint7", "joint8", "effort7", "effort8")
    if not _finite_values((target_joint7, target_joint8)):
        raise ValueError("gripper targets must be finite")
    normalized = []
    for sample in samples:
        if any(name not in sample for name in required):
            raise ValueError("gripper samples must contain joint and effort fields")
        values = {name: float(sample[name]) for name in required}
        if not _finite_values(values.values()):
            effort_finite = False
        else:
            effort_finite = True
        values["effort_finite"] = effort_finite
        normalized.append(values)

    final = normalized[-1]
    final_error_mm = max(
        abs(final["joint7"] - target_joint7),
        abs(final["joint8"] - target_joint8),
    ) * 1000.0
    symmetry_errors_mm = [abs(sample["joint7"] + sample["joint8"]) * 1000.0 for sample in normalized]
    recent = normalized[-10:]
    oscillation_mm = max(
        max(sample[name] for sample in recent) - min(sample[name] for sample in recent)
        for name in ("joint7", "joint8")
    ) * 1000.0
    symmetry_error_mm = max(symmetry_errors_mm[-10:])
    effort_finite = all(sample["effort_finite"] for sample in normalized)
    return {
        "passed": (
            final_error_mm <= GRIPPER_FINAL_ERROR_THRESHOLD_M * 1000.0
            and symmetry_error_mm <= GRIPPER_SYMMETRY_THRESHOLD_MM
            and oscillation_mm <= GRIPPER_OSCILLATION_THRESHOLD_MM
            and effort_finite
        ),
        "final_error_mm": final_error_mm,
        "symmetry_error_mm": symmetry_error_mm,
        "settled_oscillation_mm": oscillation_mm,
        "effort_finite": effort_finite,
    }


def summarize_loaded_gripper_run(
    samples,
    target_joint7,
    target_joint8,
    *,
    minimum_force_N=LOADED_GRIPPER_MINIMUM_FORCE_N,
    maximum_p95_force_N=LOADED_GRIPPER_MAX_P95_FORCE_N,
    minimum_bilateral_fraction=LOADED_GRIPPER_MINIMUM_BILATERAL_FRACTION,
    minimum_bilateral_duration_s=LOADED_GRIPPER_MINIMUM_BILATERAL_DURATION_S,
    hold_s=1.0,
):
    """Qualify a closed gripper using simultaneous left/right contact force."""

    if not samples:
        raise ValueError("loaded gripper feedback samples must not be empty")
    required = ("joint7", "joint8", "effort7", "effort8", "left_force_N", "right_force_N")
    normalized = []
    for sample in samples:
        if any(name not in sample for name in required):
            raise ValueError("loaded gripper samples are missing force fields")
        values = {name: float(sample[name]) for name in required}
        if not _finite_values(values.values()):
            raise ValueError("loaded gripper samples must be finite")
        normalized.append(values)

    left = [max(sample["left_force_N"], 0.0) for sample in normalized]
    right = [max(sample["right_force_N"], 0.0) for sample in normalized]
    bilateral = [
        left_force >= float(minimum_force_N)
        and right_force >= float(minimum_force_N)
        for left_force, right_force in zip(left, right)
    ]
    half_start = len(normalized) // 2
    left_hold = left[half_start:]
    right_hold = right[half_start:]
    left_median = float(median(left_hold))
    right_median = float(median(right_hold))
    timestamps = []
    missing_timestamp = False
    for sample in samples:
        try:
            timestamp = float(sample["sim_time_ns"])
        except (KeyError, TypeError, ValueError):
            missing_timestamp = True
            break
        if not math.isfinite(timestamp):
            missing_timestamp = True
            break
        timestamps.append(timestamp)
    actual_duration_s = (
        max(0.0, (timestamps[-1] - timestamps[0]) / 1e9)
        if timestamps else 0.0
    )
    step_s = actual_duration_s / max(len(timestamps) - 1, 1)
    longest = 0
    current = 0
    for is_bilateral in bilateral:
        current = current + 1 if is_bilateral else 0
        longest = max(longest, current)
    longest_bilateral_s = max(0.0, (longest - 1) * step_s)
    failure_reasons = []
    if missing_timestamp or len(timestamps) != len(normalized):
        failure_reasons.append("missing_sim_time")
    elif any(later < earlier for earlier, later in zip(timestamps, timestamps[1:])):
        failure_reasons.append("non_monotonic_sim_time")
    if actual_duration_s + 1e-9 < float(hold_s):
        failure_reasons.append("hold_window_short")
    bilateral_stable_fraction = sum(bilateral) / len(bilateral)
    if bilateral_stable_fraction <= 0.0:
        failure_reasons.append("missing_bilateral_force")
    if bilateral_stable_fraction < float(minimum_bilateral_fraction):
        failure_reasons.append("unstable_bilateral_force")
    if left_median < float(minimum_force_N):
        failure_reasons.append("low_left_second_half_median_force")
    if right_median < float(minimum_force_N):
        failure_reasons.append("low_right_second_half_median_force")
    if longest_bilateral_s < float(minimum_bilateral_duration_s):
        failure_reasons.append("bilateral_contact_too_short")
    if _percentile(left_hold, 0.95) > float(maximum_p95_force_N):
        failure_reasons.append("left_force_spike")
    if _percentile(right_hold, 0.95) > float(maximum_p95_force_N):
        failure_reasons.append("right_force_spike")
    gripper = summarize_gripper_run(
        normalized,
        target_joint7=target_joint7,
        target_joint8=target_joint8,
    )
    # A loaded block intentionally prevents the fingers from reaching the
    # commanded free-space opening.  Retain symmetry/oscillation/effort checks
    # from the generic summary, but do not reject the run for target-position
    # error caused by the fixture width.
    if not gripper["effort_finite"]:
        failure_reasons.append("non_finite_effort")
    if gripper["symmetry_error_mm"] > GRIPPER_SYMMETRY_THRESHOLD_MM:
        failure_reasons.append("gripper_asymmetry")
    if gripper["settled_oscillation_mm"] > GRIPPER_OSCILLATION_THRESHOLD_MM:
        failure_reasons.append("gripper_oscillation")
    return {
        **gripper,
        "passed": not failure_reasons,
        "left_median_force_N": left_median,
        "right_median_force_N": right_median,
        "left_p95_force_N": _percentile(left_hold, 0.95),
        "right_p95_force_N": _percentile(right_hold, 0.95),
        "bilateral_stable_fraction": bilateral_stable_fraction,
        "minimum_bilateral_fraction": float(minimum_bilateral_fraction),
        "longest_contiguous_bilateral_contact_s": longest_bilateral_s,
        "minimum_bilateral_duration_s": float(minimum_bilateral_duration_s),
        "hold_s": float(hold_s),
        "actual_duration_s": actual_duration_s,
        "minimum_force_N": float(minimum_force_N),
        "maximum_p95_force_N": float(maximum_p95_force_N),
        "failure_reasons": failure_reasons,
    }


def _percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return float(ordered[index])
