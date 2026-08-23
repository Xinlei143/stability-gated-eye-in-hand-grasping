"""Pure qualification summaries for the arm and gripper physics checks."""

import math


ARM_FINAL_ERROR_THRESHOLD_RAD = 0.02
ARM_SETTLED_SPREAD_THRESHOLD_RAD = 0.01
GRIPPER_FINAL_ERROR_THRESHOLD_M = 0.001
GRIPPER_SYMMETRY_THRESHOLD_MM = 1.0
GRIPPER_OSCILLATION_THRESHOLD_MM = 1.0


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
