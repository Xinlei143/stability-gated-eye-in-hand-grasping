"""Small, dependency-free records for optional gripper diagnostics."""

from __future__ import annotations


def make_trace_row(
    *,
    wall_time_s,
    joint7_command,
    joint8_command,
    joint7_feedback,
    joint8_feedback,
    joint7_stamp_s,
    joint8_stamp_s,
):
    """Return one trace row without changing or inferring missing feedback."""

    symmetry_error = None
    if joint7_feedback is not None and joint8_feedback is not None:
        symmetry_error = abs(float(joint7_feedback) + float(joint8_feedback))
    return {
        "wall_time_s": wall_time_s,
        "joint7_command": joint7_command,
        "joint8_command": joint8_command,
        "joint7_feedback": joint7_feedback,
        "joint8_feedback": joint8_feedback,
        "joint7_stamp_s": joint7_stamp_s,
        "joint8_stamp_s": joint8_stamp_s,
        "symmetry_error_m": symmetry_error,
    }
