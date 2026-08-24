"""Small, ROS-independent helpers for depth-fusion benchmark telemetry."""

from __future__ import annotations

import math
from typing import Mapping


def _finite_or_none(value):
    if value is None:
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def build_diagnostic(
    *,
    depth_stamp: float,
    mask_stamp: float | None,
    frame_id: str,
    frame_count: int,
    valid_output_count: int,
    output_rate_hz: float,
    classes: Mapping[str, Mapping[str, object]],
) -> dict[str, object]:
    """Build the stable JSON payload published for every depth frame."""

    mask_age = None
    delta = None
    if mask_stamp is not None:
        mask_age = float(depth_stamp) - float(mask_stamp)
        delta = abs(mask_age)
    return {
        "schema_version": 1,
        "depth_stamp": float(depth_stamp),
        "mask_stamp": None if mask_stamp is None else float(mask_stamp),
        "mask_age_s": mask_age,
        "mask_depth_delta_s": delta,
        "frame_id": str(frame_id),
        "frame_count": int(frame_count),
        "valid_output_count": int(valid_output_count),
        "output_rate_hz": _finite_or_none(output_rate_hz),
        "classes": {str(name): dict(values) for name, values in classes.items()},
    }
