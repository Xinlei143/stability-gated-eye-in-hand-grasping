"""Validation and naming contract for simulator grasp stabilization backends."""

from dataclasses import dataclass


MODES = frozenset(("off", "gazebo_grasp_fix", "legacy_contact_confirmed"))


@dataclass(frozen=True)
class StabilizationSelection:
    mode: str
    legacy_assist_enabled: bool


def resolve_stabilization_mode(
    mode: str,
    grasp_assist_mode: str = "off",
    grasp_assist_service: str = "",
) -> StabilizationSelection:
    """Validate one launch configuration and return its effective backend."""

    mode = str(mode).strip().lower()
    assist_mode = str(grasp_assist_mode).strip().lower()
    service = str(grasp_assist_service).strip()
    if mode not in MODES:
        raise ValueError(
            "grasp_stabilization_mode must be off, gazebo_grasp_fix, "
            "or legacy_contact_confirmed"
        )
    if assist_mode not in {"off", "contact_confirmed"}:
        raise ValueError("grasp_assist_mode must be off or contact_confirmed")
    if mode == "gazebo_grasp_fix" and (assist_mode != "off" or service):
        raise ValueError(
            "gazebo_grasp_fix and legacy grasp assist are mutually exclusive"
        )
    return StabilizationSelection(
        mode=mode,
        legacy_assist_enabled=mode == "legacy_contact_confirmed"
        or assist_mode == "contact_confirmed",
    )
