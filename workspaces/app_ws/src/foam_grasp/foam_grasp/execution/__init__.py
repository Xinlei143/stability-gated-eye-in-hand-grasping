"""Hardware-independent execution backends for the foam-grasp workflows."""

from .base_backend import ExecutionBackend, ExecutionResult
from .piper_real_backend import PiperRealBackend


def create_backend(node, name):
    """Create the explicitly requested execution backend.

    Unknown names intentionally fail instead of silently falling back to a
    hardware backend.  The latter would be an unsafe failure mode for a
    command-line typo.
    """
    normalized = str(name).strip().lower()
    if normalized == "real":
        return PiperRealBackend(node)
    if normalized == "simulation":
        # Keep the real-backend commit importable before the optional
        # ros2_control implementation is added in the following commit.
        from .ros2_control_backend import Ros2ControlBackend

        return Ros2ControlBackend(node)
    raise ValueError(
        f"unsupported execution backend {name!r}; "
        "choose 'real' or 'simulation'"
    )


__all__ = [
    "ExecutionBackend",
    "ExecutionResult",
    "PiperRealBackend",
    "create_backend",
]
