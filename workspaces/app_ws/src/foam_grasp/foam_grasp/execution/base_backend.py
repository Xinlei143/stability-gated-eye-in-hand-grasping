"""Common execution-backend contracts.

The planning nodes own MoveIt requests and safety validation.  Backends own
the final command transport and expose a small, deliberately stable surface
to those nodes.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ExecutionResult:
    """Normalized result returned by a trajectory or gripper command."""

    duration_sec: float
    final_error: float
    maximum_tracking_error: float = 0.0
    gripper_position: float | None = None

    def __iter__(self):
        """Preserve the historical ``duration, final_error = ...`` API."""
        yield self.duration_sec
        yield self.final_error


class ExecutionBackend:
    """Interface used by the research execution workflows."""

    name = "base"
    feedback_topic = "/joint_states"
    requires_piper_status = False
    is_simulation = False

    def __init__(self, node):
        self.node = node
        self._prepared = False
        self._active = False

    @property
    def execution_prepared(self):
        """Whether the command transport passed its preparation checks."""
        return self._prepared

    @property
    def execution_active(self):
        """Whether a command is currently being sent or awaited."""
        return self._active

    @property
    def can_hold(self):
        """Whether an emergency hold can be issued by this backend."""
        return self._prepared

    def _begin_execution(self):
        if not self._prepared:
            raise RuntimeError(f"{self.name} execution backend is not prepared")
        self._active = True

    def _end_execution(self):
        self._active = False

    def normalize_joint_positions(self, message):
        """Return the canonical seven-position vector or ``None``.

        The canonical names are the six Piper arm joints plus ``gripper``.
        Backends may accept aliases used by a simulator while keeping the
        upper-level planning code independent of those aliases.
        """
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in self.node.command_names):
            return None
        values = [float(positions[name]) for name in self.node.command_names]
        return values if all(map(self.node.is_finite, values)) else None

    def ensure_command_path_is_exclusive(self):
        raise NotImplementedError

    def prepare_execution(self):
        raise NotImplementedError

    def make_command(self, arm_positions, actual_gripper_m, speed_percent, effort):
        raise NotImplementedError

    def hold_position(self, actual_gripper_m, speed_percent, effort):
        raise NotImplementedError

    def execute_arm_trajectory(self, *args, **kwargs):
        raise NotImplementedError

    def execute_cartesian_trajectory(self, *args, **kwargs):
        raise NotImplementedError

    def command_gripper(self, *args, **kwargs):
        raise NotImplementedError

    def send_servo_command(self, *args, **kwargs):
        raise NotImplementedError

    def publish_message(self, message):
        """Publish a backend-native low-level command message."""
        raise NotImplementedError

    def close(self):
        """Release backend-owned ROS resources."""
