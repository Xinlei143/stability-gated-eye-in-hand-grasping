"""Pure, deterministic target-motion profiles for Gazebo experiments."""

from dataclasses import dataclass
import math


TRAJECTORY_PROFILES = (
    "static",
    "constant_velocity",
    "move_stop",
    "move_stop_move",
)


@dataclass(frozen=True)
class MotionSample:
    """Desired planar state at one elapsed simulation-time instant."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    complete: bool


def _finite_vector(name, values):
    vector = tuple(float(value) for value in values)
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{name} must contain three finite values")
    return vector


def validate_motion_parameters(
    profile,
    start_position,
    velocity,
    move_duration,
    stop_duration,
):
    """Validate the fixed stage-3 target-motion contract."""

    if profile not in TRAJECTORY_PROFILES:
        raise ValueError(
            "trajectory must be one of " + ", ".join(TRAJECTORY_PROFILES)
        )
    start = _finite_vector("start_position", start_position)
    velocity = _finite_vector("velocity", velocity)
    if abs(velocity[2]) > 1e-9:
        raise ValueError("stage-3 target motion must remain on the table plane")
    move_duration = float(move_duration)
    stop_duration = float(stop_duration)
    if not math.isfinite(move_duration) or move_duration <= 0.0:
        raise ValueError("move_duration must be positive and finite")
    if not math.isfinite(stop_duration) or stop_duration < 0.0:
        raise ValueError("stop_duration must be non-negative and finite")
    return start, velocity, move_duration, stop_duration


def _translate(position, velocity, seconds):
    return tuple(
        position[index] + velocity[index] * seconds
        for index in range(3)
    )


def sample_motion(
    profile,
    start_position,
    velocity,
    move_duration,
    stop_duration,
    elapsed_seconds,
):
    """Return the desired state for a named profile at elapsed time.

    ``move_stop`` and ``move_stop_move`` return ``complete=True`` once their
    final zero-velocity state has been requested.  The Gazebo node sends that
    final state once, then releases the physical object for later grasping.
    """

    start, velocity, move_duration, stop_duration = validate_motion_parameters(
        profile,
        start_position,
        velocity,
        move_duration,
        stop_duration,
    )
    elapsed = max(0.0, float(elapsed_seconds))
    zero = (0.0, 0.0, 0.0)

    if profile == "static":
        return MotionSample(start, zero, complete=False)
    if profile == "constant_velocity":
        return MotionSample(
            _translate(start, velocity, elapsed), velocity, complete=False
        )

    first_end = _translate(start, velocity, move_duration)
    if profile == "move_stop":
        if elapsed < move_duration:
            return MotionSample(
                _translate(start, velocity, elapsed), velocity, complete=False
            )
        return MotionSample(first_end, zero, complete=True)

    if elapsed < move_duration:
        return MotionSample(
            _translate(start, velocity, elapsed), velocity, complete=False
        )
    if elapsed < move_duration + stop_duration:
        return MotionSample(first_end, zero, complete=False)
    second_elapsed = elapsed - move_duration - stop_duration
    second_end = _translate(first_end, velocity, move_duration)
    if second_elapsed < move_duration:
        return MotionSample(
            _translate(first_end, velocity, second_elapsed),
            velocity,
            complete=False,
        )
    return MotionSample(second_end, zero, complete=True)
