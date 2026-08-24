"""Pure readiness state evaluation for the ROS simulation startup gate."""

from __future__ import annotations

from dataclasses import dataclass, field


REQUIRED_CONTROLLERS = (
    "joint_state_broadcaster",
    "arm_controller",
    "gripper_controller",
    "gripper8_controller",
)
REQUIRED_JOINTS = tuple(f"joint{index}" for index in range(1, 9))
REQUIRED_ACTION_SERVERS = (
    "/arm_controller/follow_joint_trajectory",
    "/gripper_controller/follow_joint_trajectory",
    "/gripper8_controller/follow_joint_trajectory",
)


@dataclass(frozen=True)
class ReadinessSnapshot:
    """Observed ROS state used to decide whether simulation may proceed."""

    controller_service_available: bool = False
    active_controllers: frozenset[str] = field(default_factory=frozenset)
    joint_names: frozenset[str] = field(default_factory=frozenset)
    ready_action_servers: frozenset[str] = field(default_factory=frozenset)


def missing_conditions(snapshot: ReadinessSnapshot) -> tuple[str, ...]:
    """Return stable, human-readable readiness failures in check order."""

    missing: list[str] = []
    if not snapshot.controller_service_available:
        missing.append("/controller_manager/list_controllers unavailable")
    missing.extend(
        f"controller {name} not active"
        for name in REQUIRED_CONTROLLERS
        if name not in snapshot.active_controllers
    )
    missing.extend(
        f"joint_states missing {name}"
        for name in REQUIRED_JOINTS
        if name not in snapshot.joint_names
    )
    missing.extend(
        f"action server {name} not ready"
        for name in REQUIRED_ACTION_SERVERS
        if name not in snapshot.ready_action_servers
    )
    return tuple(missing)


def format_missing_conditions(missing: tuple[str, ...] | list[str]) -> str:
    """Render missing conditions for a single timeout log line."""

    return "; ".join(missing) if missing else "none"


def controller_state_poll_allowed(ready_action_servers: frozenset[str]) -> bool:
    """Return whether the controller-manager service can be queried safely.

    Gazebo's ``gazebo_ros2_control`` plugin advertises the controller-manager
    service while the sequential controller spawners are still bringing up the
    arm.  Querying that service in this short window can make the plugin abort
    while it is still constructing its controller manager.  The trajectory
    action servers are only advertised by the fully configured arm and gripper
    controllers, so they provide a stable startup barrier before the readiness
    node sends its first ``list_controllers`` request.
    """

    return set(REQUIRED_ACTION_SERVERS).issubset(ready_action_servers)
