"""Wall-clock ROS readiness gate for the complete simulation pipeline."""

from __future__ import annotations

import argparse
import time
from typing import Callable

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from foam_grasp_sim.simulation_readiness import (
    REQUIRED_ACTION_SERVERS,
    ReadinessSnapshot,
    format_missing_conditions,
    missing_conditions,
)


class SimulationReadinessNode(Node):
    """Wait for controller, feedback, and action readiness using wall time."""

    def __init__(
        self,
        *,
        timeout_s: float = 30.0,
        poll_period_s: float = 0.1,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__("simulation_readiness")
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if poll_period_s <= 0.0:
            raise ValueError("poll_period_s must be positive")
        self._timeout_s = float(timeout_s)
        self._poll_period_s = float(poll_period_s)
        self._monotonic = monotonic
        self._joint_names: set[str] = set()
        self._active_controllers: set[str] = set()
        self._controller_service_available = False
        self._list_controllers_future = None
        self._ready_action_servers: set[str] = set()

        self._controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._action_clients = {
            name: ActionClient(self, FollowJointTrajectory, name)
            for name in REQUIRED_ACTION_SERVERS
        }
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 20
        )

    def _joint_state_callback(self, message: JointState) -> None:
        # Gate on one received feedback message containing the full arm and
        # gripper state, rather than accumulating names from unrelated partial
        # publishers across time.
        self._joint_names = set(message.name)

    def _poll_controller_state(self) -> None:
        self._controller_service_available = self._controller_client.service_is_ready()
        if not self._controller_service_available:
            self._list_controllers_future = None
            self._active_controllers.clear()
            return
        if self._list_controllers_future is None:
            self._list_controllers_future = self._controller_client.call_async(
                ListControllers.Request()
            )
            return
        if not self._list_controllers_future.done():
            return
        try:
            response = self._list_controllers_future.result()
        except Exception as error:  # pragma: no cover - middleware failure path
            self.get_logger().warning(f"list_controllers request failed: {error}")
            self._active_controllers.clear()
            self._list_controllers_future = None
            return
        self._active_controllers = {
            controller.name
            for controller in response.controller
            if controller.state == "active"
        }
        self._list_controllers_future = None

    def _poll_action_servers(self) -> None:
        self._ready_action_servers = {
            name
            for name, client in self._action_clients.items()
            if client.server_is_ready()
        }

    def snapshot(self) -> ReadinessSnapshot:
        return ReadinessSnapshot(
            controller_service_available=self._controller_service_available,
            active_controllers=frozenset(self._active_controllers),
            joint_names=frozenset(self._joint_names),
            ready_action_servers=frozenset(self._ready_action_servers),
        )

    def wait_until_ready(self) -> int:
        deadline = self._monotonic() + self._timeout_s
        last_missing: tuple[str, ...] = ()
        while rclpy.ok():
            self._poll_controller_state()
            self._poll_action_servers()
            last_missing = missing_conditions(self.snapshot())
            if not last_missing:
                self.get_logger().info(
                    "simulation readiness complete; controllers, joint states, "
                    "and trajectory action servers are ready"
                )
                return 0
            remaining = deadline - self._monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(self._poll_period_s, remaining))
        self.get_logger().error(
            "simulation readiness timeout after "
            f"{self._timeout_s:.1f}s; missing: "
            f"{format_missing_conditions(last_missing)}"
        )
        return 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parsed, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = SimulationReadinessNode(timeout_s=parsed.timeout_s)
    try:
        return node.wait_until_ready()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
