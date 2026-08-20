#!/usr/bin/env python3
"""Drive one static Gazebo target along a deterministic planar trajectory."""

import math

import rclpy
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from foam_grasp_sim.motion_profiles import (
    TRAJECTORY_PROFILES,
    sample_motion,
    validate_motion_parameters,
)


TARGET_MODELS = ("cube", "cylinder", "sphere")


class TargetMotionNode(Node):
    """Move a selected target and publish its actual Gazebo position."""

    def __init__(self):
        super().__init__("foam_target_motion")
        self.declare_parameter("target_model", "cube")
        self.declare_parameter("entity_name", "foam_cube")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("start_position", [0.40, 0.00, 0.026])
        self.declare_parameter("trajectory", "static")
        self.declare_parameter("velocity_x", 0.01)
        self.declare_parameter("velocity_y", 0.00)
        self.declare_parameter("velocity_z", 0.00)
        self.declare_parameter("move_duration", 4.0)
        self.declare_parameter("stop_duration", 6.0)
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("ground_truth_rate", 30.0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("set_entity_state_service", "/gazebo/set_entity_state")

        self.target_model = str(self.get_parameter("target_model").value)
        if self.target_model not in TARGET_MODELS:
            raise RuntimeError(
                "target_model must be one of " + ", ".join(TARGET_MODELS)
            )
        self.entity_name = str(self.get_parameter("entity_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        if not self.entity_name or not self.base_frame:
            raise RuntimeError("entity_name and base_frame must not be empty")
        self.trajectory = str(self.get_parameter("trajectory").value)
        try:
            (
                self.start_position,
                self.velocity,
                self.move_duration,
                self.stop_duration,
            ) = validate_motion_parameters(
                self.trajectory,
                self.get_parameter("start_position").value,
                [
                    self.get_parameter("velocity_x").value,
                    self.get_parameter("velocity_y").value,
                    self.get_parameter("velocity_z").value,
                ],
                self.get_parameter("move_duration").value,
                self.get_parameter("stop_duration").value,
            )
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        self.control_rate = self._positive_rate("control_rate")
        self.ground_truth_rate = self._positive_rate("ground_truth_rate")
        self.seed = int(self.get_parameter("seed").value)
        if not 0 <= self.seed <= 2**31 - 1:
            raise RuntimeError("seed must be within [0, 2147483647]")

        self.actual_position = None
        self.trajectory_started_at = None
        self.completion_command_sent = False
        self.pending_set_state = None
        self.last_service_warning_at = -math.inf

        model_states_topic = str(self.get_parameter("model_states_topic").value)
        service_name = str(self.get_parameter("set_entity_state_service").value)
        self.model_states_subscription = self.create_subscription(
            ModelStates,
            model_states_topic,
            self.model_states_callback,
            20,
        )
        self.set_entity_state_client = self.create_client(
            SetEntityState,
            service_name,
        )
        self.ground_truth_publisher = self.create_publisher(
            PointStamped,
            "/foam_grasp_sim/target_ground_truth",
            20,
        )
        self.motion_timer = self.create_timer(
            1.0 / self.control_rate,
            self.motion_tick,
        )
        self.ground_truth_timer = self.create_timer(
            1.0 / self.ground_truth_rate,
            self.publish_ground_truth,
        )
        self.get_logger().info(
            "Target motion: entity=%s, trajectory=%s, velocity=(%.3f, %.3f, %.3f), seed=%d"
            % (self.entity_name, self.trajectory, *self.velocity, self.seed)
        )

    def _positive_rate(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not 1.0 <= value <= 200.0:
            raise RuntimeError(f"{name} must be finite and within 1--200 Hz")
        return value

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def model_states_callback(self, message):
        try:
            index = message.name.index(self.entity_name)
        except ValueError:
            return
        pose = message.pose[index]
        position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        if not all(math.isfinite(value) for value in position):
            return
        self.actual_position = position
        if self.trajectory_started_at is None:
            self.trajectory_started_at = self.now_seconds()
            self.get_logger().info(
                f"Detected {self.entity_name}; trajectory clock started"
            )

    def motion_tick(self):
        if self.trajectory == "static" or self.trajectory_started_at is None:
            return
        if self.pending_set_state is not None:
            return
        elapsed = self.now_seconds() - self.trajectory_started_at
        sample = sample_motion(
            self.trajectory,
            self.start_position,
            self.velocity,
            self.move_duration,
            self.stop_duration,
            elapsed,
        )
        if sample.complete and self.completion_command_sent:
            return
        if not self.set_entity_state_client.service_is_ready():
            self._warn_if_service_missing()
            return

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.entity_name
        request.state.reference_frame = "world"
        request.state.pose.position.x = sample.position[0]
        request.state.pose.position.y = sample.position[1]
        request.state.pose.position.z = sample.position[2]
        request.state.pose.orientation.w = 1.0
        request.state.twist.linear.x = sample.velocity[0]
        request.state.twist.linear.y = sample.velocity[1]
        request.state.twist.linear.z = sample.velocity[2]
        final_command = sample.complete
        if final_command:
            self.completion_command_sent = True
        self.pending_set_state = self.set_entity_state_client.call_async(request)
        self.pending_set_state.add_done_callback(
            lambda future: self.set_state_done(future, final_command)
        )

    def _warn_if_service_missing(self):
        now = self.now_seconds()
        if now - self.last_service_warning_at >= 5.0:
            self.get_logger().warning(
                "Waiting for /gazebo/set_entity_state before moving target"
            )
            self.last_service_warning_at = now

    def set_state_done(self, future, final_command):
        self.pending_set_state = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Gazebo target-state request failed: {error}")
            response = None
        if response is None or not response.success:
            if final_command:
                self.completion_command_sent = False
            message = "unknown Gazebo error" if response is None else response.status_message
            self.get_logger().error(f"Gazebo rejected target state: {message}")
        elif final_command:
            self.get_logger().info("Target trajectory complete; Gazebo object released")

    def publish_ground_truth(self):
        if self.actual_position is None:
            return
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.point.x, message.point.y, message.point.z = self.actual_position
        self.ground_truth_publisher.publish(message)


def main():
    rclpy.init()
    node = TargetMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
