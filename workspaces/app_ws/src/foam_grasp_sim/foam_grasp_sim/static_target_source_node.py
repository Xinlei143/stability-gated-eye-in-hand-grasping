#!/usr/bin/env python3
"""Publish an ideal static Gazebo target position for the stage-2 pipeline.

This is deliberately a minimal perception substitute: it republishes the
configured spawn centre in ``base_link`` and does not observe Gazebo, add
noise, delay messages, or implement target motion.
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


TARGET_MODELS = ("cube", "cylinder", "sphere")


class StaticTargetSourceNode(Node):
    """Publish the selected static target centre at a fixed rate."""

    def __init__(self):
        super().__init__("foam_static_target_source")
        self.declare_parameter("target_model", "cube")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("cube_pose", [0.40, 0.00, 0.026])
        self.declare_parameter("cylinder_pose", [0.40, 0.00, 0.036])
        self.declare_parameter("sphere_pose", [0.40, 0.00, 0.031])

        self.target_model = str(self.get_parameter("target_model").value)
        if self.target_model not in TARGET_MODELS:
            raise RuntimeError(
                "target_model must be one of " + ", ".join(TARGET_MODELS)
            )
        self.base_frame = str(self.get_parameter("base_frame").value)
        if not self.base_frame:
            raise RuntimeError("base_frame must not be empty")
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise RuntimeError("publish_rate must be a positive finite value")

        self.target_poses = {
            name: self._read_pose(name)
            for name in TARGET_MODELS
        }
        self.target_pose = self.target_poses[self.target_model]
        self.topic = f"/foam_grasp/{self.target_model}_point_base"
        self.publisher = self.create_publisher(PointStamped, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)
        self.get_logger().info(
            "Publishing ideal static %s target on %s: "
            "(%.3f, %.3f, %.3f) in %s"
            % (
                self.target_model,
                self.topic,
                *self.target_pose,
                self.base_frame,
            )
        )

    def _read_pose(self, target_model):
        values = list(self.get_parameter(f"{target_model}_pose").value)
        if len(values) != 3:
            raise RuntimeError(f"{target_model}_pose must contain [x, y, z]")
        pose = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in pose):
            raise RuntimeError(f"{target_model}_pose must contain finite values")
        return pose

    def publish(self):
        message = PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.point.x, message.point.y, message.point.z = self.target_pose
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = StaticTargetSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
