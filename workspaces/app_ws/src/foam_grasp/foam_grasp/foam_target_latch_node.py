#!/usr/bin/env python3
"""Latch a stable foam-object point in base_link for later grasp planning.

This node is perception-only.  It never publishes a Piper command, never calls
MoveIt, and never enables or disables the arm.
"""

import math
import statistics
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


CLASS_NAMES = ("cube", "cylinder", "sphere")


class FoamTargetLatchNode(Node):
    def __init__(self):
        super().__init__("foam_target_latch")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("sample_window", 1.5)
        self.declare_parameter("minimum_samples", 15)
        self.declare_parameter("maximum_spread", 0.010)
        self.declare_parameter("workspace_x_min", 0.15)
        self.declare_parameter("workspace_x_max", 0.60)
        self.declare_parameter("workspace_y_abs_max", 0.35)
        self.declare_parameter("workspace_z_min", -0.02)
        self.declare_parameter("workspace_z_max", 0.20)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.sample_window = float(
            self.get_parameter("sample_window").value
        )
        self.minimum_samples = int(
            self.get_parameter("minimum_samples").value
        )
        self.maximum_spread = float(
            self.get_parameter("maximum_spread").value
        )
        self.workspace_x_min = float(
            self.get_parameter("workspace_x_min").value
        )
        self.workspace_x_max = float(
            self.get_parameter("workspace_x_max").value
        )
        self.workspace_y_abs_max = float(
            self.get_parameter("workspace_y_abs_max").value
        )
        self.workspace_z_min = float(
            self.get_parameter("workspace_z_min").value
        )
        self.workspace_z_max = float(
            self.get_parameter("workspace_z_max").value
        )

        if self.sample_window <= 0.0:
            raise RuntimeError("sample_window must be positive")
        if self.minimum_samples < 3:
            raise RuntimeError("minimum_samples must be at least 3")
        if self.maximum_spread <= 0.0:
            raise RuntimeError("maximum_spread must be positive")
        self.samples = {
            name: deque(maxlen=300) for name in CLASS_NAMES
        }
        self.latched_class = None
        self.latched_point = None
        self.latched_sample_count = 0
        self.latched_spread = None
        self.point_subscriptions = []

        for name in CLASS_NAMES:
            subscription = self.create_subscription(
                PointStamped,
                f"/foam_grasp/{name}_point_base",
                lambda message, current_name=name: self.point_callback(
                    current_name,
                    message,
                ),
                20,
            )
            self.point_subscriptions.append(subscription)

        self.generic_publisher = self.create_publisher(
            PointStamped,
            "/foam_grasp/target_point_base_latched",
            10,
        )
        self.class_publishers = {
            name: self.create_publisher(
                PointStamped,
                f"/foam_grasp/{name}_point_base_latched",
                10,
            )
            for name in CLASS_NAMES
        }
        self.class_name_publisher = self.create_publisher(
            String,
            "/foam_grasp/latched_target_class",
            10,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/foam_grasp/latched_target_marker",
            10,
        )

        self.latch_services = []
        for name in CLASS_NAMES:
            service = self.create_service(
                Trigger,
                f"/foam_grasp/latch_{name}",
                lambda request, response, current_name=name: self.latch_callback(
                    current_name,
                    request,
                    response,
                ),
            )
            self.latch_services.append(service)
        self.clear_service = self.create_service(
            Trigger,
            "/foam_grasp/clear_latched_target",
            self.clear_callback,
        )

        self.publish_timer = self.create_timer(0.1, self.publish_latched_target)
        self.get_logger().warning(
            "PERCEPTION ONLY: this node cannot command the Piper arm"
        )
        self.get_logger().info(
            "Services: /foam_grasp/latch_cube, latch_cylinder, "
            "latch_sphere, clear_latched_target"
        )

    def point_callback(self, name, message):
        if message.header.frame_id != self.base_frame:
            return
        point = (
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        )
        if not all(math.isfinite(value) for value in point):
            return
        if not self.point_in_workspace(point):
            return
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self.samples[name].append((stamp, point))

    def point_in_workspace(self, point):
        x, y, z = point
        return (
            self.workspace_x_min <= x <= self.workspace_x_max
            and abs(y) <= self.workspace_y_abs_max
            and self.workspace_z_min <= z <= self.workspace_z_max
        )

    def recent_points(self, name, now=None):
        if now is None:
            now = self.get_clock().now().nanoseconds * 1e-9
        cutoff = float(now) - self.sample_window
        queue = self.samples[name]
        while queue and queue[0][0] < cutoff:
            queue.popleft()
        return [entry[1] for entry in queue]

    @staticmethod
    def median_point(points):
        return tuple(
            statistics.median(point[axis] for point in points)
            for axis in range(3)
        )

    @staticmethod
    def maximum_distance(points, center):
        return max(
            math.sqrt(
                sum(
                    (point[axis] - center[axis]) ** 2
                    for axis in range(3)
                )
            )
            for point in points
        )

    def latch_callback(self, name, _request, response):
        points = self.recent_points(name)
        if len(points) < self.minimum_samples:
            response.success = False
            response.message = (
                f"{name}: only {len(points)} recent samples; "
                f"need {self.minimum_samples}"
            )
            return response

        center = self.median_point(points)
        spread = self.maximum_distance(points, center)
        if spread > self.maximum_spread:
            response.success = False
            response.message = (
                f"{name}: unstable; spread={spread:.4f} m exceeds "
                f"{self.maximum_spread:.4f} m"
            )
            return response
        if not self.point_in_workspace(center):
            response.success = False
            response.message = f"{name}: median point is outside workspace"
            return response

        self.latched_class = name
        self.latched_point = center
        self.latched_sample_count = len(points)
        self.latched_spread = spread
        response.success = True
        response.message = (
            f"latched {name}: xyz=({center[0]:.4f}, {center[1]:.4f}, "
            f"{center[2]:.4f}) m, samples={len(points)}, "
            f"spread={spread:.4f} m"
        )
        self.get_logger().info(response.message)
        return response

    def clear_callback(self, _request, response):
        previous = self.latched_class
        self.latched_class = None
        self.latched_point = None
        self.latched_sample_count = 0
        self.latched_spread = None
        # Clearing a target starts a genuinely new observation window. This
        # prevents samples from the previous object pose contaminating an
        # immediate automatic re-latch.
        for queue in self.samples.values():
            queue.clear()

        delete_marker = Marker()
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.header.frame_id = self.base_frame
        delete_marker.action = Marker.DELETEALL
        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker)
        self.marker_publisher.publish(marker_array)

        response.success = True
        response.message = (
            f"cleared {previous}" if previous is not None else "nothing was latched"
        )
        self.get_logger().info(response.message)
        return response

    def publish_latched_target(self):
        if self.latched_class is None or self.latched_point is None:
            return

        stamp = self.get_clock().now().to_msg()
        point_message = PointStamped()
        point_message.header.stamp = stamp
        point_message.header.frame_id = self.base_frame
        point_message.point.x = self.latched_point[0]
        point_message.point.y = self.latched_point[1]
        point_message.point.z = self.latched_point[2]
        self.generic_publisher.publish(point_message)
        self.class_publishers[self.latched_class].publish(point_message)

        class_message = String()
        class_message.data = self.latched_class
        self.class_name_publisher.publish(class_message)

        sphere = Marker()
        sphere.header = point_message.header
        sphere.ns = "foam_latched_target"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = point_message.point
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.045
        sphere.scale.y = 0.045
        sphere.scale.z = 0.045
        sphere.color.r = 1.0
        sphere.color.g = 0.2
        sphere.color.b = 1.0
        sphere.color.a = 0.9

        label = Marker()
        label.header = point_message.header
        label.ns = "foam_latched_target_label"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = point_message.point
        label.pose.position.z += 0.05
        label.pose.orientation.w = 1.0
        label.scale.z = 0.025
        label.color.r = 1.0
        label.color.g = 0.5
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = (
            f"LATCHED {self.latched_class.upper()} "
            f"({self.latched_sample_count} samples)"
        )

        marker_array = MarkerArray()
        marker_array.markers.extend((sphere, label))
        self.marker_publisher.publish(marker_array)


def main():
    rclpy.init()
    node = FoamTargetLatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
