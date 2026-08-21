#!/usr/bin/env python3
"""Publish class-aware top-down preview poses for foam-object grasping.

This node never sends a command to MoveIt, a controller, or the Piper arm.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


INPUT_TOPIC = "/foam_grasp/target_point_base_latched"
PREGRASP_TOPIC = "/foam_grasp/target_pregrasp_pose"
GRASP_TOPIC = "/foam_grasp/target_grasp_pose"
LIFT_TOPIC = "/foam_grasp/target_lift_pose"
MARKER_TOPIC = "/foam_grasp/cube_pose_preview"
CLASS_NAMES = ("cube", "cylinder", "sphere")


class FoamGraspPosePreviewNode(Node):
    def __init__(self):
        super().__init__("foam_grasp_pose_preview")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("input_topic", INPUT_TOPIC)
        self.declare_parameter("class_topic", "/foam_grasp/latched_target_class")
        self.declare_parameter("table_height", 0.001)
        self.declare_parameter("cube_size", 0.050)
        self.declare_parameter("cylinder_diameter", 0.070)
        self.declare_parameter("cylinder_height", 0.070)
        self.declare_parameter("sphere_diameter", 0.060)
        self.declare_parameter("tool_offset", 0.1358)
        self.declare_parameter("pregrasp_clearance", 0.055)
        self.declare_parameter("lift_clearance", 0.055)
        self.declare_parameter("target_timeout", 0.5)
        self.declare_parameter("grasp_offset_x", 0.0)
        self.declare_parameter("grasp_offset_y", 0.0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.class_topic = str(self.get_parameter("class_topic").value)
        self.table_height = float(
            self.get_parameter("table_height").value
        )
        self.cube_size = float(self.get_parameter("cube_size").value)
        self.cylinder_diameter = float(
            self.get_parameter("cylinder_diameter").value
        )
        self.cylinder_height = float(
            self.get_parameter("cylinder_height").value
        )
        self.sphere_diameter = float(
            self.get_parameter("sphere_diameter").value
        )
        self.tool_offset = float(
            self.get_parameter("tool_offset").value
        )
        self.pregrasp_clearance = float(
            self.get_parameter("pregrasp_clearance").value
        )
        self.lift_clearance = float(
            self.get_parameter("lift_clearance").value
        )
        self.target_timeout = float(
            self.get_parameter("target_timeout").value
        )
        self.validate_offset(
            float(self.get_parameter("grasp_offset_x").value),
            "grasp_offset_x",
        )
        self.validate_offset(
            float(self.get_parameter("grasp_offset_y").value),
            "grasp_offset_y",
        )
        dimensions = (
            self.cube_size,
            self.cylinder_diameter,
            self.cylinder_height,
            self.sphere_diameter,
        )
        if not all(0.020 <= value <= 0.100 for value in dimensions):
            raise RuntimeError("foam object dimensions are outside 20--100 mm")
        self.add_on_set_parameters_callback(self.validate_parameters)

        self.latest_target = None
        self.target_received_at = None
        self.latest_class = None
        self.class_received_at = None
        self.last_log_time = 0.0

        self.target_subscription = self.create_subscription(
            PointStamped,
            self.input_topic,
            self.target_callback,
            10,
        )
        self.class_subscription = self.create_subscription(
            String,
            self.class_topic,
            self.class_callback,
            10,
        )
        self.pregrasp_publisher = self.create_publisher(
            PoseStamped,
            PREGRASP_TOPIC,
            10,
        )
        self.grasp_publisher = self.create_publisher(
            PoseStamped,
            GRASP_TOPIC,
            10,
        )
        self.lift_publisher = self.create_publisher(
            PoseStamped,
            LIFT_TOPIC,
            10,
        )
        self.class_pose_publishers = {
            name: {
                "pregrasp": self.create_publisher(
                    PoseStamped,
                    f"/foam_grasp/{name}_pregrasp_pose",
                    10,
                ),
                "grasp": self.create_publisher(
                    PoseStamped,
                    f"/foam_grasp/{name}_grasp_pose",
                    10,
                ),
                "lift": self.create_publisher(
                    PoseStamped,
                    f"/foam_grasp/{name}_lift_pose",
                    10,
                ),
            }
            for name in CLASS_NAMES
        }
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            MARKER_TOPIC,
            10,
        )

        self.publish_timer = self.create_timer(0.1, self.publish_preview)

        self.get_logger().warning(
            "PREVIEW ONLY: this node never commands the Piper arm"
        )
        self.get_logger().info(f"Input: {self.input_topic}")
        self.get_logger().info(f"Class input: {self.class_topic}")
        self.get_logger().info(
            "MoveIt link6 top-down orientation: quaternion=(0, 1, 0, 0)"
        )
        self.get_logger().info(
            "Planar grasp correction (base_link): "
            f"dx={float(self.get_parameter('grasp_offset_x').value):.3f} m, "
            f"dy={float(self.get_parameter('grasp_offset_y').value):.3f} m"
        )

    @staticmethod
    def validate_offset(value, name):
        if not math.isfinite(value) or abs(value) > 0.030:
            raise ValueError(f"{name} must be finite and within +/-0.030 m")

    def validate_parameters(self, parameters):
        try:
            for parameter in parameters:
                if parameter.name in ("grasp_offset_x", "grasp_offset_y"):
                    self.validate_offset(float(parameter.value), parameter.name)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def target_callback(self, message):
        self.latest_target = message
        self.target_received_at = time.monotonic()

    def class_callback(self, message):
        value = str(message.data).strip().lower()
        if value not in CLASS_NAMES:
            return
        self.latest_class = value
        self.class_received_at = time.monotonic()

    def make_pose(self, stamp, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # In the Piper URDF, the gripper extends along link6 +Z.
        # A 180-degree rotation about base Y points link6 +Z downward.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        return pose

    @staticmethod
    def set_marker_position(marker, x, y, z):
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

    @staticmethod
    def copy_pose_to_marker(marker, pose_stamped):
        source = pose_stamped.pose
        marker.pose.position.x = source.position.x
        marker.pose.position.y = source.position.y
        marker.pose.position.z = source.position.z
        marker.pose.orientation.x = source.orientation.x
        marker.pose.orientation.y = source.orientation.y
        marker.pose.orientation.z = source.orientation.z
        marker.pose.orientation.w = source.orientation.w

    def sphere_marker(self, marker_id, stamp, name, pose, color):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = "cube_grasp_preview"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        self.copy_pose_to_marker(marker, pose)
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.3).to_msg()
        return marker

    def text_marker(self, marker_id, stamp, name, pose, color):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = "cube_grasp_preview_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        self.copy_pose_to_marker(marker, pose)
        marker.pose.position.z += 0.035
        marker.scale.z = 0.025
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = name
        marker.lifetime = Duration(seconds=0.3).to_msg()
        return marker

    def object_marker(self, class_name, stamp, x, y, z):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = "foam_grasp_preview_object"
        marker.id = 1
        marker.action = Marker.ADD
        self.set_marker_position(marker, x, y, z)
        marker.pose.orientation.w = 1.0
        if class_name == "cube":
            marker.type = Marker.CUBE
            marker.scale.x = self.cube_size
            marker.scale.y = self.cube_size
            marker.scale.z = self.cube_size
            marker.color.r = 0.15
            marker.color.g = 0.55
            marker.color.b = 1.0
        elif class_name == "cylinder":
            marker.type = Marker.CYLINDER
            marker.scale.x = self.cylinder_diameter
            marker.scale.y = self.cylinder_diameter
            marker.scale.z = self.cylinder_height
            marker.color.r = 0.15
            marker.color.g = 0.90
            marker.color.b = 0.25
        else:
            marker.type = Marker.SPHERE
            marker.scale.x = self.sphere_diameter
            marker.scale.y = self.sphere_diameter
            marker.scale.z = self.sphere_diameter
            marker.color.r = 0.95
            marker.color.g = 0.75
            marker.color.b = 0.10
        marker.color.a = 0.55
        marker.lifetime = Duration(seconds=0.3).to_msg()
        return marker

    def approach_marker(self, stamp, pregrasp, grasp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = "cube_grasp_preview_path"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point()
        start.x = pregrasp.pose.position.x
        start.y = pregrasp.pose.position.y
        start.z = pregrasp.pose.position.z
        end = Point()
        end.x = grasp.pose.position.x
        end.y = grasp.pose.position.y
        end.z = grasp.pose.position.z
        marker.points = [start, end]

        marker.scale.x = 0.012
        marker.scale.y = 0.024
        marker.scale.z = 0.030
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.05
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=0.3).to_msg()
        return marker

    def publish_preview(self):
        if (
            self.latest_target is None
            or self.target_received_at is None
            or self.latest_class is None
            or self.class_received_at is None
        ):
            return
        now = time.monotonic()
        if (
            now - self.target_received_at > self.target_timeout
            or now - self.class_received_at > self.target_timeout
        ):
            return

        target = self.latest_target
        class_name = self.latest_class
        stamp = target.header.stamp
        object_x = float(target.point.x)
        object_y = float(target.point.y)
        x = object_x + float(self.get_parameter("grasp_offset_x").value)
        y = object_y + float(self.get_parameter("grasp_offset_y").value)

        if class_name == "cube":
            object_height = self.cube_size
        elif class_name == "cylinder":
            object_height = self.cylinder_height
        else:
            object_height = self.sphere_diameter
        object_center_z = self.table_height + 0.5 * object_height

        # With link6 +Z pointing down, the link6 origin must remain above
        # the desired jaw-contact center by the modeled tool offset.
        grasp_link6_z = object_center_z + self.tool_offset
        pregrasp_link6_z = grasp_link6_z + self.pregrasp_clearance
        lift_link6_z = grasp_link6_z + self.lift_clearance

        pregrasp = self.make_pose(
            stamp,
            x,
            y,
            pregrasp_link6_z,
        )
        grasp = self.make_pose(stamp, x, y, grasp_link6_z)
        lift = self.make_pose(stamp, x, y, lift_link6_z)

        self.pregrasp_publisher.publish(pregrasp)
        self.grasp_publisher.publish(grasp)
        self.lift_publisher.publish(lift)
        class_publishers = self.class_pose_publishers[class_name]
        class_publishers["pregrasp"].publish(pregrasp)
        class_publishers["grasp"].publish(grasp)
        class_publishers["lift"].publish(lift)

        marker_array = MarkerArray()
        marker_array.markers.append(
            self.object_marker(
                class_name,
                stamp,
                object_x,
                object_y,
                object_center_z,
            )
        )

        poses = (
            (10, "PREGRASP", pregrasp, (0.1, 1.0, 0.2)),
            (20, "GRASP", grasp, (1.0, 0.15, 0.15)),
            (30, "LIFT", lift, (0.8, 0.2, 1.0)),
        )
        for marker_id, name, pose, color in poses:
            marker_array.markers.append(
                self.sphere_marker(
                    marker_id,
                    stamp,
                    name,
                    pose,
                    color,
                )
            )
            marker_array.markers.append(
                self.text_marker(
                    marker_id,
                    stamp,
                    name,
                    pose,
                    color,
                )
            )

        marker_array.markers.append(
            self.approach_marker(stamp, pregrasp, grasp)
        )
        self.marker_publisher.publish(marker_array)

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"{class_name}_center=({x:.3f}, {y:.3f}, "
                f"{object_center_z:.3f})m; "
                f"link6_pregrasp_z={pregrasp_link6_z:.3f}m; "
                f"link6_grasp_z={grasp_link6_z:.3f}m; "
                f"link6_lift_z={lift_link6_z:.3f}m"
            )
            self.last_log_time = now


def main():
    rclpy.init()
    node = None

    try:
        node = FoamGraspPosePreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
