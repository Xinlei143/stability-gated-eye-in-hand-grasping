#!/usr/bin/env python3
"""Transform segmented foam-object points from the camera frame to base_link."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


CLASS_NAMES = {
    1: "cube",
    2: "cylinder",
    3: "sphere",
}

CLASS_COLORS = {
    1: (1.0, 0.15, 0.15),
    2: (0.15, 1.0, 0.15),
    3: (0.15, 0.35, 1.0),
}

def normalize_quaternion(quaternion):
    length = math.sqrt(sum(value * value for value in quaternion))
    if length < 1e-12:
        raise RuntimeError("Quaternion has zero length")
    return tuple(value / length for value in quaternion)


def rotate_vector(quaternion, vector):
    """Rotate vector using quaternion (x, y, z, w)."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def transform_point(point, translation, quaternion):
    rotated = rotate_vector(quaternion, point)
    return tuple(rotated[index] + translation[index] for index in range(3))


class FoamCameraToBaseNode(Node):
    def __init__(self):
        super().__init__("foam_camera_to_base")

        self.declare_parameter("calibration_file", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_pose_timeout", 1.0)
        self.declare_parameter("transform_source", "calibration")
        self.declare_parameter("tf_timeout", 0.2)

        self.transform_source = str(
            self.get_parameter("transform_source").value
        ).strip().lower()
        if self.transform_source not in {"calibration", "tf"}:
            raise RuntimeError(
                "transform_source must be either 'calibration' or 'tf'"
            )
        calibration_file = str(
            self.get_parameter("calibration_file").value
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.end_pose_timeout = float(
            self.get_parameter("end_pose_timeout").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        if not math.isfinite(self.tf_timeout) or self.tf_timeout <= 0.0:
            raise RuntimeError("tf_timeout must be a finite positive number")

        self.end_pose = None
        self.end_pose_received_at = None
        self.tf_buffer = None
        self.tf_listener = None

        if self.transform_source == "calibration":
            if not calibration_file:
                raise RuntimeError("calibration_file parameter is required")

            with open(calibration_file, "r", encoding="utf-8") as file:
                calibration = json.load(file)

            self.translation_gripper_camera = tuple(
                float(value) for value in calibration["position"]
            )
            self.quaternion_gripper_camera = normalize_quaternion(
                tuple(float(value) for value in calibration["orientation"])
            )

            if len(self.translation_gripper_camera) != 3:
                raise RuntimeError("Calibration position must contain 3 values")
            if len(self.quaternion_gripper_camera) != 4:
                raise RuntimeError("Calibration orientation must contain 4 values")

        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_base_points = {}
        self.last_warning_time = 0.0

        self.end_pose_subscription = None
        if self.transform_source == "calibration":
            self.end_pose_subscription = self.create_subscription(
                Pose,
                "/end_pose",
                self.end_pose_callback,
                10,
            )

        self.point_publishers = {
            class_id: self.create_publisher(
                PointStamped,
                f"/foam_grasp/{name}_point_base",
                10,
            )
            for class_id, name in CLASS_NAMES.items()
        }

        self.point_subscriptions = []
        for class_id, name in CLASS_NAMES.items():
            subscription = self.create_subscription(
                PointStamped,
                f"/foam_grasp/{name}_point",
                lambda message, current_id=class_id: self.point_callback(
                    current_id,
                    message,
                ),
                10,
            )
            self.point_subscriptions.append(subscription)

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/foam_grasp/base_markers",
            10,
        )
        self.log_timer = self.create_timer(1.0, self.log_points)

        if self.transform_source == "calibration":
            self.get_logger().info(
                f"Loaded eye-in-hand calibration: {calibration_file}"
            )
            self.get_logger().info(
                "Transform chain: base <- gripper <- camera"
            )
        else:
            self.get_logger().info(
                "Transform chain: TF base frame <- PointStamped frame"
            )
        self.get_logger().info(
            "Outputs: /foam_grasp/cube_point_base, "
            "/foam_grasp/cylinder_point_base, "
            "/foam_grasp/sphere_point_base"
        )

    def end_pose_callback(self, message):
        self.end_pose = message
        self.end_pose_received_at = time.monotonic()

    def warn_missing_pose(self, description):
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(description)
            self.last_warning_time = now

    def output_stamp(self, source_stamp):
        """Stamp the point when the base-frame transform becomes available.

        TF lookup is allowed to wait for a future transform, so retaining the
        sensor stamp can make a freshly published point appear stale to the
        method gate.  The source timestamp remains the lookup timestamp; the
        output timestamp represents point availability.
        """

        try:
            return self.get_clock().now().to_msg()
        except (AttributeError, TypeError):
            return source_stamp

    def point_callback(self, class_id, message):
        if self.transform_source == "tf":
            self.tf_point_callback(class_id, message)
            return

        if self.end_pose is None or self.end_pose_received_at is None:
            self.warn_missing_pose("Waiting for Piper /end_pose")
            return

        pose_age = time.monotonic() - self.end_pose_received_at
        if pose_age > self.end_pose_timeout:
            self.warn_missing_pose(
                f"Piper /end_pose is stale ({pose_age:.2f} s)"
            )
            return

        camera_point = (
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        )

        # The saved OpenCV eye-in-hand result maps camera -> gripper.
        gripper_point = transform_point(
            camera_point,
            self.translation_gripper_camera,
            self.quaternion_gripper_camera,
        )

        pose = self.end_pose
        translation_base_gripper = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        quaternion_base_gripper = normalize_quaternion(
            (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            )
        )

        base_point = transform_point(
            gripper_point,
            translation_base_gripper,
            quaternion_base_gripper,
        )
        self.latest_base_points[class_id] = base_point

        output = PointStamped()
        output.header.stamp = self.output_stamp(message.header.stamp)
        output.header.frame_id = self.base_frame
        output.point.x = float(base_point[0])
        output.point.y = float(base_point[1])
        output.point.z = float(base_point[2])
        self.point_publishers[class_id].publish(output)

        marker = Marker()
        marker.header = output.header
        marker.ns = "foam_targets_in_base"
        marker.id = int(class_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = output.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        red, green, blue = CLASS_COLORS[class_id]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.30).to_msg()

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    def tf_point_callback(self, class_id, message):
        source_frame = str(message.header.frame_id).strip()
        if not source_frame:
            self.warn_missing_pose(
                "Cannot transform PointStamped without a source frame"
            )
            return

        try:
            transform = self._lookup_tf_transform(source_frame, message.header.stamp)
            transformed = do_transform_point(message, transform)
        except (TransformException, TypeError, ValueError) as error:
            self.warn_missing_pose(
                "TF unavailable for "
                f"{source_frame} -> {self.base_frame}: {error}"
            )
            return

        base_point = (
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        )
        self.latest_base_points[class_id] = base_point

        output = PointStamped()
        output.header.stamp = self.output_stamp(message.header.stamp)
        output.header.frame_id = self.base_frame
        output.point.x = base_point[0]
        output.point.y = base_point[1]
        output.point.z = base_point[2]
        self.point_publishers[class_id].publish(output)

        marker = Marker()
        marker.header = output.header
        marker.ns = "foam_targets_in_base"
        marker.id = int(class_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = output.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        red, green, blue = CLASS_COLORS[class_id]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.30).to_msg()

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)

    @staticmethod
    def _is_future_extrapolation(error):
        description = str(error).lower()
        return "future" in description and "extrapolat" in description

    def _lookup_tf_transform(self, source_frame, stamp):
        """Get a transform without blocking the RGB-D callback on future TF.

        Gazebo can publish an image whose simulation timestamp is a few
        milliseconds ahead of the latest TF sample.  A blocking lookup then
        stalls the single-threaded callback for ``tf_timeout`` and creates
        artificial observation gaps.  Probe the stamped transform
        immediately; for future extrapolation, use the latest available TF,
        which is the closest valid transform.  Other lookup failures retain
        the configured wait for startup/network delays.
        """
        target = self.base_frame
        stamped = Time.from_msg(stamp)
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source_frame,
                stamped,
                timeout=Duration(seconds=0.0),
            )
        except (TransformException, TypeError, ValueError) as error:
            if self._is_future_extrapolation(error):
                return self.tf_buffer.lookup_transform(
                    target,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=min(self.tf_timeout, 0.02)),
                )
            return self.tf_buffer.lookup_transform(
                target,
                source_frame,
                stamped,
                timeout=Duration(seconds=self.tf_timeout),
            )

    def log_points(self):
        if not self.latest_base_points:
            return

        descriptions = []
        for class_id in sorted(self.latest_base_points):
            point = self.latest_base_points[class_id]
            descriptions.append(
                f"{CLASS_NAMES[class_id]}_base="
                f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})m"
            )
        self.get_logger().info("; ".join(descriptions))


def main():
    rclpy.init()
    node = None

    try:
        node = FoamCameraToBaseNode()
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
