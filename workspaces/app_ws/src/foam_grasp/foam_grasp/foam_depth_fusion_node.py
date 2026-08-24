#!/usr/bin/env python3
"""Fuse semantic masks with registered depth to publish 3D foam-object points."""

from collections import deque
import json
import time

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from foam_grasp.depth_fusion_diagnostics import (
    build_diagnostic,
    select_closest_mask,
)


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

MASK_TOPIC = "/foam_segmentation/mask"
DEPTH_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/depth/camera_info"
MARKER_TOPIC = "/foam_grasp/markers"
DIAGNOSTIC_TOPIC = "/foam_grasp/depth_fusion_diagnostics"

MAX_TIME_DIFFERENCE_SECONDS = 0.15
MIN_COMPONENT_AREA_PIXELS = 150
MIN_VALID_DEPTH_PIXELS = 50
MIN_DEPTH_METERS = 0.15
MAX_DEPTH_METERS = 1.50
SMOOTHING_WINDOW = 5


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FoamDepthFusionNode(Node):
    def __init__(self):
        super().__init__("foam_depth_fusion")

        self.bridge = CvBridge()
        self.latest_mask = None
        self.latest_mask_stamp = None
        self.mask_history = deque(maxlen=12)
        self.camera_intrinsics = None
        self.last_log_time = time.monotonic()
        self.diagnostic_start_time = time.monotonic()
        self.depth_frame_count = 0
        self.valid_output_count = 0

        self.histories = {
            class_id: deque(maxlen=SMOOTHING_WINDOW)
            for class_id in CLASS_NAMES
        }

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.point_publishers = {
            class_id: self.create_publisher(
                PointStamped,
                f"/foam_grasp/{name}_point",
                10,
            )
            for class_id, name in CLASS_NAMES.items()
        }
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            MARKER_TOPIC,
            10,
        )
        self.diagnostic_publisher = self.create_publisher(
            String,
            DIAGNOSTIC_TOPIC,
            10,
        )

        self.mask_subscription = self.create_subscription(
            Image,
            MASK_TOPIC,
            self.mask_callback,
            sensor_qos,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            sensor_qos,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            sensor_qos,
        )

        self.get_logger().info(f"Subscribing to mask: {MASK_TOPIC}")
        self.get_logger().info(f"Subscribing to depth: {DEPTH_TOPIC}")
        self.get_logger().info(f"Subscribing to intrinsics: {CAMERA_INFO_TOPIC}")
        self.get_logger().info(
            "Outputs: /foam_grasp/cube_point, "
            "/foam_grasp/cylinder_point, /foam_grasp/sphere_point"
        )
        self.get_logger().info(
            f"Publishing per-frame diagnostics: {DIAGNOSTIC_TOPIC}"
        )

    def camera_info_callback(self, message):
        fx = float(message.k[0])
        fy = float(message.k[4])
        cx = float(message.k[2])
        cy = float(message.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(
                "Camera intrinsics are invalid (fx or fy is zero).",
                throttle_duration_sec=5.0,
            )
            return

        current = (fx, fy, cx, cy, int(message.width), int(message.height))
        if self.camera_intrinsics != current:
            self.camera_intrinsics = current
            self.get_logger().info(
                f"Intrinsics: fx={fx:.3f}, fy={fy:.3f}, "
                f"cx={cx:.3f}, cy={cy:.3f}, "
                f"size={message.width}x{message.height}"
            )

    def mask_callback(self, message):
        try:
            mask = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="mono8",
            )
        except Exception as error:
            self.get_logger().error(f"Mask conversion failed: {error}")
            return

        self.latest_mask = np.asarray(mask, dtype=np.uint8).copy()
        self.latest_mask_stamp = stamp_to_seconds(message.header.stamp)
        self.mask_history.append((self.latest_mask_stamp, self.latest_mask))

    @staticmethod
    def largest_component(binary_mask):
        component_input = binary_mask.astype(np.uint8)
        count, labels, statistics, _ = cv2.connectedComponentsWithStats(
            component_input,
            connectivity=8,
        )

        if count <= 1:
            return None

        areas = statistics[1:, cv2.CC_STAT_AREA]
        largest_label = 1 + int(np.argmax(areas))
        largest_area = int(statistics[largest_label, cv2.CC_STAT_AREA])

        if largest_area < MIN_COMPONENT_AREA_PIXELS:
            return None

        return labels == largest_label

    @staticmethod
    def depth_to_meters(depth_image):
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * 0.001
        if depth_image.dtype in (np.float32, np.float64):
            return depth_image.astype(np.float32)
        raise RuntimeError(f"Unsupported depth dtype: {depth_image.dtype}")

    def calculate_point(self, class_mask, depth_meters):
        stats = {
            "status": "no_mask",
            "mask_pixels": int(np.count_nonzero(class_mask)),
            "component_pixels": 0,
            "eroded_pixels": 0,
            "valid_depth_pixels": 0,
            "valid_depth_pixels_after_mad": 0,
            "point_camera": None,
        }
        component = self.largest_component(class_mask)
        if component is None:
            if stats["mask_pixels"]:
                stats["status"] = "component_too_small"
            return None, stats

        stats["component_pixels"] = int(component.sum())

        kernel = np.ones((5, 5), dtype=np.uint8)
        eroded = cv2.erode(
            component.astype(np.uint8),
            kernel,
            iterations=1,
        ).astype(bool)
        stats["eroded_pixels"] = int(eroded.sum())

        if stats["eroded_pixels"] < MIN_VALID_DEPTH_PIXELS:
            stats["status"] = "insufficient_eroded_pixels"
            return None, stats
        component = eroded

        valid = (
            component
            & np.isfinite(depth_meters)
            & (depth_meters >= MIN_DEPTH_METERS)
            & (depth_meters <= MAX_DEPTH_METERS)
        )

        stats["valid_depth_pixels"] = int(valid.sum())
        if stats["valid_depth_pixels"] < MIN_VALID_DEPTH_PIXELS:
            stats["status"] = "insufficient_valid_depth"
            return None, stats

        initial_depths = depth_meters[valid]
        median_depth = float(np.median(initial_depths))
        mad = float(np.median(np.abs(initial_depths - median_depth)))
        robust_sigma = 1.4826 * mad
        depth_band = min(max(0.025, 3.0 * robust_sigma), 0.080)

        valid &= np.abs(depth_meters - median_depth) <= depth_band

        stats["valid_depth_pixels_after_mad"] = int(valid.sum())
        if stats["valid_depth_pixels_after_mad"] < MIN_VALID_DEPTH_PIXELS:
            stats["status"] = "depth_outlier_rejection"
            return None, stats

        rows, columns = np.nonzero(valid)
        z = depth_meters[rows, columns]

        fx, fy, cx, cy, _, _ = self.camera_intrinsics
        x = (columns.astype(np.float32) - cx) * z / fx
        y = (rows.astype(np.float32) - cy) * z / fy

        point = np.array(
            [
                np.median(x),
                np.median(y),
                np.median(z),
            ],
            dtype=np.float64,
        )
        stats["status"] = "valid"
        stats["point_camera"] = [float(value) for value in point]
        return point, stats

    def publish_diagnostic(self, depth_stamp, frame_id, mask_stamp, classes):
        elapsed = max(time.monotonic() - self.diagnostic_start_time, 1e-9)
        payload = build_diagnostic(
            depth_stamp=depth_stamp,
            mask_stamp=mask_stamp,
            frame_id=frame_id,
            frame_count=self.depth_frame_count,
            valid_output_count=self.valid_output_count,
            output_rate_hz=self.depth_frame_count / elapsed,
            classes=classes,
        )
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.diagnostic_publisher.publish(message)

    def smooth_point(self, class_id, point):
        history = self.histories[class_id]
        history.append(point)
        values = np.stack(history, axis=0)
        return np.median(values, axis=0)

    def create_marker(self, class_id, header, point=None):
        marker = Marker()
        marker.header = header
        marker.ns = "foam_grasp_targets"
        marker.id = int(class_id)
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1.0

        if point is None:
            marker.action = Marker.DELETE
            return marker

        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.scale.x = 0.035
        marker.scale.y = 0.035
        marker.scale.z = 0.035

        red, green, blue = CLASS_COLORS[class_id]
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.30).to_msg()
        return marker

    def depth_callback(self, message):
        self.depth_frame_count += 1
        depth_stamp = stamp_to_seconds(message.header.stamp)
        selected_mask = select_closest_mask(self.mask_history, depth_stamp)
        if selected_mask is None:
            self.publish_diagnostic(
                depth_stamp,
                message.header.frame_id,
                None,
                {
                    name: {
                        "status": "no_mask",
                        "mask_pixels": 0,
                        "component_pixels": 0,
                        "eroded_pixels": 0,
                        "valid_depth_pixels": 0,
                        "valid_depth_pixels_after_mad": 0,
                        "point_camera": None,
                    }
                    for name in CLASS_NAMES.values()
                },
            )
            return
        mask_stamp, mask = selected_mask
        if self.camera_intrinsics is None:
            self.publish_diagnostic(
                depth_stamp,
                message.header.frame_id,
                mask_stamp,
                {name: {"status": "no_intrinsics"} for name in CLASS_NAMES.values()},
            )
            return

        time_difference = abs(depth_stamp - mask_stamp)
        if time_difference > MAX_TIME_DIFFERENCE_SECONDS:
            self.publish_diagnostic(
                depth_stamp,
                message.header.frame_id,
                mask_stamp,
                {name: {"status": "stale_mask"} for name in CLASS_NAMES.values()},
            )
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
            depth_meters = self.depth_to_meters(np.asarray(depth_image))
        except Exception as error:
            self.get_logger().error(f"Depth conversion failed: {error}")
            self.publish_diagnostic(
                depth_stamp,
                message.header.frame_id,
                mask_stamp,
                {
                    name: {"status": "depth_conversion_error"}
                    for name in CLASS_NAMES.values()
                },
            )
            return

        if mask.shape != depth_meters.shape:
            mask = cv2.resize(
                mask,
                (depth_meters.shape[1], depth_meters.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        detected_points = {}
        class_stats = {}
        marker_array = MarkerArray()

        for class_id in CLASS_NAMES:
            point, stats = self.calculate_point(
                mask == class_id,
                depth_meters,
            )
            class_stats[CLASS_NAMES[class_id]] = stats

            if point is None:
                self.histories[class_id].clear()
                marker_array.markers.append(
                    self.create_marker(class_id, message.header)
                )
                continue

            smoothed = self.smooth_point(class_id, point)
            detected_points[class_id] = smoothed
            stats["point_camera_smoothed"] = [float(value) for value in smoothed]

            point_message = PointStamped()
            point_message.header = message.header
            point_message.point.x = float(smoothed[0])
            point_message.point.y = float(smoothed[1])
            point_message.point.z = float(smoothed[2])
            self.point_publishers[class_id].publish(point_message)

            marker_array.markers.append(
                self.create_marker(class_id, message.header, smoothed)
            )

        self.marker_publisher.publish(marker_array)
        self.valid_output_count += len(detected_points)
        self.publish_diagnostic(
            depth_stamp,
            message.header.frame_id,
            mask_stamp,
            class_stats,
        )

        now = time.monotonic()
        if now - self.last_log_time >= 1.0:
            if detected_points:
                descriptions = []
                for class_id, point in detected_points.items():
                    descriptions.append(
                        f"{CLASS_NAMES[class_id]}="
                        f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})m "
                        f"[{class_stats[CLASS_NAMES[class_id]]['valid_depth_pixels_after_mad']}px]"
                    )
                self.get_logger().info("; ".join(descriptions))
            else:
                self.get_logger().info("No valid foam-object 3D point")
            self.last_log_time = now


def main():
    rclpy.init()
    node = None

    try:
        node = FoamDepthFusionNode()
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
