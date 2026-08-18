#!/usr/bin/env python3
"""Save the latest ROS 2 color frame when a Trigger service is called.

This utility is data-collection only. It does not subscribe to or publish any
Piper command topic.
"""

import argparse
import os
from pathlib import Path
import re
import time

import numpy as np
from PIL import Image as PILImage

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


def parse_arguments():
    parser = argparse.ArgumentParser(description="按服务请求保存当前彩色帧")
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="PNG输出目录",
    )
    parser.add_argument(
        "--topic",
        default="/camera/color/image_raw",
        help="彩色图像话题",
    )
    parser.add_argument(
        "--service",
        default="/foam_dataset/save",
        help="std_srvs/srv/Trigger服务名",
    )
    parser.add_argument("--prefix", default="frame_", help="文件名前缀")
    parser.add_argument(
        "--maximum-frame-age",
        type=float,
        default=1.0,
        help="允许保存的最大帧龄（秒）",
    )
    return parser.parse_args(remove_ros_args()[1:])


class ColorImageSaver(Node):
    def __init__(self, arguments):
        super().__init__("foam_dataset_image_saver")
        self.output_dir = arguments.output_dir.expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.topic = arguments.topic
        self.prefix = arguments.prefix
        self.maximum_frame_age = arguments.maximum_frame_age
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_received_at = 0.0
        self.next_index = self.find_next_index()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos,
        )
        self.service = self.create_service(
            Trigger,
            arguments.service,
            self.save_callback,
        )
        self.get_logger().info(f"图像输入：{self.topic}")
        self.get_logger().info(f"保存服务：{arguments.service}")
        self.get_logger().info(f"输出目录：{self.output_dir}")
        self.get_logger().warning("DATA COLLECTION ONLY: 不会发送机械臂命令")

    def find_next_index(self):
        expression = re.compile(
            rf"^{re.escape(self.prefix)}(\d+)\.png$"
        )
        indices = []
        for path in self.output_dir.glob(f"{self.prefix}*.png"):
            match = expression.match(path.name)
            if match:
                indices.append(int(match.group(1)))
        return max(indices, default=-1) + 1

    def image_callback(self, message):
        try:
            rgb = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="rgb8",
            )
        except Exception as error:
            self.get_logger().error(f"图像转换失败：{error}")
            return
        self.latest_rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        self.latest_received_at = time.monotonic()

    def save_callback(self, _request, response):
        if self.latest_rgb is None:
            response.success = False
            response.message = "尚未收到彩色图像"
            return response
        age = time.monotonic() - self.latest_received_at
        if age > self.maximum_frame_age:
            response.success = False
            response.message = f"最新彩色帧已过期：{age:.2f}s"
            return response

        filename = f"{self.prefix}{self.next_index:04d}.png"
        output_path = self.output_dir / filename
        temporary_path = output_path.with_suffix(".png.tmp")
        try:
            PILImage.fromarray(self.latest_rgb, mode="RGB").save(
                temporary_path,
                format="PNG",
            )
            os.replace(temporary_path, output_path)
        except Exception as error:
            temporary_path.unlink(missing_ok=True)
            response.success = False
            response.message = f"保存失败：{error}"
            return response

        height, width = self.latest_rgb.shape[:2]
        self.next_index += 1
        response.success = True
        response.message = (
            f"saved {output_path} ({width}x{height}, frame_age={age:.3f}s)"
        )
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = None
    try:
        arguments = parse_arguments()
        if arguments.maximum_frame_age <= 0.0:
            raise ValueError("--maximum-frame-age必须为正数")
        node = ColorImageSaver(arguments)
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
