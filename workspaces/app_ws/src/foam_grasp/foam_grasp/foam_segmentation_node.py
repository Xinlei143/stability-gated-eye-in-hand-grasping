#!/usr/bin/env python3
"""ROS 2 real-time semantic segmentation node for foam objects."""

import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from torch import nn
from torchvision.models.segmentation import deeplabv3_resnet50

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


NUM_CLASSES = 4
CLASS_NAMES = ["background", "cube", "cylinder", "sphere"]

COLORS = np.array(
    [
        [0, 0, 0],
        [255, 40, 40],
        [40, 220, 40],
        [40, 100, 255],
    ],
    dtype=np.uint8,
)


def build_model():
    model = deeplabv3_resnet50(
        weights=None,
        weights_backbone=None,
        num_classes=NUM_CLASSES,
        aux_loss=True,
    )
    return model


class FoamSegmentationNode(Node):
    def __init__(self):
        super().__init__("foam_segmentation")

        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("require_cuda", True)
        self.declare_parameter("input_width", 640)
        self.declare_parameter("input_height", 360)

        checkpoint_path = Path(
            str(self.get_parameter("checkpoint_path").value)
        ).expanduser()
        require_cuda = bool(self.get_parameter("require_cuda").value)
        self.input_width = int(self.get_parameter("input_width").value)
        self.input_height = int(self.get_parameter("input_height").value)

        if not checkpoint_path.is_file():
            raise RuntimeError(
                f"checkpoint_path does not exist: {checkpoint_path}"
            )
        if self.input_width <= 0 or self.input_height <= 0:
            raise RuntimeError("input_width and input_height must be positive")
        if require_cuda and not torch.cuda.is_available():
            raise RuntimeError("CUDA GPU was not detected")

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.use_cuda = self.device.type == "cuda"
        self.bridge = CvBridge()

        self.get_logger().info(f"Loading checkpoint: {checkpoint_path}")
        checkpoint = torch.load(
            checkpoint_path,
            map_location="cpu",
            weights_only=True,
        )

        self.model = build_model()
        self.model.load_state_dict(
            checkpoint["model_state_dict"],
            strict=True,
        )
        self.model.to(self.device)
        self.model.eval()

        self.mean = torch.tensor(
            [0.485, 0.456, 0.406],
            dtype=torch.float32,
            device=self.device,
        ).view(1, 3, 1, 1)
        self.std = torch.tensor(
            [0.229, 0.224, 0.225],
            dtype=torch.float32,
            device=self.device,
        ).view(1, 3, 1, 1)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mask_publisher = self.create_publisher(
            Image,
            "/foam_segmentation/mask",
            10,
        )
        self.overlay_publisher = self.create_publisher(
            Image,
            "/foam_segmentation/overlay",
            10,
        )
        self.latency_publisher = self.create_publisher(
            Float32,
            "/foam_segmentation/latency_ms",
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            qos,
        )

        self.frame_count = 0
        self.report_start_time = time.perf_counter()

        self.get_logger().info(
            "Warming up GPU with input size "
            f"{self.input_width}x{self.input_height}"
        )
        dummy = torch.zeros(
            (1, 3, self.input_height, self.input_width),
            dtype=torch.float32,
            device=self.device,
        )
        with torch.inference_mode():
            for _ in range(3):
                with torch.amp.autocast("cuda", enabled=self.use_cuda):
                    self.model(dummy)
        if self.use_cuda:
            torch.cuda.synchronize()

        self.get_logger().info(
            "Ready. Subscribing to /camera/color/image_raw"
        )
        self.get_logger().info(
            "Publishing /foam_segmentation/mask and "
            "/foam_segmentation/overlay"
        )

    def preprocess(self, rgb_image):
        contiguous = np.ascontiguousarray(rgb_image)
        tensor = torch.from_numpy(contiguous).to(
            self.device,
            non_blocking=True,
        )
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).float()
        tensor = tensor / 255.0
        tensor = F.interpolate(
            tensor,
            size=(self.input_height, self.input_width),
            mode="bilinear",
            align_corners=False,
        )
        tensor = (tensor - self.mean) / self.std
        return tensor

    @staticmethod
    def make_overlay(rgb_image, prediction):
        overlay = rgb_image.copy()
        foreground = prediction > 0
        colored_mask = COLORS[prediction]

        if np.any(foreground):
            blended = (
                0.55 * rgb_image[foreground].astype(np.float32)
                + 0.45 * colored_mask[foreground].astype(np.float32)
            )
            overlay[foreground] = np.clip(
                blended,
                0,
                255,
            ).astype(np.uint8)

        return overlay

    def image_callback(self, message):
        start_time = time.perf_counter()

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="rgb8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Failed to convert input image: {error}"
            )
            return

        height, width = rgb_image.shape[:2]
        input_tensor = self.preprocess(rgb_image)

        with torch.inference_mode():
            with torch.amp.autocast("cuda", enabled=self.use_cuda):
                logits = self.model(input_tensor)["out"]
                logits = F.interpolate(
                    logits,
                    size=(height, width),
                    mode="bilinear",
                    align_corners=False,
                )

        prediction = (
            logits.argmax(dim=1)[0]
            .to(torch.uint8)
            .cpu()
            .numpy()
        )
        overlay = self.make_overlay(rgb_image, prediction)

        mask_message = self.bridge.cv2_to_imgmsg(
            prediction,
            encoding="mono8",
        )
        mask_message.header = message.header
        self.mask_publisher.publish(mask_message)

        overlay_message = self.bridge.cv2_to_imgmsg(
            overlay,
            encoding="rgb8",
        )
        overlay_message.header = message.header
        self.overlay_publisher.publish(overlay_message)

        if self.use_cuda:
            torch.cuda.synchronize()
        latency_ms = (time.perf_counter() - start_time) * 1000.0
        latency_message = Float32()
        latency_message.data = float(latency_ms)
        self.latency_publisher.publish(latency_message)

        self.frame_count += 1
        elapsed = time.perf_counter() - self.report_start_time

        if elapsed >= 5.0:
            fps = self.frame_count / elapsed
            class_counts = np.bincount(
                prediction.reshape(-1),
                minlength=NUM_CLASSES,
            )
            detected = [
                f"{CLASS_NAMES[class_id]}={class_counts[class_id]}px"
                for class_id in range(1, NUM_CLASSES)
                if class_counts[class_id] > 0
            ]
            detected_text = ", ".join(detected) if detected else "none"
            self.get_logger().info(
                f"rate={fps:.1f} FPS, latency={latency_ms:.1f} ms, "
                f"detected: {detected_text}"
            )
            self.frame_count = 0
            self.report_start_time = time.perf_counter()


def main():
    rclpy.init()
    node = None

    try:
        node = FoamSegmentationNode()
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
