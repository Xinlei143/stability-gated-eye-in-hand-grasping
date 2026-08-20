#!/usr/bin/env python3
"""Re-center or dynamically follow a segmented foam target.

In one-shot mode, the node stops after the target is horizontally centered and
fully inside the image boundary. Use --follow for continuous tracking until
Ctrl+C, or --follow-seconds for a bounded tracking demonstration. Both follow
modes keep observing the semantic mask and perform small planned corrections
without starting a grasp.  --servo-follow provides fast horizontal tracking;
--workspace-follow additionally uses the calibrated 3D target point for wide
angle and forward/backward tracking.
"""

import argparse
import copy
import math
import sys
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

from foam_grasp.foam_move_to_observe import FoamMoveToObserve
from foam_grasp.foam_move_to_pregrasp import ARM_JOINTS


CONFIRM_TOKEN = "AUTO_CENTER_TARGET"
MASK_TOPIC = "/foam_segmentation/mask"
CLASS_IDS = {"cube": 1, "cylinder": 2, "sphere": 3}
MIN_COMPONENT_AREA_PIXELS = 150
WORKSPACE_JOINT_LIMIT_MARGIN_RAD = 0.04
GRASP_WORKSPACE_X_MIN = 0.15
GRASP_WORKSPACE_X_MAX = 0.60
GRASP_WORKSPACE_Y_ABS_MAX = 0.35
GRASP_WORKSPACE_Z_MIN = -0.02
GRASP_WORKSPACE_Z_MAX = 0.20


class FoamTargetCenterNode(FoamMoveToObserve):
    def __init__(self, target_class, border_margin, sample_count, execution_backend="real"):
        super().__init__(execution_backend=execution_backend)
        self.target_class = target_class
        self.target_class_id = CLASS_IDS[target_class]
        self.border_margin = int(border_margin)
        self.sample_count = int(sample_count)
        self.bridge = CvBridge()
        self.measurements = deque(maxlen=max(30, self.sample_count * 4))
        self.latest_base_point = None
        self.latest_base_point_received_at = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mask_subscription = self.create_subscription(
            Image,
            MASK_TOPIC,
            self.mask_callback,
            sensor_qos,
        )
        self.base_point_subscription = self.create_subscription(
            PointStamped,
            f"/foam_grasp/{target_class}_point_base",
            self.base_point_callback,
            10,
        )
        self.get_logger().info(
            f"Target centering ready: class={target_class}, "
            f"mask={MASK_TOPIC}, "
            f"point=/foam_grasp/{target_class}_point_base"
        )

    def base_point_callback(self, message):
        values = (
            float(message.point.x),
            float(message.point.y),
            float(message.point.z),
        )
        if (
            message.header.frame_id != "base_link"
            or not all(math.isfinite(value) for value in values)
            or not -0.10 <= values[2] <= 0.80
        ):
            return
        self.latest_base_point = values
        self.latest_base_point_received_at = time.monotonic()

    def get_latest_base_point(self, maximum_age_sec):
        if (
            self.latest_base_point is None
            or time.monotonic() - self.latest_base_point_received_at
            > maximum_age_sec
        ):
            return None
        return tuple(self.latest_base_point)

    def mask_callback(self, message):
        try:
            mask = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="mono8",
            )
        except Exception as error:
            self.get_logger().error(f"Mask conversion failed: {error}")
            return

        mask = np.asarray(mask, dtype=np.uint8)
        binary = (mask == self.target_class_id).astype(np.uint8)

        count, labels, statistics, _ = cv2.connectedComponentsWithStats(
            binary,
            connectivity=8,
        )

        if count <= 1:
            return

        areas = statistics[1:, cv2.CC_STAT_AREA]
        largest_label = 1 + int(np.argmax(areas))
        area = int(statistics[largest_label, cv2.CC_STAT_AREA])

        if area < MIN_COMPONENT_AREA_PIXELS:
            return

        x = int(statistics[largest_label, cv2.CC_STAT_LEFT])
        y = int(statistics[largest_label, cv2.CC_STAT_TOP])
        width = int(statistics[largest_label, cv2.CC_STAT_WIDTH])
        height = int(statistics[largest_label, cv2.CC_STAT_HEIGHT])

        image_height, image_width = mask.shape[:2]

        center_u = x + 0.5 * width
        center_v = y + 0.5 * height

        touch_left = x <= self.border_margin
        touch_right = x + width >= image_width - self.border_margin
        touch_top = y <= self.border_margin
        touch_bottom = y + height >= image_height - self.border_margin

        self.measurements.append(
            {
                "received_at": time.monotonic(),
                "image_width": image_width,
                "image_height": image_height,
                "center_u": center_u,
                "center_v": center_v,
                "area": area,
                "touch_left": touch_left,
                "touch_right": touch_right,
                "touch_top": touch_top,
                "touch_bottom": touch_bottom,
            }
        )

    def observe_target(self, timeout_sec):
        self.measurements.clear()
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            if len(self.measurements) >= self.sample_count:
                break

        if len(self.measurements) < self.sample_count:
            raise RuntimeError(
                f"只有{len(self.measurements)}帧有效mask；"
                f"需要{self.sample_count}帧"
            )

        samples = list(self.measurements)[-self.sample_count :]

        image_widths = {
            item["image_width"] for item in samples
        }
        image_heights = {
            item["image_height"] for item in samples
        }

        if len(image_widths) != 1 or len(image_heights) != 1:
            raise RuntimeError("采样期间mask尺寸发生变化")

        image_width = image_widths.pop()
        image_height = image_heights.pop()

        center_u = float(
            np.median(
                [item["center_u"] for item in samples]
            )
        )
        center_v = float(
            np.median(
                [item["center_v"] for item in samples]
            )
        )
        area = int(
            np.median(
                [item["area"] for item in samples]
            )
        )

        majority = self.sample_count // 2 + 1

        touch_left = (
            sum(item["touch_left"] for item in samples)
            >= majority
        )
        touch_right = (
            sum(item["touch_right"] for item in samples)
            >= majority
        )
        touch_top = (
            sum(item["touch_top"] for item in samples)
            >= majority
        )
        touch_bottom = (
            sum(item["touch_bottom"] for item in samples)
            >= majority
        )

        return {
            "image_width": image_width,
            "image_height": image_height,
            "center_u": center_u,
            "center_v": center_v,
            "pixel_error": center_u - 0.5 * image_width,
            "area": area,
            "touch_left": touch_left,
            "touch_right": touch_right,
            "touch_top": touch_top,
            "touch_bottom": touch_bottom,
        }

    def latest_target(self, maximum_age_sec):
        now = time.monotonic()
        samples = [
            item
            for item in self.measurements
            if now - item["received_at"] <= maximum_age_sec
        ]

        if len(samples) < self.sample_count:
            return None

        samples = samples[-self.sample_count :]
        image_widths = {
            item["image_width"] for item in samples
        }
        image_heights = {
            item["image_height"] for item in samples
        }

        if len(image_widths) != 1 or len(image_heights) != 1:
            self.measurements.clear()
            return None

        image_width = image_widths.pop()
        image_height = image_heights.pop()
        center_u = float(
            np.median(
                [item["center_u"] for item in samples]
            )
        )
        center_v = float(
            np.median(
                [item["center_v"] for item in samples]
            )
        )
        area = int(
            np.median(
                [item["area"] for item in samples]
            )
        )
        majority = self.sample_count // 2 + 1

        return {
            "image_width": image_width,
            "image_height": image_height,
            "center_u": center_u,
            "center_v": center_v,
            "pixel_error": center_u - 0.5 * image_width,
            "area": area,
            "touch_left": (
                sum(item["touch_left"] for item in samples)
                >= majority
            ),
            "touch_right": (
                sum(item["touch_right"] for item in samples)
                >= majority
            ),
            "touch_top": (
                sum(item["touch_top"] for item in samples)
                >= majority
            ),
            "touch_bottom": (
                sum(item["touch_bottom"] for item in samples)
                >= majority
            ),
        }


def parse_args():
    parser = argparse.ArgumentParser(
        description="根据语义分割mask小幅调整观察关节，使目标横向居中"
    )

    parser.add_argument(
        "--target-class",
        choices=tuple(CLASS_IDS),
        required=True,
    )
    parser.add_argument(
        "--execution-backend",
        choices=("real", "simulation"),
        default="real",
        help="最终执行通道；默认real，simulation使用ros2_control action",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
    )
    parser.add_argument(
        "--inspect-only",
        action="store_true",
    )
    parser.add_argument(
        "--confirm",
        default="",
    )
    parser.add_argument(
        "--pan-joint",
        type=int,
        default=1,
    )
    parser.add_argument(
        "--pixels-per-radian",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--gain",
        type=float,
        default=0.60,
    )
    parser.add_argument(
        "--tolerance-pixels",
        type=float,
        default=25.0,
    )
    parser.add_argument(
        "--border-margin-pixels",
        type=int,
        default=15,
    )
    parser.add_argument(
        "--minimum-step-rad",
        type=float,
        default=0.015,
    )
    parser.add_argument(
        "--maximum-step-rad",
        type=float,
        default=0.040,
    )
    parser.add_argument(
        "--maximum-total-offset-rad",
        type=float,
        default=0.150,
    )
    parser.add_argument(
        "--maximum-iterations",
        type=int,
        default=4,
    )
    parser.add_argument(
        "--sample-count",
        type=int,
        default=5,
    )
    parser.add_argument(
        "--mask-timeout",
        type=float,
        default=4.0,
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--follow",
        action="store_true",
        help=(
            "持续横向跟随目标，直到按Ctrl+C停止。"
            "必须同时使用--execute和正确的--confirm。"
        ),
    )
    parser.add_argument(
        "--fast-follow",
        action="store_true",
        help=(
            "加速跟随：复用碰撞场景、跳过每步RViz轨迹发布，"
            "并允许将--settle-seconds降到0.1秒。"
        ),
    )
    parser.add_argument(
        "--servo-follow",
        action="store_true",
        help=(
            "高频joint1伺服跟随；启动前检查整个允许扫描区间，"
            "运行中不再逐步调用MoveIt。"
        ),
    )
    parser.add_argument(
        "--workspace-follow",
        action="store_true",
        help=(
            "大范围三维工作区跟随；使用手眼标定后的目标点，"
            "同时实现绕底座旋转和前后伸展。"
        ),
    )
    parser.add_argument(
        "--servo-preflight-only",
        action="store_true",
        help=(
            "只规划并验证SERVO或工作区允许范围；"
            "不创建命令发布者，不移动机械臂。"
        ),
    )
    parser.add_argument(
        "--servo-rate-hz",
        type=float,
        default=20.0,
    )
    parser.add_argument(
        "--servo-max-speed-rad-s",
        type=float,
        default=0.20,
    )
    parser.add_argument(
        "--servo-max-accel-rad-s2",
        type=float,
        default=0.80,
    )
    parser.add_argument(
        "--servo-gain-per-second",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--servo-command-lead-rad",
        type=float,
        default=0.040,
    )
    parser.add_argument(
        "--servo-target-timeout",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--servo-max-target-jump-pixels",
        type=float,
        default=80.0,
    )
    parser.add_argument(
        "--servo-status-hz",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--workspace-max-angle-rad",
        type=float,
        default=0.80,
        help="相对启动姿态允许的最大左右旋转角。",
    )
    parser.add_argument(
        "--workspace-inward-m",
        type=float,
        default=0.01,
        help="目标向底座方向移动时允许的最大跟随距离。",
    )
    parser.add_argument(
        "--workspace-outward-m",
        type=float,
        default=0.15,
        help="目标远离底座时允许的最大跟随距离。",
    )
    parser.add_argument(
        "--workspace-ik-rate-hz",
        type=float,
        default=8.0,
    )
    parser.add_argument(
        "--workspace-target-timeout",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--workspace-max-target-jump-m",
        type=float,
        default=0.12,
    )
    parser.add_argument(
        "--workspace-filter-alpha",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--workspace-angular-deadband-rad",
        type=float,
        default=0.025,
    )
    parser.add_argument(
        "--workspace-radial-deadband-m",
        type=float,
        default=0.008,
    )
    parser.add_argument(
        "--workspace-ik-timeout",
        type=float,
        default=0.15,
    )
    parser.add_argument(
        "--stop-when-stable-seconds",
        type=float,
        default=0.0,
        help=(
            "工作区跟随中，目标与机械臂连续稳定指定秒数后"
            "保持并正常退出；0表示禁用。"
        ),
    )
    parser.add_argument(
        "--stable-position-spread-m",
        type=float,
        default=0.006,
        help="稳定窗口内三维目标点允许的最大离散半径。",
    )
    parser.add_argument(
        "--stable-center-error-pixels",
        type=float,
        default=30.0,
        help="触发稳定退出时允许的水平像素误差。",
    )
    parser.add_argument(
        "--stable-vertical-center-error-pixels",
        type=float,
        default=80.0,
        help=(
            "触发稳定退出时允许的垂直像素误差；"
            "工作区跟随不主动调整高度，因此该值独立于水平误差。"
        ),
    )
    parser.add_argument(
        "--stable-joint-error-rad",
        type=float,
        default=0.030,
        help="触发稳定退出时IK目标与实测关节的最大误差。",
    )
    parser.add_argument(
        "--follow-seconds",
        type=float,
        default=0.0,
        help=(
            "按指定秒数持续横向跟随；0表示一次性居中。"
            "跟随模式只做横向观察关节调整，不执行抓取。"
        ),
    )
    parser.add_argument(
        "--slowdown",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--speed-percent",
        type=int,
        default=10,
    )
    parser.add_argument(
        "--effort",
        type=float,
        default=0.5,
    )
    parser.add_argument(
        "--tracking-limit",
        type=float,
        default=0.20,
    )

    args = parser.parse_args()

    if args.inspect_only and args.execute:
        parser.error(
            "--inspect-only不能与--execute同时使用"
        )

    if args.follow and args.follow_seconds > 0.0:
        parser.error(
            "--follow不能与--follow-seconds同时使用"
        )

    if args.follow and (
        args.servo_follow
        or args.workspace_follow
    ):
        parser.error(
            "--follow不能与SERVO/工作区跟随同时使用"
        )

    if args.servo_follow and args.workspace_follow:
        parser.error(
            "--servo-follow不能与--workspace-follow同时使用"
        )

    if args.fast_follow and (
        args.servo_follow
        or args.workspace_follow
    ):
        parser.error(
            "--fast-follow不能与SERVO/工作区跟随同时使用"
        )

    if (
        args.servo_preflight_only
        and not (
            args.servo_follow
            or args.workspace_follow
        )
    ):
        parser.error(
            "--servo-preflight-only必须与"
            "--servo-follow或--workspace-follow同时使用"
        )

    if (
        args.servo_preflight_only
        and (
            args.execute
            or args.follow_seconds > 0.0
            or args.stop_when_stable_seconds > 0.0
        )
    ):
        parser.error(
            "预检模式不能使用--execute、--follow-seconds或"
            "--stop-when-stable-seconds"
        )

    follow_requested = (
        args.follow
        or args.servo_follow
        or args.workspace_follow
        or args.follow_seconds > 0.0
    )

    if args.inspect_only and follow_requested:
        parser.error(
            "--inspect-only不能与跟随模式同时使用"
        )

    if args.fast_follow and not follow_requested:
        parser.error(
            "--fast-follow必须与--follow或"
            "--follow-seconds同时使用"
        )

    if (
        follow_requested
        and not args.execute
        and not args.servo_preflight_only
    ):
        parser.error(
            "跟随模式必须同时使用--execute；"
            "仅查看目标位置请使用--inspect-only"
        )

    if args.execute and args.confirm != CONFIRM_TOKEN:
        parser.error(
            f"实际执行需要 --confirm {CONFIRM_TOKEN}"
        )

    if not 1 <= args.pan_joint <= 6:
        parser.error(
            "--pan-joint必须在1到6之间"
        )

    if (
        not args.inspect_only
        and not args.workspace_follow
        and (
            not math.isfinite(args.pixels_per_radian)
            or abs(args.pixels_per_radian) < 50.0
        )
    ):
        parser.error(
            "非inspect模式下，"
            "--pixels-per-radian绝对值必须至少为50"
        )

    if not 0.1 <= args.gain <= 1.0:
        parser.error(
            "--gain必须在0.1到1.0之间"
        )

    if args.tolerance_pixels <= 0.0:
        parser.error(
            "--tolerance-pixels必须为正"
        )

    if args.border_margin_pixels < 0:
        parser.error(
            "--border-margin-pixels不能为负"
        )

    if not (
        0.001
        <= args.minimum_step_rad
        <= args.maximum_step_rad
    ):
        parser.error(
            "步长范围无效"
        )

    if not 0.005 <= args.maximum_step_rad <= 0.08:
        parser.error(
            "--maximum-step-rad必须在0.005到0.08之间"
        )

    if not (
        args.maximum_step_rad
        <= args.maximum_total_offset_rad
        <= 0.30
    ):
        parser.error(
            "--maximum-total-offset-rad范围无效"
        )

    if not 1 <= args.maximum_iterations <= 8:
        parser.error(
            "--maximum-iterations必须在1到8之间"
        )

    if not 3 <= args.sample_count <= 15:
        parser.error(
            "--sample-count必须在3到15之间"
        )

    minimum_settle_seconds = (
        0.1
        if (
            args.fast_follow
            or args.servo_follow
            or args.workspace_follow
        )
        else 0.5
    )

    if not (
        minimum_settle_seconds
        <= args.settle_seconds
        <= 3.0
    ):
        parser.error(
            "--settle-seconds必须在"
            f"{minimum_settle_seconds:.1f}到3.0之间"
        )

    if not 0.0 <= args.follow_seconds <= 600.0:
        parser.error(
            "--follow-seconds必须在0到600秒之间"
        )

    if not 10.0 <= args.servo_rate_hz <= 30.0:
        parser.error(
            "--servo-rate-hz必须在10到30之间"
        )

    if not 0.03 <= args.servo_max_speed_rad_s <= 0.30:
        parser.error(
            "--servo-max-speed-rad-s必须在0.03到0.30之间"
        )

    if not 0.10 <= args.servo_max_accel_rad_s2 <= 2.0:
        parser.error(
            "--servo-max-accel-rad-s2必须在0.10到2.0之间"
        )

    if not 0.20 <= args.servo_gain_per_second <= 5.0:
        parser.error(
            "--servo-gain-per-second必须在0.20到5.0之间"
        )

    if not 0.010 <= args.servo_command_lead_rad <= 0.075:
        parser.error(
            "--servo-command-lead-rad必须在0.010到0.075之间"
        )

    if not 0.10 <= args.servo_target_timeout <= 1.0:
        parser.error(
            "--servo-target-timeout必须在0.10到1.0之间"
        )

    if not 30.0 <= args.servo_max_target_jump_pixels <= 200.0:
        parser.error(
            "--servo-max-target-jump-pixels必须在30到200之间"
        )

    if not 0.5 <= args.servo_status_hz <= 10.0:
        parser.error(
            "--servo-status-hz必须在0.5到10.0之间"
        )

    if not 0.20 <= args.workspace_max_angle_rad <= 1.20:
        parser.error(
            "--workspace-max-angle-rad必须在0.20到1.20之间"
        )

    if not 0.0 <= args.workspace_inward_m <= 0.12:
        parser.error(
            "--workspace-inward-m必须在0到0.12之间"
        )

    if not 0.03 <= args.workspace_outward_m <= 0.30:
        parser.error(
            "--workspace-outward-m必须在0.03到0.30之间"
        )

    if not 2.0 <= args.workspace_ik_rate_hz <= 15.0:
        parser.error(
            "--workspace-ik-rate-hz必须在2到15之间"
        )

    if not 0.20 <= args.workspace_target_timeout <= 1.0:
        parser.error(
            "--workspace-target-timeout必须在0.20到1.0之间"
        )

    if not 0.04 <= args.workspace_max_target_jump_m <= 0.30:
        parser.error(
            "--workspace-max-target-jump-m必须在0.04到0.30之间"
        )

    if not 0.05 <= args.workspace_filter_alpha <= 1.0:
        parser.error(
            "--workspace-filter-alpha必须在0.05到1.0之间"
        )

    if not 0.0 <= args.workspace_angular_deadband_rad <= 0.10:
        parser.error(
            "--workspace-angular-deadband-rad必须在0到0.10之间"
        )

    if not 0.0 <= args.workspace_radial_deadband_m <= 0.03:
        parser.error(
            "--workspace-radial-deadband-m必须在0到0.03之间"
        )

    if not 0.05 <= args.workspace_ik_timeout <= 0.50:
        parser.error(
            "--workspace-ik-timeout必须在0.05到0.50之间"
        )

    if (
        args.stop_when_stable_seconds > 0.0
        and not args.workspace_follow
    ):
        parser.error(
            "--stop-when-stable-seconds仅用于--workspace-follow"
        )

    if not (
        args.stop_when_stable_seconds == 0.0
        or 2.0 <= args.stop_when_stable_seconds <= 30.0
    ):
        parser.error(
            "--stop-when-stable-seconds必须为0或在2到30之间"
        )

    if (
        args.stop_when_stable_seconds > 0.0
        and args.follow_seconds > 0.0
        and args.follow_seconds
        < args.stop_when_stable_seconds + 5.0
    ):
        parser.error(
            "--follow-seconds必须至少比稳定触发时间长5秒"
        )

    if not 0.002 <= args.stable_position_spread_m <= 0.020:
        parser.error(
            "--stable-position-spread-m必须在0.002到0.020之间"
        )

    if not 10.0 <= args.stable_center_error_pixels <= 80.0:
        parser.error(
            "--stable-center-error-pixels必须在10到80之间"
        )

    if not (
        20.0
        <= args.stable_vertical_center_error_pixels
        <= 120.0
    ):
        parser.error(
            "--stable-vertical-center-error-pixels必须在20到120之间"
        )

    if not 0.010 <= args.stable_joint_error_rad <= 0.060:
        parser.error(
            "--stable-joint-error-rad必须在0.010到0.060之间"
        )

    if not 1.5 <= args.slowdown <= 4.0:
        parser.error(
            "--slowdown必须在1.5到4.0之间"
        )

    if not 1 <= args.speed_percent <= 15:
        parser.error(
            "--speed-percent必须在1到15之间"
        )

    if not 0.5 <= args.effort <= 1.0:
        parser.error(
            "--effort必须在0.5到1.0之间"
        )

    if not 0.10 <= args.tracking_limit <= 0.25:
        parser.error(
            "--tracking-limit必须在0.10到0.25 rad之间"
        )

    servo_tracking_limit = min(
        args.tracking_limit,
        0.10,
    )
    if (
        args.servo_command_lead_rad
        > 0.75 * servo_tracking_limit
    ):
        parser.error(
            "--servo-command-lead-rad相对跟踪误差上限过大"
        )

    return args


def bounded_step(
    raw_step,
    minimum_step,
    maximum_step,
):
    if abs(raw_step) < 1e-12:
        return 0.0

    magnitude = min(
        max(
            abs(raw_step),
            minimum_step,
        ),
        maximum_step,
    )

    return math.copysign(
        magnitude,
        raw_step,
    )


def clamp_value(value, lower, upper):
    return min(
        max(value, lower),
        upper,
    )


def approach_value(current, target, maximum_delta):
    delta = clamp_value(
        target - current,
        -maximum_delta,
        maximum_delta,
    )
    return current + delta


def desired_servo_rate(
    pixel_error,
    pixels_per_radian,
    gain_per_second,
    maximum_speed,
):
    raw_rate = (
        -gain_per_second
        * pixel_error
        / pixels_per_radian
    )
    return clamp_value(
        raw_rate,
        -maximum_speed,
        maximum_speed,
    )


def next_servo_command(
    command_pan,
    actual_pan,
    current_rate,
    desired_rate,
    delta_time,
    maximum_acceleration,
    lower_pan,
    upper_pan,
    maximum_command_lead,
):
    rate = approach_value(
        current_rate,
        desired_rate,
        maximum_acceleration * delta_time,
    )
    proposed = command_pan + rate * delta_time
    lead_lower = max(
        lower_pan,
        actual_pan - maximum_command_lead,
    )
    lead_upper = min(
        upper_pan,
        actual_pan + maximum_command_lead,
    )
    proposed = clamp_value(
        proposed,
        lead_lower,
        lead_upper,
    )

    if (
        proposed <= lower_pan + 1e-9
        and rate < 0.0
    ):
        rate = 0.0
    elif (
        proposed >= upper_pan - 1e-9
        and rate > 0.0
    ):
        rate = 0.0

    return proposed, rate


def validate_servo_sweep(
    motion_response,
    baseline_positions,
    pan_index,
    target_pan,
):
    trajectory = motion_response.trajectory.joint_trajectory
    lower_pan = min(
        baseline_positions[pan_index],
        target_pan,
    )
    upper_pan = max(
        baseline_positions[pan_index],
        target_pan,
    )
    maximum_other_joint_drift = 0.0

    for point in trajectory.points:
        positions = [
            float(value)
            for value in point.positions
        ]
        pan = positions[pan_index]
        if not (
            lower_pan - 0.010
            <= pan
            <= upper_pan + 0.010
        ):
            raise RuntimeError(
                "伺服扫描区间预检轨迹超出joint1包络"
            )

        for joint_index, value in enumerate(positions):
            if joint_index == pan_index:
                continue
            maximum_other_joint_drift = max(
                maximum_other_joint_drift,
                abs(
                    value
                    - baseline_positions[joint_index]
                ),
            )

    if maximum_other_joint_drift > 0.015:
        raise RuntimeError(
            "伺服扫描区间预检需要其他关节移动："
            f"{maximum_other_joint_drift:.4f}rad"
        )

    return maximum_other_joint_drift


def validate_servo_feedback(
    node,
    last_command,
    tracking_limit,
):
    now = time.monotonic()
    if now - node.latest_joint_received_at > 0.25:
        raise RuntimeError("伺服期间关节反馈中断")
    if node.execution_backend.requires_piper_status:
        if now - node.arm_status_received_at > 0.25:
            raise RuntimeError("伺服期间机械臂状态反馈中断")

        status = node.arm_status
        if (
            status is None
            or status.arm_status != 0
            or status.err_code != 0
        ):
            raise RuntimeError("伺服期间机械臂状态异常")
        if status.motion_status not in (0, 1):
            raise RuntimeError(
                "伺服期间运动状态异常："
                f"{status.motion_status}"
            )
        if any(
            (
                status.communication_status_joint_1,
                status.communication_status_joint_2,
                status.communication_status_joint_3,
                status.communication_status_joint_4,
                status.communication_status_joint_5,
                status.communication_status_joint_6,
            )
        ):
            raise RuntimeError("伺服期间出现关节通信异常")
        if any(
            (
                status.joint_1_angle_limit,
                status.joint_2_angle_limit,
                status.joint_3_angle_limit,
                status.joint_4_angle_limit,
                status.joint_5_angle_limit,
                status.joint_6_angle_limit,
            )
        ):
            raise RuntimeError("伺服期间触发关节角度限位")

    current = node.current_positions()
    node.validate_start_joint_limits(current[:6])
    tracking_error = max(
        abs(actual - target)
        for actual, target in zip(
            current[:6],
            last_command,
        )
    )
    if tracking_error > tracking_limit:
        raise RuntimeError(
            "伺服跟踪误差过大："
            f"{tracking_error:.4f}rad"
        )

    return current, tracking_error


def publish_servo_command(
    node,
    arm_positions,
    gripper_opening,
    speed_percent,
    effort,
):
    node.execution_backend.send_servo_command(
        arm_positions,
        gripper_opening,
        speed_percent,
        effort,
    )


def validate_short_trajectory(
    node,
    response,
    start_positions,
    goal_positions,
):
    try:
        return node.validate_trajectory(
            response,
            start_positions,
            goal_positions,
            minimum_duration=0.05,
        )
    except TypeError as error:
        if (
            "unexpected keyword argument "
            "'minimum_duration'"
            not in str(error)
        ):
            raise

        return node.validate_trajectory(
            response,
            start_positions,
            goal_positions,
        )


def preflight_servo_sweep(
    node,
    baseline_positions,
    pan_index,
    lower_pan,
    upper_pan,
):
    node.apply_table()
    start_state = node.current_robot_state()

    for label, target_pan in (
        ("lower", lower_pan),
        ("upper", upper_pan),
    ):
        goal_positions = list(
            baseline_positions[:6]
        )
        goal_positions[pan_index] = target_pan
        node.validate_joint_limits(
            goal_positions,
            margin=0.01,
        )
        response = node.plan_to_pregrasp(
            start_state,
            goal_positions,
        )
        duration, maximum_step, maximum_velocity = (
            validate_short_trajectory(
                node,
                response,
                baseline_positions,
                goal_positions,
            )
        )
        other_joint_drift = validate_servo_sweep(
            response,
            baseline_positions,
            pan_index,
            target_pan,
        )
        print(
            f"SERVO预检{label}: "
            f"joint={target_pan:.4f}rad, "
            f"duration={duration:.3f}s, "
            f"step={maximum_step:.4f}rad, "
            f"velocity={maximum_velocity:.4f}rad/s, "
            f"other_drift={other_joint_drift:.4f}rad"
        )


def run_servo_follow(
    node,
    args,
    start_positions,
    pan_index,
):
    baseline_positions = [
        float(value)
        for value in start_positions
    ]
    baseline_pan = baseline_positions[pan_index]
    lower_pan = (
        baseline_pan
        - args.maximum_total_offset_rad
    )
    upper_pan = (
        baseline_pan
        + args.maximum_total_offset_rad
    )

    print(
        "===== SERVO扫描区间预检 ====="
    )
    print(
        f"joint{args.pan_joint}: "
        f"{lower_pan:.4f}rad .. {upper_pan:.4f}rad"
    )
    preflight_servo_sweep(
        node,
        baseline_positions,
        pan_index,
        lower_pan,
        upper_pan,
    )

    node.spin_for(0.5)
    refreshed = node.validate_robot_state()
    preflight_drift = max(
        abs(actual - planned)
        for actual, planned in zip(
            refreshed[:6],
            baseline_positions[:6],
        )
    )
    if preflight_drift > 0.02:
        raise RuntimeError(
            "伺服预检期间机械臂移动了"
            f"{preflight_drift:.4f}rad"
        )

    if args.servo_preflight_only:
        print(
            "SERVO-PREFLIGHT-ONLY：扫描区间验证通过；"
            "没有创建/joint_states发布者，没有移动机械臂。"
        )
        return 0

    node.ensure_command_path_is_exclusive()
    node.prepare_command_publisher()
    node.spin_for(0.2)

    current = node.validate_robot_state()
    publisher_drift = max(
        abs(actual - planned)
        for actual, planned in zip(
            current[:6],
            baseline_positions[:6],
        )
    )
    if publisher_drift > 0.02:
        raise RuntimeError(
            "创建伺服命令发布者期间机械臂移动了"
            f"{publisher_drift:.4f}rad"
        )

    hard_tracking_limit = min(
        args.tracking_limit,
        0.10,
    )
    command_positions = list(current[:6])
    command_pan = float(current[pan_index])
    current_rate = 0.0
    gripper_opening = float(current[6])
    publish_servo_command(
        node,
        command_positions,
        gripper_opening,
        args.speed_percent,
        args.effort,
    )

    node.measurements.clear()
    warmup_deadline = (
        time.monotonic()
        + args.mask_timeout
    )
    measurement = None
    while (
        rclpy.ok()
        and time.monotonic() < warmup_deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.02)
        measurement = node.latest_target(
            args.servo_target_timeout
        )
        if measurement is not None:
            break

    if measurement is None:
        raise RuntimeError(
            "SERVO启动前没有获得稳定目标mask"
        )

    period = 1.0 / args.servo_rate_hz
    next_tick = time.monotonic()
    last_tick = next_tick
    next_status = next_tick
    follow_deadline = (
        next_tick + args.follow_seconds
        if args.follow_seconds > 0.0
        else None
    )
    last_center_u = measurement["center_u"]
    target_lost = False
    boundary_reported = False

    print(
        "===== SERVO-FOLLOW已启动 ====="
    )
    print(
        f"rate={args.servo_rate_hz:.1f}Hz, "
        f"max_speed={args.servo_max_speed_rad_s:.3f}rad/s, "
        f"max_accel={args.servo_max_accel_rad_s2:.3f}rad/s^2, "
        f"tracking_limit={hard_tracking_limit:.3f}rad"
    )

    while rclpy.ok():
        now = time.monotonic()
        if (
            follow_deadline is not None
            and now >= follow_deadline
        ):
            current = node.current_positions()
            node.publish_hold(
                current[6],
                args.speed_percent,
                args.effort,
            )
            print(
                f"SERVO跟随{args.follow_seconds:.1f}秒完成；"
                "保持当前位置。"
            )
            return 0

        next_tick += period
        while rclpy.ok():
            remaining = next_tick - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(
                node,
                timeout_sec=min(0.01, remaining),
            )

        now = time.monotonic()
        delta_time = clamp_value(
            now - last_tick,
            0.001,
            2.0 * period,
        )
        last_tick = now
        if now - next_tick > period:
            next_tick = now

        current, tracking_error = validate_servo_feedback(
            node,
            command_positions,
            hard_tracking_limit,
        )
        actual_pan = float(current[pan_index])
        measurement = node.latest_target(
            args.servo_target_timeout
        )

        if measurement is None:
            if not target_lost:
                print(
                    "SERVO目标丢失；立即保持当前位置。"
                )
            target_lost = True
            last_center_u = None
            current_rate = 0.0
            command_pan = actual_pan
            command_positions = list(current[:6])
            publish_servo_command(
                node,
                command_positions,
                current[6],
                args.speed_percent,
                args.effort,
            )
            continue

        center_u = measurement["center_u"]
        if (
            last_center_u is not None
            and abs(center_u - last_center_u)
            > args.servo_max_target_jump_pixels
        ):
            print(
                "SERVO检测到目标中心突跳："
                f"{last_center_u:.1f}px -> {center_u:.1f}px；"
                "保持并重新获取目标。"
            )
            node.measurements.clear()
            last_center_u = None
            target_lost = True
            current_rate = 0.0
            command_pan = actual_pan
            command_positions = list(current[:6])
            publish_servo_command(
                node,
                command_positions,
                current[6],
                args.speed_percent,
                args.effort,
            )
            continue

        if target_lost:
            print("SERVO重新检测到稳定目标。")
            target_lost = False
        last_center_u = center_u

        pixel_error = measurement["pixel_error"]
        touching_horizontal_border = (
            measurement["touch_left"]
            or measurement["touch_right"]
        )
        centered = (
            abs(pixel_error)
            <= args.tolerance_pixels
            and not touching_horizontal_border
        )
        if centered:
            desired_rate = 0.0
        else:
            desired_rate = desired_servo_rate(
                pixel_error,
                args.pixels_per_radian,
                args.servo_gain_per_second,
                args.servo_max_speed_rad_s,
            )

        if (
            actual_pan <= lower_pan + 0.002
            and desired_rate < 0.0
        ) or (
            actual_pan >= upper_pan - 0.002
            and desired_rate > 0.0
        ):
            desired_rate = 0.0
            if not boundary_reported:
                print(
                    "SERVO已到允许扫描边界；保持并继续观察。"
                )
                boundary_reported = True
        else:
            boundary_reported = False

        command_pan, current_rate = next_servo_command(
            command_pan,
            actual_pan,
            current_rate,
            desired_rate,
            delta_time,
            args.servo_max_accel_rad_s2,
            lower_pan,
            upper_pan,
            args.servo_command_lead_rad,
        )
        command_positions = list(
            baseline_positions[:6]
        )
        command_positions[pan_index] = command_pan
        node.validate_joint_limits(
            command_positions,
            margin=0.01,
        )
        publish_servo_command(
            node,
            command_positions,
            gripper_opening,
            args.speed_percent,
            args.effort,
        )

        if now >= next_status:
            print(
                "SERVO: "
                f"u={center_u:.1f}px, "
                f"error={pixel_error:+.1f}px, "
                f"rate={current_rate:+.3f}rad/s, "
                f"actual={actual_pan:.4f}rad, "
                f"command={command_pan:.4f}rad, "
                f"tracking={tracking_error:.4f}rad"
            )
            next_status = (
                now + 1.0 / args.servo_status_hz
            )

    return 0


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def rotate_quaternion_about_base_z(quaternion, angle):
    half_angle = 0.5 * angle
    sine = math.sin(half_angle)
    cosine = math.cos(half_angle)
    qx, qy, qz, qw = quaternion
    rotated = (
        cosine * qx - sine * qy,
        sine * qx + cosine * qy,
        cosine * qz + sine * qw,
        cosine * qw - sine * qz,
    )
    length = math.sqrt(
        sum(value * value for value in rotated)
    )
    if length < 1e-9:
        raise RuntimeError("工作区目标姿态四元数无效")
    return tuple(value / length for value in rotated)


def make_workspace_pose(
    baseline_pose,
    angular_offset,
    radial_offset,
):
    baseline_x = float(baseline_pose.position.x)
    baseline_y = float(baseline_pose.position.y)
    baseline_radius = math.hypot(
        baseline_x,
        baseline_y,
    )
    if baseline_radius < 0.04:
        raise RuntimeError(
            "当前末端过于接近底座轴线，不能启动工作区跟随"
        )

    target_radius = baseline_radius + radial_offset
    if target_radius < 0.04:
        raise RuntimeError(
            "工作区径向目标过于接近底座轴线"
        )

    baseline_angle = math.atan2(
        baseline_y,
        baseline_x,
    )
    target_angle = baseline_angle + angular_offset

    output = PoseStamped()
    output.header.frame_id = "base_link"
    output.pose.position.x = (
        target_radius * math.cos(target_angle)
    )
    output.pose.position.y = (
        target_radius * math.sin(target_angle)
    )
    output.pose.position.z = float(
        baseline_pose.position.z
    )

    rotated = rotate_quaternion_about_base_z(
        (
            float(baseline_pose.orientation.x),
            float(baseline_pose.orientation.y),
            float(baseline_pose.orientation.z),
            float(baseline_pose.orientation.w),
        ),
        angular_offset,
    )
    (
        output.pose.orientation.x,
        output.pose.orientation.y,
        output.pose.orientation.z,
        output.pose.orientation.w,
    ) = rotated
    return output


def compute_workspace_ik(
    node,
    target_pose,
    seed_positions,
    timeout_sec,
):
    request = GetPositionIK.Request()
    request.ik_request.group_name = "arm"
    request.ik_request.ik_link_name = "link6"
    request.ik_request.pose_stamped = copy.deepcopy(
        target_pose
    )
    request.ik_request.robot_state = (
        node.current_robot_state()
    )
    request.ik_request.robot_state.joint_state.name = (
        list(ARM_JOINTS)
    )
    request.ik_request.robot_state.joint_state.position = (
        list(seed_positions[:6])
    )
    request.ik_request.robot_state.is_diff = True
    request.ik_request.avoid_collisions = True
    request.ik_request.timeout.sec = int(timeout_sec)
    request.ik_request.timeout.nanosec = int(
        (timeout_sec - int(timeout_sec)) * 1e9
    )

    response = node.call_service(
        node.ik_client,
        request,
        max(0.40, timeout_sec + 0.25),
    )
    if (
        int(response.error_code.val)
        != MoveItErrorCodes.SUCCESS
    ):
        raise RuntimeError(
            "工作区IK失败："
            f"{int(response.error_code.val)}"
        )

    positions_by_name = dict(
        zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        )
    )
    if not all(
        name in positions_by_name
        for name in ARM_JOINTS
    ):
        raise RuntimeError("工作区IK结果缺少机械臂关节")

    ordered = [
        float(positions_by_name[name])
        for name in ARM_JOINTS
    ]
    node.validate_joint_limits(
        ordered,
        margin=WORKSPACE_JOINT_LIMIT_MARGIN_RAD,
    )
    return ordered


def validate_workspace_envelope(
    positions,
    lower_envelope,
    upper_envelope,
):
    for index, value in enumerate(positions):
        if not (
            lower_envelope[index]
            <= value
            <= upper_envelope[index]
        ):
            raise RuntimeError(
                "工作区IK超出已预检的关节包络："
                f"joint{index + 1}={value:.4f}rad"
            )


def preflight_workspace(
    node,
    baseline_positions,
    baseline_pose,
    args,
):
    node.apply_table()
    start_state = node.current_robot_state()
    baseline_target_pose = make_workspace_pose(
        baseline_pose,
        0.0,
        0.0,
    )
    baseline_ik = compute_workspace_ik(
        node,
        baseline_target_pose,
        baseline_positions,
        max(0.20, args.workspace_ik_timeout),
    )
    baseline_ik_difference = max(
        abs(solution - actual)
        for solution, actual in zip(
            baseline_ik,
            baseline_positions[:6],
        )
    )
    if baseline_ik_difference > 0.05:
        raise RuntimeError(
            "当前末端姿态的IK分支与真实关节不一致："
            f"{baseline_ik_difference:.4f}rad"
        )
    print(
        "WORKSPACE预检baseline: "
        f"IK差={baseline_ik_difference:.4f}rad"
    )

    samples = []
    angle_samples = (
        ("left", -args.workspace_max_angle_rad),
        ("center", 0.0),
        ("right", args.workspace_max_angle_rad),
    )
    radial_samples = (
        ("near", -args.workspace_inward_m),
        ("center", 0.0),
        ("far", args.workspace_outward_m),
    )
    seen_offsets = set()
    for angle_label, angular_offset in angle_samples:
        for radial_label, radial_offset in radial_samples:
            key = (
                round(angular_offset, 9),
                round(radial_offset, 9),
            )
            if (
                key == (0.0, 0.0)
                or key in seen_offsets
            ):
                continue
            seen_offsets.add(key)
            samples.append(
                (
                    f"{angle_label}-{radial_label}",
                    angular_offset,
                    radial_offset,
                )
            )

    envelope_samples = [
        list(baseline_positions[:6]),
        baseline_ik,
    ]
    for label, angular_offset, radial_offset in samples:
        target_pose = make_workspace_pose(
            baseline_pose,
            angular_offset,
            radial_offset,
        )
        try:
            goal_positions = compute_workspace_ik(
                node,
                target_pose,
                baseline_positions,
                max(0.20, args.workspace_ik_timeout),
            )
        except RuntimeError as error:
            raise RuntimeError(
                f"WORKSPACE边界{label}不可达：{error}"
            ) from error
        response = node.plan_to_pregrasp(
            start_state,
            goal_positions,
        )
        (
            duration,
            maximum_step,
            maximum_velocity,
        ) = validate_short_trajectory(
            node,
            response,
            baseline_positions,
            goal_positions,
        )
        envelope_samples.append(goal_positions)
        envelope_samples.extend(
            [
                [
                    float(value)
                    for value in point.positions
                ]
                for point in (
                    response.trajectory
                    .joint_trajectory.points
                )
            ]
        )
        print(
            f"WORKSPACE预检{label}: "
            f"angle={angular_offset:+.3f}rad, "
            f"radial={radial_offset:+.3f}m, "
            f"duration={duration:.3f}s, "
            f"step={maximum_step:.4f}rad, "
            f"velocity={maximum_velocity:.4f}rad/s"
        )

    envelope_margin = 0.05
    lower_envelope = [
        min(sample[index] for sample in envelope_samples)
        - envelope_margin
        for index in range(6)
    ]
    upper_envelope = [
        max(sample[index] for sample in envelope_samples)
        + envelope_margin
        for index in range(6)
    ]
    print(
        "WORKSPACE关节包络(rad)："
        + ", ".join(
            f"j{index + 1}="
            f"{lower_envelope[index]:.3f}.."
            f"{upper_envelope[index]:.3f}"
            for index in range(6)
        )
    )
    return lower_envelope, upper_envelope


def next_workspace_command(
    command_positions,
    actual_positions,
    current_rates,
    goal_positions,
    delta_time,
    gain_per_second,
    maximum_speed,
    maximum_acceleration,
    maximum_command_lead,
    lower_envelope,
    upper_envelope,
):
    next_positions = []
    next_rates = []
    for index in range(6):
        desired_rate = clamp_value(
            gain_per_second
            * (
                goal_positions[index]
                - command_positions[index]
            ),
            -maximum_speed,
            maximum_speed,
        )
        next_position, next_rate = next_servo_command(
            command_positions[index],
            actual_positions[index],
            current_rates[index],
            desired_rate,
            delta_time,
            maximum_acceleration,
            lower_envelope[index],
            upper_envelope[index],
            maximum_command_lead,
        )
        next_positions.append(next_position)
        next_rates.append(next_rate)
    return next_positions, next_rates


def wait_for_workspace_target(node, args):
    deadline = time.monotonic() + args.mask_timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        measurement = node.latest_target(
            args.servo_target_timeout
        )
        base_point = node.get_latest_base_point(
            args.workspace_target_timeout
        )
        if (
            measurement is not None
            and base_point is not None
        ):
            return measurement, base_point
    measurement = node.latest_target(
        args.servo_target_timeout
    )
    base_point = node.get_latest_base_point(
        args.workspace_target_timeout
    )
    if measurement is not None and base_point is None:
        raise RuntimeError(
            "已识别目标mask，但没有该类别的有效三维点；"
            "检查深度图、激光/LDP状态和所选类别的"
            "*_point_base话题"
        )
    if measurement is None and base_point is not None:
        raise RuntimeError(
            "已有三维目标点，但所选类别的mask不连续；"
            "确认画面中只有一个目标且没有接触图像边界"
        )
    raise RuntimeError(
        "所选类别既没有稳定mask，也没有有效三维点"
    )


def stable_point_window(points):
    center = tuple(
        float(
            np.median(
                [point[axis] for point in points]
            )
        )
        for axis in range(3)
    )
    spread = max(
        math.sqrt(
            sum(
                (point[axis] - center[axis]) ** 2
                for axis in range(3)
            )
        )
        for point in points
    )
    return center, spread


def point_in_grasp_workspace(point):
    x, y, z = point
    return (
        GRASP_WORKSPACE_X_MIN
        <= x
        <= GRASP_WORKSPACE_X_MAX
        and abs(y) <= GRASP_WORKSPACE_Y_ABS_MAX
        and GRASP_WORKSPACE_Z_MIN
        <= z
        <= GRASP_WORKSPACE_Z_MAX
    )


def run_workspace_follow(
    node,
    args,
    start_positions,
):
    baseline_positions = [
        float(value)
        for value in start_positions
    ]
    baseline_pose = copy.deepcopy(node.end_pose)

    try:
        node.validate_joint_limits(
            baseline_positions,
            margin=WORKSPACE_JOINT_LIMIT_MARGIN_RAD,
        )
    except RuntimeError as error:
        raise RuntimeError(
            "当前姿态距离关节限位太近，不能启动WORKSPACE跟随；"
            "请先用move_to_observe执行低速恢复动作。"
            f" 详情：{error}"
        ) from error

    print("===== 大范围WORKSPACE扫描预检 =====")
    print(
        f"左右角度=±{args.workspace_max_angle_rad:.3f}rad, "
        f"向内={args.workspace_inward_m:.3f}m, "
        f"向外={args.workspace_outward_m:.3f}m"
    )
    (
        lower_envelope,
        upper_envelope,
    ) = preflight_workspace(
        node,
        baseline_positions,
        baseline_pose,
        args,
    )

    node.spin_for(0.5)
    refreshed = node.validate_robot_state()
    preflight_drift = max(
        abs(actual - planned)
        for actual, planned in zip(
            refreshed[:6],
            baseline_positions[:6],
        )
    )
    if preflight_drift > 0.02:
        raise RuntimeError(
            "工作区预检期间机械臂移动了"
            f"{preflight_drift:.4f}rad"
        )

    if args.servo_preflight_only:
        print(
            "WORKSPACE-PREFLIGHT-ONLY：全部边界验证通过；"
            "没有创建/joint_states发布者，没有移动机械臂。"
        )
        return 0

    measurement, baseline_target = (
        wait_for_workspace_target(node, args)
    )
    horizontal_error = measurement["pixel_error"]
    vertical_error = (
        measurement["center_v"]
        - 0.5 * measurement["image_height"]
    )
    startup_tolerance = max(
        50.0,
        2.0 * args.tolerance_pixels,
    )
    if (
        abs(horizontal_error) > startup_tolerance
        or abs(vertical_error) > startup_tolerance
        or measurement["touch_left"]
        or measurement["touch_right"]
        or measurement["touch_top"]
        or measurement["touch_bottom"]
    ):
        raise RuntimeError(
            "启动WORKSPACE跟随前请先把目标放到画面中心附近："
            f"u误差={horizontal_error:+.1f}px, "
            f"v误差={vertical_error:+.1f}px"
        )

    baseline_target_radius = math.hypot(
        baseline_target[0],
        baseline_target[1],
    )
    if baseline_target_radius < 0.08:
        raise RuntimeError(
            "三维目标过于接近底座，拒绝启动工作区跟随"
        )
    baseline_target_angle = math.atan2(
        baseline_target[1],
        baseline_target[0],
    )

    node.ensure_command_path_is_exclusive()
    node.prepare_command_publisher()
    node.spin_for(0.2)
    current = node.validate_robot_state()

    command_positions = list(current[:6])
    current_rates = [0.0] * 6
    ik_goal_positions = list(current[:6])
    gripper_opening = float(current[6])
    publish_servo_command(
        node,
        command_positions,
        gripper_opening,
        args.speed_percent,
        args.effort,
    )

    period = 1.0 / args.servo_rate_hz
    ik_period = 1.0 / args.workspace_ik_rate_hz
    now = time.monotonic()
    next_tick = now
    last_tick = now
    next_ik = now
    next_status = now
    follow_deadline = (
        now + args.follow_seconds
        if args.follow_seconds > 0.0
        else None
    )
    filtered_angle = 0.0
    filtered_radius = 0.0
    last_target_point = baseline_target
    target_lost = False
    boundary_reported = False
    grasp_workspace_reported = False
    consecutive_ik_failures = 0
    stability_samples = deque(
        maxlen=max(
            300,
            int(
                args.servo_rate_hz
                * (
                    args.stop_when_stable_seconds
                    + 2.0
                )
            ),
        )
    )
    last_stability_point_received_at = 0.0
    stability_progress = 0.0
    hard_tracking_limit = min(
        args.tracking_limit,
        0.10,
    )

    print("===== WORKSPACE-FOLLOW已启动 =====")
    print(
        f"servo={args.servo_rate_hz:.1f}Hz, "
        f"IK={args.workspace_ik_rate_hz:.1f}Hz, "
        f"max_speed={args.servo_max_speed_rad_s:.3f}rad/s, "
        f"max_accel={args.servo_max_accel_rad_s2:.3f}rad/s^2"
    )
    if args.stop_when_stable_seconds > 0.0:
        print(
            "稳定触发已启用：目标与机械臂连续稳定"
            f"{args.stop_when_stable_seconds:.1f}秒后停止跟随，"
            "供后续抓取流程接管。"
        )

    while rclpy.ok():
        now = time.monotonic()
        if (
            follow_deadline is not None
            and now >= follow_deadline
        ):
            current = node.current_positions()
            node.publish_hold(
                current[6],
                args.speed_percent,
                args.effort,
            )
            if args.stop_when_stable_seconds > 0.0:
                raise RuntimeError(
                    f"{args.follow_seconds:.1f}秒内未达到连续"
                    f"{args.stop_when_stable_seconds:.1f}秒稳定条件"
                )
            print(
                f"大范围动态跟随{args.follow_seconds:.1f}秒完成；"
                "保持当前位置。"
            )
            return 0

        next_tick += period
        while rclpy.ok():
            remaining = next_tick - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(
                node,
                timeout_sec=min(0.01, remaining),
            )

        now = time.monotonic()
        delta_time = clamp_value(
            now - last_tick,
            0.001,
            2.0 * period,
        )
        last_tick = now
        if now - next_tick > period:
            next_tick = now

        current, tracking_error = validate_servo_feedback(
            node,
            command_positions,
            hard_tracking_limit,
        )
        measurement = node.latest_target(
            args.servo_target_timeout
        )
        base_point = node.get_latest_base_point(
            args.workspace_target_timeout
        )

        target_invalid = (
            measurement is None
            or base_point is None
            or (
                measurement is not None
                and (
                    measurement["touch_left"]
                    or measurement["touch_right"]
                    or measurement["touch_top"]
                    or measurement["touch_bottom"]
                )
            )
        )
        if target_invalid:
            if not target_lost:
                print(
                    "WORKSPACE目标或深度丢失；立即保持当前位置。"
                )
            target_lost = True
            stability_samples.clear()
            stability_progress = 0.0
            current_rates = [0.0] * 6
            command_positions = list(current[:6])
            ik_goal_positions = list(current[:6])
            publish_servo_command(
                node,
                command_positions,
                current[6],
                args.speed_percent,
                args.effort,
            )
            continue

        target_jump = math.sqrt(
            sum(
                (base_point[index] - last_target_point[index])
                ** 2
                for index in range(3)
            )
        )
        if (
            target_jump
            > args.workspace_max_target_jump_m
        ):
            print(
                "WORKSPACE检测到三维目标突跳"
                f"{target_jump:.3f}m；保持并重新获取目标。"
            )
            last_target_point = base_point
            target_lost = True
            stability_samples.clear()
            stability_progress = 0.0
            current_rates = [0.0] * 6
            command_positions = list(current[:6])
            ik_goal_positions = list(current[:6])
            publish_servo_command(
                node,
                command_positions,
                current[6],
                args.speed_percent,
                args.effort,
            )
            continue

        last_target_point = base_point
        if target_lost:
            print("WORKSPACE重新检测到稳定目标。")
            target_lost = False

        grasp_workspace_ok = point_in_grasp_workspace(
            base_point
        )
        if (
            args.stop_when_stable_seconds > 0.0
            and not grasp_workspace_ok
            and not grasp_workspace_reported
        ):
            print(
                "WORKSPACE目标位于跟踪范围内，但不在抓取安全区："
                f"xyz=({base_point[0]:.3f}, "
                f"{base_point[1]:.3f}, "
                f"{base_point[2]:.3f})m；"
                "仅继续跟踪，不会触发抓取。"
            )
            grasp_workspace_reported = True
        elif grasp_workspace_ok:
            grasp_workspace_reported = False

        target_radius = math.hypot(
            base_point[0],
            base_point[1],
        )
        target_angle = math.atan2(
            base_point[1],
            base_point[0],
        )
        raw_angle = normalize_angle(
            target_angle - baseline_target_angle
        )
        raw_radial = (
            target_radius - baseline_target_radius
        )
        bounded_angle = clamp_value(
            raw_angle,
            -args.workspace_max_angle_rad,
            args.workspace_max_angle_rad,
        )
        bounded_radial = clamp_value(
            raw_radial,
            -args.workspace_inward_m,
            args.workspace_outward_m,
        )
        at_boundary = (
            abs(raw_angle - bounded_angle) > 1e-6
            or abs(raw_radial - bounded_radial) > 1e-6
        )
        if at_boundary and not boundary_reported:
            print(
                "WORKSPACE目标超出已预检范围；"
                "跟随到安全边界并保持。"
            )
            boundary_reported = True
        elif not at_boundary:
            boundary_reported = False

        alpha = args.workspace_filter_alpha
        filtered_angle = (
            alpha * bounded_angle
            + (1.0 - alpha) * filtered_angle
        )
        filtered_radius = (
            alpha * bounded_radial
            + (1.0 - alpha) * filtered_radius
        )
        commanded_angle = (
            0.0
            if abs(filtered_angle)
            <= args.workspace_angular_deadband_rad
            else filtered_angle
        )
        commanded_radial = (
            0.0
            if abs(filtered_radius)
            <= args.workspace_radial_deadband_m
            else filtered_radius
        )

        if now >= next_ik:
            next_ik = now + ik_period
            desired_pose = make_workspace_pose(
                baseline_pose,
                commanded_angle,
                commanded_radial,
            )
            try:
                candidate = compute_workspace_ik(
                    node,
                    desired_pose,
                    current[:6],
                    args.workspace_ik_timeout,
                )
                validate_workspace_envelope(
                    candidate,
                    lower_envelope,
                    upper_envelope,
                )
                if max(
                    abs(goal - actual)
                    for goal, actual in zip(
                        candidate,
                        current[:6],
                    )
                ) > 1.0:
                    raise RuntimeError(
                        "IK解与当前姿态相差超过1.0rad"
                    )
                ik_goal_positions = candidate
                consecutive_ik_failures = 0
            except RuntimeError as error:
                consecutive_ik_failures += 1
                print(
                    "WORKSPACE IK暂时失败"
                    f"({consecutive_ik_failures}/5)：{error}"
                )
                current_rates = [0.0] * 6
                command_positions = list(current[:6])
                ik_goal_positions = list(current[:6])
                publish_servo_command(
                    node,
                    command_positions,
                    current[6],
                    args.speed_percent,
                    args.effort,
                )
                if consecutive_ik_failures >= 5:
                    raise RuntimeError(
                        "连续5次工作区IK失败"
                    )
                continue

        (
            command_positions,
            current_rates,
        ) = next_workspace_command(
            command_positions,
            current[:6],
            current_rates,
            ik_goal_positions,
            delta_time,
            args.servo_gain_per_second,
            args.servo_max_speed_rad_s,
            args.servo_max_accel_rad_s2,
            args.servo_command_lead_rad,
            lower_envelope,
            upper_envelope,
        )
        node.validate_joint_limits(
            command_positions,
            margin=WORKSPACE_JOINT_LIMIT_MARGIN_RAD,
        )
        publish_servo_command(
            node,
            command_positions,
            gripper_opening,
            args.speed_percent,
            args.effort,
        )

        horizontal_center_error = (
            measurement["center_u"]
            - 0.5 * measurement["image_width"]
        )
        vertical_center_error = (
            measurement["center_v"]
            - 0.5 * measurement["image_height"]
        )

        if args.stop_when_stable_seconds > 0.0:
            joint_goal_error = max(
                abs(goal - actual)
                for goal, actual in zip(
                    ik_goal_positions,
                    current[:6],
                )
            )
            maximum_rate = max(
                abs(rate)
                for rate in current_rates
            )
            stable_conditions = (
                not at_boundary
                and grasp_workspace_ok
                and abs(horizontal_center_error)
                <= args.stable_center_error_pixels
                and abs(vertical_center_error)
                <= args.stable_vertical_center_error_pixels
                and tracking_error
                <= args.stable_joint_error_rad
                and joint_goal_error
                <= args.stable_joint_error_rad
                and maximum_rate <= 0.025
            )

            point_received_at = (
                node.latest_base_point_received_at
            )
            if not stable_conditions:
                stability_samples.clear()
                stability_progress = 0.0
            elif (
                point_received_at
                > last_stability_point_received_at
            ):
                last_stability_point_received_at = (
                    point_received_at
                )
                stability_samples.append(
                    (
                        point_received_at,
                        tuple(base_point),
                    )
                )
                points = [
                    item[1]
                    for item in stability_samples
                ]
                _, stable_spread = stable_point_window(
                    points
                )
                if (
                    stable_spread
                    > args.stable_position_spread_m
                ):
                    newest = stability_samples[-1]
                    stability_samples.clear()
                    stability_samples.append(newest)
                    stability_progress = 0.0
                else:
                    stability_progress = (
                        stability_samples[-1][0]
                        - stability_samples[0][0]
                    )
                    minimum_samples = max(
                        15,
                        int(
                            args.stop_when_stable_seconds
                            * 5.0
                        ),
                    )
                    if (
                        stability_progress
                        >= args.stop_when_stable_seconds
                        and len(stability_samples)
                        >= minimum_samples
                    ):
                        current = node.current_positions()
                        node.publish_hold(
                            current[6],
                            args.speed_percent,
                            args.effort,
                        )
                        print(
                            "WORKSPACE_STABLE_TARGET："
                            f"{args.target_class}连续稳定"
                            f"{stability_progress:.1f}秒，"
                            f"三维离散={stable_spread:.4f}m；"
                            "保持当前位置并交接抓取。"
                        )
                        return 0

        if now >= next_status:
            print(
                "WORKSPACE: "
                f"u={measurement['center_u']:.1f}px, "
                f"v={measurement['center_v']:.1f}px, "
                f"angle={commanded_angle:+.3f}rad, "
                f"radial={commanded_radial:+.3f}m, "
                f"q1={current[0]:+.3f}rad, "
                f"tracking={tracking_error:.4f}rad, "
                f"center_error=({horizontal_center_error:+.1f},"
                f"{vertical_center_error:+.1f})px, "
                f"grasp_ws={grasp_workspace_ok}, "
                f"stable={stability_progress:.1f}/"
                f"{args.stop_when_stable_seconds:.1f}s"
            )
            next_status = (
                now + 1.0 / args.servo_status_hz
            )

    return 0


def main():
    args = parse_args()

    # Keep the ROS context valid while Python raises KeyboardInterrupt so the
    # handler can still publish a final hold command before shutdown.
    rclpy.init(
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = FoamTargetCenterNode(
        args.target_class,
        args.border_margin_pixels,
        args.sample_count,
        args.execution_backend,
    )

    pan_index = args.pan_joint - 1

    try:
        if args.inspect_only:
            measurement = node.observe_target(
                args.mask_timeout
            )

            print(
                f"target={args.target_class}, "
                f"center_u={measurement['center_u']:.1f}px, "
                f"center_v={measurement['center_v']:.1f}px, "
                f"error={measurement['pixel_error']:+.1f}px, "
                f"area={measurement['area']}px, "
                f"left={measurement['touch_left']}, "
                f"right={measurement['touch_right']}, "
                f"top={measurement['touch_top']}, "
                f"bottom={measurement['touch_bottom']}"
            )

            return 0

        node.wait_for_robot_inputs()
        node.wait_for_services()

        node.spin_for(0.8)

        start_positions = node.validate_robot_state()
        baseline_pan = float(
            start_positions[pan_index]
        )

        if args.workspace_follow:
            return run_workspace_follow(
                node,
                args,
                start_positions,
            )

        if args.servo_follow:
            return run_servo_follow(
                node,
                args,
                start_positions,
                pan_index,
            )

        if args.execute:
            node.ensure_command_path_is_exclusive()
            node.prepare_command_publisher()

        continuous_follow = args.follow
        follow_mode = (
            continuous_follow
            or args.follow_seconds > 0.0
        )
        follow_deadline = (
            time.monotonic() + args.follow_seconds
            if args.follow_seconds > 0.0
            else None
        )
        observation_index = 0
        movement_count = 0
        target_lost_since = None
        limit_warning_active = False
        table_applied = False

        if continuous_follow:
            print(
                "进入持续横向跟随模式；"
                "按Ctrl+C停止并保持当前位置。"
            )
        elif follow_deadline is not None:
            print(
                f"进入限时横向跟随模式："
                f"{args.follow_seconds:.1f}秒。"
            )

        if args.fast_follow:
            print(
                "FAST-FOLLOW：保留逐步MoveIt规划和轨迹校验；"
                "复用碰撞场景并跳过逐步RViz动画。"
            )

        while rclpy.ok():
            if (
                follow_deadline is not None
                and time.monotonic() >= follow_deadline
            ):
                current = node.current_positions()
                node.publish_hold(
                    current[6],
                    args.speed_percent,
                    args.effort,
                )
                print(
                    f"持续跟随{args.follow_seconds:.1f}秒完成；"
                    "保持当前位置，不执行抓取。"
                )
                return 0

            observation_timeout = args.mask_timeout
            if follow_deadline is not None:
                observation_timeout = min(
                    observation_timeout,
                    max(
                        0.05,
                        follow_deadline - time.monotonic(),
                    ),
                )

            observation_started_at = time.monotonic()

            try:
                measurement = node.observe_target(
                    observation_timeout
                )
            except RuntimeError as error:
                if not follow_mode:
                    raise

                if (
                    follow_deadline is not None
                    and time.monotonic() >= follow_deadline
                ):
                    current = node.current_positions()
                    node.publish_hold(
                        current[6],
                        args.speed_percent,
                        args.effort,
                    )
                    print(
                        f"持续跟随{args.follow_seconds:.1f}秒完成；"
                        "保持当前位置，不执行抓取。"
                    )
                    return 0

                if target_lost_since is None:
                    target_lost_since = observation_started_at
                lost_seconds = (
                    time.monotonic()
                    - target_lost_since
                )
                print(
                    "暂时没有获得足够的目标mask"
                    f"（已丢失约{lost_seconds:.1f}秒）：{error}；"
                    "保持当前位置并继续观察。"
                )
                node.spin_for(0.20)
                continue

            if target_lost_since is not None:
                print("已重新检测到目标，恢复横向跟随。")
                target_lost_since = None

            error = measurement["pixel_error"]

            touching_horizontal_border = (
                measurement["touch_left"]
                or measurement["touch_right"]
            )
            touching_vertical_border = (
                measurement["touch_top"]
                or measurement["touch_bottom"]
            )

            print(
                f"观测{observation_index}: "
                f"center_u={measurement['center_u']:.1f}px, "
                f"error={error:+.1f}px, "
                f"area={measurement['area']}px, "
                f"left={measurement['touch_left']}, "
                f"right={measurement['touch_right']}, "
                f"top={measurement['touch_top']}, "
                f"bottom={measurement['touch_bottom']}"
            )
            observation_index += 1

            if (
                abs(error) <= args.tolerance_pixels
                and not touching_horizontal_border
            ):
                if not follow_mode:
                    if touching_vertical_border:
                        raise RuntimeError(
                            "目标虽然横向居中，但仍触碰上/下图像边界；"
                            "当前单关节控制无法修正纵向截断，拒绝进入抓取。"
                        )

                    print(
                        "目标已横向居中且未接触图像边界。"
                    )
                    return 0

                if touching_vertical_border:
                    print(
                        "目标位于横向死区内，但触碰上/下边界；"
                        "当前只进行横向跟随。"
                    )
                else:
                    print(
                        "目标位于横向死区内；继续观察。"
                    )
                limit_warning_active = False
                node.spin_for(0.20)
                continue

            if (
                not follow_mode
                and movement_count >= args.maximum_iterations
            ):
                raise RuntimeError(
                    "达到最大调整次数，目标仍未居中"
                )

            raw_step = (
                -args.gain
                * error
                / args.pixels_per_radian
            )

            step = bounded_step(
                raw_step,
                args.minimum_step_rad,
                args.maximum_step_rad,
            )

            current = node.current_positions()

            proposed_pan = (
                float(current[pan_index])
                + step
            )

            total_offset = (
                proposed_pan
                - baseline_pan
            )

            if (
                abs(total_offset)
                > args.maximum_total_offset_rad
            ):
                proposed_pan = (
                    baseline_pan
                    + math.copysign(
                        args.maximum_total_offset_rad,
                        total_offset,
                    )
                )

                step = (
                    proposed_pan
                    - float(current[pan_index])
                )

            if abs(step) < 0.001:
                if follow_mode:
                    if not limit_warning_active:
                        print(
                            "已到本次跟随允许的总偏转边界；"
                            "保持当前位置并继续观察。"
                        )
                        limit_warning_active = True
                    node.spin_for(0.20)
                    continue

                raise RuntimeError(
                    "已到允许的总偏转边界，无法继续居中"
                )

            limit_warning_active = False

            goal_positions = [
                float(value)
                for value in current[:6]
            ]

            goal_positions[pan_index] = proposed_pan

            node.validate_joint_limits(
                goal_positions,
                margin=0.01,
            )

            if not args.fast_follow or not table_applied:
                node.apply_table()
                table_applied = True

            start_state = node.current_robot_state()

            response = node.plan_to_pregrasp(
                start_state,
                goal_positions,
            )

            try:
                trajectory_validation = node.validate_trajectory(
                    response,
                    current,
                    goal_positions,
                    minimum_duration=0.05,
                )
            except TypeError as error:
                if (
                    "unexpected keyword argument "
                    "'minimum_duration'"
                    not in str(error)
                ):
                    raise

                trajectory_validation = node.validate_trajectory(
                    response,
                    current,
                    goal_positions,
                )

            (
                duration,
                maximum_step,
                maximum_velocity,
            ) = trajectory_validation

            if not args.fast_follow:
                node.publish_display(
                    start_state,
                    response,
                )

            print(
                f"建议joint{args.pan_joint}"
                f"变化{step:+.4f}rad；"
                f"目标={proposed_pan:.4f}rad；"
                f"规划时长={duration:.3f}s；"
                f"轨迹最大步长="
                f"{maximum_step:.4f}rad；"
                f"最大速度="
                f"{maximum_velocity:.4f}rad/s"
            )

            if not args.execute:
                print(
                    "PLAN-ONLY：已生成一次调整轨迹，"
                    "没有移动机械臂。"
                )
                return 0

            refreshed = node.validate_robot_state()

            planning_drift = max(
                abs(actual - planned)
                for actual, planned in zip(
                    refreshed[:6],
                    current[:6],
                )
            )

            if planning_drift > 0.02:
                raise RuntimeError(
                    "规划期间机械臂移动了"
                    f"{planning_drift:.4f}rad"
                )

            node.execute_trajectory(
                response,
                refreshed,
                refreshed[6],
                args.slowdown,
                args.speed_percent,
                args.effort,
                args.tracking_limit,
            )

            node.spin_for(
                args.settle_seconds
            )

            node.validate_robot_state()
            movement_count += 1

        return 0

    except KeyboardInterrupt:
        if (
            rclpy.ok()
            and node.command_publisher is not None
            and node.latest_joint_state is not None
        ):
            current = node.current_positions()

            node.publish_hold(
                current[6],
                args.speed_percent,
                args.effort,
            )

        print(
            "已取消目标居中并尝试保持当前位置。"
        )

        return 130

    except Exception as error:
        if (
            rclpy.ok()
            and node.command_publisher is not None
            and node.latest_joint_state is not None
        ):
            current = node.current_positions()

            node.publish_hold(
                current[6],
                args.speed_percent,
                args.effort,
            )

        print(
            f"目标居中失败：{error}",
            file=sys.stderr,
        )

        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
