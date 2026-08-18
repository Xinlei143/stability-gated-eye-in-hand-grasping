#!/usr/bin/env python3
"""Conservative Piper gripper-only test for the local ROS 2 driver.

The driver consumes a complete sensor_msgs/JointState on /joint_states.  It
defaults every missing arm joint to zero, so this program always copies all six
arm positions from fresh, stable /joint_states_single feedback and changes only
the virtual gripper joint.

Running without --execute is read-only and prints the command that would be
sent.  Physical movement additionally requires an explicit confirmation token.
"""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Pose
from piper_msgs.msg import PiperStatusMsg
from piper_msgs.srv import Enable
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
]
CONFIRM_TOKEN = "GRIPPER_ONLY"


class PiperGripperSafeTest(Node):
    def __init__(self) -> None:
        super().__init__("piper_gripper_safe_test")
        self.samples = deque(maxlen=30)
        self.last_status = None
        self.last_pose = None
        self.last_feedback_time = 0.0
        self.last_status_time = 0.0
        self.last_pose_time = 0.0
        self.command_publisher = None

        self.feedback_subscription = self.create_subscription(
            JointState,
            "/joint_states_single",
            self.feedback_callback,
            10,
        )
        self.status_subscription = self.create_subscription(
            PiperStatusMsg,
            "/arm_status",
            self.status_callback,
            10,
        )
        self.pose_subscription = self.create_subscription(
            Pose,
            "/end_pose",
            self.pose_callback,
            10,
        )
        self.enable_client = self.create_client(Enable, "/enable_srv")

    def feedback_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            return
        values = [float(positions[name]) for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in values):
            return
        now = time.monotonic()
        self.samples.append((now, values))
        self.last_feedback_time = now

    def status_callback(self, msg: PiperStatusMsg) -> None:
        self.last_status = msg
        self.last_status_time = time.monotonic()

    def pose_callback(self, msg: Pose) -> None:
        self.last_pose = msg
        self.last_pose_time = time.monotonic()

    def spin_until_ready(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                len(self.samples) >= 10
                and self.last_status is not None
                and self.last_pose is not None
            ):
                return
        raise RuntimeError(
            "没有同时收到 /joint_states_single、/arm_status 和 /end_pose"
        )

    def validated_snapshot(
        self,
        min_tool_height: float,
        allow_not_at_target: bool = False,
    ) -> list[float]:
        now = time.monotonic()
        if now - self.last_feedback_time > 0.5:
            raise RuntimeError("关节反馈已过期")
        if now - self.last_status_time > 0.5:
            raise RuntimeError("机械臂状态已过期")
        if now - self.last_pose_time > 0.5:
            raise RuntimeError("末端位姿已过期")

        status = self.last_status
        if status.arm_status != 0 or status.err_code != 0:
            raise RuntimeError(
                f"机械臂状态异常: arm_status={status.arm_status}, "
                f"err_code={status.err_code}"
            )
        if status.motion_status != 0 and not allow_not_at_target:
            raise RuntimeError(
                "机械臂尚未到达上一个指定点: "
                f"motion_status={status.motion_status}"
            )
        communication_fields = [
            status.communication_status_joint_1,
            status.communication_status_joint_2,
            status.communication_status_joint_3,
            status.communication_status_joint_4,
            status.communication_status_joint_5,
            status.communication_status_joint_6,
        ]
        if any(communication_fields):
            raise RuntimeError("至少一个关节存在通信异常")

        if self.last_pose.position.z < min_tool_height:
            raise RuntimeError(
                f"末端高度只有 {self.last_pose.position.z:.3f} m，"
                f"低于安全阈值 {min_tool_height:.3f} m"
            )

        recent = list(self.samples)[-10:]
        arm_columns = [[sample[1][joint] for sample in recent] for joint in range(6)]
        spreads = [max(column) - min(column) for column in arm_columns]
        if max(spreads) > 0.01:
            raise RuntimeError(
                f"机械臂反馈不稳定，最大关节波动 {max(spreads):.4f} rad"
            )

        snapshot = list(recent[-1][1])
        if any(abs(value) > 3.5 for value in snapshot[:6]):
            raise RuntimeError("关节反馈超出合理范围")
        return snapshot

    def make_command(
        self,
        arm_positions: list[float],
        actual_opening_mm: float,
        multiplier: int,
        speed_percent: int,
        effort: float,
    ) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        # The local driver multiplies position[6] by gripper_val_mutiple.
        virtual_gripper_position = actual_opening_mm / 1000.0 / multiplier
        msg.position = list(arm_positions[:6]) + [virtual_gripper_position]
        # In this driver velocity[6] selects the overall MotionCtrl speed.
        msg.velocity = [0.0] * 6 + [float(speed_percent)]
        # effort[6] is clipped by the driver to [0.5, 3.0].
        msg.effort = [0.0] * 6 + [float(effort)]
        return msg

    def ensure_no_other_command_publisher(self) -> None:
        publishers = self.get_publishers_info_by_topic("/joint_states")
        if publishers:
            descriptions = ", ".join(
                f"{info.node_namespace}{info.node_name}" for info in publishers
            )
            raise RuntimeError(
                "/joint_states 已有其他发布者，拒绝执行: " + descriptions
            )

    def enable_arm(self) -> None:
        if not self.enable_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/enable_srv 不可用")
        request = Enable.Request()
        request.enable_request = True
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=7.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("使能服务调用超时")
        if not future.result().enable_response:
            raise RuntimeError("驱动报告机械臂使能失败")

    def prepare_command_publisher(self) -> None:
        self.command_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        # Give DDS discovery a short, bounded interval.
        discovery_deadline = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_command(self, msg: JointState, duration_sec: float) -> None:
        if self.command_publisher is None:
            raise RuntimeError("命令发布者尚未准备好")

        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.command_publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def observe_result(self, desired_actual_opening_m: float) -> None:
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.samples:
            print("未收到执行后的夹爪反馈")
            return
        actual = self.samples[-1][1][6]
        error = actual - desired_actual_opening_m
        print(f"执行后夹爪反馈：{actual * 1000.0:.1f} mm")
        print(f"与目标之差：{error * 1000.0:+.1f} mm")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Piper 夹爪保守测试；默认只读预检"
    )
    parser.add_argument(
        "--actual-opening-mm",
        type=float,
        required=True,
        help="期望的实际总开口，初次测试建议 20 mm",
    )
    parser.add_argument("--gripper-multiplier", type=int, default=2)
    parser.add_argument("--speed-percent", type=int, default=10)
    parser.add_argument("--effort", type=float, default=0.5)
    parser.add_argument("--min-tool-height", type=float, default=0.22)
    parser.add_argument("--command-seconds", type=float, default=0.6)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际发布命令；没有此参数时只读预检",
    )
    parser.add_argument(
        "--allow-not-at-target",
        action="store_true",
        help=(
            "仅用于机械臂已由人工移动且关节反馈稳定时，允许"
            "motion_status=1并把当前反馈重新设为保持目标"
        ),
    )
    parser.add_argument(
        "--enable-arm",
        action="store_true",
        help="执行前调用 /enable_srv；该驱动会先给夹爪发送 0 mm 命令",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"实际执行必须填写 {CONFIRM_TOKEN}",
    )
    args = parser.parse_args()

    if not 0.0 <= args.actual_opening_mm <= 110.0:
        parser.error("--actual-opening-mm 必须在 0 到 110 之间")
    if not 1 <= args.gripper_multiplier <= 10:
        parser.error("--gripper-multiplier 必须在 1 到 10 之间")
    if not 1 <= args.speed_percent <= 20:
        parser.error("安全测试将 --speed-percent 限制在 1 到 20")
    if not 0.5 <= args.effort <= 1.0:
        parser.error("安全测试将 --effort 限制在 0.5 到 1.0")
    if not 0.2 <= args.command_seconds <= 1.0:
        parser.error("--command-seconds 必须在 0.2 到 1.0 之间")
    if args.enable_arm and not args.execute:
        parser.error("--enable-arm 只能和 --execute 一起使用")
    if args.execute and args.confirm != CONFIRM_TOKEN:
        parser.error(f"实际执行需要 --confirm {CONFIRM_TOKEN}")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = PiperGripperSafeTest()
    try:
        node.spin_until_ready()
        snapshot = node.validated_snapshot(
            args.min_tool_height,
            allow_not_at_target=args.allow_not_at_target,
        )
        command = node.make_command(
            snapshot,
            args.actual_opening_mm,
            args.gripper_multiplier,
            args.speed_percent,
            args.effort,
        )

        print("===== 夹爪安全预检通过 =====")
        print(
            "当前六轴(rad)："
            + ", ".join(f"{value:.6f}" for value in snapshot[:6])
        )
        print(f"当前实际夹爪开口：{snapshot[6] * 1000.0:.1f} mm")
        print(f"目标实际夹爪开口：{args.actual_opening_mm:.1f} mm")
        print(
            "将发送的虚拟gripper位置："
            f"{command.position[6]:.6f} m "
            f"(驱动倍率 {args.gripper_multiplier})"
        )
        print(
            f"速度字段：{args.speed_percent}%（该驱动把它作为整臂速度上限）"
        )
        print(f"夹爪力度字段：{args.effort:.2f}")
        if args.allow_not_at_target:
            print(
                "恢复模式：允许motion_status=1；发送时会把当前六轴"
                "反馈重新设为保持目标。"
            )

        if not args.execute:
            print("只读预检完成：没有创建 /joint_states 发布者，没有移动机械臂或夹爪。")
            return 0

        node.ensure_no_other_command_publisher()
        print("\n即将实际执行夹爪测试。确保夹爪内没有物体或手指，急停可触及。")
        if args.enable_arm:
            print("警告：本地驱动的 /enable_srv 会先发送夹爪 0 mm 命令。")
        for remaining in range(5, 0, -1):
            print(f"{remaining} 秒后执行；按 Ctrl+C 取消。", flush=True)
            one_second_deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < one_second_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)

        # Refresh the snapshot after the countdown, and abort if the arm moved.
        node.spin_until_ready(timeout_sec=2.0)
        refreshed = node.validated_snapshot(
            args.min_tool_height,
            allow_not_at_target=args.allow_not_at_target,
        )
        max_drift = max(
            abs(refreshed[index] - snapshot[index]) for index in range(6)
        )
        if max_drift > 0.02:
            raise RuntimeError(
                f"倒计时期间机械臂发生移动，最大变化 {max_drift:.4f} rad"
            )
        command = node.make_command(
            refreshed,
            args.actual_opening_mm,
            args.gripper_multiplier,
            args.speed_percent,
            args.effort,
        )

        # Establish DDS matching before enabling. The driver's enable service
        # sends a 0 mm gripper command, so the requested opening should follow
        # it without an additional discovery delay.
        node.prepare_command_publisher()
        if args.enable_arm:
            node.enable_arm()
        node.publish_command(command, args.command_seconds)
        node.observe_result(args.actual_opening_mm / 1000.0)
        print("测试结束；程序没有发送失能、复位、回零或末端位姿命令。")
        return 0
    except KeyboardInterrupt:
        print("已由用户取消。")
        return 130
    except Exception as exc:
        print(f"安全拒绝：{exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
