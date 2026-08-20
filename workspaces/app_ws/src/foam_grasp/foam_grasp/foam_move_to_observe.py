#!/usr/bin/env python3
"""Plan and safely execute a joint-space move to the camera observation pose."""

import argparse
import sys
import time

import rclpy

from foam_grasp.foam_move_to_pregrasp import FoamMoveToPregrasp


CONFIRM_TOKEN = "AUTO_MOVE_TO_OBSERVE"
DEFAULT_OBSERVE_JOINTS = (
    0.142604700,
    0.269998232,
    -0.653208024,
    0.008059128,
    1.078754404,
    0.054111288,
)


class FoamMoveToObserve(FoamMoveToPregrasp):
    def wait_for_robot_inputs(self, timeout_sec=8.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = len(self.joint_samples) >= 10
            if self.execution_backend.requires_piper_status:
                ready = ready and self.arm_status is not None and self.end_pose is not None
            if ready:
                return
        raise RuntimeError("缺少执行后端所需的关节或状态反馈")

    def validate_robot_state(self):
        now = time.monotonic()
        timestamps = [("关节反馈", self.latest_joint_received_at)]
        if self.execution_backend.requires_piper_status:
            timestamps.extend(
                [
                    ("机械臂状态", self.arm_status_received_at),
                    ("末端位姿", self.end_pose_received_at),
                ]
            )
        for label, timestamp in timestamps:
            if now - timestamp > 0.6:
                raise RuntimeError(f"{label}已过期")

        if self.execution_backend.requires_piper_status:
            status = self.arm_status
            if status.arm_status != 0 or status.err_code != 0:
                raise RuntimeError(
                    f"机械臂异常: arm_status={status.arm_status}, "
                    f"err_code={status.err_code}"
                )
            # Piper can keep motion_status=1 after an interrupted or
            # superseded joint target even when the arm has physically
            # stopped.  Accept that recoverable state after the stable check.
            if status.motion_status not in (0, 1):
                raise RuntimeError(
                    f"机械臂运动状态异常: motion_status={status.motion_status}"
                )
            communication_errors = (
                status.communication_status_joint_1,
                status.communication_status_joint_2,
                status.communication_status_joint_3,
                status.communication_status_joint_4,
                status.communication_status_joint_5,
                status.communication_status_joint_6,
            )
            if any(communication_errors):
                raise RuntimeError("机械臂存在关节通信异常")
            angle_limit_flags = (
                status.joint_1_angle_limit,
                status.joint_2_angle_limit,
                status.joint_3_angle_limit,
                status.joint_4_angle_limit,
                status.joint_5_angle_limit,
                status.joint_6_angle_limit,
            )
            if any(angle_limit_flags):
                raise RuntimeError("真机报告至少一个关节触发角度限位")

        samples = list(self.joint_samples)[-10:]
        maximum_spread = max(
            max(sample[1][joint] for sample in samples)
            - min(sample[1][joint] for sample in samples)
            for joint in range(6)
        )
        if maximum_spread > 0.01:
            raise RuntimeError(
                f"机械臂尚未静止，最大关节波动 {maximum_spread:.4f} rad"
            )
        if (
            self.execution_backend.requires_piper_status
            and self.arm_status.motion_status == 1
        ):
            self.get_logger().warning(
                "motion_status=1但关节反馈已稳定；"
                "将从当前真实关节位置重新规划观察姿态"
            )

        positions = self.current_positions()
        self.validate_start_joint_limits(positions[:6])
        return positions


def parse_args():
    parser = argparse.ArgumentParser(description="移动到固定相机观察姿态")
    parser.add_argument(
        "--execution-backend",
        choices=("real", "simulation"),
        default="real",
        help="最终执行通道；默认real，simulation使用ros2_control action",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument(
        "--observe-joints",
        type=float,
        nargs=6,
        default=DEFAULT_OBSERVE_JOINTS,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )
    parser.add_argument("--slowdown", type=float, default=2.0)
    parser.add_argument("--speed-percent", type=int, default=10)
    parser.add_argument("--effort", type=float, default=0.5)
    parser.add_argument("--tracking-limit", type=float, default=0.20)
    parser.add_argument("--countdown-seconds", type=int, default=5)
    args = parser.parse_args()
    if args.execute and args.confirm != CONFIRM_TOKEN:
        parser.error(f"实际执行需要 --confirm {CONFIRM_TOKEN}")
    if not 1.5 <= args.slowdown <= 4.0:
        parser.error("--slowdown必须在1.5到4.0之间")
    if not 1 <= args.speed_percent <= 15:
        parser.error("--speed-percent必须在1到15之间")
    if not 0.5 <= args.effort <= 1.0:
        parser.error("--effort必须在0.5到1.0之间")
    if not 0.10 <= args.tracking_limit <= 0.25:
        parser.error("--tracking-limit必须在0.10到0.25 rad之间")
    if not 0 <= args.countdown_seconds <= 5:
        parser.error("--countdown-seconds必须在0到5之间")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = FoamMoveToObserve(execution_backend=args.execution_backend)
    try:
        node.wait_for_robot_inputs()
        node.wait_for_services()
        # Service discovery does not spin subscription callbacks. Refresh a
        # complete live feedback window before applying the strict age checks.
        node.spin_for(0.8)
        start_positions = node.validate_robot_state()
        goal_positions = [float(value) for value in args.observe_joints]
        node.validate_joint_limits(goal_positions, margin=0.01)

        goal_distance = max(
            abs(actual - goal)
            for actual, goal in zip(start_positions[:6], goal_positions)
        )
        if goal_distance <= 0.015:
            print(
                "已位于观察姿态：最大关节差"
                f"{goal_distance:.4f}rad；无需重复移动。"
            )
            node.spin_for(1.0)
            return 0

        node.apply_table()
        start_state = node.current_robot_state()
        response = node.plan_to_pregrasp(start_state, goal_positions)
        duration, maximum_step, maximum_velocity = node.validate_trajectory(
            response,
            start_positions,
            goal_positions,
            minimum_duration=0.05,
        )
        node.publish_display(start_state, response)

        print("===== 观察姿态轨迹验证通过 =====")
        print(
            "目标关节(rad)："
            + ", ".join(f"{value:.4f}" for value in goal_positions)
        )
        print(
            f"轨迹点数：{len(response.trajectory.joint_trajectory.points)}，"
            f"规划{duration:.3f}s，执行约{duration * args.slowdown:.3f}s"
        )
        print(
            f"最大步长{maximum_step:.4f}rad，"
            f"最大规划速度{maximum_velocity:.4f}rad/s"
        )

        if not args.execute:
            print("PLAN-ONLY：没有创建/joint_states发布者，没有移动机械臂。")
            return 0

        node.ensure_command_path_is_exclusive()
        print("即将自动移动到观察姿态；全程看守急停。")
        for remaining in range(args.countdown_seconds, 0, -1):
            print(f"{remaining} 秒后执行；按Ctrl+C取消。", flush=True)
            node.spin_for(1.0)

        refreshed = node.validate_robot_state()
        drift = max(
            abs(actual - planned)
            for actual, planned in zip(refreshed[:6], start_positions[:6])
        )
        if drift > 0.02:
            raise RuntimeError(f"倒计时期间机械臂移动了 {drift:.4f}rad")

        node.prepare_command_publisher()
        real_duration, final_error = node.execute_trajectory(
            response,
            refreshed,
            refreshed[6],
            args.slowdown,
            args.speed_percent,
            args.effort,
            args.tracking_limit,
        )
        print("===== 已到达观察姿态 =====")
        print(f"实际发送时长：{real_duration:.3f}s")
        print(f"最终最大关节误差：{final_error:.4f}rad")
        print("保持当前夹爪开口；未下降、闭合、复位、回零或失能。")
        return 0
    except KeyboardInterrupt:
        if node.command_publisher is not None and node.latest_joint_state is not None:
            current = node.current_positions()
            node.publish_hold(current[6], args.speed_percent, args.effort)
        print("已取消并尝试保持当前位置；异常时使用硬件急停。")
        return 130
    except Exception as error:
        if node.command_publisher is not None and node.latest_joint_state is not None:
            current = node.current_positions()
            node.publish_hold(current[6], args.speed_percent, args.effort)
        print(f"安全拒绝/中止：{error}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
