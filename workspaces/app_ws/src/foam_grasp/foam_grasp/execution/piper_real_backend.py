"""Execution adapter for the local Piper ROS 2 driver."""

import bisect
import time

import rclpy
from sensor_msgs.msg import JointState

from .base_backend import ExecutionBackend, ExecutionResult


def _duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class PiperRealBackend(ExecutionBackend):
    """Piper vendor command transport and conservative feedback execution."""

    name = "real"
    feedback_topic = "/joint_states_single"
    requires_piper_status = True
    is_simulation = False

    def __init__(self, node):
        super().__init__(node)
        self.command_publisher = None

    def ensure_command_path_is_exclusive(self):
        publishers = self.node.get_publishers_info_by_topic("/joint_states")
        if publishers:
            descriptions = ", ".join(
                f"{info.node_namespace}{info.node_name}" for info in publishers
            )
            raise RuntimeError(
                "/joint_states已有发布者，拒绝执行: " + descriptions
            )
        subscribers = self.node.get_subscriptions_info_by_topic("/joint_states")
        if not any(
            info.node_name == "piper_ctrl_single_node" for info in subscribers
        ):
            raise RuntimeError("/joint_states没有Piper驱动订阅者")

    def prepare_execution(self):
        self.ensure_command_path_is_exclusive()
        if self.command_publisher is None:
            self.command_publisher = self.node.create_publisher(
                JointState, "/joint_states", 10
            )
        self.node.spin_for(1.0)
        self._prepared = True

    def publish_message(self, message):
        if self.command_publisher is None:
            raise RuntimeError("真机执行后端尚未准备")
        self.command_publisher.publish(message)

    def make_command(self, arm_positions, actual_gripper_m, speed_percent, effort):
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(self.node.command_names)
        # Piper's real driver interprets joint7 as half of the total opening.
        message.position = list(arm_positions) + [float(actual_gripper_m) / 2.0]
        message.velocity = [0.0] * 6 + [float(speed_percent)]
        message.effort = [0.0] * 6 + [float(effort)]
        return message

    def hold_position(self, actual_gripper_m, speed_percent, effort):
        if not self.can_hold or self.node.latest_joint_state is None:
            return
        current = self.node.current_positions()
        message = self.make_command(
            current[:6], actual_gripper_m, speed_percent, effort
        )
        for _ in range(5):
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.publish_message(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def send_servo_command(
        self, arm_positions, gripper_opening, speed_percent, effort
    ):
        if not self.can_hold:
            raise RuntimeError("真机执行后端尚未准备")
        self.publish_message(
            self.make_command(
                arm_positions, gripper_opening, speed_percent, effort
            )
        )

    @staticmethod
    def _interpolate_trajectory(trajectory, target_time):
        times = [_duration_seconds(p.time_from_start) for p in trajectory.points]
        if target_time <= times[0]:
            return list(trajectory.points[0].positions)
        if target_time >= times[-1]:
            return list(trajectory.points[-1].positions)
        upper = bisect.bisect_right(times, target_time)
        lower = upper - 1
        ratio = (target_time - times[lower]) / (times[upper] - times[lower])
        return [
            float(start + ratio * (end - start))
            for start, end in zip(
                trajectory.points[lower].positions,
                trajectory.points[upper].positions,
            )
        ]

    def _check_status(self, label):
        status = self.node.arm_status
        if status is None or status.arm_status != 0 or status.err_code != 0:
            raise RuntimeError(f"{label}执行期间状态异常")
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
            raise RuntimeError(f"{label}执行期间出现关节通信异常")

    def execute_arm_trajectory(
        self,
        motion_response,
        planned_start,
        actual_gripper_m,
        slowdown,
        speed_percent,
        effort,
        tracking_limit,
        gripper_start_m=None,
        gripper_motion_start_fraction=0.10,
        gripper_motion_end_fraction=0.75,
        **kwargs,
    ):
        del kwargs
        trajectory = motion_response.trajectory.joint_trajectory
        planned_duration = _duration_seconds(trajectory.points[-1].time_from_start)
        nominal_duration = planned_duration * slowdown
        soft_limit = max(0.05, min(0.10, tracking_limit * 0.50))
        max_time = max(nominal_duration * 3.0, nominal_duration + 12.0)
        start_wall = time.monotonic()
        last_wall = start_wall
        planned_time = 0.0
        last_command = list(planned_start[:6])
        maximum_tracking_error = 0.0
        synchronized_gripper = gripper_start_m is not None
        if synchronized_gripper:
            gripper_start_m = float(gripper_start_m)
            if not 0.0 <= gripper_start_m <= 0.110:
                raise RuntimeError(f"同步夹爪起点超出范围: {gripper_start_m:.4f}m")
            if not 0.0 <= gripper_motion_start_fraction < gripper_motion_end_fraction <= 1.0:
                raise RuntimeError("同步夹爪时序参数无效")

        self._begin_execution()
        try:
            while rclpy.ok():
                now = time.monotonic()
                if now - start_wall > max_time:
                    raise RuntimeError(
                        "自适应执行超时，最大跟踪误差"
                        f"{maximum_tracking_error:.4f}rad"
                    )
                if now - self.node.latest_joint_received_at > 0.3:
                    raise RuntimeError("执行期间关节反馈中断")

                actual_before = self.node.current_positions()[:6]
                tracking_before = max(
                    abs(value - target)
                    for value, target in zip(actual_before, last_command)
                )
                maximum_tracking_error = max(maximum_tracking_error, tracking_before)
                if tracking_before > tracking_limit:
                    raise RuntimeError(f"跟踪误差过大: {tracking_before:.4f}rad")
                progress_scale = 1.0
                if tracking_before > soft_limit:
                    progress_scale = max(
                        0.0,
                        (tracking_limit - tracking_before)
                        / (tracking_limit - soft_limit),
                    )
                delta_wall = min(max(now - last_wall, 0.0), 0.10)
                last_wall = now
                planned_time = min(
                    planned_duration,
                    planned_time + delta_wall / slowdown * progress_scale,
                )
                commanded = self._interpolate_trajectory(trajectory, planned_time)

                gripper_command_m = float(actual_gripper_m)
                if synchronized_gripper:
                    fraction = (
                        planned_time / planned_duration
                        if planned_duration > 1e-9
                        else 1.0
                    )
                    ratio = min(
                        1.0,
                        max(
                            0.0,
                            (fraction - gripper_motion_start_fraction)
                            / (gripper_motion_end_fraction - gripper_motion_start_fraction),
                        ),
                    )
                    ratio = ratio * ratio * (3.0 - 2.0 * ratio)
                    gripper_command_m = gripper_start_m + (
                        actual_gripper_m - gripper_start_m
                    ) * ratio

                self.publish_message(
                    self.make_command(
                        commanded, gripper_command_m, speed_percent, effort
                    )
                )
                last_command = commanded
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if time.monotonic() - self.node.latest_joint_received_at > 0.3:
                    raise RuntimeError("执行期间关节反馈中断")
                self._check_status("轨迹")
                actual = self.node.current_positions()[:6]
                tracking_error = max(
                    abs(value - target)
                    for value, target in zip(actual, last_command)
                )
                maximum_tracking_error = max(maximum_tracking_error, tracking_error)
                if tracking_error > tracking_limit:
                    raise RuntimeError(f"跟踪误差过大: {tracking_error:.4f}rad")
                if planned_time >= planned_duration:
                    break

            final_message = self.make_command(
                list(trajectory.points[-1].positions),
                actual_gripper_m,
                speed_percent,
                effort,
            )
            final_deadline = time.monotonic() + 3.0
            while rclpy.ok() and time.monotonic() < final_deadline:
                final_message.header.stamp = self.node.get_clock().now().to_msg()
                self.publish_message(final_message)
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if time.monotonic() - self.node.latest_joint_received_at > 0.3:
                    raise RuntimeError("到达目标时关节反馈中断")
                final_error = max(
                    abs(actual - target)
                    for actual, target in zip(
                        self.node.current_positions()[:6],
                        trajectory.points[-1].positions,
                    )
                )
                maximum_tracking_error = max(maximum_tracking_error, final_error)
                if final_error > tracking_limit:
                    raise RuntimeError(f"到达目标时跟踪误差过大: {final_error:.4f}rad")
                gripper_error = abs(
                    self.node.current_positions()[6] - actual_gripper_m
                )
                if final_error <= 0.02 and (
                    not synchronized_gripper or gripper_error <= 0.004
                ):
                    break
        except Exception:
            hold_gripper = (
                self.node.current_positions()[6]
                if self.node.latest_joint_state is not None
                else actual_gripper_m
            )
            self.hold_position(hold_gripper, speed_percent, effort)
            raise
        finally:
            self._end_execution()

        final_error = max(
            abs(actual - target)
            for actual, target in zip(
                self.node.current_positions()[:6],
                trajectory.points[-1].positions,
            )
        )
        if final_error > 0.05:
            self.hold_position(actual_gripper_m, speed_percent, effort)
            raise RuntimeError(f"到达PREGRASP后的误差过大: {final_error:.4f}rad")
        if synchronized_gripper:
            gripper_error = abs(
                self.node.current_positions()[6] - actual_gripper_m
            )
            if gripper_error > 0.004:
                self.hold_position(
                    self.node.current_positions()[6], speed_percent, effort
                )
                raise RuntimeError(
                    "到达PREGRASP时夹爪未充分张开: "
                    f"error={gripper_error * 1000.0:.1f}mm"
                )
        return ExecutionResult(
            duration_sec=time.monotonic() - start_wall,
            final_error=final_error,
            maximum_tracking_error=maximum_tracking_error,
            gripper_position=self.node.current_positions()[6],
        )

    def execute_cartesian_trajectory(
        self,
        name,
        trajectory,
        gripper_target_m,
        max_joint_rate,
        speed_percent,
        effort,
        tracking_limit,
    ):
        actual_start = self.node.current_positions()[:6]
        first = list(trajectory.points[0].positions)
        start_error = max(abs(a - b) for a, b in zip(actual_start, first))
        if start_error > 0.05:
            raise RuntimeError(f"{name}起点与真机相差 {start_error:.4f}rad")
        previous = actual_start
        total_duration = 0.0
        maximum_tracking_error = 0.0
        self._begin_execution()
        try:
            for point in trajectory.points[1:]:
                target = [float(value) for value in point.positions]
                maximum_delta = max(abs(a - b) for a, b in zip(target, previous))
                segment_duration = max(0.20, maximum_delta / max_joint_rate)
                segment_start = time.monotonic()
                while rclpy.ok():
                    ratio = min(
                        1.0,
                        (time.monotonic() - segment_start) / segment_duration,
                    )
                    commanded = [
                        start + ratio * (end - start)
                        for start, end in zip(previous, target)
                    ]
                    self.publish_message(
                        self.make_command(
                            commanded, gripper_target_m, speed_percent, effort
                        )
                    )
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    if time.monotonic() - self.node.latest_joint_received_at > 0.3:
                        raise RuntimeError(f"{name}执行期间反馈中断")
                    self._check_status(name)
                    actual = self.node.current_positions()[:6]
                    error = max(abs(a - b) for a, b in zip(actual, commanded))
                    maximum_tracking_error = max(maximum_tracking_error, error)
                    if error > tracking_limit:
                        raise RuntimeError(f"{name}跟踪误差过大: {error:.4f}rad")
                    if ratio >= 1.0:
                        break
                total_duration += segment_duration
                previous = target

            final_deadline = time.monotonic() + 1.0
            final_target = list(trajectory.points[-1].positions)
            while rclpy.ok() and time.monotonic() < final_deadline:
                self.publish_message(
                    self.make_command(
                        final_target, gripper_target_m, speed_percent, effort
                    )
                )
                rclpy.spin_once(self.node, timeout_sec=0.05)
            final_error = max(
                abs(a - b)
                for a, b in zip(self.node.current_positions()[:6], final_target)
            )
            maximum_tracking_error = max(maximum_tracking_error, final_error)
            if final_error > 0.05:
                raise RuntimeError(f"{name}最终误差过大: {final_error:.4f}rad")
        except Exception:
            self.hold_position(gripper_target_m, speed_percent, effort)
            raise
        finally:
            self._end_execution()
        return ExecutionResult(
            duration_sec=total_duration,
            final_error=final_error,
            maximum_tracking_error=maximum_tracking_error,
            gripper_position=self.node.current_positions()[6],
        )

    def command_gripper(self, target_actual_m, args):
        arm_hold = self.node.current_positions()[:6]
        deadline = time.monotonic() + 1.0
        self._begin_execution()
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                self.publish_message(
                    self.make_command(
                        arm_hold,
                        target_actual_m,
                        args.speed_percent,
                        args.effort,
                    )
                )
                rclpy.spin_once(self.node, timeout_sec=0.05)
                arm_error = max(
                    abs(a - b)
                    for a, b in zip(self.node.current_positions()[:6], arm_hold)
                )
                if arm_error > 0.05:
                    self.hold_position(target_actual_m, args.speed_percent, args.effort)
                    raise RuntimeError(f"夹爪闭合期间机械臂偏移 {arm_error:.4f}rad")
        finally:
            self._end_execution()
        self.node.spin_for(2.0)
        feedback = self.node.current_positions()[6]
        if len(self.node.latest_joint_state.effort) >= 7:
            print(f"夹爪反馈力度：{float(self.node.latest_joint_state.effort[6]):.3f}")
        print(f"夹爪动作后实际开口：{feedback * 1000.0:.1f} mm")
        return feedback

    def close(self):
        self.command_publisher = None
        self._active = False
        self._prepared = False
