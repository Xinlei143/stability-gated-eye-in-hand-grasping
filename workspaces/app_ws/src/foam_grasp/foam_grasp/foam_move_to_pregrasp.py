#!/usr/bin/env python3
"""Plan and optionally execute only CURRENT -> cube PREGRASP on Piper.

Default operation is plan/validation only.  Execution requires explicit flags
and publishes complete seven-element JointState commands directly to the local
Piper ROS 2 driver.  It never descends to GRASP, closes the gripper, lifts an
object, disables the arm, resets the arm, or sends an end-pose command.
"""

import argparse
import bisect
import copy
import math
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK
from piper_msgs.msg import PiperStatusMsg
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


ARM_JOINTS = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)
COMMAND_NAMES = list(ARM_JOINTS) + ["gripper"]
CONFIRM_TOKEN = "MOVE_TO_PREGRASP_ONLY"

# Conservative physical limits matching the Piper MoveIt configuration.
JOINT_LIMITS = {
    "joint1": (-2.618, 2.618),
    "joint2": (0.0, 3.140),
    "joint3": (-2.967, 0.0),
    "joint4": (-1.745, 1.745),
    "joint5": (-1.220, 1.220),
    "joint6": (-2.0944, 2.0944),
}
START_LIMIT_TOLERANCE = 0.030

TABLE_ID = "foam_work_surface"
TABLE_SIZE = (0.60, 0.70, 0.05)
TABLE_CENTER = (0.40, 0.00, -0.024)


def duration_seconds(duration):
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class FoamMoveToPregrasp(Node):
    def __init__(
        self,
        target_class="cube",
        pregrasp_topic="/foam_grasp/cube_pregrasp_pose",
    ):
        super().__init__("foam_move_to_pregrasp")

        if target_class not in ("cube", "cylinder", "sphere"):
            raise RuntimeError(f"不支持的目标类别: {target_class!r}")
        self.target_class = target_class

        self.latest_joint_state = None
        self.latest_joint_received_at = 0.0
        self.joint_samples = deque(maxlen=30)
        self.arm_status = None
        self.arm_status_received_at = 0.0
        self.end_pose = None
        self.end_pose_received_at = 0.0
        self.pregrasp_pose = None
        self.pregrasp_received_at = 0.0
        self.latched_class = None
        self.latched_class_received_at = 0.0
        self.command_publisher = None

        self.joint_subscription = self.create_subscription(
            JointState,
            "/joint_states_single",
            self.joint_callback,
            20,
        )
        self.status_subscription = self.create_subscription(
            PiperStatusMsg,
            "/arm_status",
            self.status_callback,
            20,
        )
        self.end_pose_subscription = self.create_subscription(
            Pose,
            "/end_pose",
            self.end_pose_callback,
            20,
        )
        self.pregrasp_subscription = self.create_subscription(
            PoseStamped,
            pregrasp_topic,
            self.pregrasp_callback,
            20,
        )
        self.class_subscription = self.create_subscription(
            String,
            "/foam_grasp/latched_target_class",
            self.class_callback,
            20,
        )

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self.plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory,
            "/display_planned_path",
            10,
        )

        self.get_logger().warning(
            "DEFAULT IS PLAN-ONLY; execution is limited to CURRENT->PREGRASP"
        )

    def joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in COMMAND_NAMES):
            return
        values = [float(positions[name]) for name in COMMAND_NAMES]
        if not all(math.isfinite(value) for value in values):
            return
        now = time.monotonic()
        self.latest_joint_state = copy.deepcopy(message)
        self.latest_joint_received_at = now
        self.joint_samples.append((now, values))

    def status_callback(self, message):
        self.arm_status = copy.deepcopy(message)
        self.arm_status_received_at = time.monotonic()

    def end_pose_callback(self, message):
        self.end_pose = copy.deepcopy(message)
        self.end_pose_received_at = time.monotonic()

    def pregrasp_callback(self, message):
        self.pregrasp_pose = copy.deepcopy(message)
        self.pregrasp_received_at = time.monotonic()

    def class_callback(self, message):
        self.latched_class = str(message.data)
        self.latched_class_received_at = time.monotonic()

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_inputs(self, timeout_sec=8.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                len(self.joint_samples) >= 10
                and self.arm_status is not None
                and self.end_pose is not None
                and self.pregrasp_pose is not None
                and self.latched_class is not None
            ):
                return
        raise RuntimeError(
            "缺少关节、机械臂状态、末端位姿、PREGRASP或锁定类别"
        )

    def wait_for_services(self):
        services = (
            ("/compute_ik", self.ik_client),
            ("/apply_planning_scene", self.scene_client),
            ("/plan_kinematic_path", self.plan_client),
        )
        for name, client in services:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"服务不可用: {name}")

    def call_service(self, client, request, timeout_sec):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("MoveIt服务调用超时")
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def current_positions(self):
        positions = dict(
            zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            )
        )
        return [float(positions[name]) for name in COMMAND_NAMES]

    def validate_live_state(
        self,
        require_open_gripper=True,
        allow_not_at_target=False,
    ):
        now = time.monotonic()
        timestamps = (
            ("关节反馈", self.latest_joint_received_at),
            ("机械臂状态", self.arm_status_received_at),
            ("末端位姿", self.end_pose_received_at),
            ("PREGRASP", self.pregrasp_received_at),
            ("锁定类别", self.latched_class_received_at),
        )
        for label, timestamp in timestamps:
            if now - timestamp > 0.6:
                raise RuntimeError(f"{label}已过期")

        status = self.arm_status
        if status.arm_status != 0 or status.err_code != 0:
            raise RuntimeError(
                f"机械臂异常: arm_status={status.arm_status}, "
                f"err_code={status.err_code}"
            )
        if status.motion_status != 0 and not allow_not_at_target:
            raise RuntimeError(
                "机械臂尚未到达上一个指定点: "
                f"motion_status={status.motion_status}"
            )
        communication_errors = [
            status.communication_status_joint_1,
            status.communication_status_joint_2,
            status.communication_status_joint_3,
            status.communication_status_joint_4,
            status.communication_status_joint_5,
            status.communication_status_joint_6,
        ]
        if any(communication_errors):
            raise RuntimeError("机械臂存在关节通信异常")
        angle_limit_flags = [
            status.joint_1_angle_limit,
            status.joint_2_angle_limit,
            status.joint_3_angle_limit,
            status.joint_4_angle_limit,
            status.joint_5_angle_limit,
            status.joint_6_angle_limit,
        ]
        if any(angle_limit_flags):
            raise RuntimeError("真机报告至少一个关节触发角度限位")
        if self.latched_class != self.target_class:
            raise RuntimeError(
                f"锁定类别不是{self.target_class}，"
                f"而是 {self.latched_class!r}"
            )
        if self.pregrasp_pose.header.frame_id != "base_link":
            raise RuntimeError("PREGRASP不在base_link坐标系")

        target = self.pregrasp_pose.pose.position
        if not (
            0.20 <= target.x <= 0.55
            and abs(target.y) <= 0.30
            and 0.20 <= target.z <= 0.35
        ):
            raise RuntimeError(
                "PREGRASP超出保守工作空间: "
                f"({target.x:.3f}, {target.y:.3f}, {target.z:.3f})"
            )
        if self.end_pose.position.z < 0.22:
            raise RuntimeError(
                f"当前末端高度过低: {self.end_pose.position.z:.3f} m"
            )

        samples = list(self.joint_samples)[-10:]
        spreads = []
        for joint_index in range(6):
            column = [sample[1][joint_index] for sample in samples]
            spreads.append(max(column) - min(column))
        if max(spreads) > 0.01:
            raise RuntimeError(
                f"机械臂尚未静止，最大关节波动 {max(spreads):.4f} rad"
            )

        positions = self.current_positions()
        # Real feedback can differ slightly from the conservative MoveIt
        # boundary because of encoder/DH offsets. It is accepted only as the
        # current start, within a small tolerance, and only when the hardware
        # itself reports no angle-limit flag. Goals remain strictly bounded.
        self.validate_start_joint_limits(positions[:6])
        if require_open_gripper and positions[6] < 0.055:
            raise RuntimeError(
                f"夹爪实际开口只有 {positions[6] * 1000:.1f} mm；"
                "移动到PREGRASP前要求至少55 mm"
            )
        return positions

    @staticmethod
    def validate_joint_limits(positions, margin=0.0):
        for name, value in zip(ARM_JOINTS, positions):
            lower, upper = JOINT_LIMITS[name]
            if not lower + margin <= value <= upper - margin:
                raise RuntimeError(
                    f"{name}={value:.4f}超出保守范围"
                )

    @staticmethod
    def validate_start_joint_limits(positions):
        for name, value in zip(ARM_JOINTS, positions):
            lower, upper = JOINT_LIMITS[name]
            if not (
                lower - START_LIMIT_TOLERANCE
                <= value
                <= upper + START_LIMIT_TOLERANCE
            ):
                raise RuntimeError(
                    f"{name}={value:.4f}超出起点容差范围"
                )

    def current_robot_state(self):
        positions = self.current_positions()
        state = RobotState()
        state.is_diff = True
        state.joint_state.header = copy.deepcopy(
            self.latest_joint_state.header
        )
        state.joint_state.name = list(ARM_JOINTS)
        state.joint_state.position = positions[:6]
        return state

    def apply_table(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = TABLE_ID
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(TABLE_SIZE)

        pose = Pose()
        pose.position.x = TABLE_CENTER[0]
        pose.position.y = TABLE_CENTER[1]
        pose.position.z = TABLE_CENTER[2]
        pose.orientation.w = 1.0
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects.append(collision_object)
        response = self.call_service(self.scene_client, request, 5.0)
        if not response.success:
            raise RuntimeError("MoveIt拒绝桌面碰撞模型")

    def compute_pregrasp_ik(self, seed_state):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "link6"
        request.ik_request.pose_stamped = copy.deepcopy(self.pregrasp_pose)
        request.ik_request.robot_state = copy.deepcopy(seed_state)
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2
        response = self.call_service(self.ik_client, request, 4.0)
        if int(response.error_code.val) != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"PREGRASP IK失败: {int(response.error_code.val)}"
            )
        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if not all(name in positions for name in ARM_JOINTS):
            raise RuntimeError("IK结果缺少机械臂关节")
        ordered = [float(positions[name]) for name in ARM_JOINTS]
        self.validate_joint_limits(ordered, margin=0.01)
        return response.solution, ordered

    def goal_constraints(self, goal_positions):
        constraints = Constraints()
        constraints.name = f"{self.target_class}_pregrasp_goal"
        for name, position in zip(ARM_JOINTS, goal_positions):
            item = JointConstraint()
            item.joint_name = name
            item.position = float(position)
            item.tolerance_above = 0.001
            item.tolerance_below = 0.001
            item.weight = 1.0
            constraints.joint_constraints.append(item)
        return constraints

    def plan_to_pregrasp(self, start_state, goal_positions):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.workspace_parameters.header.frame_id = "base_link"
        motion.workspace_parameters.min_corner.x = -0.20
        motion.workspace_parameters.min_corner.y = -0.80
        motion.workspace_parameters.min_corner.z = -0.10
        motion.workspace_parameters.max_corner.x = 0.80
        motion.workspace_parameters.max_corner.y = 0.80
        motion.workspace_parameters.max_corner.z = 1.00
        motion.start_state = copy.deepcopy(start_state)
        motion.start_state.is_diff = True
        motion.goal_constraints.append(self.goal_constraints(goal_positions))
        motion.pipeline_id = "ompl"
        motion.group_name = "arm"
        motion.num_planning_attempts = 10
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.10
        motion.max_acceleration_scaling_factor = 0.10

        response = self.call_service(self.plan_client, request, 8.0)
        motion_response = response.motion_plan_response
        if int(motion_response.error_code.val) != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                "CURRENT_TO_PREGRASP规划失败: "
                f"{int(motion_response.error_code.val)}"
            )
        return motion_response

    def validate_trajectory(self, motion_response, start_positions, goal_positions, minimum_duration=1.0):
        trajectory = motion_response.trajectory.joint_trajectory
        if tuple(trajectory.joint_names) != ARM_JOINTS:
            raise RuntimeError(
                f"轨迹关节顺序异常: {list(trajectory.joint_names)}"
            )
        if len(trajectory.points) < 2:
            raise RuntimeError("轨迹点数少于2")

        times = []
        initial_positions = None
        previous_positions = None
        maximum_step = 0.0
        maximum_velocity = 0.0
        for index, point in enumerate(trajectory.points):
            positions = [float(value) for value in point.positions]
            if len(positions) != 6 or not all(
                math.isfinite(value) for value in positions
            ):
                raise RuntimeError(f"轨迹点{index}的位置无效")
            if index == 0:
                self.validate_start_joint_limits(positions)
                initial_positions = list(positions)
            else:
                # A start point may be just outside, or exactly on, a
                # conservative MoveIt boundary. Such a transition is allowed
                # only while that joint moves monotonically toward the normal
                # [lower+margin, upper-margin] interval.
                for joint_index, (joint_name, value) in enumerate(
                    zip(ARM_JOINTS, positions)
                ):
                    lower, upper = JOINT_LIMITS[joint_name]
                    normal_lower = lower + 0.005
                    normal_upper = upper - 0.005
                    if normal_lower <= value <= normal_upper:
                        continue

                    started_near_lower = (
                        initial_positions[joint_index] < normal_lower
                    )
                    lower_transition = (
                        started_near_lower
                        and lower - START_LIMIT_TOLERANCE <= value < normal_lower
                        and previous_positions is not None
                        and value + 1e-6 >= previous_positions[joint_index]
                    )
                    started_near_upper = (
                        initial_positions[joint_index] > normal_upper
                    )
                    upper_transition = (
                        started_near_upper
                        and normal_upper < value <= upper + START_LIMIT_TOLERANCE
                        and previous_positions is not None
                        and value - 1e-6 <= previous_positions[joint_index]
                    )
                    if not (lower_transition or upper_transition):
                        raise RuntimeError(
                            f"{joint_name}={value:.4f}超出保守范围"
                        )

            current_time = duration_seconds(point.time_from_start)
            if times and current_time <= times[-1]:
                raise RuntimeError("轨迹时间不是严格递增")
            times.append(current_time)

            if previous_positions is not None:
                maximum_step = max(
                    maximum_step,
                    max(
                        abs(current - previous)
                        for current, previous in zip(
                            positions,
                            previous_positions,
                        )
                    ),
                )
            previous_positions = positions
            if point.velocities:
                maximum_velocity = max(
                    maximum_velocity,
                    max(abs(float(value)) for value in point.velocities),
                )

        duration = times[-1]
        if not minimum_duration <= duration <= 15.0:
            raise RuntimeError(f"轨迹时长异常: {duration:.3f}s")
        if maximum_step > 0.15:
            raise RuntimeError(f"相邻轨迹点跳变过大: {maximum_step:.4f}rad")
        if maximum_velocity > 1.0:
            raise RuntimeError(f"规划速度过大: {maximum_velocity:.4f}rad/s")

        first = list(trajectory.points[0].positions)
        last = list(trajectory.points[-1].positions)
        start_error = max(
            abs(actual - planned)
            for actual, planned in zip(start_positions[:6], first)
        )
        goal_error = max(
            abs(goal - planned)
            for goal, planned in zip(goal_positions, last)
        )
        if start_error > 0.03:
            raise RuntimeError(
                f"轨迹起点与真机相差 {start_error:.4f}rad"
            )
        if goal_error > 0.01:
            raise RuntimeError(
                f"轨迹终点与IK相差 {goal_error:.4f}rad"
            )
        return duration, maximum_step, maximum_velocity

    def publish_display(self, start_state, motion_response):
        display = DisplayTrajectory()
        display.model_id = "piper"
        display.trajectory_start = copy.deepcopy(start_state)
        display.trajectory = [copy.deepcopy(motion_response.trajectory)]
        for _ in range(5):
            self.display_publisher.publish(display)
            rclpy.spin_once(self, timeout_sec=0.05)

    def ensure_command_path_is_exclusive(self):
        publishers = self.get_publishers_info_by_topic("/joint_states")
        if publishers:
            descriptions = ", ".join(
                f"{info.node_namespace}{info.node_name}" for info in publishers
            )
            raise RuntimeError(
                "/joint_states已有发布者，拒绝执行: " + descriptions
            )
        subscribers = self.get_subscriptions_info_by_topic("/joint_states")
        driver_found = any(
            info.node_name == "piper_ctrl_single_node" for info in subscribers
        )
        if not driver_found:
            raise RuntimeError("/joint_states没有Piper驱动订阅者")

    def prepare_command_publisher(self):
        self.command_publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )
        self.spin_for(1.0)

    @staticmethod
    def interpolate_trajectory(trajectory, target_time):
        times = [duration_seconds(point.time_from_start) for point in trajectory.points]
        if target_time <= times[0]:
            return list(trajectory.points[0].positions)
        if target_time >= times[-1]:
            return list(trajectory.points[-1].positions)
        upper = bisect.bisect_right(times, target_time)
        lower = upper - 1
        span = times[upper] - times[lower]
        ratio = (target_time - times[lower]) / span
        return [
            float(start + ratio * (end - start))
            for start, end in zip(
                trajectory.points[lower].positions,
                trajectory.points[upper].positions,
            )
        ]

    def make_command(self, arm_positions, actual_gripper_m, speed_percent, effort):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(COMMAND_NAMES)
        # Feedback is actual total opening; command is virtual opening because
        # the launched driver multiplies the seventh position by two.
        message.position = list(arm_positions) + [actual_gripper_m / 2.0]
        message.velocity = [0.0] * 6 + [float(speed_percent)]
        message.effort = [0.0] * 6 + [float(effort)]
        return message

    def publish_hold(self, actual_gripper_m, speed_percent, effort):
        if self.command_publisher is None or self.latest_joint_state is None:
            return
        current = self.current_positions()
        message = self.make_command(
            current[:6],
            actual_gripper_m,
            speed_percent,
            effort,
        )
        for _ in range(5):
            message.header.stamp = self.get_clock().now().to_msg()
            self.command_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def execute_trajectory(
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
    ):
        trajectory = motion_response.trajectory.joint_trajectory
        planned_duration = duration_seconds(
            trajectory.points[-1].time_from_start
        )
        nominal_duration = planned_duration * slowdown
        # Progress used to be tied directly to wall time.  If the Piper lagged
        # behind, the command continued moving away from the measured joints
        # until the hard tracking guard stopped the run.  Advance the plan with
        # feedback instead: run normally below the soft limit, progressively
        # slow down above it, and stop advancing near the hard limit so the
        # real arm can catch up.  The hard limit itself remains unchanged.
        soft_tracking_limit = max(
            0.05,
            min(0.10, tracking_limit * 0.50),
        )
        maximum_execution_time = max(
            nominal_duration * 3.0,
            nominal_duration + 12.0,
        )
        start_wall = time.monotonic()
        last_wall = start_wall
        planned_time = 0.0
        last_command = list(planned_start[:6])
        maximum_tracking_error = 0.0
        adaptive_slowdown_reported = False
        synchronized_gripper = gripper_start_m is not None
        if synchronized_gripper:
            gripper_start_m = float(gripper_start_m)
            if not 0.0 <= gripper_start_m <= 0.110:
                raise RuntimeError(
                    f"同步夹爪起点超出范围: {gripper_start_m:.4f}m"
                )
            if not (
                0.0 <= gripper_motion_start_fraction
                < gripper_motion_end_fraction <= 1.0
            ):
                raise RuntimeError("同步夹爪时序参数无效")

        try:
            while rclpy.ok():
                now = time.monotonic()
                elapsed = now - start_wall
                if elapsed > maximum_execution_time:
                    raise RuntimeError(
                        "自适应执行超时: "
                        f"{elapsed:.1f}s，最大跟踪误差"
                        f"{maximum_tracking_error:.4f}rad"
                    )

                if now - self.latest_joint_received_at > 0.3:
                    raise RuntimeError("执行期间关节反馈中断")
                actual_before = self.current_positions()[:6]
                tracking_before = max(
                    abs(value - target)
                    for value, target in zip(actual_before, last_command)
                )
                maximum_tracking_error = max(
                    maximum_tracking_error,
                    tracking_before,
                )
                if tracking_before > tracking_limit:
                    raise RuntimeError(
                        f"跟踪误差过大: {tracking_before:.4f}rad"
                    )

                delta_wall = min(max(now - last_wall, 0.0), 0.10)
                last_wall = now
                if tracking_before <= soft_tracking_limit:
                    progress_scale = 1.0
                else:
                    progress_scale = max(
                        0.0,
                        (tracking_limit - tracking_before)
                        / (tracking_limit - soft_tracking_limit),
                    )
                    if not adaptive_slowdown_reported:
                        self.get_logger().warning(
                            "真机跟踪滞后，已启用反馈自适应减速："
                            f"error={tracking_before:.4f}rad"
                        )
                        adaptive_slowdown_reported = True

                planned_time = min(
                    planned_duration,
                    planned_time + delta_wall / slowdown * progress_scale,
                )
                commanded = self.interpolate_trajectory(
                    trajectory,
                    planned_time,
                )
                gripper_command_m = actual_gripper_m
                if synchronized_gripper:
                    trajectory_fraction = (
                        planned_time / planned_duration
                        if planned_duration > 1e-9
                        else 1.0
                    )
                    gripper_ratio = min(
                        1.0,
                        max(
                            0.0,
                            (
                                trajectory_fraction
                                - gripper_motion_start_fraction
                            )
                            / (
                                gripper_motion_end_fraction
                                - gripper_motion_start_fraction
                            ),
                        ),
                    )
                    # Smoothstep avoids a sudden gripper target jump while the
                    # arm is accelerating along the first trajectory segment.
                    gripper_ratio = (
                        gripper_ratio
                        * gripper_ratio
                        * (3.0 - 2.0 * gripper_ratio)
                    )
                    gripper_command_m = (
                        gripper_start_m
                        + (actual_gripper_m - gripper_start_m)
                        * gripper_ratio
                    )
                message = self.make_command(
                    commanded,
                    gripper_command_m,
                    speed_percent,
                    effort,
                )
                self.command_publisher.publish(message)
                last_command = commanded
                rclpy.spin_once(self, timeout_sec=0.05)

                if time.monotonic() - self.latest_joint_received_at > 0.3:
                    raise RuntimeError("执行期间关节反馈中断")
                status = self.arm_status
                if status is None or status.arm_status != 0 or status.err_code != 0:
                    raise RuntimeError("执行期间机械臂状态异常")
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
                    raise RuntimeError("执行期间出现关节通信异常")

                actual = self.current_positions()[:6]
                tracking_error = max(
                    abs(value - target)
                    for value, target in zip(actual, last_command)
                )
                maximum_tracking_error = max(
                    maximum_tracking_error,
                    tracking_error,
                )
                if tracking_error > tracking_limit:
                    raise RuntimeError(
                        f"跟踪误差过大: {tracking_error:.4f}rad"
                    )
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
                final_message.header.stamp = self.get_clock().now().to_msg()
                self.command_publisher.publish(final_message)
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.monotonic() - self.latest_joint_received_at > 0.3:
                    raise RuntimeError("到达目标时关节反馈中断")
                final_tracking_error = max(
                    abs(actual - target)
                    for actual, target in zip(
                        self.current_positions()[:6],
                        trajectory.points[-1].positions,
                    )
                )
                maximum_tracking_error = max(
                    maximum_tracking_error,
                    final_tracking_error,
                )
                if final_tracking_error > tracking_limit:
                    raise RuntimeError(
                        f"到达目标时跟踪误差过大: "
                        f"{final_tracking_error:.4f}rad"
                    )
                final_gripper_error = abs(
                    self.current_positions()[6] - actual_gripper_m
                )
                if (
                    final_tracking_error <= 0.02
                    and (
                        not synchronized_gripper
                        or final_gripper_error <= 0.004
                    )
                ):
                    break
        except Exception:
            hold_gripper_m = (
                self.current_positions()[6]
                if self.latest_joint_state is not None
                else actual_gripper_m
            )
            self.publish_hold(hold_gripper_m, speed_percent, effort)
            raise

        actual_final = self.current_positions()[:6]
        planned_final = list(trajectory.points[-1].positions)
        final_error = max(
            abs(actual - target)
            for actual, target in zip(actual_final, planned_final)
        )
        if final_error > 0.05:
            self.publish_hold(actual_gripper_m, speed_percent, effort)
            raise RuntimeError(
                f"到达PREGRASP后的误差过大: {final_error:.4f}rad"
            )
        if synchronized_gripper:
            final_gripper_error = abs(
                self.current_positions()[6] - actual_gripper_m
            )
            if final_gripper_error > 0.004:
                self.publish_hold(
                    self.current_positions()[6],
                    speed_percent,
                    effort,
                )
                raise RuntimeError(
                    "到达PREGRASP时夹爪未充分张开: "
                    f"error={final_gripper_error * 1000.0:.1f}mm"
                )
        actual_duration = time.monotonic() - start_wall
        if adaptive_slowdown_reported:
            self.get_logger().info(
                "反馈自适应执行完成："
                f"actual={actual_duration:.3f}s, "
                f"max_tracking_error={maximum_tracking_error:.4f}rad"
            )
        return actual_duration, final_error


def parse_args():
    parser = argparse.ArgumentParser(
        description="只规划或执行当前位置到方块PREGRASP"
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--slowdown", type=float, default=2.0)
    parser.add_argument("--speed-percent", type=int, default=10)
    parser.add_argument("--effort", type=float, default=0.5)
    parser.add_argument("--tracking-limit", type=float, default=0.20)
    args = parser.parse_args()
    if not 1.25 <= args.slowdown <= 4.0:
        parser.error("--slowdown必须在1.25到4.0之间")
    if not 1 <= args.speed_percent <= 15:
        parser.error("--speed-percent必须在1到15之间")
    if not 0.5 <= args.effort <= 1.0:
        parser.error("--effort必须在0.5到1.0之间")
    if not 0.10 <= args.tracking_limit <= 0.25:
        parser.error("--tracking-limit必须在0.10到0.25 rad之间")
    if args.execute and args.confirm != CONFIRM_TOKEN:
        parser.error(f"实际执行需要 --confirm {CONFIRM_TOKEN}")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = FoamMoveToPregrasp()
    try:
        node.wait_for_inputs()
        node.wait_for_services()
        start_positions = node.validate_live_state(
            require_open_gripper=args.execute
        )
        planned_pose = copy.deepcopy(node.pregrasp_pose)
        node.apply_table()
        start_state = node.current_robot_state()
        _ik_state, goal_positions = node.compute_pregrasp_ik(start_state)
        response = node.plan_to_pregrasp(start_state, goal_positions)
        duration, maximum_step, maximum_velocity = node.validate_trajectory(
            response,
            start_positions,
            goal_positions,
        )
        node.publish_display(start_state, response)

        print("===== PREGRASP轨迹验证通过 =====")
        print(f"轨迹点数：{len(response.trajectory.joint_trajectory.points)}")
        print(f"规划时长：{duration:.3f} s")
        print(f"实际执行预计时长：{duration * args.slowdown:.3f} s")
        print(f"最大相邻点变化：{maximum_step:.4f} rad")
        print(f"最大规划速度：{maximum_velocity:.4f} rad/s")
        print(
            "目标关节(rad)："
            + ", ".join(f"{value:.4f}" for value in goal_positions)
        )

        if not args.execute:
            print("PLAN-ONLY：没有创建/joint_states发布者，没有移动机械臂。")
            return 0

        node.ensure_command_path_is_exclusive()
        print("\n即将只移动到PREGRASP；不会下降、闭合夹爪或抬升。")
        print("确保整个机械臂工作空间无人、急停可立即触及。")
        for remaining in range(5, 0, -1):
            print(f"{remaining} 秒后执行；按Ctrl+C取消。", flush=True)
            node.spin_for(1.0)

        refreshed = node.validate_live_state(require_open_gripper=True)
        drift = max(
            abs(refreshed[index] - start_positions[index])
            for index in range(6)
        )
        if drift > 0.02:
            raise RuntimeError(
                f"倒计时期间机械臂移动了 {drift:.4f}rad"
            )
        target_shift = math.sqrt(
            (node.pregrasp_pose.pose.position.x - planned_pose.pose.position.x) ** 2
            + (node.pregrasp_pose.pose.position.y - planned_pose.pose.position.y) ** 2
            + (node.pregrasp_pose.pose.position.z - planned_pose.pose.position.z) ** 2
        )
        if target_shift > 0.001:
            raise RuntimeError(
                f"倒计时期间PREGRASP变化了 {target_shift:.4f}m"
            )

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
        print("===== PREGRASP执行完成 =====")
        print(f"实际发送时长：{real_duration:.3f} s")
        print(f"最终最大关节误差：{final_error:.4f} rad")
        print("未执行下降、夹爪闭合、抬升、失能、复位或回零。")
        return 0
    except KeyboardInterrupt:
        if node.command_publisher is not None and node.latest_joint_state is not None:
            current = node.current_positions()
            node.publish_hold(current[6], args.speed_percent, args.effort)
        print("已取消；若机械臂行为异常，请使用硬件急停。")
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
