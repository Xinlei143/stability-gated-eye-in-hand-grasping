#!/usr/bin/env python3
"""Validated multi-class foam-object grasp sequence for the Piper driver.

Stages:
  1. current pose -> PREGRASP (OMPL, slowed time-parameterized trajectory)
  2. PREGRASP -> GRASP (collision-checked Cartesian path)
  3. close gripper with low effort
  4. GRASP -> LIFT (collision-checked Cartesian path)

The default is plan-only.  Real execution requires an explicit command-line
token and three operator confirmations at the physical checkpoints.
"""

import argparse
import copy
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.utilities import remove_ros_args
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from foam_grasp.benchmark_events import BenchmarkEventPublisher
from foam_grasp.foam_move_to_pregrasp import (
    ARM_JOINTS,
    FoamMoveToPregrasp,
    JOINT_LIMITS,
    duration_seconds,
)


CONFIRM_TOKEN = "FULL_OBJECT_GRASP"
SUPPORTED_CLASSES = ("cube", "cylinder", "sphere")

# The manually validated near-vertical attitude differs from the mathematically
# exact top-down quaternion by about 5.5 degrees.  Keeping it as one candidate
# helps the arm avoid a wrist singularity without allowing arbitrary sideways
# grasp poses.
RECORDED_TOP_DOWN_QUATERNION = (
    0.03339544465195058,
    0.9988390405029863,
    0.03250000758125928,
    -0.012209215813891974,
)
CLEARANCE_CANDIDATES = (0.055, 0.050, 0.045)
YAW_CANDIDATES_DEGREES = (-30.0, -15.0, 15.0, 30.0)
RADIAL_OUTWARD_TILT_DEGREES = (5.0, 10.0, 15.0, 20.0)
RADIAL_INWARD_TILT_DEGREES = (5.0, 10.0)
TANGENTIAL_TILT_DEGREES = (-10.0, -5.0, 5.0, 10.0)


class FoamCubeGraspSequence(FoamMoveToPregrasp):
    def __init__(
        self,
        target_class="cube",
        cylinder_chord_offset_m=0.018,
        execution_backend="real",
    ):
        # The preview topics keep publishing while a grasp is executed.  Once
        # an adaptive candidate is selected, callbacks must not overwrite the
        # exact pose triplet that passed IK and collision checks.
        self.candidate_locked = False
        super().__init__(
            target_class=target_class,
            pregrasp_topic="/foam_grasp/target_pregrasp_pose",
            execution_backend=execution_backend,
        )
        self.grasp_pose = None
        self.grasp_received_at = 0.0
        self.lift_pose = None
        self.lift_received_at = 0.0
        self.declare_parameter("wait_for_method_ready", False)
        self.declare_parameter("method_ready_topic", "/foam_grasp/method_ready")
        self.declare_parameter("method_ready_timeout", 60.0)
        self.declare_parameter("method", "gated")
        self.declare_parameter("commit_method_service", "/foam_grasp/commit_method_target")
        self.declare_parameter("tracking_commit_timeout", 30.0)
        self.declare_parameter("tracking_replan_threshold", 0.010)
        self.declare_parameter("tracking_commit_tolerance", 0.005)
        self.declare_parameter("tracking_max_updates", 20)
        self.declare_parameter("observation_timeout", 1.0)
        self.declare_parameter("scenario", "static")
        self.declare_parameter("seed", 42)
        self.method_ready = False
        self.method_ready_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("method_ready_topic").value),
            self.method_ready_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.grasp_subscription = self.create_subscription(
            PoseStamped,
            "/foam_grasp/target_grasp_pose",
            self.grasp_callback,
            20,
        )
        self.lift_subscription = self.create_subscription(
            PoseStamped,
            "/foam_grasp/target_lift_pose",
            self.lift_callback,
            20,
        )
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self.clear_latch_client = self.create_client(
            Trigger,
            "/foam_grasp/clear_latched_target",
        )
        self.latch_target_client = self.create_client(
            Trigger,
            f"/foam_grasp/latch_{self.target_class}",
        )
        self.commit_method_client = self.create_client(
            Trigger, str(self.get_parameter("commit_method_service").value)
        )
        self.declare_parameter("grasp_assist_service", "")
        self.grasp_assist_service = str(
            self.get_parameter("grasp_assist_service").value
        ).strip()
        self.grasp_assist_client = (
            self.create_client(Trigger, self.grasp_assist_service)
            if self.grasp_assist_service
            else None
        )
        self.method_name = str(self.get_parameter("method").value)
        if self.method_name not in ("snapshot", "tracking", "gated"):
            raise RuntimeError("method must be snapshot, tracking, or gated")
        self.scenario = str(self.get_parameter("scenario").value)
        self.seed = int(self.get_parameter("seed").value)
        self.event_publisher = BenchmarkEventPublisher(self)
        self.declare_parameter("tool_offset", 0.1358)
        self.tool_offset = float(self.get_parameter("tool_offset").value)
        if not 0.080 <= self.tool_offset <= 0.200:
            raise RuntimeError(
                f"tool_offset={self.tool_offset:.4f}m超出安全配置范围"
            )
        self.cylinder_chord_offset_m = float(cylinder_chord_offset_m)
        if not (
            abs(self.cylinder_chord_offset_m) < 1e-9
            or 0.010 <= self.cylinder_chord_offset_m <= 0.024
        ):
            raise RuntimeError(
                "cylinder_chord_offset_m必须为0或在0.010到0.024m之间"
            )
        self.get_logger().warning(
            "OBJECT SEQUENCE DEFAULTS TO PLAN-ONLY; execution has operator gates"
        )

    def pregrasp_callback(self, message):
        if self.candidate_locked:
            # Keep the immutable validated pose, but continue monitoring
            # that the preview publisher is alive.
            self.pregrasp_received_at = time.monotonic()
            return
        super().pregrasp_callback(message)

    def grasp_callback(self, message):
        if self.candidate_locked:
            self.grasp_received_at = time.monotonic()
            return
        self.grasp_pose = copy.deepcopy(message)
        self.grasp_received_at = time.monotonic()

    def lift_callback(self, message):
        if self.candidate_locked:
            self.lift_received_at = time.monotonic()
            return
        self.lift_pose = copy.deepcopy(message)
        self.lift_received_at = time.monotonic()

    def method_ready_callback(self, message):
        self.method_ready = bool(message.data)

    def wait_for_method_ready(self):
        if not bool(self.get_parameter("wait_for_method_ready").value):
            return
        timeout = float(self.get_parameter("method_ready_timeout").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise RuntimeError("method_ready_timeout must be positive and finite")
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.method_ready:
                print("方法层已READY，开始锁定执行目标。")
                return
        raise RuntimeError(f"方法层在{timeout:.1f}s内未达到READY")

    def wait_for_sequence_inputs(self):
        self.wait_for_inputs(timeout_sec=8.0)
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.grasp_pose is not None and self.lift_pose is not None:
                return
        raise RuntimeError("缺少GRASP或LIFT位姿")

    def wait_for_basic_inputs(self, timeout_sec=8.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = len(self.joint_samples) >= 10
            if self.execution_backend.requires_piper_status:
                ready = ready and self.arm_status is not None and self.end_pose is not None
            if ready:
                return
        raise RuntimeError("缺少执行后端所需的关节或状态反馈")

    def emit_event(self, name, *, details=None):
        payload = self.event_publisher.publish(
            name,
            method=self.method_name,
            scenario=self.scenario,
            seed=self.seed,
            details=details,
        )
        if str(name).upper() in {"TRIAL_FINISHED", "TRIAL_FAILED"}:
            # The runner supervises this one launch process group.  A stable
            # stdout marker gives it a terminal signal without starting a
            # second ROS graph or copying the simulation pipeline.
            print(f"BENCHMARK_TERMINAL_EVENT={payload}", flush=True)
        return payload

    def auto_latch_target(self):
        """Commit the method node's ready target (legacy CLI name retained)."""

        self.candidate_locked = False
        service_name = str(self.get_parameter("commit_method_service").value)
        if self.commit_method_client.wait_for_service(timeout_sec=1.0):
            response = self.call_service(
                self.commit_method_client,
                Trigger.Request(),
                5.0,
            )
            if not response.success:
                raise RuntimeError("提交方法目标失败: " + response.message)
            print("方法目标已提交：" + response.message)
        else:
            # Keep the real-camera workflow compatible while the simulation
            # uses the method-policy commit service above.
            if not self.clear_latch_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"服务不可用: {service_name}")
            latch_service = f"/foam_grasp/latch_{self.target_class}"
            if not self.latch_target_client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"服务不可用: {latch_service}")
            clear_response = self.call_service(
                self.clear_latch_client, Trigger.Request(), 5.0
            )
            if not clear_response.success:
                raise RuntimeError("清除旧锁定目标失败: " + clear_response.message)
            self.spin_for(2.0)
            response = self.call_service(
                self.latch_target_client, Trigger.Request(), 5.0
            )
            if not response.success:
                raise RuntimeError("兼容锁定失败: " + response.message)
            print("兼容锁定成功：" + response.message)
        self.pregrasp_pose = None
        self.pregrasp_received_at = 0.0
        self.grasp_pose = None
        self.grasp_received_at = 0.0
        self.lift_pose = None
        self.lift_received_at = 0.0
        self.spin_for(0.5)

    def prepare_grasp_assist(self):
        """Request the optional contact-confirmed attachment before lifting."""

        if self.grasp_assist_client is None:
            return False
        if not self.grasp_assist_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                f"抓取辅助服务不可用: {self.grasp_assist_service}"
            )
        response = self.call_service(
            self.grasp_assist_client,
            Trigger.Request(),
            5.0,
        )
        if not response.success:
            raise RuntimeError("接触确认辅助夹持失败: " + response.message)
        self.emit_event(
            "GRASP_ASSIST_PREPARED",
            details={"message": response.message},
        )
        return True

    @staticmethod
    def pose_distance(first, second):
        if first is None or second is None:
            return math.inf
        a = first.pose.position
        b = second.pose.position
        return math.sqrt(
            (float(a.x) - float(b.x)) ** 2
            + (float(a.y) - float(b.y)) ** 2
            + (float(a.z) - float(b.z)) ** 2
        )

    def latest_pose_age(self):
        if self.pregrasp_pose is None or self.pregrasp_received_at <= 0.0:
            return math.inf
        return max(0.0, time.monotonic() - self.pregrasp_received_at)

    def follow_tracking_target(self, *, execute=False, args=None):
        """Follow live PREGRASP with bounded receding-horizon replanning."""

        timeout = float(self.get_parameter("tracking_commit_timeout").value)
        replan_threshold = float(self.get_parameter("tracking_replan_threshold").value)
        commit_tolerance = float(self.get_parameter("tracking_commit_tolerance").value)
        max_updates = int(self.get_parameter("tracking_max_updates").value)
        observation_timeout = float(self.get_parameter("observation_timeout").value)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise RuntimeError("tracking_commit_timeout must be positive")
        if not 0.001 <= replan_threshold <= 0.100:
            raise RuntimeError("tracking_replan_threshold must be within 1--100mm")
        if not 0.001 <= commit_tolerance <= replan_threshold:
            raise RuntimeError("tracking_commit_tolerance must be <= replan threshold")
        if not 1 <= max_updates <= 100:
            raise RuntimeError("tracking_max_updates must be within 1--100")
        if not math.isfinite(observation_timeout) or observation_timeout <= 0.0:
            raise RuntimeError("observation_timeout must be positive")

        planned_pose = None
        planned_joints = None
        updates = 0
        deadline = time.monotonic() + timeout
        if execute:
            self.ensure_command_path_is_exclusive()
            self.prepare_command_publisher()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = copy.deepcopy(self.pregrasp_pose)
            age = self.latest_pose_age()
            if pose is None or age > observation_timeout:
                continue
            drift = self.pose_distance(pose, planned_pose)
            if planned_pose is None or drift > replan_threshold:
                self.emit_event("PLAN_STARTED", details={"tracking_update": updates})
                try:
                    state = self.current_robot_state()
                    positions = self.current_positions()
                    ik_state, joints = self.compute_ik_for_pose(
                        "TRACKING_PREGRASP", pose, state, ik_timeout=0.35
                    )
                    plan = self.plan_to_pregrasp(state, joints)
                    plan_duration, maximum_step, maximum_velocity = self.validate_trajectory(
                        plan, positions, joints
                    )
                    if execute:
                        self.execute_trajectory(
                            plan,
                            positions,
                            positions[6],
                            args.slowdown,
                            args.speed_percent,
                            args.effort,
                            args.tracking_limit,
                        )
                    planned_pose = pose
                    planned_joints = list(joints)
                    updates += 1
                    self.emit_event(
                        "PLAN_SUCCEEDED",
                        details={
                            "tracking_update": updates,
                            "plan_duration_s": plan_duration,
                            "maximum_step_rad": maximum_step,
                            "maximum_velocity_rad_s": maximum_velocity,
                        },
                    )
                except Exception as error:
                    self.emit_event(
                        "PLAN_FAILED",
                        details={"tracking_update": updates, "reason": str(error)},
                    )
                    raise
            if planned_pose is None or planned_joints is None:
                continue
            latest_drift = self.pose_distance(self.pregrasp_pose, planned_pose)
            if not execute:
                # Plan-only tracking validates one fresh latest-target plan;
                # it must not pretend that the arm mechanically followed it.
                if (
                    self.latest_pose_age() <= min(0.25, observation_timeout)
                    and latest_drift <= commit_tolerance
                ):
                    self.auto_latch_target()
                    return
                if updates >= max_updates:
                    break
                continue
            current = self.current_positions()
            joint_error = max(
                abs(float(actual) - float(planned))
                for actual, planned in zip(current[:6], planned_joints)
            )
            if (
                self.latest_pose_age() <= min(0.25, observation_timeout)
                and latest_drift <= commit_tolerance
                and joint_error <= 0.030
            ):
                self.auto_latch_target()
                return
            if updates >= max_updates:
                break
        raise RuntimeError(
            "tracking failed to obtain a fresh, low-drift PREGRASP commitment "
            f"within {timeout:.1f}s ({updates} updates)"
        )

    def wait_for_sequence_services(self):
        self.wait_for_services()
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("服务不可用: /compute_cartesian_path")

    def validate_sequence_targets(self):
        poses = (self.pregrasp_pose, self.grasp_pose, self.lift_pose)
        if any(pose.header.frame_id != "base_link" for pose in poses):
            raise RuntimeError("抓取位姿不全在base_link坐标系")
        reference = self.pregrasp_pose.pose
        for name, pose in (("GRASP", self.grasp_pose), ("LIFT", self.lift_pose)):
            if (
                abs(pose.pose.position.x - reference.position.x) > 0.001
                or abs(pose.pose.position.y - reference.position.y) > 0.001
            ):
                raise RuntimeError(f"{name}不是严格垂直路径")
        descent = self.pregrasp_pose.pose.position.z - self.grasp_pose.pose.position.z
        ascent = self.lift_pose.pose.position.z - self.grasp_pose.pose.position.z
        if not 0.045 <= descent <= 0.065:
            raise RuntimeError(f"下降距离异常: {descent:.4f}m")
        if not 0.045 <= ascent <= 0.065:
            raise RuntimeError(f"抬升距离异常: {ascent:.4f}m")
        return descent, ascent

    @staticmethod
    def normalize_quaternion(values):
        norm = math.sqrt(sum(float(value) ** 2 for value in values))
        if norm < 1e-9:
            raise RuntimeError("抓取姿态四元数无效")
        return tuple(float(value) / norm for value in values)

    @staticmethod
    def quaternion_multiply(left, right):
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def axis_angle_quaternion(axis, angle_radians):
        half = 0.5 * angle_radians
        sine = math.sin(half)
        return (
            float(axis[0]) * sine,
            float(axis[1]) * sine,
            float(axis[2]) * sine,
            math.cos(half),
        )

    @staticmethod
    def quaternion_distance(left, right):
        dot = abs(sum(a * b for a, b in zip(left, right)))
        dot = max(-1.0, min(1.0, dot))
        return 2.0 * math.acos(dot)

    @staticmethod
    def set_pose_quaternion(pose, quaternion):
        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])

    def orientation_candidates(self):
        nominal = self.normalize_quaternion(
            (
                self.grasp_pose.pose.orientation.x,
                self.grasp_pose.pose.orientation.y,
                self.grasp_pose.pose.orientation.z,
                self.grasp_pose.pose.orientation.w,
            )
        )
        candidates = [("EXACT_VERTICAL", nominal)]
        candidates.append(
            (
                "RECORDED_NEAR_VERTICAL",
                self.normalize_quaternion(RECORDED_TOP_DOWN_QUATERNION),
            )
        )

        for degrees in YAW_CANDIDATES_DEGREES:
            delta = self.axis_angle_quaternion(
                (0.0, 0.0, 1.0),
                math.radians(degrees),
            )
            candidates.append(
                (
                    f"YAW_{degrees:+.0f}",
                    self.normalize_quaternion(
                        self.quaternion_multiply(delta, nominal)
                    ),
                )
            )

        target_x = float(self.grasp_pose.pose.position.x)
        target_y = float(self.grasp_pose.pose.position.y)
        target_angle = math.atan2(target_y, target_x)
        radial_axis = (
            math.cos(target_angle),
            math.sin(target_angle),
            0.0,
        )
        tangential_axis = (
            -math.sin(target_angle),
            math.cos(target_angle),
            0.0,
        )

        # A negative rotation around the tangential axis tilts tool +Z
        # outward toward the target.  That lets the wrist/link6 stay closer to
        # the base while the TCP still reaches a distant object.
        for degrees in RADIAL_OUTWARD_TILT_DEGREES:
            delta = self.axis_angle_quaternion(
                tangential_axis,
                math.radians(-degrees),
            )
            candidates.append(
                (
                    f"TILT_RADIAL_OUT_{degrees:.0f}",
                    self.normalize_quaternion(
                        self.quaternion_multiply(delta, nominal)
                    ),
                )
            )
        for degrees in RADIAL_INWARD_TILT_DEGREES:
            delta = self.axis_angle_quaternion(
                tangential_axis,
                math.radians(degrees),
            )
            candidates.append(
                (
                    f"TILT_RADIAL_IN_{degrees:.0f}",
                    self.normalize_quaternion(
                        self.quaternion_multiply(delta, nominal)
                    ),
                )
            )
        for degrees in TANGENTIAL_TILT_DEGREES:
            delta = self.axis_angle_quaternion(
                radial_axis,
                math.radians(degrees),
            )
            candidates.append(
                (
                    f"TILT_TANGENTIAL_{degrees:+.0f}",
                    self.normalize_quaternion(
                        self.quaternion_multiply(delta, nominal)
                    ),
                )
            )

        # Remove numerically duplicated orientations while retaining the
        # deterministic preference order above. q and -q represent the same
        # rotation, hence the absolute dot-product comparison.
        unique = []
        for label, quaternion in candidates:
            if any(
                abs(sum(a * b for a, b in zip(quaternion, existing)))
                > 1.0 - 1e-8
                for _existing_label, existing in unique
            ):
                continue
            unique.append((label, quaternion))
        return nominal, unique

    def grasp_pose_candidates(self):
        nominal, orientations = self.orientation_candidates()
        # The preview pose is exact top-down, so its x/y are the corrected
        # desired TCP/contact-center location and its z is contact_z plus the
        # modeled tool offset.  For every tilted orientation, recompute link6
        # from that same TCP target: p_link6 = p_tcp - R*z_tool*tool_offset.
        contact_x = float(self.grasp_pose.pose.position.x)
        contact_y = float(self.grasp_pose.pose.position.y)
        contact_z = (
            float(self.grasp_pose.pose.position.z) - self.tool_offset
        )
        if (
            self.target_class == "cylinder"
            and self.cylinder_chord_offset_m > 1e-6
        ):
            # The physical gripper maximum and cylinder diameter are both
            # approximately 70 mm. A centre-line descent therefore has no
            # tolerance. Offset perpendicular to the jaw-closing direction to
            # grasp a shorter chord: an 18 mm offset presents about 60 mm.
            chord_offsets = (
                -self.cylinder_chord_offset_m,
                self.cylinder_chord_offset_m,
            )
        else:
            chord_offsets = (0.0,)

        candidates = []
        for clearance in CLEARANCE_CANDIDATES:
            for orientation_label, quaternion in orientations:
                jaw_axis = self.rotated_local_y_axis(quaternion)
                planar_norm = math.hypot(jaw_axis[0], jaw_axis[1])
                if planar_norm < 0.5:
                    continue
                # Local gripper Y is the parallel-jaw opening/closing axis in
                # the Piper URDF. Its planar perpendicular defines the chord
                # offset direction for this particular wrist orientation.
                chord_axis = (
                    -jaw_axis[1] / planar_norm,
                    jaw_axis[0] / planar_norm,
                )
                for chord_offset in chord_offsets:
                    pregrasp = copy.deepcopy(self.pregrasp_pose)
                    grasp = copy.deepcopy(self.grasp_pose)
                    lift = copy.deepcopy(self.lift_pose)
                    tool_axis = self.rotated_tool_axis(quaternion)
                    candidate_contact_x = (
                        contact_x + chord_offset * chord_axis[0]
                    )
                    candidate_contact_y = (
                        contact_y + chord_offset * chord_axis[1]
                    )
                    grasp.pose.position.x = (
                        candidate_contact_x
                        - self.tool_offset * tool_axis[0]
                    )
                    grasp.pose.position.y = (
                        candidate_contact_y
                        - self.tool_offset * tool_axis[1]
                    )
                    grasp.pose.position.z = (
                        contact_z - self.tool_offset * tool_axis[2]
                    )
                    pregrasp.pose.position.x = grasp.pose.position.x
                    pregrasp.pose.position.y = grasp.pose.position.y
                    pregrasp.pose.position.z = (
                        grasp.pose.position.z + clearance
                    )
                    lift.pose.position.x = grasp.pose.position.x
                    lift.pose.position.y = grasp.pose.position.y
                    lift.pose.position.z = grasp.pose.position.z + clearance
                    self.set_pose_quaternion(pregrasp, quaternion)
                    self.set_pose_quaternion(grasp, quaternion)
                    self.set_pose_quaternion(lift, quaternion)
                    chord_label = ""
                    if abs(chord_offset) > 1e-6:
                        chord_label = (
                            f"/CHORD_{chord_offset * 1000.0:+.0f}MM"
                        )
                    candidates.append(
                        {
                            "label": orientation_label + chord_label,
                            "clearance": clearance,
                            "orientation": quaternion,
                            "orientation_error": self.quaternion_distance(
                                nominal,
                                quaternion,
                            ),
                            "chord_offset": chord_offset,
                            "pregrasp_pose": pregrasp,
                            "grasp_pose": grasp,
                            "lift_pose": lift,
                        }
                    )
        return candidates

    @staticmethod
    def rotated_tool_axis(quaternion):
        x, y, z, w = quaternion
        # Third column of the quaternion rotation matrix: R * [0, 0, 1].
        axis = (
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        )
        norm = math.sqrt(sum(value * value for value in axis))
        if norm < 1e-9:
            raise RuntimeError("抓取姿态工具轴无效")
        return tuple(value / norm for value in axis)

    @staticmethod
    def rotated_local_y_axis(quaternion):
        x, y, z, w = quaternion
        # Second column of the quaternion rotation matrix: R * [0, 1, 0].
        axis = (
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        )
        norm = math.sqrt(sum(value * value for value in axis))
        if norm < 1e-9:
            raise RuntimeError("夹爪开合轴无效")
        return tuple(value / norm for value in axis)

    @staticmethod
    def normalized_joint_margin(joint_positions):
        margins = []
        for joint_name, value in zip(ARM_JOINTS, joint_positions):
            lower, upper = JOINT_LIMITS[joint_name]
            span = upper - lower
            margins.append(min(value - lower, upper - value) / span)
        return min(margins)

    def candidate_score(
        self,
        candidate,
        start_positions,
        pregrasp_joints,
        grasp_joints,
        lift_joints,
    ):
        start_travel = sum(
            abs(goal - start)
            for goal, start in zip(pregrasp_joints, start_positions[:6])
        )
        grasp_travel = sum(
            abs(goal - start)
            for goal, start in zip(grasp_joints, pregrasp_joints)
        )
        lift_travel = sum(
            abs(goal - start)
            for goal, start in zip(lift_joints, grasp_joints)
        )
        joint_margin = min(
            self.normalized_joint_margin(pregrasp_joints),
            self.normalized_joint_margin(grasp_joints),
            self.normalized_joint_margin(lift_joints),
        )
        clearance_loss = CLEARANCE_CANDIDATES[0] - candidate["clearance"]
        score = (
            1.5 * candidate["orientation_error"]
            + 0.20 * start_travel
            + 0.10 * (grasp_travel + lift_travel)
            + 8.0 * clearance_loss
            # Near a workspace boundary, a few milliradians of real tracking
            # error can make a Cartesian recomputation choose another IK
            # branch. Prefer solutions with materially more joint-limit room,
            # even when that means using a slightly lower safe clearance.
            - 3.00 * joint_margin
        )
        return score, joint_margin

    def select_grasp_candidate(self, start_state, start_positions):
        candidates = self.grasp_pose_candidates()
        ik_feasible = []
        failure_examples = []

        print(
            "开始搜索近似垂直抓取姿态："
            f"{len(candidates)}个候选，"
            f"高度={','.join(str(int(v * 1000)) for v in CLEARANCE_CANDIDATES)}mm"
        )
        for candidate in candidates:
            label = candidate["label"]
            clearance_mm = int(round(candidate["clearance"] * 1000.0))
            try:
                pregrasp_state, pregrasp_joints = self.compute_ik_for_pose(
                    f"{label}/{clearance_mm}mm PREGRASP",
                    candidate["pregrasp_pose"],
                    start_state,
                    ik_timeout=0.35,
                )
                grasp_state, grasp_joints = self.compute_ik_for_pose(
                    f"{label}/{clearance_mm}mm GRASP",
                    candidate["grasp_pose"],
                    pregrasp_state,
                    ik_timeout=0.35,
                )
                lift_state, lift_joints = self.compute_ik_for_pose(
                    f"{label}/{clearance_mm}mm LIFT",
                    candidate["lift_pose"],
                    grasp_state,
                    ik_timeout=0.35,
                )
                score, joint_margin = self.candidate_score(
                    candidate,
                    start_positions,
                    pregrasp_joints,
                    grasp_joints,
                    lift_joints,
                )
                candidate.update(
                    {
                        "pregrasp_state": pregrasp_state,
                        "pregrasp_joints": pregrasp_joints,
                        "grasp_state": grasp_state,
                        "grasp_joints": grasp_joints,
                        "lift_state": lift_state,
                        "lift_joints": lift_joints,
                        "base_score": score,
                        "joint_margin": joint_margin,
                    }
                )
                ik_feasible.append(candidate)
            except RuntimeError as error:
                if len(failure_examples) < 4:
                    failure_examples.append(str(error))

        if not ik_feasible:
            details = "; ".join(failure_examples)
            raise RuntimeError(
                f"全部{len(candidates)}个安全抓取候选均无完整IK解"
                + (f"; 示例: {details}" if details else "")
            )

        ik_feasible.sort(key=lambda item: item["base_score"])
        fully_feasible = []
        path_failure_examples = []

        # Full motion planning is more expensive than IK. Evaluate candidates
        # in score order and retain several complete solutions so a random
        # OMPL result cannot dictate the final choice by itself.
        for candidate in ik_feasible:
            try:
                pregrasp_plan = self.plan_to_pregrasp(
                    start_state,
                    candidate["pregrasp_joints"],
                )
                plan_duration, maximum_step, maximum_velocity = (
                    self.validate_trajectory(
                        pregrasp_plan,
                        start_positions,
                        candidate["pregrasp_joints"],
                    )
                )
                approach = self.compute_cartesian(
                    "PREGRASP_TO_GRASP_CANDIDATE",
                    candidate["pregrasp_state"],
                    candidate["grasp_pose"],
                )
                cartesian_grasp_state = self.trajectory_end_state(approach)
                lift = self.compute_cartesian(
                    "GRASP_TO_LIFT_CANDIDATE",
                    cartesian_grasp_state,
                    candidate["lift_pose"],
                )
                approach_step = self.validate_cartesian_trajectory(
                    "PREGRASP_TO_GRASP_CANDIDATE",
                    approach.solution.joint_trajectory,
                )
                lift_step = self.validate_cartesian_trajectory(
                    "GRASP_TO_LIFT_CANDIDATE",
                    lift.solution.joint_trajectory,
                )
                final_score = (
                    candidate["base_score"]
                    + 0.02 * plan_duration
                    + 0.20 * (approach_step + lift_step)
                )
                candidate.update(
                    {
                        "pregrasp_plan": pregrasp_plan,
                        "plan_duration": plan_duration,
                        "maximum_step": maximum_step,
                        "maximum_velocity": maximum_velocity,
                        "approach": approach,
                        "lift": lift,
                        "approach_step": approach_step,
                        "lift_step": lift_step,
                        "score": final_score,
                    }
                )
                fully_feasible.append(candidate)
                if len(fully_feasible) >= 5:
                    break
            except RuntimeError as error:
                if len(path_failure_examples) < 4:
                    path_failure_examples.append(
                        f"{candidate['label']}: {error}"
                    )

        if not fully_feasible:
            details = "; ".join(path_failure_examples)
            raise RuntimeError(
                f"{len(ik_feasible)}个IK候选中没有完整无碰路径"
                + (f"; 示例: {details}" if details else "")
            )

        ranked_solutions = sorted(
            fully_feasible,
            key=lambda item: item["score"],
        )
        robust_solutions = [
            item for item in ranked_solutions
            if item["joint_margin"] >= 0.040
        ]
        # Prefer a modest joint-limit buffer whenever at least one complete
        # solution provides it. Fall back to the normal score only when the
        # target has no solution with that buffer.
        selection_pool = robust_solutions or ranked_solutions
        selected = min(selection_pool, key=lambda item: item["score"])
        self.pregrasp_pose = copy.deepcopy(selected["pregrasp_pose"])
        self.grasp_pose = copy.deepcopy(selected["grasp_pose"])
        self.lift_pose = copy.deepcopy(selected["lift_pose"])
        selected_at = time.monotonic()
        self.pregrasp_received_at = selected_at
        self.grasp_received_at = selected_at
        self.lift_received_at = selected_at
        self.candidate_locked = True

        quaternion = selected["orientation"]
        print("===== 自动抓取姿态选择完成 =====")
        print(
            f"IK完整可行：{len(ik_feasible)}/{len(candidates)}，"
            f"完整路径可行：{len(fully_feasible)}"
        )
        for rank, solution in enumerate(ranked_solutions, start=1):
            print(
                f"  候选{rank}: {solution['label']}, "
                f"{solution['clearance'] * 1000.0:.0f}mm, "
                f"score={solution['score']:.4f}, "
                f"margin={solution['joint_margin']:.3f}, "
                f"robust={'yes' if solution['joint_margin'] >= 0.040 else 'no'}"
            )
        print(
            f"选中：{selected['label']}，"
            f"接近/抬升{selected['clearance'] * 1000.0:.0f}mm，"
            f"姿态偏差{math.degrees(selected['orientation_error']):.1f}°"
        )
        print(
            "四元数(xyzw)："
            + ", ".join(f"{value:.5f}" for value in quaternion)
        )
        selected_axis = self.rotated_tool_axis(quaternion)
        selected_link6 = selected["grasp_pose"].pose.position
        selected_contact = (
            selected_link6.x + self.tool_offset * selected_axis[0],
            selected_link6.y + self.tool_offset * selected_axis[1],
            selected_link6.z + self.tool_offset * selected_axis[2],
        )
        print(
            "工具偏移补偿后："
            f"link6=({selected_link6.x:.4f}, {selected_link6.y:.4f}, "
            f"{selected_link6.z:.4f})m，"
            f"接触中心=({selected_contact[0]:.4f}, "
            f"{selected_contact[1]:.4f}, {selected_contact[2]:.4f})m"
        )
        if (
            self.target_class == "cylinder"
            and abs(selected["chord_offset"]) > 1e-6
        ):
            print(
                "圆柱偏心弦夹取："
                f"横向偏移{selected['chord_offset'] * 1000.0:+.1f}mm；"
                "避开70mm最大直径中心线"
            )
        print(
            f"评分：{selected['score']:.4f}，"
            f"最小归一化关节余量：{selected['joint_margin']:.3f}"
        )
        return selected

    def compute_ik_for_pose(
        self,
        name,
        pose,
        seed_state,
        ik_timeout=2.0,
    ):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "link6"
        request.ik_request.pose_stamped = copy.deepcopy(pose)
        request.ik_request.robot_state = copy.deepcopy(seed_state)
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(ik_timeout)
        request.ik_request.timeout.nanosec = int(
            (ik_timeout - int(ik_timeout)) * 1e9
        )
        response = self.call_service(
            self.ik_client,
            request,
            max(1.0, ik_timeout + 0.75),
        )
        if int(response.error_code.val) != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"{name} IK失败: {int(response.error_code.val)}"
            )
        positions = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if not all(joint in positions for joint in ARM_JOINTS):
            raise RuntimeError(f"{name} IK缺少关节")
        ordered = [float(positions[joint]) for joint in ARM_JOINTS]
        self.validate_joint_limits(ordered, margin=0.01)
        return response.solution, ordered

    def compute_cartesian(self, name, start_state, target_pose):
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = copy.deepcopy(start_state)
        request.start_state.is_diff = True
        request.group_name = "arm"
        request.link_name = "link6"
        request.waypoints.append(copy.deepcopy(target_pose.pose))
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.01
        request.revolute_jump_threshold = 0.15
        request.avoid_collisions = True
        response = self.call_service(self.cartesian_client, request, 8.0)
        if (
            int(response.error_code.val) != MoveItErrorCodes.SUCCESS
            or float(response.fraction) < 0.999
        ):
            raise RuntimeError(
                f"{name}笛卡尔路径失败: fraction={response.fraction:.3f}, "
                f"code={int(response.error_code.val)}"
            )
        self.validate_cartesian_trajectory(name, response.solution.joint_trajectory)
        return response

    def validate_cartesian_trajectory(self, name, trajectory):
        if tuple(trajectory.joint_names) != ARM_JOINTS:
            raise RuntimeError(f"{name}轨迹关节顺序异常")
        if len(trajectory.points) < 2:
            raise RuntimeError(f"{name}轨迹点不足")
        maximum_step = 0.0
        previous = None
        for index, point in enumerate(trajectory.points):
            positions = [float(value) for value in point.positions]
            if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
                raise RuntimeError(f"{name}轨迹点{index}无效")
            self.validate_joint_limits(positions, margin=0.01)
            if previous is not None:
                maximum_step = max(
                    maximum_step,
                    max(abs(a - b) for a, b in zip(positions, previous)),
                )
            previous = positions
        if maximum_step > 0.10:
            raise RuntimeError(
                f"{name}相邻关节跳变过大: {maximum_step:.4f}rad"
            )
        return maximum_step

    @staticmethod
    def trajectory_end_state(cartesian_response):
        trajectory = cartesian_response.solution.joint_trajectory
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(trajectory.joint_names)
        state.joint_state.position = list(trajectory.points[-1].positions)
        return state

    def publish_all_paths(self, start_state, pregrasp_response, approach, lift):
        display = DisplayTrajectory()
        display.model_id = "piper"
        display.trajectory_start = copy.deepcopy(start_state)
        display.trajectory = [
            copy.deepcopy(pregrasp_response.trajectory),
            copy.deepcopy(approach.solution),
            copy.deepcopy(lift.solution),
        ]
        for _ in range(5):
            self.display_publisher.publish(display)
            rclpy.spin_once(self, timeout_sec=0.05)

    def execute_untimed_cartesian(self, *args, **kwargs):
        return self.execution_backend.execute_cartesian_trajectory(
            *args, **kwargs
        )

    def operator_gate(self, prompt, token, gripper_target_m, args):
        print("\n" + prompt)
        if args.auto:
            print(
                f"AUTO模式：等待{args.auto_pause:.1f}s并执行自动状态检查，"
                f"不请求输入{token}。"
            )
            self.spin_for(args.auto_pause)
            return
        answer = input(f"确认安全后输入 {token}，其他输入将保持当前位置并退出：").strip()
        if answer != token:
            self.publish_hold(
                gripper_target_m,
                args.speed_percent,
                args.effort,
            )
            raise RuntimeError(f"操作者未确认 {token}")

    def validate_checkpoint(
        self,
        name,
        expected_pose,
        expected_joints=None,
    ):
        # Refresh feedback after the operator may have spent time inspecting.
        self.spin_for(0.7)
        now = time.monotonic()
        if now - self.latest_joint_received_at > 0.3:
            raise RuntimeError(f"{name}检查时关节反馈过期")
        if self.execution_backend.requires_piper_status:
            if now - self.arm_status_received_at > 0.3:
                raise RuntimeError(f"{name}检查时状态反馈过期")
            if now - self.end_pose_received_at > 0.3:
                raise RuntimeError(f"{name}检查时末端位姿过期")
            if self.arm_status.arm_status != 0 or self.arm_status.err_code != 0:
                raise RuntimeError(f"{name}检查时机械臂状态异常")
            if self.arm_status.motion_status != 0:
                raise RuntimeError(
                    f"{name}检查时尚未到达指定点: "
                    f"motion_status={self.arm_status.motion_status}"
                )
            communication_errors = (
                self.arm_status.communication_status_joint_1,
                self.arm_status.communication_status_joint_2,
                self.arm_status.communication_status_joint_3,
                self.arm_status.communication_status_joint_4,
                self.arm_status.communication_status_joint_5,
                self.arm_status.communication_status_joint_6,
            )
            if any(communication_errors):
                raise RuntimeError(f"{name}检查时存在关节通信异常")
        samples = list(self.joint_samples)[-10:]
        if len(samples) < 10:
            raise RuntimeError(f"{name}检查时稳定样本不足")
        maximum_spread = max(
            max(sample[1][joint] for sample in samples)
            - min(sample[1][joint] for sample in samples)
            for joint in range(6)
        )
        if maximum_spread > 0.01:
            raise RuntimeError(
                f"{name}检查时机械臂未静止: {maximum_spread:.4f}rad"
            )

        joint_error = None
        if expected_joints is not None:
            actual_joints = self.current_positions()[:6]
            joint_error = max(
                abs(actual_value - expected_value)
                for actual_value, expected_value in zip(
                    actual_joints,
                    expected_joints,
                )
            )
            if joint_error > 0.05:
                raise RuntimeError(
                    f"{name}关节目标误差过大: {joint_error:.4f}rad"
                )

        details = "后端反馈检查通过"
        if self.execution_backend.requires_piper_status:
            actual = self.end_pose.position
            target = expected_pose.pose.position
            position_error = math.sqrt(
                (actual.x - target.x) ** 2
                + (actual.y - target.y) ** 2
                + (actual.z - target.z) ** 2
            )
            if position_error > 0.020:
                raise RuntimeError(
                    f"{name}末端位置误差过大: {position_error:.4f}m; "
                    f"actual=({actual.x:.4f},{actual.y:.4f},{actual.z:.4f}); "
                    f"selected_link6=({target.x:.4f},{target.y:.4f},"
                    f"{target.z:.4f})"
                )
            details = f"link6位置误差{position_error:.4f}m"
        if joint_error is not None:
            details += f"，关节误差{joint_error:.4f}rad"
        print(f"{name}自动检查通过：{details}")

    def command_gripper(self, target_actual_m, args):
        return self.execution_backend.command_gripper(target_actual_m, args)



def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="泡沫目标物体抓取整合流程")
    parser.add_argument(
        "--execution-backend",
        choices=("real", "simulation"),
        default="real",
        help="最终执行通道；默认real，simulation使用ros2_control action",
    )
    parser.add_argument(
        "--target-class",
        choices=SUPPORTED_CLASSES,
        default="cube",
        help="目标类别：cube / cylinder / sphere",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="取消DESCEND/CLOSE/LIFT输入，依靠自动检查连续执行",
    )
    parser.add_argument(
        "--auto-latch",
        action="store_true",
        help=(
            "启动时清除旧目标、采样并自动锁定所选类别；"
            "可用于不执行动作的自动规划预检"
        ),
    )
    parser.add_argument("--auto-pause", type=float, default=1.0)
    parser.add_argument("--confirm", default="")
    parser.add_argument("--slowdown", type=float, default=2.0)
    parser.add_argument("--speed-percent", type=int, default=10)
    parser.add_argument("--effort", type=float, default=0.5)
    parser.add_argument("--tracking-limit", type=float, default=0.20)
    parser.add_argument("--cartesian-joint-rate", type=float, default=0.08)
    parser.add_argument("--countdown-seconds", type=int, default=5)
    parser.add_argument("--preopen-opening-mm", type=float, default=70.0)
    parser.add_argument("--close-opening-mm", type=float, default=40.0)
    parser.add_argument(
        "--post-close-hold-s",
        type=float,
        default=0.0,
        help="夹爪闭合后保持目标开口的时间，默认0秒；qualification可设为1秒",
    )
    parser.add_argument(
        "--minimum-grip-margin-mm",
        type=float,
        default=5.0,
        help=(
            "接触阻挡证据阈值：闭合后的实际开口必须至少比闭合命令大该值；"
            "否则拒绝抬升；该值不等同于物理夹持力确认"
        ),
    )
    parser.add_argument(
        "--cylinder-chord-offset-mm",
        type=float,
        default=18.0,
        help="圆柱偏心弦夹取横向偏移，默认18mm",
    )
    if argv is None:
        argv = sys.argv[1:]
    ros_argv = remove_ros_args([sys.argv[0], *argv])[1:]
    args = parser.parse_args(ros_argv)
    required_token = "AUTO_FULL_OBJECT_GRASP" if args.auto else CONFIRM_TOKEN
    if args.execute and args.confirm != required_token:
        parser.error(f"执行需要 --confirm {required_token}")
    if args.auto and not args.execute:
        parser.error("--auto必须和--execute一起使用")
    if args.auto_latch and args.execute and not args.auto:
        parser.error(
            "--execute下使用--auto-latch时必须同时使用--auto"
        )
    if not 0.5 <= args.auto_pause <= 3.0:
        parser.error("--auto-pause必须在0.5到3.0之间")
    if not 1.25 <= args.slowdown <= 4.0:
        parser.error("--slowdown必须在1.25到4.0之间")
    if not 1 <= args.speed_percent <= 15:
        parser.error("--speed-percent必须在1到15之间")
    if not 0.5 <= args.effort <= 1.0:
        parser.error("--effort必须在0.5到1.0之间")
    if not 0.05 <= args.cartesian_joint_rate <= 0.10:
        parser.error("--cartesian-joint-rate必须在0.05到0.10之间")
    if not 0 <= args.countdown_seconds <= 5:
        parser.error("--countdown-seconds必须在0到5之间")
    if not 60.0 <= args.preopen_opening_mm <= 110.0:
        parser.error("--preopen-opening-mm必须在60到110之间")
    if not 30.0 <= args.close_opening_mm <= 60.0:
        parser.error("--close-opening-mm必须在30到60之间")
    if not 0.0 <= args.post_close_hold_s <= 30.0:
        parser.error("--post-close-hold-s必须在0到30秒之间")
    if not 3.0 <= args.minimum_grip_margin_mm <= 15.0:
        parser.error("--minimum-grip-margin-mm必须在3到15之间")
    if args.close_opening_mm >= args.preopen_opening_mm - 5.0:
        parser.error("闭合目标必须比预张开目标至少小5mm")
    if not (
        abs(args.cylinder_chord_offset_mm) < 1e-9
        or 10.0 <= args.cylinder_chord_offset_mm <= 24.0
    ):
        parser.error("--cylinder-chord-offset-mm必须为0或在10到24之间")
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=argv)
    node = FoamCubeGraspSequence(
        args.target_class,
        args.cylinder_chord_offset_mm / 1000.0,
        args.execution_backend,
    )
    terminal_emitted = False
    try:
        if args.auto_latch:
            node.wait_for_basic_inputs()
            node.wait_for_method_ready()
            if node.method_name == "tracking":
                # Tracking needs MoveIt clients before its first receding-
                # horizon PREGRASP plan.  The final commit is made only after
                # the latest preview is fresh and mechanically reached.
                node.wait_for_sequence_inputs()
                node.wait_for_sequence_services()
                node.spin_for(0.8)
                node.apply_table()
                node.follow_tracking_target(execute=args.execute, args=args)
            else:
                node.auto_latch_target()
        node.wait_for_sequence_inputs()
        node.wait_for_sequence_services()
        # Service discovery/planning setup can briefly leave the most recent
        # feedback older than the strict live-state threshold. Refresh a full
        # stable window immediately before validating the start state.
        node.spin_for(0.8)
        node.validate_sequence_targets()
        # The sequence actively opens the empty gripper at the safe starting
        # pose, so it does not require the gripper to be pre-opened manually.
        start_positions = node.validate_live_state(
            require_open_gripper=False,
            allow_not_at_target=True,
        )
        node.apply_table()
        start_state = node.current_robot_state()
        node.emit_event("PLAN_STARTED", details={"phase": "final_candidate"})
        try:
            selected = node.select_grasp_candidate(start_state, start_positions)
        except Exception as error:
            node.emit_event("PLAN_FAILED", details={"phase": "final_candidate", "reason": str(error)})
            raise
        node.emit_event("PLAN_SUCCEEDED", details={"phase": "final_candidate"})
        # Immutable execution snapshot: every real phase below must use the
        # exact poses that passed candidate IK and path validation, never a
        # later message from the continuously published preview topics.
        active_pregrasp_pose = copy.deepcopy(selected["pregrasp_pose"])
        active_grasp_pose = copy.deepcopy(selected["grasp_pose"])
        active_lift_pose = copy.deepcopy(selected["lift_pose"])
        active_pregrasp_joints = list(selected["pregrasp_joints"])
        pregrasp_plan = selected["pregrasp_plan"]
        plan_duration = selected["plan_duration"]
        maximum_step = selected["maximum_step"]
        maximum_velocity = selected["maximum_velocity"]
        approach = selected["approach"]
        lift = selected["lift"]
        approach_step = selected["approach_step"]
        lift_step = selected["lift_step"]
        descent = selected["clearance"]
        ascent = selected["clearance"]
        node.publish_all_paths(start_state, pregrasp_plan, approach, lift)

        print(f"===== {args.target_class}抓取全流程验证通过 =====")
        print(
            f"CURRENT->PREGRASP：{len(pregrasp_plan.trajectory.joint_trajectory.points)}点，"
            f"规划{plan_duration:.3f}s，执行约{plan_duration * args.slowdown:.3f}s"
        )
        print(
            f"垂直下降：{descent:.3f}m，最大关节步长{approach_step:.4f}rad"
        )
        print(
            f"垂直抬升：{ascent:.3f}m，最大关节步长{lift_step:.4f}rad"
        )
        print(
            f"夹爪：当前{start_positions[6] * 1000:.1f}mm -> "
            f"预张开{args.preopen_opening_mm:.1f}mm -> "
            f"闭合目标{args.close_opening_mm:.1f}mm"
        )
        print(
            f"PREGRASP规划最大步长{maximum_step:.4f}rad，"
            f"最大速度{maximum_velocity:.4f}rad/s"
        )

        if not args.execute:
            print("PLAN-ONLY：没有创建/joint_states发布者，没有移动或抓取。")
            node.emit_event(
                "TRIAL_FINISHED",
                details={"execution_mode": "plan_only", "task_success": False},
            )
            terminal_emitted = True
            node.spin_for(0.2)
            return 0

        node.ensure_command_path_is_exclusive()
        print("\n即将开始整合流程；全程看守急停。")
        for remaining in range(args.countdown_seconds, 0, -1):
            print(f"{remaining} 秒后移动到PREGRASP；按Ctrl+C取消。", flush=True)
            node.spin_for(1.0)
        refreshed = node.validate_live_state(
            require_open_gripper=False,
            allow_not_at_target=True,
        )
        drift = max(abs(a - b) for a, b in zip(refreshed[:6], start_positions[:6]))
        if drift > 0.02:
            raise RuntimeError(f"倒计时期间机械臂移动了{drift:.4f}rad")

        node.prepare_command_publisher()
        preopen_target_m = args.preopen_opening_mm / 1000.0
        print(
            "先在当前观察高位完全张开夹爪："
            f"{refreshed[6] * 1000.0:.1f}mm -> "
            f"{args.preopen_opening_mm:.1f}mm。"
        )
        preopen_feedback = node.command_gripper(preopen_target_m, args)
        if preopen_feedback < preopen_target_m - 0.004:
            raise RuntimeError(
                "夹爪没有充分张开: "
                f"目标{args.preopen_opening_mm:.1f}mm，"
                f"反馈{preopen_feedback * 1000.0:.1f}mm"
            )
        after_open = node.current_positions()
        open_arm_drift = max(
            abs(actual - previous)
            for actual, previous in zip(after_open[:6], refreshed[:6])
        )
        if open_arm_drift > 0.020:
            raise RuntimeError(
                "高位张开夹爪期间机械臂发生偏移: "
                f"{open_arm_drift:.4f}rad"
            )
        print(
            f"高位张开完成：实际开口{preopen_feedback * 1000.0:.1f} mm；"
            "现在保持最大开口移动到PREGRASP。"
        )
        node.execute_trajectory(
            pregrasp_plan,
            after_open,
            preopen_target_m,
            args.slowdown,
            args.speed_percent,
            args.effort,
            args.tracking_limit,
        )
        preopen_feedback = node.current_positions()[6]
        print(f"到达高位时夹爪开口：{preopen_feedback * 1000.0:.1f} mm")
        if preopen_feedback < preopen_target_m - 0.004:
            raise RuntimeError(
                "夹爪没有充分张开: "
                f"目标{args.preopen_opening_mm:.1f}mm，"
                f"反馈{preopen_feedback * 1000.0:.1f}mm"
            )
        print("已到达PREGRASP高位。")
        node.operator_gate(
            f"确认夹爪位于{args.target_class}正上方且有安全间隙。",
            "DESCEND",
            preopen_target_m,
            args,
        )
        node.validate_checkpoint(
            "PREGRASP",
            active_pregrasp_pose,
            active_pregrasp_joints,
        )

        # Recompute a paired approach+lift from the same actual PREGRASP
        # feedback before descending.  This prevents the approach from taking
        # one numerical IK branch while the retained lift belongs to another.
        actual_pregrasp_state = node.current_robot_state()
        try:
            approach_to_execute = node.compute_cartesian(
                "PREGRASP_TO_GRASP_REAL",
                actual_pregrasp_state,
                active_grasp_pose,
            )
            execution_grasp_state = node.trajectory_end_state(
                approach_to_execute
            )
            paired_lift = node.compute_cartesian(
                "GRASP_TO_LIFT_PAIRED_BEFORE_DESCENT",
                execution_grasp_state,
                active_lift_pose,
            )
            print("已从真实PREGRASP验证配对的下降和抬升路径。")
        except RuntimeError as paired_replan_error:
            # The original pair was checked before motion. Reuse it only when
            # the measured PREGRASP joints are already close to its start.
            approach_to_execute = selected["approach"]
            paired_lift = selected["lift"]
            approach_trajectory = (
                approach_to_execute.solution.joint_trajectory
            )
            node.validate_cartesian_trajectory(
                "PREVALIDATED_APPROACH_PAIR",
                approach_trajectory,
            )
            node.validate_cartesian_trajectory(
                "PREVALIDATED_LIFT_PAIR",
                paired_lift.solution.joint_trajectory,
            )
            actual_arm = node.current_positions()[:6]
            validated_approach_start = [
                float(value)
                for value in approach_trajectory.points[0].positions
            ]
            pair_start_error = max(
                abs(actual - planned)
                for actual, planned in zip(
                    actual_arm,
                    validated_approach_start,
                )
            )
            if pair_start_error > 0.020:
                raise RuntimeError(
                    "无法从真实PREGRASP获得配对下降/抬升路径，"
                    f"且当前关节与预验证起点相差"
                    f"{pair_start_error:.4f}rad。原因: {paired_replan_error}"
                ) from paired_replan_error
            print(
                "实时配对路径重算未完成；"
                f"起点误差仅{pair_start_error:.4f}rad，"
                "将使用原预验证下降/抬升路径对。"
            )
        grasp_checkpoint_joints = list(
            approach_to_execute.solution.joint_trajectory.points[-1].positions
        )
        node.emit_event("GRASP_STARTED", details={"target_class": args.target_class})
        node.execute_untimed_cartesian(
            "PREGRASP_TO_GRASP",
            approach_to_execute.solution.joint_trajectory,
            preopen_target_m,
            args.cartesian_joint_rate,
            args.speed_percent,
            args.effort,
            args.tracking_limit,
        )
        node.operator_gate(
            f"已到GRASP高度；确认两侧夹爪包围{args.target_class}且没有顶压桌面。",
            "CLOSE",
            preopen_target_m,
            args,
        )
        node.validate_checkpoint(
            "GRASP",
            active_grasp_pose,
            grasp_checkpoint_joints,
        )

        close_target_m = args.close_opening_mm / 1000.0
        close_feedback = node.command_gripper(close_target_m, args)
        node.emit_event(
            "GRIPPER_CLOSED",
            details={"opening_m": close_feedback, "target_m": close_target_m},
        )
        jaw_blocked_margin_mm = (close_feedback - close_target_m) * 1000.0
        node.emit_event(
            "JAW_BLOCKED",
            details={
                "opening_m": close_feedback,
                "target_m": close_target_m,
                "jaw_blocked_margin_mm": jaw_blocked_margin_mm,
                "physical_grip_confirmed": False,
            },
        )
        print(
            "接触阻挡证据："
            f"实际开口比闭合命令大{jaw_blocked_margin_mm:.1f} mm，"
            f"要求至少{args.minimum_grip_margin_mm:.1f} mm；"
            "尚未由接触力确认物理夹持"
        )
        if jaw_blocked_margin_mm < args.minimum_grip_margin_mm:
            raise RuntimeError(
                f"未获得{args.target_class}的接触阻挡证据：余量仅"
                f"{jaw_blocked_margin_mm:.1f}mm；拒绝抬升"
            )
        if args.post_close_hold_s > 0.0:
            node.emit_event(
                "GRIPPER_SETTLE_STARTED",
                details={"duration_s": args.post_close_hold_s},
            )
            node.spin_for(args.post_close_hold_s)
            node.emit_event(
                "GRIPPER_SETTLE_FINISHED",
                details={"duration_s": args.post_close_hold_s},
            )
        node.prepare_grasp_assist()
        node.operator_gate(
            f"确认{args.target_class}已被夹住，允许垂直抬升。",
            "LIFT",
            close_target_m,
            args,
        )
        node.validate_checkpoint(
            "GRASP_AFTER_CLOSE",
            active_grasp_pose,
            grasp_checkpoint_joints,
        )

        actual_grasp_state = node.current_robot_state()
        try:
            lift_to_execute = node.compute_cartesian(
                "GRASP_TO_LIFT_REAL",
                actual_grasp_state,
                active_lift_pose,
            )
            print("闭合后实时抬升路径重算成功。")
        except RuntimeError as replan_error:
            # The entire lift was collision-checked before any real motion.
            # Cartesian IK can nevertheless be numerically sensitive near a
            # workspace boundary: a tiny tracking offset may make a fresh
            # call follow a different local branch. Reuse the prevalidated
            # lift only when its first arm state is already very close to the
            # measured arm state. execute_untimed_cartesian performs another
            # independent start check and continuous tracking checks.
            validated_lift = paired_lift
            trajectory = validated_lift.solution.joint_trajectory
            if not trajectory.points:
                raise RuntimeError(
                    f"实时抬升失败且预验证轨迹为空: {replan_error}"
                ) from replan_error
            node.validate_cartesian_trajectory(
                "GRASP_TO_LIFT_PREVALIDATED_FALLBACK",
                trajectory,
            )
            actual_arm = node.current_positions()[:6]
            validated_start = [
                float(value) for value in trajectory.points[0].positions
            ]
            fallback_start_error = max(
                abs(actual - planned)
                for actual, planned in zip(actual_arm, validated_start)
            )
            if fallback_start_error > 0.020:
                raise RuntimeError(
                    "实时抬升路径失败，且当前关节与预验证"
                    f"抬升起点相差{fallback_start_error:.4f}rad；"
                    f"拒绝复用轨迹。原因: {replan_error}"
                ) from replan_error
            print(
                "实时抬升数值重算未完成；"
                "当前关节与预验证抬升起点仅相差"
                f"{fallback_start_error:.4f}rad，将使用已通过"
                "100% 笛卡尔和碰撞检查的抬升轨迹。"
            )
            lift_to_execute = validated_lift
        node.emit_event("LIFT_STARTED", details={"target_class": args.target_class})
        node.execute_untimed_cartesian(
            "GRASP_TO_LIFT",
            lift_to_execute.solution.joint_trajectory,
            close_target_m,
            args.cartesian_joint_rate,
            args.speed_percent,
            args.effort,
            args.tracking_limit,
        )
        node.emit_event("EXECUTION_FINISHED", details={"target_class": args.target_class})
        node.emit_event(
            "TRIAL_FINISHED",
            details={"execution_mode": "execute"},
        )
        terminal_emitted = True
        node.spin_for(0.2)
        print(f"===== {args.target_class}抓取并抬升完成 =====")
        print("机械臂保持LIFT姿态和夹爪闭合目标；没有失能、复位或回零。")
        return 0
    except KeyboardInterrupt:
        if terminal_emitted:
            return 0
        if node.execution_backend.can_hold and node.latest_joint_state is not None:
            current = node.current_positions()
            node.publish_hold(current[6], args.speed_percent, args.effort)
        print("已取消并尝试保持当前位置；异常时使用硬件急停。")
        node.emit_event(
            "TRIAL_FAILED",
            details={"reason": "keyboard_interrupt", "execution_mode": "execute"},
        )
        node.spin_for(0.2)
        return 130
    except Exception as error:
        if node.execution_backend.can_hold and node.latest_joint_state is not None:
            current = node.current_positions()
            node.publish_hold(current[6], args.speed_percent, args.effort)
        print(f"安全拒绝/中止：{error}", file=sys.stderr)
        node.emit_event(
            "TRIAL_FAILED",
            details={"reason": str(error), "error_type": type(error).__name__},
        )
        node.spin_for(0.2)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
