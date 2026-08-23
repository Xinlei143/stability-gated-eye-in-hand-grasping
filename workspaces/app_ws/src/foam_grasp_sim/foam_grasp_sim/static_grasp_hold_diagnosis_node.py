#!/usr/bin/env python3
"""Run a no-lift, free-cube static grasp hold and save synchronized evidence."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import ContactsState, ModelStates
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .contact_diagnostics import extract_contact_rows
from .static_grasp_diagnosis import (
    build_diagnostics_rows,
    plot_diagnostics,
    summarize_static_hold,
    write_diagnostics_csv,
)


RAW_JOINT_FIELDS = [
    "sim_time_ns", "stage", "joint7_position_m", "joint8_position_m",
    "joint7_velocity_m_s", "joint8_velocity_m_s", "joint7_effort_N", "joint8_effort_N",
    "commanded_joint7_position_m", "commanded_joint8_position_m",
]
RAW_CONTACT_FIELDS = [
    "sim_time_ns", "stage", "side", "collision1", "collision2", "contact_index",
    "position_x_m", "position_y_m", "position_z_m", "normal_x", "normal_y", "normal_z",
    "depth_m", "force_x_N", "force_y_N", "force_z_N", "torque_x_Nm", "torque_y_Nm",
    "torque_z_Nm", "normal_force_N", "tangential_force_N",
]


def _stamp_ns(message, fallback):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        if value > 0:
            return value
    return int(fallback)


def _duration_message(seconds):
    seconds = float(seconds)
    whole = int(seconds)
    point = JointTrajectoryPoint()
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
    return point


class StaticGraspHoldDiagnosis(Node):
    def __init__(self):
        super().__init__("static_grasp_hold_diagnosis")
        self.declare_parameter("config", "")
        self.declare_parameter("output_dir", "/tmp/static_grasp_hold_diagnosis")
        self.declare_parameter("cube_model_path", "")
        config_path = Path(str(self.get_parameter("config").value))
        if not config_path.is_file():
            raise RuntimeError(f"diagnosis config does not exist: {config_path}")
        with config_path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.target_entity = str(self.config.get("target_entity", "foam_cube"))
        self.raw_joints = []
        self.raw_contacts = []
        self.cube_samples = []
        self.stage = "startup"
        self.commanded = {"joint7": 0.0, "joint8": 0.0}
        self.hold_start_ns = None
        self.hold_end_ns = None
        self._last_joint = {}
        self._last_cube = {}
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 100)
        self.create_subscription(
            ContactsState, "/foam_grasp_sim/link7_contacts",
            lambda message: self._contact_callback(message, "left"), 100,
        )
        self.create_subscription(
            ContactsState, "/foam_grasp_sim/link8_contacts",
            lambda message: self._contact_callback(message, "right"), 100,
        )
        self.create_subscription(ModelStates, "/gazebo/model_states", self._model_callback, 50)
        self.arm_client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.gripper7_client = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.gripper8_client = ActionClient(self, FollowJointTrajectory, "/gripper8_controller/follow_joint_trajectory")
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

    @property
    def arm_joints(self):
        return tuple(f"joint{index}" for index in range(1, 7))

    def _joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        efforts = dict(zip(message.name, message.effort))
        if not all(name in positions for name in ("joint7", "joint8")):
            return
        timestamp = _stamp_ns(message, self.get_clock().now().nanoseconds)
        row = {
            "sim_time_ns": timestamp,
            "stage": self.stage,
            "joint7_position_m": float(positions["joint7"]),
            "joint8_position_m": float(positions["joint8"]),
            "joint7_velocity_m_s": float(velocities.get("joint7", math.nan)),
            "joint8_velocity_m_s": float(velocities.get("joint8", math.nan)),
            "joint7_effort_N": float(efforts.get("joint7", math.nan)),
            "joint8_effort_N": float(efforts.get("joint8", math.nan)),
            "commanded_joint7_position_m": float(self.commanded["joint7"]),
            "commanded_joint8_position_m": float(self.commanded["joint8"]),
        }
        self._last_joint = row
        self.raw_joints.append(row)

    def _contact_callback(self, message, side):
        timestamp = _stamp_ns(message, self.get_clock().now().nanoseconds)
        rows = extract_contact_rows(message, side, self.target_entity)
        if not rows:
            rows = [{
                "collision1": "", "collision2": "", "contact_index": -1,
                "position_x_m": math.nan, "position_y_m": math.nan, "position_z_m": math.nan,
                "normal_x": math.nan, "normal_y": math.nan, "normal_z": math.nan,
                "depth_m": 0.0, "force_x_N": 0.0, "force_y_N": 0.0, "force_z_N": 0.0,
                "torque_x_Nm": 0.0, "torque_y_Nm": 0.0, "torque_z_Nm": 0.0,
                "normal_force_N": 0.0, "tangential_force_N": 0.0,
            }]
        for contact in rows:
            self.raw_contacts.append({
                "sim_time_ns": timestamp,
                "stage": self.stage,
                "side": side,
                **{field: contact.get(field, "") for field in RAW_CONTACT_FIELDS if field not in {"sim_time_ns", "stage", "side"}},
            })

    def _model_callback(self, message):
        try:
            index = list(message.name).index(self.target_entity)
        except ValueError:
            return
        pose = message.pose[index].position
        self._last_cube = {
            "sim_time_ns": int(self.get_clock().now().nanoseconds),
            "x_m": float(pose.x), "y_m": float(pose.y), "z_m": float(pose.z),
        }
        self.cube_samples.append(dict(self._last_cube))

    def _wait_for_server(self, client):
        timeout = float(self.config.get("action_timeout_s", 30.0))
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=0.5):
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("FollowJointTrajectory action server did not become ready")

    def _goal(self, joint_names, positions):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = _duration_message(self.config.get("trajectory_duration_s", 2.0))
        point.positions = [float(value) for value in positions]
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        return goal

    def _send_pair(self, joint7, joint8):
        self.commanded = {"joint7": float(joint7), "joint8": float(joint8)}
        self._wait_for_server(self.gripper7_client)
        self._wait_for_server(self.gripper8_client)
        goals = [
            (self.gripper7_client, self._goal(("joint7",), (joint7,))),
            (self.gripper8_client, self._goal(("joint8",), (joint8,))),
        ]
        requests = [client.send_goal_async(goal) for client, goal in goals]
        handles = []
        for request in requests:
            rclpy.spin_until_future_complete(self, request, timeout_sec=float(self.config.get("action_timeout_s", 30.0)))
            if not request.done() or request.result() is None or not request.result().accepted:
                raise RuntimeError("gripper FollowJointTrajectory goal was rejected")
            handles.append(request.result())
        results = []
        for handle in handles:
            future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=float(self.config.get("action_timeout_s", 30.0)))
            if not future.done() or future.result() is None:
                raise RuntimeError("gripper FollowJointTrajectory result timed out")
            results.append(int(future.result().result.error_code))
        if any(code != 0 for code in results):
            raise RuntimeError(f"gripper trajectory failed: {results}")

    def _send_arm(self):
        self.stage = "arm_to_grasp"
        self._wait_for_server(self.arm_client)
        goal = self._goal(self.arm_joints, self.config["arm_pose"])
        request = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, request, timeout_sec=float(self.config.get("action_timeout_s", 30.0)))
        if not request.done() or request.result() is None or not request.result().accepted:
            raise RuntimeError("arm FollowJointTrajectory goal was rejected")
        future = request.result().get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=float(self.config.get("action_timeout_s", 30.0)))
        if not future.done() or future.result() is None or int(future.result().result.error_code) != 0:
            raise RuntimeError("arm grasp pose trajectory failed")

    def _spawn_free_cube(self):
        model_path = Path(str(self.get_parameter("cube_model_path").value))
        if not model_path.is_file():
            raise RuntimeError(f"free cube model does not exist: {model_path}")
        deadline = time.monotonic() + float(self.config.get("action_timeout_s", 30.0))
        while rclpy.ok() and time.monotonic() < deadline:
            if self.spawn_client.wait_for_service(timeout_sec=0.5):
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            raise RuntimeError("Gazebo SpawnEntity service did not become ready")
        pose = Pose()
        cube_pose = self.config.get("cube_pose", [0.40, 0.0, 0.026])
        pose.position.x, pose.position.y, pose.position.z = (float(value) for value in cube_pose)
        request = SpawnEntity.Request()
        request.name = self.target_entity
        request.xml = model_path.read_text(encoding="utf-8")
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = "world"
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=float(self.config.get("action_timeout_s", 30.0))
        )
        if not future.done() or future.result() is None or not future.result().success:
            status = future.result().status_message if future.done() and future.result() else "timeout"
            raise RuntimeError(f"free cube spawn failed: {status}")
        self.get_logger().info("Spawned free external cube after arm settle")

    def _wait_sim_duration(self, duration_s):
        duration_s = float(duration_s)
        start = int(self.get_clock().now().nanoseconds)
        wall_deadline = time.monotonic() + max(30.0, duration_s * 5.0)
        while rclpy.ok() and time.monotonic() < wall_deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = int(self.get_clock().now().nanoseconds)
            if now - start >= int(duration_s * 1e9):
                return now
        raise RuntimeError(f"simulation time did not advance for {duration_s:g} s")

    def _write_raw_csv(self, path, fields, rows):
        with Path(path).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows({field: row.get(field, "") for field in fields} for row in rows)

    def _write_outputs(self, rows, summary):
        self._write_raw_csv(self.output_dir / "diagnostics_raw_joints.csv", RAW_JOINT_FIELDS, self.raw_joints)
        self._write_raw_csv(self.output_dir / "diagnostics_raw_contacts.csv", RAW_CONTACT_FIELDS, self.raw_contacts)
        write_diagnostics_csv(self.output_dir / "diagnostics.csv", rows)
        summary["output_files"] = {
            "diagnostics": str(self.output_dir / "diagnostics.csv"),
            "raw_contacts": str(self.output_dir / "diagnostics_raw_contacts.csv"),
            "raw_joints": str(self.output_dir / "diagnostics_raw_joints.csv"),
            "plot": str(self.output_dir / "diagnostics.png"),
            "summary": str(self.output_dir / "summary.json"),
        }
        try:
            plot_diagnostics(
                self.output_dir / "diagnostics.png", rows,
                hold_s=summary["hold_s"],
                force_threshold_N=summary["force_threshold_N"],
                diagnosis_case=summary["diagnosis_case"],
            )
        except (ImportError, OSError, RuntimeError) as error:
            summary["plot_error"] = str(error)
        (self.output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    def run(self):
        scale = float(self.config.get("joint_position_m_per_mm", 0.0005))
        self._send_arm()
        self.stage = "arm_settle"
        self._wait_sim_duration(self.config.get("arm_settle_s", 1.0))
        open_half = float(self.config.get("open_mm", 70.0)) * scale
        self.stage = "open_pre_spawn"
        self._send_pair(open_half, -open_half)
        self.stage = "open_settle"
        self._wait_sim_duration(self.config.get("open_settle_s", 0.5))
        self.stage = "cube_spawn"
        self._spawn_free_cube()
        self.stage = "close"
        close_half = float(self.config.get("close_mm", 40.0)) * scale
        self._send_pair(close_half, -close_half)
        self.stage = "hold"
        self.hold_start_ns = int(self.get_clock().now().nanoseconds)
        self.hold_end_ns = self._wait_sim_duration(self.config.get("hold_s", 2.0))
        hold_rows = build_diagnostics_rows(
            self.raw_joints, self.raw_contacts, self.cube_samples,
            start_ns=self.hold_start_ns, end_ns=self.hold_end_ns,
            grid_hz=float(self.config.get("grid_hz", 100.0)),
        )
        summary = summarize_static_hold(
            hold_rows,
            hold_s=float(self.config.get("hold_s", 2.0)),
            force_threshold_N=float(self.config.get("force_threshold_N", 0.8)),
        )
        summary["target_entity"] = self.target_entity
        summary["grasp_pose"] = list(self.config["arm_pose"])
        summary["command_opening_mm"] = float(self.config.get("close_mm", 40.0))
        summary["free_cube_external_object"] = True
        summary["grasp_assist_enabled"] = False
        self._write_outputs(hold_rows, summary)
        self.get_logger().info(json.dumps(summary, sort_keys=True))
        return 0 if summary["static_hold_passed"] else 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = StaticGraspHoldDiagnosis()
        exit_code = node.run()
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
