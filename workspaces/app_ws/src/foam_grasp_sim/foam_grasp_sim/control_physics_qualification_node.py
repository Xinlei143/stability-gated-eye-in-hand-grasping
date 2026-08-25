#!/usr/bin/env python3
"""Run arm-only or gripper-only Gazebo control qualification cycles."""

import csv
import json
import math
import os
import time
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ContactsState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from foam_grasp_sim.control_qualification import (
    summarize_arm_run,
    summarize_loaded_gripper_run,
    summarize_gripper_run,
)
from foam_grasp_sim.contact_diagnostics import extract_contact_rows
from foam_grasp.gripper_trace import make_trace_row


ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _duration_message(seconds):
    seconds = float(seconds)
    whole = int(seconds)
    point = JointTrajectoryPoint()
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int((seconds - whole) * 1e9)
    return point


class ControlPhysicsQualification(Node):
    def __init__(self):
        super().__init__("control_physics_qualification")
        self.declare_parameter("mode", "arm")
        self.declare_parameter("config", "")
        self.declare_parameter("output_dir", "/tmp/foam_grasp_control_qualification")
        self.mode = str(self.get_parameter("mode").value)
        if self.mode not in ("arm", "gripper", "loaded_gripper"):
            raise RuntimeError("mode must be arm, gripper, or loaded_gripper")
        config_path = Path(str(self.get_parameter("config").value))
        if not config_path.is_file():
            raise RuntimeError(f"qualification config does not exist: {config_path}")
        with config_path.open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.joints = {}
        self.samples = []
        self._samples_path_initialized = False
        self.contact_forces = {"left": 0.0, "right": 0.0}
        self._gripper_trace_path = os.environ.get("FOAM_GRIPPER_TRACE_PATH", "")
        self._gripper_trace_duration_s = self._env_float(
            "FOAM_GRIPPER_TRACE_DURATION_S", 1.0
        )
        self._gripper_trace_interval_s = self._env_float(
            "FOAM_GRIPPER_TRACE_INTERVAL_S", 0.05
        )
        self._gripper_trace_pair_index = 0
        self._gripper_trace_started_s = None
        self._gripper_trace_last_sample_s = None
        self._gripper_trace_commands = (None, None)
        self._gripper_trace_rows = []
        self._gripper_feedback = {
            "joint7": {"position": None, "stamp_s": None},
            "joint8": {"position": None, "stamp_s": None},
        }
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/gripper_controller/controller_state",
            lambda message: self._gripper_feedback_callback(message, "joint7"),
            10,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/gripper8_controller/controller_state",
            lambda message: self._gripper_feedback_callback(message, "joint8"),
            10,
        )
        self._gripper_trace_timer = self.create_timer(
            self._gripper_trace_interval_s, self._sample_gripper_trace
        )
        if self.mode == "loaded_gripper":
            self.create_subscription(
                ContactsState,
                "/foam_grasp_sim/link7_contacts",
                lambda message: self._contact_callback(message, "left"),
                50,
            )
            self.create_subscription(
                ContactsState,
                "/foam_grasp_sim/link8_contacts",
                lambda message: self._contact_callback(message, "right"),
                50,
            )
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper7_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )
        self.gripper8_client = ActionClient(
            self, FollowJointTrajectory, "/gripper8_controller/follow_joint_trajectory"
        )

    @staticmethod
    def _env_float(name, default):
        try:
            value = float(os.environ.get(name, default))
        except (TypeError, ValueError):
            return float(default)
        return value if value > 0.0 else float(default)

    @staticmethod
    def _message_stamp_s(message):
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return None
        try:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except (AttributeError, TypeError, ValueError):
            return None

    def _gripper_feedback_callback(self, message, joint_name):
        try:
            index = message.joint_names.index(joint_name)
        except ValueError:
            return
        if index >= len(message.actual.positions):
            return
        self._gripper_feedback[joint_name] = {
            "position": float(message.actual.positions[index]) * 2.0,
            "stamp_s": self._message_stamp_s(message),
        }

    def _start_gripper_trace(self, joint7, joint8):
        if not self._gripper_trace_path:
            return
        self._gripper_trace_pair_index += 1
        self._gripper_trace_started_s = time.monotonic()
        self._gripper_trace_last_sample_s = None
        self._gripper_trace_commands = (float(joint7), float(joint8))
        self._gripper_trace_rows = []
        self._sample_gripper_trace(force=True)

    def _sample_gripper_trace(self, *, force=False):
        if self._gripper_trace_started_s is None:
            return
        elapsed = time.monotonic() - self._gripper_trace_started_s
        if elapsed > self._gripper_trace_duration_s:
            return
        if (
            not force
            and self._gripper_trace_last_sample_s is not None
            and elapsed - self._gripper_trace_last_sample_s
            < self._gripper_trace_interval_s
        ):
            return
        joint7 = self._gripper_feedback["joint7"]
        joint8 = self._gripper_feedback["joint8"]
        self._gripper_trace_rows.append(
            make_trace_row(
                wall_time_s=elapsed,
                joint7_command=self._gripper_trace_commands[0],
                joint8_command=self._gripper_trace_commands[1],
                joint7_feedback=joint7["position"],
                joint8_feedback=joint8["position"],
                joint7_stamp_s=joint7["stamp_s"],
                joint8_stamp_s=joint8["stamp_s"],
            )
        )
        self._gripper_trace_last_sample_s = elapsed

    def _finish_gripper_trace(self):
        if self._gripper_trace_started_s is None:
            return
        try:
            self._sample_gripper_trace()
            if not self._gripper_trace_rows:
                self._sample_gripper_trace(force=True)
            base = Path(self._gripper_trace_path)
            suffix = base.suffix or ".csv"
            stem = base.stem or "gripper_trace"
            path = base.with_name(
                f"{stem}_pair{self._gripper_trace_pair_index:02d}{suffix}"
            )
            path.parent.mkdir(parents=True, exist_ok=True)
            fields = list(self._gripper_trace_rows[0]) if self._gripper_trace_rows else [
                "wall_time_s",
                "joint7_command",
                "joint8_command",
                "joint7_feedback",
                "joint8_feedback",
                "joint7_stamp_s",
                "joint8_stamp_s",
                "symmetry_error_m",
            ]
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(self._gripper_trace_rows)
        except Exception as error:
            self.get_logger().warning(f"gripper trace write failed: {error}")
        finally:
            self._gripper_trace_started_s = None
            self._gripper_trace_last_sample_s = None
            self._gripper_trace_commands = (None, None)
            self._gripper_trace_rows = []

    def _joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        efforts = dict(zip(message.name, message.effort))
        if not all(name in positions for name in ARM_JOINTS):
            return
        self.joints = {
            "sim_time_ns": int(self.get_clock().now().nanoseconds),
            **{name: float(positions[name]) for name in ARM_JOINTS},
            "joint7": float(positions.get("joint7", math.nan)),
            "joint8": float(positions.get("joint8", math.nan)),
            "left_force_N": float(self.contact_forces["left"]),
            "right_force_N": float(self.contact_forces["right"]),
        }
        self.joints.update({
            f"velocity{index}": float(velocities.get(f"joint{index}", math.nan))
            for index in range(1, 9)
        })
        self.joints.update({
            f"effort{index}": float(efforts.get(f"joint{index}", math.nan))
            for index in range(1, 9)
        })
        self.samples.append(dict(self.joints))

    def _contact_callback(self, message, side):
        rows = extract_contact_rows(message, side, "calibration_block")
        self.contact_forces[side] = sum(
            max(float(row["normal_force_N"]), 0.0)
            for row in rows
            if math.isfinite(float(row["normal_force_N"]))
        )

    def _wait_for_server(self, client):
        deadline = time.monotonic() + float(self.config.get("action_timeout_s", 30.0))
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

    def _send_goal(self, client, goal):
        self._wait_for_server(client)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=float(self.config.get("action_timeout_s", 30.0)),
        )
        if not future.done() or future.result() is None:
            raise RuntimeError("FollowJointTrajectory goal request timed out")
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("FollowJointTrajectory goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=float(self.config.get("action_timeout_s", 30.0)),
        )
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("FollowJointTrajectory result timed out")
        result = result_future.result().result
        return int(result.error_code)

    def _send_single(self, client, joint_names, positions):
        self.samples = []
        error_code = self._send_goal(client, self._goal(joint_names, positions))
        return error_code, list(self.samples)

    def _send_gripper_pair(self, joint7, joint8):
        self._wait_for_server(self.gripper7_client)
        self._wait_for_server(self.gripper8_client)
        self.samples = []
        self._start_gripper_trace(joint7, joint8)
        goals = [
            (self.gripper7_client, self._goal(("joint7",), (joint7,))),
            (self.gripper8_client, self._goal(("joint8",), (joint8,))),
        ]
        try:
            requests = [client.send_goal_async(goal) for client, goal in goals]
            handles = []
            for request in requests:
                rclpy.spin_until_future_complete(
                    self, request, timeout_sec=float(self.config.get("action_timeout_s", 30.0))
                )
                if not request.done() or request.result() is None or not request.result().accepted:
                    raise RuntimeError("gripper FollowJointTrajectory goal was rejected")
                handles.append(request.result())
            results = []
            for handle in handles:
                result_future = handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self,
                    result_future,
                    timeout_sec=float(self.config.get("action_timeout_s", 30.0)),
                )
                if not result_future.done() or result_future.result() is None:
                    raise RuntimeError("gripper FollowJointTrajectory result timed out")
                results.append(int(result_future.result().result.error_code))
            return max(results), list(self.samples)
        finally:
            self._finish_gripper_trace()

    def _write_samples(self, cycle, stage, samples):
        path = self.output_dir / "samples.csv"
        exists = path.exists()
        fields = [
            "cycle", "stage", "sim_time_ns", *ARM_JOINTS,
            "joint7", "joint8",
            *[f"velocity{index}" for index in range(1, 9)],
            *[f"effort{index}" for index in range(1, 9)],
            "left_force_N", "right_force_N",
        ]
        mode = "w" if not self._samples_path_initialized else "a"
        with path.open(mode, newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            if mode == "w" or not exists:
                writer.writeheader()
            for sample in samples:
                writer.writerow({
                    "cycle": cycle,
                    "stage": stage,
                    **{field: sample.get(field, math.nan) for field in fields[2:]},
                })
        self._samples_path_initialized = True

    def _run_arm(self):
        arm_config = self.config["arm"]
        sequence = arm_config["sequence"]
        results = []
        cycles = int(self.config.get("cycles", 5))
        for cycle in range(1, cycles + 1):
            for item in sequence:
                code, samples = self._send_single(
                    self.arm_client, ARM_JOINTS, item["positions"]
                )
                self._write_samples(cycle, item["name"], samples)
                arm_samples = [[sample[name] for name in ARM_JOINTS] for sample in samples]
                summary = summarize_arm_run(item["positions"], arm_samples, code)
                summary.update({"cycle": cycle, "stage": item["name"]})
                results.append(summary)
        return results

    def _run_gripper(self):
        gripper_config = self.config["gripper"]
        scale = float(gripper_config["joint_position_m_per_mm"])
        results = []
        cycles = int(self.config.get("cycles", 5))
        for cycle in range(1, cycles + 1):
            for opening in gripper_config["openings_mm"]:
                half_opening = float(opening) * scale
                code, samples = self._send_gripper_pair(half_opening, -half_opening)
                self._write_samples(cycle, f"opening_{opening:g}mm", samples)
                summary = summarize_gripper_run(
                    samples,
                    target_joint7=half_opening,
                    target_joint8=-half_opening,
                )
                summary.update({"cycle": cycle, "opening_mm": float(opening), "error_code": code})
                summary["passed"] = summary["passed"] and code == 0
                results.append(summary)
        return results

    def _collect_loaded_hold(self, hold_s):
        self.samples = []
        deadline = time.monotonic() + max(float(hold_s) * 2.0, 2.0)
        first_sim_time = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.samples:
                current_sim_time = self.samples[-1].get("sim_time_ns")
                if first_sim_time is None and current_sim_time is not None:
                    first_sim_time = float(current_sim_time)
                if (
                    first_sim_time is not None
                    and float(current_sim_time) - first_sim_time >= float(hold_s) * 1e9
                ):
                    break
        return list(self.samples)

    def _run_loaded_gripper(self):
        gripper_config = self.config["gripper"]
        scale = float(gripper_config["joint_position_m_per_mm"])
        close_mm = float(gripper_config.get("loaded_close_mm", 40.0))
        hold_s = float(gripper_config.get("loaded_hold_s", 1.0))
        results = []
        cycles = int(self.config.get("cycles", 3))
        for cycle in range(1, cycles + 1):
            open_m = float(gripper_config.get("loaded_open_mm", 70.0)) * scale
            self._send_gripper_pair(open_m, -open_m)
            close_m = close_mm * scale
            code, _ = self._send_gripper_pair(close_m, -close_m)
            samples = self._collect_loaded_hold(hold_s)
            self._write_samples(cycle, "loaded_hold", samples)
            summary = summarize_loaded_gripper_run(
                samples,
                target_joint7=close_m,
                target_joint8=-close_m,
                minimum_force_N=float(gripper_config.get("minimum_force_N", 0.8)),
                minimum_bilateral_duration_s=float(
                    gripper_config.get("minimum_bilateral_duration_s", 0.8)
                ),
                hold_s=hold_s,
            )
            summary.update({"cycle": cycle, "opening_mm": close_mm, "error_code": code})
            post_open_m = float(gripper_config.get("loaded_open_mm", 70.0)) * scale
            open_code, open_samples = self._send_gripper_pair(post_open_m, -post_open_m)
            self._write_samples(cycle, "loaded_post_open", open_samples)
            summary["post_open_error_code"] = open_code
            summary["passed"] = summary["passed"] and code == 0 and open_code == 0
            results.append(summary)
        return results

    def run(self):
        if self.mode == "arm":
            results = self._run_arm()
        elif self.mode == "gripper":
            results = self._run_gripper()
        else:
            results = self._run_loaded_gripper()
        passed = bool(results) and all(result["passed"] for result in results)
        summary = {"mode": self.mode, "cycles": int(self.config.get("cycles", 5)), "passed": passed, "runs": results}
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        self.get_logger().info(json.dumps(summary, sort_keys=True))
        return 0 if passed else 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = ControlPhysicsQualification()
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
