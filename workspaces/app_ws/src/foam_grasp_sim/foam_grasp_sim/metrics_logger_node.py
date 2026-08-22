#!/usr/bin/env python3
"""Record Stage-5 simulation events, state samples and derived metrics."""

import csv
import json
import math
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import tf2_ros

from foam_grasp_sim.benchmark_event_logger import event_row, parse_event
from foam_grasp_sim.metrics_model import MetricsAccumulator


STATE_FIELDS = (
    "sim_time_ns",
    "target_ground_truth_x", "target_ground_truth_y", "target_ground_truth_z",
    "target_observed_x", "target_observed_y", "target_observed_z",
    "target_selected_x", "target_selected_y", "target_selected_z",
    "target_latched_x", "target_latched_y", "target_latched_z",
    "tcp_x", "tcp_y", "tcp_z",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper",
    "gate_ready", "method_state", "method", "scenario", "seed",
    "observation_valid", "observation_age_s", "selected_age_s", "latched_age_s",
)


def _point_values(message):
    if message is None:
        return None
    values = (float(message.point.x), float(message.point.y), float(message.point.z))
    return values if all(math.isfinite(value) for value in values) else None


def _message_stamp_ns(message):
    if message is None:
        return None
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class MetricsLoggerNode(Node):
    def __init__(self):
        super().__init__("foam_metrics_logger")
        self.declare_parameter("record_benchmark", True)
        self.declare_parameter("results_root", "results")
        self.declare_parameter("run_id", "")
        self.declare_parameter("scenario", "static")
        self.declare_parameter("method", "gated")
        self.declare_parameter("target_model", "cube")
        self.declare_parameter("seed", 42)
        self.declare_parameter("metrics_rate", 10.0)
        self.declare_parameter("tool_offset", 0.1358)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "link6")
        self.declare_parameter("config_hash", "")
        self.declare_parameter("pair_id", "")
        self.declare_parameter("condition_json", "{}")

        self.scenario = str(self.get_parameter("scenario").value)
        self.method = str(self.get_parameter("method").value)
        self.target_model = str(self.get_parameter("target_model").value)
        self.seed = int(self.get_parameter("seed").value)
        self.metrics_rate = float(self.get_parameter("metrics_rate").value)
        self.tool_offset = float(self.get_parameter("tool_offset").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tcp_frame = str(self.get_parameter("tcp_frame").value)
        self.config_hash = str(self.get_parameter("config_hash").value)
        self.pair_id = str(self.get_parameter("pair_id").value)
        self.condition_json = str(self.get_parameter("condition_json").value)
        if not 0.1 <= self.metrics_rate <= 100.0:
            raise RuntimeError("metrics_rate must be within 0.1--100 Hz")
        if not 0.0 <= self.tool_offset <= 0.30:
            raise RuntimeError("tool_offset must be within 0--0.30 m")

        root = Path(str(self.get_parameter("results_root").value))
        run_id = str(self.get_parameter("run_id").value).strip()
        if not run_id:
            run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.run_id = run_id
        self.run_dir = root / run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.events = []
        self.states = []
        self.metrics = MetricsAccumulator()
        self.latest = {
            "ground_truth": None,
            "observed": None,
            "selected": None,
            "latched": None,
        }
        self.latest_stamp = {key: None for key in self.latest}
        self.joints = {}
        self.gate_ready = False
        self.method_state = ""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.point_subscriptions = []
        topics = {
            "ground_truth": "/foam_grasp_sim/target_ground_truth",
            "observed": f"/foam_grasp/{self.target_model}_point_base",
            "selected": f"/foam_grasp/{self.target_model}_method_point_base",
            "latched": "/foam_grasp/target_point_base_latched",
        }
        for key, topic in topics.items():
            self.point_subscriptions.append(
                self.create_subscription(
                    PointStamped,
                    topic,
                    lambda message, current_key=key: self._point_callback(current_key, message),
                    30,
                )
            )
        self.event_subscription = self.create_subscription(
            String, "/foam_grasp/benchmark_event", self.event_callback, 100
        )
        self.ready_subscription = self.create_subscription(
            Bool, "/foam_grasp/method_ready", self.ready_callback, 10
        )
        self.state_subscription = self.create_subscription(
            String, "/foam_grasp/method_state", self.state_callback, 20
        )
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, 30
        )
        self.sample_timer = self.create_timer(1.0 / self.metrics_rate, self.sample_state)
        self.flush_timer = self.create_timer(5.0, self.flush)
        self.write_metadata()
        self.get_logger().info(f"Benchmark results: {self.run_dir}")

    def _point_callback(self, key, message):
        point = _point_values(message)
        if point is not None:
            self.latest[key] = point
            self.latest_stamp[key] = _message_stamp_ns(message) or int(self.get_clock().now().nanoseconds)

    def ready_callback(self, message):
        self.gate_ready = bool(message.data)

    def state_callback(self, message):
        self.method_state = str(message.data)

    def joint_callback(self, message):
        values = dict(zip(message.name, message.position))
        self.joints = {name: float(values[name]) for name in (
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"
        ) if name in values}

    def event_callback(self, message):
        try:
            event = parse_event(message.data)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f"Ignoring invalid benchmark event: {error}")
            return
        self.events.append(event)
        self.metrics.record_event(event)

    @staticmethod
    def _rotate_z(quaternion, offset):
        x, y, z, w = quaternion
        return (
            2.0 * (x * z + w * y) * offset,
            2.0 * (y * z - w * x) * offset,
            (1.0 - 2.0 * (x * x + y * y)) * offset,
        )

    def tcp_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, Time()
            ).transform
        except Exception:
            return None
        translation = transform.translation
        rotation = transform.rotation
        offset = self._rotate_z(
            (rotation.x, rotation.y, rotation.z, rotation.w), self.tool_offset
        )
        return (
            float(translation.x) + offset[0],
            float(translation.y) + offset[1],
            float(translation.z) + offset[2],
        )

    def sample_state(self):
        sim_time_ns = int(self.get_clock().now().nanoseconds)
        row = {field: "" for field in STATE_FIELDS}
        row.update({
            "sim_time_ns": sim_time_ns,
            "gate_ready": int(self.gate_ready),
            "method_state": self.method_state,
            "method": self.method,
            "scenario": self.scenario,
            "seed": self.seed,
            "observation_valid": int(self.latest["observed"] is not None),
        })
        for key in ("ground_truth", "observed", "selected", "latched"):
            point = self.latest[key]
            if point is not None:
                for axis, value in zip("xyz", point):
                    row[f"target_{key}_{axis}"] = value
                stamp = self.latest_stamp[key]
                if stamp is not None:
                    age_field = {
                        "ground_truth": "observation_age_s",
                        "observed": "observation_age_s",
                        "selected": "selected_age_s",
                        "latched": "latched_age_s",
                    }[key]
                    row[age_field] = max(
                        0.0, (sim_time_ns - stamp) / 1e9
                    )
        tcp = self.tcp_position()
        if tcp is not None:
            row.update({f"tcp_{axis}": value for axis, value in zip("xyz", tcp)})
        for name, value in self.joints.items():
            row["gripper" if name == "joint7" else name] = value
        self.states.append(row)
        self.metrics.record_state(row)

    def _atomic_write(self, path, writer):
        fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
                writer(stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def write_metadata(self):
        metadata = {
            "schema_version": 1,
            "run_id": self.run_id,
            "method": self.method,
            "scenario": self.scenario,
            "target_model": self.target_model,
            "seed": self.seed,
            "tool_offset_m": self.tool_offset,
            "base_frame": self.base_frame,
            "tcp_frame": self.tcp_frame,
            "config_hash": self.config_hash,
            "pair_id": self.pair_id,
            "condition_json": self.condition_json,
        }
        self._atomic_write(
            self.run_dir / "metadata.json",
            lambda stream: json.dump(metadata, stream, indent=2, sort_keys=True),
        )

    def flush(self):
        self._atomic_write(
            self.run_dir / "events.csv",
            lambda stream: self._write_events(stream),
        )
        self._atomic_write(
            self.run_dir / "states.csv",
            lambda stream: self._write_states(stream),
        )
        metrics = self.metrics.finalize()
        self._atomic_write(
            self.run_dir / "metrics.json",
            lambda stream: json.dump(metrics, stream, indent=2, sort_keys=True),
        )

    def _write_events(self, stream):
        fields = ("schema_version", "sim_time_ns", "event", "method", "scenario", "seed", "details")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for event in self.events:
            writer.writerow(event_row(event))

    def _write_states(self, stream):
        writer = csv.DictWriter(stream, fieldnames=STATE_FIELDS)
        writer.writeheader()
        writer.writerows(self.states)

    def destroy_node(self):
        self.flush()
        return super().destroy_node()


def main():
    rclpy.init()
    node = MetricsLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
