#!/usr/bin/env python3
"""Turn Gazebo ground truth into reproducible foam_grasp observations."""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from foam_grasp.benchmark_events import BenchmarkEventPublisher
from foam_grasp_sim.perception_model import (
    DelayedPointBuffer,
    DisturbanceModel,
    effective_latency_seconds,
    validate_perception_parameters,
)


TARGET_MODELS = ("cube", "cylinder", "sphere")


def _seconds_to_stamp(seconds, stamp):
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


class SimulatedPerceptionNode(Node):
    """Publish either ideal or disturbed observations for one target class."""

    def __init__(self):
        super().__init__("foam_simulated_perception")
        self.declare_parameter("target_model", "cube")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "ground_truth_topic", "/foam_grasp_sim/target_ground_truth"
        )
        self.declare_parameter("source", "ground_truth")
        self.declare_parameter("sampling_rate", 10.0)
        self.declare_parameter("latency_ms", 100.0)
        self.declare_parameter("noise_std_mm", 5.0)
        self.declare_parameter("dropout_probability", 0.10)
        self.declare_parameter("outlier_probability", 0.01)
        self.declare_parameter("outlier_range_mm", 50.0)
        self.declare_parameter("history_duration", 10.0)
        self.declare_parameter("seed", 42)
        self.declare_parameter("scenario", "static")
        self.declare_parameter("target_timeout", 0.5)

        self.target_model = str(self.get_parameter("target_model").value)
        if self.target_model not in TARGET_MODELS:
            raise RuntimeError(
                "target_model must be one of " + ", ".join(TARGET_MODELS)
            )
        self.base_frame = str(self.get_parameter("base_frame").value)
        if not self.base_frame:
            raise RuntimeError("base_frame must not be empty")
        try:
            self.settings = validate_perception_parameters(
                str(self.get_parameter("source").value),
                self.get_parameter("sampling_rate").value,
                self.get_parameter("latency_ms").value,
                self.get_parameter("noise_std_mm").value,
                self.get_parameter("dropout_probability").value,
                self.get_parameter("outlier_probability").value,
                self.get_parameter("outlier_range_mm").value,
                self.get_parameter("history_duration").value,
                self.get_parameter("seed").value,
            )
        except ValueError as error:
            raise RuntimeError(str(error)) from error
        if self.settings["source"] == "rgbd":
            raise RuntimeError(
                "rgbd is a reserved launch mode; do not start simulated_perception"
            )

        self.buffer = DelayedPointBuffer(self.settings["history_duration"])
        self.disturbance = DisturbanceModel(
            self.settings["seed"],
            self.settings["noise_std_m"],
            self.settings["dropout_probability"],
            self.settings["outlier_probability"],
            self.settings["outlier_range_m"],
        )
        self.last_published_source_stamp = -math.inf
        self.scenario = str(self.get_parameter("scenario").value)
        self.seed = int(self.get_parameter("seed").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        if not math.isfinite(self.target_timeout) or self.target_timeout <= 0.0:
            raise RuntimeError("target_timeout must be positive")
        self.event_publisher = BenchmarkEventPublisher(self)
        self.target_visible = False
        self.last_observation_at = None
        self.ground_truth_subscription = self.create_subscription(
            PointStamped,
            str(self.get_parameter("ground_truth_topic").value),
            self.ground_truth_callback,
            20,
        )
        self.publisher = self.create_publisher(
            PointStamped,
            f"/foam_grasp/{self.target_model}_point_base",
            20,
        )
        self.sample_timer = self.create_timer(
            1.0 / self.settings["sampling_rate"],
            self.sample_and_publish,
        )
        self.get_logger().info(
            "Simulated perception: source=%s, target=%s, output=%s"
            % (
                self.settings["source"],
                self.target_model,
                f"/foam_grasp/{self.target_model}_point_base",
            )
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def ground_truth_callback(self, message):
        if message.header.frame_id != self.base_frame:
            return
        point = (message.point.x, message.point.y, message.point.z)
        if not all(math.isfinite(float(value)) for value in point):
            return
        timestamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        if timestamp <= 0.0:
            timestamp = self.now_seconds()
        try:
            self.buffer.append(timestamp, point)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid ground truth: {error}")

    def sample_and_publish(self):
        now = self.now_seconds()
        if (
            self.target_visible
            and self.last_observation_at is not None
            and now - self.last_observation_at > self.target_timeout
        ):
            self.target_visible = False
            self.event_publisher.publish(
                "TARGET_LOST",
                method="",
                scenario=self.scenario,
                seed=self.seed,
            )
        self.buffer.prune(now)
        source = self.buffer.latest_at_or_before(
            now
            - effective_latency_seconds(
                self.settings["source"],
                self.settings["latency_seconds"],
            )
        )
        if source is None or source.timestamp <= self.last_published_source_stamp:
            return
        self.last_published_source_stamp = source.timestamp
        if self.settings["source"] == "ground_truth":
            observed = source.point
        else:
            observed = self.disturbance.apply(source.point)
        if observed is None:
            return
        message = PointStamped()
        _seconds_to_stamp(source.timestamp, message.header.stamp)
        message.header.frame_id = self.base_frame
        message.point.x, message.point.y, message.point.z = observed
        self.publisher.publish(message)
        self.last_observation_at = now
        if not self.target_visible:
            self.target_visible = True
            self.event_publisher.publish(
                "TARGET_OBSERVED",
                method="",
                scenario=self.scenario,
                seed=self.seed,
                sim_time_ns=(
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                ),
            )


def main():
    rclpy.init()
    node = SimulatedPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
