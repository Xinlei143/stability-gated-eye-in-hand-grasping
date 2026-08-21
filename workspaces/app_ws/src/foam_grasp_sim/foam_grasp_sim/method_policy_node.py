#!/usr/bin/env python3
"""ROS adapter for the deterministic stage-4 method policy."""

import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from foam_grasp.benchmark_events import BenchmarkEventPublisher
from foam_grasp_sim.method_policy import MethodConfig, MethodPolicy


TARGET_MODELS = ("cube", "cylinder", "sphere")


class MethodPolicyNode(Node):
    def __init__(self):
        super().__init__("foam_method_policy")
        self.declare_parameter("target_model", "cube")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("method", "gated")
        self.declare_parameter("input_topic", "")
        self.declare_parameter("output_topic", "")
        self.declare_parameter("stability_duration", 5.0)
        self.declare_parameter("position_spread_threshold", 0.006)
        self.declare_parameter("minimum_stable_samples", 25)
        self.declare_parameter("observation_timeout", 1.0)
        self.declare_parameter("scenario", "static")
        self.declare_parameter("seed", 42)

        target_model = str(self.get_parameter("target_model").value)
        if target_model not in TARGET_MODELS:
            raise RuntimeError("target_model must be one of " + ", ".join(TARGET_MODELS))
        self.base_frame = str(self.get_parameter("base_frame").value)
        if not self.base_frame:
            raise RuntimeError("base_frame must not be empty")
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if not input_topic:
            input_topic = f"/foam_grasp/{target_model}_point_base"
        if not output_topic:
            output_topic = f"/foam_grasp/{target_model}_method_point_base"

        try:
            config = MethodConfig(
                method=str(self.get_parameter("method").value),
                stability_duration_s=float(self.get_parameter("stability_duration").value),
                position_spread_threshold_m=float(
                    self.get_parameter("position_spread_threshold").value
                ),
                minimum_stable_samples=int(
                    self.get_parameter("minimum_stable_samples").value
                ),
                observation_timeout_s=float(
                    self.get_parameter("observation_timeout").value
                ),
            )
        except (TypeError, ValueError) as error:
            raise RuntimeError(str(error)) from error
        self.policy = MethodPolicy(config)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.point_publisher = self.create_publisher(PointStamped, output_topic, 20)
        self.latched_publisher = self.create_publisher(
            PointStamped, f"{output_topic}_latched", latched_qos
        )
        self.compatibility_publisher = self.create_publisher(
            PointStamped, "/foam_grasp/target_point_base_latched", latched_qos
        )
        self.ready_publisher = self.create_publisher(Bool, "/foam_grasp/method_ready", latched_qos)
        self.state_publisher = self.create_publisher(String, "/foam_grasp/method_state", 10)
        self.selected_class_publisher = self.create_publisher(
            String, "/foam_grasp/method_target_class", latched_qos
        )
        self.committed_class_publisher = self.create_publisher(
            String, "/foam_grasp/latched_target_class", latched_qos
        )
        self.subscription = self.create_subscription(
            PointStamped, input_topic, self.point_callback, 20
        )
        self.expiry_timer = self.create_timer(0.05, self.expiry_tick)
        self.last_stamp = None
        self._last_ready = False
        self._gate_active = False
        self.target_model = target_model
        self.scenario = str(self.get_parameter("scenario").value)
        self.seed = int(self.get_parameter("seed").value)
        self.event_publisher = BenchmarkEventPublisher(self)
        self.commit_service = self.create_service(
            Trigger, "/foam_grasp/commit_method_target", self.commit_callback
        )
        self.reset_service = self.create_service(
            Trigger, "/foam_grasp/reset_method", self.reset_callback
        )
        self.get_logger().info(
            f"Method policy: method={config.method}, input={input_topic}, output={output_topic}"
        )

    def _timestamp(self, message):
        timestamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        if timestamp <= 0.0:
            timestamp = self.get_clock().now().nanoseconds * 1e-9
        return timestamp

    def _publish(self, point, stamp, *, latched=False):
        if point is None:
            return
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.base_frame
        message.point.x, message.point.y, message.point.z = point
        if latched:
            self.latched_publisher.publish(message)
            self.compatibility_publisher.publish(message)
        else:
            self.point_publisher.publish(message)

    def _publish_selected_class(self):
        message = String()
        message.data = self.target_model
        self.selected_class_publisher.publish(message)

    def _publish_committed_class(self):
        message = String()
        message.data = self.target_model
        self.committed_class_publisher.publish(message)

    def _publish_state(self, decision):
        ready = Bool()
        ready.data = bool(decision.ready)
        self.ready_publisher.publish(ready)
        state = String()
        state.data = "READY" if decision.ready else decision.reason
        self.state_publisher.publish(state)

    def _event(self, name, *, details=None, stamp=None):
        sim_time_ns = None
        if stamp is not None:
            sim_time_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self.event_publisher.publish(
            name,
            method=self.policy.config.method,
            scenario=self.scenario,
            seed=self.seed,
            details=details,
            sim_time_ns=sim_time_ns,
        )

    def point_callback(self, message):
        if message.header.frame_id != self.base_frame:
            return
        point = (message.point.x, message.point.y, message.point.z)
        if not all(math.isfinite(float(value)) for value in point):
            return
        timestamp = self._timestamp(message)
        try:
            decision = self.policy.update(point, timestamp)
        except ValueError as error:
            self.get_logger().warning(f"Ignoring method observation: {error}")
            return
        self._publish(decision.point, message.header.stamp)
        self._publish_selected_class()
        if self.policy.config.method == "gated" and not self._gate_active:
            self._gate_active = True
            self._event("GATE_STARTED", stamp=message.header.stamp)
        if decision.reset and self._gate_active:
            self._gate_active = False
            self._event("GATE_RESET", details={"reason": decision.reason}, stamp=message.header.stamp)
        if decision.ready and not self._last_ready:
            self._event("READY", details={"sample_count": self.policy.stable_sample_count}, stamp=message.header.stamp)
        self._last_ready = bool(decision.ready)
        self._publish_state(decision)

    def commit_callback(self, _request, response):
        try:
            point = self.policy.commit()
        except RuntimeError as error:
            response.success = False
            response.message = str(error)
            return response
        stamp = self.get_clock().now().to_msg()
        self._publish(point, stamp, latched=True)
        self._publish_committed_class()
        self._event("TARGET_RELATCHED", details={"point": list(point)}, stamp=stamp)
        response.success = True
        response.message = (
            f"committed {self.target_model}: "
            f"xyz=({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f}) m"
        )
        return response

    def reset_callback(self, _request, response):
        self.policy.reset()
        self._gate_active = False
        self._last_ready = False
        decision = self.policy._decision(reset=True, reason="reset")
        self._publish_state(decision)
        self._event("GATE_RESET", details={"reason": "explicit_reset"})
        response.success = True
        response.message = "method policy reset"
        return response

    def expiry_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.policy.last_observation_at is None:
            return
        decision = self.policy.expire(now)
        if decision.reset:
            if self._gate_active:
                self._gate_active = False
                self._event("GATE_RESET", details={"reason": decision.reason})
            self._last_ready = False
            self._publish_state(decision)


def main():
    rclpy.init()
    node = MethodPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
