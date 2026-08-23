#!/usr/bin/env python3
"""Record raw Gazebo finger contact wrenches during a grasp qualification."""

import csv
import json
import math
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .contact_diagnostics import extract_contact_rows


CONTACT_FIELDS = [
    "sim_time_ns",
    "stage",
    "joint7_position_m",
    "joint8_position_m",
    "joint7_velocity_m_s",
    "joint8_velocity_m_s",
    "joint7_effort_N",
    "joint8_effort_N",
    "gripper_total_opening_m",
    "gripper_symmetry_error_m",
    "side",
    "collision1",
    "collision2",
    "contact_index",
    "position_x_m",
    "position_y_m",
    "position_z_m",
    "normal_x",
    "normal_y",
    "normal_z",
    "depth_m",
    "force_x_N",
    "force_y_N",
    "force_z_N",
    "torque_x_Nm",
    "torque_y_Nm",
    "torque_z_Nm",
    "normal_force_N",
    "tangential_force_N",
]


class ContactDiagnosticsNode(Node):
    def __init__(self):
        super().__init__("contact_diagnostics")
        self.declare_parameter("target_entity", "foam_cube")
        self.declare_parameter("output_path", "/tmp/foam_grasp_contact_diagnostics.csv")
        self.target_entity = str(self.get_parameter("target_entity").value)
        output_path = Path(str(self.get_parameter("output_path").value))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output_path.open("w", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=CONTACT_FIELDS)
        self._writer.writeheader()
        self._stream.flush()
        self.stage = "unknown"
        self.joints = {}
        self.create_subscription(
            ContactsState,
            "/foam_grasp_sim/link7_contacts",
            lambda message: self._record(message, "left"),
            50,
        )
        self.create_subscription(
            ContactsState,
            "/foam_grasp_sim/link8_contacts",
            lambda message: self._record(message, "right"),
            50,
        )
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 50)
        self.create_subscription(String, "/foam_grasp/benchmark_event", self._event_callback, 20)

    def _joint_callback(self, message):
        values = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        efforts = dict(zip(message.name, message.effort))
        for name in ("joint7", "joint8"):
            if name in values:
                self.joints[f"{name}_position_m"] = float(values[name])
            if name in velocities:
                self.joints[f"{name}_velocity_m_s"] = float(velocities[name])
            if name in efforts:
                self.joints[f"{name}_effort_N"] = float(efforts[name])

    def _event_callback(self, message):
        try:
            event = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.stage = str(event.get("event", self.stage))

    def _record(self, message, side):
        timestamp = self.get_clock().now().nanoseconds
        for contact in extract_contact_rows(message, side, self.target_entity):
            joint7 = self.joints.get("joint7_position_m", math.nan)
            joint8 = self.joints.get("joint8_position_m", math.nan)
            row = {
                "sim_time_ns": timestamp,
                "stage": self.stage,
                "joint7_position_m": joint7,
                "joint8_position_m": joint8,
                "joint7_velocity_m_s": self.joints.get("joint7_velocity_m_s", math.nan),
                "joint8_velocity_m_s": self.joints.get("joint8_velocity_m_s", math.nan),
                "joint7_effort_N": self.joints.get("joint7_effort_N", math.nan),
                "joint8_effort_N": self.joints.get("joint8_effort_N", math.nan),
                "gripper_total_opening_m": joint7 - joint8,
                "gripper_symmetry_error_m": abs(joint7 + joint8),
            }
            row.update(contact)
            self._writer.writerow(row)
        self._stream.flush()

    def close(self):
        if not self._stream.closed:
            self._stream.flush()
            self._stream.close()

    def destroy_node(self):
        self.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ContactDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
