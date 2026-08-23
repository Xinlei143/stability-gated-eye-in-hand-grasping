#!/usr/bin/env python3
"""Optional, contact-gated attachment fallback for Gazebo Classic."""

import json
import threading
import time

import rclpy
from gazebo_model_attachment_plugin_msgs.srv import Attach, Detach
from gazebo_msgs.msg import ContactsState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


ATTACHMENT_LINK = "link6"


def contact_state_has_entity(message, entity_name):
    for state in getattr(message, "states", ()):
        names = (
            str(getattr(state, "collision1_name", "")),
            str(getattr(state, "collision2_name", "")),
        )
        if any(entity_name in name for name in names):
            return True
    return False


class GraspAssistNode(Node):
    """Attach only after both fingers report stable target contact."""

    def __init__(self):
        super().__init__("foam_grasp_assist")
        self.declare_parameter("target_entity", "foam_cube")
        self.declare_parameter("contact_hold_s", 0.2)
        self.declare_parameter("max_symmetry_error_m", 0.002)
        self.declare_parameter("seed", 42)
        self.target_entity = str(self.get_parameter("target_entity").value)
        self.contact_hold_s = float(self.get_parameter("contact_hold_s").value)
        self.max_symmetry_error_m = float(
            self.get_parameter("max_symmetry_error_m").value
        )
        self.joint7 = None
        self.joint8 = None
        self.left_contact = False
        self.right_contact = False
        self.both_contact_since = None
        self.attached = False
        seed = int(self.get_parameter("seed").value)
        self.joint_name = f"foam_grasp_assist_{seed}"
        self.callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            ContactsState,
            "/foam_grasp_sim/link7_contacts",
            lambda message: self._contact_callback(message, "left"),
            20,
        )
        self.create_subscription(
            ContactsState,
            "/foam_grasp_sim/link8_contacts",
            lambda message: self._contact_callback(message, "right"),
            20,
        )
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 20)
        self.create_subscription(String, "/foam_grasp/benchmark_event", self._event_callback, 20)
        self.attach_client = self.create_client(
            Attach, "/gazebo/attach", callback_group=self.callback_group
        )
        self.detach_client = self.create_client(
            Detach, "/gazebo/detach", callback_group=self.callback_group
        )
        self.create_service(
            Trigger,
            "/foam_grasp_sim/prepare_grasp_assist",
            self.prepare_callback,
            callback_group=self.callback_group,
        )

    def _contact_callback(self, message, side):
        value = contact_state_has_entity(message, self.target_entity)
        if side == "left":
            self.left_contact = value
        else:
            self.right_contact = value
        if self.left_contact and self.right_contact:
            if self.both_contact_since is None:
                self.both_contact_since = time.monotonic()
        else:
            self.both_contact_since = None

    def _joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        if "joint7" in positions:
            self.joint7 = float(positions["joint7"])
        if "joint8" in positions:
            self.joint8 = float(positions["joint8"])

    def _event_callback(self, message):
        try:
            event = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if event.get("event") in {"TRIAL_FINISHED", "TRIAL_FAILED"}:
            self.detach()

    def prepare_callback(self, _request, response):
        now = time.monotonic()
        if not self.left_contact or not self.right_contact:
            response.success = False
            response.message = "both fingers are not in contact with the target"
            return response
        if (
            self.both_contact_since is None
            or now - self.both_contact_since < self.contact_hold_s
        ):
            response.success = False
            response.message = "dual-finger contact is not stable yet"
            return response
        if self.joint7 is None or self.joint8 is None:
            response.success = False
            response.message = "missing joint7/joint8 feedback"
            return response
        symmetry_error = abs(self.joint7 + self.joint8)
        if symmetry_error > self.max_symmetry_error_m:
            response.success = False
            response.message = f"finger symmetry error {symmetry_error:.4f}m"
            return response
        if self.attached:
            response.success = True
            response.message = "target already attached"
            return response
        try:
            attach = self._call_attach()
        except RuntimeError as error:
            response.success = False
            response.message = str(error)
            return response
        response.success = bool(attach.success)
        response.message = str(attach.message)
        self.attached = response.success
        return response

    def _call_attach(self):
        if not self.attach_client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError("/gazebo/attach service unavailable")
        request = Attach.Request(
            joint_name=self.joint_name,
            model_name_1="piper",
            # Gazebo Classic merges the upstream fixed gripper_base link into
            # the model; link6 is the actual parent link present in
            # /gazebo/link_states and follows the tool during lift.
            link_name_1=ATTACHMENT_LINK,
            model_name_2=self.target_entity,
            link_name_2="cube_link",
        )
        future = self.attach_client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=2.0):
            raise RuntimeError("/gazebo/attach returned no response")
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def detach(self):
        if not self.attached:
            return
        if not self.detach_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning("/gazebo/detach service unavailable")
            return
        future = self.detach_client.call_async(
            Detach.Request(
                joint_name=self.joint_name,
                model_name_1="piper",
                model_name_2=self.target_entity,
            )
        )
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        completed.wait(timeout=1.0)
        self.attached = False


def main():
    rclpy.init()
    node = GraspAssistNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.detach()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
