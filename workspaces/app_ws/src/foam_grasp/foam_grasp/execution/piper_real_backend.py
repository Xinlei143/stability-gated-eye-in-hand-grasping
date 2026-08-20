"""Execution adapter for the local Piper ROS 2 driver.

The existing, conservative Piper command implementation remains in the
planning node during the migration.  This adapter is the only backend that
is allowed to use the driver's ``/joint_states`` command topic and Piper
status checks.
"""

from .base_backend import ExecutionBackend
from sensor_msgs.msg import JointState


class PiperRealBackend(ExecutionBackend):
    name = "real"
    feedback_topic = "/joint_states_single"
    requires_piper_status = True
    is_simulation = False

    def ensure_command_path_is_exclusive(self):
        return self.node._real_ensure_command_path_is_exclusive()

    def prepare_execution(self):
        self.ensure_command_path_is_exclusive()
        if self.node.command_publisher is None:
            self.node.command_publisher = self.node.create_publisher(
                JointState,
                "/joint_states",
                10,
            )
        self.node.spin_for(1.0)

    def publish_message(self, message):
        if self.node.command_publisher is None:
            raise RuntimeError("真机执行后端尚未准备")
        self.node.command_publisher.publish(message)

    def make_command(self, arm_positions, actual_gripper_m, speed_percent, effort):
        return self.node._real_make_command(
            arm_positions, actual_gripper_m, speed_percent, effort
        )

    def hold_position(self, actual_gripper_m, speed_percent, effort):
        return self.node._real_publish_hold(
            actual_gripper_m, speed_percent, effort
        )

    def execute_arm_trajectory(self, *args, **kwargs):
        return self.node._real_execute_trajectory(*args, **kwargs)

    def execute_cartesian_trajectory(self, *args, **kwargs):
        return self.node._real_execute_untimed_cartesian(*args, **kwargs)

    def command_gripper(self, *args, **kwargs):
        return self.node._real_command_gripper(*args, **kwargs)

    def send_servo_command(self, *args, **kwargs):
        return self.node._real_send_servo_command(*args, **kwargs)
