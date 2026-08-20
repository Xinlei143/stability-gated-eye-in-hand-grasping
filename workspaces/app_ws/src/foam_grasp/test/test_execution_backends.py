import math
import types
import unittest

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from foam_grasp.execution import create_backend
from foam_grasp.execution.base_backend import ExecutionResult
from foam_grasp.execution.ros2_control_backend import Ros2ControlBackend


class FakeNode:
    command_names = (
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    )
    arm_joint_names = command_names[:6]

    @staticmethod
    def is_finite(value):
        return math.isfinite(float(value))


class ExecutionBackendTest(unittest.TestCase):
    def test_execution_result_keeps_legacy_unpacking(self):
        result = ExecutionResult(1.25, 0.01, 0.02)
        self.assertEqual((1.25, 0.01), tuple(result))

    def test_simulation_backend_normalizes_joint7_alias(self):
        backend = object.__new__(Ros2ControlBackend)
        backend.node = FakeNode()
        message = JointState()
        message.name = list(FakeNode.arm_joint_names) + ["joint7"]
        message.position = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6, 0.025]
        self.assertEqual(
            backend.normalize_joint_positions(message),
            list(message.position),
        )

    def test_simulation_backend_rejects_incomplete_feedback(self):
        backend = object.__new__(Ros2ControlBackend)
        backend.node = FakeNode()
        message = JointState()
        message.name = list(FakeNode.arm_joint_names)
        message.position = [0.0] * 6
        self.assertIsNone(backend.normalize_joint_positions(message))

    def test_unknown_backend_fails_closed(self):
        with self.assertRaises(ValueError):
            create_backend(types.SimpleNamespace(), "typo")

    def test_single_point_trajectory_is_standard_ros_message(self):
        trajectory = Ros2ControlBackend._single_point_trajectory(
            ["joint1"], [0.25], 0.15
        )
        self.assertIsInstance(trajectory, JointTrajectory)
        self.assertEqual(trajectory.joint_names, ["joint1"])
        self.assertAlmostEqual(trajectory.points[0].positions[0], 0.25)
        self.assertEqual(trajectory.points[0].time_from_start.nanosec, 150000000)


if __name__ == "__main__":
    unittest.main()
