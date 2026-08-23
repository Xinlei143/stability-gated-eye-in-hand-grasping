import math
from pathlib import Path
import types
import unittest

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

    def current_positions(self):
        return [0.0] * 6 + [0.07]


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

    def test_gripper_trajectories_command_both_fingers_symmetrically(self):
        trajectories = Ros2ControlBackend._paired_gripper_trajectories(0.04, 0.5)

        self.assertEqual(trajectories[0].joint_names, ["joint7"])
        self.assertEqual(trajectories[1].joint_names, ["joint8"])
        self.assertAlmostEqual(trajectories[0].points[0].positions[0], 0.02)
        self.assertAlmostEqual(trajectories[1].points[0].positions[0], -0.02)

    def test_cartesian_trajectory_is_retimed_from_requested_joint_rate(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = ["joint1", "joint2"]
        first = JointTrajectoryPoint()
        first.positions = [0.0, 0.0]
        second = JointTrajectoryPoint()
        second.positions = [0.1, 0.04]
        trajectory.points = [first, second]

        retimed = Ros2ControlBackend._retime_trajectory(trajectory, 0.05)

        self.assertEqual(retimed.points[0].time_from_start.sec, 0)
        self.assertEqual(retimed.points[1].time_from_start.sec, 2)
        self.assertEqual(retimed.points[1].time_from_start.nanosec, 0)

    def test_simulation_gripper_final_error_compares_in_feedback_units(self):
        backend = object.__new__(Ros2ControlBackend)
        backend.node = FakeNode()
        backend.gripper_joint_name = "joint7"
        backend.gripper_command_scale = 0.5
        backend.gripper_feedback_scale = 2.0
        trajectory = Ros2ControlBackend._single_point_trajectory(
            ["joint7"], [0.035], 0.15
        )
        self.assertAlmostEqual(backend._final_error(trajectory), 0.0)

    def test_simulation_gripper_command_allows_contact_stop(self):
        backend = object.__new__(Ros2ControlBackend)
        backend.node = FakeNode()
        backend.gripper_joint_name = "joint7"
        backend.gripper_command_scale = 0.5
        backend.gripper_client = object()
        backend.gripper8_client = object()
        backend._prepared = True
        calls = {}

        def fake_execute_pair(trajectories, label):
            calls["trajectory_count"] = len(trajectories)
            return ExecutionResult(1.0, 0.0, 0.0, gripper_position=0.07)

        backend._execute_gripper_pair = fake_execute_pair
        backend._gripper_feedback_position = 0.04
        backend._gripper8_feedback_position = -0.04
        backend.node.spin_for = lambda seconds: None
        backend.command_gripper(0.04, types.SimpleNamespace())
        self.assertEqual(calls["trajectory_count"], 2)

    def test_simulation_backend_does_not_use_node_publisher_hack(self):
        backend = object.__new__(Ros2ControlBackend)
        backend.node = FakeNode()
        backend._prepared = False
        backend._active = False
        self.assertFalse(backend.execution_prepared)
        self.assertFalse(hasattr(backend.node, "command_publisher"))

    def test_vendor_execution_methods_live_in_real_backend(self):
        source_root = Path(__file__).parents[1] / "foam_grasp"
        pregrasp_source = (source_root / "foam_move_to_pregrasp.py").read_text()
        sequence_source = (source_root / "foam_cube_grasp_sequence.py").read_text()
        self.assertNotIn("def _real_", pregrasp_source)
        self.assertNotIn("def _real_", sequence_source)

if __name__ == "__main__":
    unittest.main()
