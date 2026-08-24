import re
import types
import unittest
from pathlib import Path
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState

from foam_grasp.foam_cube_grasp_sequence import FoamCubeGraspSequence


def pose(x=0.4, y=0.0, z=0.2):
    message = PoseStamped()
    message.header.frame_id = "base_link"
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation.w = 1.0
    return message


class TrackingPregraspPlanningTest(unittest.TestCase):
    def make_node(self):
        node = object.__new__(FoamCubeGraspSequence)
        node.target_class = "cube"
        node.tool_offset = 0.1358
        node.cylinder_chord_offset_m = 0.018
        node.grasp_pose = pose(z=0.162)
        node.pregrasp_pose = pose(z=0.217)
        node.lift_pose = pose(z=0.217)
        return node

    def test_tracking_selector_falls_back_after_first_ik_failure(self):
        node = self.make_node()
        candidates = [
            {
                "label": "NOMINAL",
                "clearance": 0.055,
                "orientation_error": 0.0,
                "pregrasp_pose": pose(z=0.217),
                "grasp_pose": pose(z=0.162),
                "lift_pose": pose(z=0.217),
            },
            {
                "label": "RECORDED_NEAR_VERTICAL",
                "clearance": 0.050,
                "orientation_error": 0.1,
                "pregrasp_pose": pose(x=0.39, z=0.212),
                "grasp_pose": pose(x=0.39, z=0.157),
                "lift_pose": pose(x=0.39, z=0.212),
            },
        ]
        node.grasp_pose_candidates_from_poses = Mock(return_value=candidates)
        node.compute_ik_for_pose = Mock(
            side_effect=[
                RuntimeError("nominal IK failed"),
                (RobotState(), [0.1] * 6),
            ]
        )
        node.plan_to_pregrasp = Mock(return_value=types.SimpleNamespace())
        node.validate_trajectory = Mock(return_value=(1.0, 0.01, 0.1))

        selected = node.select_tracking_pregrasp_candidate(
            (pose(z=0.217), pose(z=0.162), pose(z=0.217)),
            RobotState(),
            [0.0] * 8,
        )

        self.assertEqual(selected["label"], "RECORDED_NEAR_VERTICAL")
        self.assertEqual(node.compute_ik_for_pose.call_count, 2)
        node.plan_to_pregrasp.assert_called_once()

    def test_tracking_selector_falls_back_after_first_plan_failure(self):
        node = self.make_node()
        candidates = [
            {
                "label": "FIRST",
                "clearance": 0.055,
                "orientation_error": 0.0,
                "pregrasp_pose": pose(z=0.217),
                "grasp_pose": pose(z=0.162),
                "lift_pose": pose(z=0.217),
            },
            {
                "label": "SECOND",
                "clearance": 0.050,
                "orientation_error": 0.1,
                "pregrasp_pose": pose(x=0.39, z=0.212),
                "grasp_pose": pose(x=0.39, z=0.157),
                "lift_pose": pose(x=0.39, z=0.212),
            },
        ]
        node.grasp_pose_candidates_from_poses = Mock(return_value=candidates)
        node.compute_ik_for_pose = Mock(
            return_value=(RobotState(), [0.1] * 6)
        )
        node.plan_to_pregrasp = Mock(
            side_effect=[RuntimeError("first path failed"), types.SimpleNamespace()]
        )
        node.validate_trajectory = Mock(return_value=(1.0, 0.01, 0.1))

        selected = node.select_tracking_pregrasp_candidate(
            (pose(z=0.217), pose(z=0.162), pose(z=0.217)),
            RobotState(),
            [0.0] * 8,
        )

        self.assertEqual(selected["label"], "SECOND")
        self.assertEqual(node.plan_to_pregrasp.call_count, 2)
        self.assertEqual(selected["plan_attempts"], 2)

    def test_tracking_snapshot_requires_one_preview_timestamp(self):
        node = self.make_node()
        node.pregrasp_received_at = 1.0
        node.grasp_received_at = 1.0
        node.lift_received_at = 1.0
        node.pregrasp_pose.header.stamp.sec = 4
        node.grasp_pose.header.stamp.sec = 4
        node.lift_pose.header.stamp.sec = 5
        self.assertIsNone(node.tracking_pose_snapshot())
        node.lift_pose.header.stamp.sec = 4
        self.assertIsNotNone(node.tracking_pose_snapshot())

    def test_pose_triplet_wrapper_preserves_formal_candidate_order(self):
        node = self.make_node()
        explicit = node.grasp_pose_candidates_from_poses(
            node.pregrasp_pose,
            node.grasp_pose,
            node.lift_pose,
        )
        wrapped = node.grasp_pose_candidates()
        self.assertEqual([item["label"] for item in explicit], [item["label"] for item in wrapped])

    def test_observation_drift_ignores_tilted_execution_pose(self):
        node = self.make_node()
        observation = pose(x=0.4, y=0.0, z=0.217)
        execution = pose(x=0.4, y=0.0, z=0.180)

        self.assertLess(
            node.tracking_observation_drift(observation, observation),
            1e-9,
        )
        self.assertGreater(
            node.pose_distance(observation, execution),
            0.03,
        )

    def test_tracking_failure_is_reported_as_planning(self):
        source = Path(__file__).parents[1] / "foam_grasp" / "foam_cube_grasp_sequence.py"
        text = source.read_text(encoding="utf-8")
        self.assertRegex(
            text,
            re.compile(
                r'_task_phase\(\s+"planning",\s+node\.follow_tracking_target'
            ),
        )


if __name__ == "__main__":
    unittest.main()
