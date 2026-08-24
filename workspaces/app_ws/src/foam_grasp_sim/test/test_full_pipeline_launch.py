import unittest
from pathlib import Path


class FullPipelineLaunchTest(unittest.TestCase):
    def test_composes_sim_bringup_and_real_rgbd_nodes(self):
        root = Path(__file__).parents[1]
        source = (root / "launch" / "full_pipeline.launch.py").read_text(encoding="utf-8")
        self.assertIn("sim_bringup.launch.py", source)
        self.assertIn('"perception_source": "rgbd"', source)
        self.assertIn('executable="segmentation_node"', source)
        self.assertIn('executable="depth_fusion_node"', source)
        self.assertIn('executable="camera_to_base_node"', source)
        self.assertIn('"transform_source": "tf"', source)
        self.assertIn("piper_eye_in_hand_physics.xacro", source)
        self.assertIn('"prepare_observation_pose": LaunchConfiguration("prepare_observation_pose")', source)
        self.assertIn('DeclareLaunchArgument("prepare_observation_pose", default_value="true")', source)
        self.assertIn('"start_moveit": LaunchConfiguration("start_moveit")', source)
        self.assertIn('DeclareLaunchArgument("start_moveit", default_value="true")', source)
        self.assertNotIn('executable="move_to_observe"', source)
        self.assertNotIn("TimerAction", source)

    def test_supervises_each_rgbd_infrastructure_process(self):
        source = (Path(__file__).parents[1] / "launch" / "full_pipeline.launch.py").read_text(encoding="utf-8")
        self.assertIn("RegisterEventHandler", source)
        self.assertIn("OnProcessExit", source)
        self.assertIn("target_action=segmentation", source)
        self.assertIn("target_action=depth_fusion", source)
        self.assertIn("target_action=camera_to_base", source)
        self.assertIn("infrastructure failure", source)

    def test_moveit_simulation_does_not_enable_unavailable_kinect_updaters(self):
        source = (Path(__file__).parents[1] / "launch" / "sim_moveit.launch.py").read_text(encoding="utf-8")
        self.assertIn("sensors_3d", source)
        self.assertIn("replace", source)

    def test_pipeline_does_not_duplicate_simulation_nodes(self):
        source = (Path(__file__).parents[1] / "launch" / "full_pipeline.launch.py").read_text(encoding="utf-8")
        self.assertNotIn("target_motion_node", source)
        self.assertNotIn("gazebo_ros", source)
        self.assertNotIn("method_policy_node", source)


if __name__ == "__main__":
    unittest.main()
