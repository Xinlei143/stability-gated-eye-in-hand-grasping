import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


class SceneAssetTest(unittest.TestCase):
    def test_simulation_contract_is_parameterized(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        execution = config["execution"]
        self.assertEqual(execution["execution_backend"], "simulation")
        self.assertEqual(
            execution["arm_trajectory_action"],
            "/arm_controller/follow_joint_trajectory",
        )
        self.assertEqual(
            execution["gripper_trajectory_action"],
            "/gripper_controller/follow_joint_trajectory",
        )
        self.assertEqual(execution["gripper_joint_name"], "joint7")
        self.assertEqual(execution["gripper_command_scale"], 0.5)
        self.assertEqual(execution["gripper_feedback_scale"], 2.0)
        self.assertEqual(execution["final_joint_tolerance"], 0.05)
        self.assertEqual(execution["gripper_tolerance"], 0.004)

    def test_scene_config_aligns_table_and_target_bases(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        table = config["scene"]["table"]
        tabletop = table["pose"][2] + table["size"][2] / 2.0
        self.assertAlmostEqual(tabletop, 0.001)
        heights = {"cube": 0.050, "cylinder": 0.070, "sphere": 0.060}
        for name, height in heights.items():
            pose = config["scene"]["targets"][name]["pose"]
            self.assertAlmostEqual(pose[2] - height / 2.0, tabletop)
            self.assertEqual(pose[:2], [0.40, 0.00])

    def test_stage3_motion_and_perception_defaults_preserve_static_ideal_mode(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        self.assertEqual(config["motion"]["trajectory"], "static")
        self.assertEqual(config["motion"]["velocity"], [0.01, 0.00, 0.00])
        self.assertEqual(config["motion"]["seed"], 42)
        self.assertEqual(config["perception"]["source"], "ground_truth")
        self.assertEqual(config["perception"]["seed"], 42)

    def test_stage4_method_defaults_preserve_gated_behavior(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        method = config["method"]
        self.assertEqual(method["name"], "gated")
        self.assertEqual(method["stability_duration"], 5.0)
        self.assertEqual(method["position_spread_threshold"], 0.006)
        self.assertEqual(method["minimum_stable_samples"], 25)

    def test_piper_composition_owns_world_and_reuses_upstream_assets(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        piper = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        moveit = (PACKAGE_ROOT / "launch" / "sim_moveit.launch.py").read_text()
        self.assertIn("piper_sim.launch.py", bringup)
        self.assertNotIn("piper_gazebo.launch.py", bringup)
        self.assertIn("piper_description_gazebo.xacro", piper)
        self.assertIn("piper_gazebo", piper)
        self.assertIn("joint8_ctrl.py", piper)
        self.assertIn("grasp_table.world", piper)
        for controller in (
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "gripper8_controller",
        ):
            self.assertIn(controller, piper)
        self.assertIn("allow_trajectory_execution", moveit)
        self.assertIn('"allow_trajectory_execution": False', moveit)
        self.assertNotIn("joint_states_single", moveit)

    def test_table_model_matches_scene_config(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        root = ET.parse(PACKAGE_ROOT / "models" / "table" / "model.sdf").getroot()
        model_size = [
            float(value)
            for value in root.findtext("model/link/collision/geometry/box/size").split()
        ]
        self.assertEqual(model_size, config["scene"]["table"]["size"])

    def test_all_static_scene_assets_are_valid_sdf(self):
        for path in sorted((PACKAGE_ROOT / "models").glob("*/model.sdf")):
            root = ET.parse(path).getroot()
            self.assertEqual(root.tag, "sdf")
            model = root.find("model")
            self.assertIsNotNone(model, path)
            self.assertIsNotNone(model.find("link/collision"), path)
            self.assertIsNotNone(model.find("link/visual"), path)
            if path.parent.name != "table":
                self.assertIsNotNone(model.find("link/inertial/mass"), path)
                self.assertIsNotNone(model.find("link/inertial/inertia"), path)
                self.assertIsNotNone(
                    model.find("link/collision/surface/friction"), path
                )

    def test_world_contains_only_physics_and_gravity(self):
        root = ET.parse(PACKAGE_ROOT / "worlds" / "grasp_table.world").getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.findtext("gravity"), "0 0 -9.81")
        self.assertEqual(world.findtext("physics/max_step_size"), "0.001")
        self.assertEqual(world.findtext("physics/real_time_update_rate"), "1000")
        self.assertIsNone(world.find("model[@name='grasp_table']"))

    def test_bringup_exposes_stage3_motion_and_perception_pipeline(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        for value in (
            "target_model",
            "run_grasp_pipeline",
            "execute_motion",
            "trajectory",
            "perception_source",
            "target_motion_node",
            "simulated_perception_node",
            "rgbd",
            "target_latch_node",
            "grasp_pose_preview_node",
            "object_grasp_sequence",
            "AUTO_FULL_OBJECT_GRASP",
        ):
            self.assertIn(value, bringup)
        self.assertNotIn("start_executor", bringup)
        self.assertNotIn('executable="executor"', bringup)

    def test_stage4_method_layer_is_wired_before_latch_and_execution(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        setup = (PACKAGE_ROOT / "setup.py").read_text()
        sequence = (
            Path(__file__).parents[2]
            / "foam_grasp"
            / "foam_grasp"
            / "foam_cube_grasp_sequence.py"
        ).read_text()
        for method in ("snapshot", "tracking", "gated"):
            self.assertIn(method, bringup)
        for parameter in (
            "stability_duration",
            "position_spread_threshold",
            "center_error_threshold",
            "joint_error_threshold",
            "minimum_stable_samples",
            "method_ready",
        ):
            self.assertIn(parameter, bringup)
        self.assertIn("method_policy_node", setup)
        self.assertIn("wait_for_method_ready", sequence)
        latch = (
            Path(__file__).parents[2]
            / "foam_grasp"
            / "foam_grasp"
            / "foam_target_latch_node.py"
        ).read_text()
        self.assertIn('self.method == "tracking"', latch)
        self.assertIn("points[-1]", latch)
        self.assertNotIn("requires trajectory:=static", bringup)

    def test_python_nodes_install_into_ros2_lib_directory(self):
        setup_cfg = PACKAGE_ROOT / "setup.cfg"
        self.assertTrue(setup_cfg.exists())
        content = setup_cfg.read_text()
        self.assertIn("script_dir=$base/lib/foam_grasp_sim", content)
        self.assertIn("install_scripts=$base/lib/foam_grasp_sim", content)

    def test_static_source_uses_selected_base_frame_topic(self):
        source = (
            PACKAGE_ROOT / "foam_grasp_sim" / "static_target_source_node.py"
        ).read_text()
        self.assertIn('TARGET_MODELS = ("cube", "cylinder", "sphere")', source)
        self.assertIn('f"/foam_grasp/{self.target_model}_point_base"', source)
        self.assertIn('message.header.frame_id = self.base_frame', source)

    def test_stage3_nodes_preserve_existing_observation_interface(self):
        motion = (
            PACKAGE_ROOT / "foam_grasp_sim" / "target_motion_node.py"
        ).read_text()
        perception = (
            PACKAGE_ROOT / "foam_grasp_sim" / "simulated_perception_node.py"
        ).read_text()
        self.assertIn('"/foam_grasp_sim/target_ground_truth"', motion)
        self.assertIn('f"/foam_grasp/{self.target_model}_point_base"', perception)
        self.assertIn("SetEntityState", motion)
        self.assertIn("ModelStates", motion)


if __name__ == "__main__":
    unittest.main()
