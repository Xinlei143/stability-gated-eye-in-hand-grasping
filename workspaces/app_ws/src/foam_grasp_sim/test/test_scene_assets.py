import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


class SceneAssetTest(unittest.TestCase):
    def test_simulation_contract_is_parameterized(self):
        config = (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        for value in (
            "execution_backend: simulation",
            "arm_trajectory_action: /arm_controller/follow_joint_trajectory",
            "gripper_trajectory_action: /gripper_controller/follow_joint_trajectory",
            "gripper_joint_name: joint7",
            "final_joint_tolerance: 0.05",
            "gripper_tolerance: 0.004",
        ):
            self.assertIn(value, config)

    def test_bringup_preserves_upstream_launch_and_planning_only_moveit(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        moveit = (PACKAGE_ROOT / "launch" / "sim_moveit.launch.py").read_text()
        self.assertIn("piper_gazebo", bringup)
        self.assertIn("piper_gazebo.launch.py", bringup)
        self.assertIn("allow_trajectory_execution", moveit)
        self.assertIn('"allow_trajectory_execution": False', moveit)
        self.assertNotIn("joint_states_single", moveit)

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

    def test_world_contains_static_table_and_gravity(self):
        root = ET.parse(PACKAGE_ROOT / "worlds" / "grasp_table.world").getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.findtext("gravity"), "0 0 -9.81")
        table = world.find("model[@name='grasp_table']")
        self.assertIsNotNone(table)
        self.assertEqual(table.findtext("static"), "true")


if __name__ == "__main__":
    unittest.main()
