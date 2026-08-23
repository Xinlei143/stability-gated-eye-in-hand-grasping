import types
import unittest
import xml.etree.ElementTree as ET

from foam_grasp_sim.contact_diagnostics import extract_contact_rows
from foam_grasp_sim.contact_diagnostics_node import CONTACT_FIELDS
from foam_grasp_sim.contact_qualification import summarize_contact_rows
from foam_grasp_sim.physics_description import inject_pid_parameters


def _vector(x=0.0, y=0.0, z=0.0):
    return types.SimpleNamespace(x=x, y=y, z=z)


class ContactDiagnosticsTest(unittest.TestCase):
    def test_contact_csv_schema_reserves_joint_effort_columns(self):
        self.assertIn("joint7_effort_N", CONTACT_FIELDS)
        self.assertIn("joint8_effort_N", CONTACT_FIELDS)

    def test_pid_parameter_is_injected_into_existing_gazebo_control_plugin(self):
        source = (
            '<robot><gazebo><plugin filename="libgazebo_ros2_control.so">'
            '<parameters>upstream.yaml</parameters>'
            '</plugin></gazebo></robot>'
        )

        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")

        self.assertIn("<parameters>upstream.yaml</parameters>", rendered)
        self.assertIn("<parameters>/tmp/pid.yaml</parameters>", rendered)

    def test_rendered_physics_description_exports_finger_effort_states(self):
        source = (
            '<robot><ros2_control name="GazeboSystem" type="system">'
            '<hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>'
            '<joint name="joint1"><command_interface name="position"/>'
            '<state_interface name="position"/></joint>'
            + "".join(
                f'<joint name="joint{i}"><command_interface name="position"/></joint>'
                for i in range(2, 7)
            )
            +
            '<joint name="joint7"><command_interface name="position"/>'
            '<state_interface name="position"/></joint>'
            '<joint name="joint8"><command_interface name="position"/>'
            '<state_interface name="position"/></joint>'
            '</ros2_control><gazebo><plugin '
            'filename="libgazebo_ros2_control.so"><parameters>upstream.yaml</parameters>'
            '</plugin></gazebo></robot>'
        )

        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")

        self.assertEqual(rendered.count('<state_interface name="effort" />'), 2)

    def test_rendered_physics_description_splits_arm_and_gripper_control_systems(self):
        source = (
            '<robot><ros2_control name="GazeboSystem" type="system">'
            '<hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>'
            '<joint name="joint1"><command_interface name="position"/>'
            '<state_interface name="position"/></joint>'
            '<joint name="joint2"><command_interface name="position"/></joint>'
            '<joint name="joint3"><command_interface name="position"/></joint>'
            '<joint name="joint4"><command_interface name="position"/></joint>'
            '<joint name="joint5"><command_interface name="position"/></joint>'
            '<joint name="joint6"><command_interface name="position"/></joint>'
            '<joint name="joint7"><command_interface name="position"/></joint>'
            '<joint name="joint8"><command_interface name="position"/></joint>'
            '</ros2_control><gazebo><plugin '
            'filename="libgazebo_ros2_control.so"><parameters>upstream.yaml</parameters>'
            '</plugin></gazebo></robot>'
        )

        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")
        root = ET.fromstring(rendered)
        systems = {system.attrib["name"]: system for system in root.findall("ros2_control")}

        self.assertEqual(set(systems), {"PiperArmSystem", "PiperGripperSystem"})
        arm = systems["PiperArmSystem"]
        gripper = systems["PiperGripperSystem"]
        self.assertEqual(
            {joint.attrib["name"] for joint in arm.findall("joint")},
            {f"joint{i}" for i in range(1, 7)},
        )
        self.assertEqual(
            {joint.attrib["name"] for joint in gripper.findall("joint")},
            {"joint7", "joint8"},
        )
        self.assertEqual(
            {command.attrib["name"] for command in arm.findall("joint/command_interface")},
            {"position"},
        )
        self.assertEqual(
            {command.attrib["name"] for command in gripper.findall("joint/command_interface")},
            {"position_pid"},
        )
        self.assertFalse(arm.findall("joint/state_interface[@name='effort']"))
        self.assertEqual(len(gripper.findall("joint/state_interface[@name='effort']")), 2)

    def test_rendered_physics_description_rejects_incomplete_control_system(self):
        source = (
            '<robot><ros2_control name="GazeboSystem" type="system">'
            '<hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>'
            '<joint name="joint1"><command_interface name="position"/></joint>'
            '</ros2_control><gazebo><plugin '
            'filename="libgazebo_ros2_control.so"/></gazebo></robot>'
        )

        with self.assertRaises(ValueError):
            inject_pid_parameters(source, "/tmp/pid.yaml")

    def test_pid_parameter_injection_is_idempotent(self):
        source = (
            '<robot><ros2_control name="PiperArmSystem" type="system">'
            '<hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>'
            + "".join(
                f'<joint name="joint{i}"><command_interface name="position"/></joint>'
                for i in range(1, 7)
            )
            + '</ros2_control><ros2_control name="PiperGripperSystem" type="system">'
            '<hardware><plugin>gazebo_ros2_control/GazeboSystem</plugin></hardware>'
            '<joint name="joint7"><command_interface name="position_pid"/></joint>'
            '<joint name="joint8"><command_interface name="position_pid"/></joint>'
            '</ros2_control><gazebo><plugin filename="libgazebo_ros2_control.so">'
            '<parameters>/tmp/pid.yaml</parameters></plugin></gazebo></robot>'
        )
        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")
        rendered_again = inject_pid_parameters(rendered, "/tmp/pid.yaml")
        self.assertEqual(rendered_again.count("/tmp/pid.yaml"), 1)

    def test_extracts_normal_and_tangential_force_for_target_contact(self):
        state = types.SimpleNamespace(
            collision1_name="piper::link7_collision",
            collision2_name="foam_cube::cube_collision",
            wrenches=[
                types.SimpleNamespace(
                    force=_vector(3.0, 4.0, 0.0),
                    torque=_vector(0.0, 0.0, 1.0),
                )
            ],
            total_wrench=types.SimpleNamespace(
                force=_vector(3.0, 4.0, 0.0),
                torque=_vector(0.0, 0.0, 1.0),
            ),
            contact_positions=[_vector(0.1, 0.2, 0.3)],
            contact_normals=[_vector(1.0, 0.0, 0.0)],
            depths=[0.002],
        )
        message = types.SimpleNamespace(states=[state])

        rows = extract_contact_rows(message, "left", "foam_cube")

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["side"], "left")
        self.assertEqual(rows[0]["collision1"], "piper::link7_collision")
        self.assertAlmostEqual(rows[0]["normal_force_N"], 3.0)
        self.assertAlmostEqual(rows[0]["tangential_force_N"], 4.0)
        self.assertAlmostEqual(rows[0]["depth_m"], 0.002)

    def test_ignores_contacts_that_do_not_contain_target_entity(self):
        state = types.SimpleNamespace(
            collision1_name="piper::link7_collision",
            collision2_name="table::collision",
            wrenches=[],
            total_wrench=types.SimpleNamespace(
                force=_vector(1.0, 0.0, 0.0), torque=_vector()
            ),
            contact_positions=[],
            contact_normals=[],
            depths=[],
        )

        self.assertEqual(
            extract_contact_rows(types.SimpleNamespace(states=[state]), "left", "foam_cube"),
            [],
        )

    def test_contact_summary_requires_both_fingers_and_sustained_force(self):
        rows = []
        for stamp in (0, 500_000_000, 1_000_000_000):
            for side in ("left", "right"):
                rows.append({
                    "sim_time_ns": stamp,
                    "stage": "GRIPPER_SETTLE_STARTED",
                    "side": side,
                    "normal_force_N": 0.8,
                })
        summary = summarize_contact_rows(rows, hold_s=1.0)
        self.assertTrue(summary["passed"])
        self.assertEqual(set(summary["sides"]), {"left", "right"})
        self.assertAlmostEqual(summary["per_side"]["left"]["median_normal_force_N"], 0.8)

    def test_contact_summary_rejects_missing_finger(self):
        rows = [
            {
                "sim_time_ns": stamp,
                "stage": "GRIPPER_SETTLE_STARTED",
                "side": "left",
                "normal_force_N": 0.8,
            }
            for stamp in (0, 500_000_000, 1_000_000_000)
        ]
        summary = summarize_contact_rows(rows, hold_s=1.0)
        self.assertFalse(summary["passed"])
        self.assertIn("missing_side", summary["failure_reasons"])


if __name__ == "__main__":
    unittest.main()
