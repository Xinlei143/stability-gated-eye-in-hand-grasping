import types
import unittest

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
            '<robot><ros2_control><joint name="joint7">'
            '<state_interface name="position"/>'
            '</joint><joint name="joint8">'
            '<state_interface name="position"/>'
            '</joint></ros2_control><plugin '
            'filename="libgazebo_ros2_control.so"><parameters>upstream.yaml</parameters>'
            '</plugin></robot>'
        )

        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")

        self.assertEqual(rendered.count('<state_interface name="effort" />'), 2)

    def test_rendered_physics_description_uses_position_pid_for_controlled_joints(self):
        source = (
            '<robot><ros2_control><joint name="joint1">'
            '<command_interface name="position"/>'
            '</joint><joint name="joint7">'
            '<command_interface name="position"/>'
            '</joint><joint name="joint8">'
            '<command_interface name="position"/>'
            '</joint></ros2_control><plugin '
            'filename="libgazebo_ros2_control.so"><parameters>upstream.yaml</parameters>'
            '</plugin></robot>'
        )

        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")

        self.assertEqual(rendered.count('name="position_pid"'), 3)
        self.assertNotIn('name="position" />', rendered)

    def test_pid_parameter_injection_is_idempotent(self):
        source = (
            '<robot><plugin filename="libgazebo_ros2_control.so">'
            '<parameters>/tmp/pid.yaml</parameters>'
            '</plugin></robot>'
        )
        rendered = inject_pid_parameters(source, "/tmp/pid.yaml")
        self.assertEqual(rendered.count("/tmp/pid.yaml"), 1)

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
