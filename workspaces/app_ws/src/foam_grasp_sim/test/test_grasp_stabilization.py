import unittest

from foam_grasp_sim.grasp_stabilization import resolve_stabilization_mode


class GraspStabilizationTest(unittest.TestCase):
    def test_off_has_no_legacy_assist(self):
        selection = resolve_stabilization_mode("off")
        self.assertEqual(selection.mode, "off")
        self.assertFalse(selection.legacy_assist_enabled)

    def test_legacy_mode_enables_only_legacy_assist(self):
        selection = resolve_stabilization_mode("legacy_contact_confirmed")
        self.assertTrue(selection.legacy_assist_enabled)

    def test_gazebo_grasp_fix_rejects_legacy_assist(self):
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            resolve_stabilization_mode(
                "gazebo_grasp_fix", grasp_assist_mode="contact_confirmed"
            )
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            resolve_stabilization_mode(
                "gazebo_grasp_fix", grasp_assist_service="/gazebo/attach"
            )


if __name__ == "__main__":
    unittest.main()
