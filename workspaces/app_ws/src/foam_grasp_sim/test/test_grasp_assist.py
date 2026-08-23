import types
import unittest

from foam_grasp_sim.grasp_assist_node import ATTACHMENT_LINK, contact_state_has_entity


class GraspAssistTest(unittest.TestCase):
    def test_attachment_uses_a_link_that_exists_in_gazebo_model(self):
        self.assertEqual(ATTACHMENT_LINK, "link6")

    def test_attachment_service_uses_a_multithreaded_executor(self):
        source = __import__(
            "pathlib"
        ).Path(__file__).parents[1].joinpath(
            "foam_grasp_sim", "grasp_assist_node.py"
        ).read_text()
        self.assertIn("MultiThreadedExecutor", source)
        self.assertIn("threading.Event", source)

    def test_contact_detection_requires_target_entity(self):
        message = types.SimpleNamespace(states=[
            types.SimpleNamespace(
                collision1_name="piper::link7::link7_collision",
                collision2_name="foam_cube::cube_link::collision",
            )
        ])

        self.assertTrue(contact_state_has_entity(message, "foam_cube"))
        self.assertFalse(contact_state_has_entity(message, "foam_sphere"))

    def test_contact_detection_handles_empty_contact_message(self):
        self.assertFalse(contact_state_has_entity(types.SimpleNamespace(states=[]), "foam_cube"))


if __name__ == "__main__":
    unittest.main()
