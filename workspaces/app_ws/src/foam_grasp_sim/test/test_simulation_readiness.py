import unittest

from foam_grasp_sim.simulation_readiness import (
    REQUIRED_ACTION_SERVERS,
    REQUIRED_CONTROLLERS,
    REQUIRED_JOINTS,
    ReadinessSnapshot,
    format_missing_conditions,
    missing_conditions,
)


class SimulationReadinessTest(unittest.TestCase):
    def _complete(self):
        return ReadinessSnapshot(
            controller_service_available=True,
            active_controllers=frozenset(REQUIRED_CONTROLLERS),
            joint_names=frozenset(REQUIRED_JOINTS),
            ready_action_servers=frozenset(REQUIRED_ACTION_SERVERS),
        )

    def test_complete_snapshot_has_no_missing_conditions(self):
        self.assertEqual(missing_conditions(self._complete()), ())

    def test_missing_controller_and_joint_and_action_are_named(self):
        snapshot = ReadinessSnapshot(
            controller_service_available=False,
            active_controllers=frozenset({"joint_state_broadcaster"}),
            joint_names=frozenset({"joint1", "joint2"}),
            ready_action_servers=frozenset(),
        )
        missing = missing_conditions(snapshot)
        self.assertIn("/controller_manager/list_controllers unavailable", missing)
        self.assertIn("controller arm_controller not active", missing)
        self.assertIn("joint_states missing joint3", missing)
        self.assertIn(
            "action server /arm_controller/follow_joint_trajectory not ready",
            missing,
        )
        rendered = format_missing_conditions(missing)
        self.assertIn("/controller_manager/list_controllers unavailable", rendered)
        self.assertIn("action server /gripper8_controller/follow_joint_trajectory not ready", rendered)


if __name__ == "__main__":
    unittest.main()
