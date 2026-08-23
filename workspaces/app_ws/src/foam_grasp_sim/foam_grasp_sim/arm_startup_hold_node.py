"""Send a short startup trajectory so the simulated arm cannot fall before planning."""

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]


class ArmStartupHold(Node):
    def __init__(self):
        super().__init__("foam_arm_startup_hold")
        self.declare_parameter(
            "joint_positions",
            # Empirical gravity-equilibrium pose of the pinned Piper model
            # under the local position-PID overlay. This is simulation-only;
            # MoveIt still plans from the live feedback after the hold.
            [0.0, 0.0, 0.0, 0.04, 0.467, 1.50],
        )
        self.declare_parameter("settle_duration_s", 0.75)
        self.declare_parameter("hold_duration_s", 120.0)
        self.declare_parameter(
            "action_name",
            "/arm_controller/follow_joint_trajectory",
        )
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
        )

    def send_hold(self):
        positions = [
            float(value)
            for value in self.get_parameter("joint_positions").value
        ]
        if len(positions) != len(ARM_JOINTS):
            raise RuntimeError("joint_positions must contain six arm values")
        settle_duration = float(
            self.get_parameter("settle_duration_s").value
        )
        hold_duration = float(self.get_parameter("hold_duration_s").value)
        if settle_duration <= 0.0 or hold_duration <= settle_duration:
            raise RuntimeError(
                "hold_duration_s must exceed positive settle_duration_s"
            )
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("arm trajectory action server unavailable")

        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(settle_duration)
        point.time_from_start.nanosec = int(
            (settle_duration - int(settle_duration)) * 1e9
        )
        hold_point = JointTrajectoryPoint()
        hold_point.positions = positions
        hold_point.time_from_start.sec = int(hold_duration)
        hold_point.time_from_start.nanosec = int(
            (hold_duration - int(hold_duration)) * 1e9
        )
        trajectory.points = [point, hold_point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=5.0)
        if not goal_future.done() or goal_future.result() is None:
            raise RuntimeError("arm startup hold goal timed out")
        handle = goal_future.result()
        if not handle.accepted:
            raise RuntimeError("arm startup hold goal rejected")
        self.get_logger().info(
            "Arm startup hold accepted; startup target remains commanded until "
            "the grasp sequence preempts it"
        )


def main():
    rclpy.init()
    node = ArmStartupHold()
    exit_code = 0
    try:
        node.send_hold()
        rclpy.spin(node)
    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
