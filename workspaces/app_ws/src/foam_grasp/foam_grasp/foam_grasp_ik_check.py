#!/usr/bin/env python3
"""Check cube grasp preview poses with MoveIt's IK service only.

This node subscribes to PREGRASP, GRASP, and LIFT PoseStamped topics and calls
the read-only /compute_ik service.  It does not create a trajectory, publish a
joint command, call an execution action, or control the Piper arm.
"""

from collections import OrderedDict
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node


TARGET_TOPICS = OrderedDict(
    [
        ("PREGRASP", "/foam_grasp/cube_pregrasp_pose"),
        ("GRASP", "/foam_grasp/cube_grasp_pose"),
        ("LIFT", "/foam_grasp/cube_lift_pose"),
    ]
)


ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


class FoamGraspIKCheck(Node):
    def __init__(self):
        super().__init__("foam_grasp_ik_check")

        self.targets = {}
        self.target_received_at = {}
        self.pose_subscriptions = []
        for name, topic in TARGET_TOPICS.items():
            subscription = self.create_subscription(
                PoseStamped,
                topic,
                lambda message, target_name=name: self.target_callback(
                    target_name,
                    message,
                ),
                10,
            )
            self.pose_subscriptions.append(subscription)

        self.client = self.create_client(GetPositionIK, "/compute_ik")
        self.target_names = list(TARGET_TOPICS.keys())
        self.current_index = 0
        self.pending_future = None
        self.results = OrderedDict()
        self.finished = False
        self.last_wait_log = 0.0
        self.timer = self.create_timer(0.2, self.step)

        self.get_logger().warning(
            "IK CHECK ONLY: no trajectory or robot command can be produced"
        )
        for name, topic in TARGET_TOPICS.items():
            self.get_logger().info(f"Waiting for {name}: {topic}")

    def target_callback(self, name, message):
        self.targets[name] = message
        self.target_received_at[name] = time.monotonic()

    def log_waiting(self, message):
        now = time.monotonic()
        if now - self.last_wait_log >= 5.0:
            self.get_logger().info(message)
            self.last_wait_log = now

    def step(self):
        if self.finished or self.pending_future is not None:
            return

        missing = [name for name in self.target_names if name not in self.targets]
        if missing:
            self.log_waiting("Waiting for pose topics: " + ", ".join(missing))
            return

        if not self.client.service_is_ready():
            self.log_waiting("Waiting for MoveIt service: /compute_ik")
            return

        if self.current_index >= len(self.target_names):
            self.print_summary()
            self.finished = True
            return

        name = self.target_names[self.current_index]
        pose = self.targets[name]

        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "link6"
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2

        position = pose.pose.position
        orientation = pose.pose.orientation
        self.get_logger().info(
            f"Checking {name}: frame={pose.header.frame_id}, "
            f"position=({position.x:.4f}, {position.y:.4f}, "
            f"{position.z:.4f}), quaternion=({orientation.x:.4f}, "
            f"{orientation.y:.4f}, {orientation.z:.4f}, "
            f"{orientation.w:.4f})"
        )

        self.pending_future = self.client.call_async(request)
        self.pending_future.add_done_callback(
            lambda future, target_name=name: self.response_callback(
                target_name,
                future,
            )
        )

    def response_callback(self, name, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"{name}: service call failed: {error}")
            self.results[name] = (False, "SERVICE_CALL_FAILED", [])
        else:
            code = int(response.error_code.val)
            error_name = ERROR_NAMES.get(code, f"ERROR_CODE_{code}")
            success = code == MoveItErrorCodes.SUCCESS

            joint_state = response.solution.joint_state
            positions_by_name = dict(zip(joint_state.name, joint_state.position))
            arm_positions = [
                (joint_name, positions_by_name[joint_name])
                for joint_name in (
                    "joint1",
                    "joint2",
                    "joint3",
                    "joint4",
                    "joint5",
                    "joint6",
                )
                if joint_name in positions_by_name
            ]
            self.results[name] = (success, error_name, arm_positions)

            if success:
                formatted = ", ".join(
                    f"{joint_name}={position:.4f}"
                    for joint_name, position in arm_positions
                )
                self.get_logger().info(
                    f"{name}: IK SUCCESS; {formatted}"
                )
            else:
                self.get_logger().error(
                    f"{name}: IK FAILED; {error_name} ({code})"
                )

        self.pending_future = None
        self.current_index += 1

    def print_summary(self):
        print("\n===== IK CHECK SUMMARY =====")
        for name in self.target_names:
            success, error_name, positions = self.results.get(
                name,
                (False, "NO_RESULT", []),
            )
            status = "REACHABLE" if success else "NOT REACHABLE"
            print(f"{name}: {status} ({error_name})")
            if positions:
                print(
                    "  "
                    + ", ".join(
                        f"{joint_name}={position:.4f}"
                        for joint_name, position in positions
                    )
                )
        print("No trajectory was generated or executed.")


def main():
    rclpy.init()
    node = FoamGraspIKCheck()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
