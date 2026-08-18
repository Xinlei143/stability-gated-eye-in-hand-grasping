#!/usr/bin/env python3
"""Plan three cube-grasp segments in MoveIt without executing anything.

Safety properties:
- aborts if any publisher exists on Piper's /joint_states command topic;
- adds a table collision box to the planning scene;
- calls only /compute_ik, /apply_planning_scene, and /plan_kinematic_path;
- never creates an action client or publishes a robot/joint command;
- publishes only DisplayTrajectory for visualization in RViz.
"""

from collections import OrderedDict
import copy
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


ARM_JOINTS = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)

TARGET_TOPICS = OrderedDict(
    [
        ("PREGRASP", "/foam_grasp/cube_pregrasp_pose"),
        ("GRASP", "/foam_grasp/cube_grasp_pose"),
        ("LIFT", "/foam_grasp/cube_lift_pose"),
    ]
)

TABLE_ID = "foam_work_surface"
TABLE_SIZE = (0.60, 0.70, 0.05)
TABLE_CENTER = (0.40, 0.00, -0.024)


class FoamGraspPlanCheck(Node):
    def __init__(self):
        super().__init__("foam_grasp_plan_check")

        self.targets = {}
        self.latest_joint_state = None
        self.pose_subscriptions = []

        for name, topic in TARGET_TOPICS.items():
            subscription = self.create_subscription(
                PoseStamped,
                topic,
                lambda message, target_name=name: self.pose_callback(
                    target_name,
                    message,
                ),
                10,
            )
            self.pose_subscriptions.append(subscription)

        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states_single",
            self.joint_state_callback,
            10,
        )

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        self.plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory,
            "/display_planned_path",
            10,
        )

        self.get_logger().warning(
            "PLAN CHECK ONLY: trajectory execution is not implemented"
        )

    def pose_callback(self, name, message):
        self.targets[name] = copy.deepcopy(message)

    def joint_state_callback(self, message):
        self.latest_joint_state = copy.deepcopy(message)

    def wait_for_inputs(self, timeout_seconds=15.0):
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.latest_joint_state is not None
                and all(name in self.targets for name in TARGET_TOPICS)
            ):
                return True
        return False

    def wait_for_services(self, timeout_seconds=10.0):
        services = (
            ("/compute_ik", self.ik_client),
            ("/apply_planning_scene", self.scene_client),
            ("/plan_kinematic_path", self.plan_client),
        )
        for name, client in services:
            if not client.wait_for_service(timeout_sec=timeout_seconds):
                self.get_logger().error(f"Service unavailable: {name}")
                return False
        return True

    def call_service(self, client, request, timeout_seconds):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_seconds,
        )
        if not future.done():
            raise RuntimeError("Service call timed out")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(str(exception))
        return future.result()

    def current_robot_state(self):
        positions_by_name = dict(
            zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            )
        )
        missing = [joint for joint in ARM_JOINTS if joint not in positions_by_name]
        if missing:
            raise RuntimeError(f"Real joint feedback omitted joints: {missing}")

        state = RobotState()
        state.joint_state.header = copy.deepcopy(self.latest_joint_state.header)
        state.joint_state.name = list(ARM_JOINTS)
        state.joint_state.position = [
            float(positions_by_name[joint]) for joint in ARM_JOINTS
        ]
        state.is_diff = True
        return state

    def apply_table(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = TABLE_ID
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(TABLE_SIZE)

        pose = Pose()
        pose.position.x = TABLE_CENTER[0]
        pose.position.y = TABLE_CENTER[1]
        pose.position.z = TABLE_CENTER[2]
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects.append(collision_object)

        response = self.call_service(
            self.scene_client,
            request,
            timeout_seconds=5.0,
        )
        if not response.success:
            raise RuntimeError("MoveIt rejected the table planning scene")

        self.get_logger().info(
            "Added table collision object: "
            f"size={TABLE_SIZE}, center={TABLE_CENTER}"
        )

    def compute_ik(self, name, pose, seed_state):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "link6"
        request.ik_request.pose_stamped = copy.deepcopy(pose)
        request.ik_request.robot_state = copy.deepcopy(seed_state)
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2

        response = self.call_service(
            self.ik_client,
            request,
            timeout_seconds=4.0,
        )
        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{name} IK failed with error code {code}")

        positions_by_name = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        missing = [joint for joint in ARM_JOINTS if joint not in positions_by_name]
        if missing:
            raise RuntimeError(f"{name} IK omitted joints: {missing}")

        formatted = ", ".join(
            f"{joint}={positions_by_name[joint]:.4f}"
            for joint in ARM_JOINTS
        )
        self.get_logger().info(f"{name} IK: {formatted}")
        return response.solution

    @staticmethod
    def joint_goal_constraints(robot_state):
        positions_by_name = dict(
            zip(
                robot_state.joint_state.name,
                robot_state.joint_state.position,
            )
        )
        constraints = Constraints()
        constraints.name = "foam_cube_joint_goal"

        for joint_name in ARM_JOINTS:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(positions_by_name[joint_name])
            joint_constraint.tolerance_above = 0.001
            joint_constraint.tolerance_below = 0.001
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def plan_segment(self, name, start_state, goal_state):
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.workspace_parameters.header.frame_id = "base_link"
        motion_request.workspace_parameters.min_corner.x = -0.20
        motion_request.workspace_parameters.min_corner.y = -0.80
        motion_request.workspace_parameters.min_corner.z = -0.10
        motion_request.workspace_parameters.max_corner.x = 0.80
        motion_request.workspace_parameters.max_corner.y = 0.80
        motion_request.workspace_parameters.max_corner.z = 1.00
        motion_request.start_state = copy.deepcopy(start_state)
        motion_request.start_state.is_diff = True
        motion_request.goal_constraints.append(
            self.joint_goal_constraints(goal_state)
        )
        motion_request.pipeline_id = "ompl"
        motion_request.group_name = "arm"
        motion_request.num_planning_attempts = 5
        motion_request.allowed_planning_time = 5.0
        motion_request.max_velocity_scaling_factor = 0.10
        motion_request.max_acceleration_scaling_factor = 0.10

        response = self.call_service(
            self.plan_client,
            request,
            timeout_seconds=8.0,
        ).motion_plan_response

        code = int(response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{name} planning failed with error code {code}")

        trajectory = response.trajectory
        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            raise RuntimeError(f"{name} returned an empty trajectory")

        last_point = joint_trajectory.points[-1]
        duration = (
            float(last_point.time_from_start.sec)
            + float(last_point.time_from_start.nanosec) * 1e-9
        )
        self.get_logger().info(
            f"{name}: PLAN SUCCESS; points={len(joint_trajectory.points)}, "
            f"planning_time={response.planning_time:.3f}s, "
            f"trajectory_duration={duration:.3f}s"
        )
        return response

    @staticmethod
    def trajectory_end_state(motion_response):
        joint_trajectory = motion_response.trajectory.joint_trajectory
        final_point = joint_trajectory.points[-1]

        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(joint_trajectory.joint_names)
        state.joint_state.position = list(final_point.positions)
        return state

    def publish_display(self, first_start_state, motion_responses):
        display = DisplayTrajectory()
        display.model_id = "piper"
        display.trajectory_start = copy.deepcopy(first_start_state)
        display.trajectory = [
            copy.deepcopy(response.trajectory)
            for response in motion_responses
        ]

        for _ in range(5):
            self.display_publisher.publish(display)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            "Published three planned segments to /display_planned_path"
        )

    def run(self):
        command_publishers = self.get_publishers_info_by_topic("/joint_states")
        if command_publishers:
            raise RuntimeError(
                "ABORTED: /joint_states has one or more publishers"
            )

        if not self.wait_for_services():
            raise RuntimeError("Required MoveIt services are unavailable")

        if not self.wait_for_inputs():
            raise RuntimeError(
                "Timed out waiting for real joint feedback or grasp poses"
            )

        self.apply_table()

        current_state = self.current_robot_state()
        ik_states = OrderedDict()
        seed_state = current_state
        for name in TARGET_TOPICS:
            ik_states[name] = self.compute_ik(
                name,
                self.targets[name],
                seed_state,
            )
            seed_state = ik_states[name]

        responses = []
        start_state = current_state
        segments = (
            ("CURRENT_TO_PREGRASP", "PREGRASP"),
            ("PREGRASP_TO_GRASP", "GRASP"),
            ("GRASP_TO_LIFT", "LIFT"),
        )
        for segment_name, goal_name in segments:
            response = self.plan_segment(
                segment_name,
                start_state,
                ik_states[goal_name],
            )
            responses.append(response)
            start_state = self.trajectory_end_state(response)

        self.publish_display(current_state, responses)

        print("\n===== PLAN-ONLY SUMMARY =====")
        print("TABLE COLLISION OBJECT: APPLIED")
        for segment_name, _ in segments:
            print(f"{segment_name}: PLAN SUCCESS")
        print("No trajectory was sent to the Piper arm.")


def main():
    rclpy.init()
    node = FoamGraspPlanCheck()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
