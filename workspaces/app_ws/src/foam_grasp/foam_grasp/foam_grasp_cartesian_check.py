#!/usr/bin/env python3
"""Check vertical cube approach/lift paths without executing the Piper arm.

The node calls MoveIt's IK, planning-scene, and Cartesian-path services only.
It never creates an execution action client and never publishes to a robot or
joint command topic.  The resulting paths are published only for RViz display.
"""

from collections import OrderedDict
import copy
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject,
    DisplayTrajectory,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPositionIK,
)
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

TABLE_SIZE = (0.60, 0.70, 0.05)
TABLE_CENTER = (0.40, 0.00, -0.024)


class FoamGraspCartesianCheck(Node):
    def __init__(self):
        super().__init__("foam_grasp_cartesian_check")

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
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self.display_publisher = self.create_publisher(
            DisplayTrajectory,
            "/display_planned_path",
            10,
        )

        self.get_logger().warning(
            "CARTESIAN CHECK ONLY: trajectory execution is not implemented"
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
            ("/compute_cartesian_path", self.cartesian_client),
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
        state.is_diff = True
        state.joint_state.header = copy.deepcopy(self.latest_joint_state.header)
        state.joint_state.name = list(ARM_JOINTS)
        state.joint_state.position = [
            float(positions_by_name[joint]) for joint in ARM_JOINTS
        ]
        return state

    def apply_table(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "foam_work_surface"
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
        self.get_logger().info("Table collision object applied")

    def compute_pregrasp_ik(self, pose, seed_state):
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
            raise RuntimeError(f"PREGRASP IK failed with error code {code}")
        self.get_logger().info("PREGRASP IK succeeded")
        return response.solution

    @staticmethod
    def vertical_snapshot(targets):
        snapshot = {
            name: copy.deepcopy(pose)
            for name, pose in targets.items()
        }
        reference = snapshot["PREGRASP"]
        x = float(reference.pose.position.x)
        y = float(reference.pose.position.y)
        orientation = copy.deepcopy(reference.pose.orientation)

        for pose in snapshot.values():
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = copy.deepcopy(orientation)

        return snapshot

    def compute_cartesian(self, name, start_state, target_pose):
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_link"
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = copy.deepcopy(start_state)
        request.start_state.is_diff = True
        request.group_name = "arm"
        request.link_name = "link6"
        request.waypoints.append(copy.deepcopy(target_pose.pose))
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.01
        request.revolute_jump_threshold = 0.15
        request.avoid_collisions = True

        response = self.call_service(
            self.cartesian_client,
            request,
            timeout_seconds=8.0,
        )
        code = int(response.error_code.val)
        fraction = float(response.fraction)
        if code != MoveItErrorCodes.SUCCESS or fraction < 0.999:
            raise RuntimeError(
                f"{name} incomplete: fraction={fraction:.3f}, "
                f"error_code={code}"
            )

        points = response.solution.joint_trajectory.points
        if not points:
            raise RuntimeError(f"{name} returned an empty trajectory")

        max_joint_step = 0.0
        for previous, current in zip(points, points[1:]):
            for before, after in zip(previous.positions, current.positions):
                max_joint_step = max(
                    max_joint_step,
                    abs(float(after) - float(before)),
                )

        if max_joint_step > 0.15 + 1e-9:
            raise RuntimeError(
                f"{name} joint jump too large: {max_joint_step:.4f} rad"
            )

        self.get_logger().info(
            f"{name}: CARTESIAN SUCCESS; fraction={fraction:.3f}, "
            f"points={len(points)}, max_joint_step={max_joint_step:.4f}rad"
        )
        return response

    @staticmethod
    def trajectory_end_state(cartesian_response):
        trajectory = cartesian_response.solution.joint_trajectory
        last_point = trajectory.points[-1]
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(trajectory.joint_names)
        state.joint_state.position = list(last_point.positions)
        return state

    def publish_display(self, start_state, responses):
        display = DisplayTrajectory()
        display.model_id = "piper"
        display.trajectory_start = copy.deepcopy(start_state)
        display.trajectory = [
            copy.deepcopy(response.solution)
            for response in responses
        ]
        for _ in range(5):
            self.display_publisher.publish(display)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            "Published Cartesian approach/lift paths for RViz display"
        )

    def run(self):
        if self.get_publishers_info_by_topic("/joint_states"):
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
        snapshot = self.vertical_snapshot(self.targets)
        current_state = self.current_robot_state()
        pregrasp_state = self.compute_pregrasp_ik(
            snapshot["PREGRASP"],
            current_state,
        )

        approach = self.compute_cartesian(
            "PREGRASP_TO_GRASP",
            pregrasp_state,
            snapshot["GRASP"],
        )
        grasp_state = self.trajectory_end_state(approach)
        lift = self.compute_cartesian(
            "GRASP_TO_LIFT",
            grasp_state,
            snapshot["LIFT"],
        )

        self.publish_display(pregrasp_state, [approach, lift])

        descent = abs(
            float(snapshot["PREGRASP"].pose.position.z)
            - float(snapshot["GRASP"].pose.position.z)
        )
        ascent = abs(
            float(snapshot["LIFT"].pose.position.z)
            - float(snapshot["GRASP"].pose.position.z)
        )
        print("\n===== CARTESIAN PLAN-ONLY SUMMARY =====")
        print("TABLE COLLISION OBJECT: APPLIED")
        print(
            f"PREGRASP_TO_GRASP: 100% VERTICAL ({descent:.3f} m)"
        )
        print(f"GRASP_TO_LIFT: 100% VERTICAL ({ascent:.3f} m)")
        print("No trajectory was sent to the Piper arm.")


def main():
    rclpy.init()
    node = FoamGraspCartesianCheck()
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
