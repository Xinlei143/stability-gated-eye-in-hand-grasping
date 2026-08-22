"""Standard ``ros2_control`` FollowJointTrajectory backend."""

import copy
import time

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .base_backend import ExecutionBackend, ExecutionResult


class Ros2ControlBackend(ExecutionBackend):
    """Execute arm and gripper trajectories through standard actions.

    The action names are parameters so the backend can work with the pinned
    Piper Gazebo package once its controller launch is assembled in stage 2.
    """

    name = "simulation"
    feedback_topic = "/joint_states"
    requires_piper_status = False
    is_simulation = True

    def __init__(self, node):
        super().__init__(node)
        self.arm_action_name = str(
            node.declare_or_get_parameter(
                "arm_trajectory_action",
                "/arm_controller/follow_joint_trajectory",
            )
        )
        self.gripper_action_name = str(
            node.declare_or_get_parameter(
                "gripper_trajectory_action",
                "/gripper_controller/follow_joint_trajectory",
            )
        )
        self.action_timeout = float(
            node.declare_or_get_parameter("action_server_timeout", 5.0)
        )
        self.final_tolerance = float(
            node.declare_or_get_parameter("final_joint_tolerance", 0.05)
        )
        self.gripper_tolerance = float(
            node.declare_or_get_parameter("gripper_tolerance", 0.004)
        )
        self.gripper_joint_name = str(
            node.declare_or_get_parameter("gripper_joint_name", "gripper")
        )
        self.gripper_command_scale = float(
            node.declare_or_get_parameter("gripper_command_scale", 1.0)
        )
        self.gripper_feedback_scale = float(
            node.declare_or_get_parameter("gripper_feedback_scale", 1.0)
        )
        self.gripper_feedback_topic = str(
            node.declare_or_get_parameter(
                "gripper_feedback_topic",
                "/gripper_controller/controller_state",
            )
        )
        self.arm_client = ActionClient(
            node, FollowJointTrajectory, self.arm_action_name
        )
        self.gripper_client = ActionClient(
            node, FollowJointTrajectory, self.gripper_action_name
        )
        self._gripper_feedback_position = None
        self._gripper_feedback_effort = None
        self.gripper_feedback_subscription = node.create_subscription(
            JointTrajectoryControllerState,
            self.gripper_feedback_topic,
            self._gripper_feedback_callback,
            10,
        )
        self._active_goals = []

    def _gripper_feedback_callback(self, message):
        try:
            index = message.joint_names.index(self.gripper_joint_name)
        except ValueError:
            return
        if index >= len(message.actual.positions):
            return
        position = float(message.actual.positions[index])
        self._gripper_feedback_position = position * self.gripper_feedback_scale
        if index < len(message.actual.effort):
            self._gripper_feedback_effort = float(message.actual.effort[index])

    def normalize_joint_positions(self, message):
        positions = dict(zip(message.name, message.position))
        gripper_from_message = "gripper" in positions
        # Gazebo/Piper revisions use either ``gripper`` or ``joint7`` for the
        # actuated jaw.  Normalize both at the backend boundary.
        if "gripper" not in positions and "joint7" in positions:
            positions["gripper"] = positions["joint7"]
            gripper_from_message = True
        if "gripper" not in positions and "finger_joint" in positions:
            positions["gripper"] = positions["finger_joint"]
            gripper_from_message = True
        cached_gripper = getattr(self, "_gripper_feedback_position", None)
        if "gripper" not in positions and cached_gripper is not None:
            positions["gripper"] = cached_gripper
        required = list(self.node.arm_joint_names) + ["gripper"]
        if not all(name in positions for name in required):
            return None
        values = [float(positions[name]) for name in required]
        if not all(self.node.is_finite(value) for value in values):
            return None
        if gripper_from_message:
            values[6] *= getattr(self, "gripper_feedback_scale", 1.0)
        return values

    def ensure_command_path_is_exclusive(self):
        # An action server is the ownership boundary for simulation; unlike
        # the real Piper topic, a publisher-count check is not meaningful.
        self._wait_for_server(self.arm_client, self.arm_action_name)
        self._wait_for_server(self.gripper_client, self.gripper_action_name)

    def prepare_execution(self):
        self.ensure_command_path_is_exclusive()
        self._prepared = True

    def make_command(self, arm_positions, actual_gripper_m, speed_percent, effort):
        del speed_percent, effort
        return list(arm_positions), float(actual_gripper_m)

    def hold_position(self, actual_gripper_m, speed_percent, effort):
        del speed_percent, effort
        if not self._prepared or self.node.latest_joint_state is None:
            return
        arm = self.node.current_positions()[:6]
        try:
            self._execute(
                self._single_point_trajectory(
                    self.node.arm_joint_names, arm, 0.15
                ),
                self.arm_client,
                "hold arm",
            )
            self._execute(
                self._single_point_trajectory(
                    [self.gripper_joint_name],
                    [actual_gripper_m * self.gripper_command_scale],
                    0.15,
                ),
                self.gripper_client,
                "hold gripper",
            )
        except RuntimeError as error:
            self.node.get_logger().error(f"simulation hold failed: {error}")

    def execute_arm_trajectory(
        self,
        motion_response,
        planned_start,
        actual_gripper_m,
        slowdown,
        speed_percent,
        effort,
        tracking_limit,
        **kwargs,
    ):
        del planned_start, actual_gripper_m, slowdown, speed_percent, effort
        del tracking_limit, kwargs
        trajectory = copy.deepcopy(motion_response.trajectory.joint_trajectory)
        return self._execute(trajectory, self.arm_client, "arm trajectory")

    def execute_cartesian_trajectory(
        self,
        name,
        trajectory,
        gripper_target_m,
        max_joint_rate,
        speed_percent,
        effort,
        tracking_limit,
    ):
        del gripper_target_m, max_joint_rate, speed_percent, effort, tracking_limit
        return self._execute(copy.deepcopy(trajectory), self.arm_client, name)

    def command_gripper(self, target_actual_m, args):
        del args
        result = self._execute(
            self._single_point_trajectory(
                [self.gripper_joint_name],
                [float(target_actual_m) * self.gripper_command_scale],
                1.0,
            ),
            self.gripper_client,
            "gripper",
            check_final_error=False,
        )
        self.node.spin_for(0.2)
        feedback = self.node.current_positions()[6]
        return feedback if result.gripper_position is None else result.gripper_position

    def send_servo_command(
        self, arm_positions, gripper_opening, speed_percent, effort
    ):
        del speed_percent, effort
        if not self._prepared:
            raise RuntimeError("simulation backend is not prepared")
        # Servo-follow runs at 8--20 Hz.  Submit a short rolling goal without
        # waiting for completion so the tracking loop remains responsive.
        if self._active_goals:
            previous = self._active_goals[-1]
            try:
                previous.cancel_goal_async()
            except Exception:
                pass
            self._active = False
        self._wait_for_server(self.arm_client, self.arm_action_name)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._single_point_trajectory(
            self.node.arm_joint_names, arm_positions, 0.12
        )
        future = self.arm_client.send_goal_async(goal)
        goal_handle = self._spin_until(future, self.action_timeout, "servo")
        if goal_handle.accepted:
            self._active_goals = [goal_handle]
            self._active = True
        # The gripper is held by the controller and does not need a new goal
        # on every arm servo tick.

    def publish_message(self, message):
        del message
        raise RuntimeError("simulation backend does not publish JointState commands")

    def close(self):
        for goal_handle in list(self._active_goals):
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._active_goals.clear()

    def _wait_for_server(self, client, name):
        if not client.wait_for_server(timeout_sec=self.action_timeout):
            raise RuntimeError(f"action server unavailable: {name}")

    @staticmethod
    def _single_point_trajectory(joint_names, positions, duration_sec):
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int(
            (duration_sec - int(duration_sec)) * 1e9
        )
        trajectory.points = [point]
        return trajectory

    def _execute(self, trajectory, client, label, *, check_final_error=True):
        if not self._prepared:
            raise RuntimeError("simulation backend is not prepared")
        if not trajectory.points:
            raise RuntimeError(f"{label} has no trajectory points")
        action_name = (
            self.arm_action_name
            if client is self.arm_client
            else self.gripper_action_name
        )
        self._wait_for_server(client, action_name)
        self._begin_execution()
        goal_handle = None
        start = time.monotonic()
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            goal_future = client.send_goal_async(goal)
            goal_handle = self._spin_until(goal_future, self.action_timeout, label)
            if not goal_handle.accepted:
                raise RuntimeError(f"{label} goal rejected")
            self._active_goals.append(goal_handle)
            wrapped = self._spin_until(
                goal_handle.get_result_async(),
                max(self.action_timeout, self._trajectory_duration(trajectory) + 10.0),
                label,
            )
            result = wrapped.result
            if int(result.error_code) != 0:
                raise RuntimeError(
                    f"{label} failed with FollowJointTrajectory error "
                    f"{int(result.error_code)}"
                )
            final_error = self._final_error(trajectory)
            if check_final_error:
                tolerance = (
                    self.gripper_tolerance
                    if client is self.gripper_client
                    else self.final_tolerance
                )
                if final_error > tolerance:
                    raise RuntimeError(
                        f"{label} final joint error {final_error:.4f} exceeds "
                        f"{tolerance:.4f}"
                    )
            return ExecutionResult(
                duration_sec=time.monotonic() - start,
                final_error=final_error,
                maximum_tracking_error=final_error,
                gripper_position=(
                    self.node.current_positions()[6]
                    if len(trajectory.joint_names) == 1
                    else None
                ),
            )
        except RuntimeError:
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
            raise
        finally:
            if goal_handle is not None:
                try:
                    self._active_goals.remove(goal_handle)
                except ValueError:
                    pass
            self._end_execution()

    def _spin_until(self, future, timeout_sec, label):
        deadline = time.monotonic() + float(timeout_sec)
        while not future.done() and time.monotonic() < deadline:
            self.node.spin_for(0.02)
        if not future.done():
            raise RuntimeError(f"{label} timed out")
        if future.exception() is not None:
            raise RuntimeError(f"{label}: {future.exception()}")
        return future.result()

    @staticmethod
    def _trajectory_duration(trajectory):
        if not trajectory.points:
            return 0.0
        point = trajectory.points[-1].time_from_start
        return float(point.sec) + float(point.nanosec) * 1e-9

    def _final_error(self, trajectory):
        actual = self.node.current_positions()
        positions = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        errors = []
        for index, name in enumerate(self.node.arm_joint_names):
            if name in positions:
                errors.append(abs(actual[index] - float(positions[name])))
        if self.gripper_joint_name in positions:
            command_position = float(positions[self.gripper_joint_name])
            # ``trajectory`` already contains the controller-space command
            # after ``gripper_command_scale`` was applied.  Convert that
            # command to the normalized opening exactly once; dividing by the
            # command scale again compares against a value four times too
            # large for Piper's 0.5/2.0 jaw mapping.
            feedback_position = command_position * self.gripper_feedback_scale
            errors.append(abs(actual[6] - feedback_position))
        return max(errors, default=0.0)

    def close(self):
        for goal_handle in list(self._active_goals):
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._active_goals.clear()
        self._active = False
        self._prepared = False
