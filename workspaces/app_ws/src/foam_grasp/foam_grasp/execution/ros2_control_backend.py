"""Standard ``ros2_control`` FollowJointTrajectory backend."""

import copy
import csv
import os
from pathlib import Path
import time

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .base_backend import ExecutionBackend, ExecutionResult
from ..gripper_trace import make_trace_row


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
        self.gripper8_action_name = str(
            node.declare_or_get_parameter(
                "gripper8_trajectory_action",
                "/gripper8_controller/follow_joint_trajectory",
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
        self.gripper8_joint_name = str(
            node.declare_or_get_parameter("gripper8_joint_name", "joint8")
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
        self.gripper8_feedback_topic = str(
            node.declare_or_get_parameter(
                "gripper8_feedback_topic",
                "/gripper8_controller/controller_state",
            )
        )
        self.arm_client = ActionClient(
            node, FollowJointTrajectory, self.arm_action_name
        )
        self.gripper_client = ActionClient(
            node, FollowJointTrajectory, self.gripper_action_name
        )
        self.gripper8_client = ActionClient(
            node, FollowJointTrajectory, self.gripper8_action_name
        )
        self._gripper_feedback_position = None
        self._gripper8_feedback_position = None
        self._gripper_feedback_effort = None
        self._gripper_feedback_stamp_s = None
        self._gripper8_feedback_stamp_s = None
        self._gripper_trace_path = os.environ.get("FOAM_GRIPPER_TRACE_PATH", "")
        self._gripper_trace_duration_s = self._env_float(
            "FOAM_GRIPPER_TRACE_DURATION_S", 1.0
        )
        self._gripper_trace_interval_s = self._env_float(
            "FOAM_GRIPPER_TRACE_INTERVAL_S", 0.05
        )
        self._gripper_trace_started_s = None
        self._gripper_trace_last_sample_s = None
        self._gripper_trace_commands = (None, None)
        self._gripper_trace_rows = []
        self._gripper_trace_pair_index = 0
        self.gripper_feedback_subscription = node.create_subscription(
            JointTrajectoryControllerState,
            self.gripper_feedback_topic,
            lambda message: self._gripper_feedback_callback(
                message, self.gripper_joint_name
            ),
            10,
        )
        self.gripper8_feedback_subscription = node.create_subscription(
            JointTrajectoryControllerState,
            self.gripper8_feedback_topic,
            lambda message: self._gripper_feedback_callback(
                message, self.gripper8_joint_name
            ),
            10,
        )
        self._active_goals = []

    def _gripper_feedback_callback(self, message, joint_name):
        try:
            index = message.joint_names.index(joint_name)
        except ValueError:
            return
        if index >= len(message.actual.positions):
            return
        position = float(message.actual.positions[index])
        if joint_name == self.gripper_joint_name:
            self._gripper_feedback_position = position * self.gripper_feedback_scale
            self._gripper_feedback_stamp_s = self._message_stamp_s(message)
        else:
            self._gripper8_feedback_position = position * self.gripper_feedback_scale
            self._gripper8_feedback_stamp_s = self._message_stamp_s(message)
        if index < len(message.actual.effort):
            self._gripper_feedback_effort = float(message.actual.effort[index])

    @staticmethod
    def _env_float(name, default):
        try:
            value = float(os.environ.get(name, default))
        except (TypeError, ValueError):
            return float(default)
        return value if value > 0.0 else float(default)

    @staticmethod
    def _message_stamp_s(message):
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return None
        try:
            return float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except (AttributeError, TypeError, ValueError):
            return None

    def _begin_gripper_trace(self, trajectories):
        if not self._gripper_trace_path:
            return
        self._gripper_trace_pair_index += 1
        self._gripper_trace_started_s = time.monotonic()
        self._gripper_trace_last_sample_s = None
        self._gripper_trace_commands = tuple(
            float(trajectory.points[-1].positions[0])
            if trajectory.points and trajectory.points[-1].positions
            else None
            for trajectory in trajectories
        )
        self._gripper_trace_rows = []
        self._sample_gripper_trace(force=True)

    def _sample_gripper_trace(self, *, force=False):
        if self._gripper_trace_started_s is None:
            return
        elapsed = time.monotonic() - self._gripper_trace_started_s
        if elapsed > self._gripper_trace_duration_s:
            return
        if (
            not force
            and self._gripper_trace_last_sample_s is not None
            and elapsed - self._gripper_trace_last_sample_s
            < self._gripper_trace_interval_s
        ):
            return
        self._gripper_trace_rows.append(
            make_trace_row(
                wall_time_s=elapsed,
                joint7_command=self._gripper_trace_commands[0],
                joint8_command=self._gripper_trace_commands[1],
                joint7_feedback=self._gripper_feedback_position,
                joint8_feedback=self._gripper8_feedback_position,
                joint7_stamp_s=self._gripper_feedback_stamp_s,
                joint8_stamp_s=self._gripper8_feedback_stamp_s,
            )
        )
        self._gripper_trace_last_sample_s = elapsed

    def _gripper_gate_snapshot(self):
        """Capture the exact feedback sample used by the post-action gate."""

        joint7 = self._gripper_feedback_position
        joint8 = self._gripper8_feedback_position
        symmetry_error = None
        if joint7 is not None and joint8 is not None:
            symmetry_error = abs(float(joint7) + float(joint8))
        return {
            "wall_time_s": time.monotonic(),
            "joint7_feedback": joint7,
            "joint8_feedback": joint8,
            "joint7_stamp_s": getattr(self, "_gripper_feedback_stamp_s", None),
            "joint8_stamp_s": getattr(self, "_gripper8_feedback_stamp_s", None),
            "symmetry_error_m": symmetry_error,
        }

    def _write_gripper_gate_snapshot(self, snapshot):
        trace_path = getattr(self, "_gripper_trace_path", "")
        if not trace_path:
            return
        try:
            base = Path(trace_path)
            suffix = base.suffix or ".csv"
            stem = base.stem or "gripper_trace"
            pair = (
                f"_pair{self._gripper_trace_pair_index:02d}"
                if self._gripper_trace_pair_index
                else ""
            )
            path = base.with_name(f"{stem}{pair}_gate{suffix}")
            path.parent.mkdir(parents=True, exist_ok=True)
            fields = [
                "wall_time_s",
                "joint7_feedback",
                "joint8_feedback",
                "joint7_stamp_s",
                "joint8_stamp_s",
                "symmetry_error_m",
            ]
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerow({field: snapshot.get(field) for field in fields})
        except Exception as error:
            self.node.get_logger().warning(
                f"gripper gate trace write failed: {error}"
            )

    def _finish_gripper_trace(self):
        if self._gripper_trace_started_s is None:
            return
        try:
            self._sample_gripper_trace()
            if not self._gripper_trace_rows:
                self._sample_gripper_trace(force=True)
            base = Path(self._gripper_trace_path)
            suffix = base.suffix or ".csv"
            stem = base.stem or "gripper_trace"
            path = base.with_name(
                f"{stem}_pair{self._gripper_trace_pair_index:02d}{suffix}"
            )
            path.parent.mkdir(parents=True, exist_ok=True)
            fields = list(self._gripper_trace_rows[0]) if self._gripper_trace_rows else [
                "wall_time_s",
                "joint7_command",
                "joint8_command",
                "joint7_feedback",
                "joint8_feedback",
                "joint7_stamp_s",
                "joint8_stamp_s",
                "symmetry_error_m",
            ]
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(self._gripper_trace_rows)
        except Exception as error:
            self.node.get_logger().warning(f"gripper trace write failed: {error}")
        finally:
            self._gripper_trace_started_s = None
            self._gripper_trace_last_sample_s = None
            self._gripper_trace_commands = (None, None)
            self._gripper_trace_rows = []

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
        self._wait_for_server(self.gripper8_client, self.gripper8_action_name)

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
                    self.node.arm_joint_names, arm, 1.0
                ),
                self.arm_client,
                "hold arm",
            )
            self._execute_gripper_pair(
                self._paired_gripper_trajectories(
                    actual_gripper_m, self.gripper_command_scale, 2.0
                ),
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
        del gripper_target_m, speed_percent, effort, tracking_limit
        retimed = self._retime_trajectory(trajectory, max_joint_rate)
        return self._execute(retimed, self.arm_client, name)

    def command_gripper(self, target_actual_m, args):
        result = self._execute_gripper_pair(
            self._paired_gripper_trajectories(
                float(target_actual_m), self.gripper_command_scale, 2.0
            ),
            "gripper",
        )
        self.node.spin_for(0.2)
        gate_snapshot = self._gripper_gate_snapshot()
        self._write_gripper_gate_snapshot(gate_snapshot)
        symmetry_error = gate_snapshot["symmetry_error_m"]
        if symmetry_error is not None and symmetry_error > 0.002:
            close_opening_mm = getattr(args, "close_opening_mm", None)
            try:
                is_post_close = abs(
                    float(target_actual_m) - float(close_opening_mm) / 1000.0
                ) <= 1e-6
            except (TypeError, ValueError):
                is_post_close = False
            if not is_post_close:
                raise RuntimeError(
                    f"gripper fingers are not symmetric: {symmetry_error:.4f}m"
                )
            get_logger = getattr(self.node, "get_logger", None)
            if callable(get_logger):
                get_logger().warning(
                    "post-close gripper symmetry diagnostic above 2 mm: "
                    f"joint7={gate_snapshot['joint7_feedback']:.4f}m, "
                    f"joint8={gate_snapshot['joint8_feedback']:.4f}m, "
                    f"error={symmetry_error:.4f}m, "
                    f"stamps=({gate_snapshot['joint7_stamp_s']}, "
                    f"{gate_snapshot['joint8_stamp_s']})"
                )
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

    def gripper_symmetry_error(self):
        if self._gripper_feedback_position is None or self._gripper8_feedback_position is None:
            return None
        return abs(self._gripper_feedback_position + self._gripper8_feedback_position)

    @staticmethod
    def _paired_gripper_trajectories(actual_gripper_m, command_scale, duration_sec=1.0):
        command = float(actual_gripper_m) * float(command_scale)
        return (
            Ros2ControlBackend._single_point_trajectory(
                ["joint7"], [command], duration_sec
            ),
            Ros2ControlBackend._single_point_trajectory(
                ["joint8"], [-command], duration_sec
            ),
        )

    @staticmethod
    def _retime_trajectory(trajectory, max_joint_rate):
        rate = float(max_joint_rate)
        if rate <= 0.0:
            raise RuntimeError("max_joint_rate must be positive")
        retimed = copy.deepcopy(trajectory)
        elapsed = 0.0
        previous = None
        for point in retimed.points:
            positions = [float(value) for value in point.positions]
            if previous is not None:
                distance = max(
                    (abs(current - old) for current, old in zip(positions, previous)),
                    default=0.0,
                )
                elapsed += max(distance / rate, 0.02)
            point.time_from_start.sec = int(elapsed)
            point.time_from_start.nanosec = int(
                round((elapsed - int(elapsed)) * 1e9)
            )
            point.velocities = []
            point.accelerations = []
            point.effort = []
            previous = positions
        return retimed

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

    def _execute_gripper_pair(self, trajectories, label):
        if not self._prepared:
            raise RuntimeError("simulation backend is not prepared")
        if len(trajectories) != 2:
            raise RuntimeError(f"{label} requires two finger trajectories")
        gripper_action_name = getattr(
            self, "gripper_action_name", "/gripper_controller/follow_joint_trajectory"
        )
        gripper8_action_name = getattr(
            self,
            "gripper8_action_name",
            "/gripper8_controller/follow_joint_trajectory",
        )
        self._wait_for_server(self.gripper_client, gripper_action_name)
        self._wait_for_server(
            self.gripper8_client, gripper8_action_name
        )
        self._begin_execution()
        handles = []
        start = time.monotonic()
        self._begin_gripper_trace(trajectories)
        try:
            goals = []
            for trajectory in trajectories:
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = trajectory
                goals.append(goal)
            goal_futures = [
                client.send_goal_async(goal)
                for client, goal in zip(
                    (self.gripper_client, self.gripper8_client), goals
                )
            ]
            handles = [
                self._spin_until(future, self.action_timeout, label)
                for future in goal_futures
            ]
            if not all(handle.accepted for handle in handles):
                raise RuntimeError(f"{label} goal rejected")
            self._active_goals.extend(handles)
            result_futures = [handle.get_result_async() for handle in handles]
            wrapped = [
                self._spin_until(
                    future,
                    max(
                        self.action_timeout,
                        self._trajectory_duration(trajectory) + 10.0,
                    ),
                    label,
                )
                for future, trajectory in zip(result_futures, trajectories)
            ]
            if any(int(item.result.error_code) != 0 for item in wrapped):
                raise RuntimeError(f"{label} failed with FollowJointTrajectory error")
            return ExecutionResult(
                duration_sec=time.monotonic() - start,
                final_error=0.0,
                maximum_tracking_error=0.0,
                gripper_position=self.node.current_positions()[6],
            )
        except RuntimeError:
            for handle in handles:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            raise
        finally:
            self._finish_gripper_trace()
            for handle in handles:
                try:
                    self._active_goals.remove(handle)
                except ValueError:
                    pass
            self._end_execution()

    def _spin_until(self, future, timeout_sec, label):
        deadline = time.monotonic() + float(timeout_sec)
        while not future.done() and time.monotonic() < deadline:
            self.node.spin_for(0.02)
            self._sample_gripper_trace()
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
