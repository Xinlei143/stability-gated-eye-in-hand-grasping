"""Planning-only MoveIt launch for a real Piper arm.

This launch intentionally does not start joint_state_publisher, ros2_control,
controller_manager, fake hardware, or any user motion-control node.  The real
Piper feedback topic is remapped from /joint_states_single for consumers that
normally listen on /joint_states.  Trajectory execution is disabled.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    moveit_config = (
        MoveItConfigsBuilder(
            "piper",
            package_name="piper_with_gripper_moveit",
        )
        .to_moveit_configs()
    )

    # Piper uses /joint_states as its command input and publishes real feedback
    # on /joint_states_single.  Remap only the consumers created below.
    feedback_remapping = [("/joint_states", "/joint_states_single")]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="piper_robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
        remappings=feedback_remapping,
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "allow_trajectory_execution": False,
                "moveit_manage_controllers": False,
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
        ],
        remappings=feedback_remapping,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="piper_moveit_plan_only_rviz",
        output="screen",
        arguments=[
            "-d",
            str(moveit_config.package_path / "config" / "moveit.rviz"),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
        remappings=feedback_remapping,
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            robot_state_publisher,
            move_group,
            rviz,
        ]
    )
