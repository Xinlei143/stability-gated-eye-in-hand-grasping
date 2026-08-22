"""Launch the pinned Piper Gazebo model in this package's chosen world.

The robot description, ros2_control configuration and joint8 mirror remain
upstream Piper assets.  This launch owns composition only, so benchmark physics
and scene selection are controlled by foam_grasp_sim without forking a URDF.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _controller_spawner(name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
        output="screen",
    )


def generate_launch_description():
    simulation_share = get_package_share_directory("foam_grasp_sim")
    description_share = get_package_share_directory("piper_description")
    world = LaunchConfiguration("world")
    default_robot_xacro = (
        Path(description_share) / "urdf" / "piper_description_gazebo.xacro"
    )
    robot_xacro = LaunchConfiguration("robot_xacro")

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", robot_xacro]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="foam_grasp_sim_robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )
    spawn_piper = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_piper",
        output="screen",
        arguments=[
            "-entity",
            "piper",
            "-topic",
            "robot_description",
            "-timeout",
            "60.0",
        ],
    )

    joint_state_broadcaster = _controller_spawner("joint_state_broadcaster")
    arm_controller = _controller_spawner("arm_controller")
    gripper_controller = _controller_spawner("gripper_controller")
    gripper8_controller = _controller_spawner("gripper8_controller")
    joint8_mirror = Node(
        package="piper_gazebo",
        executable="joint8_ctrl.py",
        name="foam_grasp_sim_joint8_mirror",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    Path(simulation_share) / "worlds" / "grasp_table.world"
                ),
                description="Gazebo Classic world managed by foam_grasp_sim",
            ),
            DeclareLaunchArgument(
                "robot_xacro",
                default_value=str(default_robot_xacro),
                description=(
                    "Robot Xacro passed to robot_state_publisher and Gazebo; "
                    "defaults to the pinned upstream Piper description"
                ),
            ),
            gazebo,
            robot_state_publisher,
            spawn_piper,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_piper,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[arm_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_controller,
                    on_exit=[gripper_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gripper_controller,
                    on_exit=[gripper8_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gripper8_controller,
                    on_exit=[joint8_mirror],
                )
            ),
        ]
    )
