"""Launch the pinned Piper Gazebo model in this package's chosen world.

The arm and controllers remain upstream Piper assets.  The local wrapper adds
the eye-in-hand sensor and simulation contact parameters without modifying the
vendor checkout.
"""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _controller_spawner(name):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", "/controller_manager"],
        output="screen",
    )


def generate_launch_description():
    simulation_share = get_package_share_directory("foam_grasp_sim")
    world = LaunchConfiguration("world")
    gazebo_executable = LaunchConfiguration("gazebo_executable")
    default_robot_xacro = (
        Path(simulation_share) / "urdf" / "piper_eye_in_hand_physics.xacro"
    )
    robot_xacro = LaunchConfiguration("robot_xacro")
    qualification_mode = LaunchConfiguration("qualification_mode")
    qualification_config = LaunchConfiguration("qualification_config")
    qualification_output = LaunchConfiguration("qualification_output")
    physics_pid_config = Path(simulation_share) / "config" / "ros2_controllers_physics.yaml"
    physics_pid_config_arg = LaunchConfiguration("physics_pid_config")
    physics_renderer = (
        Path(get_package_prefix("foam_grasp_sim"))
        / "lib"
        / "foam_grasp_sim"
        / "render_physics_description"
    )

    gazebo = ExecuteProcess(
        cmd=[
            gazebo_executable,
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
        [
            str(physics_renderer),
            " ",
            robot_xacro,
            " ",
            physics_pid_config_arg,
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # piper_description_gazebo.xacro's gazebo_ros2_control plugin waits
        # for this exact node name when requesting robot_description.
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
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
    qualification_node = Node(
        package="foam_grasp_sim",
        executable="control_physics_qualification",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", qualification_mode, "' != 'off'"])
        ),
        parameters=[
            {
                "mode": qualification_mode,
                "config": qualification_config,
                "output_dir": qualification_output,
                "use_sim_time": True,
            }
        ],
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
                    "defaults to the local eye-in-hand wrapper around Piper"
                ),
            ),
            DeclareLaunchArgument("qualification_mode", default_value="off"),
            DeclareLaunchArgument("qualification_config", default_value=""),
            DeclareLaunchArgument(
                "qualification_output",
                default_value="/tmp/foam_grasp_control_qualification",
            ),
            DeclareLaunchArgument(
                "physics_pid_config",
                default_value=str(physics_pid_config),
                description="Controller/PID overlay passed to the physics renderer",
            ),
            DeclareLaunchArgument(
                "gazebo_executable",
                default_value="gzserver",
                description="Gazebo executable; use gazebo only when a GUI is available",
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
                    on_exit=[qualification_node],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=qualification_node, on_exit=[Shutdown()])
            ),
        ]
    )
