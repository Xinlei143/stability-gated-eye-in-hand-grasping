"""Run a fixed-pose, no-lift, free-cube gripper effort/contact diagnosis."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("foam_grasp_sim")
    package_path = Path(package_share)
    default_config = package_path / "config" / "static_grasp_hold_diagnosis.yaml"
    default_world = package_path / "worlds" / "grasp_table_no_attachment.world"
    with default_config.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    piper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_path / "launch" / "piper_sim.launch.py")),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gazebo_executable": LaunchConfiguration("gazebo_executable"),
            "robot_xacro": LaunchConfiguration("robot_xacro"),
            "physics_pid_config": LaunchConfiguration("physics_pid_config"),
            "qualification_mode": "off",
        }.items(),
    )
    diagnosis = Node(
        package="foam_grasp_sim",
        executable="static_grasp_hold_diagnosis",
        name="static_grasp_hold_diagnosis",
        output="screen",
        parameters=[
            {
                "config": LaunchConfiguration("config"),
                "output_dir": LaunchConfiguration("output_dir"),
                "cube_model_path": str(package_path / "models" / "cube" / "model.sdf"),
                "use_sim_time": True,
            }
        ],
    )
    scene = TimerAction(
        period=LaunchConfiguration("diagnosis_start_delay_s"),
        actions=[diagnosis],
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(default_config)),
        DeclareLaunchArgument("output_dir", default_value="/tmp/static_grasp_hold_diagnosis"),
        DeclareLaunchArgument("world", default_value=str(default_world)),
        DeclareLaunchArgument(
            "robot_xacro",
            default_value=str(package_path / "urdf" / "piper_eye_in_hand_physics.xacro"),
        ),
        DeclareLaunchArgument(
            "physics_pid_config",
            default_value=str(package_path / "config" / "ros2_controllers_physics.yaml"),
        ),
        DeclareLaunchArgument("gazebo_executable", default_value="gzserver"),
        DeclareLaunchArgument("diagnosis_start_delay_s", default_value="3.0"),
        piper,
        scene,
        RegisterEventHandler(
            OnProcessExit(target_action=diagnosis, on_exit=[Shutdown()])
        ),
    ])
