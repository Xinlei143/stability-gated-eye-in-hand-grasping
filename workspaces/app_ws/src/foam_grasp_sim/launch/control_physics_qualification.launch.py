"""Launch Piper without a target and run arm/gripper physics qualification."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("foam_grasp_sim")
    default_config = Path(package_share) / "config" / "control_physics_qualification.yaml"
    mode = LaunchConfiguration("mode")
    config = LaunchConfiguration("config")
    output_dir = LaunchConfiguration("output_dir")
    robot_xacro = LaunchConfiguration("robot_xacro")
    physics_pid_config = LaunchConfiguration("physics_pid_config")
    piper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(package_share) / "launch" / "piper_sim.launch.py")
        ),
        launch_arguments={
            "gazebo_executable": LaunchConfiguration("gazebo_executable"),
            "qualification_mode": mode,
            "qualification_config": config,
            "qualification_output": output_dir,
            "robot_xacro": robot_xacro,
            "physics_pid_config": physics_pid_config,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="arm"),
            DeclareLaunchArgument("config", default_value=str(default_config)),
            DeclareLaunchArgument(
                "output_dir", default_value="/tmp/foam_grasp_control_qualification"
            ),
            DeclareLaunchArgument(
                "robot_xacro",
                default_value=str(Path(package_share) / "urdf" / "piper_eye_in_hand_physics.xacro"),
                description=(
                    "Robot wrapper; pass piper_eye_in_hand_loaded_qualification.xacro "
                    "for loaded_gripper mode"
                ),
            ),
            DeclareLaunchArgument(
                "physics_pid_config",
                default_value=str(Path(package_share) / "config" / "ros2_controllers_physics.yaml"),
            ),
            DeclareLaunchArgument("gazebo_executable", default_value="gzserver"),
            piper,
        ]
    )
