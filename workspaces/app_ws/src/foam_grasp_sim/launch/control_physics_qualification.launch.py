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
    piper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(package_share) / "launch" / "piper_sim.launch.py")
        ),
        launch_arguments={
            "gazebo_executable": LaunchConfiguration("gazebo_executable"),
            "qualification_mode": mode,
            "qualification_config": config,
            "qualification_output": output_dir,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="arm"),
            DeclareLaunchArgument("config", default_value=str(default_config)),
            DeclareLaunchArgument(
                "output_dir", default_value="/tmp/foam_grasp_control_qualification"
            ),
            DeclareLaunchArgument("gazebo_executable", default_value="gzserver"),
            piper,
        ]
    )
