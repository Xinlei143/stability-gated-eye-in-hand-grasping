"""Bring up the complete foam-grasp perception and planning stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    checkpoint = LaunchConfiguration("checkpoint")
    calibration_file = LaunchConfiguration("calibration_file")
    can_port = LaunchConfiguration("can_port")
    auto_enable = LaunchConfiguration("auto_enable")
    start_camera = LaunchConfiguration("start_camera")
    start_piper = LaunchConfiguration("start_piper")
    start_moveit = LaunchConfiguration("start_moveit")
    use_rviz = LaunchConfiguration("use_rviz")

    package_share = FindPackageShare("foam_grasp")
    runtime_config = PathJoinSubstitution(
        [package_share, "config", "runtime.yaml"]
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orbbec_camera"), "launch", "dabai.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "depth_registration": "true",
            "enable_ir": "false",
            "enable_point_cloud": "true",
        }.items(),
    )

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("piper"), "launch", "start_single_piper.launch.py"]
            )
        ),
        condition=IfCondition(start_piper),
        launch_arguments={
            "can_port": can_port,
            "auto_enable": auto_enable,
            "gripper_exist": "true",
            "gripper_val_mutiple": "2",
            "log_level": "warn",
        }.items(),
    )

    safe_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "safe_plan_only.launch.py"]
            )
        ),
        condition=IfCondition(start_moveit),
        launch_arguments={"use_rviz": use_rviz}.items(),
    )

    segmentation = Node(
        package="foam_grasp",
        executable="segmentation_node",
        name="foam_segmentation",
        output="screen",
        parameters=[runtime_config, {"checkpoint_path": checkpoint}],
    )
    depth_fusion = Node(
        package="foam_grasp",
        executable="depth_fusion_node",
        name="foam_depth_fusion",
        output="screen",
    )
    camera_to_base = Node(
        package="foam_grasp",
        executable="camera_to_base_node",
        name="foam_camera_to_base",
        output="screen",
        parameters=[runtime_config, {"calibration_file": calibration_file}],
    )
    target_latch = Node(
        package="foam_grasp",
        executable="target_latch_node",
        name="foam_target_latch",
        output="screen",
        parameters=[runtime_config],
    )
    pose_preview = Node(
        package="foam_grasp",
        executable="grasp_pose_preview_node",
        name="foam_grasp_pose_preview",
        output="screen",
        parameters=[runtime_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "checkpoint",
                description="Absolute path to best_model.pth",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                description="Absolute path to hand-eye calibration JSON",
            ),
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("auto_enable", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_piper", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            camera_launch,
            piper_launch,
            safe_moveit,
            target_latch,
            pose_preview,
            camera_to_base,
            TimerAction(period=4.0, actions=[segmentation]),
            TimerAction(period=7.0, actions=[depth_fusion]),
        ]
    )

