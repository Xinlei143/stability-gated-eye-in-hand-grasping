"""Compose the simulated eye-in-hand RGB-D perception and grasp pipeline.

``sim_bringup.launch.py`` remains the only Gazebo trial entry point.  This
launch adds the real RGB-D perception nodes around it; it does not reproduce
the simulator, target motion, method policy, or sequence implementation.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    package_share = Path(get_package_share_directory("foam_grasp_sim"))
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "sim_bringup.launch.py")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_moveit": LaunchConfiguration("start_moveit"),
            "target_model": LaunchConfiguration("target_model"),
            "run_grasp_pipeline": LaunchConfiguration("run_grasp_pipeline"),
            "execute_motion": LaunchConfiguration("execute_motion"),
            "prepare_observation_pose": LaunchConfiguration("prepare_observation_pose"),
            "method": LaunchConfiguration("method"),
            "trajectory": LaunchConfiguration("trajectory"),
            "scenario": LaunchConfiguration("trajectory"),
            "perception_source": "rgbd",
            "record_benchmark": LaunchConfiguration("record_benchmark"),
            "results_root": LaunchConfiguration("results_root"),
            "run_id": LaunchConfiguration("run_id"),
            "config_hash": LaunchConfiguration("config_hash"),
            "pair_id": LaunchConfiguration("pair_id"),
            "condition_json": LaunchConfiguration("condition_json"),
            "robot_xacro": LaunchConfiguration("robot_xacro"),
            "gazebo_executable": LaunchConfiguration("gazebo_executable"),
            "grasp_stabilization_mode": LaunchConfiguration("grasp_stabilization_mode"),
            "grasp_assist_mode": LaunchConfiguration("grasp_assist_mode"),
            "grasp_assist_service": LaunchConfiguration("grasp_assist_service"),
        }.items(),
    )

    segmentation = Node(
        package="foam_grasp",
        executable="segmentation_node",
        name="foam_rgbd_segmentation",
        output="screen",
        parameters=[
            {
                "checkpoint_path": LaunchConfiguration("checkpoint"),
                "require_cuda": _bool("require_cuda"),
                "input_width": 640,
                "input_height": 360,
            }
        ],
    )
    depth_fusion = Node(
        package="foam_grasp",
        executable="depth_fusion_node",
        name="foam_rgbd_depth_fusion",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    camera_to_base = Node(
        package="foam_grasp",
        executable="camera_to_base_node",
        name="foam_rgbd_camera_to_base",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "transform_source": "tf",
                "base_frame": "base_link",
                "tf_timeout": ParameterValue(LaunchConfiguration("tf_timeout"), value_type=float),
            }
        ],
    )

    def _shutdown_on_rgbd_exit(node_name):
        def on_exit(event, context):
            del context
            return [
                Shutdown(
                    reason=(
                        f"{node_name} infrastructure failure; shutting down "
                        f"RGB-D pipeline (returncode={event.returncode})"
                    )
                )
            ]

        return on_exit

    rgbd_supervision_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=segmentation,
                on_exit=_shutdown_on_rgbd_exit("segmentation_node"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=depth_fusion,
                on_exit=_shutdown_on_rgbd_exit("depth_fusion_node"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=camera_to_base,
                on_exit=_shutdown_on_rgbd_exit("camera_to_base_node"),
            )
        ),
    ]

    declarations = [
        DeclareLaunchArgument("checkpoint", description="Absolute path to best_model.pth"),
        DeclareLaunchArgument("require_cuda", default_value="true"),
        DeclareLaunchArgument("tf_timeout", default_value="0.2"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("start_moveit", default_value="true"),
        DeclareLaunchArgument("target_model", default_value="cube"),
        DeclareLaunchArgument("run_grasp_pipeline", default_value="true"),
        DeclareLaunchArgument("execute_motion", default_value="false"),
        DeclareLaunchArgument("prepare_observation_pose", default_value="true"),
        DeclareLaunchArgument("method", default_value="gated"),
        DeclareLaunchArgument("trajectory", default_value="static"),
        DeclareLaunchArgument("record_benchmark", default_value="true"),
        DeclareLaunchArgument("results_root", default_value="results"),
        DeclareLaunchArgument("run_id", default_value=""),
        DeclareLaunchArgument("config_hash", default_value=""),
        DeclareLaunchArgument("pair_id", default_value=""),
        DeclareLaunchArgument("condition_json", default_value="{}"),
        DeclareLaunchArgument("robot_xacro", default_value=str(package_share / "urdf" / "piper_eye_in_hand_physics.xacro")),
        DeclareLaunchArgument("gazebo_executable", default_value="gzserver"),
        DeclareLaunchArgument("grasp_stabilization_mode", default_value="off"),
        DeclareLaunchArgument("grasp_assist_mode", default_value="off"),
        DeclareLaunchArgument("grasp_assist_service", default_value=""),
    ]
    return LaunchDescription(
        declarations
        + [
            sim_launch,
            *rgbd_supervision_handlers,
            segmentation,
            depth_fusion,
            camera_to_base,
        ]
    )
