"""One-launch Piper simulation with controlled target motion and perception."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


TARGET_MODELS = ("cube", "cylinder", "sphere")


def _load_simulation_config(package_share):
    config_path = Path(package_share) / "config" / "simulation.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    try:
        table = config["scene"]["table"]
        targets = config["scene"]["targets"]
        execution = config["execution"]
        pipeline = config["pipeline"]
        method = config["method"]
        motion = config["motion"]
        perception = config["perception"]
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            f"Invalid simulation configuration {config_path}: {error}"
        ) from error
    for key in ("size", "pose"):
        if len(table[key]) != 3:
            raise RuntimeError(f"scene.table.{key} must contain three values")
    for model in TARGET_MODELS:
        if model not in targets or len(targets[model]["pose"]) != 3:
            raise RuntimeError(
                f"scene.targets.{model}.pose must contain three values"
            )
    if len(motion["velocity"]) != 3:
        raise RuntimeError("motion.velocity must contain three values")
    return table, targets, execution, pipeline, method, motion, perception


def _scene_spawn(package_share, model, entity, pose, condition=None):
    model_file = Path(package_share) / "models" / model / "model.sdf"
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=f"spawn_{entity}",
        output="screen",
        arguments=[
            "-entity",
            entity,
            "-file",
            str(model_file),
            "-x",
            str(pose[0]),
            "-y",
            str(pose[1]),
            "-z",
            str(pose[2]),
            "-timeout",
            "60.0",
        ],
        condition=condition,
    )


def _target_condition(target_model, name):
    return IfCondition(PythonExpression(["'", target_model, f"' == '{name}'"]))


def _parameter(name, value_type):
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description():
    package_share = get_package_share_directory("foam_grasp_sim")
    table, targets, execution, pipeline, method_config, motion, perception = _load_simulation_config(
        package_share
    )
    use_rviz = LaunchConfiguration("use_rviz")
    target_model = LaunchConfiguration("target_model")
    run_grasp_pipeline = LaunchConfiguration("run_grasp_pipeline")
    execute_motion = LaunchConfiguration("execute_motion")
    trajectory = LaunchConfiguration("trajectory")
    perception_source = LaunchConfiguration("perception_source")
    method = LaunchConfiguration("method")

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(package_share) / "launch" / "piper_sim.launch.py")
        )
    )
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(package_share) / "launch" / "sim_moveit.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "start_robot_state_publisher": "false",
        }.items(),
    )

    table_spawn = _scene_spawn(
        package_share, "table", "grasp_table", table["pose"]
    )
    target_spawns = [
        _scene_spawn(
            package_share,
            model,
            f"foam_{model}",
            targets[model]["pose"],
            _target_condition(target_model, model),
        )
        for model in TARGET_MODELS
    ]

    motion_parameters = {
        "use_sim_time": execution["use_sim_time"],
        "target_model": target_model,
        "entity_name": PythonExpression(["'foam_' + '", target_model, "'"]),
        "base_frame": pipeline["target_latch"]["base_frame"],
        "start_position": targets["cube"]["pose"],
        "trajectory": trajectory,
        "velocity_x": _parameter("velocity_x", float),
        "velocity_y": _parameter("velocity_y", float),
        "velocity_z": _parameter("velocity_z", float),
        "move_duration": _parameter("move_duration", float),
        "stop_duration": _parameter("stop_duration", float),
        "control_rate": _parameter("motion_control_rate", float),
        "ground_truth_rate": _parameter("ground_truth_rate", float),
        "seed": _parameter("seed", int),
    }
    # The selected target pose is resolved by three mutually exclusive launch
    # nodes below; its start_position must use the matching model pose too.
    motion_nodes = [
        Node(
            package="foam_grasp_sim",
            executable="target_motion_node",
            name="foam_target_motion",
            output="screen",
            parameters=[
                {
                    **motion_parameters,
                    "start_position": targets[model]["pose"],
                }
            ],
            condition=_target_condition(target_model, model),
        )
        for model in TARGET_MODELS
    ]

    perception_parameters = {
        "use_sim_time": execution["use_sim_time"],
        "target_model": target_model,
        "base_frame": pipeline["target_latch"]["base_frame"],
        "source": perception_source,
        "sampling_rate": _parameter("perception_sampling_rate", float),
        "latency_ms": _parameter("latency_ms", float),
        "noise_std_mm": _parameter("noise_std_mm", float),
        "dropout_probability": _parameter("dropout_probability", float),
        "outlier_probability": _parameter("outlier_probability", float),
        "outlier_range_mm": _parameter("outlier_range_mm", float),
        "history_duration": _parameter("history_duration", float),
        "seed": _parameter("seed", int),
        "scenario": LaunchConfiguration("scenario"),
        "target_timeout": _parameter("observation_timeout", float),
    }
    simulated_perception = Node(
        package="foam_grasp_sim",
        executable="simulated_perception_node",
        name="foam_simulated_perception",
        output="screen",
        parameters=[perception_parameters],
        condition=IfCondition(
            PythonExpression(["'", perception_source, "' != 'rgbd'"])
        ),
    )

    method_policy = Node(
        package="foam_grasp_sim",
        executable="method_policy_node",
        name="foam_method_policy",
        output="screen",
        parameters=[
            {
                "use_sim_time": execution["use_sim_time"],
                "target_model": target_model,
                "base_frame": pipeline["target_latch"]["base_frame"],
                "method": method,
                "stability_duration": _parameter("stability_duration", float),
                "position_spread_threshold": _parameter(
                    "position_spread_threshold", float
                ),
                "minimum_stable_samples": _parameter(
                    "minimum_stable_samples", int
                ),
                "observation_timeout": _parameter("observation_timeout", float),
                "scenario": LaunchConfiguration("scenario"),
                "seed": _parameter("seed", int),
            }
        ],
    )

    pose_parameters = {
        "use_sim_time": execution["use_sim_time"],
        "input_topic": PythonExpression(
            ["'/foam_grasp/' + '", target_model, "' + '_method_point_base'"]
        ),
        "class_topic": "/foam_grasp/method_target_class",
    }
    pose_parameters.update(pipeline["pose_preview"])
    grasp_pose_preview = Node(
        package="foam_grasp",
        executable="grasp_pose_preview_node",
        name="foam_grasp_pose_preview",
        output="screen",
        parameters=[pose_parameters],
        condition=IfCondition(run_grasp_pipeline),
    )

    sequence_parameters = dict(execution)
    sequence_parameters.update(
        {
            "table_size": table["size"],
            "table_pose": table["pose"],
            "wait_for_method_ready": True,
            "method_ready_timeout": 60.0,
            "method": method,
            "commit_method_service": "/foam_grasp/commit_method_target",
            "class_topic": "/foam_grasp/method_target_class",
            "scenario": LaunchConfiguration("scenario"),
            "tracking_commit_timeout": _parameter("tracking_commit_timeout", float),
            "tracking_replan_threshold": _parameter("tracking_replan_threshold", float),
            "tracking_commit_tolerance": _parameter("tracking_commit_tolerance", float),
            "tracking_max_updates": _parameter("tracking_max_updates", int),
            "observation_timeout": _parameter("observation_timeout", float),
        }
    )
    sequence_arguments = [
        "--execution-backend",
        "simulation",
        "--target-class",
        target_model,
        "--auto-latch",
    ]
    plan_sequence = Node(
        package="foam_grasp",
        executable="object_grasp_sequence",
        name="foam_static_grasp_sequence",
        output="screen",
        parameters=[sequence_parameters],
        arguments=sequence_arguments,
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    run_grasp_pipeline,
                    "' == 'true' and '",
                    execute_motion,
                    "' != 'true'",
                ]
            )
        ),
    )
    execute_sequence = Node(
        package="foam_grasp",
        executable="object_grasp_sequence",
        name="foam_static_grasp_sequence",
        output="screen",
        parameters=[sequence_parameters],
        arguments=sequence_arguments
        + [
            "--execute",
            "--auto",
            "--confirm",
            "AUTO_FULL_OBJECT_GRASP",
            "--countdown-seconds",
            "0",
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    run_grasp_pipeline,
                    "' == 'true' and '",
                    execute_motion,
                    "' == 'true'",
                ]
            )
        ),
    )

    benchmark_logger = Node(
        package="foam_grasp_sim",
        executable="metrics_logger_node",
        name="foam_metrics_logger",
        output="screen",
        parameters=[
            {
                "use_sim_time": execution["use_sim_time"],
                "record_benchmark": True,
                "results_root": LaunchConfiguration("results_root"),
                "run_id": LaunchConfiguration("run_id"),
                "scenario": LaunchConfiguration("scenario"),
                "method": method,
                "target_model": target_model,
                "seed": _parameter("seed", int),
                "metrics_rate": _parameter("metrics_rate", float),
                "tool_offset": 0.1358,
                "config_hash": LaunchConfiguration("config_hash"),
                "pair_id": LaunchConfiguration("pair_id"),
                "condition_json": LaunchConfiguration("condition_json"),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    run_grasp_pipeline,
                    "' == 'true' and '",
                    LaunchConfiguration("record_benchmark"),
                    "' == 'true'",
                ]
            )
        ),
    )

    # Gazebo must expose model state and factory services before the selected
    # target can be detected and moved.  The perception node consumes actual
    # Gazebo state, never trajectory-commanded coordinates.
    scene = TimerAction(
        period=5.0,
        actions=[
            table_spawn,
            *target_spawns,
            *motion_nodes,
            simulated_perception,
            method_policy,
            benchmark_logger,
        ],
    )
    sequence = TimerAction(period=10.0, actions=[plan_sequence, execute_sequence])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "target_model",
                default_value="cube",
                description="Static target: cube, cylinder, or sphere",
            ),
            DeclareLaunchArgument(
                "run_grasp_pipeline",
                default_value="true",
                description="Start method policy, pose preview and sequence",
            ),
            DeclareLaunchArgument(
                "execute_motion",
                default_value="false",
                description="Execute the selected method after readiness",
            ),
            DeclareLaunchArgument(
                "method",
                default_value=str(method_config["name"]),
                description="snapshot, tracking, or gated",
            ),
            DeclareLaunchArgument(
                "stability_duration",
                default_value=str(method_config["stability_duration"]),
            ),
            DeclareLaunchArgument(
                "position_spread_threshold",
                default_value=str(method_config["position_spread_threshold"]),
            ),
            DeclareLaunchArgument(
                "minimum_stable_samples",
                default_value=str(method_config["minimum_stable_samples"]),
            ),
            DeclareLaunchArgument(
                "observation_timeout",
                default_value=str(method_config["observation_timeout"]),
            ),
            DeclareLaunchArgument(
                "tracking_commit_timeout",
                default_value=str(method_config["tracking_commit_timeout"]),
            ),
            DeclareLaunchArgument(
                "tracking_replan_threshold",
                default_value=str(method_config["tracking_replan_threshold"]),
            ),
            DeclareLaunchArgument(
                "tracking_commit_tolerance",
                default_value=str(method_config["tracking_commit_tolerance"]),
            ),
            DeclareLaunchArgument(
                "tracking_max_updates",
                default_value=str(method_config["tracking_max_updates"]),
            ),
            DeclareLaunchArgument(
                "scenario", default_value=str(motion["trajectory"])
            ),
            DeclareLaunchArgument(
                "record_benchmark", default_value="false"
            ),
            DeclareLaunchArgument(
                "results_root", default_value="results"
            ),
            DeclareLaunchArgument(
                "run_id", default_value=""
            ),
            DeclareLaunchArgument(
                "metrics_rate", default_value="10.0"
            ),
            DeclareLaunchArgument("config_hash", default_value=""),
            DeclareLaunchArgument("pair_id", default_value=""),
            DeclareLaunchArgument("condition_json", default_value="{}"),
            DeclareLaunchArgument(
                "trajectory",
                default_value=str(motion["trajectory"]),
                description="static, constant_velocity, move_stop, or move_stop_move",
            ),
            DeclareLaunchArgument(
                "velocity_x", default_value=str(motion["velocity"][0])
            ),
            DeclareLaunchArgument(
                "velocity_y", default_value=str(motion["velocity"][1])
            ),
            DeclareLaunchArgument(
                "velocity_z", default_value=str(motion["velocity"][2])
            ),
            DeclareLaunchArgument(
                "move_duration", default_value=str(motion["move_duration"])
            ),
            DeclareLaunchArgument(
                "stop_duration", default_value=str(motion["stop_duration"])
            ),
            DeclareLaunchArgument(
                "motion_control_rate", default_value=str(motion["control_rate"])
            ),
            DeclareLaunchArgument(
                "ground_truth_rate", default_value=str(motion["ground_truth_rate"])
            ),
            DeclareLaunchArgument(
                "perception_source",
                default_value=str(perception["source"]),
                description="ground_truth, disturbed, or reserved rgbd",
            ),
            DeclareLaunchArgument(
                "perception_sampling_rate",
                default_value=str(perception["sampling_rate"]),
            ),
            DeclareLaunchArgument(
                "latency_ms", default_value=str(perception["latency_ms"])
            ),
            DeclareLaunchArgument(
                "noise_std_mm", default_value=str(perception["noise_std_mm"])
            ),
            DeclareLaunchArgument(
                "dropout_probability",
                default_value=str(perception["dropout_probability"]),
            ),
            DeclareLaunchArgument(
                "outlier_probability",
                default_value=str(perception["outlier_probability"]),
            ),
            DeclareLaunchArgument(
                "outlier_range_mm",
                default_value=str(perception["outlier_range_mm"]),
            ),
            DeclareLaunchArgument(
                "history_duration",
                default_value=str(perception["history_duration"]),
            ),
            DeclareLaunchArgument("seed", default_value=str(perception["seed"])),
            piper_launch,
            moveit_launch,
            grasp_pose_preview,
            scene,
            sequence,
        ]
    )
