"""One-launch Piper simulation with controlled target motion and perception."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from foam_grasp_sim.grasp_stabilization import resolve_stabilization_mode


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


def _validate_stabilization_configuration(context):
    resolve_stabilization_mode(
        context.launch_configurations.get("grasp_stabilization_mode", "off"),
        context.launch_configurations.get("grasp_assist_mode", "off"),
        context.launch_configurations.get("grasp_assist_service", ""),
    )
    return []


def generate_launch_description():
    package_share = get_package_share_directory("foam_grasp_sim")
    description_share = get_package_share_directory("piper_description")
    table, targets, execution, pipeline, method_config, motion, perception = _load_simulation_config(
        package_share
    )
    use_rviz = LaunchConfiguration("use_rviz")
    start_moveit = LaunchConfiguration("start_moveit")
    target_model = LaunchConfiguration("target_model")
    run_grasp_pipeline = LaunchConfiguration("run_grasp_pipeline")
    execute_motion = LaunchConfiguration("execute_motion")
    prepare_observation_pose = LaunchConfiguration("prepare_observation_pose")
    trajectory = LaunchConfiguration("trajectory")
    perception_source = LaunchConfiguration("perception_source")
    method = LaunchConfiguration("method")
    robot_xacro = LaunchConfiguration("robot_xacro")
    gazebo_executable = LaunchConfiguration("gazebo_executable")
    grasp_assist_mode = LaunchConfiguration("grasp_assist_mode")
    grasp_stabilization_mode = LaunchConfiguration("grasp_stabilization_mode")
    record_contact_diagnostics = LaunchConfiguration("record_contact_diagnostics")
    contact_diagnostics_output = LaunchConfiguration("contact_diagnostics_output")
    simulation_readiness_timeout_s = LaunchConfiguration(
        "simulation_readiness_timeout_s"
    )

    physics_xacro = str(Path(package_share) / "urdf" / "piper_eye_in_hand_physics.xacro")
    grasp_fix_xacro = str(Path(package_share) / "urdf" / "piper_eye_in_hand_grasp_fix.xacro")
    no_attachment_world = str(Path(package_share) / "worlds" / "grasp_table_no_attachment.world")
    legacy_world = str(Path(package_share) / "worlds" / "grasp_table.world")
    effective_robot_xacro = PythonExpression([
        "'", grasp_stabilization_mode, "' == 'gazebo_grasp_fix' and '",
        grasp_fix_xacro, "' or '", physics_xacro, "'",
    ])
    effective_world = PythonExpression([
        "'", grasp_stabilization_mode, "' == 'gazebo_grasp_fix' and '",
        no_attachment_world, "' or ( '", grasp_stabilization_mode,
        "' == 'legacy_contact_confirmed' or '", grasp_assist_mode,
        "' == 'contact_confirmed' ) and '", legacy_world, "' or '",
        no_attachment_world, "'",
    ])

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(package_share) / "launch" / "piper_sim.launch.py")
        ),
        # The historical override was "robot_xacro": robot_xacro.  The
        # stabilization selector now owns this value so OFF cannot smuggle in
        # a second attachment backend.
        launch_arguments={
            "robot_xacro": effective_robot_xacro,
            "world": effective_world,
            "gazebo_executable": gazebo_executable,
        }.items(),
    )
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(package_share) / "launch" / "sim_moveit.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "start_robot_state_publisher": "false",
        }.items(),
        condition=IfCondition(start_moveit),
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
        condition=IfCondition(run_grasp_pipeline),
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
    sequence_parameters["seed"] = _parameter("seed", int)
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
            "grasp_assist_service": LaunchConfiguration("grasp_assist_service"),
        }
    )
    sequence_arguments = [
        "--execution-backend",
        "simulation",
        "--target-class",
        target_model,
        "--auto-latch",
        "--post-close-hold-s",
        LaunchConfiguration("post_close_hold_s"),
        "--auto-pause",
        LaunchConfiguration("auto_pause_s"),
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
            LaunchConfiguration("countdown_seconds"),
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

    grasp_assist = Node(
        package="foam_grasp_sim",
        executable="grasp_assist_node",
        name="foam_grasp_assist",
        output="screen",
        parameters=[
            {
                "target_entity": PythonExpression(["'foam_' + '", target_model, "'"]),
                "seed": _parameter("seed", int),
            }
        ],
        condition=IfCondition(
            PythonExpression([
                "'", grasp_stabilization_mode,
                "' == 'legacy_contact_confirmed' or '",
                grasp_assist_mode, "' == 'contact_confirmed'",
            ])
        ),
    )

    contact_diagnostics = Node(
        package="foam_grasp_sim",
        executable="contact_diagnostics_node",
        name="foam_contact_diagnostics",
        output="screen",
        parameters=[
            {
                "use_sim_time": execution["use_sim_time"],
                "target_entity": PythonExpression(["'foam_' + '", target_model, "'"]),
                "output_path": contact_diagnostics_output,
            }
        ],
        condition=IfCondition(record_contact_diagnostics),
    )

    simulation_readiness = Node(
        package="foam_grasp_sim",
        executable="simulation_readiness",
        name="foam_simulation_readiness",
        output="screen",
        arguments=["--timeout-s", simulation_readiness_timeout_s],
    )

    move_to_observe = Node(
        package="foam_grasp",
        executable="move_to_observe",
        name="foam_move_to_observe_sim",
        output="screen",
        arguments=[
            "--execution-backend",
            "simulation",
            "--execute",
            "--confirm",
            "AUTO_MOVE_TO_OBSERVE",
            "--countdown-seconds",
            "0",
        ],
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
                # JSON-looking launch values are otherwise parsed as YAML
                # mappings by launch_ros; the logger contract is a string.
                "condition_json": _parameter("condition_json", str),
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

    # The table is built into the world as a static model. Only the selected
    # target uses the Gazebo factory, avoiding a known static-model spawn queue
    # timeout while keeping target insertion serialized before the pipeline.
    # The readiness process below is the sole gate before this action list.
    def _after_readiness(event, context):
        del context
        if event.returncode != 0:
            return [
                Shutdown(
                    reason=(
                        "simulation readiness failed; refusing to start target "
                        f"scene and grasp pipeline (returncode={event.returncode})"
                    )
                )
            ]
        return [*target_spawns]

    def _pipeline_actions():
        return [
            *motion_nodes,
            simulated_perception,
            method_policy,
            benchmark_logger,
            grasp_pose_preview,
            grasp_assist,
            contact_diagnostics,
            plan_sequence,
            execute_sequence,
        ]

    def _after_target_spawn(event, context):
        returncode = event.returncode
        if returncode not in (None, 0):
            return [
                Shutdown(
                    reason=(
                        "target spawn failed; refusing to start the grasp pipeline "
                        f"(returncode={returncode})"
                    )
                )
            ]
        if context.perform_substitution(prepare_observation_pose).lower() == "true":
            return [move_to_observe]
        return _pipeline_actions()

    def _after_observation_pose(event, context):
        del context
        if event.returncode != 0:
            return [
                Shutdown(
                    reason=(
                        "observation pose preparation failed; refusing to start "
                        f"the grasp pipeline (returncode={event.returncode})"
                    )
                )
            ]
        return _pipeline_actions()

    pipeline_after_target_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=target_spawn,
                on_exit=_after_target_spawn,
            )
        )
        for target_spawn in target_spawns
    ]
    observation_pose_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=move_to_observe,
            on_exit=_after_observation_pose,
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument(
                "robot_xacro",
                default_value=str(
                    Path(package_share)
                    / "urdf"
                    / "piper_eye_in_hand_physics.xacro"
                ),
                description="Local physics wrapper forwarded to piper_sim.launch.py",
            ),
            DeclareLaunchArgument(
                "gazebo_executable",
                default_value="gzserver",
                description="Gazebo executable; use gazebo only when a GUI is available",
            ),
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
                "prepare_observation_pose",
                default_value="false",
                description="Move to the verified observation pose before the pipeline",
            ),
            DeclareLaunchArgument(
                "grasp_assist_mode",
                default_value="off",
                description="off or contact_confirmed",
            ),
            DeclareLaunchArgument(
                "grasp_stabilization_mode",
                default_value="off",
                description="off, gazebo_grasp_fix, or legacy_contact_confirmed",
            ),
            DeclareLaunchArgument(
                "grasp_assist_service",
                default_value="",
                description="Optional service used to attach after dual-finger contact",
            ),
            DeclareLaunchArgument(
                "record_contact_diagnostics",
                default_value="false",
                description="Record raw finger contact wrench samples",
            ),
            DeclareLaunchArgument(
                "contact_diagnostics_output",
                default_value="/tmp/foam_grasp_contact_diagnostics.csv",
                description="CSV output path for contact diagnostics",
            ),
            DeclareLaunchArgument(
                "post_close_hold_s",
                default_value="1.0",
                description="Seconds to hold the closed gripper before lift",
            ),
            DeclareLaunchArgument(
                "auto_pause_s",
                default_value="1.0",
                description="Seconds to let the arm settle at each automatic checkpoint",
            ),
            DeclareLaunchArgument(
                "countdown_seconds",
                default_value="0",
                description="Seconds to settle after planning before execution",
            ),
            DeclareLaunchArgument(
                "target_spawn_delay_s",
                default_value="12.0",
                description=(
                    "Deprecated compatibility argument; startup is gated by "
                    "simulation readiness rather than a fixed delay"
                ),
            ),
            DeclareLaunchArgument(
                "simulation_readiness_timeout_s",
                default_value="30.0",
                description="Wall-clock timeout for controller and action readiness",
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
            OpaqueFunction(function=_validate_stabilization_configuration),
            piper_launch,
            moveit_launch,
            simulation_readiness,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=simulation_readiness,
                    on_exit=_after_readiness,
                )
            ),
            observation_pose_handler,
            *pipeline_after_target_handlers,
        ]
    )
