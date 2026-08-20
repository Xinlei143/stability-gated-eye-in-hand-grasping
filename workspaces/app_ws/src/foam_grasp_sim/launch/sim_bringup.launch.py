"""One-launch static grasp pipeline for the Piper Gazebo simulation.

The scene is intentionally fixed to one static target.  Stage 3 concepts such
as target motion, camera perception, noise, delay and benchmarking do not
belong in this launch.
"""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    return table, targets, execution, pipeline


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


def generate_launch_description():
    package_share = get_package_share_directory("foam_grasp_sim")
    table, targets, execution, pipeline = _load_simulation_config(package_share)
    use_rviz = LaunchConfiguration("use_rviz")
    target_model = LaunchConfiguration("target_model")
    run_grasp_pipeline = LaunchConfiguration("run_grasp_pipeline")
    execute_motion = LaunchConfiguration("execute_motion")

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

    source_parameters = {
        "use_sim_time": execution["use_sim_time"],
        "target_model": target_model,
        "base_frame": pipeline["target_latch"]["base_frame"],
        "publish_rate": pipeline["target_publish_rate"],
    }
    source_parameters.update(
        {
            f"{model}_pose": targets[model]["pose"]
            for model in TARGET_MODELS
        }
    )
    static_target_source = Node(
        package="foam_grasp_sim",
        executable="static_target_source_node",
        name="foam_static_target_source",
        output="screen",
        parameters=[source_parameters],
        condition=IfCondition(run_grasp_pipeline),
    )

    latch_parameters = {"use_sim_time": execution["use_sim_time"]}
    latch_parameters.update(pipeline["target_latch"])
    target_latch = Node(
        package="foam_grasp",
        executable="target_latch_node",
        name="foam_target_latch",
        output="screen",
        parameters=[latch_parameters],
        condition=IfCondition(run_grasp_pipeline),
    )
    pose_parameters = {"use_sim_time": execution["use_sim_time"]}
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

    # Let Gazebo expose its factory service before static SDF entities are
    # spawned.  The source begins at the same point, after its target exists.
    scene = TimerAction(
        period=5.0,
        actions=[table_spawn, *target_spawns, static_target_source],
    )
    # The sequence itself waits for feedback, services and target samples.  A
    # short delay prevents its first checks racing controller initialisation.
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
                description="Start static target, latch, pose preview and sequence",
            ),
            DeclareLaunchArgument(
                "execute_motion",
                default_value="false",
                description="Drive controllers only after explicit opt-in",
            ),
            piper_launch,
            moveit_launch,
            target_latch,
            grasp_pose_preview,
            scene,
            sequence,
        ]
    )
