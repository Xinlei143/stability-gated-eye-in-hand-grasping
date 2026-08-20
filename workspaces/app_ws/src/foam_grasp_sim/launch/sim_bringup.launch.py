"""Compose Piper Gazebo, MoveIt planning and the static grasp scene."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
        ],
        condition=condition,
    )


def generate_launch_description():
    package_share = get_package_share_directory("foam_grasp_sim")
    piper_share = get_package_share_directory("piper_gazebo")
    use_rviz = LaunchConfiguration("use_rviz")
    target_model = LaunchConfiguration("target_model")
    start_executor = LaunchConfiguration("start_executor")
    execute_motion = LaunchConfiguration("execute_motion")
    executor = LaunchConfiguration("executor")
    simulation_config = str(Path(package_share) / "config" / "simulation.yaml")

    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(piper_share)
                / "launch"
                / "piper_with_gripper"
                / "piper_gazebo.launch.py"
            )
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

    table = _scene_spawn(package_share, "table", "grasp_table", (0.40, 0.0, 0.0))
    cube = _scene_spawn(
        package_share,
        "cube",
        "foam_cube",
        (0.40, 0.0, 0.055),
        IfCondition(PythonExpression(["'", target_model, "' == 'cube'"])),
    )
    cylinder = _scene_spawn(
        package_share,
        "cylinder",
        "foam_cylinder",
        (0.40, 0.16, 0.070),
        IfCondition(PythonExpression(["'", target_model, "' == 'cylinder'"])),
    )
    sphere = _scene_spawn(
        package_share,
        "sphere",
        "foam_sphere",
        (0.40, -0.16, 0.060),
        IfCondition(PythonExpression(["'", target_model, "' == 'sphere'"])),
    )

    executor_plan_node = Node(
        package="foam_grasp",
        executable=executor,
        name="foam_grasp_sim_executor",
        output="screen",
        parameters=[simulation_config],
        arguments=["--execution-backend", "simulation"],
        condition=IfCondition(
            PythonExpression(
                ["'", start_executor, "' == 'true' and '", execute_motion, "' == 'false'"]
            )
        ),
    )
    executor_execute_node = Node(
        package="foam_grasp",
        executable=executor,
        name="foam_grasp_sim_executor",
        output="screen",
        parameters=[simulation_config],
        arguments=[
            "--execution-backend",
            "simulation",
            "--execute",
            "--confirm",
            "AUTO_MOVE_TO_OBSERVE",
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", start_executor, "' == 'true' and '", execute_motion, "' == 'true'"]
            )
        ),
    )

    # The pinned Piper launch starts Gazebo itself and does not expose a world
    # argument.  Spawn the package-owned static assets after Gazebo has loaded.
    scene = TimerAction(
        period=5.0,
        actions=[table, cube, cylinder, sphere],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("target_model", default_value="cube"),
            DeclareLaunchArgument("start_executor", default_value="false"),
            DeclareLaunchArgument("execute_motion", default_value="false"),
            DeclareLaunchArgument("executor", default_value="move_to_observe"),
            piper_launch,
            moveit_launch,
            scene,
            executor_plan_node,
            executor_execute_node,
        ]
    )
