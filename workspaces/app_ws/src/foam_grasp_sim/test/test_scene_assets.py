import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


class SceneAssetTest(unittest.TestCase):
    def test_simulation_contract_is_parameterized(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        execution = config["execution"]
        self.assertEqual(execution["execution_backend"], "simulation")
        self.assertEqual(
            execution["arm_trajectory_action"],
            "/arm_controller/follow_joint_trajectory",
        )
        self.assertEqual(
            execution["gripper_trajectory_action"],
            "/gripper_controller/follow_joint_trajectory",
        )
        self.assertEqual(
            execution["gripper8_trajectory_action"],
            "/gripper8_controller/follow_joint_trajectory",
        )
        self.assertEqual(execution["gripper_joint_name"], "joint7")
        self.assertEqual(execution["gripper8_joint_name"], "joint8")
        self.assertEqual(execution["gripper_command_scale"], 0.5)
        self.assertEqual(execution["gripper_feedback_scale"], 2.0)
        self.assertEqual(execution["final_joint_tolerance"], 0.05)
        self.assertEqual(execution["gripper_tolerance"], 0.004)

    def test_scene_config_aligns_table_and_target_bases(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        table = config["scene"]["table"]
        tabletop = table["pose"][2] + table["size"][2] / 2.0
        self.assertAlmostEqual(tabletop, 0.001)
        heights = {"cube": 0.050, "cylinder": 0.070, "sphere": 0.060}
        for name, height in heights.items():
            pose = config["scene"]["targets"][name]["pose"]
            self.assertAlmostEqual(pose[2] - height / 2.0, tabletop)
            self.assertEqual(pose[:2], [0.40, 0.00])

    def test_stage3_motion_and_perception_defaults_preserve_static_ideal_mode(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        self.assertEqual(config["motion"]["trajectory"], "static")
        self.assertEqual(config["motion"]["velocity"], [0.01, 0.00, 0.00])
        self.assertEqual(config["motion"]["seed"], 42)
        self.assertEqual(config["perception"]["source"], "ground_truth")
        self.assertEqual(config["perception"]["seed"], 42)

    def test_stage4_method_defaults_preserve_gated_behavior(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        method = config["method"]
        self.assertEqual(method["name"], "gated")
        self.assertEqual(method["stability_duration"], 5.0)
        self.assertEqual(method["position_spread_threshold"], 0.006)
        self.assertEqual(method["minimum_stable_samples"], 25)
        self.assertNotIn("center_error_threshold", method)
        self.assertNotIn("joint_error_threshold", method)
        self.assertEqual(method["tracking_max_updates"], 20)

    def test_piper_composition_owns_world_and_reuses_upstream_assets(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        piper = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        moveit = (PACKAGE_ROOT / "launch" / "sim_moveit.launch.py").read_text()
        self.assertIn("piper_sim.launch.py", bringup)
        self.assertNotIn("piper_gazebo.launch.py", bringup)
        self.assertIn("piper_description_gazebo.xacro", piper)
        self.assertIn("gazebo_ros2_control", piper)
        self.assertNotIn("joint8_ctrl.py", piper)
        self.assertIn("piper_eye_in_hand_physics.xacro", piper)
        self.assertIn("grasp_table.world", piper)
        for controller in (
            "joint_state_broadcaster",
            "arm_controller",
            "gripper_controller",
            "gripper8_controller",
        ):
            self.assertIn(controller, piper)
        self.assertNotIn("arm_startup_hold", piper)
        self.assertIn("allow_trajectory_execution", moveit)
        self.assertIn('"allow_trajectory_execution": False', moveit)
        self.assertNotIn("joint_states_single", moveit)

    def test_eye_in_hand_xacro_declares_rgbd_sensor_and_optical_frames(self):
        path = PACKAGE_ROOT / "urdf" / "piper_eye_in_hand_gazebo.xacro"
        self.assertNotIn("Piper's upstream Gazebo description plus", path.read_text())
        root = ET.parse(path).getroot()
        self.assertEqual(root.tag, "robot")
        include = root.find("{http://www.ros.org/wiki/xacro}include")
        self.assertIsNotNone(include)
        self.assertIn("piper_description_gazebo.xacro", include.attrib["filename"])

        links = {link.attrib["name"] for link in root.findall("link")}
        self.assertTrue(
            {
                "camera_link",
                "camera_color_frame",
                "camera_color_optical_frame",
                "camera_depth_frame",
                "camera_depth_optical_frame",
            }.issubset(links)
        )
        joints = {
            joint.attrib["name"]: (
                joint.find("parent").attrib["link"],
                joint.find("child").attrib["link"],
            )
            for joint in root.findall("joint")
        }
        self.assertEqual(joints["camera_mount_joint"], ("link6", "camera_link"))
        self.assertEqual(
            joints["camera_color_optical_joint"],
            ("camera_color_frame", "camera_color_optical_frame"),
        )
        self.assertEqual(
            joints["camera_depth_optical_joint"],
            ("camera_depth_frame", "camera_depth_optical_frame"),
        )

        sensor = next(
            sensor
            for sensor in root.findall("gazebo/sensor")
            if sensor.attrib.get("type") == "depth"
        )
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.attrib["type"], "depth")
        self.assertEqual(sensor.findtext("update_rate"), "15")
        self.assertEqual(sensor.findtext("camera/image/width"), "640")
        self.assertEqual(sensor.findtext("camera/image/height"), "360")
        self.assertIsNotNone(sensor.find("camera/depth_camera"))
        plugin = sensor.find("plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.attrib["filename"], "libgazebo_ros_camera.so")
        remappings = {
            item.text for item in plugin.findall("ros/remapping") if item.text
        }
        self.assertIn(
            "eye_in_hand_rgbd/image_raw:=color/image_raw", remappings
        )
        self.assertIn(
            "eye_in_hand_rgbd/depth/image_raw:=depth/image_raw", remappings
        )
        self.assertIn(
            "eye_in_hand_rgbd/camera_info:=color/camera_info", remappings
        )
        self.assertIn(
            "eye_in_hand_rgbd/depth/camera_info:=depth/camera_info", remappings
        )
        for finger in ("link7", "link8"):
            surface = root.find(f"gazebo[@reference='{finger}']")
            self.assertIsNotNone(surface)
            self.assertEqual(surface.findtext("mu1"), "1.0")
            self.assertEqual(surface.findtext("mu2"), "1.0")
            self.assertEqual(surface.findtext("maxVel"), "0.01")
            contact_sensor = surface.find("sensor")
            self.assertIsNotNone(contact_sensor)
            self.assertEqual(contact_sensor.attrib["type"], "contact")
            self.assertIsNotNone(contact_sensor.find("plugin"))

    def test_piper_launch_accepts_a_robot_xacro_override(self):
        piper = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        self.assertIn('"robot_xacro",', piper)
        self.assertIn('LaunchConfiguration("robot_xacro")', piper)
        self.assertIn("piper_eye_in_hand_physics.xacro", piper)
        self.assertIn("render_physics_description", piper)
        self.assertIn("get_package_prefix", piper)
        self.assertIn("ros2_controllers_physics.yaml", piper)
        self.assertIn("default_value=str(default_robot_xacro)", piper)

    def test_piper_launch_accepts_a_physics_pid_config_override(self):
        piper = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        self.assertIn('"physics_pid_config",', piper)
        self.assertIn('LaunchConfiguration("physics_pid_config")', piper)
        self.assertIn('default_value=str(physics_pid_config)', piper)

    def test_physics_xacro_uses_position_pid_for_both_fingers(self):
        path = PACKAGE_ROOT / "urdf" / "piper_eye_in_hand_physics.xacro"
        text = path.read_text()
        self.assertIn("piper_eye_in_hand_gazebo.xacro", text)
        self.assertIn("physics PID overlay", text)

    def test_physics_controller_config_declares_gripper_position_controllers(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "ros2_controllers_physics.yaml").read_text()
        )
        self.assertEqual(
            config["gripper_controller"]["ros__parameters"]["command_interfaces"],
            ["position"],
        )
        self.assertEqual(
            config["gripper8_controller"]["ros__parameters"]["command_interfaces"],
            ["position"],
        )
        pid = config["gazebo_ros2_control"]["ros__parameters"]["pid_gains"]["position_pid"]
        for name in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"):
            self.assertNotIn(name, pid)
        self.assertEqual(pid["joint7"]["ki"], 0.0)
        self.assertEqual(pid["joint8"]["ki"], 0.0)
        self.assertEqual(pid["joint7"]["kp"], 30.0)
        self.assertEqual(pid["joint8"]["kp"], 30.0)

    def test_grasp_fix_wrapper_is_separate_from_physics_only_wrapper(self):
        physics = (PACKAGE_ROOT / "urdf" / "piper_eye_in_hand_physics.xacro").read_text()
        wrapper = (PACKAGE_ROOT / "urdf" / "piper_eye_in_hand_grasp_fix.xacro").read_text()
        self.assertNotIn("libgazebo_grasp_fix.so", physics)
        for token in ("libgazebo_grasp_fix.so", "<palm_link>link6</palm_link>",
                      "<gripper_link>link7</gripper_link>", "<gripper_link>link8</gripper_link>",
                      "<forces_angle_tolerance>100</forces_angle_tolerance>",
                      "<update_rate>10</update_rate>",
                      "<grip_count_threshold>2</grip_count_threshold>",
                      "<max_grip_count>3</max_grip_count>",
                      "<release_tolerance>0.005</release_tolerance>",
                      "<disable_collisions_on_attach>false</disable_collisions_on_attach>"):
            self.assertIn(token, wrapper)

    def test_loaded_qualification_fixture_is_explicit_and_mode_is_supported(self):
        qualification = (PACKAGE_ROOT / "foam_grasp_sim" / "control_physics_qualification_node.py").read_text()
        fixture = (PACKAGE_ROOT / "urdf" / "piper_eye_in_hand_loaded_qualification.xacro").read_text()
        self.assertIn('"loaded_gripper"', qualification)
        self.assertIn("calibration_block", fixture)
        self.assertIn('<gazebo reference="calibration_block">', fixture)
        self.assertIn('<selfCollide>true</selfCollide>', fixture)
        self.assertIn('"velocity{index}"', qualification)
        self.assertIn('"effort{index}"', qualification)
        self.assertIn("<self_collide>true</self_collide>", fixture)
        self.assertIn("loaded_gripper", (PACKAGE_ROOT / "launch" / "control_physics_qualification.launch.py").read_text())

    def test_control_qualification_config_is_target_free_and_three_stage_ready(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "control_physics_qualification.yaml").read_text()
        )
        self.assertEqual(config["cycles"], 5)
        self.assertEqual(
            [item["name"] for item in config["arm"]["sequence"]],
            ["home", "pregrasp_like", "safe_pose", "return"],
        )
        self.assertEqual(config["gripper"]["openings_mm"], [70.0, 40.0, 70.0])

    def test_control_qualification_launch_does_not_spawn_a_target(self):
        launch = (
            PACKAGE_ROOT / "launch" / "control_physics_qualification.launch.py"
        ).read_text()
        self.assertIn("control_physics_qualification", launch)
        self.assertIn("piper_sim.launch.py", launch)
        self.assertNotIn("foam_cube", launch)
        self.assertNotIn("sim_bringup.launch.py", launch)

    def test_sim_bringup_forwards_robot_xacro_to_piper_launch(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn('LaunchConfiguration("robot_xacro")', bringup)
        self.assertIn("piper_eye_in_hand_physics.xacro", bringup)

    def test_contact_diagnostics_is_opt_in_and_has_a_csv_output_argument(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn('executable="contact_diagnostics_node"', bringup)
        self.assertIn('"use_sim_time": execution["use_sim_time"]', bringup)
        self.assertIn('"record_contact_diagnostics"', bringup)
        self.assertIn('"contact_diagnostics_output"', bringup)

    def test_target_spawn_failure_stops_pipeline_before_sequence_start(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn("event.returncode", bringup)
        self.assertIn("Shutdown", bringup)
        self.assertIn("plan_sequence", bringup)
        self.assertNotIn("TimerAction(period=10.0, actions=[plan_sequence, execute_sequence])", bringup)

    def test_readiness_success_is_the_only_scene_start_path(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn('executable="simulation_readiness"', bringup)
        self.assertIn('"simulation_readiness_timeout_s"', bringup)
        self.assertIn('default_value="30.0"', bringup)
        self.assertIn("target_action=simulation_readiness", bringup)
        self.assertIn("Shutdown(", bringup)
        self.assertIn("returncode != 0", bringup)
        self.assertIn("return [*target_spawns]", bringup)
        self.assertNotIn("scene = TimerAction(", bringup)
        self.assertNotIn("TimerAction(period=1.0, actions=[plan_sequence, execute_sequence])", bringup)
        failure_path = (
            bringup.split("def _after_readiness", 1)[1]
            .split("def _after_target_spawn", 1)[0]
            .split("if event.returncode != 0:", 1)[1]
            .split("return [*target_spawns]", 1)[0]
        )
        self.assertIn("Shutdown", failure_path)
        self.assertNotIn("target_spawns", failure_path)

    def test_observation_pose_is_an_optional_event_driven_gate(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn('"prepare_observation_pose"', bringup)
        self.assertIn('executable="move_to_observe"', bringup)
        self.assertIn("_after_observation_pose", bringup)
        self.assertIn("target_action=move_to_observe", bringup)
        self.assertIn("observation pose preparation failed", bringup)
        self.assertNotIn("TimerAction", bringup)

    def test_run_grasp_pipeline_false_disables_method_policy(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        method_policy = bringup.split("method_policy = Node", 1)[1].split(
            "pose_parameters =", 1
        )[0]
        self.assertIn('executable="method_policy_node"', method_policy)
        self.assertIn("condition=IfCondition(run_grasp_pipeline)", method_policy)

    def test_moveit_is_explicitly_gated_for_perception_only_runs(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn('start_moveit = LaunchConfiguration("start_moveit")', bringup)
        moveit_block = bringup.split("moveit_launch = IncludeLaunchDescription", 1)[1].split(
            "target_spawns =", 1
        )[0]
        self.assertIn('condition=IfCondition(start_moveit)', moveit_block)
        self.assertIn('DeclareLaunchArgument("start_moveit", default_value="true")', bringup)

    def test_physics_qualification_has_a_configurable_post_close_hold(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        sequence = (PACKAGE_ROOT.parent / "foam_grasp" / "foam_grasp" / "foam_cube_grasp_sequence.py").read_text()
        self.assertIn('"post_close_hold_s"', bringup)
        self.assertIn("--post-close-hold-s", sequence)
        self.assertIn('"auto_pause_s"', bringup)
        self.assertIn("--auto-pause", sequence)
        self.assertIn('"countdown_seconds"', bringup)
        self.assertIn("--countdown-seconds", bringup)

    def test_grasp_assist_is_opt_in_and_world_loads_attachment_plugin(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        world = (PACKAGE_ROOT / "worlds" / "grasp_table.world").read_text()
        self.assertIn('"grasp_assist_mode"', bringup)
        self.assertIn("grasp_assist_node", bringup)
        self.assertIn("libgazebo_model_attachment_plugin_lib.so", world)
        self.assertIn('"robot_xacro": robot_xacro', bringup)

    def test_benchmark_condition_json_is_forced_to_string(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn('_parameter("condition_json", str)', bringup)

    def test_eye_in_hand_urdf_is_installed_with_gazebo_plugins_dependency(self):
        setup = (PACKAGE_ROOT / "setup.py").read_text()
        package = (PACKAGE_ROOT / "package.xml").read_text()
        self.assertIn('"share/" + package_name + "/urdf"', setup)
        self.assertIn('glob("urdf/*.xacro")', setup)
        self.assertIn("<exec_depend>gazebo_plugins</exec_depend>", package)

    def test_table_model_matches_scene_config(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "simulation.yaml").read_text()
        )
        root = ET.parse(PACKAGE_ROOT / "models" / "table" / "model.sdf").getroot()
        model_size = [
            float(value)
            for value in root.findtext("model/link/collision/geometry/box/size").split()
        ]
        self.assertEqual(model_size, config["scene"]["table"]["size"])

    def test_all_static_scene_assets_are_valid_sdf(self):
        for path in sorted((PACKAGE_ROOT / "models").glob("*/model.sdf")):
            root = ET.parse(path).getroot()
            self.assertEqual(root.tag, "sdf")
            model = root.find("model")
            self.assertIsNotNone(model, path)
            self.assertIsNotNone(model.find("link/collision"), path)
            self.assertIsNotNone(model.find("link/visual"), path)
            if path.parent.name != "table":
                self.assertIsNotNone(model.find("link/inertial/mass"), path)
                self.assertIsNotNone(model.find("link/inertial/inertia"), path)
                self.assertIsNotNone(
                    model.find("link/collision/surface/friction"), path
                )

    def test_world_contains_static_grasp_table_with_physics_and_gravity(self):
        root = ET.parse(PACKAGE_ROOT / "worlds" / "grasp_table.world").getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.findtext("gravity"), "0 0 -9.81")
        self.assertEqual(world.findtext("physics/max_step_size"), "0.001")
        self.assertEqual(world.findtext("physics/real_time_update_rate"), "1000")
        table = world.find("model[@name='grasp_table']")
        self.assertIsNotNone(table)
        self.assertEqual(table.findtext("static"), "true")

    def test_bringup_exposes_stage3_motion_and_perception_pipeline(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        for value in (
            "target_model",
            "run_grasp_pipeline",
            "execute_motion",
            "trajectory",
            "perception_source",
            "target_motion_node",
            "simulated_perception_node",
            "rgbd",
            "grasp_pose_preview_node",
            "object_grasp_sequence",
            "AUTO_FULL_OBJECT_GRASP",
            "metrics_logger_node",
        ):
            self.assertIn(value, bringup)
        self.assertNotIn("start_executor", bringup)
        self.assertNotIn('executable="executor"', bringup)

    def test_grasp_sequence_receives_trial_seed(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn(
            'sequence_parameters["seed"] = _parameter("seed", int)',
            bringup,
        )

    def test_piper_ros2_control_uses_the_upstream_robot_state_publisher_name(self):
        launch = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        self.assertIn('name="robot_state_publisher"', launch)
        self.assertNotIn('name="foam_grasp_sim_robot_state_publisher"', launch)

    def test_piper_sim_defaults_to_headless_gzserver(self):
        launch = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        self.assertIn('"gazebo_executable"', launch)
        self.assertIn('default_value="gzserver"', launch)

    def test_controller_spawners_use_explicit_startup_service_timeouts(self):
        launch = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        for option, value in (
            ("--controller-manager-timeout", "60.0"),
            ("--service-call-timeout", "30.0"),
            ("--switch-timeout", "30.0"),
        ):
            self.assertIn(f'"{option}"', launch)
            self.assertIn(f'"{value}"', launch)

    def test_readiness_shutdown_guard_is_idempotent(self):
        readiness = (
            PACKAGE_ROOT / "foam_grasp_sim" / "simulation_readiness_node.py"
        ).read_text()
        self.assertIn("if rclpy.ok():", readiness)
        self.assertIn("rclpy.shutdown()", readiness)

    def test_piper_robot_description_is_explicitly_a_string_parameter(self):
        launch = (PACKAGE_ROOT / "launch" / "piper_sim.launch.py").read_text()
        self.assertIn("ParameterValue(robot_description, value_type=str)", launch)

    def test_world_loads_gazebo_state_plugin_for_target_ground_truth(self):
        world = (PACKAGE_ROOT / "worlds" / "grasp_table.world").read_text()
        self.assertIn('filename="libgazebo_ros_state.so"', world)
        self.assertIn("<namespace>/gazebo</namespace>", world)

    def test_target_spawn_is_serialized_before_motion_and_perception(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        self.assertIn("target_spawns", bringup)
        self.assertIn("target_action=target_spawn", bringup)

    def test_table_model_has_inertial_data_for_gazebo_factory(self):
        table = (PACKAGE_ROOT / "models" / "table" / "model.sdf").read_text()
        self.assertIn("<inertial>", table)
        self.assertIn("<mass>", table)


    def test_stage4_method_layer_is_wired_before_latch_and_execution(self):
        bringup = (PACKAGE_ROOT / "launch" / "sim_bringup.launch.py").read_text()
        setup = (PACKAGE_ROOT / "setup.py").read_text()
        sequence = (
            Path(__file__).parents[2]
            / "foam_grasp"
            / "foam_grasp"
            / "foam_cube_grasp_sequence.py"
        ).read_text()
        for method in ("snapshot", "tracking", "gated"):
            self.assertIn(method, bringup)
        for parameter in (
            "stability_duration",
            "position_spread_threshold",
            "minimum_stable_samples",
            "method_ready",
            "commit_method_target",
            "tracking_replan_threshold",
            "tracking_max_updates",
        ):
            self.assertIn(parameter, bringup)
        self.assertIn("method_policy_node", setup)
        self.assertIn("wait_for_method_ready", sequence)
        self.assertIn("commit_method_service", sequence)
        latch = (
            Path(__file__).parents[2]
            / "foam_grasp"
            / "foam_grasp"
            / "foam_target_latch_node.py"
        ).read_text()
        self.assertNotIn('self.method == "tracking"', latch)
        self.assertIn("message.header.stamp", latch)
        self.assertNotIn("requires trajectory:=static", bringup)

    def test_stage5_benchmark_contract_is_wired(self):
        setup = (PACKAGE_ROOT / "setup.py").read_text()
        logger = (PACKAGE_ROOT / "foam_grasp_sim" / "metrics_logger_node.py").read_text()
        events = (
            Path(__file__).parents[2]
            / "foam_grasp"
            / "foam_grasp"
            / "benchmark_events.py"
        ).read_text()
        self.assertIn("metrics_logger_node", setup)
        self.assertIn("metadata.json", logger)
        self.assertIn("states.csv", logger)
        self.assertIn("events.csv", logger)
        self.assertIn("metrics.json", logger)
        self.assertIn("self.run_dir.mkdir(parents=True, exist_ok=True)", logger)
        self.assertIn("schema_version", events)

    def test_python_nodes_install_into_ros2_lib_directory(self):
        setup_cfg = PACKAGE_ROOT / "setup.cfg"
        self.assertTrue(setup_cfg.exists())
        content = setup_cfg.read_text()
        self.assertIn("script_dir=$base/lib/foam_grasp_sim", content)
        self.assertIn("install_scripts=$base/lib/foam_grasp_sim", content)

    def test_static_source_uses_selected_base_frame_topic(self):
        source = (
            PACKAGE_ROOT / "foam_grasp_sim" / "static_target_source_node.py"
        ).read_text()
        self.assertIn('TARGET_MODELS = ("cube", "cylinder", "sphere")', source)
        self.assertIn('f"/foam_grasp/{self.target_model}_point_base"', source)
        self.assertIn('message.header.frame_id = self.base_frame', source)

    def test_stage3_nodes_preserve_existing_observation_interface(self):
        motion = (
            PACKAGE_ROOT / "foam_grasp_sim" / "target_motion_node.py"
        ).read_text()
        perception = (
            PACKAGE_ROOT / "foam_grasp_sim" / "simulated_perception_node.py"
        ).read_text()
        self.assertIn('"/foam_grasp_sim/target_ground_truth"', motion)
        self.assertIn('f"/foam_grasp/{self.target_model}_point_base"', perception)
        self.assertIn("SetEntityState", motion)
        self.assertIn("ModelStates", motion)


if __name__ == "__main__":
    unittest.main()
