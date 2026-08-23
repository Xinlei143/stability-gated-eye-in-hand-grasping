"""Render a Xacro description with a local Gazebo PID parameter overlay."""

import argparse
import copy
import sys
import xml.etree.ElementTree as ET


ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
GRIPPER_JOINTS = ("joint7", "joint8")
ALL_CONTROLLED_JOINTS = set(ARM_JOINTS + GRIPPER_JOINTS)


def _control_systems(root):
    return [element for element in root.iter("ros2_control")]


def _joint_map(control):
    joints = {}
    for joint in control.findall("joint"):
        name = joint.attrib.get("name")
        if not name or name in joints:
            raise ValueError(f"duplicate or unnamed ros2_control joint: {name!r}")
        joints[name] = joint
    return joints


def _configure_arm_system(control):
    control.attrib["name"] = "PiperArmSystem"
    joints = _joint_map(control)
    for name in list(joints):
        if name not in ARM_JOINTS:
            control.remove(joints[name])
    for joint in control.findall("joint"):
        for command in joint.findall("command_interface"):
            if command.attrib.get("name") == "position_pid":
                command.attrib["name"] = "position"
        for state in list(joint.findall("state_interface")):
            if state.attrib.get("name") == "effort":
                joint.remove(state)


def _configure_gripper_system(control):
    control.attrib["name"] = "PiperGripperSystem"
    joints = _joint_map(control)
    for name in list(joints):
        if name not in GRIPPER_JOINTS:
            control.remove(joints[name])
    for joint in control.findall("joint"):
        for command in joint.findall("command_interface"):
            if command.attrib.get("name") == "position":
                command.attrib["name"] = "position_pid"
        state_names = {
            state.attrib.get("name") for state in joint.findall("state_interface")
        }
        if "effort" not in state_names:
            ET.SubElement(joint, "state_interface", {"name": "effort"})


def _validate_split_systems(controls):
    by_name = {control.attrib.get("name"): control for control in controls}
    if set(by_name) != {"PiperArmSystem", "PiperGripperSystem"}:
        raise ValueError("ros2_control must contain exactly the Piper arm and gripper systems")
    arm_joints = _joint_map(by_name["PiperArmSystem"])
    gripper_joints = _joint_map(by_name["PiperGripperSystem"])
    if set(arm_joints) != set(ARM_JOINTS) or set(gripper_joints) != set(GRIPPER_JOINTS):
        raise ValueError("split ros2_control systems contain an invalid joint set")
    if set(arm_joints) & set(gripper_joints):
        raise ValueError("a ros2_control joint appears in both Piper systems")
    for joint in arm_joints.values():
        if any(
            command.attrib.get("name") != "position"
            for command in joint.findall("command_interface")
        ):
            raise ValueError("PiperArmSystem must use raw position command interfaces")
    for joint in gripper_joints.values():
        if any(
            command.attrib.get("name") != "position_pid"
            for command in joint.findall("command_interface")
        ):
            raise ValueError("PiperGripperSystem must use position_pid command interfaces")


def _split_control_system(root):
    controls = _control_systems(root)
    if not controls:
        return
    split_names = {control.attrib.get("name") for control in controls}
    if split_names & {"PiperArmSystem", "PiperGripperSystem"}:
        _validate_split_systems(controls)
        return
    if len(controls) != 1:
        raise ValueError("robot description must contain one unsplit ros2_control system")
    original = controls[0]
    joints = _joint_map(original)
    if set(joints) != ALL_CONTROLLED_JOINTS:
        raise ValueError(
            "unsplit ros2_control must contain exactly joint1 through joint8"
        )
    arm = copy.deepcopy(original)
    gripper = copy.deepcopy(original)
    _configure_arm_system(arm)
    _configure_gripper_system(gripper)

    parent = next(
        (candidate for candidate in root.iter() if original in list(candidate)),
        None,
    )
    if parent is None:
        raise ValueError("unable to locate ros2_control parent element")
    index = list(parent).index(original)
    parent.remove(original)
    parent.insert(index, arm)
    parent.insert(index + 1, gripper)
    _validate_split_systems([arm, gripper])


def inject_pid_parameters(xml_text, parameter_path):
    """Split Piper control resources and append the local PID parameter file."""

    root = ET.fromstring(xml_text)
    parameter_path = str(parameter_path)
    _split_control_system(root)
    matched = 0
    for plugin in root.iter("plugin"):
        if plugin.attrib.get("filename") != "libgazebo_ros2_control.so":
            continue
        matched += 1
        parameter_values = [
            str(element.text or "")
            for element in plugin.findall("parameters")
        ]
        if parameter_path not in parameter_values:
            ET.SubElement(plugin, "parameters").text = parameter_path
    if matched == 0:
        raise ValueError("robot description has no gazebo_ros2_control plugin")
    return ET.tostring(root, encoding="unicode")


def render_description(xacro_path, parameter_path):
    import xacro

    document = xacro.process_file(str(xacro_path))
    return inject_pid_parameters(document.toxml(), parameter_path)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("xacro_path")
    parser.add_argument("parameter_path")
    args = parser.parse_args(argv)
    sys.stdout.write(render_description(args.xacro_path, args.parameter_path))
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
