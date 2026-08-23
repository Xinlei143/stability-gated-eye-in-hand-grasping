"""Render a Xacro description with a local Gazebo PID parameter overlay."""

import argparse
import sys
import xml.etree.ElementTree as ET


def inject_pid_parameters(xml_text, parameter_path):
    """Append one local PID parameter file to each Gazebo control plugin."""

    root = ET.fromstring(xml_text)
    parameter_path = str(parameter_path)
    # Use the force-producing position-PID path for the complete simulated
    # mechanism.  The simulation-only startup hold keeps the arm at its
    # collision-free equilibrium until the grasp sequence preempts it.
    pid_joint_names = {
        "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6", "joint7", "joint8",
    }
    for control in root.iter("ros2_control"):
        for joint in control.findall("joint"):
            joint_name = joint.attrib.get("name")
            if joint_name not in pid_joint_names:
                continue
            for command in joint.findall("command_interface"):
                if command.attrib.get("name") == "position":
                    # gazebo_ros2_control's position_pid mode converts the
                    # position target into a force through the loaded PID.
                    command.attrib["name"] = "position_pid"
            state_names = {
                element.attrib.get("name")
                for element in joint.findall("state_interface")
            }
            if "effort" not in state_names:
                ET.SubElement(joint, "state_interface", {"name": "effort"})
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
