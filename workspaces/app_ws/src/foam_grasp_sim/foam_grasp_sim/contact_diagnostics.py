"""Pure contact-wrench extraction helpers for Gazebo grasp diagnostics."""

import math


def _component(vector, name):
    return float(getattr(vector, name, 0.0))


def _vector_tuple(vector):
    return tuple(_component(vector, name) for name in ("x", "y", "z"))


def _contact_value(values, index, fallback):
    if index < len(values):
        return values[index]
    return fallback


def _wrench_force(wrench):
    return _vector_tuple(getattr(wrench, "force", None))


def _wrench_torque(wrench):
    return _vector_tuple(getattr(wrench, "torque", None))


def _project_force(force, normal):
    normal_length = math.sqrt(sum(value * value for value in normal))
    force_length = math.sqrt(sum(value * value for value in force))
    if normal_length <= 1e-12:
        return math.nan, force_length
    unit_normal = tuple(value / normal_length for value in normal)
    normal_force = abs(sum(f * n for f, n in zip(force, unit_normal)))
    tangential_squared = max(force_length * force_length - normal_force * normal_force, 0.0)
    return normal_force, math.sqrt(tangential_squared)


def extract_contact_rows(message, side, target_entity):
    """Return normalized rows for contacts involving ``target_entity``.

    The function deliberately accepts duck-typed ROS messages so it remains
    unit-testable without starting Gazebo or importing rclpy.
    """

    rows = []
    for state in getattr(message, "states", ()):
        collision1 = str(getattr(state, "collision1_name", ""))
        collision2 = str(getattr(state, "collision2_name", ""))
        if target_entity not in collision1 and target_entity not in collision2:
            continue

        wrenches = list(getattr(state, "wrenches", ()) or ())
        total_wrench = getattr(state, "total_wrench", None)
        positions = list(getattr(state, "contact_positions", ()) or ())
        normals = list(getattr(state, "contact_normals", ()) or ())
        depths = list(getattr(state, "depths", ()) or ())
        count = max(len(wrenches), len(positions), len(normals), len(depths), 1)
        for index in range(count):
            wrench = _contact_value(wrenches, index, total_wrench)
            force = _wrench_force(wrench)
            torque = _wrench_torque(wrench)
            position = _vector_tuple(_contact_value(positions, index, None))
            normal = _vector_tuple(_contact_value(normals, index, None))
            normal_force, tangential_force = _project_force(force, normal)
            depth = _contact_value(depths, index, math.nan)
            rows.append(
                {
                    "side": str(side),
                    "collision1": collision1,
                    "collision2": collision2,
                    "contact_index": index,
                    "position_x_m": position[0],
                    "position_y_m": position[1],
                    "position_z_m": position[2],
                    "normal_x": normal[0],
                    "normal_y": normal[1],
                    "normal_z": normal[2],
                    "depth_m": float(depth),
                    "force_x_N": force[0],
                    "force_y_N": force[1],
                    "force_z_N": force[2],
                    "torque_x_Nm": torque[0],
                    "torque_y_Nm": torque[1],
                    "torque_z_Nm": torque[2],
                    "normal_force_N": normal_force,
                    "tangential_force_N": tangential_force,
                }
            )
    return rows
