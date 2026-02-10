#!/usr/bin/env python3
"""Modify x500_depth model SDF to replace fixed camera joint with a 2-axis gimbal.

Run during Docker build to add pitch/yaw gimbal joints and JointPositionController
plugins so the camera angle can be controlled at runtime via gz topics.
"""

import xml.etree.ElementTree as ET

MODEL_PATH = "/root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf"

# Camera mount point on the drone body
MOUNT_X = 0.15
MOUNT_Y = 0.029
MOUNT_Z = 0.21

# Initial pitch angle (radians) — 45 degrees downward
INITIAL_PITCH = 0.7854


def main():
    tree = ET.parse(MODEL_PATH)
    root = tree.getroot()
    model = root.find("model")

    # --- Remove the existing fixed CameraJoint ---
    for joint in model.findall("joint"):
        if joint.get("name") == "CameraJoint":
            model.remove(joint)
            break

    # --- Update OakD-Lite include pose (remove tilt, gimbal handles it) ---
    for include in model.findall("include"):
        uri = include.find("uri")
        if uri is not None and "OakD" in uri.text:
            pose = include.find("pose")
            if pose is not None:
                pose.text = f"{MOUNT_X} {MOUNT_Y} {MOUNT_Z} 0 0 0"
            break

    # --- Add gimbal_link (intermediate link between body and camera) ---
    gimbal_link = ET.SubElement(model, "link", name="gimbal_link")
    gl_pose = ET.SubElement(gimbal_link, "pose")
    gl_pose.set("relative_to", "base_link")
    gl_pose.text = f"{MOUNT_X} {MOUNT_Y} {MOUNT_Z} 0 0 0"
    gl_inertial = ET.SubElement(gimbal_link, "inertial")
    gl_mass = ET.SubElement(gl_inertial, "mass")
    gl_mass.text = "0.01"
    gl_inertia = ET.SubElement(gl_inertial, "inertia")
    for tag in ["ixx", "iyy", "izz"]:
        el = ET.SubElement(gl_inertia, tag)
        el.text = "0.00001"
    for tag in ["ixy", "ixz", "iyz"]:
        el = ET.SubElement(gl_inertia, tag)
        el.text = "0"

    # --- Add yaw joint: base_link -> gimbal_link ---
    yaw_joint = ET.SubElement(model, "joint", name="gimbal_yaw_joint", type="revolute")
    ET.SubElement(yaw_joint, "parent").text = "base_link"
    ET.SubElement(yaw_joint, "child").text = "gimbal_link"
    yaw_axis = ET.SubElement(yaw_joint, "axis")
    ET.SubElement(yaw_axis, "xyz").text = "0 0 1"
    yaw_limit = ET.SubElement(yaw_axis, "limit")
    ET.SubElement(yaw_limit, "lower").text = "-1.57"
    ET.SubElement(yaw_limit, "upper").text = "1.57"
    ET.SubElement(yaw_limit, "effort").text = "0.3"
    yaw_dynamics = ET.SubElement(yaw_axis, "dynamics")
    ET.SubElement(yaw_dynamics, "damping").text = "3.0"

    # --- Add pitch joint: gimbal_link -> camera_link ---
    pitch_joint = ET.SubElement(model, "joint", name="gimbal_pitch_joint", type="revolute")
    ET.SubElement(pitch_joint, "parent").text = "gimbal_link"
    ET.SubElement(pitch_joint, "child").text = "camera_link"
    pitch_axis = ET.SubElement(pitch_joint, "axis")
    ET.SubElement(pitch_axis, "xyz").text = "0 1 0"
    pitch_limit = ET.SubElement(pitch_axis, "limit")
    ET.SubElement(pitch_limit, "lower").text = "-1.57"
    ET.SubElement(pitch_limit, "upper").text = "0.5"
    ET.SubElement(pitch_limit, "effort").text = "0.3"
    pitch_dynamics = ET.SubElement(pitch_axis, "dynamics")
    ET.SubElement(pitch_dynamics, "damping").text = "3.0"

    # --- Add JointPositionController for yaw (very low gains to avoid reaction torques) ---
    yaw_ctrl = ET.SubElement(model, "plugin",
                             filename="gz-sim-joint-position-controller-system",
                             name="gz::sim::systems::JointPositionController")
    ET.SubElement(yaw_ctrl, "joint_name").text = "gimbal_yaw_joint"
    ET.SubElement(yaw_ctrl, "topic").text = "/gimbal/cmd_yaw"
    ET.SubElement(yaw_ctrl, "p_gain").text = "1.0"
    ET.SubElement(yaw_ctrl, "i_gain").text = "0"
    ET.SubElement(yaw_ctrl, "d_gain").text = "0.1"
    ET.SubElement(yaw_ctrl, "cmd_max").text = "0.3"
    ET.SubElement(yaw_ctrl, "cmd_min").text = "-0.3"
    ET.SubElement(yaw_ctrl, "initial_position").text = "0"

    # --- Add JointPositionController for pitch (very low gains to avoid reaction torques) ---
    pitch_ctrl = ET.SubElement(model, "plugin",
                               filename="gz-sim-joint-position-controller-system",
                               name="gz::sim::systems::JointPositionController")
    ET.SubElement(pitch_ctrl, "joint_name").text = "gimbal_pitch_joint"
    ET.SubElement(pitch_ctrl, "topic").text = "/gimbal/cmd_pitch"
    ET.SubElement(pitch_ctrl, "p_gain").text = "1.0"
    ET.SubElement(pitch_ctrl, "i_gain").text = "0"
    ET.SubElement(pitch_ctrl, "d_gain").text = "0.1"
    ET.SubElement(pitch_ctrl, "cmd_max").text = "0.3"
    ET.SubElement(pitch_ctrl, "cmd_min").text = "-0.3"
    ET.SubElement(pitch_ctrl, "initial_position").text = str(INITIAL_PITCH)

    # --- Write modified SDF ---
    ET.indent(tree, space="  ")
    tree.write(MODEL_PATH, xml_declaration=True, encoding="UTF-8")
    print(f"Gimbal joints added to {MODEL_PATH}")
    print(f"  Initial pitch: {INITIAL_PITCH} rad ({INITIAL_PITCH * 180 / 3.14159:.1f} deg)")
    print(f"  Topics: /gimbal/cmd_pitch, /gimbal/cmd_yaw")


if __name__ == "__main__":
    main()
