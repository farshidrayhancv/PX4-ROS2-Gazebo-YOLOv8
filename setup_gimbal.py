#!/usr/bin/env python3
"""Modify x500_depth model SDF to replace fixed camera joint with a 3-axis gimbal.

Run during Docker build to add pitch/yaw/roll gimbal joints and
JointPositionController plugins so the camera angle can be controlled
at runtime via gz topics.
"""

import xml.etree.ElementTree as ET

MODEL_PATH = "/root/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf"

# Camera mount point on the drone body
MOUNT_X = 0.15
MOUNT_Y = 0.029
MOUNT_Z = 0.21

# Initial pitch angle (radians) — 45 degrees downward
INITIAL_PITCH = 0.7854


def add_link(model, name, relative_to, pose_text):
    """Add a lightweight link for a gimbal axis."""
    link = ET.SubElement(model, "link", name=name)
    pose = ET.SubElement(link, "pose")
    pose.set("relative_to", relative_to)
    pose.text = pose_text
    inertial = ET.SubElement(link, "inertial")
    mass = ET.SubElement(inertial, "mass")
    mass.text = "0.01"
    inertia = ET.SubElement(inertial, "inertia")
    for tag in ["ixx", "iyy", "izz"]:
        ET.SubElement(inertia, tag).text = "0.00001"
    for tag in ["ixy", "ixz", "iyz"]:
        ET.SubElement(inertia, tag).text = "0"
    return link


def add_revolute_joint(model, name, parent, child, axis_xyz,
                       lower, upper, effort=0.3, damping=3.0):
    """Add a revolute joint between two links."""
    joint = ET.SubElement(model, "joint", name=name, type="revolute")
    ET.SubElement(joint, "parent").text = parent
    ET.SubElement(joint, "child").text = child
    axis = ET.SubElement(joint, "axis")
    ET.SubElement(axis, "xyz").text = axis_xyz
    limit = ET.SubElement(axis, "limit")
    ET.SubElement(limit, "lower").text = str(lower)
    ET.SubElement(limit, "upper").text = str(upper)
    ET.SubElement(limit, "effort").text = str(effort)
    dynamics = ET.SubElement(axis, "dynamics")
    ET.SubElement(dynamics, "damping").text = str(damping)
    return joint


def add_controller(model, joint_name, topic, initial=0):
    """Add a JointPositionController plugin."""
    ctrl = ET.SubElement(model, "plugin",
                         filename="gz-sim-joint-position-controller-system",
                         name="gz::sim::systems::JointPositionController")
    ET.SubElement(ctrl, "joint_name").text = joint_name
    ET.SubElement(ctrl, "topic").text = topic
    ET.SubElement(ctrl, "p_gain").text = "1.0"
    ET.SubElement(ctrl, "i_gain").text = "0"
    ET.SubElement(ctrl, "d_gain").text = "0.1"
    ET.SubElement(ctrl, "cmd_max").text = "0.3"
    ET.SubElement(ctrl, "cmd_min").text = "-0.3"
    ET.SubElement(ctrl, "initial_position").text = str(initial)
    return ctrl


def main():
    tree = ET.parse(MODEL_PATH)
    root = tree.getroot()
    model = root.find("model")

    # Remove the existing fixed CameraJoint
    for joint in model.findall("joint"):
        if joint.get("name") == "CameraJoint":
            model.remove(joint)
            break

    # Update OakD-Lite include pose (remove tilt, gimbal handles it)
    for include in model.findall("include"):
        uri = include.find("uri")
        if uri is not None and "OakD" in uri.text:
            pose = include.find("pose")
            if pose is not None:
                pose.text = f"{MOUNT_X} {MOUNT_Y} {MOUNT_Z} 0 0 0"
            break

    # Chain: base_link → (yaw) → gimbal_yaw_link → (roll) → gimbal_roll_link → (pitch) → camera_link
    mount_pose = f"{MOUNT_X} {MOUNT_Y} {MOUNT_Z} 0 0 0"

    add_link(model, "gimbal_yaw_link", "base_link", mount_pose)
    add_link(model, "gimbal_roll_link", "gimbal_yaw_link", "0 0 0 0 0 0")

    add_revolute_joint(model, "gimbal_yaw_joint", "base_link", "gimbal_yaw_link",
                       axis_xyz="0 0 1", lower=-1.57, upper=1.57)
    add_revolute_joint(model, "gimbal_roll_joint", "gimbal_yaw_link", "gimbal_roll_link",
                       axis_xyz="1 0 0", lower=-0.785, upper=0.785)
    add_revolute_joint(model, "gimbal_pitch_joint", "gimbal_roll_link", "camera_link",
                       axis_xyz="0 1 0", lower=-1.57, upper=0.5)

    add_controller(model, "gimbal_yaw_joint", "/gimbal/cmd_yaw", initial=0)
    add_controller(model, "gimbal_roll_joint", "/gimbal/cmd_roll", initial=0)
    add_controller(model, "gimbal_pitch_joint", "/gimbal/cmd_pitch", initial=INITIAL_PITCH)

    ET.indent(tree, space="  ")
    tree.write(MODEL_PATH, xml_declaration=True, encoding="UTF-8")
    print(f"3-axis gimbal added to {MODEL_PATH}")
    print(f"  Initial pitch: {INITIAL_PITCH} rad ({INITIAL_PITCH * 180 / 3.14159:.1f} deg)")
    print(f"  Topics: /gimbal/cmd_pitch, /gimbal/cmd_yaw, /gimbal/cmd_roll")


if __name__ == "__main__":
    main()
