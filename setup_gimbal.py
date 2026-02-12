#!/usr/bin/env python3
"""Modify the x500_gimbal's gimbal model for keyboard-controlled operation.

1. Renames JointPositionController sub_topics from 'command/gimbal_*' to
   'manual/gimbal_*' so PX4's gz_bridge gimbal handler (which publishes to
   'command/gimbal_*') doesn't override keyboard commands.
2. Adds a thermal camera sensor co-located with the RGB camera.

Run during Docker build after PX4 is built.
"""

import xml.etree.ElementTree as ET

MODEL_PATH = "/root/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf"

# Rename gimbal topics: PX4 publishes to 'command/*' (ignored),
# keyboard publishes to 'manual/*' (actually controls the joints)
TOPIC_RENAMES = {
    "command/gimbal_pitch": "manual/gimbal_pitch",
    "command/gimbal_roll": "manual/gimbal_roll",
    "command/gimbal_yaw": "manual/gimbal_yaw",
}


def rename_gimbal_topics(model):
    """Rename JointPositionController sub_topics to avoid PX4 override."""
    count = 0
    for plugin in model.findall("plugin"):
        name = plugin.get("name", "")
        if "JointPositionController" not in name:
            continue
        sub_topic = plugin.find("sub_topic")
        if sub_topic is not None and sub_topic.text in TOPIC_RENAMES:
            old = sub_topic.text
            sub_topic.text = TOPIC_RENAMES[old]
            count += 1
            print(f"  Renamed: {old} -> {sub_topic.text}")
    return count


# PID tuning: increase responsiveness for smoother tracking of continuous
# position commands.  Stock values are very conservative (P≈0.3-0.8,
# cmd_max=0.3) which causes visible lag when the keyboard sends small
# incremental steps.
PID_OVERRIDES = {
    "p_gain": "2.0",
    "d_gain": "0.1",
    "cmd_max": "1.0",
    "cmd_min": "-1.0",
}


def tune_pid_gains(model):
    """Increase PID responsiveness on JointPositionController plugins."""
    count = 0
    for plugin in model.findall("plugin"):
        name = plugin.get("name", "")
        if "JointPositionController" not in name:
            continue
        for tag, value in PID_OVERRIDES.items():
            elem = plugin.find(tag)
            if elem is not None:
                elem.text = value
            else:
                elem = ET.SubElement(plugin, tag)
                elem.text = value
        count += 1
    return count


def add_thermal_sensor(camera_link):
    """Add a thermal camera sensor to the gimbal's camera_link."""
    # Check if thermal sensor already exists
    for sensor in camera_link.findall("sensor"):
        if sensor.get("name") == "thermal_camera":
            print("  Thermal camera sensor already exists, skipping.")
            return False

    sensor = ET.SubElement(camera_link, "sensor")
    sensor.set("name", "thermal_camera")
    sensor.set("type", "thermal")

    pose = ET.SubElement(sensor, "pose")
    pose.text = "-0.0412 0 -0.162 0 0 3.14"

    gz_frame = ET.SubElement(sensor, "gz_frame_id")
    gz_frame.text = "camera_link"

    camera = ET.SubElement(sensor, "camera")

    hfov = ET.SubElement(camera, "horizontal_fov")
    hfov.text = "1.047"

    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = "320"
    ET.SubElement(image, "height").text = "240"
    ET.SubElement(image, "format").text = "L16"

    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "100"

    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = "10"
    ET.SubElement(sensor, "visualize").text = "true"

    return True


def main():
    tree = ET.parse(MODEL_PATH)
    root = tree.getroot()
    model = root.find("model")

    # 1. Rename gimbal topics to prevent PX4 override
    print(f"Renaming gimbal topics in {MODEL_PATH}:")
    n = rename_gimbal_topics(model)
    print(f"  {n} topics renamed")

    # 2. Tune PID for smoother, more responsive gimbal tracking
    n = tune_pid_gains(model)
    print(f"  {n} JointPositionControllers tuned (P={PID_OVERRIDES['p_gain']}, "
          f"D={PID_OVERRIDES['d_gain']}, cmd_max={PID_OVERRIDES['cmd_max']})")

    # 3. Add thermal camera
    camera_link = None
    for link in model.findall("link"):
        if link.get("name") == "camera_link":
            camera_link = link
            break

    if camera_link is None:
        print(f"ERROR: camera_link not found in {MODEL_PATH}")
    else:
        if add_thermal_sensor(camera_link):
            print(f"  Thermal camera added (320x240, L16, 10 Hz)")

    ET.indent(tree, space="  ")
    tree.write(MODEL_PATH, xml_declaration=True, encoding="UTF-8")


if __name__ == "__main__":
    main()
