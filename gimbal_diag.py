#!/usr/bin/env python3
"""Gimbal diagnostic: systematically move gimbal axes and verify
direction from image changes.

Run AFTER auto_takeoff is complete (drone hovering at viewing position).

Steps:
1. Command gimbal to neutral (0, 0) and capture reference image
2. Move pitch NEGATIVE by 0.3 rad, capture image, compute shift
3. Return to neutral
4. Move pitch POSITIVE by 0.3 rad, capture image, compute shift
5. Return to neutral
6. Move yaw NEGATIVE by 0.3 rad, capture image, compute shift
7. Return to neutral
8. Move yaw POSITIVE by 0.3 rad, capture image, compute shift
9. Print summary of axis→image shift mappings

The image shift is computed using template matching (cross-correlation)
between the reference and test images.
"""

import subprocess
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

GIMBAL_TOPIC_PITCH = '/model/x500_gimbal_0/command/gimbal_pitch'
GIMBAL_TOPIC_YAW = '/model/x500_gimbal_0/command/gimbal_yaw'
GZ_JOINT_STATE_TOPIC = '/world/default/model/x500_gimbal_0/joint_state'

STEP_SIZE = 0.3  # radians (~17 degrees)
SETTLE_TIME = 2.0  # seconds to wait for gimbal to reach target


def set_gimbal(topic, value):
    """Publish a gimbal position command via gz topic."""
    subprocess.run(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double',
         '-p', f'data: {value:.4f}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        timeout=5)


def get_joint_positions():
    """Read current joint positions from Gazebo transport."""
    try:
        result = subprocess.run(
            ['gz', 'topic', '-t', GZ_JOINT_STATE_TOPIC, '-e', '-n', '1'],
            capture_output=True, text=True, timeout=5)
        output = result.stdout
        joints = {}
        current_name = None
        in_axis = False
        for line in output.split('\n'):
            line = line.strip()
            if line.startswith('name:') and 'joint' in line.lower():
                import re
                m = re.search(r'"([^"]+)"', line)
                if m:
                    current_name = m.group(1)
                    in_axis = False
            if current_name and line.startswith('axis1'):
                in_axis = True
            if current_name and in_axis and line.startswith('position:'):
                import re
                m = re.search(r'position:\s*([0-9eE.+-]+)', line)
                if m:
                    joints[current_name] = float(m.group(1))
                    current_name = None
                    in_axis = False
        return joints
    except Exception as e:
        print(f"  [WARN] Failed to read joint state: {e}")
        return {}


class DiagCapture(Node):
    """Captures a single camera frame."""
    def __init__(self):
        super().__init__('gimbal_diag')
        self.frame = None
        self.sub = self.create_subscription(Image, 'camera', self._cb, 1)
        self.br = CvBridge()

    def _cb(self, msg):
        self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def capture(self, timeout=3.0):
        """Spin until a frame is available."""
        self.frame = None
        start = time.time()
        while self.frame is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.frame


def compute_shift(ref, test):
    """Compute (dx, dy) pixel shift between ref and test images using
    phase correlation (sub-pixel accurate).
    """
    ref_gray = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32)
    test_gray = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY).astype(np.float32)
    (dx, dy), _ = cv2.phaseCorrelate(ref_gray, test_gray)
    return dx, dy


def main():
    rclpy.init()
    node = DiagCapture()
    print("=" * 60)
    print("GIMBAL DIAGNOSTIC — Axis Direction Verification")
    print("=" * 60)

    # Check joint state
    print("\n[0] Reading joint state...")
    joints = get_joint_positions()
    if joints:
        for name, pos in joints.items():
            print(f"    {name}: {pos:.6f} rad ({pos*180/3.14159:.2f} deg)")
    else:
        print("    WARNING: No joint state data!")

    # Step 0: Neutral position
    print("\n[1] Moving gimbal to neutral (0, 0)...")
    set_gimbal(GIMBAL_TOPIC_PITCH, 0.0)
    set_gimbal(GIMBAL_TOPIC_YAW, 0.0)
    time.sleep(SETTLE_TIME)

    joints = get_joint_positions()
    if joints:
        print(f"    Joint state after neutral command:")
        for name, pos in joints.items():
            print(f"      {name}: {pos:.6f} rad ({pos*180/3.14159:.2f} deg)")

    ref = node.capture()
    if ref is None:
        print("ERROR: No camera frame received!")
        node.destroy_node()
        rclpy.shutdown()
        return
    h, w = ref.shape[:2]
    print(f"    Reference frame: {w}x{h}")

    results = []

    # Test each axis direction
    tests = [
        ("PITCH -0.3", GIMBAL_TOPIC_PITCH, -STEP_SIZE, GIMBAL_TOPIC_YAW, 0.0),
        ("PITCH +0.3", GIMBAL_TOPIC_PITCH, +STEP_SIZE, GIMBAL_TOPIC_YAW, 0.0),
        ("YAW -0.3",   GIMBAL_TOPIC_YAW,   -STEP_SIZE, GIMBAL_TOPIC_PITCH, 0.0),
        ("YAW +0.3",   GIMBAL_TOPIC_YAW,   +STEP_SIZE, GIMBAL_TOPIC_PITCH, 0.0),
    ]

    for i, (label, topic, value, other_topic, other_val) in enumerate(tests, 2):
        print(f"\n[{i}] Testing {label}...")
        set_gimbal(topic, value)
        set_gimbal(other_topic, other_val)
        time.sleep(SETTLE_TIME)

        # Read joint state
        joints = get_joint_positions()
        if joints:
            for name, pos in joints.items():
                print(f"    {name}: {pos:.6f} rad ({pos*180/3.14159:.2f} deg)")

        test_frame = node.capture()
        if test_frame is None:
            print(f"    SKIP: no frame")
            results.append((label, None, None))
        else:
            dx, dy = compute_shift(ref, test_frame)
            print(f"    Image shift: dx={dx:+.1f}px, dy={dy:+.1f}px")
            results.append((label, dx, dy))

            # Save frames for visual inspection
            cv2.imwrite(f'/tmp/gimbal_diag_{label.replace(" ", "_")}.jpg',
                        test_frame)

        # Return to neutral
        set_gimbal(GIMBAL_TOPIC_PITCH, 0.0)
        set_gimbal(GIMBAL_TOPIC_YAW, 0.0)
        time.sleep(SETTLE_TIME)

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY: Gimbal axis → Image shift mapping")
    print("=" * 60)
    for label, dx, dy in results:
        if dx is not None:
            primary = "horizontal" if abs(dx) > abs(dy) else "vertical"
            dir_x = "RIGHT" if dx > 0 else "LEFT"
            dir_y = "DOWN" if dy > 0 else "UP"
            print(f"  {label:12s} → dx={dx:+7.1f}px ({dir_x:5s}), "
                  f"dy={dy:+7.1f}px ({dir_y:4s}) | primary: {primary}")
        else:
            print(f"  {label:12s} → NO DATA")

    print("\nExpected for correct tracking:")
    print("  Car RIGHT of center → need NEGATIVE err_ang_x → gimbal yaw should shift image LEFT")
    print("  Car BELOW center    → need NEGATIVE err_ang_y → gimbal pitch should shift image UP")
    print("=" * 60)

    cv2.imwrite('/tmp/gimbal_diag_reference.jpg', ref)
    print(f"\nFrames saved to /tmp/gimbal_diag_*.jpg")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
