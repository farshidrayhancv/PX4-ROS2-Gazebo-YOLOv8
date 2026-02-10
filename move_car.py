#!/usr/bin/env python3
"""Move a car model in a circle inside the Gazebo simulation."""

import math
import subprocess
import time

# Circle parameters
CENTER_X = 280.0
CENTER_Y = -140.0
Z = 2.6
RADIUS = 15.0
SPEED = 5.0  # m/s
OMEGA = SPEED / RADIUS  # angular velocity (rad/s)

MODEL_NAME = "hatchback_blue_1"
WORLD_NAME = "default"
UPDATE_HZ = 10


def set_pose(x, y, z, yaw):
    """Set model pose via gz service."""
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)

    req = (
        f'name: "{MODEL_NAME}", '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
        f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}'
    )

    subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '100', '--req', req],
        capture_output=True
    )


def main():
    print(f"Moving '{MODEL_NAME}' in a circle")
    print(f"  Center : ({CENTER_X}, {CENTER_Y})")
    print(f"  Radius : {RADIUS}m")
    print(f"  Speed  : {SPEED} m/s")
    print(f"  Period : {2 * math.pi * RADIUS / SPEED:.1f}s per lap")
    print("Press Ctrl+C to stop.\n")

    dt = 1.0 / UPDATE_HZ
    t0 = time.time()

    while True:
        t = time.time() - t0
        angle = OMEGA * t

        x = CENTER_X + RADIUS * math.cos(angle)
        y = CENTER_Y + RADIUS * math.sin(angle)
        # Yaw points tangent to the circle (direction of travel)
        yaw = angle + math.pi / 2

        set_pose(x, y, Z, yaw)
        time.sleep(dt)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
