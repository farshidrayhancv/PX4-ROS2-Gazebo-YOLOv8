#!/usr/bin/env python3
"""Auto-takeoff: waits for drone ready, arms, takes off, flies to a
pre-defined waypoint (NED relative to home), then switches to HOLD mode
so the keyboard controller can take over.

Target waypoint was captured from a manual flight session:
  Gazebo ENU: (370.97, -57.77, 19.54)  →  ~15.7 m AGL, heading 170° ENU
  Spawn ENU:  (268.08, -128.22, 3.86)  →  yaw -0.7 rad (-40.1°)
  NED offset: North +70.5, East +102.9, Up 15.7 m, heading 280° NED

Connects on UDP 14540 (MAVSDK API port) so it doesn't conflict with
keyboard-mavsdk-test.py which uses 14550 (GCS port).
"""

import asyncio
import sys
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# Target position in NED (relative to home/spawn)
# Derived from Gazebo pose: drone at (370.97, -57.77, 19.54), spawn at (268.08, -128.22, 3.86)
TARGET_NORTH = -49.4   # meters  (Gazebo ΔY)
TARGET_EAST = 184.7    # meters  (Gazebo ΔX)
TARGET_ALT = 15.6      # meters AGL
TARGET_DOWN = -TARGET_ALT

# Heading: 170° Gazebo ENU → NED = 90° - 170° = -80° → 280°
TARGET_HEADING_DEG = 280.0

# How long to hold at waypoint before handing to HOLD mode
SETTLE_TIME = 5.0  # seconds

# Position tolerance for "arrived" check
POS_TOLERANCE = 3.0  # meters


def log(msg):
    sys.stdout.write(f"{msg}\r\n")
    sys.stdout.flush()


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    log("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            log("-- Connected!")
            break

    log("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log("-- Global position OK")
            break

    # Extra settling time for PX4 EKF to stabilize
    log("-- Waiting for EKF to settle...")
    await asyncio.sleep(5.0)

    # Set initial offboard setpoint BEFORE arming (PX4 requirement:
    # must have valid setpoint before entering offboard mode)
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, TARGET_HEADING_DEG))

    log("-- Arming...")
    await drone.action.arm()

    log("-- Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        log(f"-- Offboard failed: {e}, falling back to action.takeoff()")
        await drone.action.set_takeoff_altitude(TARGET_ALT)
        await drone.action.takeoff()
        return

    # Phase 1: Climb to target altitude above home first (safe vertical ascent)
    log(f"-- Phase 1: Climbing to {TARGET_ALT}m above home...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, TARGET_DOWN, TARGET_HEADING_DEG))

    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        if alt >= TARGET_ALT - 2.0:
            log(f"-- Altitude reached: {alt:.1f}m AGL")
            break

    # Phase 2: Fly to target waypoint (north + east translation at altitude)
    log(f"-- Phase 2: Flying to waypoint N={TARGET_NORTH}, E={TARGET_EAST}, "
        f"heading {TARGET_HEADING_DEG}°...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(TARGET_NORTH, TARGET_EAST, TARGET_DOWN,
                       TARGET_HEADING_DEG))

    # Wait until position reached (check NED position via odometry)
    check_count = 0
    async for odom in drone.telemetry.position_velocity_ned():
        n = odom.position.north_m
        e = odom.position.east_m
        d = odom.position.down_m
        dn = TARGET_NORTH - n
        de = TARGET_EAST - e
        dd = TARGET_DOWN - d
        dist = (dn**2 + de**2 + dd**2) ** 0.5
        check_count += 1
        if check_count % 20 == 0:
            log(f"   Distance to target: {dist:.1f}m "
                f"(N:{n:.1f} E:{e:.1f} D:{d:.1f})")
        if dist < POS_TOLERANCE:
            log(f"-- Waypoint reached! (error: {dist:.1f}m)")
            break

    # Phase 3: Hold at waypoint to settle
    log(f"-- Phase 3: Holding at waypoint for {SETTLE_TIME}s...")
    settle_end = asyncio.get_event_loop().time() + SETTLE_TIME
    while asyncio.get_event_loop().time() < settle_end:
        await drone.offboard.set_position_ned(
            PositionNedYaw(TARGET_NORTH, TARGET_EAST, TARGET_DOWN,
                           TARGET_HEADING_DEG))
        await asyncio.sleep(0.1)

    # Hand back to HOLD mode so keyboard controller can take over
    log("-- Switching to HOLD mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        log(f"-- Offboard stop warning: {e}")

    log(f"-- Done! Drone at waypoint: N={TARGET_NORTH} E={TARGET_EAST} "
        f"alt={TARGET_ALT}m heading={TARGET_HEADING_DEG}° NED")
    log("   Keyboard controller can now fly manually.")

    # Keep script alive
    while True:
        await asyncio.sleep(1.0)


if __name__ == "__main__":
    asyncio.run(run())
