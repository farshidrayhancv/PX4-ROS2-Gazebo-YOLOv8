import asyncio
import math
import sys
from mavsdk import System
from mavsdk.gimbal import GimbalMode
from pymavlink import mavutil
import KeyPressModule as kp

kp.init()
drone = System()

roll, pitch, throttle, yaw = 0, 0, 0, 0
is_connected = False

# Gimbal state (x500_gimbal model: negative pitch = camera down)
gimbal_pitch = -0.7854  # initial 45 deg down
gimbal_yaw = 0.0
gimbal_roll = 0.0
GIMBAL_SPEED = 0.6  # rad/sec — smooth continuous motion
GIMBAL_PITCH_MIN = -2.356  # -135 deg (max downward)
GIMBAL_PITCH_MAX = 0.785   # +45 deg (max upward)
GIMBAL_YAW_MIN = -3.14
GIMBAL_YAW_MAX = 3.14
GIMBAL_ROLL_MIN = -0.785
GIMBAL_ROLL_MAX = 0.785

# Gimbal velocity (set by keyboard handler, applied by gimbal_publisher)
gimbal_vel_pitch = 0.0
gimbal_vel_yaw = 0.0
gimbal_vel_roll = 0.0

# ArduCopter flight mode mapping (key -> mode name)
ARDUPILOT_MODES = {
    '1': 'LOITER',
    '2': 'ALT_HOLD',
    '3': 'GUIDED',
    '4': 'STABILIZE',
    '5': 'LAND',
}

# pymavlink connection for mode changes (separate MAVLink stream)
_mav_conn = None


def log(msg):
    sys.stdout.write(f"{msg}\r\n")
    sys.stdout.flush()


def get_mav_connection():
    """Get or create pymavlink connection for flight mode changes."""
    global _mav_conn
    if _mav_conn is None:
        _mav_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14560')
        _mav_conn.wait_heartbeat(timeout=10)
        log("-- pymavlink connected for mode switching")
    return _mav_conn


def set_flight_mode(mode_name):
    """Set ArduCopter flight mode via pymavlink."""
    try:
        conn = get_mav_connection()
        if mode_name not in conn.mode_mapping():
            log(f"-- Unknown mode: {mode_name}")
            return
        mode_id = conn.mode_mapping()[mode_name]
        conn.set_mode(mode_id)
        log(f"-- Mode: {mode_name}")
    except Exception as e:
        log(f"-- Mode change failed: {e}")


async def gimbal_publisher():
    """Continuously apply gimbal velocity and publish via MAVSDK (through PX4).

    Runs at 20 Hz. Key presses set gimbal_vel_* (velocity); this task
    integrates velocity -> position and sends the command via MAVSDK's gimbal
    API, which routes through PX4 for attitude-stabilized gimbal control.
    """
    global gimbal_pitch, gimbal_yaw, gimbal_roll
    dt = 0.05  # 20 Hz

    try:
        await drone.gimbal.set_mode(GimbalMode.YAW_FOLLOW)
        log("-- Gimbal mode: YAW_FOLLOW")
    except Exception as e:
        log(f"-- Gimbal mode set failed (non-fatal): {e}")

    while True:
        changed = False
        if gimbal_vel_pitch != 0:
            gimbal_pitch = max(GIMBAL_PITCH_MIN, min(GIMBAL_PITCH_MAX,
                               gimbal_pitch + gimbal_vel_pitch * dt))
            changed = True
        if gimbal_vel_yaw != 0:
            gimbal_yaw = max(GIMBAL_YAW_MIN, min(GIMBAL_YAW_MAX,
                             gimbal_yaw + gimbal_vel_yaw * dt))
            changed = True
        if gimbal_vel_roll != 0:
            gimbal_roll = max(GIMBAL_ROLL_MIN, min(GIMBAL_ROLL_MAX,
                              gimbal_roll + gimbal_vel_roll * dt))
            changed = True
        if changed:
            try:
                await drone.gimbal.set_pitch_and_yaw(
                    math.degrees(gimbal_pitch),
                    math.degrees(gimbal_yaw))
            except Exception:
                pass  # Don't spam errors during rapid key input
        await asyncio.sleep(dt)


async def getKeyboardInput(my_drone):
    global roll, pitch, throttle, yaw
    global gimbal_vel_pitch, gimbal_vel_yaw, gimbal_vel_roll
    while True:
        roll, pitch, throttle, yaw = 0, 0, 0.5, 0
        value = 0.75

        # Flight controls
        if kp.getKey("LEFT"):
            pitch = -value
        elif kp.getKey("RIGHT"):
            pitch = value
        if kp.getKey("UP"):
            roll = value
        elif kp.getKey("DOWN"):
            roll = -value
        if kp.getKey("w"):
            throttle = 0.8
        elif kp.getKey("s"):
            throttle = 0.2
        if kp.getKey("a"):
            yaw = -value
        elif kp.getKey("d"):
            yaw = value
        elif kp.getKey("i"):
            asyncio.ensure_future(print_flight_mode(my_drone))
        elif kp.getKey("r"):
            try:
                await my_drone.action.arm()
                log("-- Armed!")
            except Exception as e:
                log(f"-- Arm failed: {e}")
        elif kp.getKey("l"):
            try:
                await my_drone.action.land()
                log("-- Landing!")
            except Exception as e:
                log(f"-- Land failed: {e}")

        # Gimbal controls — set velocity, gimbal_publisher applies it smoothly
        gimbal_vel_pitch = (-GIMBAL_SPEED if kp.getKey("j") else
                            GIMBAL_SPEED if kp.getKey("k") else 0.0)
        # Yaw sign inverted due to gimbal 180° mount offset (pose yaw=3.14)
        gimbal_vel_yaw = (GIMBAL_SPEED if kp.getKey("n") else
                          -GIMBAL_SPEED if kp.getKey("m") else 0.0)
        gimbal_vel_roll = (-GIMBAL_SPEED if kp.getKey("u") else
                           GIMBAL_SPEED if kp.getKey("o") else 0.0)

        # Flight mode switching (ArduCopter)
        for key, mode in ARDUPILOT_MODES.items():
            if kp.getKey(key):
                set_flight_mode(mode)
                break

        await asyncio.sleep(0.1)


async def print_flight_mode(my_drone):
    async for flight_mode in my_drone.telemetry.flight_mode():
        log(f"FlightMode: {flight_mode}")


async def manual_control_drone(my_drone):
    global roll, pitch, throttle, yaw
    while True:
        await my_drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
        await asyncio.sleep(0.1)


async def run_drone():
    global is_connected
    asyncio.ensure_future(getKeyboardInput(drone))
    await drone.connect(system_address="udpin://0.0.0.0:14550")
    log("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            log("-- Connected to drone!")
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log("-- Global position state is good enough for flying.")
            break
    is_connected = True

    # Send initial manual control inputs before arming (ArduCopter requirement)
    log("-- Sending initial manual control inputs...")
    for _ in range(20):
        await drone.manual_control.set_manual_control_input(0, 0, 0, 0)
        await asyncio.sleep(0.05)

    # Set initial gimbal pitch (45 deg down) via MAVSDK (through PX4)
    try:
        await drone.gimbal.set_pitch_and_yaw(
            math.degrees(gimbal_pitch), math.degrees(gimbal_yaw))
    except Exception as e:
        log(f"-- Initial gimbal set failed (non-fatal): {e}")

    log("-- Ready! Press 'r' to arm the drone.")
    asyncio.ensure_future(manual_control_drone(drone))
    asyncio.ensure_future(gimbal_publisher())


async def run():
    global roll, pitch, throttle, yaw
    """Main function to connect to the drone and input manual controls"""
    await asyncio.gather(run_drone())


if __name__ == "__main__":
    asyncio.ensure_future(run())
    asyncio.get_event_loop().run_forever()
