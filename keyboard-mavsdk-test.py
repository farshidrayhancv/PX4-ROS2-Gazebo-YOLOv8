import asyncio
import subprocess
import sys
from mavsdk import System
import KeyPressModule as kp

kp.init()
drone = System()

roll, pitch, throttle, yaw = 0, 0, 0.5, 0
is_connected = False

# Gimbal state
gimbal_pitch = 0.7854  # initial 45 deg down
gimbal_yaw = 0.0
GIMBAL_STEP = 0.03  # rad per tick (~1.7 deg)
GIMBAL_PITCH_MIN = -1.57
GIMBAL_PITCH_MAX = 0.5
GIMBAL_YAW_MIN = -1.57
GIMBAL_YAW_MAX = 1.57


def log(msg):
    sys.stdout.write(f"{msg}\r\n")
    sys.stdout.flush()


_gimbal_procs = {}

def set_gimbal(topic, value):
    # Don't spawn if previous command for this topic is still running
    prev = _gimbal_procs.get(topic)
    if prev is not None and prev.poll() is None:
        return
    _gimbal_procs[topic] = subprocess.Popen(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double',
         '-p', f'data: {value:.4f}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


async def getKeyboardInput(my_drone):
    global roll, pitch, throttle, yaw, gimbal_pitch, gimbal_yaw
    while True:
        roll, pitch, throttle, yaw = 0, 0, 0.5, 0
        value = 0.5

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
            throttle = 1
        elif kp.getKey("s"):
            throttle = 0
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

        # Gimbal controls
        if kp.getKey("j"):
            gimbal_pitch = max(GIMBAL_PITCH_MIN, gimbal_pitch - GIMBAL_STEP)
            set_gimbal('/gimbal/cmd_pitch', gimbal_pitch)
        elif kp.getKey("k"):
            gimbal_pitch = min(GIMBAL_PITCH_MAX, gimbal_pitch + GIMBAL_STEP)
            set_gimbal('/gimbal/cmd_pitch', gimbal_pitch)
        if kp.getKey("n"):
            gimbal_yaw = max(GIMBAL_YAW_MIN, gimbal_yaw - GIMBAL_STEP)
            set_gimbal('/gimbal/cmd_yaw', gimbal_yaw)
        elif kp.getKey("m"):
            gimbal_yaw = min(GIMBAL_YAW_MAX, gimbal_yaw + GIMBAL_STEP)
            set_gimbal('/gimbal/cmd_yaw', gimbal_yaw)

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
    await drone.connect(system_address="udp://:14540")
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
    log("-- Ready! Press 'r' to arm the drone.")
    asyncio.ensure_future(manual_control_drone(drone))


async def run():
    global roll, pitch, throttle, yaw
    """Main function to connect to the drone and input manual controls"""
    await asyncio.gather(run_drone())


if __name__ == "__main__":
    asyncio.ensure_future(run())
    asyncio.get_event_loop().run_forever()
