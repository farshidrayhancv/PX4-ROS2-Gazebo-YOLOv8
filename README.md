# PX4-ROS2-Gazebo-YOLOv8

<p align="center">
  <img src="assets/ros2_sim.png" alt="Simulation overview — Gazebo 3D view, lane detection camera, and terminal output" width="100%"/>
</p>

Aerial Object Detection using a Drone with PX4 Autopilot or ArduPilot, ROS 2, Gazebo Garden, and YOLOv11. Features autonomous gimbal-based car tracking with drone flight control, thermal imaging, depth sensing, and vision-based lane-keeping on Sonoma Raceway.

Two autopilot stacks are supported — choose based on your hardware:
- **ArduPilot/ArduCopter** — for Pixhawk 6X/6C (STM32H753 @ 480 MHz) and other ArduCopter hardware
- **PX4 Autopilot** — for PX4-based flight controllers (x500_gimbal model with RGB, thermal, and depth cameras)

## Demo

https://github.com/user-attachments/assets/878b75fd-134b-4159-94c9-36afef6882e8

## Features
- **Autonomous gimbal tracking** — drone tracks cars using YOLOv11, controls both gimbal and flight (PX4 offboard mode)
- **Thermal imaging** — 320x240 thermal camera with false-color visualization (INFERNO colormap)
- **Depth sensing** — depth camera for 3D perception
- **Auto takeoff** — autonomous flight to optimal viewing position with altitude and yaw control
- Keyboard-controlled drone flight (WASD + arrow keys) via MAVSDK
- 3-axis gimbal camera control (pitch, yaw, roll) with native gz.transport13 Python bindings
- ArduCopter flight mode switching (Loiter, AltHold, Guided, Stabilize, Land)
- YOLOv11 real-time object detection with custom aerial car detection model
- **Vision-based lane keeping** — hatchback car autonomously drives Sonoma Raceway using camera + OpenCV
- Tmuxinator orchestration — all services in a single tiled-pane window
- Docker with GPU passthrough and X11 forwarding

## Autonomous Gimbal Tracking (PX4)

The **gimbal_tracker.py** node provides fully autonomous car tracking using computer vision and PX4 offboard control. The system controls both the gimbal (for stabilized tracking) and drone flight (to maintain optimal viewing angles).

**Architecture:**
- **Layer 1 (Gimbal)**: VehicleCommand (MAV_CMD 1000) → PX4 gimbal module → earth-frame pitch stabilization
- **Layer 2 (Drone Flight)**: TrajectorySetpoint + OffboardControlMode → PX4 offboard mode → yaw toward target, position tracking

**State Machine:**
```
WAITING → SEARCHING → ACQUIRING → TRACKING → LOST → SEARCHING
```

**How it works:**
1. **WAITING**: Position-based trigger at N:70.5 E:102.9 D:-15.7 (5m tolerance, 10s dwell)
2. **SEARCHING**: Gimbal at -55° pitch, 0° yaw (vehicle-relative), scanning for cars
3. **ACQUIRING**: Car detected, centering gimbal on target
4. **TRACKING**: P controller (Kp=0.5) maintains car in frame center, drone yaw follows gimbal direction
5. **LOST**: Target lost, return to SEARCHING after timeout

**Features:**
- Custom **drone_car_yolov11n.pt** model trained for aerial car detection
- Native gz.transport13 Python bindings (no subprocess overhead)
- Gimbal pitch range: -135° to +45° (full SDF joint limits)
- 15px deadzone, 0.05 rad/frame max step (~3°)
- CSV logging to `/tmp/gimbal_tracker.csv` (frame, state, detections, angles, errors)

**Auto Takeoff:**
The **auto_takeoff.py** script autonomously flies the drone to the viewing position (N:70.5 E:102.9 D:-15.7) using PX4 offboard mode, holds for 30 seconds, then transfers control to the gimbal tracker.

## Thermal & Depth Imaging (PX4)

The **x500_gimbal** model includes co-located RGB, thermal, and depth cameras on the gimbal mount, providing multi-modal sensing for object detection and tracking.

**Thermal Camera (320x240, 10 Hz):**
- 16-bit grayscale format (L16) encoding temperature in Kelvin
- Scene objects tagged with temperatures via `gz-sim-thermal-system`:
  - `hatchback_blue` → 350 K (engine heat)
  - `pickup` → 340 K
  - `casual_female` → 310 K (body temperature)
  - Ambient objects → ~293 K
- **thermal_camera_viewer.py** displays false-color visualization using OpenCV INFERNO colormap
- Bridge topic: `/thermal_camera` (sensor_msgs/Image)

**Depth Camera:**
- Range sensor for 3D perception
- **depth_camera_viewer.py** displays normalized depth values
- Bridge topic: `/depth_camera` (sensor_msgs/Image)

Both cameras stabilize with the gimbal, maintaining the same FOV as the RGB camera for aligned multi-modal data.

## Lane Keeping

The hatchback car on Sonoma Raceway drives itself using a forward-facing camera and OpenCV-based yellow lane detection. No waypoints or pre-mapped paths — pure vision.

**How it works:**
- HSV thresholding detects yellow lane lines from the car's camera (640x480 @ 15Hz, 120° FOV)
- Near-field classification (bottom half of image) prevents lane misdetection in sharp curves
- Adaptive PID controller: gain scales quadratically with lane proximity — gentle on straights, aggressive near walls
- Road curvature feedforward: averages both lane angles to steer into curves even when centered
- Trend steering: slow-moving average remembers the turn direction so the car doesn't straighten out when lanes temporarily vanish
- Merge detection: when both lanes converge (on-ramp merge), the car steers right for 2 seconds then resumes normal lane keeping
- Speed control: 2.5–5.0 m/s, exponential ramp-up on straights, drops during corrections, crawls at 1 m/s with no lanes

**Debug output:**
- OpenCV window with lane overlay, danger scores, steering angle, speed, and frame number
- CSV log at `/tmp/lane_keeping_log.csv` with per-frame telemetry

## Docker

Two Dockerfiles are provided:
- `Dockerfile` — ArduPilot/ArduCopter (default)
- `Dockerfile.px4` — PX4 Autopilot

### ArduPilot (default)
```bash
git clone https://github.com/farshidrayhancv/PX4-ROS2-Gazebo-YOLOv8.git
cd PX4-ROS2-Gazebo-YOLOv8
bash run_ardupilot.sh
```

### PX4
```bash
git clone https://github.com/farshidrayhancv/PX4-ROS2-Gazebo-YOLOv8.git
cd PX4-ROS2-Gazebo-YOLOv8
bash run_px4.sh
```

See [How_to_run.md](How_to_run.md) for the full `docker run` commands if you need to customize flags.

### What Launches in Docker

**ArduPilot** (`Dockerfile` / `run_ardupilot.sh`):

| Pane | Service |
|------|---------|
| 1 | Gazebo Garden simulator |
| 2 | ArduCopter SITL (iris drone with gimbal) |
| 3 | ROS-Gazebo camera bridge |
| 4 | YOLOv8 detection display |
| 5 | Moving car (hatchback driving in circles) |
| 6 | Keyboard drone controller |

**PX4** (`Dockerfile.px4` / `run_px4.sh`):

| Pane | Service |
|------|---------|
| 1 | Micro XRCE-DDS Agent |
| 2 | PX4 SITL (x500_gimbal drone with RGB, thermal, depth cameras) |
| 3 | ROS-Gazebo RGB camera bridge |
| 4 | Autonomous gimbal tracker (YOLOv11 + offboard control) |
| 5 | ROS-Gazebo car camera + cmd_vel bridges |
| 6 | Lane keeping (autonomous car) |
| 7 | Keyboard drone controller |
| 8 | Auto takeoff to viewing position |
| 9 | ROS-Gazebo thermal camera bridge |
| 10 | Thermal camera viewer (false-color display) |
| 11 | ROS-Gazebo depth camera bridge |
| 12 | Depth camera viewer |

Switch between panes with `Ctrl+b` then arrow keys.

## Keyboard Controls

### Flight Controls
| Key | Action |
|-----|--------|
| `r` | Arm the drone |
| `l` | Land |
| `w` / `s` | Throttle up / down |
| `a` / `d` | Yaw left / right |
| Arrow keys | Roll / Pitch |
| `i` | Print flight mode |
| `Ctrl+C` | Quit |

### Flight Mode Switching (ArduCopter)
| Key | Mode |
|-----|------|
| `1` | Loiter (position hold) |
| `2` | AltHold (altitude hold) |
| `3` | Guided (autonomous control) |
| `4` | Stabilize (manual) |
| `5` | Land |

### Gimbal Camera Controls
| Key | Action |
|-----|--------|
| `j` / `k` | Gimbal pitch down / up |
| `n` / `m` | Gimbal yaw left / right |

## Architecture

### ArduPilot Stack
```
ArduCopter SITL  <-- JSON/UDP -->  ardupilot_gazebo plugin  <-->  Gazebo Garden
      |                                                              |
      | MAVLink (UDP 14550)                                gz.msgs.Image
      |                                                              |
   MAVSDK Python                                          ROS-GZ bridge
   pymavlink (UDP 14560)                                        |
      |                                                  /camera (ROS 2)
keyboard-mavsdk-test.py                                         |
                                                      uav_camera_det.py (YOLOv8)
```

### PX4 Stack
```
PX4 SITL (x500_gimbal)  <-->  Gazebo Garden (in-process plugin)
    |                              |
    |                         RGB Camera (1280x720)
    |                         Thermal Camera (320x240, L16)
    |                         Depth Camera
    |                              |
Micro XRCE-DDS              ROS-GZ bridge
    |                              |
px4_msgs (ROS 2)            /camera, /thermal_camera, /depth_camera
    |                              |
    |                         gimbal_tracker.py
    |                         (YOLOv11 + offboard control)
    |                              |
VehicleCommand          thermal_camera_viewer.py
TrajectorySetpoint      depth_camera_viewer.py
OffboardControlMode
    |
auto_takeoff.py
keyboard-mavsdk-test.py
```

## Manual Installation

### ArduPilot

```bash
# Virtual environment
python -m venv ~/ardupilot-venv
source ~/ardupilot-venv/bin/activate

# Clone this repo
git clone https://github.com/farshidrayhancv/PX4-ROS2-Gazebo-YOLOv8.git

# ArduPilot
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter

# ardupilot_gazebo plugin
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)

# ROS 2 Humble + Gazebo Garden bridge
sudo apt install ros-humble-desktop ros-dev-tools ros-humble-ros-gzgarden

# Python packages
pip install mavsdk pymavlink aioconsole opencv-python ultralytics numpy

# Bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=~/.gz/models:~/ardupilot_gazebo/models:~/ardupilot_gazebo/worlds' >> ~/.bashrc
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build' >> ~/.bashrc
echo 'export PATH=$PATH:~/ardupilot/Tools/autotest' >> ~/.bashrc

# Copy models
cp -r ~/PX4-ROS2-Gazebo-YOLOv8/models/* ~/.gz/models/

# Configure gimbal camera
python ~/PX4-ROS2-Gazebo-YOLOv8/setup_gimbal_ardupilot.py
```

#### Run (ArduPilot)
```bash
# Terminal 1: Gazebo
gz sim -r ~/ardupilot_gazebo/worlds/ardupilot_default.sdf

# Terminal 2: ArduCopter SITL
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --add-param-file ~/PX4-ROS2-Gazebo-YOLOv8/ardupilot_sitl.parm \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560 \
  -l 47.397971,8.546164,0,0

# Terminal 3: Camera bridge
ros2 run ros_gz_bridge parameter_bridge \
  /drone/camera@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /drone/camera:=/camera

# Terminal 4: YOLOv8 detection
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python uav_camera_det.py

# Terminal 5: Moving car
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python move_car.py

# Terminal 6: Keyboard controller
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python keyboard-mavsdk-test.py
```

### PX4

```bash
# Virtual environment
python -m venv ~/px4-venv
source ~/px4-venv/bin/activate

# Clone this repo
git clone https://github.com/farshidrayhancv/PX4-ROS2-Gazebo-YOLOv8.git

# PX4 Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot && make px4_sitl

# Micro XRCE-DDS Agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/

# ROS 2 Humble + Gazebo Garden bridge
sudo apt install ros-humble-desktop ros-dev-tools ros-humble-ros-gzgarden

# ROS 2 workspaces
mkdir -p ~/ws_sensor_combined/src && cd ~/ws_sensor_combined/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd .. && source /opt/ros/humble/setup.bash && colcon build

# Python packages
pip install mavsdk aioconsole opencv-python ultralytics numpy

# Bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=~/.gz/models' >> ~/.bashrc

# Copy models and world
cp -r ~/PX4-ROS2-Gazebo-YOLOv8/models/* ~/.gz/models/
cp ~/PX4-ROS2-Gazebo-YOLOv8/worlds/default_docker.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

# Configure gimbal camera
python ~/PX4-ROS2-Gazebo-YOLOv8/setup_gimbal.py
```

#### Run (PX4)
```bash
# Terminal 1: DDS bridge
cd ~/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888

# Terminal 2: PX4 SITL (x500_gimbal with RGB, thermal, depth cameras)
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4019 PX4_GZ_MODEL_POSE="268.08,-128.22,3.86,0.00,0,-0.7" \
  PX4_GZ_MODEL=x500_gimbal ./build/px4_sitl_default/bin/px4

# Terminal 3: RGB camera bridge
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image:=/camera

# Terminal 4: Autonomous gimbal tracker (YOLOv11)
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python gimbal_tracker.py

# Terminal 5: Car camera + cmd_vel bridges
ros2 run ros_gz_bridge parameter_bridge \
  /car/camera@sensor_msgs/msg/Image[gz.msgs.Image \
  '/model/hatchback_blue_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  --ros-args -r /car/camera:=/car_camera

# Terminal 6: Lane keeping (autonomous car)
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python lane_keeping.py

# Terminal 7: Auto takeoff to viewing position
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python auto_takeoff.py

# Terminal 8: Thermal camera bridge
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/thermal_camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /world/default/model/x500_gimbal_0/link/camera_link/sensor/thermal_camera/image:=/thermal_camera

# Terminal 9: Thermal camera viewer
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python thermal_camera_viewer.py

# Terminal 10: Depth camera bridge
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_gimbal_0/link/camera_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image \
  --ros-args -r /world/default/model/x500_gimbal_0/link/camera_link/sensor/depth_camera/depth_image:=/depth_camera

# Terminal 11: Depth camera viewer
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python depth_camera_viewer.py

# Terminal 12 (optional): Keyboard controller (for manual override)
cd ~/PX4-ROS2-Gazebo-YOLOv8 && python keyboard-mavsdk-test.py
```

## Project Structure

| File | Description |
|------|-------------|
| `Dockerfile` | ArduPilot Docker build |
| `Dockerfile.px4` | PX4 Docker build |
| `run_ardupilot.sh` | Build and run ArduPilot Docker |
| `run_px4.sh` | Build and run PX4 Docker |
| `keyboard-mavsdk-test.py` | Keyboard flight control, gimbal, and mode switching |
| `KeyPressModule.py` | Terminal keyboard input handler |
| `gimbal_tracker.py` | **NEW**: Autonomous car tracking with gimbal + drone offboard control (PX4) |
| `auto_takeoff.py` | **NEW**: Autonomous flight to viewing position (PX4 offboard mode) |
| `thermal_camera_viewer.py` | **NEW**: Thermal camera viewer with INFERNO false-color mapping |
| `depth_camera_viewer.py` | **NEW**: Depth camera viewer |
| `gimbal_diag.py` | **NEW**: Gimbal diagnostic tool for axis direction verification |
| `uav_camera_det.py` | ROS 2 YOLOv11 detection node |
| `lane_keeping.py` | Vision-based autonomous lane keeping for PX4 |
| `move_car.py` | Drives hatchback in circles for drone tracking (ArduPilot only) |
| `setup_gimbal_ardupilot.py` | Configures gimbal camera for ArduPilot iris model |
| `setup_gimbal.py` | Configures gimbal + thermal + depth cameras for PX4 x500_gimbal model |
| `ardupilot_ros2_gazebo.yml` | Tmuxinator config (ArduPilot) |
| `px4_ros2_gazebo.yml` | Tmuxinator config (PX4) — launches all 12 services |
| `ardupilot_sitl.parm` | ArduCopter SITL parameters |
| `worlds/ardupilot_default.sdf` | Gazebo world (ArduPilot) |
| `worlds/default_docker.sdf` | Gazebo world (PX4) with thermal system plugin |
| `drone_car_yolov11n.pt` | Custom YOLOv11n model trained for aerial car detection |

## Acknowledgement
- https://github.com/ArduPilot/ardupilot
- https://github.com/ArduPilot/ardupilot_gazebo
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/
