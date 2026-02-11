# PX4-ROS2-Gazebo-YOLOv8
Aerial Object Detection using a Drone with PX4 Autopilot / ArduPilot and ROS 2. SITL and Gazebo Garden used for Simulation. YOLOv8 used for Object Detection.

## Supported Hardware
- **Pixhawk 6X/6C (H7 series)** — STM32H753 processor @ 480 MHz, running ArduCopter
- Any ArduPilot-compatible flight controller
- PX4-based flight controllers (legacy support via `Dockerfile.px4`)

## Features
- Keyboard-controlled drone flight (WASD + arrow keys) via MAVSDK
- 3-axis gimbal camera control (pitch, yaw, roll) adjustable during flight
- ArduCopter flight mode switching (Loiter, AltHold, Guided, Stabilize, Land)
- YOLOv8 real-time object detection with resizable display window
- Moving car target in the simulation for tracking demonstrations
- All services orchestrated via tmuxinator in a single tiled-pane window
- Docker-based setup with GPU passthrough and X11 forwarding
- Dual Dockerfile support: ArduPilot (default) and PX4

## Demo
https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8/assets/58460889/fab19f49-0be6-43ea-a4e4-8e9bc8d59af9

## Docker

Two Dockerfiles are provided:
- `Dockerfile` — **ArduPilot/ArduCopter** (default, recommended for Pixhawk hardware)
- `Dockerfile.px4` — PX4 Autopilot (legacy)

### Prerequisites
Allow Docker to access the X11 display:
```commandline
xhost +local:docker
```

### Build and Run (ArduPilot)
```commandline
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git
cd PX4-ROS2-Gazebo-YOLOv8
bash run_ardupilot.sh
```

### Build and Run (PX4)
```commandline
bash run_px4.sh
```

See [How_to_run.md](How_to_run.md) for detailed manual Docker commands.

### What Launches in Docker (ArduPilot)
The container starts a single tmux window with 6 tiled panes:

| Pane | Service |
|------|---------|
| 1 | Gazebo Garden simulator |
| 2 | ArduCopter SITL (iris drone) |
| 3 | ROS-Gazebo camera bridge |
| 4 | YOLOv8 detection display |
| 5 | Moving car (hatchback driving in circles) |
| 6 | Keyboard drone controller |

Switch between panes with `Ctrl+b` then arrow keys.

## Keyboard Controls

All keyboard input is handled directly in the terminal (no separate window needed).

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
| `3` | Guided (for autonomous control) |
| `4` | Stabilize (manual) |
| `5` | Land |

Manual control (WASD/arrows) works best in Stabilize or AltHold modes.

### Gimbal Camera Controls
| Key | Action |
|-----|--------|
| `j` / `k` | Gimbal pitch down / up |
| `n` / `m` | Gimbal yaw left / right |

The camera starts at 45 degrees downward. Pitch range: -90 to +30 degrees. Yaw range: -90 to +90 degrees.

## Gimbal Camera System

The drone uses the `iris_with_gimbal` model from ardupilot_gazebo, which includes a 3-axis gimbal (`gimbal_small_3d`) with pitch, yaw, and roll control. During Docker build, `setup_gimbal_ardupilot.py` configures the camera sensor (sets the Gazebo topic to `/drone/camera`, removes unavailable plugins, and increases the frame rate to 30 Hz).

Each joint is controlled by a `JointPositionController` plugin responding to Gazebo transport topics:
- `/gimbal/cmd_pitch` — pitch angle command
- `/gimbal/cmd_yaw` — yaw angle command
- `/gimbal/cmd_roll` — roll angle command

## Moving Car

`move_car.py` drives the `hatchback_blue_1` model in a circle within the simulation using `gz service /world/default/set_pose`. This provides a moving target for the YOLOv8 detection system to track.

## Architecture

### ArduPilot Stack
```
ArduCopter SITL  <-- JSON -->  ardupilot_gazebo plugin  <-->  Gazebo Garden
      |                                                           |
      | MAVLink (UDP 14550)                           gz.msgs.Image
      |                                                           |
   MAVSDK Python                                      ROS-GZ bridge
   pymavlink (UDP 14560)                                    |
      |                                              /camera (ROS 2)
keyboard-mavsdk-test.py                                     |
                                                   uav_camera_det.py (YOLOv8)
```

### PX4 Stack (Legacy)
```
PX4 SITL  <-->  Gazebo Garden
    |               |
Micro XRCE-DDS   Camera sensor
    |               |
px4_msgs (ROS 2)  ROS-GZ bridge
    |               |
MAVSDK Python   /camera (ROS 2)
```

## Manual Installation (ArduPilot)
### Create a virtual environment
```commandline
python -m venv ~/ardupilot-venv
source ~/ardupilot-venv/bin/activate
```
### Clone repository
```commandline
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git
```
### Install ArduPilot
```commandline
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```
### Build ardupilot_gazebo Plugin
```commandline
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```
### Install ROS 2
```commandline
cd ~
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
sudo apt install ros-humble-ros-gzgarden
```
### Install Python Packages
```commandline
pip install mavsdk pymavlink aioconsole opencv-python ultralytics numpy
```
### Additional Configs
- Put below lines in your bashrc:
```commandline
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models:~/ardupilot_gazebo/models:~/ardupilot_gazebo/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build
export PATH=$PATH:~/ardupilot/Tools/autotest
```
- Copy the content of models from main repo to ~/.gz/models
- Set up the gimbal camera by running `setup_gimbal_ardupilot.py` (edit `MODEL_PATH` to match your ardupilot_gazebo installation path)

## Run (ArduPilot)
### Fly using Keyboard
You need several terminals.
```commandline
Terminal #1:
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=~/.gz/models:~/ardupilot_gazebo/models:~/ardupilot_gazebo/worlds
gz sim -r ~/ardupilot_gazebo/worlds/ardupilot_default.sdf

Terminal #2:
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console \
  --add-param-file ~/PX4-ROS2-Gazebo-YOLOv8/ardupilot_sitl.parm \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14560 \
  -l 47.397971,8.546164,0,0

Terminal #3:
ros2 run ros_gz_bridge parameter_bridge /drone/camera@sensor_msgs/msg/Image[gz.msgs.Image --ros-args -r /drone/camera:=/camera

Terminal #4:
source ~/ardupilot-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python uav_camera_det.py

Terminal #5:
source ~/ardupilot-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python move_car.py

Terminal #6:
source ~/ardupilot-venv/bin/activate
cd ~/PX4-ROS2-Gazebo-YOLOv8
python keyboard-mavsdk-test.py
```
Focus on the keyboard controller terminal, then press `r` to arm the drone. Use WASD and arrow keys for flight, `1`-`5` for flight mode switching, `j`/`k`/`n`/`m` for gimbal control, and `l` for landing.

## Project Structure

| File | Description |
|------|-------------|
| `keyboard-mavsdk-test.py` | Keyboard-controlled drone flight, gimbal, and mode switching |
| `KeyPressModule.py` | Raw terminal keyboard input handler (works in Docker/tmux) |
| `uav_camera_det.py` | ROS 2 node for YOLOv8 object detection on drone camera feed |
| `move_car.py` | Drives the hatchback model in circles for tracking demos |
| `setup_gimbal_ardupilot.py` | Configures gimbal camera topic and frame rate for ArduPilot iris model |
| `setup_gimbal.py` | Adds gimbal to PX4 x500_depth model SDF |
| `ardupilot_ros2_gazebo.yml` | Tmuxinator config for ArduPilot Docker (6 tiled panes) |
| `px4_ros2_gazebo.yml` | Tmuxinator config for PX4 Docker |
| `ardupilot_sitl.parm` | ArduCopter SITL parameters (scheduler rate, fence, gyro cal) |
| `Dockerfile` | Docker build with ArduPilot, Gazebo, MAVSDK, YOLOv8 |
| `Dockerfile.px4` | Docker build with PX4 |
| `run_ardupilot.sh` | Build and run ArduPilot Docker |
| `run_px4.sh` | Build and run PX4 Docker |
| `worlds/ardupilot_default.sdf` | Gazebo world with racetrack, vehicles, and iris drone |
| `worlds/default_docker.sdf` | Gazebo world for PX4 |

## Acknowledgement
- https://github.com/ArduPilot/ardupilot
- https://github.com/ArduPilot/ardupilot_gazebo
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ultralytics/ultralytics
- https://www.ros.org/
- https://gazebosim.org/
