#!/usr/bin/env python3
"""Gimbal car-tracking pipeline for PX4 drone simulation.

Subscribes to /camera (ROS 2 Image), runs YOLOv8 to detect cars, and
controls the gimbal (pitch + yaw) to keep the tracked car centered in
the camera frame.

Architecture:
  - Gimbal commands go through PX4 via VehicleCommand (MAV_CMD 1000):
      tracker → /fmu/in/vehicle_command → PX4 gimbal manager
        → pitch stabilization (IMU) → GZGimbal → Gazebo joints
  - PX4 stabilizes pitch in earth frame (PITCH_LOCK): when the drone
    tilts, the gimbal compensates to maintain horizon-relative pitch.
  - Yaw follows vehicle heading (no YAW_LOCK): yaw=0 means forward.
  - Feedback comes from PX4 via /fmu/out/gimbal_device_attitude_status.
  - WAITING state triggers when drone reaches the viewing position
    (position-based, not time-based).

State machine:
  WAITING -> SEARCHING -> ACQUIRING -> TRACKING -> LOST -> SEARCHING

Writes a per-frame CSV log to /tmp/gimbal_tracker.csv for diagnostics.
"""

import csv
import math
import os
import time
from enum import Enum, auto

import cv2
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import (GimbalDeviceAttitudeStatus, VehicleCommand,
                          VehicleLocalPosition)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

# ── Gimbal limits (from SDF joint definitions) ────────────────────
GIMBAL_PITCH_MIN = -2.356   # -135 deg (cgo3_camera_joint lower limit)
GIMBAL_PITCH_MAX = 0.785    # +45 deg (cgo3_camera_joint upper limit)
GIMBAL_YAW_MIN = -3.14
GIMBAL_YAW_MAX = 3.14

# ── Search pose (default yaw, tilted down) ────────────────────────
SEARCH_PITCH = -0.9599      # -55 deg (default look-down angle)
SEARCH_YAW = 0.0            # no yaw rotation

# ── Camera parameters ────────────────────────────────────────────
HFOV = 1.047                # horizontal FOV in rad (60 deg), from model SDF

# ── P controller gain ─────────────────────────────────────────────
Kp = 0.5                    # proportional gain (on angular error)
MAX_STEP_RAD = 0.05         # max gimbal change per frame (~3 deg)
DEADZONE_PX = 15            # pixel deadzone to prevent jitter

# ── Detection settings ───────────────────────────────────────────
YOLO_MODEL = os.path.join(os.path.dirname(__file__), 'assets', 'drone_car_yolov11n.pt')
TARGET_CLASS = 0             # custom model: class 0 = car (single-class)
CONF_THRESHOLD = 0.6

# ── Viewing position (from auto_takeoff.py, NED frame) ──────────
VIEW_POS_NORTH = -49.4       # meters from home
VIEW_POS_EAST = 184.7
VIEW_POS_DOWN = -15.6        # negative = up
VIEW_POS_TOLERANCE = 5.0     # meters
VIEW_POS_DWELL = 10.0        # seconds drone must stay within tolerance

# ── Tracking state parameters ────────────────────────────────────
HOLD_TIME = 3.0              # seconds to hold position after losing track
ACQUIRING_THRESHOLD = 30     # pixels from center to consider "acquired"
ACQUIRING_FRAMES = 5         # consecutive centered frames to confirm lock
MAX_TARGET_JUMP_PX = 200     # reject detections jumping too far from last

# ── Log file and frame saving ────────────────────────────────────
LOG_PATH = '/tmp/gimbal_tracker.csv'
SAVE_FRAMES = True
FRAMES_DIR = '/tmp/gimbal_frames'
SAVE_FRAME_START = 2700
SAVE_FRAME_END = 3400


class State(Enum):
    WAITING = auto()
    SEARCHING = auto()
    ACQUIRING = auto()
    TRACKING = auto()
    LOST = auto()


# ── Quaternion helper (for feedback decoding) ────────────────────

def quaternion_to_pitch_yaw(q):
    """Extract pitch and yaw from [w, x, y, z] quaternion.

    Returns (pitch_rad, yaw_rad).
    """
    w, x, y, z = q
    # Pitch (rotation about Y axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw (rotation about Z axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return pitch, yaw


# ── QoS profile for PX4 topics ──────────────────────────────────
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class GimbalTracker(Node):
    def __init__(self):
        super().__init__('gimbal_tracker')

        # ── ROS 2 subscriptions ─────────────────────────────────
        self.create_subscription(Image, 'camera', self.on_image, 1)
        self.br = CvBridge()

        # Gimbal attitude feedback from PX4
        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            '/fmu/out/gimbal_device_attitude_status',
            self._on_gimbal_attitude, PX4_QOS)

        # ── ROS 2 publisher: gimbal commands via VehicleCommand ──
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Drone position (for WAITING → SEARCHING trigger)
        # Note: PX4 publishes _v1 versioned topic
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._on_local_position, PX4_QOS)
        self.drone_pos = None  # (north, east, down) in NED
        self.at_position_since = None  # timestamp when drone first reached position

        # YOLOv8
        self.model = YOLO(YOLO_MODEL)
        self.model.to('cuda' if __import__('torch').cuda.is_available() else 'cpu')

        # Actual gimbal angles (updated from PX4 attitude feedback)
        self.actual_yaw = 0.0
        self.actual_pitch = SEARCH_PITCH
        self.joint_state_received = False

        # Commanded gimbal angles
        self.cmd_yaw = SEARCH_YAW
        self.cmd_pitch = SEARCH_PITCH

        # Tracking state
        self.state = State.WAITING
        self.last_target_center = None
        self.last_detection_time = 0.0
        self.acquiring_count = 0
        self.frame_num = 0
        self.last_configure_time = 0.0  # for periodic CONFIGURE re-send
        self.gimbal_control_claimed = False

        # Display window
        cv2.namedWindow('Gimbal Tracker', cv2.WINDOW_NORMAL)

        # Frame saving
        if SAVE_FRAMES:
            os.makedirs(FRAMES_DIR, exist_ok=True)
            self.get_logger().info(
                f'Saving frames {SAVE_FRAME_START}-{SAVE_FRAME_END} '
                f'to {FRAMES_DIR}/')

        # CSV log
        self.log_file = open(LOG_PATH, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            'frame', 'time', 'state',
            'detected', 'det_cx', 'det_cy', 'det_w', 'det_h', 'det_conf',
            'img_w', 'img_h',
            'err_px_x', 'err_px_y',
            'err_ang_x', 'err_ang_y',
            'target_yaw', 'target_pitch',
            'actual_yaw', 'actual_pitch',
            'actual_yaw_deg', 'actual_pitch_deg',
        ])
        self.log_file.flush()

        self.get_logger().info(
            f'Gimbal tracker initialized (PX4 VehicleCommand) — '
            f'WAITING for drone at N:{VIEW_POS_NORTH} E:{VIEW_POS_EAST}, '
            f'logging to {LOG_PATH}')

    # ── Gimbal control authority ────────────────────────────────

    def _claim_gimbal_control(self):
        """Send MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE to claim primary control.

        PX4's gimbal manager denies all PITCHYAW commands until a controller
        has been configured. This sets our sysid/compid as primary controller.
        """
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 1001   # MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        msg.param1 = 1.0    # primary control sysid (match source_system)
        msg.param2 = 1.0    # primary control compid (match source_component)
        msg.param3 = -1.0   # secondary control sysid (-1 = leave unchanged)
        msg.param4 = -1.0   # secondary control compid (-1 = leave unchanged)
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0    # gimbal device ID (0 = primary)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)
        self.get_logger().info('Sent GIMBAL_MANAGER_CONFIGURE — claiming control')

    # ── Gimbal command (through PX4 VehicleCommand) ────────────

    def _send_gimbal(self, pitch_rad, yaw_rad):
        """Send gimbal command through PX4 via MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW.

        Uses PITCH_LOCK (earth-frame pitch stabilization).
        Yaw is vehicle-relative (yaw=0 = forward).
        """
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = 1000   # MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        msg.param1 = math.degrees(pitch_rad)  # pitch angle (deg)
        msg.param2 = math.degrees(yaw_rad)    # yaw angle (deg)
        msg.param3 = float('nan')             # pitch rate (NaN = position mode)
        msg.param4 = float('nan')             # yaw rate (NaN = position mode)
        msg.param5 = 8.0                      # PITCH_LOCK only (earth-frame pitch)
        msg.param6 = 0.0
        msg.param7 = 0.0                      # gimbal device ID (0 = primary)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    # ── Gimbal feedback from PX4 ───────────────────────────────

    def _on_gimbal_attitude(self, msg):
        """Callback: actual gimbal orientation from PX4."""
        pitch, yaw = quaternion_to_pitch_yaw(msg.q)
        self.actual_pitch = pitch
        self.actual_yaw = yaw
        self.joint_state_received = True

    # ── Drone position feedback ──────────────────────────────

    def _on_local_position(self, msg):
        """Callback: drone NED position from PX4."""
        self.drone_pos = (msg.x, msg.y, msg.z)

    # ── Camera frame processing ────────────────────────────────

    def on_image(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        now = time.time()
        self.frame_num += 1

        # Periodically re-send CONFIGURE to maintain gimbal control
        if (self.gimbal_control_claimed and
                now - self.last_configure_time > 5.0):
            self._claim_gimbal_control()
            self.last_configure_time = now

        # ── Claim gimbal control as soon as PX4 feedback arrives ─
        if not self.gimbal_control_claimed and self.joint_state_received:
            self._claim_gimbal_control()
            self.gimbal_control_claimed = True
            self.last_configure_time = now
            # Send forward-looking stabilized pose during transit
            self._send_gimbal(SEARCH_PITCH, SEARCH_YAW)
            self.get_logger().info(
                'Gimbal control claimed — stabilizing during transit')

        # ── WAITING state: wait for drone to reach viewing position ─
        if self.state == State.WAITING:
            # Keep sending stabilized pose during transit
            if self.gimbal_control_claimed:
                self._send_gimbal(SEARCH_PITCH, SEARCH_YAW)

            dist = None
            in_range = False
            if self.drone_pos is not None:
                n, e, d = self.drone_pos
                dist = math.sqrt(
                    (n - VIEW_POS_NORTH) ** 2 +
                    (e - VIEW_POS_EAST) ** 2 +
                    (d - VIEW_POS_DOWN) ** 2)
                in_range = dist < VIEW_POS_TOLERANCE

            # Track dwell time — drone must stay in range for VIEW_POS_DWELL
            if in_range:
                if self.at_position_since is None:
                    self.at_position_since = now
                    self.get_logger().info(
                        f'Drone in range (dist={dist:.1f}m), '
                        f'dwelling {VIEW_POS_DWELL:.0f}s...')
                dwell = now - self.at_position_since
            else:
                if self.at_position_since is not None:
                    self.get_logger().info('Drone left range, resetting dwell.')
                self.at_position_since = None
                dwell = 0.0

            if in_range and dwell >= VIEW_POS_DWELL:
                self._send_gimbal(SEARCH_PITCH, SEARCH_YAW)
                self.cmd_pitch = SEARCH_PITCH
                self.cmd_yaw = SEARCH_YAW
                self.state = State.SEARCHING
                self.get_logger().info(
                    f'Drone stable at position for {dwell:.0f}s — '
                    f'gimbal set to P:{math.degrees(SEARCH_PITCH):.0f} '
                    f'Y:0 — SEARCHING')
            else:
                if (SAVE_FRAMES and
                        SAVE_FRAME_START <= self.frame_num <= SAVE_FRAME_END):
                    cv2.imwrite(
                        f'{FRAMES_DIR}/frame_{self.frame_num:06d}.jpg', frame)
                if dist is not None:
                    if in_range:
                        remaining = VIEW_POS_DWELL - dwell
                        status = f'holding {remaining:.0f}s'
                    else:
                        status = f'dist:{dist:.0f}m'
                else:
                    status = 'no position'
                cv2.putText(frame, f'[WAITING] {status}',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (128, 128, 128), 2)
                cv2.putText(frame, f'F:{self.frame_num}', (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.imshow('Gimbal Tracker', frame)
                cv2.waitKey(1)
                return

        # Run YOLOv8 detection
        results = self.model.predict(
            frame, classes=[TARGET_CLASS], conf=CONF_THRESHOLD, verbose=False)
        target = self._select_target(results, w, h)

        # Per-frame log defaults
        log_detected = 0
        log_det_cx = log_det_cy = 0.0
        log_det_w = log_det_h = 0.0
        log_det_conf = 0.0
        log_err_px_x = log_err_px_y = 0.0
        log_err_ang_x = log_err_ang_y = 0.0
        log_target_yaw = self.cmd_yaw
        log_target_pitch = self.cmd_pitch

        # ── State machine ──────────────────────────────────────
        if target is not None:
            tcx, tcy, x1, y1, x2, y2, conf = target
            self.last_detection_time = now
            self.last_target_center = (tcx, tcy)

            log_detected = 1
            log_det_cx, log_det_cy = tcx, tcy
            log_det_w, log_det_h = x2 - x1, y2 - y1
            log_det_conf = conf

            if self.state == State.SEARCHING:
                self.state = State.ACQUIRING
                self.acquiring_count = 0
                self.get_logger().info(
                    f'Car detected at ({tcx:.0f}, {tcy:.0f}) '
                    f'conf={conf:.2f} — acquiring...')
            elif self.state == State.LOST:
                self.state = State.TRACKING
                self.get_logger().info('Car re-acquired — tracking.')

            log_err_px_x, log_err_px_y, log_err_ang_x, log_err_ang_y, \
                log_target_yaw, log_target_pitch = self._track_target(
                    tcx, tcy, w, h)

            if self.state == State.ACQUIRING:
                err_px = math.sqrt(
                    (tcx - w / 2) ** 2 + (tcy - h / 2) ** 2)
                if err_px < ACQUIRING_THRESHOLD:
                    self.acquiring_count += 1
                    if self.acquiring_count >= ACQUIRING_FRAMES:
                        self.state = State.TRACKING
                        self.get_logger().info('Target acquired — tracking.')
                else:
                    self.acquiring_count = 0
        else:
            # No target — keep sending search pose during SEARCHING
            if self.state == State.SEARCHING:
                self._send_gimbal(SEARCH_PITCH, SEARCH_YAW)
            elif self.state in (State.TRACKING, State.ACQUIRING):
                self.state = State.LOST
                self.get_logger().info(
                    f'Target lost at frame {self.frame_num} — '
                    f'actual P:{math.degrees(self.actual_pitch):+.1f} '
                    f'Y:{math.degrees(self.actual_yaw):+.1f}')
            elif self.state == State.LOST:
                elapsed = now - self.last_detection_time
                if elapsed >= HOLD_TIME:
                    self._return_to_search()
                    if (abs(self.actual_pitch - SEARCH_PITCH) < 0.05 and
                            abs(self.actual_yaw - SEARCH_YAW) < 0.05):
                        self.state = State.SEARCHING
                        self.get_logger().info('Returned to search pose.')

        # ── CSV log ────────────────────────────────────────────
        self.log_writer.writerow([
            self.frame_num, f'{now:.3f}', self.state.name,
            log_detected,
            f'{log_det_cx:.1f}', f'{log_det_cy:.1f}',
            f'{log_det_w:.1f}', f'{log_det_h:.1f}',
            f'{log_det_conf:.3f}',
            w, h,
            f'{log_err_px_x:.1f}', f'{log_err_px_y:.1f}',
            f'{log_err_ang_x:.4f}', f'{log_err_ang_y:.4f}',
            f'{log_target_yaw:.4f}', f'{log_target_pitch:.4f}',
            f'{self.actual_yaw:.4f}', f'{self.actual_pitch:.4f}',
            f'{math.degrees(self.actual_yaw):.1f}',
            f'{math.degrees(self.actual_pitch):.1f}',
        ])
        if self.frame_num % 50 == 0:
            self.log_file.flush()

        # ── Save frames ───────────────────────────────────────
        if (SAVE_FRAMES and
                SAVE_FRAME_START <= self.frame_num <= SAVE_FRAME_END):
            cv2.imwrite(
                f'{FRAMES_DIR}/frame_{self.frame_num:06d}.jpg', frame)

        # ── Overlay + display ──────────────────────────────────
        self._draw_overlay(frame, target, w, h)
        cv2.imshow('Gimbal Tracker', frame)
        cv2.waitKey(1)

    # ── Target selection ───────────────────────────────────────

    def _select_target(self, results, img_w, img_h):
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None
        xyxy = boxes.xyxy
        if len(xyxy) == 0:
            return None

        if (self.state in (State.TRACKING, State.LOST)
                and self.last_target_center is not None):
            centers_x = (xyxy[:, 0] + xyxy[:, 2]) / 2
            centers_y = (xyxy[:, 1] + xyxy[:, 3]) / 2
            lx, ly = self.last_target_center
            distances = ((centers_x - lx) ** 2 + (centers_y - ly) ** 2).sqrt()
            valid_mask = distances <= MAX_TARGET_JUMP_PX
            if not valid_mask.any():
                return None
            distances[~valid_mask] = float('inf')
            best_idx = distances.argmin().item()
        else:
            areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
            best_idx = areas.argmax().item()

        box = xyxy[best_idx]
        x1, y1, x2, y2 = box.tolist()
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        conf = float(boxes.conf[best_idx])
        return (cx, cy, x1, y1, x2, y2, conf)

    # ── Closed-loop P gimbal control ──────────────────────────

    def _track_target(self, target_cx, target_cy, img_w, img_h):
        """Compute target angles and send through PX4."""
        err_px_x = target_cx - img_w / 2.0
        err_px_y = target_cy - img_h / 2.0
        raw_err_px_x, raw_err_px_y = err_px_x, err_px_y

        if abs(err_px_x) < DEADZONE_PX:
            err_px_x = 0.0
        if abs(err_px_y) < DEADZONE_PX:
            err_px_y = 0.0

        vfov = HFOV * (img_h / img_w)
        err_ang_x = err_px_x * (HFOV / img_w)
        err_ang_y = err_px_y * (vfov / img_h)

        # YAW: use cmd_yaw (vehicle-relative) — NOT actual_yaw (world-frame)
        # because with flags=8 (PITCH_LOCK only), yaw commands are
        # vehicle-relative but GimbalDeviceAttitudeStatus reports world-frame.
        # PITCH: use actual_pitch — both command and feedback are earth-frame
        # with PITCH_LOCK, so closed-loop feedback is correct.
        desired_yaw = self.cmd_yaw + Kp * err_ang_x
        desired_pitch = self.actual_pitch - Kp * err_ang_y

        # Limit step size per frame to prevent overshoot / oscillation
        d_yaw = max(-MAX_STEP_RAD, min(MAX_STEP_RAD,
                                       desired_yaw - self.cmd_yaw))
        d_pitch = max(-MAX_STEP_RAD, min(MAX_STEP_RAD,
                                         desired_pitch - self.cmd_pitch))
        target_yaw = self.cmd_yaw + d_yaw
        target_pitch = self.cmd_pitch + d_pitch

        target_yaw = max(GIMBAL_YAW_MIN, min(GIMBAL_YAW_MAX, target_yaw))
        target_pitch = max(GIMBAL_PITCH_MIN, min(GIMBAL_PITCH_MAX, target_pitch))

        self._send_gimbal(target_pitch, target_yaw)
        self.cmd_yaw = target_yaw
        self.cmd_pitch = target_pitch

        return (raw_err_px_x, raw_err_px_y, err_ang_x, err_ang_y,
                target_yaw, target_pitch)

    # ── Return to search pose ─────────────────────────────────

    def _return_to_search(self):
        self._send_gimbal(SEARCH_PITCH, SEARCH_YAW)
        self.cmd_pitch = SEARCH_PITCH
        self.cmd_yaw = SEARCH_YAW

    # ── Visual overlay ────────────────────────────────────────

    def _draw_overlay(self, frame, target, w, h):
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx - 25, cy), (cx + 25, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 25), (cx, cy + 25), (0, 255, 0), 1)

        state_colors = {
            State.WAITING:   (128, 128, 128),
            State.SEARCHING: (128, 128, 128),
            State.ACQUIRING: (0, 255, 255),
            State.TRACKING:  (0, 255, 0),
            State.LOST:      (0, 0, 255),
        }
        color = state_colors[self.state]

        cv2.putText(frame, f'[{self.state.name}]', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(frame, f'F:{self.frame_num}', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cmd_pitch_deg = math.degrees(self.cmd_pitch)
        cmd_yaw_deg = math.degrees(self.cmd_yaw)
        fb = 'PX4' if self.joint_state_received else 'OL'
        angle_text = f'P:{cmd_pitch_deg:+.1f} Y:{cmd_yaw_deg:+.1f} [{fb}]'
        text_size = cv2.getTextSize(
            angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
        cv2.putText(frame, angle_text, (w - text_size[0] - 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        if target is not None:
            tcx, tcy, x1, y1, x2, y2, conf = target
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          color, 2)
            cv2.putText(frame, f'car {conf:.0%}',
                        (int(x1), int(y1) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.arrowedLine(frame, (cx, cy), (int(tcx), int(tcy)),
                            (0, 165, 255), 2, tipLength=0.15)
            cv2.putText(frame, f'err: ({tcx - cx:+.0f}, {tcy - cy:+.0f})px',
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def destroy_node(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()
            self.get_logger().info(
                f'Log saved: {LOG_PATH} ({self.frame_num} frames)')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
