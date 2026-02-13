#!/usr/bin/env python3
"""Gimbal car-tracking pipeline for PX4 drone simulation.

Subscribes to /camera (ROS 2 Image), runs YOLOv8 to detect cars, and
controls the gimbal (pitch + yaw) to keep the tracked car centered in
the camera frame.

Architecture (matches industry-standard inner-outer loop):
  - Inner loop: Gazebo JointPositionController (PID, closed-loop on
    joint angle, ~250 Hz physics rate, cmd_max=1.0 rad/s)
  - Outer loop: This node — vision-based P controller that reads
    ACTUAL gimbal angles from joint state, computes absolute target
    angles, and publishes position commands.

Key difference from naive approach: we read actual gimbal joint angles
each frame instead of accumulating incremental commands blindly.
This prevents software/physical drift that causes "misfiring."

State machine:
  SEARCHING -> ACQUIRING -> TRACKING -> LOST -> SEARCHING

Writes a per-frame CSV log to /tmp/gimbal_tracker.csv for diagnostics.
"""

import csv
import math
import os
import threading
import time
from enum import Enum, auto

# gz.msgs protobuf needs this workaround for protobuf version mismatch
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gz.msgs10.double_pb2 import Double as GzDouble
from gz.msgs10.model_pb2 import Model as GzModel
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

# ── Gimbal topics (must match setup_gimbal.py 'manual/*' naming) ──
GIMBAL_TOPIC_PITCH = '/model/x500_gimbal_0/manual/gimbal_pitch'
GIMBAL_TOPIC_YAW = '/model/x500_gimbal_0/manual/gimbal_yaw'

# ── Gazebo joint state topic (published by JointStatePublisher plugin) ──
GZ_JOINT_STATE_TOPIC = '/world/default/model/x500_gimbal_0/joint_state'

# ── Gimbal joint names (from x500_gimbal model SDF) ────────────────
# Verified against JointPositionController sub_topic mapping:
#   manual/gimbal_yaw   → cgo3_vertical_arm_joint
#   manual/gimbal_pitch → cgo3_camera_joint
#   manual/gimbal_roll  → cgo3_horizontal_arm_joint
JOINT_YAW = 'cgo3_vertical_arm_joint'
JOINT_PITCH = 'cgo3_camera_joint'
JOINT_ROLL = 'cgo3_horizontal_arm_joint'

# ── Gimbal limits (from SDF joint definitions) ────────────────────
GIMBAL_PITCH_MIN = -2.356   # -135 deg (cgo3_camera_joint lower limit)
GIMBAL_PITCH_MAX = 0.785    # +45 deg (cgo3_camera_joint upper limit)
GIMBAL_YAW_MIN = -3.14
GIMBAL_YAW_MAX = 3.14

# ── Search pose (default yaw, tilted down) ────────────────────────
SEARCH_PITCH = -0.7854      # -45 deg (default look-down angle)
SEARCH_YAW = 0.0            # no yaw rotation

# ── Camera parameters ────────────────────────────────────────────
HFOV = 1.047                # horizontal FOV in rad (60 deg), from model SDF

# ── P controller gain ─────────────────────────────────────────────
# With closed-loop feedback (actual angle read each frame), a simple
# P controller is stable and converges.  Kp=1.0 means "point directly
# at the car."  Kp<1.0 dampens and converges over multiple frames.
# No rate limiter needed — Gazebo's JointPositionController has
# cmd_max=1.0 rad/s which physically limits joint velocity.
Kp = 0.8                    # proportional gain (on angular error)
DEADZONE_PX = 15            # pixel deadzone to prevent jitter

# ── Detection settings ───────────────────────────────────────────
YOLO_MODEL = os.path.join(os.path.dirname(__file__), 'assets', 'drone_car_yolov11n.pt')
TARGET_CLASS = 0             # custom model: class 0 = car (single-class)
CONF_THRESHOLD = 0.4

# ── Startup: wait for drone to reach viewing position ───────────
# The tracker starts at sleep 30, auto_takeoff at sleep 25.
# auto_takeoff needs ~30s to reach the waypoint (arm + takeoff + fly).
# So we wait ~30s after tracker start before enabling search.
STARTUP_DELAY = 30.0         # seconds to wait before enabling search

# ── Tracking state parameters ────────────────────────────────────
HOLD_TIME = 3.0              # seconds to hold position after losing track
SEARCH_RETURN_SPEED = 0.3    # rad/sec for smooth return to search pose
ACQUIRING_THRESHOLD = 30     # pixels from center to consider "acquired"
ACQUIRING_FRAMES = 5         # consecutive centered frames to confirm lock
MAX_TARGET_JUMP_PX = 200     # reject detections jumping too far from last

# ── Log file and frame saving ────────────────────────────────────
LOG_PATH = '/tmp/gimbal_tracker.csv'
SAVE_FRAMES = True          # save raw frames for YOLO training
FRAMES_DIR = '/tmp/gimbal_frames'
SAVE_FRAME_START = 2700     # only save frames in this range
SAVE_FRAME_END = 3400       # (max ~700 images)


class State(Enum):
    WAITING = auto()      # waiting for drone to reach viewing position
    SEARCHING = auto()
    ACQUIRING = auto()
    TRACKING = auto()
    LOST = auto()


# ── Gimbal publisher (native gz.transport — no subprocess overhead) ──
_gz_node = GzNode()
_gz_pubs = {}  # topic -> publisher


def set_gimbal(topic, value):
    """Publish a gimbal joint position via native gz.transport.

    Uses a persistent publisher (no subprocess spawning), so every
    command is delivered immediately and reliably.
    """
    pub = _gz_pubs.get(topic)
    if pub is None:
        pub = _gz_node.advertise(topic, GzDouble)
        _gz_pubs[topic] = pub
        time.sleep(0.2)  # brief pause for first advertisement to register
    msg = GzDouble()
    msg.data = value
    return pub.publish(msg)


class GimbalTracker(Node):
    def __init__(self):
        super().__init__('gimbal_tracker')

        # ROS 2 subscription (camera only)
        self.create_subscription(Image, 'camera', self.on_image, 1)
        self.br = CvBridge()

        # YOLOv8 (explicit CUDA to avoid first-frame cold start)
        self.model = YOLO(YOLO_MODEL)
        self.model.to('cuda' if __import__('torch').cuda.is_available() else 'cpu')

        # Actual gimbal angles (updated from Gazebo joint state feedback)
        self.actual_yaw = 0.0
        self.actual_pitch = SEARCH_PITCH
        self.joint_state_received = False
        self._joint_lock = threading.Lock()

        # Subscribe to joint state via native gz.transport (no subprocess)
        if not _gz_node.subscribe(GzModel,
                                   GZ_JOINT_STATE_TOPIC,
                                   self._on_joint_state):
            self.get_logger().warn(
                f'Failed to subscribe to {GZ_JOINT_STATE_TOPIC}')

        # Commanded gimbal angles (what we last sent to Gazebo)
        self.cmd_yaw = SEARCH_YAW
        self.cmd_pitch = SEARCH_PITCH

        # Tracking state — start in WAITING (no gimbal commands during takeoff)
        self.state = State.WAITING
        self.start_time = time.time()
        self.last_target_center = None
        self.last_detection_time = 0.0
        self.acquiring_count = 0
        self.frame_num = 0

        # Display window
        cv2.namedWindow('Gimbal Tracker', cv2.WINDOW_NORMAL)

        # ── Frame saving for YOLO training ───────────────────────
        if SAVE_FRAMES:
            import os
            os.makedirs(FRAMES_DIR, exist_ok=True)
            self.get_logger().info(
                f'Saving frames {SAVE_FRAME_START}-{SAVE_FRAME_END} '
                f'to {FRAMES_DIR}/')

        # ── CSV log file ──────────────────────────────────────────
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

        # Do NOT send gimbal commands yet — wait for drone to reach position
        self.get_logger().info(
            f'Gimbal tracker initialized — WAITING {STARTUP_DELAY:.0f}s '
            f'for drone to reach viewing position, '
            f'logging to {LOG_PATH}')

    def _on_joint_state(self, msg):
        """Callback for native gz.transport joint state subscriber.

        Called on gz.transport's internal thread whenever new joint state
        is published by the JointStatePublisher plugin in Gazebo.
        The msg is a gz.msgs.Model protobuf with repeated Joint entries.
        """
        # Handle raw bytes if gz.transport passes serialized protobuf
        if isinstance(msg, (bytes, memoryview)):
            model = GzModel()
            model.ParseFromString(bytes(msg))
            msg = model

        yaw = pitch = None
        for joint in msg.joint:
            if joint.name == JOINT_YAW:
                yaw = joint.axis1.position
            elif joint.name == JOINT_PITCH:
                pitch = joint.axis1.position

        with self._joint_lock:
            if yaw is not None:
                self.actual_yaw = yaw
            if pitch is not None:
                self.actual_pitch = pitch
            if yaw is not None or pitch is not None:
                self.joint_state_received = True

    def on_image(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        now = time.time()
        self.frame_num += 1

        # ── WAITING state: no detection, no gimbal commands ──────
        if self.state == State.WAITING:
            elapsed = now - self.start_time
            remaining = STARTUP_DELAY - elapsed
            if remaining <= 0:
                # Drone should be at viewing position — reset gimbal
                set_gimbal(GIMBAL_TOPIC_PITCH, SEARCH_PITCH)
                set_gimbal(GIMBAL_TOPIC_YAW, SEARCH_YAW)
                self.cmd_pitch = SEARCH_PITCH
                self.cmd_yaw = SEARCH_YAW
                self.state = State.SEARCHING
                self.get_logger().info(
                    f'Startup delay complete — gimbal reset to '
                    f'P:{math.degrees(SEARCH_PITCH):.0f}° Y:0° — '
                    f'SEARCHING')
            else:
                # Show waiting overlay and save frames, but skip detection
                if (SAVE_FRAMES and
                        SAVE_FRAME_START <= self.frame_num <= SAVE_FRAME_END):
                    cv2.imwrite(
                        f'{FRAMES_DIR}/frame_{self.frame_num:06d}.jpg', frame)
                cv2.putText(frame, f'[WAITING] {remaining:.0f}s',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (128, 128, 128), 2)
                cv2.putText(frame, f'F:{self.frame_num}', (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.imshow('Gimbal Tracker', frame)
                cv2.waitKey(1)
                return

        # Run YOLOv8 detection (cars only)
        results = self.model.predict(
            frame, classes=[TARGET_CLASS], conf=CONF_THRESHOLD, verbose=False)

        # Select best target
        target = self._select_target(results, w, h)

        # Per-frame log data (defaults for no-detection)
        log_detected = 0
        log_det_cx = log_det_cy = 0.0
        log_det_w = log_det_h = 0.0
        log_det_conf = 0.0
        log_err_px_x = log_err_px_y = 0.0
        log_err_ang_x = log_err_ang_y = 0.0
        log_target_yaw = self.cmd_yaw
        log_target_pitch = self.cmd_pitch

        # ── State machine ─────────────────────────────────────────
        if target is not None:
            tcx, tcy, x1, y1, x2, y2, conf = target
            self.last_detection_time = now
            self.last_target_center = (tcx, tcy)

            log_detected = 1
            log_det_cx = tcx
            log_det_cy = tcy
            log_det_w = x2 - x1
            log_det_h = y2 - y1
            log_det_conf = conf

            if self.state == State.SEARCHING:
                self.state = State.ACQUIRING
                self.acquiring_count = 0
                self.get_logger().info(
                    f'Car detected at ({tcx:.0f}, {tcy:.0f}) conf={conf:.2f} — acquiring...')

            elif self.state == State.LOST:
                self.state = State.TRACKING
                self.get_logger().info('Car re-acquired — tracking.')

            # Compute and apply gimbal correction
            log_err_px_x, log_err_px_y, log_err_ang_x, log_err_ang_y, \
                log_target_yaw, log_target_pitch = self._track_target(
                    tcx, tcy, w, h)

            # Check if acquired (centered for N consecutive frames)
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
            # No detection this frame
            if self.state in (State.TRACKING, State.ACQUIRING):
                self.state = State.LOST
                self.get_logger().info(
                    f'Target lost at frame {self.frame_num} — '
                    f'actual P:{math.degrees(self.actual_pitch):+.1f}° '
                    f'Y:{math.degrees(self.actual_yaw):+.1f}°')

            elif self.state == State.LOST:
                elapsed = now - self.last_detection_time
                if elapsed >= HOLD_TIME:
                    self._return_to_search()
                    if (abs(self.actual_pitch - SEARCH_PITCH) < 0.05 and
                            abs(self.actual_yaw - SEARCH_YAW) < 0.05):
                        self.state = State.SEARCHING
                        self.get_logger().info('Returned to search pose.')

        # ── Write CSV log row ─────────────────────────────────────
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

        # ── Save raw frames (before overlay) for YOLO training ────
        if (SAVE_FRAMES and
                SAVE_FRAME_START <= self.frame_num <= SAVE_FRAME_END):
            cv2.imwrite(
                f'{FRAMES_DIR}/frame_{self.frame_num:06d}.jpg', frame)

        # ── Draw overlay and display ──────────────────────────────
        self._draw_overlay(frame, target, w, h)
        cv2.imshow('Gimbal Tracker', frame)
        cv2.waitKey(1)

    # ── Target selection ──────────────────────────────────────────

    def _select_target(self, results, img_w, img_h):
        """Pick the best car detection to track.

        Returns (cx, cy, x1, y1, x2, y2, conf) or None.
        """
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
            dx = centers_x - lx
            dy = centers_y - ly
            distances = (dx ** 2 + dy ** 2).sqrt()

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
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        conf = float(boxes.conf[best_idx])
        return (cx, cy, x1, y1, x2, y2, conf)

    # ── Closed-loop P gimbal control ─────────────────────────────

    def _track_target(self, target_cx, target_cy, img_w, img_h):
        """Compute ABSOLUTE target angles using actual gimbal feedback.

        Instead of accumulating incremental deltas (which drifts), we:
        1. Read actual gimbal angle from joint state feedback
        2. Compute angular error from pixel offset
        3. target = actual + Kp * error  (absolute position command)
        4. Publish target to Gazebo JointPositionController

        This matches the industry-standard approach used by DJI ActiveTrack,
        SimpleBGC, and every servo-based face tracker.

        Returns (err_px_x, err_px_y, err_ang_x, err_ang_y,
                 target_yaw, target_pitch) for logging.
        """
        # Pixel error from image center
        err_px_x = target_cx - img_w / 2.0
        err_px_y = target_cy - img_h / 2.0
        raw_err_px_x = err_px_x
        raw_err_px_y = err_px_y

        # Deadzone — suppress jitter when nearly centered
        if abs(err_px_x) < DEADZONE_PX:
            err_px_x = 0.0
        if abs(err_px_y) < DEADZONE_PX:
            err_px_y = 0.0

        # Convert pixel error to angular error (radians)
        vfov = HFOV * (img_h / img_w)
        err_ang_x = err_px_x * (HFOV / img_w)
        err_ang_y = err_px_y * (vfov / img_h)

        # Absolute target computation:
        #   Sign convention (verified empirically with gimbal_diag.py):
        #   - PITCH: negative = camera looks DOWN, positive = looks UP
        #     Car BELOW center → positive err_ang_y → decrease pitch → look more down
        #   - YAW: negative yaw → FOV pans LEFT, positive yaw → FOV pans RIGHT
        #     Car RIGHT of center → positive err_ang_x → increase yaw → pan right
        target_yaw = self.actual_yaw + Kp * err_ang_x
        target_pitch = self.actual_pitch - Kp * err_ang_y

        # Clamp to gimbal limits
        target_yaw = max(GIMBAL_YAW_MIN, min(GIMBAL_YAW_MAX, target_yaw))
        target_pitch = max(GIMBAL_PITCH_MIN, min(GIMBAL_PITCH_MAX, target_pitch))

        # Publish to Gazebo — the JointPositionController will slew
        # to the target at cmd_max=1.0 rad/s (57°/s)
        set_gimbal(GIMBAL_TOPIC_YAW, target_yaw)
        set_gimbal(GIMBAL_TOPIC_PITCH, target_pitch)
        self.cmd_yaw = target_yaw
        self.cmd_pitch = target_pitch

        return (raw_err_px_x, raw_err_px_y, err_ang_x, err_ang_y,
                target_yaw, target_pitch)

    # ── Return to search pose ─────────────────────────────────────

    def _return_to_search(self):
        """Smoothly slew gimbal back to the search pose."""
        # Just command the search pose directly — the JointPositionController
        # will slew there at cmd_max speed.  No need for manual stepping.
        set_gimbal(GIMBAL_TOPIC_PITCH, SEARCH_PITCH)
        set_gimbal(GIMBAL_TOPIC_YAW, SEARCH_YAW)
        self.cmd_pitch = SEARCH_PITCH
        self.cmd_yaw = SEARCH_YAW

    # ── Visual overlay ────────────────────────────────────────────

    def _draw_overlay(self, frame, target, w, h):
        """Draw tracking HUD on the frame."""
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

        # Show ACTUAL angles (from joint state) and whether feedback is active
        pitch_deg = math.degrees(self.actual_pitch)
        yaw_deg = math.degrees(self.actual_yaw)
        fb = 'FB' if self.joint_state_received else 'OL'
        angle_text = f'P:{pitch_deg:+.1f} Y:{yaw_deg:+.1f} [{fb}]'
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
            err_x = tcx - cx
            err_y = tcy - cy
            cv2.putText(frame, f'err: ({err_x:+.0f}, {err_y:+.0f})px',
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
