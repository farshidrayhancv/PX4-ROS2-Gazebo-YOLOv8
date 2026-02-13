#!/usr/bin/env python3
"""Gimbal car-tracking pipeline for PX4 drone simulation.

Subscribes to /camera (ROS 2 Image), runs YOLOv8 to detect cars, and
controls the gimbal (pitch + yaw) to keep the tracked car centered in
the camera frame.

State machine:
  SEARCHING -> ACQUIRING -> TRACKING -> LOST -> SEARCHING

Writes a per-frame CSV log to /tmp/gimbal_tracker.csv for diagnostics.
"""

import csv
import math
import os
import subprocess
import time
from enum import Enum, auto

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

# ── Gimbal topics (must match setup_gimbal.py 'manual/*' naming) ──
GIMBAL_TOPIC_PITCH = '/model/x500_gimbal_0/manual/gimbal_pitch'
GIMBAL_TOPIC_YAW = '/model/x500_gimbal_0/manual/gimbal_yaw'

# ── Gimbal limits (internal frame: 0 = forward) ──────────────────
GIMBAL_PITCH_MIN = -2.356   # -135 deg (max downward)
GIMBAL_PITCH_MAX = 0.785    # +45 deg (max upward)
GIMBAL_YAW_MIN = -3.14
GIMBAL_YAW_MAX = 3.14

# ── Search pose (default yaw, tilted down) ────────────────────────
SEARCH_PITCH = -0.9599      # -55 deg below horizontal
SEARCH_YAW = 0.0            # no yaw rotation — camera stays at default orientation

# ── Camera parameters ────────────────────────────────────────────
HFOV = 1.047                # horizontal FOV in rad (60 deg), from model SDF

# ── Nonlinear PD controller gains ───────────────────────────────
Kp = 0.5                    # proportional gain (on angular error)
Kd = 0.15                   # derivative gain
CTRL_EXPONENT = 2.0         # power curve exponent for proportional term
                            # >1 = quadratic: small errors → tiny corrections,
                            #       large errors → stronger corrections
                            # like car steering — progressive, smooth response
DEADZONE_PX = 15            # pixel deadzone to prevent jitter
MAX_CMD_STEP = 0.02         # rad per frame (~1.1 deg) — rate limiter
                            # prevents software position racing ahead of
                            # physical gimbal (subprocess + PID latency)

# ── Detection settings ───────────────────────────────────────────
YOLO_MODEL = 'yolov8m.pt'
TARGET_CLASS = 2             # COCO class 2 = car
CONF_THRESHOLD = 0.4

# ── Tracking state parameters ────────────────────────────────────
HOLD_TIME = 3.0              # seconds to hold position after losing track
SEARCH_RETURN_SPEED = 0.3    # rad/sec for smooth return to search pose
ACQUIRING_THRESHOLD = 30     # pixels from center to consider "acquired"
ACQUIRING_FRAMES = 5         # consecutive centered frames to confirm lock
MAX_TARGET_JUMP_PX = 200     # reject detections that jump more than this
                             # from last known position (prevents false
                             # positives from jerking the gimbal)

# ── Log file ──────────────────────────────────────────────────────
LOG_PATH = '/tmp/gimbal_tracker.csv'


class State(Enum):
    SEARCHING = auto()
    ACQUIRING = auto()
    TRACKING = auto()
    LOST = auto()


# ── Gimbal subprocess helper (same pattern as keyboard controller) ─
_gimbal_procs = {}


def set_gimbal(topic, value):
    """Publish a gimbal joint position via gz topic subprocess."""
    prev = _gimbal_procs.get(topic)
    if prev is not None and prev.poll() is None:
        return  # previous command still running — throttle
    _gimbal_procs[topic] = subprocess.Popen(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double',
         '-p', f'data: {value:.4f}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


class GimbalTracker(Node):
    def __init__(self):
        super().__init__('gimbal_tracker')

        # ROS 2 subscription — queue_size=1 so we always get the latest frame
        self.subscription = self.create_subscription(
            Image, 'camera', self.on_image, 1)
        self.br = CvBridge()

        # YOLOv8
        self.model = YOLO(YOLO_MODEL)

        # Gimbal state
        self.gimbal_pitch = SEARCH_PITCH
        self.gimbal_yaw = SEARCH_YAW

        # Tracking state
        self.state = State.SEARCHING
        self.last_target_center = None
        self.last_detection_time = 0.0
        self.acquiring_count = 0
        self.frame_num = 0

        # PD controller state
        self.prev_err_ang_x = 0.0
        self.prev_err_ang_y = 0.0

        # Last PD commands (for logging when no detection)
        self.last_cmd_yaw = 0.0
        self.last_cmd_pitch = 0.0

        # Display window
        cv2.namedWindow('Gimbal Tracker', cv2.WINDOW_NORMAL)

        # ── CSV log file ──────────────────────────────────────────
        self.log_file = open(LOG_PATH, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            'frame', 'time', 'state',
            'detected', 'det_cx', 'det_cy', 'det_w', 'det_h', 'det_conf',
            'img_w', 'img_h',
            'err_px_x', 'err_px_y',
            'err_ang_x', 'err_ang_y',
            'cmd_yaw', 'cmd_pitch',
            'gimbal_yaw', 'gimbal_pitch',
            'gimbal_yaw_deg', 'gimbal_pitch_deg',
        ])
        self.log_file.flush()

        # Set gimbal to search pose
        set_gimbal(GIMBAL_TOPIC_PITCH, self.gimbal_pitch)
        set_gimbal(GIMBAL_TOPIC_YAW, self.gimbal_yaw)

        self.get_logger().info(
            f'Gimbal tracker initialized — search pose pitch={math.degrees(SEARCH_PITCH):.0f}°, '
            f'logging to {LOG_PATH}')

    def on_image(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        now = time.time()
        self.frame_num += 1

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
        log_cmd_yaw = log_cmd_pitch = 0.0

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
                self.prev_err_ang_x = 0.0
                self.prev_err_ang_y = 0.0
                self.get_logger().info(
                    f'Car detected at ({tcx:.0f}, {tcy:.0f}) conf={conf:.2f} — acquiring...')

            elif self.state == State.LOST:
                self.state = State.TRACKING
                self.get_logger().info('Car re-acquired — tracking.')

            # Apply PD gimbal correction (returns log values)
            log_err_px_x, log_err_px_y, log_err_ang_x, log_err_ang_y, \
                log_cmd_yaw, log_cmd_pitch = self._track_target(tcx, tcy, w, h)

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
                    f'gimbal P:{math.degrees(self.gimbal_pitch):+.1f}° '
                    f'Y:{math.degrees(self.gimbal_yaw):+.1f}°')

            elif self.state == State.LOST:
                elapsed = now - self.last_detection_time
                if elapsed >= HOLD_TIME:
                    self._return_to_search()
                    if (abs(self.gimbal_pitch - SEARCH_PITCH) < 0.05 and
                            abs(self.gimbal_yaw - SEARCH_YAW) < 0.05):
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
            f'{log_cmd_yaw:.4f}', f'{log_cmd_pitch:.4f}',
            f'{self.gimbal_yaw:.4f}', f'{self.gimbal_pitch:.4f}',
            f'{math.degrees(self.gimbal_yaw):.1f}',
            f'{math.degrees(self.gimbal_pitch):.1f}',
        ])
        # Flush every 50 frames to keep I/O reasonable
        if self.frame_num % 50 == 0:
            self.log_file.flush()

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
            # Persistence: pick detection closest to last known position
            centers_x = (xyxy[:, 0] + xyxy[:, 2]) / 2
            centers_y = (xyxy[:, 1] + xyxy[:, 3]) / 2
            lx, ly = self.last_target_center
            dx = centers_x - lx
            dy = centers_y - ly
            distances = (dx ** 2 + dy ** 2).sqrt()

            # Filter out detections that jump too far from last known position
            valid_mask = distances <= MAX_TARGET_JUMP_PX
            if not valid_mask.any():
                return None  # all detections too far — likely false positives

            # Among valid detections, pick closest
            distances[~valid_mask] = float('inf')
            best_idx = distances.argmin().item()
        else:
            # Pick largest bounding box (most prominent car)
            areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
            best_idx = areas.argmax().item()

        box = xyxy[best_idx]
        x1, y1, x2, y2 = box.tolist()
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        conf = float(boxes.conf[best_idx])
        return (cx, cy, x1, y1, x2, y2, conf)

    # ── Nonlinear PD gimbal control ─────────────────────────────────

    def _track_target(self, target_cx, target_cy, img_w, img_h):
        """Compute and apply nonlinear PD gimbal correction to center the target.

        Uses a quadratic (power-curve) proportional term so small errors
        produce tiny corrections and large errors produce stronger ones,
        similar to car steering.  The derivative term remains linear for
        damping.  A rate limiter caps the max step per frame.

        Returns (err_px_x, err_px_y, err_ang_x, err_ang_y, cmd_yaw, cmd_pitch)
        for logging.
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

        # Derivative term
        d_err_x = err_ang_x - self.prev_err_ang_x
        d_err_y = err_ang_y - self.prev_err_ang_y
        self.prev_err_ang_x = err_ang_x
        self.prev_err_ang_y = err_ang_y

        # Nonlinear PD command (quadratic proportional):
        #   P term: sign(err) * Kp * |err|^CTRL_EXPONENT
        #     → small errors get tiny corrections, large errors get stronger ones
        #   D term: linear damping (prevents overshoot)
        #   Both axes NEGATE (empirically verified from 48-frame log analysis):
        #   Yaw:   car right (+err) → -(pos) = -cmd → decrease yaw → pan right
        #          car left  (-err) → -(neg) = +cmd → increase yaw → pan left
        #   Pitch: car below (+err) → -(pos) = -cmd → tilt down to follow
        #          car above (-err) → -(neg) = +cmd → tilt up to follow
        p_yaw = math.copysign(Kp * abs(err_ang_x) ** CTRL_EXPONENT, err_ang_x)
        p_pitch = math.copysign(Kp * abs(err_ang_y) ** CTRL_EXPONENT, err_ang_y)
        cmd_yaw = -(p_yaw + Kd * d_err_x)
        cmd_pitch = -(p_pitch + Kd * d_err_y)

        # Rate limiter — prevents software position from racing ahead of
        # the physical gimbal (subprocess throttling + Gazebo PID latency).
        # Without this, large errors cause the software to accumulate
        # corrections much faster than the hardware can follow.
        cmd_yaw = max(-MAX_CMD_STEP, min(MAX_CMD_STEP, cmd_yaw))
        cmd_pitch = max(-MAX_CMD_STEP, min(MAX_CMD_STEP, cmd_pitch))

        self.last_cmd_yaw = cmd_yaw
        self.last_cmd_pitch = cmd_pitch

        # Apply and clamp
        self.gimbal_yaw = max(GIMBAL_YAW_MIN, min(
            GIMBAL_YAW_MAX, self.gimbal_yaw + cmd_yaw))
        self.gimbal_pitch = max(GIMBAL_PITCH_MIN, min(
            GIMBAL_PITCH_MAX, self.gimbal_pitch + cmd_pitch))

        # Publish to Gazebo
        set_gimbal(GIMBAL_TOPIC_YAW, self.gimbal_yaw)
        set_gimbal(GIMBAL_TOPIC_PITCH, self.gimbal_pitch)

        return (raw_err_px_x, raw_err_px_y, err_ang_x, err_ang_y,
                cmd_yaw, cmd_pitch)

    # ── Return to search pose ─────────────────────────────────────

    def _return_to_search(self):
        """Smoothly slew gimbal back to the search pose (straight down)."""
        dt = 0.1  # approximate frame interval
        max_step = SEARCH_RETURN_SPEED * dt

        # Pitch
        pitch_err = SEARCH_PITCH - self.gimbal_pitch
        if abs(pitch_err) > max_step:
            self.gimbal_pitch += max_step * (1 if pitch_err > 0 else -1)
        else:
            self.gimbal_pitch = SEARCH_PITCH

        # Yaw
        yaw_err = SEARCH_YAW - self.gimbal_yaw
        if abs(yaw_err) > max_step:
            self.gimbal_yaw += max_step * (1 if yaw_err > 0 else -1)
        else:
            self.gimbal_yaw = SEARCH_YAW

        set_gimbal(GIMBAL_TOPIC_PITCH, self.gimbal_pitch)
        set_gimbal(GIMBAL_TOPIC_YAW, self.gimbal_yaw)

    # ── Visual overlay ────────────────────────────────────────────

    def _draw_overlay(self, frame, target, w, h):
        """Draw tracking HUD on the frame."""
        cx, cy = w // 2, h // 2

        # Crosshair at image center
        cv2.line(frame, (cx - 25, cy), (cx + 25, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 25), (cx, cy + 25), (0, 255, 0), 1)

        # State indicator colors
        state_colors = {
            State.SEARCHING: (128, 128, 128),   # gray
            State.ACQUIRING: (0, 255, 255),      # yellow
            State.TRACKING:  (0, 255, 0),        # green
            State.LOST:      (0, 0, 255),        # red
        }
        color = state_colors[self.state]

        # State label (top-left)
        cv2.putText(frame, f'[{self.state.name}]', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Frame counter (below state)
        cv2.putText(frame, f'F:{self.frame_num}', (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Gimbal angles (top-right)
        pitch_deg = math.degrees(self.gimbal_pitch)
        yaw_deg = math.degrees(self.gimbal_yaw)
        angle_text = f'P:{pitch_deg:+.1f} Y:{yaw_deg:+.1f}'
        text_size = cv2.getTextSize(
            angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
        cv2.putText(frame, angle_text, (w - text_size[0] - 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        if target is not None:
            tcx, tcy, x1, y1, x2, y2, conf = target

            # Bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          color, 2)
            cv2.putText(frame, f'car {conf:.0%}',
                        (int(x1), int(y1) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # Error vector arrow from center to target
            cv2.arrowedLine(frame, (cx, cy), (int(tcx), int(tcy)),
                            (0, 165, 255), 2, tipLength=0.15)

            # Pixel error readout (bottom-left)
            err_x = tcx - cx
            err_y = tcy - cy
            cv2.putText(frame, f'err: ({err_x:+.0f}, {err_y:+.0f})px',
                        (10, h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def destroy_node(self):
        """Flush and close the log file on shutdown."""
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
