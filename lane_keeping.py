#!/usr/bin/env python3
"""Vision-based lane keeping for the hatchback car on Sonoma Raceway.

PID controller that stays between the two yellow lane lines.
In curves where only the inside lane is visible, follows the lane's
curvature instead of being repelled from it.

  - Finds the inner edge (road-side edge) of each yellow lane line
  - Computes a "danger" score for left and right walls
  - PID on the danger difference (right_danger - left_danger)
  - Single-lane curve following: estimates lane angle via linear
    regression and blends it with the danger-based steering
  - Constant 3.5 m/s forward speed

Requires two bridges:
    ros2 run ros_gz_bridge parameter_bridge \
        /car/camera@sensor_msgs/msg/Image[gz.msgs.Image \
        --ros-args -r /car/camera:=/car_camera

    ros2 run ros_gz_bridge parameter_bridge \
        '/model/hatchback_blue_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
"""

import csv
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MIN_SPEED = 2.5           # speed during corrections (m/s)
MAX_SPEED = 5.0           # top speed on straights (m/s)
SPEED_RAMP_UP = 0.005     # exponential ramp-up rate per frame (very slow ~30s to near max)
SPEED_DROP = 0.6          # how fast speed drops when correcting (0-1, lower=faster drop)

# Spawn position: right lane near pickup_2, heading -0.7 rad
SPAWN_X = 294.55
SPAWN_Y = -148.30
SPAWN_Z = 3.5
MAX_STEER_LEFT = 0.4      # max left turn (positive angular.z)
MAX_STEER_RIGHT = -0.35   # max right turn (negative angular.z)

# Adaptive PID gains — KP scales quadratically with danger
KP_MIN = 0.3             # gain when lanes are far (gentle on straights)
KP_MAX = 2.5             # gain when lane is very close (strong near walls)
KI = 0.03                # integral gain (low to avoid oscillation)
KD = 0.15                # derivative gain (low — error is noisy at 15Hz)
I_MAX = 0.15             # anti-windup clamp for integral term

SMOOTHING = 0.25         # exponential smoothing (0=ignore new, 1=no smoothing)

# Steering ramp: correction starts gentle and builds over sustained detection
RAMP_FRAMES = 12         # frames to reach full correction (~0.8s at 15Hz)
RAMP_DECAY = 0.85        # how fast ramp decays when lanes disappear

# Curve-following: when only one lane visible
CURVE_ANGLE_GAIN = 0.3   # how much lane angle influences steering
MAX_LANE_ANGLE = 0.3     # clamp raw lane angle to ±0.3 rad (~17 deg)

# Trend steering: slow-moving average that remembers "which way we were turning"
# Used for HOLD/single-lane so post-PID overcorrection doesn't lose the turn
TREND_ALPHA = 0.03       # update rate (~23 frame half-life = 1.5s at 15Hz)

# Merge detection: both lanes very close = merging onto main racetrack
# At merge the car's right lane becomes the main track's left lane.
# Action: steer right to merge, then resume normal once on main track.
MERGE_DANGER_THRESH = 0.90   # both dangers must exceed this to trigger
MERGE_TRIGGER_FRAMES = 20    # sustained frames above threshold to confirm (~1.3s)
MERGE_STEER = -0.25          # steer right during merge (stronger)
MERGE_SPEED = 2.5            # speed during merge
MERGE_DURATION = 2.0         # seconds to hold merge steer
MERGE_COOLDOWN = 60.0        # seconds after merge before it can retrigger

# Yellow lane HSV thresholds
YELLOW_LOW = np.array([18, 80, 30])
YELLOW_HIGH = np.array([40, 255, 255])

# Full ROI: bottom 60% of image (for yellow detection + angle estimation)
ROI_TOP_FRAC = 0.40

# Near-field zone: just below mid-image (for left/right classification only)
# Lanes right beside the car — can't cross center here even in sharp curves
NEAR_TOP_FRAC = 0.45

# Minimum yellow pixels on one side to count as a lane
MIN_PIXELS = 40


class LaneKeepingNode(Node):
    def __init__(self):
        super().__init__("lane_keeping")
        self.create_subscription(Image, "/car_camera", self.on_image, 5)
        self.cmd_pub = self.create_publisher(
            Twist, "/model/hatchback_blue_1/cmd_vel", 10
        )
        self.br = CvBridge()
        self.last_steer = 0.0
        self.trend_steer = 0.0  # slow-moving average of turn direction
        self.prev_error = 0.0
        self.integral = 0.0
        self.current_speed = MIN_SPEED
        self.steer_ramp = 0.0  # 0..1, builds up over sustained lane detection
        self.frame_count = 0
        # Merge state machine
        self.narrow_count = 0        # consecutive frames with both dangers > threshold
        self.merge_active = False    # currently in merge manoeuvre
        self.merge_start_time = 0.0  # when merge started
        self.merge_end_time = 0.0    # when last merge ended (for cooldown)
        # CSV log
        self.log_file = open("/tmp/lane_keeping_log.csv", "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "frame", "time", "have_L", "have_R", "L_danger", "R_danger",
            "lane_angle", "ramp", "pid_or_curve", "raw_steer", "final_steer",
            "speed", "mode", "last_steer_before",
        ])
        cv2.namedWindow("Lane Keeping", cv2.WINDOW_NORMAL)
        self.get_logger().info("Lane keeping node started (PID + curve following, logging to /tmp/lane_keeping_log.csv)")

    @staticmethod
    def _lane_angle(ys, xs, h_roi):
        """Estimate lane line angle from yellow pixel coordinates.

        Fits a line to (y, x) pixels.  Returns the angle in radians of the
        lane relative to vertical — positive means the lane curves right
        (top of lane is to the right of bottom).
        """
        if len(ys) < 30:
            return 0.0
        # Fit x = a*y + b   (y is the vertical axis in ROI)
        try:
            a, _ = np.polyfit(ys, xs, 1)
        except (np.linalg.LinAlgError, ValueError):
            return 0.0
        # 'a' is dx/dy — positive means lane goes rightward as you go down
        # Convert to a steering-relevant angle: arctan(slope)
        return float(np.arctan(a))

    def on_image(self, msg: Image):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]
        img_cx = w / 2.0

        # --- Yellow mask on bottom portion (full ROI) ---
        roi_top = int(h * ROI_TOP_FRAC)
        roi = frame[roi_top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)

        ys, xs = np.where(mask > 0)

        # --- Classify lanes using NEAR-FIELD strip only ---
        # The near-field (bottom 25% of image) shows lanes right beside
        # the car. Even in sharp curves, lanes can't cross center here.
        # This prevents a lane curving across the top from being
        # misclassified as the wrong side.
        near_top_in_roi = int(h * NEAR_TOP_FRAC) - roi_top
        near_mask = ys >= near_top_in_roi
        near_xs = xs[near_mask]

        near_left = near_xs[near_xs < img_cx]
        near_right = near_xs[near_xs >= img_cx]

        have_left = len(near_left) >= MIN_PIXELS
        have_right = len(near_right) >= MIN_PIXELS

        # --- Gather ALL yellow pixels for the detected side(s) ---
        # Use full ROI for danger + angle, but only for sides confirmed
        # by the near-field classification.
        left_mask_all = xs < img_cx
        right_mask_all = xs >= img_cx
        left_xs = xs[left_mask_all] if have_left else np.array([])
        left_ys = ys[left_mask_all] if have_left else np.array([])
        right_xs = xs[right_mask_all] if have_right else np.array([])
        right_ys = ys[right_mask_all] if have_right else np.array([])

        # --- Compute danger for each side ---
        left_danger = 0.0
        right_danger = 0.0
        left_inner = 0.0
        right_inner = float(w)

        if have_left:
            left_inner = float(np.percentile(near_left, 95))
            left_danger = left_inner / img_cx

        if have_right:
            right_inner = float(np.percentile(near_right, 5))
            right_danger = (w - right_inner) / img_cx

        # --- Steering ramp: builds up over sustained detection ---
        only_left = have_left and not have_right
        only_right = have_right and not have_left
        last_steer_before = self.last_steer
        lane_angle_log = 0.0
        pid_or_curve_log = ""
        raw_steer_log = 0.0

        if have_left or have_right:
            self.steer_ramp = min(1.0, self.steer_ramp + 1.0 / RAMP_FRAMES)
        else:
            self.steer_ramp *= RAMP_DECAY

        # --- Merge detection state machine ---
        # When both lanes are very close for a sustained period, we're at
        # the merge onto the main racetrack. Steer right to merge, then
        # resume normal lane keeping after MERGE_DURATION seconds.
        if not self.merge_active:
            # Only allow merge if cooldown has elapsed since last merge
            cooldown_ok = (time.time() - self.merge_end_time) > MERGE_COOLDOWN
            if (cooldown_ok and have_left and have_right
                    and left_danger > MERGE_DANGER_THRESH
                    and right_danger > MERGE_DANGER_THRESH):
                self.narrow_count += 1
            else:
                self.narrow_count = 0

            if self.narrow_count >= MERGE_TRIGGER_FRAMES:
                self.merge_active = True
                self.merge_start_time = time.time()
                self.narrow_count = 0
                self.integral = 0.0
                self.get_logger().info(
                    f"MERGE triggered at frame {self.frame_count} — steering right for {MERGE_DURATION}s")
        else:
            elapsed = time.time() - self.merge_start_time
            if elapsed >= MERGE_DURATION:
                self.merge_active = False
                self.merge_end_time = time.time()
                self.integral = 0.0
                self.trend_steer = 0.0
                self.get_logger().info(
                    f"MERGE complete at frame {self.frame_count} — resuming normal lane keeping")

        # --- Steering logic ---
        if self.merge_active:
            # ---- MERGE MODE: steer right, ignore left lane ----
            elapsed = time.time() - self.merge_start_time
            steer = MERGE_STEER
            self.last_steer = steer
            pid_or_curve_log = f"MERGE t={elapsed:.1f}/{MERGE_DURATION:.0f}s"
            raw_steer_log = MERGE_STEER

        elif have_left and have_right:
            # ---- BOTH LANES VISIBLE: PID + road curvature feedforward ----
            error = right_danger - left_danger

            # Reset integral on sign change to prevent post-turn overcorrection
            if (error > 0 and self.integral < 0) or (error < 0 and self.integral > 0):
                self.integral = 0.0
            self.integral += error
            self.integral = max(-I_MAX, min(I_MAX, self.integral))
            derivative = error - self.prev_error
            self.prev_error = error

            # Adaptive KP: quadratic with max danger
            # Far lanes → gentle, close lanes → aggressive
            max_danger = max(left_danger, right_danger)
            adaptive_kp = KP_MIN + (KP_MAX - KP_MIN) * max_danger ** 2

            pid_steer = adaptive_kp * error + KI * self.integral + KD * derivative

            # Feedforward: average lane angle tells us which way the road curves
            # so we steer into curves even when centered between lanes
            left_ang = self._lane_angle(left_ys, left_xs, h - roi_top)
            right_ang = self._lane_angle(right_ys, right_xs, h - roi_top)
            road_angle = (left_ang + right_ang) / 2.0
            road_angle = max(-MAX_LANE_ANGLE, min(MAX_LANE_ANGLE, road_angle))
            feedforward = CURVE_ANGLE_GAIN * road_angle

            raw_steer = (pid_steer + feedforward) * self.steer_ramp
            raw_steer = max(MAX_STEER_RIGHT, min(MAX_STEER_LEFT, raw_steer))
            steer = SMOOTHING * raw_steer + (1 - SMOOTHING) * self.last_steer
            self.last_steer = steer
            pid_or_curve_log = f"PID e={error:.3f} kp={adaptive_kp:.2f} ff={feedforward:.3f}"
            raw_steer_log = raw_steer

        elif only_right or only_left:
            # ---- SINGLE LANE: adaptive response based on danger ----
            if only_right:
                lane_angle = self._lane_angle(right_ys, right_xs, h - roi_top)
                danger = right_danger
            else:
                lane_angle = self._lane_angle(left_ys, left_xs, h - roi_top)
                danger = left_danger
            lane_angle_log = lane_angle

            lane_angle = max(-MAX_LANE_ANGLE, min(MAX_LANE_ANGLE, lane_angle))

            # Adaptive: high danger = react strongly (bypass ramp), low = gentle
            urgency = danger ** 2

            # Steer AWAY from the dangerous lane
            if only_right:
                dodge = urgency * 0.4    # right lane close → steer left (positive)
            else:
                dodge = -urgency * 0.4   # left lane close → steer right (negative)

            # Blend: more urgency = more dodge, otherwise follow turn trend
            blend = min(0.5, urgency)  # up to 50% new when very close
            steer = (1 - blend) * self.trend_steer + blend * dodge
            steer = max(MAX_STEER_RIGHT, min(MAX_STEER_LEFT, steer))
            raw_steer_log = dodge
            self.last_steer = steer
            pid_or_curve_log = f"CURVE ang={lane_angle_log:.3f} dng={danger:.2f} dodge={dodge:.3f} trend={self.trend_steer:.3f}"

        else:
            # ---- NO LANES: follow turn trend ----
            steer = self.trend_steer
            self.last_steer = steer
            pid_or_curve_log = f"HOLD trend={self.trend_steer:.3f}"

        # --- Update trend (slow-moving average of turn direction) ---
        self.trend_steer = (1 - TREND_ALPHA) * self.trend_steer + TREND_ALPHA * steer

        # --- Log ---
        self.frame_count += 1
        mode = "MERGE" if self.merge_active else (
            "BOTH" if (have_left and have_right) else (
                "L" if only_left else ("R" if only_right else "NONE")))
        self.log_writer.writerow([
            self.frame_count, f"{time.time():.3f}",
            int(have_left), int(have_right),
            f"{left_danger:.3f}", f"{right_danger:.3f}",
            f"{lane_angle_log:.3f}", f"{self.steer_ramp:.3f}",
            pid_or_curve_log, f"{raw_steer_log:.3f}",
            f"{steer:.3f}", f"{self.current_speed:.2f}",
            mode, f"{last_steer_before:.3f}",
        ])
        self.log_file.flush()

        # --- Speed ---
        no_lanes = not have_left and not have_right
        steer_amount = abs(steer)
        if self.merge_active:
            # Merge manoeuvre — controlled slow speed
            self.current_speed = max(MERGE_SPEED, self.current_speed * 0.92)
        elif no_lanes:
            # No lanes at all — slow to crawl while holding heading
            self.current_speed = max(1.0, self.current_speed * 0.9)
        elif steer_amount > 0.08:
            # Correcting — drop speed proportional to steering
            target_speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * max(0, 1 - steer_amount / MAX_STEER_LEFT)
            self.current_speed = SPEED_DROP * target_speed + (1 - SPEED_DROP) * self.current_speed
        else:
            # Straight — exponential ramp-up toward max speed
            self.current_speed += SPEED_RAMP_UP * (MAX_SPEED - self.current_speed)
        self.current_speed = max(1.0 if no_lanes else MIN_SPEED, min(MAX_SPEED, self.current_speed))

        # --- Send command ---
        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = steer
        self.cmd_pub.publish(twist)

        # --- Debug visualisation ---
        debug = frame.copy()
        mask_color = np.zeros_like(roi)
        mask_color[mask > 0] = (0, 255, 255)
        debug[roi_top:] = cv2.addWeighted(roi, 0.7, mask_color, 0.3, 0)

        if have_left:
            li = int(left_inner)
            cv2.line(debug, (li, roi_top), (li, h), (0, 0, 255), 3)
        if have_right:
            ri = int(right_inner)
            cv2.line(debug, (ri, roi_top), (ri, h), (255, 0, 0), 3)
        cv2.line(debug, (int(img_cx), roi_top), (int(img_cx), h), (0, 255, 0), 1)
        # Near-field boundary line
        near_line_y = int(h * NEAR_TOP_FRAC)
        cv2.line(debug, (0, near_line_y), (w, near_line_y), (255, 255, 0), 1)

        # HUD
        col = (0, 255, 0) if (have_left or have_right) else (0, 0, 255)
        mode = "BOTH" if (have_left and have_right) else (
            "LEFT" if have_left else ("RIGHT" if have_right else "NONE"))
        cv2.putText(debug, f"#{self.frame_count} L:{left_danger:.2f} R:{right_danger:.2f} [{mode}]",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        cv2.putText(debug, f"steer:{steer:+.2f} spd:{self.current_speed:.1f} ramp:{self.steer_ramp:.0%}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("Lane Keeping", debug)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneKeepingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
