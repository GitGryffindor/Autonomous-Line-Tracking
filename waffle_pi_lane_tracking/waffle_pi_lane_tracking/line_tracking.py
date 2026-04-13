#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line_tracking.py — 4-state LiDAR-assisted line-following & obstacle
bypass state machine.  Runs entirely on-board a Raspberry Pi 4 (no GPU).

States:
  0  FOLLOWING_LINE  — OpenCV centroid line-follow + front LiDAR monitor
  1  EVADING         — 90° right turn using odometry yaw
  2  CLEARING        — drive straight, wall-follow left LiDAR slice
  3  ARC_RECOVERY    — forward-left arc until line is re-acquired
  4  STUCK           — recovery state (reverse then retry)

Subscriptions:  /image_raw/compressed, /scan, /odom
Publisher:      /cmd_vel
"""

import math
import sys
import time
from enum import Enum

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan

# ─────────────────────────── CONSTANTS ────────────────────────────
# Image processing (resized for Pi 4 real-time performance)
IMG_WIDTH = 320
IMG_HEIGHT = 240

# Line-following (State 0)
KP_STEER = 0.003                        # proportional gain  pixels → rad/s
FOLLOW_LINEAR_X = 0.15                   # forward speed while following (m/s)

# Front-cone LiDAR obstacle detection
FRONT_CONE_INDICES_LOW = 15              # 0 .. 14  (right of center)
FRONT_CONE_INDICES_HIGH = 345            # 345 .. 359  (left of center)
FRONT_CONE_THRESHOLD_M = 0.25           # obstacle trigger distance (m)

# State 1 — EVADING (90° right turn)
TURN_ANGULAR_Z = -0.5                    # right-turn angular velocity (rad/s)
TURN_TARGET_DEG = 88.0                   # yaw delta to exit State 1 (°)

# State 2 — CLEARING (straight drive, left-wall follow)
CLEAR_LINEAR_X = 0.12                    # forward speed (m/s)
CLEAR_LEFT_SLICE_START = 75              # left-side LiDAR start index
CLEAR_LEFT_SLICE_END = 105               # left-side LiDAR end index
CLEAR_DISTANCE_M = 1.0                   # obstacle-cleared threshold (m)
CLEAR_BUFFER_SEC = 0.15                  # rear-wheel clearance buffer (s)

# State 3 — ARC_RECOVERY
ARC_LINEAR_X = 0.10                      # arc forward speed (m/s)
ARC_ANGULAR_Z = 0.4                      # arc left-turn rate (rad/s)
ARC_LINE_AREA_MIN = 500                  # min contour area to re-detect line (px²)

# Watchdog timeouts (seconds)
TIMEOUT_EVADING_SEC = 5.0
TIMEOUT_CLEARING_SEC = 20.0
TIMEOUT_ARC_RECOVERY_SEC = 15.0

# STUCK recovery
STUCK_REVERSE_X = -0.1                   # reverse speed (m/s)
STUCK_REVERSE_SEC = 1.5                  # reverse duration (s)


# ──────────────────────────── STATE ENUM ──────────────────────────
class RobotState(Enum):
    """Robot state machine states."""
    FOLLOWING_LINE = 0
    EVADING = 1
    CLEARING = 2
    ARC_RECOVERY = 3
    STUCK = 4


# ──────────────────────────── HELPERS ─────────────────────────────
def quaternion_to_yaw(q):
    """Extract yaw angle (radians) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def sanitize_ranges(ranges):
    """Remove nan and inf from a LiDAR ranges list, return numpy array."""
    arr = np.array(ranges, dtype=np.float64)
    return arr[np.isfinite(arr)]


def angle_diff(target, current):
    """Shortest signed angular difference (radians), handles ±π wrap."""
    d = target - current
    return math.atan2(math.sin(d), math.cos(d))


# ──────────────────────────── NODE ────────────────────────────────
class LineTrackingController(Node):
    """4-state line-following + LiDAR obstacle bypass controller."""

    def __init__(self):
        super().__init__('line_tracking_controller')

        # ── State bookkeeping ──
        self.state = RobotState.FOLLOWING_LINE
        self.state_entry_time = self.get_clock().now()
        self.entry_yaw = 0.0
        self.clear_buffer_started = False
        self.clear_buffer_time = 0.0
        self.stuck_reverse_start = None

        # ── cv_bridge ──
        self.bridge = CvBridge()

        # ── QoS (sensor data) ──
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ──
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            sensor_qos,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            sensor_qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            sensor_qos,
        )

        # ── Publisher ──
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # ── Watchdog timer (10 Hz check) ──
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info('LineTrackingController started — State: FOLLOWING_LINE')

    # ──────────── velocity helpers ────────────
    def _publish_vel(self, linear_x=0.0, angular_z=0.0):
        """Publish a Twist message to /cmd_vel safely."""
        if not rclpy.ok():
            return
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        try:
            self.cmd_pub.publish(msg)
        except Exception:
            pass

    def _stop(self):
        """Publish zero velocity."""
        self._publish_vel(0.0, 0.0)

    # ──────────── state transitions ───────────
    def _transition(self, new_state):
        """Log and transition to a new state, reset state-entry time."""
        old = self.state.name
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        self.clear_buffer_started = False
        self.get_logger().info(f'State transition: {old} → {new_state.name}')

    def _state_elapsed_sec(self):
        """Seconds since current state was entered."""
        return (self.get_clock().now() - self.state_entry_time).nanoseconds * 1e-9

    # ─────────── OpenCV line detection ────────
    def _detect_line(self, cv_img):
        """
        Fast line-detection pipeline optimised for Pi 4.

        Pipeline: resize → grayscale → Gaussian blur → adaptive threshold
                  → find contours → largest contour → centroid.

        Returns (centroid_x, contour_area) or (None, 0) if no line found.
        """
        small = cv2.resize(cv_img, (IMG_WIDTH, IMG_HEIGHT),
                           interpolation=cv2.INTER_NEAREST)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 11, 2,
        )
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None, 0

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 50:                       # noise filter
            return None, 0

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, 0

        cx = int(M['m10'] / M['m00'])
        return cx, area

    # ─────────── front-cone LiDAR check ──────
    def _front_cone_min(self, ranges):
        """
        Extract front-cone LiDAR slice and return minimum distance.

        Handles array wrap-around: indices [0:15] ∪ [345:360].
        Returns float('inf') if no valid readings.
        """
        if len(ranges) == 0:
            return float('inf')

        n = len(ranges)
        hi_start = min(FRONT_CONE_INDICES_HIGH, n)
        lo_end = min(FRONT_CONE_INDICES_LOW, n)

        cone = list(ranges[0:lo_end]) + list(ranges[hi_start:n])
        clean = sanitize_ranges(cone)

        if len(clean) == 0:
            return float('inf')
        return float(np.min(clean))

    # ━━━━━━━━━━━━ CALLBACKS ━━━━━━━━━━━━
    def _image_callback(self, msg):
        """Handle incoming camera frames (raw Image — RGB888 from camera_ros)."""
        # CPU guard: skip image processing in States 1 & 2
        if self.state in (RobotState.EVADING, RobotState.CLEARING, RobotState.STUCK):
            return

        enc = msg.encoding.lower()
        try:
            if enc in ('nv21', 'nv12'):
                # cv_bridge cannot decode NV21/NV12 — bypass it entirely
                # Read raw YUV bytes directly from the ROS message
                yuv = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                    (msg.height * 3 // 2, msg.width))
                if enc == 'nv21':
                    cv_img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
                else:
                    cv_img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            elif enc == 'rgb8':
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                cv_img = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Image decode error ({msg.encoding}): {e}')
            return

        cx, area = self._detect_line(cv_img)

        # ── State 0: FOLLOWING_LINE ──
        if self.state == RobotState.FOLLOWING_LINE:
            if cx is not None:
                error = cx - IMG_WIDTH / 2
                angular_z = -KP_STEER * error
                self._publish_vel(FOLLOW_LINEAR_X, angular_z)
            else:
                # Lost line — slow down, keep last command
                self._publish_vel(FOLLOW_LINEAR_X * 0.5, 0.0)

        # ── State 3: ARC_RECOVERY — look for line ──
        elif self.state == RobotState.ARC_RECOVERY:
            if cx is not None and area > ARC_LINE_AREA_MIN:
                self.get_logger().info(
                    f'Line re-acquired (area={area:.0f}), realigning')
                self._stop()
                self._transition(RobotState.FOLLOWING_LINE)

    def _scan_callback(self, msg):
        """Handle incoming LiDAR scans."""
        ranges = msg.ranges

        # ── State 0: monitor front cone for obstacles ──
        if self.state == RobotState.FOLLOWING_LINE:
            f_min = self._front_cone_min(ranges)
            if f_min < FRONT_CONE_THRESHOLD_M:
                self.get_logger().info(f'Obstacle detected! dist={f_min:.2f}m')
                self._stop()
                self._transition(RobotState.EVADING)

        # ── State 2: left-wall follow ──
        elif self.state == RobotState.CLEARING:
            n = len(ranges)
            start = min(CLEAR_LEFT_SLICE_START, n)
            end = min(CLEAR_LEFT_SLICE_END, n)
            left_slice = ranges[start:end]
            clean = sanitize_ranges(left_slice)

            if len(clean) > 0:
                median_dist = float(np.median(clean))
                if median_dist >= CLEAR_DISTANCE_M and not self.clear_buffer_started:
                    self.get_logger().info(
                        f'Left side clear (median={median_dist:.2f} m), '
                        f'buffer {CLEAR_BUFFER_SEC}s')
                    self.clear_buffer_started = True
                    self.clear_buffer_time = time.monotonic()

            # keep driving forward
            self._publish_vel(CLEAR_LINEAR_X, 0.0)

            # check buffer expiry
            if self.clear_buffer_started:
                if (time.monotonic() - self.clear_buffer_time) >= CLEAR_BUFFER_SEC:
                    self._transition(RobotState.ARC_RECOVERY)
                    self._publish_vel(ARC_LINEAR_X, ARC_ANGULAR_Z)

        # ── State 3: monitor front cone during arc ──
        elif self.state == RobotState.ARC_RECOVERY:
            if self._front_cone_min(ranges) < FRONT_CONE_THRESHOLD_M:
                self.get_logger().warn(
                    'Obstacle detected during arc recovery — re-evading')
                self._stop()
                self._transition(RobotState.EVADING)

    def _odom_callback(self, msg):
        """Handle odometry for yaw tracking (State 1)."""
        if self.state == RobotState.EVADING:
            current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

            # snapshot yaw on first odom after entering EVADING
            if not hasattr(self, '_evade_yaw_set') or not self._evade_yaw_set:
                self.entry_yaw = current_yaw
                self._evade_yaw_set = True
                self.get_logger().info(
                    f'EVADING: entry yaw = {math.degrees(self.entry_yaw):.1f}°')

            # compute absolute delta with wrap-around
            delta = abs(angle_diff(self.entry_yaw, current_yaw))
            target_rad = math.radians(TURN_TARGET_DEG)

            if delta >= target_rad:
                self.get_logger().info(
                    f'EVADING complete: turned {math.degrees(delta):.1f}°')
                self._stop()
                self._evade_yaw_set = False
                self._transition(RobotState.CLEARING)
            else:
                # keep turning right
                self._publish_vel(0.0, TURN_ANGULAR_Z)
        else:
            # reset flag when not evading
            self._evade_yaw_set = False

    # ━━━━━━━━━━━━ WATCHDOG ━━━━━━━━━━━━
    def _watchdog_tick(self):
        """10 Hz watchdog timer — enforce state timeouts and STUCK recovery."""
        elapsed = self._state_elapsed_sec()

        if self.state == RobotState.EVADING and elapsed > TIMEOUT_EVADING_SEC:
            self.get_logger().warn(
                f'EVADING timeout ({elapsed:.1f}s) — entering STUCK')
            self._stop()
            self._transition(RobotState.STUCK)
            self.stuck_reverse_start = None

        elif self.state == RobotState.CLEARING and elapsed > TIMEOUT_CLEARING_SEC:
            self.get_logger().warn(
                f'CLEARING timeout ({elapsed:.1f}s) — entering STUCK')
            self._stop()
            self._transition(RobotState.STUCK)
            self.stuck_reverse_start = None

        elif self.state == RobotState.ARC_RECOVERY and elapsed > TIMEOUT_ARC_RECOVERY_SEC:
            self.get_logger().warn(
                f'ARC_RECOVERY timeout ({elapsed:.1f}s) — entering STUCK')
            self._stop()
            self._transition(RobotState.STUCK)
            self.stuck_reverse_start = None

        elif self.state == RobotState.STUCK:
            self._handle_stuck()

    def _handle_stuck(self):
        """STUCK recovery: reverse briefly then retry EVADING."""
        if self.stuck_reverse_start is None:
            self.get_logger().error('STUCK — reversing for recovery')
            self.stuck_reverse_start = time.monotonic()
            self._publish_vel(STUCK_REVERSE_X, 0.0)
        elif (time.monotonic() - self.stuck_reverse_start) >= STUCK_REVERSE_SEC:
            self.get_logger().info('STUCK recovery complete — retrying EVADING')
            self._stop()
            self.stuck_reverse_start = None
            self._transition(RobotState.EVADING)

    # ━━━━━━━━━━━━ SHUTDOWN ━━━━━━━━━━━━
    def destroy_node(self):
        """Ensure motors are stopped before shutdown."""
        self.get_logger().info('Shutting down — stopping motors')
        self._stop()
        super().destroy_node()


# ─────────────────────────── MAIN ─────────────────────────────────
def main(args=None):
    """Entry point for the line_tracking node."""
    rclpy.init(args=args)
    node = LineTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
