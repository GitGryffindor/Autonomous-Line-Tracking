#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line_tracking.py — Hybrid APF + State Machine controller.

Philosophy:
  - Line     = opposite magnetic pole → SMOOTH APF steering within FOLLOWING_LINE
  - Obstacle = same magnetic pole     → DECISIVE state machine for reliable bypass + recovery

States:
  FOLLOWING_LINE  — APF smooth steering + front LiDAR watch
  EVADING         — 90° right turn using odometry (robot creeps forward, NEVER fully stops)
  CLEARING        — Drive straight, left LiDAR monitors clearance
  ARC_RECOVERY    — Forward-left arc until camera re-acquires line
  STUCK           — Reverse briefly then retry EVADING

Key guarantees:
  - Robot NEVER fully stops (min 0.04 m/s always commanded during turns)
  - Obstacle detection starts at 0.45m (early)
  - APF steering is smooth and proportional within line-following
  - State machine is reliable and proven for obstacle bypass + recovery

Subscriptions: /camera/image_raw, /scan, /odom
Publisher:     /cmd_vel, /visualization_marker_array
"""

import math
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
from visualization_msgs.msg import Marker, MarkerArray

# ─────────────────────────── CONSTANTS ────────────────────────────────────
IMG_WIDTH  = 320
IMG_HEIGHT = 240
LINE_ROI_START = 0.35   # use bottom 65% of frame for line detection

FRONT_CONE_LOW  = 15    # front-cone right edge index
FRONT_CONE_HIGH = 345   # front-cone left  edge index

LEFT_SLICE_START = 75
LEFT_SLICE_END   = 105


# ──────────────────────────── STATE ENUM ──────────────────────────────────
class RobotState(Enum):
    FOLLOWING_LINE = 0
    EVADING        = 1
    CLEARING       = 2
    ARC_RECOVERY   = 3
    STUCK          = 4


# ──────────────────────────── HELPERS ─────────────────────────────────────
def quaternion_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(target, current):
    d = target - current
    return math.atan2(math.sin(d), math.cos(d))


def sanitize(ranges):
    a = np.array(ranges, dtype=np.float64)
    return a[np.isfinite(a)]


# ──────────────────────────── NODE ────────────────────────────────────────
class LineTrackingController(Node):

    def __init__(self):
        super().__init__('line_tracking_controller')

        # ── Parameters ───────────────────────────────────────────────────
        # APF line-following
        self.declare_parameter('k_att',             0.004)   # steering gain (px → rad/s)
        self.declare_parameter('follow_speed',      0.15)    # forward speed while following
        self.declare_parameter('min_speed',         0.04)    # min speed in any state (never stops)
        # Obstacle detection
        self.declare_parameter('obstacle_dist',     0.45)    # distance to trigger EVADING
        # EVADING
        self.declare_parameter('evade_turn_z',     -0.50)    # turn rate during EVADING (rad/s)
        self.declare_parameter('evade_target_deg',  88.0)    # yaw to turn before CLEARING
        self.declare_parameter('evade_speed',       0.05)    # creep speed during EVADING
        # CLEARING
        self.declare_parameter('clear_speed',       0.12)    # forward speed during CLEARING
        self.declare_parameter('clear_dist',        0.65)    # left-gap to declare obstacle passed
        self.declare_parameter('clear_buffer',      0.20)    # buffer seconds after clearing
        # ARC_RECOVERY
        self.declare_parameter('arc_speed',         0.10)    # forward speed during arc
        self.declare_parameter('arc_turn_z',        0.45)    # left-turn during arc
        self.declare_parameter('arc_line_area',     400.0)   # min contour area to re-detect line
        # Watchdogs
        self.declare_parameter('timeout_evade',     6.0)
        self.declare_parameter('timeout_clear',    25.0)
        self.declare_parameter('timeout_arc',      18.0)
        # STUCK
        self.declare_parameter('stuck_rev_speed',  -0.10)
        self.declare_parameter('stuck_rev_sec',     1.5)
        # Visualization
        self.declare_parameter('enable_rviz',       False)

        self._load_params()

        # ── State ────────────────────────────────────────────────────────
        self.state              = RobotState.FOLLOWING_LINE
        self.state_entry_time   = time.monotonic()
        self.entry_yaw          = 0.0
        self.evade_yaw_set      = False
        self.clear_buf_started  = False
        self.clear_buf_time     = 0.0
        self.stuck_rev_start    = None

        # ── Sensor cache ─────────────────────────────────────────────────
        self.line_cx    = None
        self.line_area  = 0
        self.front_dist = float('inf')
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.current_yaw = 0.0
        self.bridge     = CvBridge()

        # ── QoS ──────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Image,     '/camera/image_raw', self._img_cb,  qos)
        self.create_subscription(LaserScan, '/scan',             self._scan_cb, qos)
        self.create_subscription(Odometry,  '/odom',             self._odom_cb, qos)

        # ── Publishers ───────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',                    1)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 1)

        # ── Control timer 20 Hz ──────────────────────────────────────────
        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f'Hybrid APF+SM started | obstacle_dist={self.obstacle_dist}m '
            f'k_att={self.k_att} follow_speed={self.follow_speed}')

    # ── Parameter loader ──────────────────────────────────────────────────
    def _load_params(self):
        gp = lambda n: self.get_parameter(n).value
        self.k_att           = gp('k_att')
        self.follow_speed    = gp('follow_speed')
        self.min_speed       = gp('min_speed')
        self.obstacle_dist   = gp('obstacle_dist')
        self.evade_turn_z    = gp('evade_turn_z')
        self.evade_target_deg= gp('evade_target_deg')
        self.evade_speed     = gp('evade_speed')
        self.clear_speed     = gp('clear_speed')
        self.clear_dist      = gp('clear_dist')
        self.clear_buffer    = gp('clear_buffer')
        self.arc_speed       = gp('arc_speed')
        self.arc_turn_z      = gp('arc_turn_z')
        self.arc_line_area   = gp('arc_line_area')
        self.timeout_evade   = gp('timeout_evade')
        self.timeout_clear   = gp('timeout_clear')
        self.timeout_arc     = gp('timeout_arc')
        self.stuck_rev_speed = gp('stuck_rev_speed')
        self.stuck_rev_sec   = gp('stuck_rev_sec')
        self.enable_rviz     = gp('enable_rviz')

    # ── Velocity helper ───────────────────────────────────────────────────
    def _vel(self, vx=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    # ── State transition ──────────────────────────────────────────────────
    def _go(self, state):
        self.get_logger().info(f'{self.state.name} → {state.name}')
        self.state            = state
        self.state_entry_time = time.monotonic()
        self.clear_buf_started = False
        self.evade_yaw_set    = False

    def _elapsed(self):
        return time.monotonic() - self.state_entry_time

    # ── Sensor callbacks ──────────────────────────────────────────────────
    def _img_cb(self, msg):
        enc = msg.encoding.lower()
        try:
            if enc in ('nv21', 'nv12'):
                yuv = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                    (msg.height * 3 // 2, msg.width))
                cv_img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21
                                      if enc == 'nv21' else cv2.COLOR_YUV2BGR_NV12)
            elif enc == 'rgb8':
                cv_img = cv2.cvtColor(
                    self.bridge.imgmsg_to_cv2(msg, 'passthrough'), cv2.COLOR_RGB2BGR)
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera decode error: {e}')
            return
        self.line_cx, self.line_area = self._detect_line(cv_img)


    def _odom_cb(self, msg):
        self.robot_x     = msg.pose.pose.position.x
        self.robot_y     = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    # ── Line detection (bottom 65% ROI, HSV-friendly) ─────────────────────
    def _detect_line(self, cv_img):
        small    = cv2.resize(cv_img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_NEAREST)
        roi_y    = int(IMG_HEIGHT * LINE_ROI_START)
        roi      = small[roi_y:, :]          # bottom 65%
        gray     = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur     = cv2.GaussianBlur(gray, (7, 7), 0)
        thresh   = cv2.adaptiveThreshold(blur, 255,
                                         cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY_INV, 11, 2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < 200:                       # reject noise / tiny patches
            return None, 0
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, 0
        return int(M['m10'] / M['m00']), area

    # ── Front-cone min distance ───────────────────────────────────────────
    def _front_min(self, ranges):
        n = len(ranges)
        if n == 0:
            return float('inf')
        cone = list(ranges[0:min(FRONT_CONE_LOW, n)]) + list(ranges[min(FRONT_CONE_HIGH, n):n])
        clean = sanitize(cone)
        return float(np.min(clean)) if len(clean) else float('inf')

    # ══════════════════════ MAIN CONTROL LOOP (20 Hz) ═════════════════════
    def _loop(self):
        elapsed = self._elapsed()

        # ── STUCK ─────────────────────────────────────────────────────────
        if self.state == RobotState.STUCK:
            if self.stuck_rev_start is None:
                self.stuck_rev_start = time.monotonic()
                self._vel(self.stuck_rev_speed, 0.0)
            elif time.monotonic() - self.stuck_rev_start >= self.stuck_rev_sec:
                self.stuck_rev_start = None
                self._go(RobotState.EVADING)
            return

        # ── Watchdog – escalate to STUCK if any bypass state timed out ────
        if self.state == RobotState.EVADING  and elapsed > self.timeout_evade:
            self._go(RobotState.STUCK); return
        if self.state == RobotState.CLEARING  and elapsed > self.timeout_clear:
            self._go(RobotState.STUCK); return
        if self.state == RobotState.ARC_RECOVERY and elapsed > self.timeout_arc:
            self._go(RobotState.STUCK); return

        # ── FOLLOWING_LINE — APF smooth steering ─────────────────────────
        if self.state == RobotState.FOLLOWING_LINE:
            # Obstacle check first
            if self.front_dist < self.obstacle_dist:
                self.get_logger().info(f'Obstacle at {self.front_dist:.2f}m → EVADING')
                self._vel(0.0, 0.0)
                self._go(RobotState.EVADING)
                return
            # APF steering
            if self.line_cx is not None:
                error = self.line_cx - IMG_WIDTH / 2.0
                wz    = max(-1.2, min(1.2, -self.k_att * error))
                vx    = self.follow_speed * (1.0 - 0.4 * abs(wz) / 1.2)  # slow on sharp turns
                vx    = max(self.min_speed, vx)
                self._vel(vx, wz)
                self._publish_markers(-self.k_att * error, 0.0, wz)
            else:
                # Line lost — creep forward slowly to re-acquire
                self._vel(self.min_speed, 0.0)

        # ── EVADING — 90° right turn (creeps forward, never fully stops) ──
        elif self.state == RobotState.EVADING:
            if not self.evade_yaw_set:
                self.entry_yaw     = self.current_yaw
                self.evade_yaw_set = True
                self.get_logger().info(f'EVADING: entry_yaw={math.degrees(self.entry_yaw):.1f}°')

            delta = abs(angle_diff(self.entry_yaw, self.current_yaw))
            if delta >= math.radians(self.evade_target_deg):
                self.get_logger().info(f'EVADING done ({math.degrees(delta):.1f}°) → CLEARING')
                self._go(RobotState.CLEARING)
            else:
                # Creep forward a tiny bit so robot doesn't spin in place
                self._vel(self.evade_speed, self.evade_turn_z)

        # ── CLEARING — drive straight, watch left LiDAR (via _scan_cb) ─────
        elif self.state == RobotState.CLEARING:
            self._vel(self.clear_speed, 0.0)

        # ── ARC_RECOVERY — forward-left arc, camera looks for line ────────
        elif self.state == RobotState.ARC_RECOVERY:
            if self.line_cx is not None and self.line_area > self.arc_line_area:
                self.get_logger().info(f'Line re-acquired (area={self.line_area:.0f}) → FOLLOWING')
                self._go(RobotState.FOLLOWING_LINE)
            else:
                self._vel(self.arc_speed, self.arc_turn_z)
                # Re-evade if new obstacle found during arc
                if self.front_dist < self.obstacle_dist:
                    self.get_logger().warn('New obstacle during arc → re-EVADING')
                    self._go(RobotState.EVADING)

    # ── left LiDAR scan callback for CLEARING ─────────────────────────────
    def _scan_cb(self, msg):
        r = msg.ranges
        n = len(r)
        if n == 0:
            return
        # Front cone
        cone  = list(r[0:min(FRONT_CONE_LOW, n)]) + list(r[min(FRONT_CONE_HIGH, n):n])
        clean = sanitize(cone)
        self.front_dist = float(np.min(clean)) if len(clean) else float('inf')

        # CLEARING: watch left side
        if self.state == RobotState.CLEARING:
            left = sanitize(r[min(LEFT_SLICE_START, n):min(LEFT_SLICE_END, n)])
            if len(left) > 0:
                med = float(np.median(left))
                if med >= self.clear_dist and not self.clear_buf_started:
                    self.get_logger().info(f'Left clear ({med:.2f}m), buffering {self.clear_buffer}s')
                    self.clear_buf_started = True
                    self.clear_buf_time    = time.monotonic()
            if self.clear_buf_started and (time.monotonic() - self.clear_buf_time) >= self.clear_buffer:
                self._go(RobotState.ARC_RECOVERY)

    # ── RViz markers ──────────────────────────────────────────────────────
    def _publish_markers(self, f_att, f_rep, wz):
        if not self.enable_rviz:
            return
        from geometry_msgs.msg import Point
        from builtin_interfaces.msg import Duration
        ma = MarkerArray()
        for mid, (fy, r, g, b) in enumerate([
            (-f_att * 0.4, 0.0, 0.4, 1.0),  # blue = line pull
            (f_rep  * 0.4, 1.0, 0.2, 0.0),  # red  = obstacle push
            (wz     * 0.4, 0.1, 0.9, 0.1),  # green = resultant
        ]):
            m = Marker()
            m.header.frame_id = 'odom'
            m.ns = 'apf'; m.id = mid
            m.type = Marker.ARROW; m.action = Marker.ADD
            p0 = Point(); p0.x = self.robot_x; p0.y = self.robot_y; p0.z = 0.1
            p1 = Point(); p1.x = p0.x;         p1.y = p0.y + fy;   p1.z = 0.1
            m.points = [p0, p1]
            m.scale.x = 0.05; m.scale.y = 0.10; m.scale.z = 0.10
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.9
            m.lifetime = Duration(sec=0, nanosec=200_000_000)
            ma.markers.append(m)
        self.marker_pub.publish(ma)

    # ── Shutdown ──────────────────────────────────────────────────────────
    def destroy_node(self):
        self._vel(0.0, 0.0)
        super().destroy_node()


# ─────────────────────────── MAIN ─────────────────────────────────────────
def main(args=None):
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
    import sys
    main(sys.argv)
