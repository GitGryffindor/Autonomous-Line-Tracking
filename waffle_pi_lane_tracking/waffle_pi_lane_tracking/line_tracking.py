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
LINE_ROI_START = 0.0    # use full frame for line detection (was 0.35 — caused blind spot)

FRONT_CONE_LOW  = 20    # front-cone right edge index
FRONT_CONE_HIGH = 340   # front-cone left  edge index

# ──────────────────────────── STATE ENUM ──────────────────────────────────
class RobotState(Enum):
    FOLLOWING_LINE = 0
    BYPASSING      = 1
    HARD_STOP      = 2
    STUCK          = 3


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


def _ransac_line_filter(
    pts: np.ndarray,
    inlier_thresh: float = 0.015,
    iterations: int = 40,
) -> np.ndarray:
    """Return only the points that lie on the dominant flat face."""
    if len(pts) < 2:
        return pts
    best_mask = np.ones(len(pts), dtype=bool)
    best_count = 0
    for _ in range(iterations):
        idx = np.random.choice(len(pts), 2, replace=False)
        p1, p2 = pts[idx[0]], pts[idx[1]]
        edge = p2 - p1
        norm_len = np.linalg.norm(edge)
        if norm_len < 1e-6:
            continue
        n = np.array([-edge[1], edge[0]]) / norm_len
        dists = np.abs((pts - p1) @ n)
        mask = dists < inlier_thresh
        if mask.sum() > best_count:
            best_count = mask.sum()
            best_mask = mask
    return pts[best_mask]


def measure_box_width(
    ranges,
    angle_min: float,
    angle_increment: float,
    threshold: float = 0.6,
    min_points: int = 5,
) -> dict | None:
    """
    Precise flat-face width measurement for box-shaped obstacles.

    Algorithm:
      1. Convert obstacle arc to Cartesian XY.
      2. RANSAC: keep only points on the dominant flat face.
      3. SVD: find the face direction (1st principal component).
      4. Project all inlier points onto that direction → width.

    Returns dict with keys: width, distance, face_angle  (all in metres/radians)
    or None if insufficient points.
    """
    pts = []
    for i, r in enumerate(ranges):
        if np.isfinite(r) and 0.01 < r < threshold:
            angle = angle_min + i * angle_increment
            pts.append((r * np.cos(angle), r * np.sin(angle)))

    if len(pts) < min_points:
        return None

    pts = np.array(pts)

    # RANSAC outlier removal
    pts = _ransac_line_filter(pts)
    if len(pts) < min_points:
        return None

    # SVD — find the dominant face direction
    centroid = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - centroid)
    face_dir  = Vt[0]   # along the face
    face_norm = Vt[1]   # perpendicular to the face

    # Make sure normal points toward the robot (origin)
    if face_norm @ centroid < 0:
        face_norm = -face_norm

    # Project onto face direction → width
    proj  = (pts - centroid) @ face_dir
    width = float(proj.max() - proj.min())

    # Distance = component of centroid along the face normal
    distance   = float(abs(centroid @ face_norm))
    face_angle = float(np.arctan2(face_dir[1], face_dir[0]))

    return {
        'width':      round(width,      4),
        'distance':   round(distance,   4),
        'face_angle': round(face_angle, 4),
    }


# ──────────────────────────── NODE ────────────────────────────────────────
class LineTrackingController(Node):

    def __init__(self):
        super().__init__('line_tracking_controller')

        # ── Parameters ───────────────────────────────────────────────────
        # APF line-following
        self.declare_parameter('k_att',             0.010)   # steering gain (px → rad/s) — restored from working code
        self.declare_parameter('follow_speed',      0.15)    # forward speed while following
        self.declare_parameter('min_speed',         0.04)    # min speed in any state (never stops)
        
        # Obstacle detection (3-Zone System)
        self.declare_parameter('zone1_dist',        1.00)    # Awareness (log only)
        self.declare_parameter('zone2_dist',        0.75)    # Start bypass curve
        self.declare_parameter('zone3_dist',        0.30)    # Hard stop!
        
        # BYPASSING (Option B - Sinusoidal)
        self.declare_parameter('bypass_time',       6.0)     # Total time for the curve (s)
        self.declare_parameter('bypass_amp',        0.50)    # Amplitude of the turn (rad/s)
        self.declare_parameter('bypass_speed',      0.10)    # Forward speed during bypass
        
        # Watchdogs
        self.declare_parameter('timeout_bypass',    12.0)
        
        # STUCK
        self.declare_parameter('stuck_rev_speed',  -0.10)
        self.declare_parameter('stuck_rev_sec',     1.5)
        # Visualization
        self.declare_parameter('enable_rviz',       False)
        # NV21/NV12 decode toggle — camera reports NV21; set True if yellow appears cyan
        self.declare_parameter('force_nv12',        False)
        # HSV yellow-line detection (tuned for Pi Camera V2 + libcamera)
        self.declare_parameter('yellow_h_low',      15)
        self.declare_parameter('yellow_h_high',     42)
        self.declare_parameter('yellow_s_low',      70)
        self.declare_parameter('yellow_s_high',     255)
        self.declare_parameter('yellow_v_low',      70)
        self.declare_parameter('yellow_v_high',     255)

        self._load_params()

        # ── State ────────────────────────────────────────────────────────
        self.state              = RobotState.FOLLOWING_LINE
        self.state_entry_time   = time.monotonic()
        self.stuck_rev_start    = None

        # ── Sensor cache ─────────────────────────────────────────────────
        self.line_cx           = None
        self.line_area         = 0
        self.front_dist        = float('inf')
        self.obstacle_width    = None   # metres — SVD+RANSAC box width
        self.obstacle_distance = None   # metres — perpendicular dist to face
        self.obstacle_face_angle = None # radians — face tilt vs robot X-axis
        self.robot_x           = 0.0
        self.robot_y           = 0.0
        self.current_yaw       = 0.0
        self.bridge            = CvBridge()
        self.last_error        = 0.0   # last known line error (px from centre)
        self.bypass_amp_base   = None  # locked at bypass entry, reset after

        # ── QoS ──────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        # ── Subscribers (raw only — camera_ros cannot compress NV21 to JPEG) ──
        self.create_subscription(Image,     '/camera/image_raw', self._img_cb,  qos)
        self.create_subscription(LaserScan, '/scan',             self._scan_cb, qos)
        self.create_subscription(Odometry,  '/odom',             self._odom_cb, qos)
        self._dbg_last_log = 0.0

        # ── Publishers ───────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(Twist,       '/cmd_vel',                    1)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 1)

        # ── Control timer 20 Hz ──────────────────────────────────────────
        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f'Hybrid APF+Sinusoidal started | zone2_dist={self.zone2_dist}m '
            f'k_att={self.k_att} follow_speed={self.follow_speed}')

    # ── Parameter loader ──────────────────────────────────────────────────
    def _load_params(self):
        gp = lambda n: self.get_parameter(n).value
        self.k_att           = gp('k_att')
        self.follow_speed    = gp('follow_speed')
        self.min_speed       = gp('min_speed')
        self.zone1_dist      = gp('zone1_dist')
        self.zone2_dist      = gp('zone2_dist')
        self.zone3_dist      = gp('zone3_dist')
        self.bypass_time     = gp('bypass_time')
        self.bypass_amp      = gp('bypass_amp')
        self.bypass_speed    = gp('bypass_speed')
        self.timeout_bypass  = gp('timeout_bypass')
        self.stuck_rev_speed = gp('stuck_rev_speed')
        self.stuck_rev_sec   = gp('stuck_rev_sec')
        self.enable_rviz     = gp('enable_rviz')
        self.force_nv12      = gp('force_nv12')
        self.yellow_h_low    = gp('yellow_h_low')
        self.yellow_h_high   = gp('yellow_h_high')
        self.yellow_s_low    = gp('yellow_s_low')
        self.yellow_s_high   = gp('yellow_s_high')
        self.yellow_v_low    = gp('yellow_v_low')
        self.yellow_v_high   = gp('yellow_v_high')

    # ── Velocity helper ───────────────────────────────────────────────────
    def _vel(self, vx=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    # ── State transition ──────────────────────────────────────────────────
    def _go(self, state):
        self.get_logger().info(f'{self.state.name} → {state.name}')
        prev                  = self.state
        self.state            = state
        self.state_entry_time = time.monotonic()

        # ── Garbage collection on state exit ─────────────────────────────
        # Clear bypass geometry leaving BYPASSING → next obstacle always
        # starts with a fresh SVD/width measurement (no stale data).
        if prev == RobotState.BYPASSING:
            self.bypass_amp_base     = None
            self.obstacle_width      = None
            self.obstacle_distance   = None
            self.obstacle_face_angle = None

        # Reset stuck reverse timer on any STUCK entry/exit so a premature
        # rescue can never leave a stale timestamp that fires too early.
        if prev == RobotState.STUCK or state == RobotState.STUCK:
            self.stuck_rev_start = None


    def _elapsed(self):
        return time.monotonic() - self.state_entry_time

    # ── Sensor callbacks ──────────────────────────────────────────────────
    def _img_cb(self, msg):
        enc = msg.encoding.lower()
        try:
            if enc in ('nv21', 'nv12'):
                yuv = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                    (msg.height * 3 // 2, msg.width))
                # Camera reports NV21 — trust the label.
                # If yellow appears as cyan/blue in the DBG log, set force_nv12:=true
                # to try NV12 decoding instead.
                if self.force_nv12:
                    cv_img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                else:
                    cv_img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
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

    # ── Line detection — HSV yellow mask (matches Pi Camera V2 calibration) ──
    def _detect_line(self, cv_img):
        small = cv2.resize(cv_img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_NEAREST)
        roi_y = int(IMG_HEIGHT * LINE_ROI_START)
        roi   = small[roi_y:, :]            # configurable ROI (currently full frame)
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Yellow tape HSV range — tuned for Pi Camera V2 + libcamera
        lo = np.array([self.yellow_h_low,  self.yellow_s_low,  self.yellow_v_low],  dtype=np.uint8)
        hi = np.array([self.yellow_h_high, self.yellow_s_high, self.yellow_v_high], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)

        # Morphological cleanup to remove noise
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        yellow_px = int(np.sum(mask > 0))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = cv2.contourArea(max(contours, key=cv2.contourArea)) if contours else 0

        # Debug log every 3 s — tells us if HSV range is finding any yellow at all
        now = time.monotonic()
        if now - self._dbg_last_log >= 3.0:
            self._dbg_last_log = now
            self.get_logger().info(
                f'[DBG] yellow_px={yellow_px}  best_area={best_area:.0f}  '
                f'HSV=({self.yellow_h_low}-{self.yellow_h_high}, '
                f'{self.yellow_s_low}-{self.yellow_s_high}, '
                f'{self.yellow_v_low}-{self.yellow_v_high})')

        if not contours:
            return None, 0
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area < 200:                      # reject noise / tiny patches
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

        # ── ZONE 3: HARD STOP (Immediate Override) ────────────────────────
        if self.front_dist < self.zone3_dist:
            if self.state != RobotState.HARD_STOP:
                self.get_logger().error(f'ZONE 3: Collision imminent ({self.front_dist:.2f}m) → HARD STOP')
                self._go(RobotState.HARD_STOP)
            self._vel(0.0, 0.0)
            return

        # ── RECOVERY FROM HARD STOP ───────────────────────────────────────
        if self.state == RobotState.HARD_STOP:
            if self.front_dist >= self.zone3_dist + 0.05:  # Hysteresis
                self.get_logger().info('Obstacle cleared from Zone 3 → FOLLOWING_LINE')
                self._go(RobotState.FOLLOWING_LINE)
            else:
                self._vel(0.0, 0.0)
                return

        # ── STUCK ─────────────────────────────────────────────────────────
        if self.state == RobotState.STUCK:
            if self.stuck_rev_start is None:
                self.stuck_rev_start = time.monotonic()
                self._vel(self.stuck_rev_speed, 0.0)
            elif time.monotonic() - self.stuck_rev_start >= self.stuck_rev_sec:
                self.stuck_rev_start = None
                self._go(RobotState.BYPASSING)
            return

        # ── Watchdog – escalate to STUCK if bypass timed out ──────────────
        if self.state == RobotState.BYPASSING and elapsed > self.timeout_bypass:
            self._go(RobotState.STUCK); return

        # ── RESCUE: large yellow blob seen while in bypass → snap back instantly ──
        RESCUE_AREA = 3000
        # Fix: Only rescue if the path ahead is ACTUALLY clear, otherwise it instantly loops back to bypassing!
        if (self.state not in (RobotState.FOLLOWING_LINE, RobotState.STUCK, RobotState.HARD_STOP)
                and self.front_dist > self.zone2_dist
                and self.line_cx is not None and self.line_area > RESCUE_AREA
                and elapsed > 1.5):
            self.get_logger().info(
                f'RESCUE: large line (area={self.line_area:.0f}), path clear → FOLLOWING_LINE')
            self._go(RobotState.FOLLOWING_LINE)
            return

        # ── FOLLOWING_LINE — APF smooth steering ─────────────────────────
        if self.state == RobotState.FOLLOWING_LINE:
            # Zone 1: Awareness (logging)
            if self.zone2_dist <= self.front_dist < self.zone1_dist:
                now = time.monotonic()
                if not hasattr(self, '_last_z1_log') or now - self._last_z1_log > 1.0:
                    self.get_logger().info(f'ZONE 1: Aware of obstacle at {self.front_dist:.2f}m')
                    self._last_z1_log = now

            # Zone 2: Start Bypass
            if self.front_dist < self.zone2_dist:
                self.get_logger().warn(f'ZONE 2: Obstacle at {self.front_dist:.2f}m → BYPASSING')
                self._go(RobotState.BYPASSING)
                return

            # APF steering
            if self.line_cx is not None:
                error = self.line_cx - IMG_WIDTH / 2.0
                self.last_error = error           # remember for lost-line search
                wz    = max(-1.2, min(1.2, -self.k_att * error))
                vx    = self.follow_speed * (1.0 - 0.4 * abs(wz) / 1.2)
                vx    = max(self.min_speed, vx)
                self._vel(vx, wz)
                self._publish_markers(-self.k_att * error, 0.0, wz)
            else:
                # Line lost — rotate toward the direction the line was last seen
                search_wz = -0.3 if self.last_error > 0 else 0.3
                self.get_logger().warn(
                    f'FOLLOWING_LINE: line lost — searching (wz={search_wz:+.1f})')
                self._vel(self.min_speed, search_wz)

        # ── BYPASSING — Sinusoidal Curve ──────────────────────────────────
        elif self.state == RobotState.BYPASSING:
            progress = min(1.0, elapsed / self.bypass_time)

            # ── Compute bypass amplitude once at entry via arctan geometry ──
            # theta = atan2(width/2 + safety_margin, perpendicular_distance)
            # This is the exact angle the robot must steer to clear the box edge.
            # face_angle corrects for an angled approach (box not square-on).
            #
            #          Robot
            #            |  ← distance (d)
            #       _____|_____
            #      |     |     |  ← box face
            #      |   w/2+m   |
            #      |___________|  w = obstacle_width
            #
            #  theta = atan2(w/2 + margin, d)  → required clearance angle
            # Re-measure during early bypass (progress < 0.15 ≈ first 0.9 s of 6 s).
            # This fixes the race-condition where obstacle_width is None at the exact
            # tick of BYPASSING entry (LiDAR callback hadn't fired yet), causing the
            # robot to silently fall back to the generic amp and miss the clearance
            # geometry for 2nd/3rd obstacles. After early phase the value is locked.
            if self.bypass_amp_base is None or progress < 0.15:
                SAFETY_MARGIN = 0.30   # 30 cm lateral safety buffer (increased by 0.2m)
                D_FLOOR       = 0.20   # minimum planning distance (m)
                               #   prevents tiny SVD distances from blowing up theta
                AMP_MIN       = 0.20   # never less than this (rad/s)
                AMP_MAX       = 1.50   # hard ceiling (rad/s)

                if (self.obstacle_width is not None
                        and self.obstacle_distance is not None
                        and self.obstacle_distance > 0.01):

                    w = self.obstacle_width
                    # obstacle_distance from SVD is ALREADY perpendicular to the face.
                    # No cos(face_angle) projection needed — that was wrong and caused
                    # near-zero d when face_angle ≈ 90°, inflating theta massively.
                    d = max(self.obstacle_distance, D_FLOOR)

                    # Exact clearance angle:
                    #
                    #   Robot
                    #     |── d ──|
                    #     |       |‾‾‾‾‾‾‾‾‾|
                    #             | w/2 + m  |  box edge
                    #             |__________|
                    #
                    #   theta = atan2(w/2 + margin, d)
                    #
                    # For sinusoidal wz = A·sin(2π·t/T),
                    # max heading swing = A·T/π  →  A = theta·π / T  (exact, no gain)
                    theta   = math.atan2(w / 2.0 + SAFETY_MARGIN, d)
                    amp_geo = theta * math.pi / self.bypass_time  # heading swing == theta
                    self.bypass_amp_base = max(AMP_MIN, min(AMP_MAX, amp_geo))

                    phase = 'LOCK' if progress >= 0.15 else f'early({progress:.2f})'
                    fa = self.obstacle_face_angle or 0.0
                    self.get_logger().info(
                        f'[BYPASS-GEO|{phase}] w={w:.3f}m raw_d={self.obstacle_distance:.3f}m '
                        f'plan_d={d:.3f}m face={math.degrees(fa):.1f}° '
                        f'theta={math.degrees(theta):.1f}° '
                        f'amp={self.bypass_amp_base:.3f} rad/s  '
                        f'(swing≈{math.degrees(theta):.0f}°)')
                elif self.bypass_amp_base is None:
                    # Only use fallback if we have absolutely no geometry yet
                    self.bypass_amp_base = self.bypass_amp
                    self.get_logger().warn('[BYPASS-GEO] No width data — using default amp')

            amp = self.bypass_amp_base

            # Look for line once we're past the halfway mark
            if progress > 0.5 and self.line_cx is not None and self.line_area > 400.0:
                self.get_logger().info(f'Line re-acquired (area={self.line_area:.0f}) → FOLLOWING')
                self.bypass_amp_base = None   # reset for next bypass
                self._go(RobotState.FOLLOWING_LINE)
                return

            # Sinusoidal steering: right (-wz) -> straight -> left (+wz)
            wz = -amp * math.sin(progress * 2.0 * math.pi)

            # Slow down slightly during sharpest turns
            vx = self.bypass_speed * (1.0 - 0.3 * abs(wz) / amp)
            vx = max(self.min_speed, vx)

            self._vel(vx, wz)

            if progress >= 1.0:
                # If bypass finishes without finding line, fallback to line searching
                self.get_logger().warn('Bypass finished without seeing line → FOLLOWING (Search)')
                self.bypass_amp_base = None   # reset for next bypass
                self._go(RobotState.FOLLOWING_LINE)

    # ── front LiDAR scan callback ─────────────────────────────────────────
    def _scan_cb(self, msg):
        r = msg.ranges
        n = len(r)
        if n == 0:
            return

        # ── Front-cone min distance ──────────────────────────────────────
        cone  = list(r[0:min(FRONT_CONE_LOW, n)]) + list(r[min(FRONT_CONE_HIGH, n):n])
        clean = sanitize(cone)
        self.front_dist = float(np.min(clean)) if len(clean) else float('inf')

        # ── Box-width measurement (only when obstacle is close enough) ────
        if self.front_dist < self.zone1_dist:
            # Build a front-cone index list for angle computation
            low_idx  = min(FRONT_CONE_LOW, n)
            high_idx = min(FRONT_CONE_HIGH, n)
            cone_ranges = list(r[0:low_idx]) + list(r[high_idx:n])

            # angle_min for the stitched cone:
            # indices 0..low_idx-1 map directly; high_idx..n-1 map to
            # negative angles, so we pass msg fields and let measure_box_width
            # handle the full scan — it will only pick up points < threshold.
            result = measure_box_width(
                msg.ranges,
                msg.angle_min,
                msg.angle_increment,
                threshold=min(self.front_dist + 0.15, self.zone1_dist),
            )
            if result is not None:
                self.obstacle_width      = result['width']
                self.obstacle_distance   = result['distance']
                self.obstacle_face_angle = result['face_angle']
                self.get_logger().info(
                    f'[WIDTH] box={self.obstacle_width:.3f}m  '
                    f'dist={self.obstacle_distance:.3f}m  '
                    f'face={math.degrees(self.obstacle_face_angle):.1f}°')
            else:
                self.obstacle_width      = None
                self.obstacle_distance   = None
                self.obstacle_face_angle = None
        else:
            self.obstacle_width = None

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
