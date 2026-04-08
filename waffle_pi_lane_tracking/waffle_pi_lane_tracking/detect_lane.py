#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# detect_lane.py - Lane detection ROS2 node (ported from ROS1 detect_lane.py)
#
# Key changes from ROS1:
#   - rospy -> rclpy, inherits rclpy.node.Node
#   - CompressedImage subscription uses SensorDataQoS for reliable Raspberry Pi Camera streaming
#   - rospy.get_param() -> self.declare_parameter() / self.get_parameter()
#   - np.int() -> int()  (deprecated in NumPy >=1.20)
#   - Camera topic changed to /image_raw/compressed (raspicam2 / v4l2_camera default on ROS2)
#

import sys
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

from waffle_pi_lane_tracking.curve_fit import CurveFit
from waffle_pi_lane_tracking.birds_eye_view import BirdsEyeView
from waffle_pi_lane_tracking.lane_filter import LaneFilter


class DetectLane(Node):
    def __init__(self):
        super().__init__('detect_lane')

        # --- Camera parameters (declare with defaults matching the original) ---
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)

        cam_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        cam_height = self.get_parameter('camera_height').get_parameter_value().integer_value

        # Original code resized to 0.5x; pre-compute the working resolution
        self.w = int(cam_width * 0.5)
        self.h = int(cam_height * 0.5)

        self.get_logger().info(f'Working resolution: {self.w}x{self.h}')

        # --- cv_bridge ---
        self.bridge = CvBridge()

        # --- Bird's Eye View source/destination points ---
        # Calibrated for Pi Camera V2 at ~10 cm height, ~45° downward tilt.
        # The usable ground trapezoid sits in the lower ~45% of the image at this height.
        # TOP corners are narrow (0.30w - 0.70w) because the ground region converges
        # toward the centre of the image when the camera is close to the floor.
        # ADJUST these values if the projected lane looks skewed — see README for calibration steps.
        self.src = np.float32([
            (self.w * 0.30, self.h * 0.55),   # top-left
            (self.w * 0.70, self.h * 0.55),   # top-right
            (self.w * 0.95, self.h),           # bottom-right
            (self.w * 0.05, self.h),           # bottom-left
        ])
        self.dst = np.float32([
            (0,      0),      (self.w, 0),
            (self.w, self.h), (0,      self.h)
        ])

        self.birds_eye_view = BirdsEyeView(self.src, self.dst)
        self.lane_filter = LaneFilter()
        self.curve_fit = CurveFit(number_of_windows=10, margin=300, minimum_pixels=100)
        self.final_fit = None

        # --- QoS: use a sensor-data profile to match the camera node ---
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Subscribers / Publishers ---
        # ROS2 Waffle Pi default compressed image topic from raspicam2 / v4l2_camera
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.callback,
            sensor_qos)

        self.lane_error_pub = self.create_publisher(Float32, '/detect_lane/lane_error', 1)
        self.final_vis_img_pub = self.create_publisher(Image, '/detect_lane/final_vis_img', 1)

        self.lane_error_msg = Float32()
        self.lane_error_msg.data = float('nan')

    def callback(self, data):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridgeError: {e}')
            return

        cv_img = cv2.resize(cv_img, (0, 0), fx=0.5, fy=0.5)
        warp_img = self.birds_eye_view.sky_view(cv_img)
        bin_warp_img = self.lane_filter.processed_img(warp_img)

        try:
            lane_fit = self.curve_fit.method_two(bin_warp_img, self.final_fit)
        except Exception:
            lane_fit = self.curve_fit.sliding_windows(bin_warp_img)
        self.final_fit = lane_fit

        if self.final_fit is not None:
            ploty = np.linspace(0, bin_warp_img.shape[0] - 1, bin_warp_img.shape[0])
            lane_fitx = lane_fit[0] * (ploty ** 2) + lane_fit[1] * ploty + lane_fit[2]
            pts = np.array(np.transpose(np.vstack([lane_fitx, ploty])))
            lane_error = int(bin_warp_img.shape[1] / 2 - pts[0][0])
            self.lane_error_msg.data = float(lane_error)
        else:
            self.lane_error_msg.data = float('nan')

        self.lane_error_pub.publish(self.lane_error_msg)

    def destroy_node(self):
        # Publish NaN to signal shutdown to control_robot
        self.lane_error_msg.data = float('nan')
        self.lane_error_pub.publish(self.lane_error_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
