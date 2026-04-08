#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# control_robot.py - PD controller ROS2 node (ported from ROS1 control_robot.py)
#
# Key changes from ROS1:
#   - rospy -> rclpy, inherits rclpy.node.Node
#   - rospy.get_rostime() -> self.get_clock().now()
#   - Shutdown hook -> destroy_node() override
#
# Controller gains are UNCHANGED from the original:
#   Kp = 0.0122,  Ki = 0.0,  Kd = 0.0013
#

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class ControlRobot(Node):
    def __init__(self):
        super().__init__('control_robot')

        # --- Identical gains from original control_robot.py ---
        self.Kp = 0.0122
        self.Ki = 0.0
        self.Kd = 0.0013

        self._input = float('nan')
        self.last_error = 0.0
        self.p_error, self.i_error, self.d_error = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        self.cmd_vel_msg = Twist()

        # --- Subscribe to lane error, publish velocity ---
        self.lane_error_sub = self.create_subscription(
            Float32,
            '/detect_lane/lane_error',
            self.update_error,
            1)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

    def update_error(self, data):
        self._input = data.data
        self.current_error = self._input if not math.isnan(self._input) else None

        now = self.get_clock().now().nanoseconds * 1e-9
        delta = now - self.last_time if (now - self.last_time) > 0 else 0.1
        self.last_time = now

        if self.current_error is not None:
            # --- Identical PD computation from original ---
            self.p_error = self.Kp * self.current_error
            self.i_error = self.Ki * (self.current_error + self.last_error) * delta
            self.d_error = self.Kd * (self.current_error - self.last_error) / delta
            self.last_error = self.current_error

            angular_z = self.p_error + self.d_error + self.i_error

            # Identical velocity expression from original
            self.cmd_vel_msg.linear.x = min(0.2 * ((1 - abs(self.current_error) / 103 ** 2.2)), 0.2)
            self.cmd_vel_msg.angular.z = max(angular_z, -2.0) if angular_z < 0 else min(angular_z, 2.0)

            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def destroy_node(self):
        # Stop the robot cleanly on shutdown
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
