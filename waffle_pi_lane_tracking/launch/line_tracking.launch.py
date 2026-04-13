#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
antigravity.launch.py — Launch the antigravity_controller node.

Usage (on Raspberry Pi, after colcon build & sourcing):
  ros2 launch waffle_pi_lane_tracking antigravity.launch.py

Assumes turtlebot3_bringup robot.launch.py and camera_node are already running.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    line_tracking_node = Node(
        package='waffle_pi_lane_tracking',
        executable='line_tracking',
        name='line_tracking_controller',
        output='screen',
    )

    return LaunchDescription([
        line_tracking_node,
    ])
