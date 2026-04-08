#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# lane_tracking.launch.py
#
# Launches both nodes for TurtleBot3 Waffle Pi lane tracking.
#
# Usage (on the Raspberry Pi, after colcon build & sourcing):
#   ros2 launch waffle_pi_lane_tracking lane_tracking.launch.py
#
# Optional overrides:
#   ros2 launch waffle_pi_lane_tracking lane_tracking.launch.py camera_width:=320 camera_height:=240
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    camera_width_arg = DeclareLaunchArgument(
        'camera_width', default_value='640',
        description='Native width of the Raspberry Pi Camera V2 stream')

    camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='480',
        description='Native height of the Raspberry Pi Camera V2 stream')

    detect_lane_node = Node(
        package='waffle_pi_lane_tracking',
        executable='detect_lane',
        name='detect_lane',
        output='screen',
        parameters=[{
            'camera_width':  LaunchConfiguration('camera_width'),
            'camera_height': LaunchConfiguration('camera_height'),
        }]
    )

    control_robot_node = Node(
        package='waffle_pi_lane_tracking',
        executable='control_robot',
        name='control_robot',
        output='screen',
    )

    return LaunchDescription([
        camera_width_arg,
        camera_height_arg,
        detect_lane_node,
        control_robot_node,
    ])
