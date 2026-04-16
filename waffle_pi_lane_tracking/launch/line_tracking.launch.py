#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
line_tracking.launch.py — Hybrid APF + State Machine controller.

Usage:
  ros2 launch waffle_pi_lane_tracking line_tracking.launch.py
  ros2 launch waffle_pi_lane_tracking line_tracking.launch.py rviz:=true enable_rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('waffle_pi_lane_tracking')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'vision.rviz')

    args = [
        # APF / line-following
        DeclareLaunchArgument('k_att',             default_value='0.004',  description='Steering gain (px→rad/s)'),
        DeclareLaunchArgument('follow_speed',       default_value='0.15',   description='Forward speed while following (m/s)'),
        DeclareLaunchArgument('min_speed',          default_value='0.04',   description='Min speed in any state (never stops)'),
        # Obstacle
        DeclareLaunchArgument('obstacle_dist',      default_value='0.45',   description='Distance to trigger EVADING (m)'),
        # EVADING
        DeclareLaunchArgument('evade_turn_z',       default_value='-0.50',  description='Turn rate during EVADING (rad/s)'),
        DeclareLaunchArgument('evade_target_deg',   default_value='88.0',   description='Yaw to turn before CLEARING (°)'),
        DeclareLaunchArgument('evade_speed',        default_value='0.05',   description='Creep speed during EVADING (m/s)'),
        # CLEARING
        DeclareLaunchArgument('clear_speed',        default_value='0.12',   description='Forward speed during CLEARING (m/s)'),
        DeclareLaunchArgument('clear_dist',         default_value='0.65',   description='Left gap to declare obstacle passed (m)'),
        DeclareLaunchArgument('clear_buffer',       default_value='0.20',   description='Buffer seconds after clearing (s)'),
        # ARC_RECOVERY
        DeclareLaunchArgument('arc_speed',          default_value='0.10',   description='Forward speed during arc (m/s)'),
        DeclareLaunchArgument('arc_turn_z',         default_value='0.45',   description='Left-turn during arc (rad/s)'),
        DeclareLaunchArgument('arc_line_area',      default_value='400.0',  description='Min contour area to re-detect line (px²)'),
        # Watchdogs
        DeclareLaunchArgument('timeout_evade',      default_value='6.0',    description='EVADING timeout (s)'),
        DeclareLaunchArgument('timeout_clear',      default_value='25.0',   description='CLEARING timeout (s)'),
        DeclareLaunchArgument('timeout_arc',        default_value='18.0',   description='ARC_RECOVERY timeout (s)'),
        # STUCK
        DeclareLaunchArgument('stuck_rev_speed',    default_value='-0.10',  description='Reverse speed (m/s)'),
        DeclareLaunchArgument('stuck_rev_sec',      default_value='1.5',    description='Reverse duration (s)'),
        # Visualization
        DeclareLaunchArgument('enable_rviz',        default_value='false',  description='Publish force markers'),
        DeclareLaunchArgument('rviz',               default_value='false',  description='Launch RViz2'),
    ]

    node = Node(
        package='waffle_pi_lane_tracking',
        executable='line_tracking',
        name='line_tracking_controller',
        output='screen',
        parameters=[{
            'k_att':           LaunchConfiguration('k_att'),
            'follow_speed':    LaunchConfiguration('follow_speed'),
            'min_speed':       LaunchConfiguration('min_speed'),
            'obstacle_dist':   LaunchConfiguration('obstacle_dist'),
            'evade_turn_z':    LaunchConfiguration('evade_turn_z'),
            'evade_target_deg':LaunchConfiguration('evade_target_deg'),
            'evade_speed':     LaunchConfiguration('evade_speed'),
            'clear_speed':     LaunchConfiguration('clear_speed'),
            'clear_dist':      LaunchConfiguration('clear_dist'),
            'clear_buffer':    LaunchConfiguration('clear_buffer'),
            'arc_speed':       LaunchConfiguration('arc_speed'),
            'arc_turn_z':      LaunchConfiguration('arc_turn_z'),
            'arc_line_area':   LaunchConfiguration('arc_line_area'),
            'timeout_evade':   LaunchConfiguration('timeout_evade'),
            'timeout_clear':   LaunchConfiguration('timeout_clear'),
            'timeout_arc':     LaunchConfiguration('timeout_arc'),
            'stuck_rev_speed': LaunchConfiguration('stuck_rev_speed'),
            'stuck_rev_sec':   LaunchConfiguration('stuck_rev_sec'),
            'enable_rviz':     LaunchConfiguration('enable_rviz'),
        }],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [node, rviz_node])
