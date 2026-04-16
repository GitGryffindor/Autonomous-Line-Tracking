#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
full_robot.launch.py — One-command launch for the complete Waffle Pi system.

Starts on the Pi (fully self-contained, no laptop required):
  1. TurtleBot3 bringup   (robot drivers, /odom, /scan)
  2. Camera node          (publishes /camera/image_raw)
  3. Line tracking brain  (APF + state machine controller)

Usage:
  ros2 launch waffle_pi_lane_tracking full_robot.launch.py

With tuning overrides:
  ros2 launch waffle_pi_lane_tracking full_robot.launch.py k_att:=0.005 obstacle_dist:=0.5
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Locate turtlebot3 bringup package ─────────────────────────────────
    tb3_bringup_dir = get_package_share_directory('turtlebot3_bringup')

    # ── Launch arguments (tuning) ──────────────────────────────────────────
    args = [
        DeclareLaunchArgument('k_att',           default_value='0.004'),
        DeclareLaunchArgument('follow_speed',     default_value='0.15'),
        DeclareLaunchArgument('min_speed',        default_value='0.04'),
        DeclareLaunchArgument('obstacle_dist',    default_value='0.45'),
        DeclareLaunchArgument('evade_turn_z',     default_value='-0.50'),
        DeclareLaunchArgument('evade_target_deg', default_value='88.0'),
        DeclareLaunchArgument('evade_speed',      default_value='0.05'),
        DeclareLaunchArgument('clear_speed',      default_value='0.12'),
        DeclareLaunchArgument('clear_dist',       default_value='0.65'),
        DeclareLaunchArgument('clear_buffer',     default_value='0.20'),
        DeclareLaunchArgument('arc_speed',        default_value='0.10'),
        DeclareLaunchArgument('arc_turn_z',       default_value='0.45'),
        DeclareLaunchArgument('arc_line_area',    default_value='400.0'),
        DeclareLaunchArgument('timeout_evade',    default_value='6.0'),
        DeclareLaunchArgument('timeout_clear',    default_value='25.0'),
        DeclareLaunchArgument('timeout_arc',      default_value='18.0'),
        DeclareLaunchArgument('stuck_rev_speed',  default_value='-0.10'),
        DeclareLaunchArgument('stuck_rev_sec',    default_value='1.5'),
    ]

    # ── 1. TurtleBot3 Bringup ──────────────────────────────────────────────
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_bringup_dir, 'launch', 'robot.launch.py')
        )
    )

    # ── 2. Camera Node (start 3 seconds after bringup to allow serial init) ─
    camera_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='camera_ros',
                executable='camera_node',
                name='camera_node',
                output='screen',
                parameters=[{
                    'width':  640,
                    'height': 480,
                }],
            )
        ]
    )

    # ── 3. Line Tracking Brain (start 5 seconds after bringup) ───────────
    line_tracking = TimerAction(
        period=5.0,
        actions=[
            Node(
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
                    'enable_rviz':     False,
                }],
            )
        ]
    )

    return LaunchDescription(args + [robot_bringup, camera_node, line_tracking])
