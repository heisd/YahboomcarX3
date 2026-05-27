#!/usr/bin/env python3
"""Glue launch: QR-priority line_track + collision_detector.

Same wiring as linefollow_safe_launch.py (collision_detector emits a Bool,
line_track owns /cmd_vel and pauses on it), but with the QR priority feature
exposed up front:

    line_track scans each frame for a QR code; when one resolves to a
    fork-road action in qr_actions.json it interrupts line following and
    executes Left / Right / Straight / Station / Stop.

    ros2 launch yahboomcar_linefollow qr_linefollow_launch.py \
        linear:=0.15 qr_check_every:=3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COLLISION_FLAG_TOPIC = '/collision_detector/collision'


def generate_launch_description():
    share = get_package_share_directory('yahboomcar_linefollow')
    default_params = os.path.join(share, 'params', 'line_track.yaml')
    default_hsv    = os.path.join(share, 'params', 'HSV.txt')
    default_qr     = os.path.join(share, 'params', 'qr_actions.json')

    args = [
        # ---- shared ----
        DeclareLaunchArgument('hsv_file', default_value=default_hsv),
        DeclareLaunchArgument('params_file', default_value=default_params),

        # ---- line_track / QR overrides ----
        DeclareLaunchArgument('linear', default_value='0.15'),
        DeclareLaunchArgument('collision_pause_sec', default_value='2.0'),
        DeclareLaunchArgument('enable_qr', default_value='true'),
        DeclareLaunchArgument('qr_actions_file', default_value=default_qr),
        DeclareLaunchArgument('qr_check_every', default_value='3',
            description='Run QR detection once every N camera frames.'),
        DeclareLaunchArgument('qr_cooldown_sec', default_value='4.0',
            description='Ignore the same QR payload again for this long.'),

        # ---- collision_detector knobs ----
        DeclareLaunchArgument('imu_topic', default_value='imu/data_raw'),
        DeclareLaunchArgument('accel_threshold', default_value='12.0'),
        DeclareLaunchArgument('gyro_threshold', default_value='6.0'),
        DeclareLaunchArgument('use_gyro', default_value='false'),
        DeclareLaunchArgument('baseline_alpha', default_value='0.02'),
        DeclareLaunchArgument('min_trigger_samples', default_value='2'),
        DeclareLaunchArgument('cooldown_sec', default_value='1.5'),
    ]

    line_track = Node(
        package='yahboomcar_linefollow',
        executable='line_track',
        name='line_track',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'hsv_file': LaunchConfiguration('hsv_file'),
                'linear':   LaunchConfiguration('linear'),
                'collision_topic': COLLISION_FLAG_TOPIC,
                'collision_pause_sec':
                    LaunchConfiguration('collision_pause_sec'),
                'enable_qr':       LaunchConfiguration('enable_qr'),
                'qr_actions_file': LaunchConfiguration('qr_actions_file'),
                'qr_check_every':  LaunchConfiguration('qr_check_every'),
                'qr_cooldown_sec': LaunchConfiguration('qr_cooldown_sec'),
            },
        ],
    )

    collision = Node(
        package='yahboomcar_collision',
        executable='collision_detector',
        name='collision_detector',
        output='screen',
        parameters=[{
            'imu_topic':           LaunchConfiguration('imu_topic'),
            'accel_threshold':     LaunchConfiguration('accel_threshold'),
            'gyro_threshold':      LaunchConfiguration('gyro_threshold'),
            'use_gyro':            LaunchConfiguration('use_gyro'),
            'baseline_alpha':      LaunchConfiguration('baseline_alpha'),
            'min_trigger_samples': LaunchConfiguration('min_trigger_samples'),
            'cooldown_sec':        LaunchConfiguration('cooldown_sec'),
            'stop_on_collision':   False,
        }],
    )

    return LaunchDescription(args + [collision, line_track])
