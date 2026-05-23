#!/usr/bin/env python3
"""Glue launch: line_track + collision_detector wired together.

Architecture:
    collision_detector  --(Bool)-->  /collision_detector/collision
                                            |
                                            v
    line_track  --(Twist)-->  /cmd_vel    pauses for collision_pause_sec
                                          on each True pulse

We intentionally set stop_on_collision:=false on the detector so that the
two nodes do NOT both publish on /cmd_vel. Pausing is owned by line_track,
which is the publisher running at the high rate; the detector only emits
the Bool flag.

All knobs (PID, speed, ROI, IMU thresholds, hold time, ...) are exposed as
launch arguments and can be overridden on the command line, e.g.:

    ros2 launch yahboomcar_linefollow linefollow_safe_launch.py \
        linear:=0.2 accel_threshold:=10.0 collision_pause_sec:=3.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COLLISION_FLAG_TOPIC = '/collision_detector/collision'


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'line_track.yaml')

    args = [
        # ---- shared ----
        DeclareLaunchArgument('hsv_file',
            default_value='/tmp/yahboomcar_linefollow_hsv.txt'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        # ---- line_track overrides ----
        DeclareLaunchArgument('linear', default_value='0.15'),
        DeclareLaunchArgument('collision_pause_sec', default_value='2.0',
            description='How long line_track pauses after each /collision.'),

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
            # Don't let the detector also write /cmd_vel; line_track owns
            # the stop via its collision hold-off.
            'stop_on_collision':   False,
        }],
    )

    return LaunchDescription(args + [collision, line_track])
