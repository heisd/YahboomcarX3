#!/usr/bin/env python3
"""Launch the IMU-based collision detector.

All thresholds are exposed as launch arguments so you can override them on
the command line, e.g.:

    ros2 launch yahboomcar_collision collision_detector_launch.py \
        accel_threshold:=15.0 cooldown_sec:=2.0 use_gyro:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Defaults intentionally conservative (high) to minimise false positives.
    args = [
        DeclareLaunchArgument(
            'imu_topic', default_value='imu/data_raw',
            description='sensor_msgs/Imu topic to listen on'),
        DeclareLaunchArgument(
            'accel_threshold', default_value='12.0',
            description='Linear-acceleration shock threshold (m/s^2). '
                        'Lower = more sensitive.'),
        DeclareLaunchArgument(
            'gyro_threshold', default_value='6.0',
            description='Angular-velocity shock threshold (rad/s). '
                        'Only used when use_gyro:=true.'),
        DeclareLaunchArgument(
            'use_gyro', default_value='false',
            description='Also require a gyro spike for a trigger (stricter).'),
        DeclareLaunchArgument(
            'baseline_alpha', default_value='0.02',
            description='EMA weight for the running baseline (0..1).'),
        DeclareLaunchArgument(
            'min_trigger_samples', default_value='2',
            description='Consecutive over-threshold IMU samples required.'),
        DeclareLaunchArgument(
            'cooldown_sec', default_value='1.5',
            description='Seconds to ignore further triggers after one fires.'),
        DeclareLaunchArgument(
            'stop_on_collision', default_value='true',
            description='Publish zero Twist on cmd_vel_topic when triggered.'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Where to publish the emergency-stop Twist.'),
    ]

    node = Node(
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
            'stop_on_collision':   LaunchConfiguration('stop_on_collision'),
            'cmd_vel_topic':       LaunchConfiguration('cmd_vel_topic'),
        }],
    )

    return LaunchDescription(args + [node])
