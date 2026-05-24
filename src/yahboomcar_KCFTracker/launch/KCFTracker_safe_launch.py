#!/usr/bin/env python3
"""Glue launch: KCF tracker + collision_detector wired together.

Architecture (mirrors yahboomcar_linefollow/linefollow_safe_launch.py):

    collision_detector  --(Bool)-->  /collision_detector/collision
                                            |
                                            v
    KCF_Tracker_Node  --(Twist)-->  /cmd_vel    pauses follow PID for
                                                collision_pause_sec on each
                                                True pulse.

`stop_on_collision:=false` on the detector so only the tracker publishes on
/cmd_vel -- pausing is owned by the high-rate publisher.

Override knobs on the command line, e.g.:

    ros2 launch yahboomcar_KCFTracker KCFTracker_safe_launch.py \
        minDist:=0.8 accel_threshold:=10.0 collision_pause_sec:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COLLISION_FLAG_TOPIC = '/collision_detector/collision'


def generate_launch_description():
    args = [
        # ---- KCF tracker knobs ----
        DeclareLaunchArgument('minDist', default_value='1.0',
            description='Follow distance in meters (PID setpoint).'),
        DeclareLaunchArgument('lost_threshold', default_value='0.15',
            description='KCF peak below this is treated as low-confidence.'),
        DeclareLaunchArgument('lost_patience', default_value='8',
            description='Consecutive low-confidence frames before LOST.'),
        DeclareLaunchArgument('recover_threshold', default_value='0.30',
            description='Mean back-projection response needed to recover.'),
        DeclareLaunchArgument('enable_redetect', default_value='true'),
        DeclareLaunchArgument('collision_pause_sec', default_value='2.0',
            description='How long the tracker freezes /cmd_vel after each '
                        'collision pulse.'),

        # ---- collision_detector knobs (same set as linefollow_safe) ----
        DeclareLaunchArgument('imu_topic', default_value='imu/data_raw'),
        DeclareLaunchArgument('accel_threshold', default_value='12.0'),
        DeclareLaunchArgument('gyro_threshold', default_value='6.0'),
        DeclareLaunchArgument('use_gyro', default_value='false'),
        DeclareLaunchArgument('baseline_alpha', default_value='0.02'),
        DeclareLaunchArgument('min_trigger_samples', default_value='2'),
        DeclareLaunchArgument('cooldown_sec', default_value='1.5'),
    ]

    kcf = Node(
        package='yahboomcar_KCFTracker',
        executable='KCF_Tracker_Node',
        name='image_converter',
        output='screen',
        parameters=[{
            'minDist_':            LaunchConfiguration('minDist'),
            'lost_threshold':      LaunchConfiguration('lost_threshold'),
            'lost_patience':       LaunchConfiguration('lost_patience'),
            'recover_threshold':   LaunchConfiguration('recover_threshold'),
            'enable_redetect':     LaunchConfiguration('enable_redetect'),
            'collision_topic':     COLLISION_FLAG_TOPIC,
            'collision_pause_sec': LaunchConfiguration('collision_pause_sec'),
        }],
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
            # KCF tracker owns /cmd_vel; detector only emits the Bool flag.
            'stop_on_collision':   False,
        }],
    )

    return LaunchDescription(args + [collision, kcf])
