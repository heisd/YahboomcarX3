#!/usr/bin/env python3
"""Launch the line-follow TRACK node (publishes /cmd_vel)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('yahboomcar_linefollow')
    default_params = os.path.join(share, 'params', 'line_track.yaml')
    default_hsv    = os.path.join(share, 'params', 'HSV.txt')

    params_file = LaunchConfiguration('params_file')
    hsv_file = LaunchConfiguration('hsv_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with line_track parameters.'),
        DeclareLaunchArgument(
            'hsv_file',
            default_value=default_hsv,
            description='HSV calibration file. Defaults to the persistent '
                        'HSV.txt shipped in this package, updated by '
                        'line_detect.'),
        Node(
            package='yahboomcar_linefollow',
            executable='line_track',
            name='line_track',
            output='screen',
            parameters=[params_file, {'hsv_file': hsv_file}],
        ),
    ])
