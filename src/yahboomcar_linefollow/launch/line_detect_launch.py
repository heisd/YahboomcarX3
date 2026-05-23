#!/usr/bin/env python3
"""Launch the line-follow DETECT (HSV-learning) node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_hsv = os.path.join(
        get_package_share_directory('yahboomcar_linefollow'),
        'params', 'HSV.txt')

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument(
            'hsv_file',
            default_value=default_hsv,
            description='Persistent HSV file. With colcon --symlink-install '
                        'this writes back into the source tree so it can be '
                        'committed to git and reused by line_track.'),
        DeclareLaunchArgument(
            'autosave', default_value='true',
            description='If true, every learned ROI is written to hsv_file '
                        'immediately (no need to press s).'),
        Node(
            package='yahboomcar_linefollow',
            executable='line_detect',
            name='line_detect',
            output='screen',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'hsv_file':     LaunchConfiguration('hsv_file'),
                'autosave':     LaunchConfiguration('autosave'),
            }],
        ),
    ])
