#!/usr/bin/env python3
"""Launch the line-follow DETECT (HSV-learning) node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_index = LaunchConfiguration('camera_index')
    hsv_file = LaunchConfiguration('hsv_file')

    return LaunchDescription([
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument(
            'hsv_file',
            default_value='/tmp/yahboomcar_linefollow_hsv.txt',
            description='Where to save / read the HSV range.'),
        Node(
            package='yahboomcar_linefollow',
            executable='line_detect',
            name='line_detect',
            output='screen',
            parameters=[{
                'camera_index': camera_index,
                'hsv_file': hsv_file,
            }],
        ),
    ])
