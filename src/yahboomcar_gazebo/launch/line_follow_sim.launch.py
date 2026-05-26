"""Line following in Gazebo.

Brings up the line-track world (a yellow line on the floor) with the X3,
tilts the camera down, and runs yahboomcar_linefollow's line_track node fed
from the simulated camera topic instead of a USB camera.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('yahboomcar_gazebo')
    world = os.path.join(pkg, 'worlds', 'yahboom_line.world')
    hsv_file = os.path.join(pkg, 'config', 'line_hsv_sim.txt')

    gui = LaunchConfiguration('gui')
    image_topic = LaunchConfiguration('image_topic')
    show_window = LaunchConfiguration('show_window')
    linear = LaunchConfiguration('linear')

    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('show_window', default_value='false',
                              description='OpenCV debug window (needs a display)'),
        DeclareLaunchArgument('linear', default_value='0.12',
                              description='Forward speed (m/s)'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo_world.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'camera_pitch': '0.6',
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.1',
            'yaw': '0.0',
        }.items(),
    )

    line_track = Node(
        package='yahboomcar_linefollow',
        executable='line_track',
        name='line_track',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'hsv_file': hsv_file,
            'show_window': show_window,
            'use_sim_time': True,
            'linear': linear,
            'angular_max': 1.2,
            'kp': 0.9,
            'ki': 0.0,
            'kd': 0.25,
            'roi_top_ratio': 0.55,
            'roi_bottom_ratio': 1.0,
            # no collision companion in this minimal demo
            'collision_topic': '',
        }],
    )

    return LaunchDescription(args + [gazebo, line_track])
