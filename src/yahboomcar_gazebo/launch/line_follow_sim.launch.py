"""Line following in Gazebo, with mode switching.

Brings up the line-track world (a yellow line on the floor) with the X3 and
tilts the camera down. A `mode` argument selects which line-follow node runs,
both fed from the simulated camera topic instead of a USB camera:

    mode:=track   (default) run line_track -> drives /cmd_vel along the line
    mode:=detect            run line_detect -> mouse box-select on the sim
                            image to learn the line's HSV (needs a display)

Typical flow:
    # 1) learn the HSV by box-selecting the line on the sim image
    ros2 launch yahboomcar_gazebo line_follow_sim.launch.py mode:=detect
    # (drag a rectangle over the yellow line; HSV auto-saves to hsv_file)
    # 2) follow the line
    ros2 launch yahboomcar_gazebo line_follow_sim.launch.py mode:=track
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('yahboomcar_gazebo')
    world = os.path.join(pkg, 'worlds', 'yahboom_line.world')
    hsv_file = os.path.join(pkg, 'config', 'line_hsv_sim.txt')

    gui = LaunchConfiguration('gui')
    mode = LaunchConfiguration('mode')
    image_topic = LaunchConfiguration('image_topic')
    show_window = LaunchConfiguration('show_window')
    linear = LaunchConfiguration('linear')

    is_detect = IfCondition(PythonExpression(["'", mode, "' == 'detect'"]))
    is_track = IfCondition(PythonExpression(["'", mode, "' == 'track'"]))

    args = [
        DeclareLaunchArgument('mode', default_value='track',
                              choices=['track', 'detect'],
                              description='track = follow the line; '
                                          'detect = box-select to learn HSV'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('show_window', default_value='false',
                              description='track-mode OpenCV debug window '
                                          '(needs a display)'),
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
        condition=is_track,
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
            'collision_topic': '',
        }],
    )

    # detect mode always needs the window (mouse selection); ignores show_window
    line_detect = Node(
        package='yahboomcar_linefollow',
        executable='line_detect',
        name='line_detect',
        output='screen',
        condition=is_detect,
        parameters=[{
            'image_topic': image_topic,
            'hsv_file': hsv_file,
            'autosave': True,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription(args + [gazebo, line_track, line_detect])
