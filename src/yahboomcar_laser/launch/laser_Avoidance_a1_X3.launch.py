from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    laser_Avoidance_node = Node(
        package='yahboomcar_laser',
        executable='laser_Avoidance_a1_X3',
    )

    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld19_lidar',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan',
            'frame_id': 'laser_frame',
            'port_name': '/dev/ttyUSB0',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0
        }]
    )
    bringup_node = Node(
        package='yahboomcar_bringup',
        executable='Mcnamu_driver_X3',  # 根据实际可执行文件名调整
    )
    
    return LaunchDescription([
        laser_Avoidance_node,
        lidar_node,
        bringup_node
    ])
