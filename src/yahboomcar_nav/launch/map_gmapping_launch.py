from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument

# 定义一个launch文件，用于启动gmapping节点
def generate_launch_description():
    RPLIDAR_TYPE = os.getenv('RPLIDAR_TYPE','ld19')
    # 定义一个launch参数，用于指定雷达类型
    rplidar_type_arg = DeclareLaunchArgument(name='rplidar_type', default_value=RPLIDAR_TYPE, 
                                              choices=['ld19'],
                                              description='The type of rplidar')
    # LD19雷达启动配置
    gmapping_ld19_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('yahboomcar_nav'), 'launch'),
        '/map_gmapping_ld19_launch.py']),
        condition=LaunchConfigurationEquals('rplidar_type', 'ld19')
    )
    
    return LaunchDescription([
        rplidar_type_arg,
        gmapping_ld19_launch
    ])
