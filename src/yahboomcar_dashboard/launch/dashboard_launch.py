from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host_arg = DeclareLaunchArgument(
        'host', default_value='0.0.0.0',
        description='HTTP bind address for the dashboard.')
    port_arg = DeclareLaunchArgument(
        'port', default_value='8088',
        description='HTTP port for the dashboard.')

    dashboard = Node(
        package='yahboomcar_dashboard',
        executable='dashboard_node',
        name='yahboomcar_dashboard',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
        }],
    )

    return LaunchDescription([host_arg, port_arg, dashboard])
