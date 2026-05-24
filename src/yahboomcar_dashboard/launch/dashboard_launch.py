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
    collision_topic_arg = DeclareLaunchArgument(
        'collision_topic', default_value='/collision_detector/collision',
        description='Topic where the collision detector publishes its Bool '
                    'flag. Default matches yahboomcar_collision\'s ~/collision '
                    'on a node named "collision_detector".')

    dashboard = Node(
        package='yahboomcar_dashboard',
        executable='dashboard_node',
        name='yahboomcar_dashboard',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'collision_topic': LaunchConfiguration('collision_topic'),
        }],
    )

    return LaunchDescription([host_arg, port_arg, collision_topic_arg, dashboard])
