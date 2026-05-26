import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('yahboomcar_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    default_world = os.path.join(pkg, 'worlds', 'yahboom_room.world')
    xacro_file = os.path.join(pkg, 'urdf', 'yahboomcar_X3_gazebo.urdf.xacro')
    scenes_file = os.path.join(pkg, 'config', 'scenes.yaml')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    scene = LaunchConfiguration('scene')
    gui = LaunchConfiguration('gui')

    args = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Path to the Gazebo .world file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.1'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch the Gazebo client GUI'),
        DeclareLaunchArgument('scene', default_value='',
                              description='Scene spawned on startup '
                                          '(name from config/scenes.yaml, '
                                          '"" = none)'),
    ]

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'verbose': 'true'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'yahboomcar',
            '-x', x_pose, '-y', y_pose, '-z', z_pose, '-Y', yaw,
        ],
    )

    scene_switcher = Node(
        package='yahboomcar_gazebo',
        executable='scene_switcher',
        name='scene_switcher',
        output='screen',
        parameters=[{
            'scenes_file': scenes_file,
            'default_scene': scene,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription(
        args + [gzserver, gzclient, robot_state_publisher,
                spawn_robot, scene_switcher])
