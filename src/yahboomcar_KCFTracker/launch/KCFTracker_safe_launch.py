#!/usr/bin/env python3
"""KCF 跟踪 + 碰撞检测 联合启动 (glue launch)。

结构（参考 yahboomcar_linefollow/linefollow_safe_launch.py）：

    collision_detector  --(Bool)-->  /collision_detector/collision
                                            |
                                            v
    KCF_Tracker_Node  --(Twist)-->  /cmd_vel    收到 True 脉冲后
                                                暂停跟随 PID
                                                collision_pause_sec 秒。

detector 设 `stop_on_collision:=false`，
所以只有 KCF 节点写 /cmd_vel —— 暂停由真正持续输出的发布者来掌控。

命令行可覆盖任意参数，例如：

    ros2 launch yahboomcar_KCFTracker KCFTracker_safe_launch.py \
        minDist:=0.8 accel_threshold:=10.0 collision_pause_sec:=3.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


COLLISION_FLAG_TOPIC = '/collision_detector/collision'


def generate_launch_description():
    args = [
        # ---- KCF 跟踪节点参数 ----
        DeclareLaunchArgument('minDist', default_value='1.0',
            description='跟随距离（米），即 PID 的设定点。'),
        DeclareLaunchArgument('lost_threshold', default_value='0.15',
            description='KCF 响应峰值低于此值视为低置信度帧。'),
        DeclareLaunchArgument('lost_patience', default_value='8',
            description='连续多少低置信度帧后判定为 LOST。'),
        DeclareLaunchArgument('recover_threshold', default_value='0.30',
            description='反向投影平均响应高于此值即认为目标重新出现。'),
        DeclareLaunchArgument('enable_redetect', default_value='true',
            description='关掉后退化为原始开环 KCF。'),
        # 重检测外观模型（HSV 色调直方图）的可调参数
        DeclareLaunchArgument('hue_bins', default_value='32',
            description='HSV 色调直方图 bin 数。'),
        DeclareLaunchArgument('sat_min', default_value='30',
            description='S 通道下限，过滤过白 / 灰像素。'),
        DeclareLaunchArgument('val_min', default_value='30',
            description='V 通道下限，过滤过暗像素。'),
        DeclareLaunchArgument('collision_pause_sec', default_value='2.0',
            description='每次收到碰撞脉冲后，冻结 /cmd_vel 的秒数。'),

        # ---- collision_detector 参数（与 linefollow_safe 保持一致）----
        DeclareLaunchArgument('imu_topic', default_value='imu/data_raw'),
        DeclareLaunchArgument('accel_threshold', default_value='12.0'),
        DeclareLaunchArgument('gyro_threshold', default_value='6.0'),
        DeclareLaunchArgument('use_gyro', default_value='false'),
        DeclareLaunchArgument('baseline_alpha', default_value='0.02'),
        DeclareLaunchArgument('min_trigger_samples', default_value='2'),
        DeclareLaunchArgument('cooldown_sec', default_value='1.5'),
    ]

    kcf = Node(
        package='yahboomcar_KCFTracker',
        executable='KCF_Tracker_Node',
        name='image_converter',
        output='screen',
        parameters=[{
            'minDist_':            LaunchConfiguration('minDist'),
            'lost_threshold':      LaunchConfiguration('lost_threshold'),
            'lost_patience':       LaunchConfiguration('lost_patience'),
            'recover_threshold':   LaunchConfiguration('recover_threshold'),
            'enable_redetect':     LaunchConfiguration('enable_redetect'),
            'hue_bins':            LaunchConfiguration('hue_bins'),
            'sat_min':             LaunchConfiguration('sat_min'),
            'val_min':             LaunchConfiguration('val_min'),
            'collision_topic':     COLLISION_FLAG_TOPIC,
            'collision_pause_sec': LaunchConfiguration('collision_pause_sec'),
        }],
    )

    collision = Node(
        package='yahboomcar_collision',
        executable='collision_detector',
        name='collision_detector',
        output='screen',
        parameters=[{
            'imu_topic':           LaunchConfiguration('imu_topic'),
            'accel_threshold':     LaunchConfiguration('accel_threshold'),
            'gyro_threshold':      LaunchConfiguration('gyro_threshold'),
            'use_gyro':            LaunchConfiguration('use_gyro'),
            'baseline_alpha':      LaunchConfiguration('baseline_alpha'),
            'min_trigger_samples': LaunchConfiguration('min_trigger_samples'),
            'cooldown_sec':        LaunchConfiguration('cooldown_sec'),
            # /cmd_vel 由 KCF 跟踪节点独占；detector 只发布 Bool 脉冲。
            'stop_on_collision':   False,
        }],
    )

    return LaunchDescription(args + [collision, kcf])
