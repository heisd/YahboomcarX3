# yahboomcar_bringup

Yahboomcar X3 底盘的 **启动 (bringup)** 包，是上电后启动整车的主要入口。

## 功能概述

- **底盘驱动节点**：`Mcnamu_driver_X3.py` 通过 `Rosmaster_Lib` 与下位机串口通信，发布 `/imu`、`/vel_raw`、`/joint_states` 等话题，并订阅 `/cmd_vel`、`/RGBLight` 等控制话题。
- **里程标定工具**：`calibrate_linear_X3.py` / `calibrate_angular_X3.py` 用于线速度与角速度标定。
- **巡逻例程**：`patrol.py`、`patrol_a1_X3.py`、`patrol_4ROS.py` 等示例脚本。
- **EKF 融合**：`launch/ekf_x1_x3_launch.py` + `param/ekf_x1_x3.yaml` 使用 `robot_localization` 融合编码器与 IMU。
- **整车启动**：`launch/yahboomcar_bringup_X3_launch.py` 一键启动底盘驱动、IMU 滤波、EKF、机器人描述等。

## 目录结构

```
yahboomcar_bringup/
├── yahboomcar_bringup/   # Python 节点源码
├── launch/               # 启动文件
├── param/                # ekf / imu_filter 参数
├── rviz/                 # rviz 配置
└── resource/
```

## 依赖

- `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- `Rosmaster_Lib`（位于 `src/py_install`）
- `robot_localization`（EKF 融合）

## 使用

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

可执行节点见 `setup.py` 中的 `entry_points`，例如：

```bash
ros2 run yahboomcar_bringup Mcnamu_driver_X3
ros2 run yahboomcar_bringup calibrate_linear_X3
```
