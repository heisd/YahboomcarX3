# yahboomcar_description

Yahboomcar 的 **机器人描述 (URDF / Xacro / 网格模型)** 包，用于在 RViz、Gazebo 和导航栈中可视化与提供 TF 关系。

## 功能概述

- 提供 X3（麦克纳姆轮）与 X4 / R2 等车型的 URDF / Xacro 模型；
- 提供轮子、激光雷达、深度相机等传感器的 STL / DAE 网格；
- 提供多机器人 (`robot1`、`robot2`) 的命名空间化描述；
- 提供 `display_X3.launch.py` 等启动文件，结合 `robot_state_publisher` + `joint_state_publisher_gui` 在 RViz 中显示机器人。

## 目录结构

```
yahboomcar_description/
├── urdf/                  # URDF / Xacro 文件
│   ├── yahboomcar_X3.urdf(.xacro)
│   ├── yahboomcar_X4.urdf
│   └── yahboomcar_X3_robot1/2.urdf
├── meshes/                # STL / DAE 三维模型
│   ├── mecanum/           # 麦轮 X3 模型
│   ├── Ackermann/         # 阿克曼 R2 模型
│   └── sensor/            # 雷达、相机等传感器
├── rviz/                  # RViz 配置
└── launch/                # display_X3.launch.py 等
```

## 依赖（exec）

- `xacro`, `robot_state_publisher`, `joint_state_publisher(_gui)`
- `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`

## 使用

```bash
# 仅显示模型 (RViz)
ros2 launch yahboomcar_description display_X3.launch.py

# 多机器人
ros2 launch yahboomcar_description description_X3_multi_robot1.launch.py
ros2 launch yahboomcar_description description_X3_multi_robot2.launch.py
```
