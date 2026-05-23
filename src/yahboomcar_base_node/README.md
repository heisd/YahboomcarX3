# yahboomcar_base_node

ROS2 C++ 版本的 **底盘里程基础节点**，提供 `odom` → `base_footprint` 的 TF 广播与里程发布。

## 功能概述

- `base_node_X3`：X3（麦克纳姆轮）车型的里程节点，订阅底盘速度或编码器数据，发布 `nav_msgs/Odometry` 与 TF。
- `base_node_R2`：R2（阿克曼）车型的里程节点。
- `base_node_x1`：x1 车型的里程节点。
- `talker`：示例发布节点，用于联调。

> 通常的整车启动由 Python 版本的 `yahboomcar_bringup/Mcnamu_driver_X3.py` 完成；本包作为 C++ 实现的备选 / 性能优化版本存在。

## 目录结构

```
yahboomcar_base_node/
├── src/
│   ├── base_node_X3.cpp
│   ├── base_node_R2.cpp
│   ├── base_node_x1.cpp
│   └── talker.cpp
├── CMakeLists.txt
└── package.xml
```

## 依赖

- `rclcpp`, `geometry_msgs`, `nav_msgs`
- `tf2`, `tf2_ros`
- `turtlesim`（仅用于示例消息类型）

## 构建与运行

```bash
colcon build --packages-select yahboomcar_base_node

ros2 run yahboomcar_base_node base_node_X3
```
