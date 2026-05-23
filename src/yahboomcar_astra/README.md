# yahboomcar_astra

基于 **Astra 深度相机** 的视觉应用包，提供颜色识别与跟随能力。

## 功能概述

- `colorTracker.py`：颜色目标跟踪节点。订阅 `/camera/depth/image_raw` 与目标像素位置 `/Current_point`，使用线速度 / 角速度双 PID 控制小车跟随目标并保持深度距离。
- `colorHSV.py`：HSV 颜色阈值调试工具，配合 `colorHSV.text` 存储阈值。
- `astra_common.py`：共享工具函数（PID、阈值加载等）。
- `scripts/opencv/`：纯 OpenCV 示例脚本。

## 目录结构

```
yahboomcar_astra/
├── yahboomcar_astra/       # Python 节点
│   ├── colorTracker.py
│   ├── colorHSV.py
│   ├── colorHSV.text       # HSV 阈值
│   └── astra_common.py
├── launch/
│   └── colorTracker_X3.launch.py
├── scripts/                # OpenCV 示例
└── resource/
```

## 依赖

- `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- `cv_bridge`, `opencv-python`
- `yahboomcar_msgs`（自定义消息 `Position`）
- Astra 相机驱动（独立安装），发布 `/camera/depth/image_raw` 等话题

## 使用

```bash
# 启动 Astra 驱动 + 颜色跟随
ros2 launch yahboomcar_astra colorTracker_X3.launch.py

# 调试 HSV 阈值
ros2 run yahboomcar_astra colorHSV
```
