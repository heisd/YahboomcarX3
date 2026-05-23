# yahboomcar_KCFTracker

基于 **KCF (Kernelized Correlation Filter)** 算法的目标跟踪节点（C++ 实现）。

## 功能概述

- `KCF_Tracker_Node`：订阅深度相机的 RGB 图像，弹出 OpenCV 窗口让用户用鼠标框选目标，随后用 KCF 跟踪并通过 PID 输出 `/cmd_vel`，驱动小车跟随目标。
- `include/yahboomcar_KCFTracker/` 内含上游 KCF 算法实现 (`kcftracker.cpp`、`fhog.cpp`) 及内部 `PID.cpp`。

操作约定：
- 鼠标左键拖拽：框选目标；
- `Space`：开始跟踪；
- `Esc`：退出。

## 目录结构

```
yahboomcar_KCFTracker/
├── src/                    # 节点入口
│   ├── KCF_Tracker.cpp
│   └── KCF_Tracker_main.cpp
├── include/yahboomcar_KCFTracker/
│   ├── KCF_Tracker.h
│   ├── kcftracker.{h,cpp}  # KCF 算法
│   ├── fhog.{h,cpp}        # FHOG 特征
│   └── PID.{h,cpp}
├── launch/
│   └── KCFTracker_X3.launch.py
├── CMakeLists.txt
└── package.xml
```

## 依赖

- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`
- `cv_bridge`, `OpenCV`

## 构建与运行

```bash
colcon build --packages-select yahboomcar_KCFTracker
ros2 launch yahboomcar_KCFTracker KCFTracker_X3.launch.py
```
