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

## 闭环检测 (Closed-Loop Re-detection)

KCF 本身是开环短时跟踪器，目标被遮挡 / 出视野后会跟丢且无法自恢复。本包在节点层加入了闭环检测：

1. **置信度监测**：每帧从 `KCFTracker::update()` 读取响应峰值 `peak_value`，发布到 `/KCF_confidence`。
2. **丢失判定**：当 `peak_value < lost_threshold` 连续 `lost_patience` 帧时，状态机进入 `LOST`，立刻发布零 `Twist` 停车。
3. **重检测**：首次选定目标时构建 HSV 色调直方图作为外观模板；进入 `LOST` 后每帧在全图做反向投影 + CamShift 搜索；得分超过 `recover_threshold` 即视为重新发现。
4. **闭环回归**：用重检测得到的 ROI 重新初始化 KCF，状态回到 `TRACKING`，PID 跟随重新启用。

### 话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| Pub | `/KCF_image` | `sensor_msgs/Image` | 叠加框 / 状态文字的可视化图 |
| Pub | `/KCF_status` | `std_msgs/String` | `IDLE` / `TRACKING` / `LOST` / `RECOVERED` |
| Pub | `/KCF_confidence` | `std_msgs/Float32` | 实时响应峰值 |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | PID 输出（LOST 时为 0） |

### 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `minDist_` | `1.0` | **PID 跟随距离**（米）。容差 ±0.1m。 |
| `lost_threshold` | `0.15` | 峰值低于此值视为低置信度帧 |
| `lost_patience` | `8` | 连续多少低置信度帧后判定丢失 |
| `recover_threshold` | `0.30` | 反向投影平均响应阈值，超过则恢复 |
| `enable_redetect` | `true` | 关掉后退化为原始开环 KCF |
| `collision_topic` | `/collision_detector/collision` | 碰撞 Bool 脉冲来源 |
| `collision_pause_sec` | `2.0` | 收到碰撞脉冲后冻结 `/cmd_vel` 的秒数 |

## 与 collision_detector 联动 (glue launch)

`launch/KCFTracker_safe_launch.py` 仿 `yahboomcar_linefollow/linefollow_safe_launch.py`：

- 同时拉起 `KCF_Tracker_Node` 和 `collision_detector`。
- detector 用 `stop_on_collision:=false`，**只**发布 `/collision_detector/collision`（`std_msgs/Bool` 脉冲），不抢 `/cmd_vel`。
- KCF 节点订阅这个话题，收到 True 后：
  1. 立刻发一帧零 `Twist` 急停；
  2. 把 `collision_pause_until_` 推迟 `collision_pause_sec` 秒；
  3. `depthCb` 在暂停窗口内一律发零 `Twist`，PID 不输出；
  4. 暂停结束打印 `collision pause released, resuming follow`。
- 暂停**不会**改变 KCF 跟踪状态（仍然在跟），只是冻结底盘运动，等冲击平息后继续跟随。

运行：

```bash
ros2 launch yahboomcar_KCFTracker KCFTracker_safe_launch.py \
    minDist:=0.8 collision_pause_sec:=3.0 accel_threshold:=10.0
```

### 运行环境

需要 ROS 2 Humble + OpenCV ≥ 4。如果使用本机的 `ros:humble` Docker 镜像，可这样进镜像构建：

```bash
docker run --rm -it \
  -v $(pwd):/ws \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros:humble bash
# 在容器内：
cd /ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select yahboomcar_KCFTracker
source install/setup.bash
ros2 launch yahboomcar_KCFTracker KCFTracker_X3.launch.py
```

