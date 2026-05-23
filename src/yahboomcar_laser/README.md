# yahboomcar_laser

基于 2D 激光雷达 (`/scan`) 的 **避障 / 跟随 / 防撞** 应用包。

## 功能概述

- `laser_Avoidance_a1_X3`：激光避障。按设定的扫描角 `LaserAngle` 与响应距离 `ResponseDist` 检测左 / 中 / 右扇区的最近障碍，并发布 `/cmd_vel` 避让。
- `laser_Tracker_a1_X3`：激光跟随。锁定前方最近目标并保持距离，使用内部 PID 输出线 / 角速度。
- `laser_Warning_a1_X3`：激光防撞 / 警告，进入危险距离时减速或停车。
- `common.py`：共享的 `SinglePID` 等工具。

节点同时订阅 `/JoyState`：当手柄遥控接管时自动暂停自主行为。

## 目录结构

```
yahboomcar_laser/
├── yahboomcar_laser/       # Python 节点
│   ├── laser_Avoidance_a1_X3.py
│   ├── laser_Tracker_a1_X3.py
│   ├── laser_Warning_a1_X3.py
│   └── common.py
├── launch/                 # 三个对应的 launch 文件
└── resource/
```

## 依赖

- `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`
- `Rosmaster_Lib`
- 一个激光节点（如 `ldlidar_stl_ros2` 或 `ydlidar`）发布 `/scan`

## 使用

```bash
# 避障
ros2 launch yahboomcar_laser laser_Avoidance_a1_X3.launch.py

# 跟随
ros2 launch yahboomcar_laser laser_Tracker_a1_X3.launch.py

# 防撞
ros2 launch yahboomcar_laser laser_Warning_a1_X3.launch.py
```

运行参数（如 `linear`、`angular`、`LaserAngle`、`ResponseDist`、`Switch`）可通过 `ros2 param set` 在线调节。
