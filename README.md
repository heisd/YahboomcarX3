# YahboomcarX3

Yahboomcar X3（麦克纳姆轮）机器人在 **ROS 2** 下的完整工作空间，包含底盘驱动、
遥控、SLAM 建图、Nav2 导航、激光与视觉应用等全部功能包。

## 目录结构

```
YahboomcarX3/
└── src/
    ├── ldlidar_stl_ros2/         # 乐动 LD06 / LD19 激光雷达驱动
    ├── py_install/               # Rosmaster_Lib，下位机串口通信 Python 库
    ├── yahboomcar_KCFTracker/    # KCF 目标跟踪（C++ + OpenCV）
    ├── yahboomcar_astra/         # Astra 深度相机：颜色识别 / 跟随
    ├── yahboomcar_base_node/     # C++ 版底盘里程节点（odom + TF）
    ├── yahboomcar_bringup/       # 整车启动包：底盘驱动 / EKF / 标定 / 巡逻
    ├── yahboomcar_collision/     # 基于 IMU 加速度突变的碰撞检测
    ├── yahboomcar_ctrl/          # 键盘 / 手柄遥控
    ├── yahboomcar_description/   # URDF / Xacro / 网格模型
    ├── yahboomcar_laser/         # 激光避障 / 跟随 / 防撞
    ├── yahboomcar_linefollow/    # 视觉巡线（HSV + ROI + PID）
    └── yahboomcar_nav/           # 建图 (GMapping / Cartographer / RTAB-Map) 与 Nav2 导航
```

## 功能包简介

| 功能包 | 类型 | 说明 |
|---|---|---|
| `ldlidar_stl_ros2` | C++ | 乐动 LD06 / LD19 激光雷达 ROS 2 驱动，发布 `/scan` |
| `py_install` | Python 库 | `Rosmaster_Lib`，封装与下位机的串口通信（电机、IMU、灯效等） |
| `yahboomcar_KCFTracker` | C++ | 基于 KCF 算法的目标跟踪，鼠标框选目标后 PID 跟随 |
| `yahboomcar_astra` | Python | Astra 深度相机颜色识别与跟随（含 HSV 调试工具） |
| `yahboomcar_base_node` | C++ | C++ 版底盘里程节点，发布 `odom` 与 `odom→base_footprint` TF |
| `yahboomcar_bringup` | Python | **整车启动入口**：底盘驱动、IMU 滤波、EKF 融合、里程标定、巡逻示例 |
| `yahboomcar_collision` | Python | 基于 IMU 冲击信号的碰撞检测，触发后可发布 `/collision` 并急停 |
| `yahboomcar_ctrl` | Python | 终端键盘遥控 (`yahboom_keyboard`)、手柄遥控 (`yahboom_joy_X3` / `R2`) |
| `yahboomcar_description` | Python (ament) | X3 / X4 / R2 等车型 URDF、网格与 RViz 显示 |
| `yahboomcar_laser` | Python | 激光避障 / 跟随 / 警告三个节点，支持 `/JoyState` 手柄接管暂停 |
| `yahboomcar_linefollow` | Python | 视觉巡线，detect 模式学习 HSV、track 模式 PID 巡线 |
| `yahboomcar_nav` | Python | GMapping / Cartographer / RTAB-Map 建图 + Nav2 (DWA / TEB) 导航 |

## 依赖环境

- **OS**：Ubuntu 24.04
- **ROS 2**：Jazzy
- **Python**：rclpy、opencv-python、numpy
- **C++**：rclcpp、tf2、cv_bridge、OpenCV
- **第三方 ROS 包**：
  - `robot_localization`（EKF 融合）
  - `nav2_*`（Nav2 导航栈）
  - `slam_toolbox` / `cartographer_ros` / `rtabmap_ros`（按需）
  - `joy`、`teleop_twist_keyboard`（遥控）
- **硬件库**：`Rosmaster_Lib`（见 `src/py_install`）

## 编译

```bash
# 1. 安装 Rosmaster_Lib（下位机串口库）
# 注意：Ubuntu 24.04 (Jazzy) 的 setuptools 已移除 "setup.py install"，且系统 Python
# 受 PEP 668 保护，需使用 pip 安装（--break-system-packages 写入系统 site-packages）。
cd src/py_install
sudo pip3 install . --break-system-packages
cd ../..

# 2. 安装 ROS 依赖
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译工作空间（推荐使用 symlink-install，方便 Python 修改 / 巡线 HSV 回写）
colcon build --symlink-install

# 4. 加载环境
source install/setup.bash
```

## 快速上手

```bash
# 启动整车（底盘驱动 + IMU + EKF + robot_state_publisher）
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py

# 键盘遥控
ros2 run yahboomcar_ctrl yahboom_keyboard

# 激光雷达
ros2 launch ldlidar_stl_ros2 ld19.launch.py

# GMapping 建图
ros2 launch yahboomcar_nav map_gmapping_launch.py
# 保存地图
ros2 launch yahboomcar_nav save_map_launch.py

# Nav2 导航（DWA）
ros2 launch yahboomcar_nav navigation_dwa_launch.py
```

## 话题约定

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令，所有遥控 / 自主节点的输出 |
| `/odom` | `nav_msgs/Odometry` | 底盘里程 |
| `/imu/data_raw`、`/imu/data` | `sensor_msgs/Imu` | IMU 原始 / 滤波后数据 |
| `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达 |
| `/JoyState` | `std_msgs/Bool` | 手柄是否接管；自主节点据此暂停 |
| `/RGBLight` | `std_msgs/Int32` | 车顶 RGB 灯效控制 |
| `/collision` | `std_msgs/Bool` | 碰撞检测标志 |

## 各车型说明

工作空间以 **X3（麦克纳姆轮）** 为主，部分包同时兼容 **R2（阿克曼）** 与 **X1**，
通常通过节点 / launch 文件名后缀 `_X3` / `_R2` / `_x1` 区分。

## 许可

各功能包遵循其各自 `package.xml` 中声明的开源许可（多数为 Apache-2.0 / BSD）。
`ldlidar_stl_ros2` 由深圳乐动机器人有限公司提供，遵循其 `LICENSE` 文件。

## 更多文档

每个功能包根目录下都有独立的 `README.md`，包含详细的节点说明、参数列表与使用示例。
