# YahboomcarX3 启动流程

本文件梳理 YahboomcarX3（麦克纳姆轮）在 ROS 2（推荐 Humble）下从编译到各功能的
**完整启动顺序**，包括整车底盘、激光、SLAM 建图、Nav2 导航、视觉巡线 + 二维码岔路
决策，以及 Gazebo 仿真。命令均以工作空间根目录 `YahboomcarX3/` 为起点。

> 约定：每开一个新终端都要先 `source install/setup.bash`。

## 0. 编译与环境

```bash
# 1) 安装下位机串口库 Rosmaster_Lib
cd src/py_install && sudo python3 setup.py install && cd ../..

# 2) 安装 ROS 依赖
rosdep install --from-paths src --ignore-src -r -y

# 3) 编译（用 --symlink-install，巡线学到的 HSV 才能回写源码树并提交）
colcon build --symlink-install

# 4) 加载环境（每个新终端都要执行）
source install/setup.bash
```

## 1. 整车启动（真实硬件，所有上层功能的前置）

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

这一步拉起底盘驱动 `Mcnamu_driver_X3`、里程节点 `base_node_X3`、IMU 滤波
（imu_filter_madgwick）、EKF 融合（robot_localization）与 `robot_state_publisher`。

启动后应能看到的核心话题：

| 话题 | 说明 |
|---|---|
| `/cmd_vel` | 速度指令入口（所有遥控 / 自主节点都往这里发） |
| `/odom` | 底盘里程 |
| `/imu/data_raw`、`/imu/data` | IMU 原始 / 滤波 |
| TF：`odom → base_footprint` | 由 EKF 发布（`pub_odom_tf:=false` 时） |

可选遥控（手动验证底盘）：

```bash
ros2 run yahboomcar_ctrl yahboom_keyboard      # 键盘
ros2 run yahboomcar_ctrl yahboom_joy_X3        # 手柄
```

> 手柄接管会发布 `/JoyState=true`，巡线 / 激光等自主节点据此自动暂停。

## 2. 激光雷达

```bash
# LD19（按你的雷达型号选择对应 launch）
ros2 launch ldlidar_stl_ros2 ld19.launch.py
# 或经由 nav 包的封装
ros2 launch yahboomcar_nav laser_bringup_launch.py
```

发布 `/scan`（`sensor_msgs/LaserScan`），供 SLAM 与 Nav2 使用。

## 3. SLAM 建图

先确保已完成第 1、2 步（底盘 + 激光），再选一种建图算法：

```bash
ros2 launch yahboomcar_nav map_gmapping_launch.py       # GMapping
ros2 launch yahboomcar_nav map_cartographer_launch.py   # Cartographer
ros2 launch yahboomcar_nav map_rtabmap_launch.py        # RTAB-Map（带相机）
```

用遥控把车开一圈把环境扫全，然后保存地图：

```bash
ros2 launch yahboomcar_nav save_map_launch.py
# 地图默认落在 src/yahboomcar_nav/maps/（yahboomcar.yaml + .pgm）
```

## 4. Nav2 导航（全局选路 + 局部避障）

地图建好后启动导航。全局规划器为 `nav2_navfn_planner/NavfnPlanner`
（见 `params/teb_nav_params.yaml` 的 `planner_server`）：在 `global_costmap`
上算出一条**代价最低的全局路线**——这就是"在多条路里自动挑一条简单 / 最短的路"。
局部规划器按 launch 不同分别是 DWA / TEB。

```bash
ros2 launch yahboomcar_nav navigation_dwa_launch.py     # 全局 Navfn + 局部 DWA
ros2 launch yahboomcar_nav navigation_teb_launch.py     # 全局 Navfn + 局部 TEB
```

在 RViz 中用 **2D Pose Estimate** 给初始位姿、用 **2D Goal Pose** 下发目标点。

调参提示（让全局选路更"简单"）：
- `planner_server.GridBased.use_astar`：`false` = Dijkstra（稳），`true` = A\*（快）。
- `planner_server.GridBased.tolerance`：到不了精确目标时允许的偏差。
- `global_costmap` 的 `inflation_radius` / `cost_scaling_factor`：膨胀越大越远离墙。

## 5. 视觉巡线 + 二维码岔路决策

巡线分两步：先 **detect 学颜色**，再 **track 巡线**；岔路口由二维码决定走哪条。

```bash
# 5.1 学习要追踪的线色（鼠标框选线条，HSV 自动写回 HSV.txt）
ros2 launch yahboomcar_linefollow line_detect_launch.py

# 5.2 纯巡线（读取 HSV.txt，对横向误差做 PID，输出 /cmd_vel）
ros2 launch yahboomcar_linefollow line_track_launch.py

# 5.3 巡线 + 二维码岔路 + 碰撞安全联动（推荐整套）
ros2 launch yahboomcar_linefollow qr_linefollow_launch.py \
    linear:=0.15 qr_check_every:=3
```

**二维码岔路语义**（`params/qr_actions.json`）：车在岔路口看到二维码即查表执行一段
开环定时机动，结束后恢复巡线。

| 二维码 | 动作 | 含义 |
|---|---|---|
| `FORK_LEFT` / `FORK_RIGHT` | left / right | 在岔路走左 / 右分支 |
| `FORK_STRAIGHT` | straight | 忽略线条、直穿岔路 |
| `STATION_A` / `STATION_B` | station | 该分支是站点，停车（`hold_time` 控制停多久） |
| `STOP` | stop | 急停 / 锁定（`hold_time<=0` 一直停到 `switch` 切换） |

优先级：**碰撞急停 / 手柄接管 > 二维码机动 > 普通巡线**。相关话题：
- 订阅 `/JoyState`（手柄接管时暂停）、`/collision_detector/collision`（碰撞暂停）。
- 发布 `/cmd_vel`、`~/qr`（当前二维码动作，便于调试）。

运行时可热调（无需重启节点）：

```bash
ros2 param set /line_track enable_qr false      # 临时关掉二维码优先
ros2 param set /line_track switch false          # 解除站点 / 停车的锁定
```

> 注：`qr_cooldown_sec` 控制同一个二维码多久内不重复触发；冷却从机动**结束**起算，
> 因此即使站点 `hold_time` 较长，车驶离前也不会被同一张码反复触发。

## 6. Gazebo 仿真（无硬件验证）

```bash
# 仅起世界 + 机器人
ros2 launch yahboomcar_gazebo gazebo_world.launch.py

# 仿真巡线（从仿真相机话题 /camera/image_raw 取图）
ros2 launch yahboomcar_gazebo line_follow_sim.launch.py mode:=detect   # 先学色
ros2 launch yahboomcar_gazebo line_follow_sim.launch.py mode:=track    # 再巡线
# 七色线世界可直接按色选线：
ros2 launch yahboomcar_gazebo line_follow_sim.launch.py \
    world:=yahboom_lines.world color:=green
```

## 启动顺序总览

```
[0] colcon build --symlink-install  →  source install/setup.bash
                     │
        ┌────────────┴─────────────────────────────┐
        ▼ 真实硬件                                   ▼ 仿真
[1] bringup_X3 (底盘+IMU+EKF+TF)            gazebo_world / line_follow_sim
        │
[2] 激光 ld19 → /scan
        │
   ┌────┴───────────────┬───────────────────────────┐
   ▼ 建图                ▼ 导航                        ▼ 巡线
[3] map_gmapping...     [4] navigation_dwa/teb       [5] line_detect → line_track
    → save_map              (RViz 给目标点)               → qr_linefollow (含安全联动)
```
