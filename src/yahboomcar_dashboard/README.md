# yahboomcar_dashboard

Yahboomcar X3 的 **Web Dashboard**，在浏览器中实时显示：

- 各设备的在线 / 离线状态（底盘、IMU、里程计、激光雷达、cmd_vel、手柄、安全状态）
- 电池电压 / 固件版本 / 安全 / 碰撞 / 手柄接管标志
- 线速度 `vx, vy, vz`（来自 `/odom`）+ 实时折线图
- 线加速度 `ax, ay, az`（来自 `/imu/data_raw`）+ 实时折线图
- 角速度 `ωx, ωy, ωz`
- 位姿 `x, y, yaw / roll / pitch`
- `/cmd_vel` 控制指令
- 激光雷达点数 / 最近 / 最远距离

## 设计

- ROS 2 节点 `dashboard_node` 订阅相关话题，记录每个设备的最后到达时间
- 节点内嵌一个 **Python 标准库** HTTP 服务（`http.server.ThreadingHTTPServer`），
  无需 Flask / FastAPI / rosbridge 等额外依赖
- 前端为单页 HTML（原生 JS + Canvas），10 Hz 轮询 `/api/state` 获取 JSON

## 订阅的话题

| 话题 | 类型 | 用途 |
|---|---|---|
| `voltage` | `std_msgs/Float32` | 电池电压 + 判定底盘在线 |
| `edition` | `std_msgs/Float32` | 固件版本号 |
| `safety_status` | `std_msgs/Bool` | 安全保护是否正常 |
| `cmd_vel` | `geometry_msgs/Twist` | 当前下发的控制指令 |
| `vel_raw` | `geometry_msgs/Twist` | 底盘原始速度反馈 |
| `odom` | `nav_msgs/Odometry` | 位姿 + 实际线 / 角速度 |
| `imu/data_raw` | `sensor_msgs/Imu` | ax/ay/az + ωx/ωy/ωz |
| `scan` | `sensor_msgs/LaserScan` | 激光雷达统计 |
| `collision` | `std_msgs/Bool` | 碰撞检测 |
| `JoyState` | `std_msgs/Bool` | 手柄是否接管 |

## 编译

```bash
cd ~/YahboomcarX3
colcon build --symlink-install --packages-select yahboomcar_dashboard
source install/setup.bash
```

## 运行

```bash
# 先启动整车（底盘 + IMU + EKF）
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py

# 再启动 Dashboard（默认 0.0.0.0:8088）
ros2 launch yahboomcar_dashboard dashboard_launch.py

# 自定义端口
ros2 launch yahboomcar_dashboard dashboard_launch.py port:=9000
```

在浏览器中打开：

```
http://<机器人IP>:8088/
```

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `host` | `0.0.0.0` | HTTP 绑定地址 |
| `port` | `8088` | HTTP 端口 |

## API

- `GET /` — Dashboard HTML 页面
- `GET /api/state` — 当前所有状态的 JSON 快照（前端 10 Hz 轮询）

## 设备在线判定

每收到一次对应话题就刷新该设备的时间戳，前端根据 `age`（最后到达至今的秒数）判断：

| 设备 | 超时阈值 |
|---|---|
| chassis (`voltage`) | 2 s |
| imu | 1 s |
| odom | 2 s |
| lidar | 2 s |
| cmd_vel | 2 s |
| safety | 5 s |
| joy | 5 s |
