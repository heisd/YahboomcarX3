# yahboomcar_collision

基于 **IMU 加速度突变** 的碰撞检测节点。订阅底盘驱动发布的 `sensor_msgs/Imu`，
在监测到加速度（或同时角速度）短时间内剧烈跳变时，发布 `/collision` 标志，并可
选地立刻发布零 `Twist` 急停。

## 工作原理（设计成尽量少误判）

1. **基线 EMA**：用慢速 EMA 估计 IMU 线加速度的稳态基线，自动吸收重力分量与稳定偏置——
   因此不依赖 IMU 安装方向；
2. **冲击信号**：`shock = ‖a - baseline‖`，正常颠簸时它很小，撞击会有明显尖峰；
3. **可选角速度联合判据**：`use_gyro:=true` 时同时要求 `‖ω - ω_base‖` 也超阈值，
   进一步过滤纯震动；
4. **去毛刺**：必须 **连续 `min_trigger_samples` 帧** 都超阈值才触发；
5. **冷却时间**：触发后 `cooldown_sec` 秒内不再重复触发，避免同一次撞击的余震反复报警；
6. **冲击时冻结基线**：处于疑似冲击中时不更新 baseline，防止把撞击平滑成基线被吃掉。

默认阈值故意调得 **比较高**（保守），用户可在 launch 启动时按需调低。

## 目录结构

```
yahboomcar_collision/
├── yahboomcar_collision/
│   ├── __init__.py
│   └── collision_detector.py
├── launch/
│   └── collision_detector_launch.py
├── params/
│   └── collision.yaml
├── resource/yahboomcar_collision
├── package.xml
└── setup.py
```

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| sub | `imu_topic` (默认 `imu/data_raw`) | `sensor_msgs/Imu` | 由 `yahboomcar_bringup` 的 `Mcnamu_driver_X3` 发布 |
| pub | `~/collision` | `std_msgs/Bool` | 触发时发 True 脉冲 |
| pub | `~/shock` | `std_msgs/Float32` | 当前冲击幅值，方便调阈值 |
| pub | `cmd_vel_topic` (默认 `/cmd_vel`) | `geometry_msgs/Twist` | 触发时发零速急停（`stop_on_collision:=true` 时）|

## 参数 / Launch 参数

所有阈值都暴露成 launch 参数，运行时可改：

| 参数 | 默认 | 说明 |
|---|---|---|
| `imu_topic` | `imu/data_raw` | 监听的 IMU 话题 |
| `accel_threshold` | `12.0` | 线加速度冲击阈值（m/s²），**默认偏高，降低误判** |
| `gyro_threshold`  | `6.0`  | 角速度冲击阈值（rad/s），需 `use_gyro:=true` 才生效 |
| `use_gyro` | `false` | 加速度+角速度双判据（更严，更少误判） |
| `baseline_alpha` | `0.02` | 基线 EMA 权重，越小越稳但响应慢 |
| `min_trigger_samples` | `2` | 连续超阈值的 IMU 采样数，去毛刺 |
| `cooldown_sec` | `1.5` | 触发后的冷却时间（秒）|
| `stop_on_collision` | `true` | 触发时是否发零 Twist 急停 |
| `cmd_vel_topic` | `/cmd_vel` | 急停 Twist 发到哪 |

## 使用

启动底盘驱动后（确保 `imu/data_raw` 在发布），运行：

```bash
# 默认参数（保守、不易误判）
ros2 launch yahboomcar_collision collision_detector_launch.py

# 调阈值，例如更敏感、更长冷却
ros2 launch yahboomcar_collision collision_detector_launch.py \
    accel_threshold:=8.0 \
    cooldown_sec:=2.0 \
    use_gyro:=true \
    gyro_threshold:=4.0

# 用 EKF 融合后的 IMU
ros2 launch yahboomcar_collision collision_detector_launch.py \
    imu_topic:=/imu/data

# 只检测、不急停，由上层决定怎么做
ros2 launch yahboomcar_collision collision_detector_launch.py \
    stop_on_collision:=false
```

调试时观察实时冲击：

```bash
ros2 topic echo /collision_detector/shock
ros2 topic echo /collision_detector/collision
```

在线改参数：

```bash
ros2 param set /collision_detector accel_threshold 10.0
ros2 param set /collision_detector use_gyro true
```
