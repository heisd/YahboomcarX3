# yahboomcar_linefollow

基于 **HSV 颜色 + ROI 选择** 的视觉巡线包，提供两种模式：

| 模式 | 节点 | 作用 |
|---|---|---|
| 检测模式 (detect) | `line_detect` | 打开摄像头，鼠标框选线条样本，自动学习 HSV 阈值并保存到文件 |
| 跟踪模式 (track)  | `line_track`  | 加载已保存的 HSV，在画面下方 ROI 带内提取线条质心，PID 控制 `/cmd_vel` 巡线 |

两个模式共用同一个 **持久化** HSV 文件：`params/HSV.txt`，跟随包一起提交到仓库。
检测模式 **每学到一次新 ROI 就自动覆盖** 这个文件，无需手动按 `s`；之后任何 launch
（包括 `linefollow_safe_launch.py`）默认就读它，可以直接开跑。

> 建议用 `colcon build --symlink-install`，这样 `<share>/params/HSV.txt` 与源码 `params/HSV.txt`
> 是符号链接，detect 写回的内容直接落到源码、被 git 跟踪。普通 `colcon build` 写入的是
> install 空间，下一次 build 会被仓库里的种子覆盖。

## 目录结构

```
yahboomcar_linefollow/
├── yahboomcar_linefollow/
│   ├── __init__.py
│   ├── line_common.py     # HSV I/O、ROI 学习、掩膜、PID 等共享工具
│   ├── line_detect.py     # 检测模式：鼠标框选 ROI → 学习 HSV
│   └── line_track.py      # 跟踪模式：HSV mask → 质心 → PID → /cmd_vel
├── launch/
│   ├── line_detect_launch.py
│   └── line_track_launch.py
├── params/
│   └── line_track.yaml    # 跟踪模式默认参数
├── resource/yahboomcar_linefollow
├── package.xml
└── setup.py
```

## 依赖

- `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- `cv_bridge`, `opencv-python` (`cv2`)
- 图像来源二选一：
  - 一个可被 OpenCV 打开的 USB 摄像头（`camera_index` 参数，默认）；
  - 或一个 `sensor_msgs/Image` 话题（`image_topic` 参数，例如 Gazebo 仿真的
    `/camera/image_raw`）。

## 图像来源（USB vs ROS 话题）

`line_track` 新增 `image_topic` 参数：

| `image_topic` | 行为 |
|---|---|
| `''`（空，默认） | 走原有逻辑，用 `cv.VideoCapture(camera_index)` 开本地 USB 摄像头 |
| 非空（如 `/camera/image_raw`） | 改为订阅该 ROS 话题（cv_bridge 转 `bgr8`），不打开本地摄像头 |

这样同一个节点既能跑真车的 USB 摄像头，也能跑 Gazebo 仿真相机。仿真一键启动见
`yahboomcar_gazebo` 包的 `line_follow_sim.launch.py`。

```bash
# 仿真：从 Gazebo 相机话题取图
ros2 run yahboomcar_linefollow line_track --ros-args \
    -p image_topic:=/camera/image_raw -p show_window:=false
```

## 使用

### 1) 检测模式 — 标定 HSV

```bash
# 默认就写 params/HSV.txt，不用再传 hsv_file
ros2 launch yahboomcar_linefollow line_detect_launch.py

# 想写到别的位置（例如临时 try）：
ros2 launch yahboomcar_linefollow line_detect_launch.py \
    hsv_file:=/tmp/try.txt autosave:=false
```

在弹出的 `line_detect` 窗口里：

- **鼠标左键拖拽** 在线条上框选一个矩形区域，节点自动取该 ROI 内像素的 HSV 极值
  并加 padding，作为 inRange 范围；实时显示二值化结果与最大轮廓质心；
- **每框一次都会自动覆盖 `hsv_file`**（受 `autosave` 控制，默认 `true`），
  跟踪模式可以直接接着用；
- 框得不好就 **`r`** 重置后重新框；
- 也可以按 **`s`** 手动再保存一次；
- **`q` / ESC** 退出。

### 2) 跟踪模式 — 自主巡线

```bash
# 默认读 params/HSV.txt
ros2 launch yahboomcar_linefollow line_track_launch.py
```

- 节点读取 `hsv_file`，在画面下方 `roi_top_ratio ~ roi_bottom_ratio` 的水平带内做掩膜，取最大轮廓的质心 `cx`；
- 归一化横向误差 `err = (cx - W/2)/(W/2)`；
- `SimplePID` 输出角速度，乘 `-1` 后塞进 `Twist.angular.z`（向线条方向打方向），`linear.x` 取常量；
- 发布到 `/cmd_vel`。
- 订阅 `/JoyState`：手柄接管时自动暂停并清零 PID。

无头机器人请把 `show_window: false`。

### 在线调参

```bash
# 实时改 PID / 速度 / ROI 带
ros2 param set /line_track kp 2.0
ros2 param set /line_track linear 0.2
ros2 param set /line_track switch false      # 临时停车，节点继续运行
ros2 param set /line_track roi_top_ratio 0.7
```

### 3) 巡线 + IMU 防撞（推荐）

只把 `yahboomcar_collision/collision_detector` 和 `line_track` 同时跑起来 **不够** ——
`line_track` 以 ~33 Hz 持续发布 `/cmd_vel`，碰撞节点发出的那一帧零速会被下一帧立刻覆盖，
车不会真正停。本包提供一个胶水 launch + 在 `line_track` 内部加了一层 **collision hold-off**：

- `line_track` 订阅 `collision_topic`（默认 `/collision_detector/collision`）；
- 一收到 `True` 脉冲就进入 `collision_pause_sec` 秒的 hold-off：
  期间持续发零 Twist + 清 PID 积分 + 在画面上叠加 `COLLISION HOLD`；
- 胶水 launch 顺便把 `collision_detector` 的 `stop_on_collision` 关掉，避免两边抢 `/cmd_vel`。

```bash
# 默认直接读 params/HSV.txt，标定过一次后即可一键运行
ros2 launch yahboomcar_linefollow linefollow_safe_launch.py \
    linear:=0.15 \
    accel_threshold:=12.0 \
    collision_pause_sec:=2.0
```

常用 launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `linear` | `0.15` | 巡线前进速度 (m/s) |
| `collision_pause_sec` | `2.0` | 撞到后暂停时长（秒）|
| `imu_topic` | `imu/data_raw` | IMU 话题 |
| `accel_threshold` | `12.0` | 加速度冲击阈值 (m/s²)，默认偏高减少误判 |
| `gyro_threshold` | `6.0` | 角速度冲击阈值 (rad/s) |
| `use_gyro` | `false` | 启用加速度+角速度双判据 |
| `min_trigger_samples` | `2` | 连续帧去毛刺 |
| `cooldown_sec` | `1.5` | 碰撞器自身去重冷却 |

> 想把 IMU 防撞接到其它自主节点，复刻 `line_track` 里的两段代码即可：
> 订阅 `Bool` 的碰撞话题、在 timer 里检查 `now < hold_until`，hold 期间发零 Twist。

## 话题

| 方向 | 话题 | 类型 | 节点 |
|---|---|---|---|
| pub | `/cmd_vel` | `geometry_msgs/Twist` | `line_track` |
| sub | `/JoyState` | `std_msgs/Bool` | `line_track` |
| pub | `~/status` | `std_msgs/String` | `line_detect` |

## 设计要点

- **ROI 学习而不是猜阈值**：用户直接在画面上指线，不需要先验颜色知识，对不同颜色 / 灯光环境都能用。
- **掩膜带 ROI**：跟踪时只看画面下方的一条带，避开远处干扰与水平线，质心更稳。
- **归一化误差**：PID 输入与图像分辨率解耦，调好的 gain 换分辨率也基本能用。
- **detect / track 两节点同盘 HSV 文件**：标定与运行解耦，标定一次可重复跑。
