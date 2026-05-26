# yahboomcar_gazebo

Yahboomcar X3（麦克纳姆轮）在 **Gazebo Classic** 下的仿真包，包含：

- 用 Gazebo 自带模型（`sun` / `ground_plane`）+ 内置几何体（box/cylinder）搭建的演示世界；
- 带 Gazebo 插件的 X3 仿真模型（全向底盘 / 激光 / IMU / 相机）；
- `scene_switcher` 节点：运行时**动态切换场景**（增删障碍物组），无需重启 Gazebo。

> 适用环境：ROS 2 Humble + Gazebo Classic 11（`gazebo_ros` / `gazebo_ros_pkgs`）。
>
> 本包已在 `ros:humble` + `gazebo-ros-pkgs` 容器中 headless 跑通：世界加载、机器人落地、
> `/odom` `/scan` `/imu/data` 正常、全向 `/cmd_vel` 运动、以及场景热切换（spawn/delete）。
> （headless 无 GPU 时相机不出图，属环境限制，非代码问题。）

## 目录结构

```
yahboomcar_gazebo/
├── launch/
│   ├── gazebo_world.launch.py   # 一键启动：Gazebo + 世界 + 机器人 + 场景切换
│   └── spawn_robot.launch.py    # 仅把机器人 spawn 进已运行的 Gazebo
├── worlds/
│   ├── yahboom_room.world        # 默认世界：6x6 墙体 + 桌子 + 立柱（内置几何体，离线可用）
│   └── yahboom_base.world        # 极简画布：仅地面 + 光源，配合场景切换使用
├── urdf/
│   └── yahboomcar_X3_gazebo.urdf.xacro   # X3 + Gazebo 插件（复用 description 的网格）
├── config/
│   └── scenes.yaml               # 各场景的模型与位姿定义
└── yahboomcar_gazebo/
    └── scene_switcher.py         # 场景切换节点
```

## 编译

```bash
cd ~/YahboomcarX3
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select yahboomcar_gazebo --symlink-install
source install/setup.bash
source /usr/share/gazebo/setup.sh   # 让 Gazebo 找到自带模型（sun / ground_plane）
```

> **关于模型**：世界与默认场景全部用 Gazebo 自带模型 + 内置几何体（box/cylinder）搭建，
> 无需联网下载，离线即可运行。原本常用的 `cafe_table` / `bookshelf` / `grey_wall` 等模型
> 依赖已停服的旧模型库（`models.gazebosim.org`），在原版 Gazebo 11 上无法自动下载，因此
> 不再默认使用；若你本地已缓存这些模型，可在 `scenes.yaml` 里用 `model:` 字段引用。
>
> `gazebo_world.launch.py` 会自动把工作空间的 share 目录与 Gazebo 自带模型目录加入
> `GAZEBO_MODEL_PATH`（否则机器人网格解析失败、会穿过地面下坠）。

## 快速开始

```bash
# 启动默认世界（yahboom_room）并把 X3 放进去
ros2 launch yahboomcar_gazebo gazebo_world.launch.py

# 启动时直接加载一个动态场景（在极简世界上叠加障碍物）
ros2 launch yahboomcar_gazebo gazebo_world.launch.py \
    world:=$(ros2 pkg prefix yahboomcar_gazebo)/share/yahboomcar_gazebo/worlds/yahboom_base.world \
    scene:=warehouse

# 键盘遥控（已有功能包）
ros2 run yahboomcar_ctrl yahboom_keyboard
```

机器人订阅 `/cmd_vel`，发布 `/odom`、`/scan`、`/imu/data`、`/camera/image_raw`，
并广播 `odom → base_footprint` TF。

## 场景切换

`scene_switcher` 通过 Gazebo 的 `/spawn_entity`、`/delete_entity` 服务工作：
切换时先删除上一组模型，再 spawn 新一组。

```bash
# 切到指定场景（名字来自 scenes.yaml）
ros2 topic pub --once /scene_cmd std_msgs/msg/String "{data: office}"
ros2 topic pub --once /scene_cmd std_msgs/msg/String "{data: warehouse}"

# 循环切到下一个场景
ros2 service call /scene/next  std_srvs/srv/Trigger

# 清空当前场景 / 重新加载当前场景
ros2 service call /scene/clear  std_srvs/srv/Trigger
ros2 service call /scene/reload std_srvs/srv/Trigger
```

`/scene_cmd`（`std_msgs/String`）支持的取值：

| data | 行为 |
|---|---|
| `<场景名>` | 切换到该场景（office / warehouse / obstacles / parking …） |
| `next` | 切换到下一个场景 |
| `clear` 或 `empty` | 删除所有已生成的模型 |
| `list` | 在日志中列出所有可用场景 |

## 新增 / 修改场景

编辑 `config/scenes.yaml`，每个场景是一组物体。每个物体支持以下几种写法：

```yaml
scenes:
  my_scene:
    - {box:      [0.5, 0.5, 0.5],  pose: [2.0, 0.0, 0.25, 0, 0, 0], color: [0.8, 0.7, 0.4]}
    - {cylinder: [0.2, 0.8],       pose: [1.5, 0.6, 0.40, 0, 0, 0], color: [0.9, 0.4, 0.0]}
    - {sphere:   [0.3],            pose: [1.0, 1.0, 0.30, 0, 0, 0]}
    - {model:    cafe_table,       pose: [2.5, 0.0, 0.00, 0, 0, 0]}   # 需本地有该模型
```

- `box` = `[sx, sy, sz]`，`cylinder` = `[半径, 长度]`，`sphere` = `[半径]`，`model` = Gazebo 模型名（`model://<name>`）。
- `pose = [x, y, z, roll, pitch, yaw]`（米 / 弧度，世界坐标系）。box/cylinder 贴地时 `z` 取高度的一半。
- `color = [r, g, b]`（0~1，可省略，默认灰色；仅对 box/cylinder/sphere 生效）。

几何体默认以 `<static>true</static>` 生成（机器人可碰撞但不会推动），且无需联网。

## 节点参数（scene_switcher）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenes_file` | `""` | scenes.yaml 的绝对路径（launch 已自动传入） |
| `default_scene` | `""` | 启动后自动加载的场景（空 = 不加载） |
| `spawn_timeout` | `10.0` | 每次 spawn/delete 服务调用的超时时间（秒） |
