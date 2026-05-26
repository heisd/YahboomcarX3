# yahboomcar_gazebo

Yahboomcar X3（麦克纳姆轮）在 **Gazebo Classic** 下的仿真包，包含：

- 用 Gazebo **内置模型**（model database / Fuel）搭建的演示世界；
- 带 Gazebo 插件的 X3 仿真模型（全向底盘 / 激光 / IMU / 相机）；
- `scene_switcher` 节点：运行时**动态切换场景**（增删障碍物组），无需重启 Gazebo。

> 适用环境：ROS 2 Humble + Gazebo Classic 11（`gazebo_ros` / `gazebo_ros_pkgs`）。

## 目录结构

```
yahboomcar_gazebo/
├── launch/
│   ├── gazebo_world.launch.py   # 一键启动：Gazebo + 世界 + 机器人 + 场景切换
│   └── spawn_robot.launch.py    # 仅把机器人 spawn 进已运行的 Gazebo
├── worlds/
│   ├── yahboom_room.world        # 默认世界：墙体 + 桌子 + 书架 + 锥桶（全部内置模型）
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
```

> 世界与场景使用 Gazebo 内置模型（`cafe_table`、`bookshelf`、`construction_cone`、
> `construction_barrel`、`cardboard_box`、`grey_wall`、`jersey_barrier` 等）。首次使用时
> Gazebo 会从模型库自动下载；离线环境请确保这些模型已存在于 `~/.gazebo/models` 或
> `GAZEBO_MODEL_PATH` 中。

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

编辑 `config/scenes.yaml`，每个场景是一组内置模型及其位姿：

```yaml
scenes:
  my_scene:
    - {model: cafe_table,       pose: [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
    - {model: construction_cone, pose: [1.5, 0.6, 0.0, 0.0, 0.0, 0.0]}
```

`pose = [x, y, z, roll, pitch, yaw]`（单位：米 / 弧度，世界坐标系）。
`model` 为 Gazebo 模型库中的模型名（即 `model://<name>`）。

## 节点参数（scene_switcher）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenes_file` | `""` | scenes.yaml 的绝对路径（launch 已自动传入） |
| `default_scene` | `""` | 启动后自动加载的场景（空 = 不加载） |
| `spawn_timeout` | `10.0` | 每次 spawn/delete 服务调用的超时时间（秒） |
