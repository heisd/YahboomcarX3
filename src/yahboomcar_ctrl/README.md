# yahboomcar_ctrl

Yahboomcar 的 **遥控 (control)** 包，提供键盘与手柄两种方式向 `/cmd_vel` 发布速度指令。

## 功能概述

- `yahboom_keyboard`：终端键盘遥控节点，使用 `u i o / j k l / m , .` 等按键控制小车移动，`q/z`、`w/x`、`e/c` 分别调节总速、线速、角速。
- `yahboom_joy_X3`：X3 麦克纳姆轮车型的手柄遥控节点，订阅 `/joy` 并发布 `Twist`，同时发布 `/JoyState` 用于在巡逻 / 避障节点中切换自动 / 手动模式。
- `yahboom_joy_R2`：阿克曼车型 R2 的手柄遥控节点。

## 目录结构

```
yahboomcar_ctrl/
├── yahboomcar_ctrl/       # Python 节点
│   ├── yahboom_keyboard.py
│   ├── yahboom_joy_X3.py
│   └── yahboom_joy_R2.py
├── launch/                # 启动文件
└── resource/
```

## 依赖

- `rclpy`, `geometry_msgs`, `std_msgs`, `sensor_msgs`（手柄）
- 系统 `joy` 节点（手柄输入）

## 使用

```bash
# 键盘遥控
ros2 run yahboomcar_ctrl yahboom_keyboard

# 手柄遥控
ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py
```
