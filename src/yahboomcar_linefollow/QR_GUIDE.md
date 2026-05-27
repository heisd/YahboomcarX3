# 二维码岔路口优先级（QR Code）工作原理与配置指南

本文档说明 `yahboomcar_linefollow` 包里**二维码识别**功能的工作流程、相关代码、
以及**如何设置优先级**。配套代码：

| 文件 | 作用 |
|---|---|
| `yahboomcar_linefollow/qr_common.py` | 二维码解码 + JSON 动作表解析（核心工具） |
| `yahboomcar_linefollow/line_track.py` | 巡线主节点，内置二维码优先级状态机 |
| `yahboomcar_linefollow/qr_check.py` | 独立诊断节点：贴码前验证每个码命中哪个动作 |
| `params/qr_actions.json` | 二维码内容 → 岔路动作 的映射表 |
| `launch/qr_linefollow_launch.py` | 巡线 + 二维码 + IMU 防撞 一键启动 |
| `test/test_qr_state_machine.py` | 不依赖摄像头/ROS 的状态机闭环测试 |

---

## 一、整体工作流程

二维码识别并**不是单独的节点**，而是"顺手"集成在巡线节点 `line_track` 里：
每隔 N 帧在同一张摄像头画面里扫一次二维码，命中后中断 PID 巡线、执行一个岔路口
动作，做完再恢复巡线。

数据流如下：

```
摄像头一帧画面 (BGR)
   │
   ▼
QRReader.detect()            ← qr_common.py，封装 cv2.QRCodeDetector
   │  解码出文本 payload（例如 "FORK_LEFT" 或 内联 JSON）
   ▼
resolve_action(payload, 表)  ← qr_common.py，查 qr_actions.json
   │  得到标准化动作 dict，例如 {"type":"left","turn_speed":0.6,...}
   ▼
_start_qr_maneuver()         ← line_track.py，把动作"锁存"为当前机动
   │
   ▼
_qr_maneuver_twist()         ← 每帧产生 Twist；左右转/直行是开环定时
   │
   ▼
发布到 /cmd_vel              ← 机动期间巡线 PID 被旁路
   │  动作结束 → 清状态 → 发布 ~/qr = "resume" → 恢复巡线
```

### 1. 解码（`QRReader`，qr_common.py:34）

- 用的是 OpenCV 自带的 `cv2.QRCodeDetector`，**不依赖** `zbar` / `pyzbar` 系统库。
- 如果当前 OpenCV 没编译二维码支持，构造函数把 `self._det` 置空，`detect()` 永远
  返回 `(None, None)`，整条流水线**优雅降级**（巡线照常跑，只是没有二维码功能），
  不会崩溃。
- `detect()` 返回 `(payload, points)`：`payload` 是解码文本，`points` 是四个角点
  （用于画框）。

### 2. 解析动作（`resolve_action`，qr_common.py:105）

二维码内容支持**两种写法**，同一套代码都能吃：

1. **纯字符串键**：内容是 `FORK_LEFT` 这种 key → 去 `qr_actions.json` 的 `actions`
   里查表。
2. **内联 JSON**：内容直接是 `{"type":"right","turn_time":2.0}` → 无需查表，
   直接当动作用（以 `{` 开头时走这条路）。

解析时会把动作字段合并到 `defaults` 默认值之上（`_normalize`，qr_common.py:89），
并校验 `type` 必须是合法类型，否则返回 `None`（不触发任何动作）。

### 3. 执行机动（状态机，line_track.py:238 起）

命中动作后，`_start_qr_maneuver()` 把动作锁存成"当前机动"，之后每帧由
`_qr_maneuver_twist()` 产生速度指令：

| `type` | 含义 | 行为 | 关键字段 |
|---|---|---|---|
| `left` / `right` | 岔路口左/右转 | 开环定时转：`linear.x=cross_speed`，`angular.z=±turn_speed`，持续 `turn_time` 秒 | `turn_speed`(rad/s) `turn_time`(s) `cross_speed`(m/s) |
| `straight` | 直行穿过岔路 | 短暂忽略线直行 `cross_time` 秒 | `cross_speed`(m/s) `cross_time`(s) |
| `station` | 站点：停车 | 停车 `hold_time` 秒；`hold_time<=0` 表示**一直停（锁存）**直到 `switch` 切一下复位 | `hold_time`(s) |
| `stop` | 完全停车/锁定 | 同 station 锁存逻辑 | `hold_time`(s) |

> **岔路口语义**：一条岔路通站点（到站 `station` 就停），另一条是继续走的主路
> （`left`/`right`/`straight`）。每个二维码背后的 JSON 决定这个岔路口属于哪种情况。

### 4. 防抖与重触发

- **`qr_check_every`**：每 N 帧才检测一次二维码，降低对巡线帧率的拖累（默认 3）。
- **`qr_cooldown_sec`**：同一个 payload 在这段时间内不重复触发（默认 4s），避免
  车还压在码上时反复触发。
- **定时机动进行中跳过检测**（line_track.py:297）：左右转/直行进行中不再扫码，
  防止同一个码在机动途中被重复触发；而锁存停车（station/stop）**仍继续扫码**，
  这样可以用一个新码把车从站点"唤醒"重新出发（见测试 `test_new_qr_unlocks_latched_station`）。

---

## 二、优先级（重点：如何设置优先级）

### 1. 内置的固定优先级

`line_track` 内部有一套**写死的安全优先级**，从高到低：

```
手柄接管 (/JoyState)  >  碰撞安全停  >  二维码岔路动作  >  普通巡线
        最高                                              最低
```

即：**二维码会压过普通巡线，但不会盖过安全停车与手动接管。**

对应代码在 `line_track.py` 的 `_tick()` 里：

```python
manual_or_safety = self.joy_active or in_collision_hold     # 手柄 或 碰撞
...
# 手柄/碰撞会取消正在进行的二维码机动
if manual_or_safety and self._qr_state is not None:
    self._qr_state = None
# 只有在没有手柄/碰撞时才检测二维码
if enable_qr and self._qr is not None and not manual_or_safety:
    self._maybe_detect_qr(frame, now)
# 二维码机动拥有 /cmd_vel；此帧巡线 PID 被旁路
if self._qr_state is not None and not manual_or_safety:
    twist, still = self._qr_maneuver_twist()
elif centroid is not None:
    ... 普通巡线 PID ...
```

这套安全优先级是**设计上固定的**，不需要也不应该用参数改动。

### 2. 你可以调的"优先级"开关与参数

虽然安全优先级是固定的，但你能通过参数控制二维码功能**是否参与**、以及
**每个二维码的具体动作和强度**：

#### (a) 开/关二维码优先级（运行时即时生效）

```bash
# 关掉二维码 → 退回纯巡线
ros2 param set /line_track enable_qr false
# 重新打开
ros2 param set /line_track enable_qr true
```

`enable_qr` 在每帧都会重新读取（line_track.py:385），所以 `ros2 param set`
**立即生效，无需重启节点**。

#### (b) 通过 `qr_actions.json` 设置"每个码的优先动作"

这是设置二维码行为优先级的**核心方式**——决定每个二维码命中后车做什么。
编辑 `params/qr_actions.json`：

```json
{
  "defaults": {
    "turn_speed": 0.6,
    "turn_time": 1.2,
    "cross_speed": 0.12,
    "cross_time": 0.6,
    "hold_time": 0.0
  },
  "actions": {
    "FORK_LEFT":     {"type": "left"},
    "FORK_RIGHT":    {"type": "right"},
    "FORK_STRAIGHT": {"type": "straight"},
    "STATION_A":     {"type": "station"},
    "STATION_B":     {"type": "station", "hold_time": 5.0},
    "STOP":          {"type": "stop"}
  }
}
```

- `defaults`：所有动作的默认参数；某个动作里写了同名字段就覆盖默认值
  （例如 `STATION_B` 把 `hold_time` 覆盖成 5 秒）。
- `actions`：二维码文本 → 动作。键名就是你打印在二维码里的文本。
- 站点优先级例子：
  - `STATION_A`（`hold_time` 用默认 0）→ **永久停车锁存**，必须 `switch` 切一下才走；
  - `STATION_B`（`hold_time=5.0`）→ 停 5 秒后**自动**恢复巡线。

#### (c) 检测频率与去抖

```bash
# 每隔几帧扫一次码（值越小越灵敏、越占 CPU）
ros2 param set /line_track qr_check_every 3
# 同一个码多久内不重复触发
ros2 param set /line_track qr_cooldown_sec 4.0
```

#### (d) 锁存停车的"复位"（最高优先级的人工干预）

当车被 `station`/`stop`（`hold_time<=0`）**永久锁存**停住时，把 `switch`
切一下即可解锁继续：

```bash
ros2 param set /line_track switch false   # 解除锁存（车停）
ros2 param set /line_track switch true    # 重新使能巡线
```

---

## 三、如何运行

### 1. 先标定巡线 HSV（只需一次）

```bash
ros2 launch yahboomcar_linefollow line_detect_launch.py
# 鼠标在线条上框选 → 自动学习并写回 params/HSV.txt
```

### 2. 贴码前用诊断节点验证每个码（强烈建议）

`qr_check` 单独打开摄像头，实时解码并显示命中的动作类型，方便你确认每张二维码
贴对了。注意：**它和 `line_track` 抢同一个摄像头，不要同时跑。**

```bash
ros2 run yahboomcar_linefollow qr_check
```

它发布两个话题：
- `~/payload`（`std_msgs/String`）：解码到的二维码文本；
- `~/action`（`std_msgs/String`）：命中的动作类型（没命中为空）。

### 3. 巡线 + 二维码优先级 + IMU 防撞 一键启动

```bash
ros2 launch yahboomcar_linefollow qr_linefollow_launch.py \
    linear:=0.15 qr_check_every:=3
```

常用 launch 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `linear` | `0.15` | 巡线前进速度 (m/s) |
| `enable_qr` | `true` | 是否启用二维码优先级 |
| `qr_actions_file` | 包内 `params/qr_actions.json` | 动作映射表路径 |
| `qr_check_every` | `3` | 每 N 帧检测一次二维码 |
| `qr_cooldown_sec` | `4.0` | 同一个码的去重冷却时间 |
| `collision_pause_sec` | `2.0` | 碰撞后暂停时长 |

---

## 四、相关话题

| 方向 | 话题 | 类型 | 节点 | 说明 |
|---|---|---|---|---|
| pub | `~/qr` | `std_msgs/String` | `line_track` | 命中动作 `payload:type`，或机动结束的 `resume` |
| pub | `~/payload` | `std_msgs/String` | `qr_check` | 解码到的二维码文本 |
| pub | `~/action` | `std_msgs/String` | `qr_check` | 命中的动作类型 |
| pub | `/cmd_vel` | `geometry_msgs/Twist` | `line_track` | 速度指令（巡线/二维码机动共用） |
| sub | `/JoyState` | `std_msgs/Bool` | `line_track` | 手柄接管（最高优先级） |
| sub | `/collision_detector/collision` | `std_msgs/Bool` | `line_track` | 碰撞标志（第二优先级） |

---

## 五、测试

不依赖摄像头/ROS/显示器即可跑完整状态机测试：

```bash
pytest src/yahboomcar_linefollow/test/test_qr_state_machine.py -v
```

覆盖范围：`resolve_action` 解析（字符串键 / 内联 JSON / 默认值合并 / 非法类型）、
`load_actions` 容错（缺文件 / 坏 JSON）、以及左右转定时到期、站点锁存与定时恢复、
冷却去重、新码唤醒锁存站点等状态转移。
