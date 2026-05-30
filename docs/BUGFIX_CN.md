# YahboomcarX3 代码库 Bug 修复文档

> 修复日期:2026-05-30
> 分支:`claude/nav2-component-review-znIsb`
> 范围:对整个 `src/` 下的 yahboomcar_* 功能包做了一次系统性 bug 排查(第三方 `ldlidar_stl_ros2` 与 C++ 包不在本次范围内)。

本次先用 `py_compile` + `pyflakes` 全量静态扫描,再人工逐个核实,最终修复了 **10 处真实 bug**(分布在 7 个文件)。每条都已通过编译验证。下面按"会直接崩溃/失效"到"行为异常"排序。

---

## 一、会导致节点崩溃 / 功能完全失效(高危)

### 1. `yahboomcar_bringup/patrol_4ROS.py` — `RAD2DEG` 未定义
- **现象**:`LaserScanCallback` 第 315 行 `angle = (... ) * RAD2DEG`,但全文件从未定义 `RAD2DEG`,每次激光回调都抛 `NameError`,巡逻避障完全不工作。
- **原因**:其它同类文件(`patrol_a1_X3.py`)在顶部有 `RAD2DEG = 180 / math.pi`,这个文件漏写了。
- **修复**:在导入区补上 `RAD2DEG = 180 / math.pi`。

### 2. `yahboomcar_bringup/calibrate_angular_X3.py` — 把发布器当函数调用
- **现象**:退出时 `class_calibrateangular.cmd_vel().publish(Twist())`,但 `cmd_vel` 是一个 publisher 对象(`self.cmd_vel = self.create_publisher(...)`),不是函数。调用 `cmd_vel()` 会抛 `TypeError`,导致退出时的"停车"指令发不出去,小车可能继续乱转。
- **修复**:`cmd_vel().publish(...)` → `cmd_vel.publish(...)`(与 `calibrate_linear_X3.py` 的正确写法一致)。

### 3. `yahboomcar_ctrl/yahboom_joy_X3.py` — PC 手柄模式发布裸 `int`/`bool`
- **现象**:`user_pc` 分支里
  - `self.pub_RGBLight.publish(self.RGBLight_index)` 把裸 `int` 发给类型为 `Int32` 的发布器;
  - `self.pub_Buzzer.publish(self.Buzzer_active)` 把裸 `bool` 发给类型为 `Bool` 的发布器。
  
  rclpy 要求发布的是消息对象,这两处会在运行时报错,PC 手柄下的 RGB 灯/蜂鸣器一按就崩。(Jetson 分支 `user_jetson` 是正确地包了 `Int32()`/`Bool()` 的。)
- **修复**:构造消息再发布:
  ```python
  RGBLight_ctrl = Int32(); RGBLight_ctrl.data = self.RGBLight_index
  self.pub_RGBLight.publish(RGBLight_ctrl)
  Buzzer_ctrl = Bool(); Buzzer_ctrl.data = self.Buzzer_active
  self.pub_Buzzer.publish(Buzzer_ctrl)
  ```

### 4. `yahboomcar_ctrl/yahboom_joy_R2.py` — 同 X3 的裸类型发布问题
- **现象**:`user_pc` 分支同样发布裸 `int`/`bool` 给 `Int32`/`Bool` 发布器。
- **修复**:同上,包成 `Int32()`/`Bool()` 再发布。

### 5. `yahboomcar_laser/laser_Avoidance_a1_X3.py` — 每帧激光都重开串口
- **现象**:`registerScan` 回调里 `bot = Rosmaster(com="/dev/ttyUSB1")`,**每来一帧激光就新建一次驱动板串口连接**,既不关闭也根本没用到 `bot`。会迅速耗尽/抢占串口,而该串口已被 bringup 的驱动节点占用,导致冲突、卡死。
- **修复**:删除这行无用且有害的代码。

---

## 二、控制逻辑错误(中危)

### 6. `yahboomcar_ctrl/yahboom_joy_R2.py` — Jetson 手柄无法转向
- **现象**:`user_jetson` 里算好了 `angular_speed`,但 `#twist.angular.z = angular_speed` 被注释掉了,Jetson 手柄模式下小车永远不会转向。
- **修复**:取消注释,恢复 `twist.angular.z = angular_speed`。

### 7. `yahboomcar_ctrl/yahboom_joy_R2.py` — 横移与转向共用同一摇杆轴
- **现象**:`user_jetson` 里 `ylinear_speed` 和 `angular_speed` 都用 `joy_data.axes[2]`,横移(Y)和旋转无法独立控制。
- **修复**:Y 轴改用 `axes[0]`(与本文件 `user_pc` 以及 `yahboom_joy_X3.py` 的映射保持一致)。

### 8. `yahboomcar_bringup/patrol.py` — 退出时蜂鸣器没复位
- **现象**:`exit_pro` 里本应发 `/beep data:0` 关蜂鸣器,但 `cmd = cmd1 + cmd2`(复用了 cmd_vel 的字符串),`cmd3 + cmd4`(beep 复位命令)根本没被执行,退出后蜂鸣器一直响。
- **修复**:`cmd = cmd1 + cmd2` → `cmd = cmd3 + cmd4`。

### 9. `yahboomcar_ctrl/yahboom_keyboard.py` — 切换 X/Y 后旧速度残留
- **现象**:`xspeed_switch` 在 X/Y 间切换时,每轮只更新 `twist.linear.x` 或 `twist.linear.y` 其中一个,另一个保留上次的非零值;切到 Y 后 X 方向仍以旧速度运动。
- **修复**:每轮赋值前先把 `twist.linear.x` / `twist.linear.y` 都清零。

### 10. `yahboomcar_astra/colorTracker.py` — 深度距离被多加了 1000
- **现象**:`depth_img_Callback` 里累加器初始化成 `distance_ = 1000.0`,然后把有效深度点加进去再除以点数,算出来的物体距离被人为抬高了 `1000 / 点数`(5 个点全有效时多 200mm)。这个错误距离喂进 PID,导致跟随/停距不对。
- **修复**:初始值改为 `distance_ = 0.0`(全无效点的情况已有 `num_depth_points == 0 → distance_ = self.minDist` 兜底,改 0 安全)。

---

## 三、Launch 文件 Bug(中危)

### 11. `yahboomcar_description/description_X3_multi_robot1.launch.py` 与 `..._multi_robot2.launch.py` — 参数声明顺序错误
- **现象**:`robot_name_arg`(`DeclareLaunchArgument('robot_name', ...)`)被放在返回列表的**最后**,排在使用 `LaunchConfiguration('robot_name')` 作为 namespace 的各节点之后。launch 按列表顺序访问 entity,节点先被访问时该参数尚未声明,若命令行没显式传 `robot_name:=...`,namespace 取不到默认值会报错/解析错误。
- **修复**:把 `robot_name_arg` 移到列表最前面。

---

## 四、排查到但**未自动修改**的点(需你决定)

这些要么有歧义、要么属于打包/版本相关决策,贸然改可能改变现网行为,故仅列出建议:

1. **`laser_Avoidance_a1_X3.py` 内层避障分支自相矛盾**(约 106、119 行):如内层 `if self.Left_warning > 10 and self.Right_warning <= 10:` 处于外层已保证 `Right_warning > 10` 的分支里,条件永真假矛盾,属死代码。原意不明,改动会影响避障行为,建议你确认意图后再调。

2. **`laser_Tracker_a1_X3.py` / `laser_Warning_a1_X3.py` 把 numpy 标量赋给 Twist 字段**:`minDist` 来自 `np.array(scan_data.ranges)`,PID 输出可能是 numpy 浮点。在部分 numpy 版本(尤其 float32 / numpy 2.0 NEP50)下,赋给 `Twist.linear.x` 会触发 rclpy 类型断言。由于整套代码风格统一且出厂可运行,本次未改;若你遇到该断言,用 `float(...)` 包一层即可。

3. **`yahboomcar_astra/launch/colorTracker_X3.launch.py`** 用了 Foxy 时代的 `node_executable=` / `node_name=`,在 Humble+ 已移除,会导致 launch 报 `TypeError`。如目标是 Humble,应改为 `executable=` / `name=`。

4. **`yahboomcar_description/launch/display_R2.launch.py`** 引用的 `urdf/yahboomcar_R2.urdf.xacro` 不在被安装的 `urdf/` 目录(只存在于未安装的嵌套目录),安装后会找不到文件。需要在 `setup.py` 里补 data_files 或移动该 xacro,属打包决策。

5. **`yahboomcar_bringup/setup.py`** 注册了 `patrol_a1_R2` 入口,但 `patrol_a1_R2.py` 源文件不存在(只有 `.ipynb_checkpoints` 副本),该入口运行会 import 失败。

---

## 验证

所有 10 处修改涉及的文件均已通过 `python3 -m py_compile` 编译检查;`yahboom_joy_X3.py` / `yahboom_joy_R2.py` 已确认顶部存在 `from std_msgs.msg import Int32, Bool`,新构造的消息对象可用。
