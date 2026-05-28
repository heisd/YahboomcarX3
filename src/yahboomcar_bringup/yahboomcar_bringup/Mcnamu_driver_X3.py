#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock

#from dynamic_reconfigure.server import Server
car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}

# 低电量模式状态 / Low-battery mode states
BATTERY_NORMAL = 0    # 正常
BATTERY_LOW = 1       # 低电量 (蜂鸣器报警)
BATTERY_CRITICAL = 2  # 临界电量 (强制停车)
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		#初始化小车硬件
		self.car = Rosmaster(com="/dev/ttyUSB1")
		self.car.set_car_type(1)
		#get parameter获取参数
		self.declare_parameter('car_type', 'X3')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		#
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 5.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)

		 # 新增安全参数
		self.declare_parameter('safety_timeout', 0.5)  # 超时时间(秒)
		self.safety_timeout = self.get_parameter('safety_timeout').get_parameter_value().double_value
		self.declare_parameter('enable_safety', True)  # 是否启用安全保护
		self.enable_safety = self.get_parameter('enable_safety').get_parameter_value().bool_value
		self.last_cmd_time = self.get_clock().now()  # 最后收到命令的时间
		self.last_speed = [0.0, 0.0, 0.0]  # 最后发送的速度
		self.is_safety_stopped = False  # 是否因超时停止了

		# 低电量保护参数 (类似手机电量模式)
		# Low-battery protection parameters (inspired by phone battery saver)
		self.declare_parameter('enable_low_battery_protection', True)
		self.enable_low_battery_protection = self.get_parameter(
			'enable_low_battery_protection').get_parameter_value().bool_value
		# 电池电压标定 (X3 默认 3S 锂电: 9.0V 空 -> 12.6V 满，与仪表盘一致)
		self.declare_parameter('battery_voltage_full', 12.6)
		self.battery_voltage_full = self.get_parameter(
			'battery_voltage_full').get_parameter_value().double_value
		self.declare_parameter('battery_voltage_empty', 9.0)
		self.battery_voltage_empty = self.get_parameter(
			'battery_voltage_empty').get_parameter_value().double_value
		# 两档阈值 (%)：低电量蜂鸣器，临界电量停车
		self.declare_parameter('battery_warning_pct', 20.0)
		self.battery_warning_pct = self.get_parameter(
			'battery_warning_pct').get_parameter_value().double_value
		self.declare_parameter('battery_critical_pct', 5.0)
		self.battery_critical_pct = self.get_parameter(
			'battery_critical_pct').get_parameter_value().double_value
		# 滞回 (%): 高于阈值 +hysteresis 才能回到上一档，防止抖动
		self.declare_parameter('battery_hysteresis_pct', 2.0)
		self.battery_hysteresis_pct = self.get_parameter(
			'battery_hysteresis_pct').get_parameter_value().double_value
		# 蜂鸣周期 (秒)
		self.declare_parameter('battery_warning_beep_period', 5.0)
		self.battery_warning_beep_period = self.get_parameter(
			'battery_warning_beep_period').get_parameter_value().double_value
		self.declare_parameter('battery_critical_beep_period', 1.0)
		self.battery_critical_beep_period = self.get_parameter(
			'battery_critical_beep_period').get_parameter_value().double_value
		# 电压低通滤波 alpha (0..1, 越小越平滑)，N 次连续越线才切换状态
		self.declare_parameter('battery_filter_alpha', 0.2)
		self.battery_filter_alpha = self.get_parameter(
			'battery_filter_alpha').get_parameter_value().double_value
		self.declare_parameter('battery_debounce_samples', 5)
		self.battery_debounce_samples = self.get_parameter(
			'battery_debounce_samples').get_parameter_value().integer_value

		# 校验参数 (越界值会被夹紧或禁用保护)
		self._validate_battery_params()

		# 低电量内部状态
		self.battery_state = BATTERY_NORMAL
		self.battery_voltage_filtered = None
		self._battery_pending_state = BATTERY_NORMAL
		self._battery_pending_count = 0
		self._last_beep_time = self.get_clock().now()
		# 启动暖机：先收到 N 个有效电压再允许状态切换，避免上电瞬态误触发
		self._voltage_sample_count = 0
		self._warmup_samples = 10
		# 连续无效电压计数：传感器卡死时 fail-safe 进入 CRITICAL
		self._consecutive_invalid_voltage = 0
		self._invalid_voltage_threshold = 50  # 5s @ 10Hz
		# 用户通过 /Buzzer 持续按下蜂鸣器时让位，避免冲突
		self._user_buzzer_on = False
		# 由低电量发布过 safety_status=False 时记一笔，仅在我们置位时清除
		self._battery_safety_published = False

		#create subcriber 创建订阅者
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		#create publisher  创建发布者
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"imu/mag",100)
		self.safety_pub = self.create_publisher(Bool, "safety_status", 10) # 安全状态发布者
		self.battery_state_pub = self.create_publisher(Int32, "battery_state", 10) # 低电量状态发布者
		self.battery_pct_pub = self.create_publisher(Float32, "battery_pct", 10) # 滤波后的电量百分比

		#create timer 创建定时器
		self.timer = self.create_timer(0.1, self.pub_data)  # 10Hz发布数据
		self.safety_timer = self.create_timer(0.05, self.safety_check)  # 20Hz安全检查
		self.battery_beep_timer = self.create_timer(0.2, self._battery_beep_tick)  # 5Hz蜂鸣调度

		#create and init variable  创建并初始化变量	
		self.edition = Float32()
		self.edition.data = 1.0
		self.car.create_receive_threading()

		# 初始安全状态
		self.publish_safety_status(True, "Initialized")

		# 发布安全状态
	def publish_safety_status(self, is_normal, reason=""):
		msg = Bool()
		msg.data = is_normal
		self.safety_pub.publish(msg)
		if not is_normal:
			self.get_logger().warn(f"Safety activated: {reason}")
	#callback function
	def cmd_vel_callback(self,msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # 临界电量：拒绝下发任何运动指令，确保小车停止
        # Critical battery: refuse motion commands and hold the car stopped
        # 不更新 last_cmd_time，让 safety_check 的超时仍然可以观察到"无有效指令"
		if (self.enable_low_battery_protection
				and self.battery_state == BATTERY_CRITICAL):
			try:
				self.car.set_car_motion(0.0, 0.0, 0.0)
			except Exception as e:
				self.get_logger().warn(f"set_car_motion stop failed: {e}")
			self.last_speed = [0.0, 0.0, 0.0]
			return
		self.last_cmd_time = self.get_clock().now()
        # 下发线速度和角速度
        # Issue linear vel and angular vel
		vx = msg.linear.x*1.0
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y*1.0
		angular = msg.angular.z*1.0     # wait for chang
        # 如果之前因安全原因停止，现在恢复正常
		if self.is_safety_stopped:
			self.is_safety_stopped = False
			self.publish_safety_status(True, "Command received after timeout")
		# 记录最后发送的速度
		self.last_speed = [vx, vy, angular]
		self.car.set_car_motion(vx, vy, angular)
		'''print("cmd_vx: ",vx)
		print("cmd_vy: ",vy)
		print("cmd_angular: ",angular)'''
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)
	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)
	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		# 记录用户的蜂鸣意图，避免低电量周期性鸣笛覆盖"持续打开/关闭"
		self._user_buzzer_on = bool(msg.data)
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	# 安全检查函数
	def safety_check(self):
		"""安全检查定时回调函数"""
		if not self.enable_safety:
			return
		current_time = self.get_clock().now()
		time_diff = (current_time - self.last_cmd_time).nanoseconds / 1e9  # 转换为秒
        # 检查是否超时
		if time_diff > self.safety_timeout:
            # 如果当前没有停止，则执行停止
			if not self.is_safety_stopped and any(abs(s) > 0.01 for s in self.last_speed):
				self.get_logger().warn(f"Safety timeout! Stopping car. Time since last cmd: {time_diff:.2f}s")
				self.car.set_car_motion(0.0, 0.0, 0.0)
				self.last_speed = [0.0, 0.0, 0.0]
				self.is_safety_stopped = True
				self.publish_safety_status(False, f"Timeout ({time_diff:.1f}s)")
		else:
            # 如果之前停止了但现在有命令，恢复状态
			if self.is_safety_stopped:
				self.is_safety_stopped = False
				self.publish_safety_status(True, "Normal operation")
	#pub data
	def pub_data(self):
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
		
		#print ("mag: ",self.car.get_magnetometer_data())		
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()
		gx, gy, gz = self.car.get_gyroscope_data()
		mx, my, mz = self.car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()
		'''print("vx: ",vx)
		print("vy: ",vy)
		print("angular: ",angular)'''
		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0
		
		# 将小车当前的线速度和角速度发布出去
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0
		twist.angular.z = angular*1.0    
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		self.imuPublisher.publish(imu)
		self.magPublisher.publish(mag)
		self.volPublisher.publish(battery)
		self.EdiPublisher.publish(edition)
		# 更新并发布低电量状态
		# Update and publish low-battery state
		if self.enable_low_battery_protection:
			self._update_battery_state(battery.data)
			state_msg = Int32()
			state_msg.data = int(self.battery_state)
			self.battery_state_pub.publish(state_msg)
			# 发布滤波后的电量百分比 (来源唯一，仪表盘据此渲染)
			pct_msg = Float32()
			source = (self.battery_voltage_filtered
					  if self.battery_voltage_filtered is not None
					  else battery.data)
			pct_msg.data = float(self._voltage_to_pct(source))
			self.battery_pct_pub.publish(pct_msg)

	# ---- 低电量保护辅助函数 / low-battery helpers --------------------------
	def _validate_battery_params(self):
		"""启动时夹紧/否决越界参数；致命冲突 (e.g. full <= empty) 直接关掉保护。"""
		if not self.enable_low_battery_protection:
			return
		log = self.get_logger()
		if self.battery_voltage_full <= self.battery_voltage_empty:
			log.error(
				f"battery_voltage_full ({self.battery_voltage_full}) must be > "
				f"battery_voltage_empty ({self.battery_voltage_empty}); "
				f"disabling low-battery protection")
			self.enable_low_battery_protection = False
			return
		if self.battery_critical_pct >= self.battery_warning_pct:
			log.warn(
				f"battery_critical_pct ({self.battery_critical_pct}) should be "
				f"< battery_warning_pct ({self.battery_warning_pct})")
		if not (0.0 < self.battery_filter_alpha <= 1.0):
			old = self.battery_filter_alpha
			self.battery_filter_alpha = max(0.01, min(1.0, old))
			log.warn(f"battery_filter_alpha out of (0,1]; clamped "
					 f"{old} -> {self.battery_filter_alpha}")
		if self.battery_debounce_samples < 1:
			old = self.battery_debounce_samples
			self.battery_debounce_samples = 1
			log.warn(f"battery_debounce_samples must be >= 1; "
					 f"clamped {old} -> 1")
		if self.battery_hysteresis_pct < 0.0:
			old = self.battery_hysteresis_pct
			self.battery_hysteresis_pct = 0.0
			log.warn(f"battery_hysteresis_pct must be >= 0; "
					 f"clamped {old} -> 0.0")

	def _voltage_to_pct(self, v):
		rng = max(0.001, self.battery_voltage_full - self.battery_voltage_empty)
		return max(0.0, min(100.0, (v - self.battery_voltage_empty) / rng * 100.0))

	def _update_battery_state(self, raw_voltage):
		"""低通滤波 + 滞回 + 防抖：根据电压更新 self.battery_state。"""
		# 上电瞬间或传感器卡死时电压可能为 0；连续 N 次无效 fail-safe 进入 CRITICAL
		if raw_voltage is None or raw_voltage <= 0.1:
			self._consecutive_invalid_voltage += 1
			if (self._consecutive_invalid_voltage
					== self._invalid_voltage_threshold
					and self.battery_state != BATTERY_CRITICAL):
				self.get_logger().error(
					f"Battery voltage invalid for "
					f"{self._invalid_voltage_threshold} consecutive samples; "
					f"forcing CRITICAL (fail-safe)")
				old = self.battery_state
				self.battery_state = BATTERY_CRITICAL
				self._on_battery_state_change(old, BATTERY_CRITICAL, 0.0)
			return
		self._consecutive_invalid_voltage = 0
		# EMA 低通滤波，平滑掉电机启动的瞬时压降
		a = self.battery_filter_alpha
		if self.battery_voltage_filtered is None:
			self.battery_voltage_filtered = raw_voltage
		else:
			self.battery_voltage_filtered = (
				a * raw_voltage + (1.0 - a) * self.battery_voltage_filtered)
		# 暖机：先收 N 个有效采样让 EMA 收敛，再允许状态切换
		self._voltage_sample_count += 1
		if self._voltage_sample_count < self._warmup_samples:
			return
		pct = self._voltage_to_pct(self.battery_voltage_filtered)
		# 按当前状态计算目标态，恢复时需多 hysteresis 个百分点
		# 恢复路径强制 CRITICAL -> LOW -> NORMAL，确保用户先听到一段告警鸣笛
		h = self.battery_hysteresis_pct
		warn = self.battery_warning_pct
		crit = self.battery_critical_pct
		if self.battery_state == BATTERY_NORMAL:
			target = (BATTERY_CRITICAL if pct <= crit
					  else BATTERY_LOW if pct <= warn
					  else BATTERY_NORMAL)
		elif self.battery_state == BATTERY_LOW:
			target = (BATTERY_CRITICAL if pct <= crit
					  else BATTERY_NORMAL if pct >= warn + h
					  else BATTERY_LOW)
		else:  # BATTERY_CRITICAL
			target = (BATTERY_LOW if pct >= crit + h
					  else BATTERY_CRITICAL)
		# 防抖：连续 N 个采样都指向同一新状态才切换
		if target == self.battery_state:
			self._battery_pending_state = target
			self._battery_pending_count = 0
			return
		if target == self._battery_pending_state:
			self._battery_pending_count += 1
		else:
			self._battery_pending_state = target
			self._battery_pending_count = 1
		if self._battery_pending_count >= self.battery_debounce_samples:
			old = self.battery_state
			self.battery_state = target
			self._battery_pending_count = 0
			self._on_battery_state_change(old, target, pct)

	def _on_battery_state_change(self, old, new, pct):
		names = {BATTERY_NORMAL: 'NORMAL', BATTERY_LOW: 'LOW',
				 BATTERY_CRITICAL: 'CRITICAL'}
		v = (self.battery_voltage_filtered
			 if self.battery_voltage_filtered is not None else 0.0)
		text = f"Battery {names[old]} -> {names[new]} ({pct:.1f}%, {v:.2f}V)"
		if new == BATTERY_NORMAL:
			self.get_logger().info(text)
		elif new == BATTERY_LOW:
			self.get_logger().warn(text)
		else:
			self.get_logger().error(text)
		# 进入临界电量立刻停车，不等下一条 cmd_vel
		if new == BATTERY_CRITICAL:
			try:
				self.car.set_car_motion(0.0, 0.0, 0.0)
			except Exception as e:
				self.get_logger().error(f"Failed to stop motors on CRITICAL: {e}")
			self.last_speed = [0.0, 0.0, 0.0]
			# 让订阅 /safety_status 的节点也感知到这次强制停车
			self.publish_safety_status(False, f"Low battery ({pct:.1f}%)")
			self._battery_safety_published = True
		elif old == BATTERY_CRITICAL and self._battery_safety_published:
			# 仅清除我们自己之前置位的 safety_status
			self.publish_safety_status(True, "Battery recovered")
			self._battery_safety_published = False

	def _battery_beep_tick(self):
		"""按当前电量档位周期性地短促鸣笛。"""
		if (not self.enable_low_battery_protection
				or self.battery_state == BATTERY_NORMAL):
			return
		# 用户通过 /Buzzer 持续开启了蜂鸣器，不要用我们的短鸣覆盖它
		if self._user_buzzer_on:
			return
		if self.battery_state == BATTERY_CRITICAL:
			period = self.battery_critical_beep_period
			duration_ms = 500
		else:
			period = self.battery_warning_beep_period
			duration_ms = 200
		now = self.get_clock().now()
		elapsed = (now - self._last_beep_time).nanoseconds / 1e9
		# sim_time 回放或时钟回退时 elapsed 会变负；重置基准，下个 tick 正常计
		if elapsed < 0:
			self._last_beep_time = now
			return
		if elapsed < period:
			return
		self._last_beep_time = now
		try:
			# 硬件支持 >=10ms 自动关闭
			self.car.set_beep(duration_ms)
		except Exception as e:
			self.get_logger().warn(f"Low-battery buzzer error: {e}")


def main():
    rclpy.init()
    driver = yahboomcar_driver('driver_node')
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        # 程序退出前确保小车停止
        driver.get_logger().info("Shutting down, stopping car...")
        driver.car.set_car_motion(0.0, 0.0, 0.0)
    finally:
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

		
		
