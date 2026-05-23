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

		#create timer 创建定时器
		self.timer = self.create_timer(0.1, self.pub_data)  # 10Hz发布数据
		self.safety_timer = self.create_timer(0.05, self.safety_check)  # 20Hz安全检查	

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

		
		
