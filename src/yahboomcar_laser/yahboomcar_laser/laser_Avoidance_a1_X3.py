#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#commom lib
import math
import numpy as np
import time
from time import sleep
from yahboomcar_laser.common import *
from Rosmaster_Lib import Rosmaster
print ("improt done")
RAD2DEG = 180 / math.pi
# 激光避障节点// 处理激光扫描数据并发布运动指令
class laserAvoid(Node):
    def __init__(self,name):
        super().__init__(name)
        # 订阅激光扫描数据// 订阅激光扫描数据话题
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan,1)
        self.sub_JoyState = self.create_subscription(Bool,'/JoyState', self.JoyStateCallback,1)
        # 发布运动指令// 发布运动指令话题
        self.pub_vel = self.create_publisher(Twist,'/cmd_vel',1)   

        # 声明参数// 声明参数话题
        self.declare_parameter("linear",0.05)          # 线速度参数// 线速度参数话题
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.declare_parameter("angular",1.0)          # 角速度参数// 角速度参数话题
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.declare_parameter("LaserAngle",40.0)      # 激光扫描角度参数// 激光扫描角度参数话题
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.2)    # 响应距离参数// 响应距离参数话题
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)         # 开关参数// 开关参数话题
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        # 初始化警告标志// 初始化警告标志话题
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        # 初始化移动标志// 初始化移动标志话题
        self.Moving = False
        # 创建定时器// 创建定时器话题
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):# 定时器回调函数// 处理激光扫描数据
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value                # 开关参数
        self.angular = self.get_parameter('angular').get_parameter_value().double_value            # 角速度参数
        self.linear = self.get_parameter('linear').get_parameter_value().double_value              # 线速度参数
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value      # 激光扫描角度参数
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value  # 响应距离参数

    # 处理JoyState消息// 处理JoyState消息话题
    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
    # 处理激光扫描数据// 处理激光扫描数据话题
    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        bot = Rosmaster(com="/dev/ttyUSB1")
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0                     # 右侧警告标志
        self.Left_warning = 0                      # 左侧警告标志
        self.front_warning = 0                     # 前方警告标志
        # 处理激光扫描数据// 处理激光扫描数据话题
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG      # 计算当前激光扫描点的角度
            # 检查是否在右侧激光扫描区域// 检查是否在右侧激光扫描区域话题
            if 160 > angle > 180 - self.LaserAngle:
                if ranges[i] < self.ResponseDist*1.5:
                    self.Right_warning += 1
            # 检查是否在左侧激光扫描区域// 检查是否在左侧激光扫描区域话题
            if - 160 < angle < self.LaserAngle - 180:
                if ranges[i] < self.ResponseDist*1.5:
                    self.Left_warning += 1
            # 检查是否在前方激光扫描区域// 检查是否在前方激光扫描区域话题
            if abs(angle) > 160:
                 if ranges[i] <= self.ResponseDist*1.5: 
                        self.front_warning += 1
        # 根据警告标志发布运动指令// 根据警告标志发布运动指令话题
        if self.Joy_active or self.Switch == True:
            if self.Moving == True:
                self.pub_vel.publish(Twist())

                self.Moving = not self.Moving
            return
        self.Moving = True # 标志位，用于判断是否正在移动
        twist = Twist() # 初始化运动指令
        # 检查是否有障碍物在前方// 检查是否有障碍物在前方话题
        if self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print ('1, there are obstacles in the left and right, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        # 检查是否有障碍物在右侧// 检查是否有障碍物在右侧话题
        elif self.front_warning <= 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print ('3, there is an obstacle in the middle right, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
            # 检查是否有障碍物在左侧// 检查是否有障碍物在左侧话题
            if self.Left_warning > 10 and self.Right_warning <= 10:
                twist.linear.x = 0.0
                twist.angular.z = -self.angular
                self.pub_vel.publish(twist)
                sleep(0.5)
        # 检查是否有障碍物在左侧// 检查是否有障碍物在左侧话题
        elif self.front_warning > 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
            # 检查是否有障碍物在右侧// 检查是否有障碍物在右侧话题
            if self.Left_warning <= 10 and self.Right_warning > 10:
                twist.linear.x = 0.0
                twist.angular.z = self.angular
                self.pub_vel.publish(twist)
                sleep(0.5)
        # 检查是否有障碍物在前方// 检查是否有障碍物在前方话题
        elif self.front_warning > 10 and self.Left_warning < 10 and self.Right_warning < 10:

            print ('6, there is an obstacle in the middle, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        # 检查是否有障碍物在前方// 检查是否有障碍物在前方话题
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning > 10:
            print ('7. There are obstacles on the left and right, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.4)
        # 检查是否有障碍物在左侧// 检查是否有障碍物在左侧话题
        elif self.front_warning < 10 and self.Left_warning > 10 and self.Right_warning <= 10:
            print ('8, there is an obstacle on the left, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        # 检查是否有障碍物在右侧// 检查是否有障碍物在右侧话题
        elif self.front_warning < 10 and self.Left_warning <= 10 and self.Right_warning > 10:
            print ('9, there is an obstacle on the right, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.pub_vel.publish(twist)
            sleep(0.2)
        # 检查是否有障碍物在前方// 检查是否有障碍物在前方话题
        elif self.front_warning <= 10 and self.Left_warning <= 10 and self.Right_warning <= 10:
            print ('10, no obstacles, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0.0
            self.pub_vel.publish(twist)

def main():
    rclpy.init(args=None)                    # 初始化ROS 2
    laser_avoid = laserAvoid("laser_Avoidance_a1") # 创建节点
    print ("start it")
    try:
        rclpy.spin(laser_avoid)              # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        laser_avoid.pub_vel.publish(Twist())  # 发布停止指令
        laser_avoid.destroy_node()            # 销毁节点
        rclpy.shutdown()                      # 关闭ROS 2
