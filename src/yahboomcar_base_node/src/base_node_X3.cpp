#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.h>

#include <memory>
#include <string>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// 里程计发布节点
class OdomPublisher:public rclcpp ::Node
{
    // 订阅速度指令话题
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
   // 发布里程计话题
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
   // TF广播器
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   // 线速度缩放因子
   double linear_scale_x_ = 0.0 ;
   double linear_scale_y_ = 0.0;
   // 速度时间间隔
   double vel_dt_ = 0.0;
   // 位置坐标
   double x_pos_ = 0.0;
   double y_pos_ = 0.0;
   //  heading角度
   double heading_ = 0.0;
   // 线速度
   double linear_velocity_x_ = 0.0;
   double linear_velocity_y_ = 0.0;
   // 角速度
   double angular_velocity_z_ = 0.0;
   // 轮距
   double wheelbase_ = 0.25;
   // 是否发布里程计TF
   bool pub_odom_tf_ = false;
   // 上一次速度指令时间
   rclcpp::Time last_vel_time_  ;
    // 里程计坐标系和机器人基座坐标系名称
   std::string odom_frame = "odom";
   std::string base_footprint_frame = "base_footprint";
	public:
	  OdomPublisher()
	  : Node("base_node")
	  {     // 声明参数
            this->declare_parameter<double>("wheelbase",0.25);                             // 轮距
            this->declare_parameter<std::string>("odom_frame","odom");                     // 里程计坐标系名称
            this->declare_parameter<std::string>("base_footprint_frame","base_footprint"); // 机器人基座坐标系名称
            this->declare_parameter<double>("linear_scale_x",1.0);                         // 线速度x缩放因子
            this->declare_parameter<double>("linear_scale_y",1.0);                         // 线速度y缩放因子
            this->declare_parameter<bool>("pub_odom_tf",false);                            // 是否发布里程计TF

            this->get_parameter<double>("linear_scale_x",linear_scale_x_);                 // 线速度x缩放因子
            this->get_parameter<double>("linear_scale_y",linear_scale_y_);                 // 线速度y缩放因子
            this->get_parameter<double>("wheelbase",wheelbase_);                           // 轮距
            this->get_parameter<bool>("pub_odom_tf",pub_odom_tf_);                        // 是否发布里程计TF
            this->get_parameter<std::string>("odom_frame",odom_frame);                     // 里程计坐标系名称
            this->get_parameter<std::string>("base_footprint_frame",base_footprint_frame); // 机器人基座坐标系名称
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        
	  	subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("vel_raw",50,std::bind(&OdomPublisher::handle_vel,this,_1));// 订阅速度指令话题
	  	odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 50);      // 发布里程计话题
	  	}
	  	private:
        // 处理速度指令回调函数
	  	  void handle_vel(const std::shared_ptr<geometry_msgs::msg::Twist > msg)
	  	  {
            //geometry_msgs::msg::Twist twist;
            // 计算当前时间与上一次时间的时间间隔
	  	  	rclcpp::Time curren_time = rclcpp::Clock().now();
	  	  	linear_velocity_x_ = msg->linear.x * linear_scale_x_;// scale = 1
    		linear_velocity_y_ = msg->linear.y * linear_scale_y_;
    		angular_velocity_z_ = msg->angular.z ;
			vel_dt_ = (curren_time - last_vel_time_).seconds();
            //std::cout<<"curren_time: "<<curren_time.seconds()<<std::endl;// 打印当前时间
           // std::cout<<"vel_dt: "<<vel_dt_<<std::endl;// 打印速度时间间隔
    		last_vel_time_ = curren_time;
			double steer_angle = linear_velocity_y_;
			double MI_PI = 3.1416;
			
    		double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    		double delta_x = (linear_velocity_x_ * cos(heading_)-linear_velocity_y_*sin(heading_)) * vel_dt_; //m
    		double delta_y = (linear_velocity_x_ * sin(heading_)+linear_velocity_y_*cos(heading_)) * vel_dt_; //m	
 			x_pos_ += delta_x;
    		y_pos_ += delta_y;
			heading_ += delta_heading;
            // 计算四元数表示的朝向// 计算机器人在x,y,z方向的位置
            tf2::Quaternion myQuaternion;
			geometry_msgs::msg::Quaternion odom_quat ; 
			myQuaternion.setRPY(0.00,0.00,heading_ );

            odom_quat.x = myQuaternion.x();
            odom_quat.y = myQuaternion.y();
            odom_quat.z = myQuaternion.z();
            odom_quat.w = myQuaternion.w();
  
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = curren_time;
			odom.header.frame_id = odom_frame;
			odom.child_frame_id =  base_footprint_frame;
			// robot's position in x,y and z// 机器人在x,y,z方向的位置
			odom.pose.pose.position.x = x_pos_;
			odom.pose.pose.position.y = y_pos_;
			odom.pose.pose.position.z = 0.0;
			// robot's heading in quaternion// 机器人的四元数表示的朝向
            odom.pose.pose.orientation = odom_quat;
			odom.pose.covariance[0] = 0.001;
			odom.pose.covariance[7] = 0.001;
			odom.pose.covariance[35] = 0.001;
			// linear speed from encoders// 编码器测量的线速度
			odom.twist.twist.linear.x = linear_velocity_x_;
			odom.twist.twist.linear.y = linear_velocity_y_;
			odom.twist.twist.linear.y = 0.0; // vy = 0.0
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			// angular speed from encoders// 编码器测量的角速度
			odom.twist.twist.angular.z = angular_velocity_z_;
			odom.twist.covariance[0] = 0.0001;
			odom.twist.covariance[7] = 0.0001;
			odom.twist.covariance[35] = 0.0001;
			// ROS_INFO("ODOM PUBLISH");// 打印发布里程计信息
			odom_publisher_ -> publish(odom);// 发布里程计信息
            if (pub_odom_tf_)
            {
                // 发布里程计TF// 发布里程计的变换信息
                geometry_msgs::msg::TransformStamped t;
                rclcpp::Time now = this->get_clock()->now();
                t.header.stamp = now;
                t.header.frame_id = odom_frame;
                t.child_frame_id = base_footprint_frame;
                t.transform.translation.x = x_pos_;
                t.transform.translation.y = y_pos_;
                t.transform.translation.z = 0.0;
                
                t.transform.rotation.x = myQuaternion.x();
                t.transform.rotation.y = myQuaternion.y();
                t.transform.rotation.z = myQuaternion.z();
                t.transform.rotation.w = myQuaternion.w();
                
                tf_broadcaster_->sendTransform(t);
                  
            }
		  	  }

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomPublisher>());
	rclcpp::shutdown();
    return 0;
}

