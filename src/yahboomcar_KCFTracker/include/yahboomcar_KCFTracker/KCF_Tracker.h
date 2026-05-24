//
// Created by yahboom on 2021/7/30.
//

#ifndef TRANSBOT_ASTRA_KCF_TRACKER_H
#define TRANSBOT_ASTRA_KCF_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "kcftracker.h"
#include "PID.h"
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <time.h>

using namespace std;
using namespace cv;
using std::placeholders::_1;

// Closed-loop tracking state: TRACKING when KCF response is healthy, LOST when
// confidence drops below threshold for too long, RECOVERING while we try to
// re-acquire the target by colour-histogram back-projection.
enum class TrackState { IDLE, TRACKING, LOST, RECOVERING };

class ImageConverter :public rclcpp::Node{
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Joy_sub_;

public:
    ImageConverter():Node("image_converter")
    {
    float linear_KP=3.0;
    float linear_KI=0.0;
    float linear_KD=1.0;
    float angular_KP=0.5;
    float angular_KI=0.0;
    float angular_KD=2.0;
    float minDist = 1.0;
    bool refresh = false;
        
    this->declare_parameter<float>("linear_KP_",3.0);
    this->declare_parameter<float>("linear_KI_",0.0);
    this->declare_parameter<float>("linear_KD_",1.0);
    this->declare_parameter<float>("angular_KP_",0.5);
    this->declare_parameter<float>("angular_KI_",0.0);
    this->declare_parameter<float>("angular_KD_",2.0);
    this->declare_parameter<float>("minDist_",1.0);
    this->declare_parameter<bool>("refresh_",false);
    // Closed-loop re-detection parameters
    this->declare_parameter<double>("lost_threshold", 0.15);     // peak below this for N frames -> LOST
    this->declare_parameter<double>("recover_threshold", 0.30);  // re-init when redetector score above this
    this->declare_parameter<int>("lost_patience", 8);            // consecutive low-confidence frames before LOST
    this->declare_parameter<bool>("enable_redetect", true);

        
    this->get_parameter<float>("linear_KP_",linear_KP);
    this->get_parameter<float>("linear_KI_",linear_KI);
    this->get_parameter<float>("linear_KD_",linear_KD);
    this->get_parameter<float>("angular_KP_",angular_KP);
    this->get_parameter<float>("angular_KI_",angular_KI);
    this->get_parameter<float>("angular_KD_",angular_KD);
    this->get_parameter<float>("minDist_",minDist);
    this->get_parameter<bool>("refresh_",refresh);

        
    this->linear_PID = new PID(linear_KP, linear_KI, linear_KD);
    this->angular_PID = new PID(angular_KP, angular_KI, angular_KD);
        //sub
        image_sub_=this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",1,std::bind(&ImageConverter::imageCb,this,_1));
        depth_sub_=this->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw",1,std::bind(&ImageConverter::depthCb,this,_1));
        Joy_sub_=this->create_subscription<std_msgs::msg::Bool>("JoyState",1,std::bind(&ImageConverter::JoyCb,this,_1));
        //pub
        image_pub_=this->create_publisher<sensor_msgs::msg::Image>("/KCF_image",1);
        vel_pub_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/KCF_status", 1);
        confidence_pub_ = this->create_publisher<std_msgs::msg::Float32>("/KCF_confidence", 1);

        this->get_parameter<double>("lost_threshold", lost_threshold);
        this->get_parameter<double>("recover_threshold", recover_threshold);
        this->get_parameter<int>("lost_patience", lost_patience);
        this->get_parameter<bool>("enable_redetect", enable_redetect);
    }
    //ros::Publisher pub;
    PID *linear_PID;
    PID *angular_PID;
    
    /*ros::NodeHandle n;
    ros::Subscriber image_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber Joy_sub_;
    ros::Publisher image_pub_;*/
    const char *RGB_WINDOW = "rgb_img";
    const char *DEPTH_WINDOW = "depth_img";
    float minDist = 1.0;
    float linear_speed = 0;
    float rotation_speed = 0;
    bool enable_get_depth = false;
    float dist_val[5];
    bool HOG = true;
    bool FIXEDWINDOW = false;
    bool MULTISCALE = true;
    bool LAB = false;
    int center_x;
    KCFTracker tracker;

    // Closed-loop re-detection state
    TrackState track_state = TrackState::IDLE;
    double lost_threshold = 0.15;
    double recover_threshold = 0.30;
    int lost_patience = 8;
    bool enable_redetect = true;
    int low_conf_count = 0;
    float last_peak_value = 0.0f;
    cv::Mat target_hist;          // HSV hue histogram of the originally selected target
    cv::Size target_size;         // remembered ROI size for re-init
    bool has_target_model = false;

    void buildTargetModel(const cv::Mat &bgr, const cv::Rect &roi);
    bool redetect(const cv::Mat &bgr, cv::Rect &found);
    void publishStatus(const std::string &s);
    void publishConfidence(float v);
    
    //dynamic_reconfigure::Server<yahboomcar_astra::KCFTrackerPIDConfig> server;
    //dynamic_reconfigure::Server<yahboomcar_astra::KCFTrackerPIDConfig>::CallbackType f;

    //ImageConverter(ros::NodeHandle &n);
    //ImageConverter(Node);

    //~ImageConverter();

    void PIDcallback();

    void Reset();

    void Cancel();

    void imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) ;

    void depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) ;

    void JoyCb(const std::shared_ptr<std_msgs::msg::Bool> msg) ;
    
    void StopCarb() ;

    //void depthCb(const sensor_msgs::ImageConstPtr &msg);

    //void JoyCb(const std_msgs::BoolConstPtr &msg);

};


#endif //TRANSBOT_ASTRA_KCF_TRACKER_H
