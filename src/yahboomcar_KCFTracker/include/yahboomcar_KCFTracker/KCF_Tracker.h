//
// Created by yahboom on 2021/7/30.
//

#ifndef TRANSBOT_ASTRA_KCF_TRACKER_H
#define TRANSBOT_ASTRA_KCF_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>

#include <cv_bridge/cv_bridge.hpp>

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

// 闭环跟踪状态机：
//   IDLE       - 还没框选目标 / 已 Reset；
//   TRACKING   - KCF 响应正常，正常跟随；
//   LOST       - 响应连续低于阈值刚刚确认丢失（瞬态，立即停车）；
//   RECOVERING - 正在用颜色直方图反向投影做全图重检测，找到即回 TRACKING。
// 转换：
//   IDLE       -> TRACKING                 用户框选完成
//   TRACKING   -> LOST                     连续 lost_patience 帧低置信度
//   LOST       -> RECOVERING               下一帧开始主动重检测（需 enable_redetect）
//   RECOVERING -> TRACKING                 重检测得分超过 recover_threshold
//   任意状态   -> IDLE                     调用 Reset()
// 若 enable_redetect=false，状态会停在 LOST 不进 RECOVERING。
enum class TrackState { IDLE, TRACKING, LOST, RECOVERING };

class ImageConverter :public rclcpp::Node{
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;

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
    // 闭环重检测相关参数
    this->declare_parameter<double>("lost_threshold", 0.15);     // 峰值低于此值视为低置信度帧
    this->declare_parameter<double>("recover_threshold", 0.30);  // 重检测得分高于此值即认为重新发现目标
    this->declare_parameter<int>("lost_patience", 8);            // 连续多少帧低置信度后判定 LOST
    this->declare_parameter<bool>("enable_redetect", true);
    // 重检测外观模型（HSV 色调直方图）的可调参数。
    // 对不同相机 / 光照需要现场微调，无需重编译，
    // 可以用 `ros2 param set /image_converter sat_min 40` 实时改。
    this->declare_parameter<int>("hue_bins", 32);   // 直方图 bin 数
    this->declare_parameter<int>("sat_min", 30);    // S 通道下限，过滤过白 / 灰像素
    this->declare_parameter<int>("val_min", 30);    // V 通道下限，过滤过暗像素
    // 与 collision_detector 的联动：
    // 收到 collision_topic 上的 Bool=true 脉冲时，
    // 在 collision_pause_sec 秒内冻结 /cmd_vel。
    this->declare_parameter<std::string>("collision_topic", "/collision_detector/collision");
    this->declare_parameter<double>("collision_pause_sec", 2.0);

        
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
        this->get_parameter<int>("hue_bins", hue_bins);
        this->get_parameter<int>("sat_min", sat_min);
        this->get_parameter<int>("val_min", val_min);
        this->get_parameter<double>("collision_pause_sec", collision_pause_sec);

        std::string collision_topic;
        this->get_parameter<std::string>("collision_topic", collision_topic);
        collision_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            collision_topic, 10,
            std::bind(&ImageConverter::CollisionCb, this, _1));
        collision_pause_until_ = this->get_clock()->now();
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

    // 闭环重检测的运行时状态
    TrackState track_state = TrackState::IDLE;
    double lost_threshold = 0.15;
    double recover_threshold = 0.30;
    int lost_patience = 8;
    bool enable_redetect = true;
    int low_conf_count = 0;
    float last_peak_value = 0.0f;
    cv::Mat target_hist;          // 首次框选目标时计算的 HSV 色调直方图
    cv::Size target_size;         // 记住原始 ROI 尺寸，重检测后重新初始化用
    bool has_target_model = false;
    int hue_bins = 32;            // 直方图 bin 数（外观模型可调参数）
    int sat_min = 30;             // S 通道下限
    int val_min = 30;             // V 通道下限

    // 碰撞暂停：只要 now() < collision_pause_until_，
    // 无论跟踪处于什么状态，都强制把 /cmd_vel 压成零。
    rclcpp::Time collision_pause_until_;
    double collision_pause_sec = 2.0;
    bool was_paused_ = false;

    void buildTargetModel(const cv::Mat &bgr, const cv::Rect &roi);
    bool redetect(const cv::Mat &bgr, cv::Rect &found);
    void publishConfidence(float v);

    // 边沿触发的状态切换：仅在 new_state 与 track_state 不同时
    // 才打印日志并向 /KCF_status 发布一次。
    void setState(TrackState new_state, const std::string &reason = "");
    static const char *stateName(TrackState s);
    void onStateChanged(TrackState from, TrackState to, const std::string &reason);
    
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

    void CollisionCb(const std::shared_ptr<std_msgs::msg::Bool> msg) ;

    bool inCollisionPause();

    void StopCarb() ;

    //void depthCb(const sensor_msgs::ImageConstPtr &msg);

    //void JoyCb(const std_msgs::BoolConstPtr &msg);

};


#endif //TRANSBOT_ASTRA_KCF_TRACKER_H
