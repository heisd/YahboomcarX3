#include <iostream>
#include "KCF_Tracker.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "kcftracker.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cstdio>
Rect selectRect;
Point origin;
Rect result;
bool select_flag = false;
bool bRenewROI = false;
bool bBeginKCF = false;
Mat rgbimage;
Mat depthimage;
const int &ACTION_ESC = 27;
const int &ACTION_SPACE = 32;



void onMouse(int event, int x, int y, int, void *) {
    if (select_flag) {
        selectRect.x = MIN(origin.x, x);
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);
        selectRect.height = abs(y - origin.y);
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == 1) {
//    if (event == CV_EVENT_LBUTTONDOWN) {
        bBeginKCF = false;
        select_flag = true;
        origin = Point(x, y);
        selectRect = Rect(x, y, 0, 0);
    } else if (event == 4) {
//    } else if (event == CV_EVENT_LBUTTONUP) {
        select_flag = false;
        bRenewROI = true;
    }
}

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    this->linear_PID->reset();
    this->angular_PID->reset();
    vel_pub_->publish(geometry_msgs::msg::Twist());

    low_conf_count = 0;
    last_peak_value = 0.0f;
    has_target_model = false;
    target_hist.release();
    setState(TrackState::IDLE, "节点 Reset");
}

const char *ImageConverter::stateName(TrackState s) {
    switch (s) {
        case TrackState::IDLE:       return "IDLE";
        case TrackState::TRACKING:   return "TRACKING";
        case TrackState::LOST:       return "LOST";
        case TrackState::RECOVERING: return "RECOVERING";
    }
    return "UNKNOWN";
}

void ImageConverter::onStateChanged(TrackState from, TrackState to, const std::string &reason) {
    const char *from_s = stateName(from);
    const char *to_s = stateName(to);
    if (reason.empty()) {
        RCLCPP_INFO(this->get_logger(), "[KCF] 状态切换 %s -> %s", from_s, to_s);
    } else {
        RCLCPP_INFO(this->get_logger(), "[KCF] 状态切换 %s -> %s（原因：%s）",
                    from_s, to_s, reason.c_str());
    }
    std_msgs::msg::String m;
    m.data = to_s;
    status_pub_->publish(m);
}

void ImageConverter::setState(TrackState new_state, const std::string &reason) {
    if (new_state == track_state) return;
    TrackState prev = track_state;
    track_state = new_state;
    onStateChanged(prev, new_state, reason);
}

void ImageConverter::publishConfidence(float v) {
    std_msgs::msg::Float32 m;
    m.data = v;
    confidence_pub_->publish(m);
}

// 对目标 ROI 计算 HSV 色调直方图，作为重检测时的外观模板。
// 通过 saturation / value 阈值掩膜过滤低信息像素，
// 让直方图在轻度光照变化下仍有辨识度。
// hue_bins / sat_min / val_min 都来自 ROS 参数，每次进入函数都
// 重新读一次，支持 `ros2 param set` 现场调（注意：直方图是在 ROI 框
// 选时一次性算出来的，改了阈值后想让新的模型生效需要重选 ROI）。
void ImageConverter::buildTargetModel(const cv::Mat &bgr, const cv::Rect &roi) {
    cv::Rect safe = roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (safe.width < 4 || safe.height < 4) {
        has_target_model = false;
        return;
    }
    this->get_parameter<int>("hue_bins", hue_bins);
    this->get_parameter<int>("sat_min", sat_min);
    this->get_parameter<int>("val_min", val_min);
    int bins = std::max(2, std::min(180, hue_bins));
    int smin = std::max(0, std::min(255, sat_min));
    int vmin = std::max(0, std::min(255, val_min));

    cv::Mat hsv, mask;
    cv::cvtColor(bgr(safe), hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, smin, vmin), cv::Scalar(180, 255, 255), mask);
    float hrange[] = {0, 180};
    const float *ranges = hrange;
    int channels = 0;
    cv::calcHist(&hsv, 1, &channels, mask, target_hist, 1, &bins, &ranges, true, false);
    cv::normalize(target_hist, target_hist, 0, 255, cv::NORM_MINMAX);
    target_size = safe.size();
    has_target_model = true;
}

// 闭环重检测：用保存的色调直方图在整帧上做反向投影，
// 然后以图像中心为初始窗口跑 CamShift 收敛。
// 当返回窗口内反向投影的平均响应高于 recover_threshold，
// 即认为目标重新出现，把这个 ROI 交给 KCF 重新初始化。
bool ImageConverter::redetect(const cv::Mat &bgr, cv::Rect &found) {
    if (!has_target_model || target_hist.empty()) return false;

    // 在搜索时也每帧重读 sat_min / val_min，方便 LOST 状态下现场试值。
    this->get_parameter<int>("sat_min", sat_min);
    this->get_parameter<int>("val_min", val_min);
    int smin = std::max(0, std::min(255, sat_min));
    int vmin = std::max(0, std::min(255, val_min));

    cv::Mat hsv, mask, backproj;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, smin, vmin), cv::Scalar(180, 255, 255), mask);
    float hrange[] = {0, 180};
    const float *ranges = hrange;
    int channels = 0;
    cv::calcBackProject(&hsv, 1, &channels, target_hist, backproj, &ranges, 1.0, true);
    backproj &= mask;

    int w = std::max(8, target_size.width);
    int h = std::max(8, target_size.height);
    cv::Rect window(
        std::max(0, bgr.cols / 2 - w / 2),
        std::max(0, bgr.rows / 2 - h / 2),
        std::min(w, bgr.cols - 1),
        std::min(h, bgr.rows - 1));
    cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1);
    cv::RotatedRect rr = cv::CamShift(backproj, window, crit);
    cv::Rect candidate = rr.boundingRect() & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (candidate.width < 8 || candidate.height < 8) return false;

    cv::Scalar mean_resp = cv::mean(backproj(candidate));
    float score = static_cast<float>(mean_resp[0]) / 255.0f;
    publishConfidence(score);
    if (score < recover_threshold) return false;

    found = candidate;
    return true;
}

void ImageConverter::Cancel() {
    this->Reset();
    // RGB_WINDOW / DEPTH_WINDOW 是字符串字面量(const char*)，绝不能 delete：
    // 对非 new 得到的指针 delete 是未定义行为，原代码按 'q'/ESC 即崩溃。
    destroyWindow(RGB_WINDOW);
    vel_pub_->publish(geometry_msgs::msg::Twist());
//        destroyWindow(DEPTH_WINDOW);
}

void ImageConverter::PIDcallback() {

    this->minDist=1.0;
    this->linear_PID->Set_PID(3.0, 0.0, 1.0);
    this->angular_PID->Set_PID(0.5, 0.0, 2.0);
    this->linear_PID->reset();
    this->angular_PID->reset();
}


void ImageConverter::imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        std::cout<<"cv_bridge exception"<<std::endl;
        return;
    }
    
    cv_ptr->image.copyTo(rgbimage);
    setMouseCallback(RGB_WINDOW, onMouse, 0);
    if (bRenewROI) {
         if (selectRect.width <= 0 || selectRect.height <= 0)
         {
             bRenewROI = false;
             return;
         }
        tracker.init(selectRect, rgbimage);
        buildTargetModel(rgbimage, selectRect);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
        low_conf_count = 0;
        setState(TrackState::TRACKING, "用户框选目标完成");
    }
    if (bBeginKCF) {
        const bool searching = enable_redetect &&
            (track_state == TrackState::LOST ||
             track_state == TrackState::RECOVERING);
        if (searching) {
            // LOST 只是"刚确认丢失"的瞬态；一旦准备开始搜索，
            // 立刻升级到 RECOVERING，告诉 /KCF_status 的订阅者
            // 当前正在主动找回目标。setState 是边沿触发，
            // 在 RECOVERING 期间不会重复打日志。
            if (track_state == TrackState::LOST) {
                setState(TrackState::RECOVERING, "开始全图重检测");
            }
            cv::Rect recovered;
            if (redetect(rgbimage, recovered)) {
                tracker.init(recovered, rgbimage);
                result = recovered;
                low_conf_count = 0;
                char reason[96];
                snprintf(reason, sizeof(reason),
                         "重检测成功 位置(%d,%d) 尺寸 %dx%d",
                         recovered.x, recovered.y,
                         recovered.width, recovered.height);
                setState(TrackState::TRACKING, reason);
            } else {
                // 还没找回目标，先停车继续搜索，保持 RECOVERING。
                vel_pub_->publish(geometry_msgs::msg::Twist());
                rectangle(rgbimage, result, Scalar(0, 0, 255), 2, 8);
                putText(rgbimage, "RECOVERING - searching", Point(10, 30),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
            }
        } else {
            float peak = 0.0f;
            result = tracker.update(rgbimage, peak);
            last_peak_value = peak;
            publishConfidence(peak);

            if (peak < lost_threshold) {
                low_conf_count++;
                if (low_conf_count >= lost_patience) {
                    vel_pub_->publish(geometry_msgs::msg::Twist());
                    char reason[96];
                    snprintf(reason, sizeof(reason),
                             "峰值 %.3f < %.3f 持续 %d 帧",
                             peak, lost_threshold, low_conf_count);
                    setState(TrackState::LOST, reason);
                }
            } else {
                low_conf_count = 0;
                char reason[64];
                snprintf(reason, sizeof(reason), "峰值 %.3f 已回升", peak);
                setState(TrackState::TRACKING, reason);
            }

            Scalar color = (track_state == TrackState::TRACKING) ? Scalar(0, 255, 255) : Scalar(0, 0, 255);
            rectangle(rgbimage, result, color, 1, 8);
            circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
            char buf[64];
            snprintf(buf, sizeof(buf), "peak=%.3f", peak);
            putText(rgbimage, buf, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6,
                    Scalar(0, 255, 0), 2);
        }
    } else rectangle(rgbimage, selectRect, Scalar(255, 0, 0), 2, 8, 0);
    //sensor_msgs::ImagePtr kcf_imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbimage).toImageMsg();
    //mage_pub_ -> publish(kcf_imagemsg.get());


    sensor_msgs::msg::Image kcf_imagemsg;
    std_msgs::msg::Header _header;
    cv_bridge::CvImage _cv_bridge;
    _header.stamp = this->get_clock() -> now();
    _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, rgbimage);
    _cv_bridge.toImageMsg(kcf_imagemsg);
    image_pub_-> publish(kcf_imagemsg);
    imshow(RGB_WINDOW, rgbimage);
    int action = waitKey(1) & 0xFF;
    if (action == 'q' || action == ACTION_ESC) this->Cancel();
    else if (action == 'r')  this->Reset();
    else if (action == ACTION_SPACE) enable_get_depth = true;
}

void ImageConverter::depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
	this->get_parameter<float>("minDist_",this->minDist);
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        std::cout<<"Could not convert from  to 'TYPE_32FC1'."<<std::endl;
    }
    if (inCollisionPause()) {
        vel_pub_->publish(geometry_msgs::msg::Twist());
        waitKey(1);
        return;
    }
    if (enable_get_depth && track_state == TrackState::TRACKING) {
        int center_x = (int)(result.x + result.width / 2);
        std::cout<<"center_x: "<<center_x<<std::endl;
        int center_y = (int)(result.y + result.height / 2);
        std::cout<<"center_y: "<<center_y<<std::endl;
        if (depthimage.empty() || depthimage.cols < 12 || depthimage.rows < 12) {
            vel_pub_->publish(geometry_msgs::msg::Twist());
            waitKey(1);
            return;
        }
        // 把采样点夹到有效范围，避免目标贴边时 at<float> 越界(未定义行为/崩溃)。
        int sx = std::min(std::max(center_x, 5), depthimage.cols - 6);
        int sy = std::min(std::max(center_y, 5), depthimage.rows - 6);
        dist_val[0] = depthimage.at<float>(sy - 5, sx - 5)/1000.0;
        dist_val[1] = depthimage.at<float>(sy - 5, sx + 5)/1000.0;
        dist_val[2] = depthimage.at<float>(sy + 5, sx + 5)/1000.0;
        dist_val[3] = depthimage.at<float>(sy + 5, sx - 5)/1000.0;
        // 中心点同样要除以 1000：原代码漏除导致它与其余四点单位不一致，
        // 会把平均距离整体带偏(且毫米值常>10被判为无效而丢弃)。
        dist_val[4] = depthimage.at<float>(sy, sx)/1000.0;
        std::cout<<"dist_val[0]: "<<dist_val[0]<<std::endl;
        std::cout<<"dist_val[1]: "<<dist_val[1]<<std::endl;
        std::cout<<"dist_val[2]: "<<dist_val[2]<<std::endl;
        std::cout<<"dist_val[3]: "<<dist_val[3]<<std::endl;
        std::cout<<"dist_val[4]: "<<dist_val[4]<<std::endl;
        float distance = 0;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
            if (dist_val[i] > 0.4 && dist_val[i] < 10.0) distance += dist_val[i];
            else num_depth_points--;
        }
        if (num_depth_points != 0) {
            distance /= num_depth_points;   // 先判 !=0 再除，避免 5 点全无效时除零
            std::cout<<distance<<std::endl;
        	std::cout<<"minDist: "<<minDist<<std::endl;
            if (abs(distance - this->minDist) < 0.1) linear_speed = 0;
            else linear_speed = -linear_PID->compute(this->minDist, distance);//-linear_PID->compute(minDist, distance)
        } else {
            // 五点全无效：本帧没有可信距离，前进/后退速度归零并复位 PID，
            // 避免沿用上一帧旧值盲目前冲、以及恢复时的微分冲击；
            // 转向仍按视觉中心保持对准目标。
            linear_speed = 0;
            linear_PID->reset();
        }
        // 用实际图像宽度的中心，而不是硬编码 320：换相机分辨率时也能正确居中。
        double frame_cx = rgbimage.empty() ? 320.0 : rgbimage.cols / 2.0;
        rotation_speed = angular_PID->compute(frame_cx / 100.0, center_x / 100.0);
        if (abs(rotation_speed) < 0.1)rotation_speed = 0;
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_speed;
        twist.angular.z = rotation_speed;
        vel_pub_->publish(twist);
        
    }  
    else{
    	geometry_msgs::msg::Twist twist;
    	vel_pub_->publish(twist);
    }
//        imshow(DEPTH_WINDOW, depthimage);
    waitKey(1);
}

void ImageConverter::JoyCb(const std::shared_ptr<std_msgs::msg::Bool> msg) {
    enable_get_depth = msg->data;
}

// 由 collision_detector 触发的暂停门闸。
// 这里**故意不**改 KCF 状态机 —— 视觉跟踪继续进行 ——
// 只是把 /cmd_vel 冻结 collision_pause_sec 秒，让小车冲击平息。
// 暂停起始/结束用边沿日志输出，便于和状态切换日志一起观察。
void ImageConverter::CollisionCb(const std::shared_ptr<std_msgs::msg::Bool> msg) {
    if (!msg->data) return;
    auto now = this->get_clock()->now();
    auto until = now + rclcpp::Duration::from_seconds(collision_pause_sec);
    collision_pause_until_ = until;
    vel_pub_->publish(geometry_msgs::msg::Twist());  // 立刻急停一帧
    RCLCPP_WARN(this->get_logger(),
                "[KCF] 收到碰撞脉冲 -> 冻结 cmd_vel %.2f 秒",
                collision_pause_sec);
    was_paused_ = true;
}

bool ImageConverter::inCollisionPause() {
    bool paused = this->get_clock()->now() < collision_pause_until_;
    if (!paused && was_paused_) {
        // 从暂停窗口跳出来时打印一次，避免每帧刷屏
        RCLCPP_INFO(this->get_logger(), "[KCF] 碰撞暂停结束，恢复跟随");
        was_paused_ = false;
    }
    return paused;
}








