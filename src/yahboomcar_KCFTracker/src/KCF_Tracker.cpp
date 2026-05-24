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

    track_state = TrackState::IDLE;
    low_conf_count = 0;
    last_peak_value = 0.0f;
    has_target_model = false;
    target_hist.release();
    publishStatus("IDLE");
}

void ImageConverter::publishStatus(const std::string &s) {
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
}

void ImageConverter::publishConfidence(float v) {
    std_msgs::msg::Float32 m;
    m.data = v;
    confidence_pub_->publish(m);
}

// Build an HSV hue histogram of the target ROI to use as the re-detection
// appearance model. Saturation / value gating drops low-information pixels so
// the histogram stays discriminative under modest lighting changes.
void ImageConverter::buildTargetModel(const cv::Mat &bgr, const cv::Rect &roi) {
    cv::Rect safe = roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (safe.width < 4 || safe.height < 4) {
        has_target_model = false;
        return;
    }
    cv::Mat hsv, mask;
    cv::cvtColor(bgr(safe), hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 30), cv::Scalar(180, 255, 255), mask);
    int histSize = 32;
    float hrange[] = {0, 180};
    const float *ranges = hrange;
    int channels = 0;
    cv::calcHist(&hsv, 1, &channels, mask, target_hist, 1, &histSize, &ranges, true, false);
    cv::normalize(target_hist, target_hist, 0, 255, cv::NORM_MINMAX);
    target_size = safe.size();
    has_target_model = true;
}

// Closed-loop re-detection: back-project the saved hue histogram over the
// whole frame, then run CamShift seeded from the image centre. We accept the
// recovery when the mean back-projection response inside the returned window
// exceeds recover_threshold, signalling the target is visible again.
bool ImageConverter::redetect(const cv::Mat &bgr, cv::Rect &found) {
    if (!has_target_model || target_hist.empty()) return false;

    cv::Mat hsv, mask, backproj;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 30, 30), cv::Scalar(180, 255, 255), mask);
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
    
    delete RGB_WINDOW;
    delete DEPTH_WINDOW;
    //delete this->linear_PID;
    //delete this->angular_PID;
    
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
        track_state = TrackState::TRACKING;
        low_conf_count = 0;
        publishStatus("TRACKING");
    }
    if (bBeginKCF) {
        if (track_state == TrackState::LOST && enable_redetect) {
            cv::Rect recovered;
            if (redetect(rgbimage, recovered)) {
                tracker.init(recovered, rgbimage);
                result = recovered;
                track_state = TrackState::TRACKING;
                low_conf_count = 0;
                publishStatus("RECOVERED");
            } else {
                // Stop the car while we keep searching for the target.
                vel_pub_->publish(geometry_msgs::msg::Twist());
                rectangle(rgbimage, result, Scalar(0, 0, 255), 2, 8);
                putText(rgbimage, "LOST - searching", Point(10, 30),
                        FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                publishStatus("LOST");
            }
        } else {
            float peak = 0.0f;
            result = tracker.update(rgbimage, peak);
            last_peak_value = peak;
            publishConfidence(peak);

            if (peak < lost_threshold) {
                low_conf_count++;
                if (low_conf_count >= lost_patience) {
                    track_state = TrackState::LOST;
                    vel_pub_->publish(geometry_msgs::msg::Twist());
                    publishStatus("LOST");
                }
            } else {
                low_conf_count = 0;
                track_state = TrackState::TRACKING;
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
    if (enable_get_depth && track_state == TrackState::TRACKING) {
        int center_x = (int)(result.x + result.width / 2);
        std::cout<<"center_x: "<<center_x<<std::endl;
        int center_y = (int)(result.y + result.height / 2);
        std::cout<<"center_y: "<<center_y<<std::endl;
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5)/1000.0;
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5)/1000.0;
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5)/1000.0;
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5)/1000.0;
        dist_val[4] = depthimage.at<float>(center_y, center_x);
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
        distance /= num_depth_points;
        std::cout<<distance<<std::endl;
        if (num_depth_points != 0) {
        	std::cout<<"minDist: "<<minDist<<std::endl;
            if (abs(distance - this->minDist) < 0.1) linear_speed = 0;
            else linear_speed = -linear_PID->compute(this->minDist, distance);//-linear_PID->compute(minDist, distance)
        }
        rotation_speed = angular_PID->compute(320 / 100.0, center_x / 100.0);//angular_PID->compute(320 / 100.0, center_x / 100.0)
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








