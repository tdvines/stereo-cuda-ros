#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

#include <thread>
#include <mutex>

std::string camera_topic_0, camera_topic_1;

namespace CSICameraROS {

    class CSICameraROS {
    public:
        CSICameraROS(int camera_id, std::string camera_topic, int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method);
        void start(const std::string& gstreamer_pipeline);
        bool read(cv::Mat& frame);
        void stop();
        void release();
        image_transport::Publisher image_pub_0; // Change this name to avoid confusion

    private:
        int camera_id;
        std::string camera_topic_;
        int capture_width_;
        int capture_height_;
        int display_width_;
        int display_height_;
        int framerate_;
        int flip_method_;
        bool running_;
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        cv::VideoCapture video_capture_;
        cv::Mat frame_;
        std::thread read_thread_;
        std::mutex frame_mutex_;

        void updateCamera();
    };

}  // namespace CSICameraROS
