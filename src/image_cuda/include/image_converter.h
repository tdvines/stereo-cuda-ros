#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "cuda_filter.h"

namespace ImageConverter {
    class ImageConverter {
    public:
        ImageConverter(ros::NodeHandle& node_handle);
        void Run();
        void imageCallback1(const sensor_msgs::ImageConstPtr& msg);
        void imageCallback2(const sensor_msgs::ImageConstPtr& msg);
        sensor_msgs::PointCloud2 depthMapToPointCloudMsg(const cv::Mat& inputImage1, const cv::Mat& inputImage2,
                                                  float focal_length, float baseline);


    private:
        image_transport::ImageTransport it;
        cv::Mat image1;
        cv::Mat image2;
        ros::Publisher depth_map_pub; // Changed to ros::Publisher
        image_transport::Subscriber cam0_sub;
        image_transport::Subscriber cam1_sub;

        std::string image_format;  // image format
        float focal_length;  // focal length
        float baseline;  // baseline
    };
}
