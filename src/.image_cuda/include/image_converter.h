#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cuda_filter.h"
#include "disparity_calculator.h"

namespace ImageConverter {
    class ImageConverter {
    public:
        ImageConverter(ros::NodeHandle& node_handle);
        void Run();
        // cv::Mat computeDisparityMap(const cv::Mat& left_image, const cv::Mat& right_image);

    private:
        image_transport::ImageTransport it;
        cv::Mat image1;
        cv::Mat image2;
        ros::Publisher depth_map_pub;
        message_filters::Subscriber<sensor_msgs::Image> cam0_sub;
        message_filters::Subscriber<sensor_msgs::Image> cam1_sub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
        message_filters::Synchronizer<SyncPolicy> sync;
        std::string image_format;
        float focal_length;
        float baseline;

        void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2);
        cv::Mat computeDisparityMap(const cv::Mat& left_image, const cv::Mat& right_image);

    };
}
