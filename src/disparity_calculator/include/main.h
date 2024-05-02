#ifndef DISPARITY_CALCULATOR_H
#define DISPARITY_CALCULATOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

namespace DisparityCalculator {
    class DisparityCalculator {
    public:
        // DisparityCalculator();
        DisparityCalculator(ros::NodeHandle& nh); // Add constructor with ros::NodeHandle& argument
        void Run();
        // void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2);

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

#endif // DISPARITY_CALCULATOR_H



