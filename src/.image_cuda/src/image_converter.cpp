#include "image_converter.h"

namespace ImageConverter {
    ImageConverter::ImageConverter(ros::NodeHandle& node_handle)
        : it(node_handle), cam0_sub(node_handle, "/camera0/image_raw", 1), cam1_sub(node_handle, "/camera1/image_raw", 1), sync(SyncPolicy(10), cam0_sub, cam1_sub)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("image_format_", image_format, "bgr8");
        private_nh.param<float>("focal_length", focal_length, 0.0);
        private_nh.param<float>("baseline", baseline, 0.0);
        depth_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("depth_map", 1);
        sync.registerCallback(boost::bind(&ImageConverter::imageCallback, this, _1, _2));
    }

    void ImageConverter::Run()
    {
        ros::spin();
            // mRate.sleep();
    }

    void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
    {
        try 
        {
            image1 = cv_bridge::toCvShare(msg1, image_format)->image;
            image2 = cv_bridge::toCvShare(msg2, image_format)->image;

            cv::imshow("left", image1);
            cv::imshow("right", image2);
            cv::waitKey(1);

            DisparityCalculator::DisparityCalculator disparity_calculator;
            cv::Mat disparity = disparity_calculator.computeDisparityMap(image1, image2);

            // // Publish the point cloud
            // if (!depth_cloud.data.empty()) {
            //     depth_map_pub.publish(depth_cloud);
            //     ROS_WARN_STREAM("Published point cloud");
            // } else {
            //     ROS_ERROR("Empty point cloud received");
            // }
        }
        catch (const std::exception& e) {}
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle node_handle;  // Create a NodeHandle here
    ImageConverter::ImageConverter node(node_handle); // Pass the NodeHandle to the constructor
    node.Run();
    return 0;
}
