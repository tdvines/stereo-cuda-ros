#include "main.h"
#include "disparity_calculator.h"

namespace DisparityCalculator {
    DisparityCalculator::DisparityCalculator(ros::NodeHandle& node_handle)
     : it(node_handle),
     cam0_sub(node_handle,
     "/camera0/image_raw", 1),
     cam1_sub(node_handle, "/camera1/image_raw", 1),
     sync(SyncPolicy(10),
     cam0_sub, cam1_sub)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("image_format_", image_format, "bgr8");
        private_nh.param<float>("focal_length", focal_length, 0.0);
        private_nh.param<float>("baseline", baseline, 0.0);
        depth_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("depth_map", 1);
        sync.registerCallback(boost::bind(&DisparityCalculator::imageCallback, this, _1, _2));
    }

    void DisparityCalculator::imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2) 
    {
        try 
        {
            image1 = cv_bridge::toCvShare(msg1, image_format)->image;
            image2 = cv_bridge::toCvShare(msg2, image_format)->image;

            cv::imshow("left", image1);
            cv::imshow("right", image2);
            cv::waitKey(1);
            cv::Mat disparity = DisparityCalculator::computeDisparityMap(image1, image2);

            // Display the disparity map as well if needed
            cv::imshow("disparity", disparity);
            cv::waitKey(1);
        }
        catch (const std::exception& e) {}
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle node_handle;  // Create a NodeHandle here
    DisparityCalculator::DisparityCalculator node(node_handle); // Pass the NodeHandle to the constructor
    
    ros::spin();  // Start spinning to handle ROS callbacks
    
    return 0;
}