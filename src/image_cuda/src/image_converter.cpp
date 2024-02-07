#include "image_converter.h"

namespace ImageConverter {
    ImageConverter::ImageConverter(ros::NodeHandle& node_handle)
        : it(node_handle), focal_length(0.0), baseline(0.0)
    {
        ros::NodeHandle private_nh("~"); // Private node handle to access private parameters
        private_nh.param<std::string>("image_format_", image_format, "bgr8");
        private_nh.param<float>("focal_length", focal_length, 0.0); // Load focal length parameter
        private_nh.param<float>("baseline", baseline, 0.0); // Load baseline parameter

        depth_map_pub = node_handle.advertise<sensor_msgs::PointCloud2>("depth_map", 1); // Changed to ros::Publisher

        cam0_sub = it.subscribe("/camera0/image_raw", 0, &ImageConverter::imageCallback1, this);
        cam1_sub = it.subscribe("/camera1/image_raw", 0, &ImageConverter::imageCallback2, this);
    }
    void ImageConverter::Run()
    {
        ros::Rate mRate(30);
        focal_length= 3.6; 
        baseline = 65;

        while (ros::ok())
        {
            sensor_msgs::PointCloud2 depth_cloud;

            if (!image1.empty() && !image2.empty())
            {
                try {
                    depth_cloud = depthMapToPointCloudMsg(image1, image2, focal_length, baseline);
                }
                catch (const std::exception& e){
                    // Handle exception if necessary
                }
            }
            else
            {
                ROS_WARN("Image1 or Image2 is empty");
            }

            // Publish the point cloud
            if (!depth_cloud.data.empty()) {
                depth_map_pub.publish(depth_cloud);
                ROS_WARN_STREAM("Published point cloud");
            } else {
                ROS_ERROR("Empty point cloud received");
            }

            ros::spinOnce();
            mRate.sleep();
        }
    }

    void ImageConverter::imageCallback1(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            image1 = cv_bridge::toCvShare(msg, image_format)->image;
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'image_format'.", msg->encoding.c_str());
        }
    }

    void ImageConverter::imageCallback2(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            image2 = cv_bridge::toCvShare(msg, image_format)->image;
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("Could not convert from to ." << image_format << ", " << msg->encoding.c_str());
        }
    }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;  // Create a NodeHandle here
  ImageConverter::ImageConverter node(nh); // Pass the NodeHandle to the constructor
  node.Run();
  return 0;
}