#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// try return pcl point cloud. 

__global__ void depthMapToPointCloud(const unsigned char* inputImage1, const unsigned char* inputImage2,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, int width, int height,
                                     float focal_length, float baseline);


pcl::PointCloud<pcl::PointXYZ>::Ptr depthMapToPointCloudMsg(const cv::Mat& inputImage1, const cv::Mat& inputImage2,
                                                             float focal_length, float baseline);

