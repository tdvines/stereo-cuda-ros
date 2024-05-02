// disparity_calculator.cpp
#include "disparity_calculator.h"


namespace DisparityCalculator {
    cv::Mat DisparityCalculator::computeDisparityMap(const cv::Mat& left_image, const cv::Mat& right_image) 
    {
        cv::Mat gray_left, gray_right;
        cv::cuda::GpuMat d_left_image, d_right_image;

        // Convert input images to grayscale
        cv::cvtColor(left_image, gray_left, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_image, gray_right, cv::COLOR_BGR2GRAY);

        // cv::imshow("left", gray_left);
        // cv::imshow("right", gray_right);

        // Upload grayscale images to GPU memory
        d_left_image.upload(gray_left);
        d_right_image.upload(gray_right);

        // Check if images are empty after conversion
        if (gray_left.empty() || gray_right.empty()) {
            std::cerr << "Error: Failed to convert images to grayscale." << std::endl;
            return cv::Mat();
        }

        // Create StereoBM object for CUDA
        cv::Ptr<cv::cuda::StereoBM> stereoBM_CUDA = cv::cuda::createStereoBM();

        // Check if stereoBM_CUDA object is valid
        if (stereoBM_CUDA.empty()) {
            std::cerr << "Error: Failed to create StereoBM object for CUDA." << std::endl;
            return cv::Mat();
        }

        // Set parameters for stereoBM_CUDA (if needed)
        stereoBM_CUDA->setBlockSize(9);  // Adjust parameters as needed

        // Compute disparity map on GPU
        cv::cuda::GpuMat d_disparity;
        stereoBM_CUDA->compute(d_left_image, d_right_image, d_disparity);

        // Download disparity map from GPU to CPU
        cv::Mat disparity;
        d_disparity.download(disparity);

        // Check if disparity map is empty after download
        if (disparity.empty()) {
            std::cerr << "Error: Failed to compute disparity map." << std::endl;
            return cv::Mat();
        }

        // Normalize the disparity map for visualization
        cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

        // Show the normalized disparity map
        cv::imshow("disparity", disparity);

        cv::waitKey(1);

        return disparity;
    }
}
