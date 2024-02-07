// disparity_calculator.cpp
#include "disparity_calculator.h"

namespace ImageConverter {

    cv::Mat DisparityCalculator::computeDisparityMap(const cv::Mat& leftImage, const cv::Mat& rightImage) 
    {
        // Upload images to GPU memory
        cv::cuda::GpuMat d_leftImage, d_rightImage;
        d_leftImage.upload(leftImage);
        d_rightImage.upload(rightImage);

        // Create a StereoSGBM object
        cv::Ptr<cv::cuda::StereoSGBM> stereoSGBM = cv::cuda::StereoSGBM::create();

        // Set parameters (if needed)
        stereoSGBM->setPreFilterCap(63);
        stereoSGBM->setBlockSize(9);  // Odd number, usually in the range 5-11
        stereoSGBM->setP1(8 * 1 * stereoSGBM->getBlockSize() * stereoSGBM->getBlockSize());
        stereoSGBM->setP2(32 * 1 * stereoSGBM->getBlockSize() * stereoSGBM->getBlockSize());
        stereoSGBM->setMinDisparity(0);
        stereoSGBM->setNumDisparities(64);  // Must be divisible by 16
        stereoSGBM->setUniquenessRatio(10);
        stereoSGBM->setSpeckleWindowSize(100);
        stereoSGBM->setSpeckleRange(32);
        stereoSGBM->setDisp12MaxDiff(1);

        // Compute disparity map
        cv::cuda::GpuMat d_disparity;
        stereoSGBM->compute(d_leftImage, d_rightImage, d_disparity);

        // Download disparity map from GPU to CPU
        cv::Mat disparity;
        d_disparity.download(disparity);

        return disparity;
    }
}
