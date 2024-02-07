// disparity_calculator.h
#ifndef DISPARITY_CALCULATOR_H
#define DISPARITY_CALCULATOR_H

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>

class DisparityCalculator {
public:
    DisparityCalculator();
    ~DisparityCalculator();

    cv::Mat computeDisparityMap(const cv::Mat& leftImage, const cv::Mat& rightImage);
};

#endif

