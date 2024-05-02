// disparity_calculator.h
#ifndef DISPARITY_CALCULATOR_H
#define DISPARITY_CALCULATOR_H

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudastereo.hpp>
#include <iostream>

namespace DisparityCalculator {
class DisparityCalculator {
public:
    cv::Mat computeDisparityMap(const cv::Mat& left_image, const cv::Mat& right_image);
};
}

#endif

