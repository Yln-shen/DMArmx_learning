#include "detector/detector.hpp"

#include <iostream>

namespace DM_ARMX_VISION {

cv::Mat CubeDetector::process(const cv::Mat& img_input)
{
    if (img_input.empty()) {
        std::cerr << "Error: Input image is empty." << std::endl;
        return cv::Mat();
    }

    img = img_input.clone();  // 深拷贝，独立内存
    cv::Mat working_img = img;

    cv::Mat hsv_img;
    cv::cvtColor(working_img, hsv_img, cv::COLOR_BGR2HSV);  // BGR 转 HSV

    cv::Mat mask;
    // lower/upper 是 cv::Scalar(H_min, S_min, V_min) 和 (H_max, S_max, V_max)
    cv::inRange(hsv_img, params.lower, params.upper, mask);
    return mask;
}

}  // namespace DM_ARMX_VISION
