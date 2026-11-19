#ifndef DETECTOR__DETECTOR_HPP_
#define DETECTOR__DETECTOR_HPP_

#include <opencv2/opencv.hpp>

namespace DM_ARMX_VISION {

class Cube {
public:
    int cx;
    int cy;
    int cz;
    int orientation_x;
    int orientation_y;
    int orientation_z;
    int orientation_w;
    int color;
    int lenth;
    int width;
};

struct Params {
    // HSV 颜色阈值，默认检测红色（OpenCV 的 H 范围 0~180）
    cv::Scalar lower{0, 100, 80};
    cv::Scalar upper{10, 255, 255};
};

class CubeDetector {
public:
    cv::Mat img;
    cv::Mat img_binary;
    cv::Mat img_cube;

    int binary_val = 100;

    Params params;  // 每个 CubeDetector 对象按值持有一份完整的检测参数配置

    cv::Mat process(const cv::Mat& img_input);
};

}  // namespace DM_ARMX_VISION

#endif  // DETECTOR__DETECTOR_HPP_
