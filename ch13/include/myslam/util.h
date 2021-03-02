#pragma once
#ifndef MYSLAM_UTIL_H
#define MYSLAM_UTIL_H

#include <string>
#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"

namespace myslam {

template <typename T>
inline std::string to_string(cv::Point_<T> pt) {
    std::stringstream ss;
    ss << "[" << pt.x << "," << pt.y << "]";
    return ss.str();
}

class DataPlot {
public:
    typedef std::shared_ptr<DataPlot> Ptr;

    DataPlot();

    void Setup();

    void Push(double value);

    void Draw(cv::Mat canvas, cv::Point topleft);

    double value_max_, value_min_;
    int item_w_;
    cv::Size size_;

private:

    cv::Rect bar_data_;
    cv::Rect box_over_;
    cv::Rect box_under_;

    cv::Mat cache_;
    cv::Mat cache_dst_;

};

}  // namespace myslam

#endif  // MYSLAM_UTIL_H
