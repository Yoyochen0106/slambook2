
#include "myslam/util.h"

namespace myslam {

inline double map(double f1, double f2, double t1, double t2, double v) {
    return (v - f1) / (f2 - f1) * (t2 - t1) + t1;
}

template <typename T>
inline T clamp(float v, float min, float max) {
    if (v > max) return max;
    if (v < min) return min;
    return v;
}

inline std::string getImgType(cv::Mat img)
{
    int imgTypeInt = img.type();
    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
                             CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
                             CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                             CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                             CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                             CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                             CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

    std::string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                             "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                             "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                             "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                             "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                             "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                             "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

    for(int i=0; i<numImgTypes; i++)
    {
        if(imgTypeInt == enum_ints[i]) return enum_strings[i];
    }
    return "unknown image type";
}

DataPlot::DataPlot() {
}

void DataPlot::Setup() {
    cache_.create(size_.height, size_.width, CV_8UC3);
    cache_dst_.create(size_.height, size_.width, CV_8UC3);
    std::cout << getImgType(cache_) << "|" << getImgType(cache_dst_) << std::endl;

    cv::Rect right_bar(size_.width - item_w_, 0, item_w_, size_.height);
    int box_over_under_height = (int)(size_.height * 0.1);

    box_over_ = right_bar;
    box_over_.height = box_over_under_height;

    box_under_ = box_over_;
    box_under_.y = size_.height - box_over_under_height;

    bar_data_ = right_bar;
    bar_data_.height = size_.height - 2 * box_over_under_height;
    bar_data_.y = box_over_under_height;
}

void DataPlot::Push(double value) {
    int w = size_.width;
    int h = size_.height;
    cv::Scalar color(255, 255, 255);

    // Shift
    cv::Rect src(item_w_, 0, w - item_w_, h);
    cv::Rect dst(src);
    dst.x = 0;
    cache_(src).copyTo(cache_dst_(dst));

    // Draw new data
    double over = 0;
    double under = 0;
    cv::Rect value_bar(bar_data_);
    double value_display = value;
    if (value_display > value_max_) {
        over = clamp<double>(1.0 * (value_display - value_max_) / (value_max_ - value_min_), 0.0, 1.0);
        value_display = value_max_;
        std::cout << over << "|" << under << std::endl;
    } else if (value_display < value_min_) {
        under = clamp<double>(1.0 * (value_min_ - value_display) / (value_max_ - value_min_), 0.0, 1.0);
        value_display = value_min_;
        std::cout << over << "|" << under << std::endl;
    }
    int top = (int)map(value_max_, value_min_, bar_data_.y, bar_data_.y + bar_data_.height, value_display);
    top = clamp<int>(top, bar_data_.y, bar_data_.y + bar_data_.height);
    value_bar.height = value_bar.height - top + value_bar.y;
    value_bar.y = top;
    cv::rectangle(cache_dst_, bar_data_, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::rectangle(cache_dst_, value_bar, color, cv::FILLED);

    // Draw over/under indicator
    cv::Scalar color_over(
        int(255.0 * 0.2),
        int(255.0 * 0.2),
        int(255.0 * map(0.0, 1.0, 0.2, 1.0, over))
    );
    cv::Scalar color_under(
        int(255.0 * 0.2),
        int(255.0 * 0.2),
        int(255.0 * map(0.0, 1.0, 0.2, 1.0, under))
    );
    cv::rectangle(cache_dst_, box_over_, color_over, cv::FILLED);
    cv::rectangle(cache_dst_, box_under_, color_under, cv::FILLED);

    // Swap cache
    cv::Mat tmp = cache_;
    cache_ = cache_dst_;
    cache_dst_ = tmp;
}

void DataPlot::Draw(cv::Mat canvas, cv::Point topleft) {
    std::cout << "draw: " << getImgType(cache_) << "|" << getImgType(canvas) << std::endl;
    cache_.copyTo(canvas(cv::Rect(topleft, size_)));
}

}  // namespace myslam
