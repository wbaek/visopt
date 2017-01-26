#ifndef __CORE_KEYFRAME_HPP__
#define __CORE_KEYFRAME_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
class KeyFrame : public Base {
    public:

    public:
        cv::Mat image;
        std::vector<cv::Point2f> points;
        std::vector<int> indicies;
};
}

#endif //__CORE_KEYFRAME_HPP__
