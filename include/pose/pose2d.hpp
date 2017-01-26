#ifndef __POSE_POSE2D_HPP__
#define __POSE_POSE2D_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class Pose2D : public Base {
    public:
        virtual ~Pose2D() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const = 0;
};
}

#endif //__POSE_POSE2D_HPP__
