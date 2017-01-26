#ifndef __POSE_FUNDAMENTAL_HPP__
#define __POSE_FUNDAMENTAL_HPP__

#include "pose/pose2d.hpp"

namespace visopt{
class Fundamental : public Pose2D {
    public:
        virtual ~Fundamental() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const;
};
}

#endif //__POSE_FUNDAMENTAL_HPP__
