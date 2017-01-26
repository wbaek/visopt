#ifndef __POSE_HOMOGRAPHY_HPP__
#define __POSE_HOMOGRAPHY_HPP__

#include "pose/pose.hpp"

namespace visopt{
class Homography : public Pose {
    public:
        virtual ~Homography() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const;
};
}

#endif //__POSE_HOMOGRAPHY_HPP__
