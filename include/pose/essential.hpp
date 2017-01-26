#ifndef __POSE_ESSENTIAL_HPP__
#define __POSE_ESSENTIAL_HPP__

#include "pose/pose2d.hpp"

namespace visopt{
class Essential : public Pose2D {
    public:
        Essential(const cv::Mat& intrinsic) {
            this->intrinsic = intrinsic;
        }
        virtual ~Essential() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const;

    protected:
        cv::Mat intrinsic;
};
}

#endif //__POSE_ESSENTIAL_HPP__
