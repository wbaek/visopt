#ifndef __POSE_CAMERA2D_HPP__
#define __POSE_CAMERA2D_HPP__

#include "pose/essential.hpp"

namespace visopt{
class Camera2D : public Essential {
    public:
        Camera2D(const cv::Mat& intrinsic) : Essential(intrinsic) {
        }
        virtual ~Camera2D() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const;
};
}

#endif //__POSE_CAMERA2D_HPP__
