#ifndef __POSE_CAMERA_POSE_HPP__
#define __POSE_CAMERA_POSE_HPP__

#include "pose/essential.hpp"

namespace visopt{
class CameraPose : public Essential {
    public:
        CameraPose(const cv::Mat& intrinsic) : Essential(intrinsic) {
        }
        virtual ~CameraPose() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const;
        virtual const cv::Mat calc(const std::vector<cv::Point3f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const;

};
}

#endif //__POSE_CAMERA_POSE_HPP___
