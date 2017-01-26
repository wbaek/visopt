#ifndef __POSE_POSE_HPP__
#define __POSE_POSE_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class Pose : public Base {
    public:
        virtual ~Pose() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
            throw std::logic_error("Pose calc 2D-to-2D method is not implemented");
        }
        virtual const cv::Mat calc(const std::vector<cv::Point3f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
            throw std::logic_error("Pose calc 3D-to-2D method is not implemented");
        }
};
}

#endif //__POSE_POSE_HPP__
