#include "pose/camera2d.hpp"

using namespace visopt;
const cv::Mat Camera2D::calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
    cv::Mat essential = Essential::calc(pt1, pt2, status);

    double focalLength = (this->intrinsic.at<double>(0,0) + this->intrinsic.at<double>(1,1)) / 2.0;
    cv::Point2d priciplePoint(this->intrinsic.at<double>(0, 2), this->intrinsic.at<double>(1, 2));
    cv::Mat_<double> R, t;
    cv::recoverPose(essential, pt1, pt2, R, t, focalLength, priciplePoint);

    cv::Matx34d pose(R(0,0), R(0,1), R(0,2), t(0),
            R(1,0), R(1,1), R(1,2), t(1),
            R(2,0), R(2,1), R(2,2), t(2) );
    return cv::Mat(pose);
}
