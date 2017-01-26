#include "pose/camera_pose.hpp"

using namespace visopt;
const cv::Mat CameraPose::calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
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

const cv::Mat CameraPose::calc(const std::vector<cv::Point3f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
    cv::Mat rvec, tvec;
    cv::solvePnPRansac(pt1, pt2, this->intrinsic, cv::noArray(),
            rvec, tvec,
            false, 100, 1.0, 0.99, status);

    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(pt1, rvec, tvec, this->intrinsic, cv::Mat(), reprojected);
    for(size_t i=0; i<status.size(); i++) {
        if(status[i] && cv::norm(cv::Mat(reprojected[i]), cv::Mat(pt2[i]), cv::NORM_L2) > 2.0) {
            status[i] = 0;
        }
    }

    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);

    cv::Mat_<double> R = rotation;
    cv::Mat_<double> t = tvec;
    cv::Matx34d pose(R(0,0), R(0,1), R(0,2), t(0),
                     R(1,0), R(1,1), R(1,2), t(1),
                     R(2,0), R(2,1), R(2,2), t(2) );

    return cv::Mat(pose);
}
