#include "pose/homography.hpp"

using namespace visopt;

const cv::Mat Homography::calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
    cv::Mat homography = cv::findHomography(
            pt1, pt2,
            cv::RANSAC, 2.0, status);
    return homography;
}
