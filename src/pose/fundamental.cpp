#include "pose/fundamental.hpp"

using namespace visopt;

const cv::Mat Fundamental::calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
    cv::Mat fundamental = cv::findFundamentalMat(
            pt1, pt2,
            cv::FM_RANSAC, /*param1|distance=*/2.0, /*param2|prob=*/0.99, /*inliers=*/status);
    return fundamental;
}
