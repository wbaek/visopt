#include "pose/essential.hpp"

using namespace visopt;
const cv::Mat Essential::calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
    cv::Mat essential = cv::findEssentialMat( pt1, pt2, this->intrinsic,
            cv::RANSAC, /*prob=*/0.99, /*threshold=*/2.0, /*inliers=*/status );
    if(std::abs(cv::determinant(essential)) > 1e-3) {
        throw std::logic_error(instant::Utils::String::Format("determinant is not zero det(E)=%f", cv::determinant(essential)));
    }
    return essential;
}
