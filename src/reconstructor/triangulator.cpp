#include "reconstructor/triangulator.hpp"

#define CERES_FOUND 1
#include <opencv2/sfm.hpp>
#include <utils/string.hpp>

using namespace visopt;
const cv::Mat Triangulator::pose(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, std::vector<unsigned char>& status) {
    cv::Mat essential = cv::findEssentialMat( points1, points2, this->intrinsic,
            cv::RANSAC, 0.99, 1.0, status );
    if(std::abs(cv::determinant(essential)) > 1e-7) {
        throw std::logic_error(instant::Utils::String::Format("determinant is not zero det(E)=%f", cv::determinant(essential)));
    }

    double focalLength = this->intrinsic.at<double>(0);
    cv::Point2d priciplePoint(this->intrinsic.at<double>(0, 2), this->intrinsic.at<double>(1, 2));
    cv::Mat_<double> R, t;
    cv::recoverPose(essential, points1, points2, R, t, focalLength, priciplePoint);
    
    cv::Matx34d pose(R(0,0), R(0,1), R(0,2), t(0),
            R(1,0), R(1,1), R(1,2), t(1),
            R(2,0), R(2,1), R(2,2), t(2) );
    return cv::Mat(pose);
}
 
const std::vector<cv::Point3f> Triangulator::reconstruct(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const cv::Mat& pose1, const cv::Mat& pose2) {
    cv::Mat reconstructedPoints;
    cv::triangulatePoints(this->intrinsic*pose1, this->intrinsic*pose2, points1, points2, reconstructedPoints );

    std::vector<cv::Point3f> results;
    for(size_t i=0; i<reconstructedPoints.size().width; i++) {
        cv::Mat_<double> p = reconstructedPoints.col(i);
        results.push_back( cv::Point3f( p(0)/p(3), p(1)/p(3), p(2)/p(3) ) );
    }
    return results;
}
