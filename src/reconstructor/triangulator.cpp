#include "reconstructor/triangulator.hpp"

using namespace visopt;
const std::vector<cv::Point3f> Triangulator::reconstruct(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2) {
    cv::Mat fundamental = cv::findFundamentalMat( points1, points2, CV_FM_RANSAC, 3.0, 0.99 );
    cv::Mat essential = this->intrinsic.t() * fundamental * this->intrinsic;
    cv::SVD svd(essential);

    cv::Matx33d W(0, -1, 0,  1, 0, 0,  0, 0, 1);

    cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt;
    cv::Mat_<double> t = svd.u.col( 2 );

    cv::Mat reconstructedPoints;

    cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1 );
    cv::Matx34d P2(R(0,0), R(0,1), R(0,2), t(0),
                  R(1,0), R(1,1), R(1,2), t(1),
                  R(2,0), R(2,1), R(2,2), t(2) );
    cv::triangulatePoints( P1, cv::Mat(P2), points1, points2, reconstructedPoints );
    std::vector<cv::Point3f> results;
    cv::convertPointsHomogeneous(reconstructedPoints.reshape(4,1), results);
    return results;
}
