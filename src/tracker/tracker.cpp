#include "tracker/tracker.hpp"

using namespace visopt;
void Tracker::draw(cv::Mat& image) const {
    double scale = 100.0;
    std::vector<cv::Matx31d> axis;
    axis.push_back( cv::Matx31d(0, 0, 0) * scale);
    axis.push_back( cv::Matx31d(1, 0, 0) * scale);
    axis.push_back( cv::Matx31d(0, 1, 0) * scale);
    axis.push_back( cv::Matx31d(0, 0, 1) * scale);
    std::vector<cv::Scalar> colors;
    colors.push_back(cv::Scalar(0, 0, 0));
    colors.push_back(cv::Scalar(0, 0, 255));
    colors.push_back(cv::Scalar(0, 255, 0));
    colors.push_back(cv::Scalar(255, 0, 0));

    std::vector<cv::Point2f> points = this->project(axis);
    for(size_t i=1; i<points.size(); i++) {
        cv::line(image, points[0], points[i], colors[i], 3, 8);
    }

    std::cout << "zero pt=" << points[0] << std::endl;
}

std::tuple<cv::Mat, cv::Mat> Tracker::calcMotion(const cv::Mat& fundamental) const {
    cv::Mat essential = this->intrinsic.t() * fundamental * this->intrinsic;
    if(std::abs(cv::determinant(essential)) > 1e-7) {
        std::cerr << "determinant is not zero det(E)=" << cv::determinant(essential) << std::endl;
        return std::tuple<cv::Mat, cv::Mat>( cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F) );
    }
    cv::SVD svd(essential, cv::SVD::MODIFY_A);
    double singular_values_ratio = std::abs(svd.w.at<double>(0) / svd.w.at<double>(1));
    if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio;
    if (singular_values_ratio < 0.7) {
        std::cerr << "singular values are too far" << singular_values_ratio << std::endl;
        return std::tuple<cv::Mat, cv::Mat>( cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F) );

    }

    cv::Matx33d W( 0, -1, 0,   1, 0, 0,  0, 0, 1);
    cv::Matx33d Wt(0,  1, 0,  -1, 0, 0,  0, 0, 1);
    cv::Mat_<double> R = svd.u * cv::Mat(W) * svd.vt;
    cv::Mat_<double> t = svd.u.col(2);

    return std::tuple<cv::Mat, cv::Mat>( cv::Mat(R), cv::Mat(t) );
}

void Tracker::updateMotion(const cv::Mat& rotation, const cv::Mat& translation) {
    this->translation = rotation * this->translation + translation;
    this->rotation = rotation * this->rotation;
}

std::vector<cv::Point2f> Tracker::project(const std::vector<cv::Matx31d>& points3D) const {
    std::vector<cv::Point2f> results;
    for(size_t i=0; i<points3D.size(); i++) {
        cv::Mat_<double> p = (this->rotation * cv::Mat(points3D[i]) + this->translation);
        cv::Mat points2D = cv::Mat(p) / p(2);
        cv::Mat_<double> point = this->intrinsic * points2D;
        results.push_back( cv::Point2f(point(0), point(1)) );
    }
    return results;
}
