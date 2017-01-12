#include "tracker/tracker.hpp"

using namespace visopt;
void Tracker::draw(cv::Mat& image) const {
    double m[4][4] = {{0.0, 0.0, 0.0, 1.0}, {1.0, 0.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 1.0}, {0.0, 0.0, -1.0, 1.0}};
    cv::Mat axis(4, 4, CV_64F, m);
    cv::Mat points = this->homography * axis.t();

    std::vector<cv::Scalar> colors(3);
    colors[0]=cv::Scalar(0, 0, 255); colors[1]=cv::Scalar(0, 255, 0); colors[2]=cv::Scalar(255, 0, 0);
    for(size_t i=0; i<3; i++) {
        double z = points.at<double>(2, 0);
        cv::Point2f pt1( points.at<double>(0, 0)/z, points.at<double>(1, 0)/z );
        z = points.at<double>(2, i+1);
        cv::Point2f pt2( points.at<double>(0, i+1)/z, points.at<double>(1, i+1)/z );

        cv::line(image, pt1, pt2, colors[i], 3, 8);
    }

}
