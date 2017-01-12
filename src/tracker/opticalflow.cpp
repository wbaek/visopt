#include "tracker/opticalflow.hpp"

using namespace visopt;
void Opticalflow::init(const cv::Mat& image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    this->images.push_back( gray );

    this->points[0] = this->extract(gray);
}

void Opticalflow::track(const cv::Mat& image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(this->images[0], gray, this->points[0], this->points[1], status, err, cv::Size(31, 31), 3, this->termCriteria, 0, 0.001);

    // remote failure traked points
    size_t k;
    for(size_t i=k=0; i<this->points[1].size(); i++) {
        if(!status[i])
            continue;
        this->points[0][k] = this->points[0][i];
        this->points[1][k++] = this->points[1][i];
    }
    this->points[0].resize(k);
    this->points[1].resize(k);

    // calc homography
    cv::Mat delta = cv::findHomography( this->points[0], this->points[1], cv::RANSAC, 3.0 );
    double m[4][4] = {{delta.at<double>(0, 0), delta.at<double>(0, 1), 0.0, delta.at<double>(0, 2)},
                      {delta.at<double>(1, 0), delta.at<double>(1, 1), 0.0, delta.at<double>(1, 2)},
                      {delta.at<double>(2, 0), delta.at<double>(2, 1), 0.0, delta.at<double>(2, 2)},
                      {0.0, 0.0, 0.0, 1.0} };
    cv::Mat t = cv::Mat(4, 4, CV_64F, m);
    this->homography = t * this->homography;

    // remove outliers
    std::vector<cv::Point2f> points;
    cv::perspectiveTransform(this->points[0], points, delta);
    for(size_t i=k=0; i<this->points[0].size(); i++) {
        if( cv::norm(this->points[1][i] - points[i]) > 1.0 ) //outlier
            continue;
        this->points[0][k] = this->points[0][i];
        this->points[1][k++] = this->points[1][i];
    }

    // add new features in this frame
    points = this->extract(gray);
    for(int i=0; i<points.size(); i++) {
        bool found = false;
        for(k=0; k<this->points[1].size(); k++) {
            if( cv::norm(points[i] - this->points[1][k]) <= 5.0 ) {
                found = true;
                break;
            }
        }
        if(!found) {
            this->points[1].push_back( points[i] );
        }
    }

    // swap prev status
    this->images[0] = gray.clone();
    std::swap(this->points[0], this->points[1]);
}

void Opticalflow::draw(cv::Mat& image) const {
    for(size_t i=0; i<this->points[0].size(); i++) {
        if(i<this->points[1].size()) {
            cv::circle(image, this->points[0][i], 3, cv::Scalar(0,0,255), -1, 8);
        } else { // added features in this frame
            cv::circle(image, this->points[0][i], 3, cv::Scalar(0,255,255), -1, 8);
        }
    }
    for(size_t i=0; i<std::min(this->points[0].size(), this->points[1].size()); i++) {
        cv::line(image, this->points[0][i], this->points[1][i], cv::Scalar(0,255,0), 1, 8);
    }

    Tracker::draw(image);
}

const std::vector<cv::Point2f> Opticalflow::extract(const cv::Mat& gray) const {
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(gray, points, 500, 0.01, 10, cv::Mat(), 3, 0, 0.04);
    cv::cornerSubPix(gray, points, cv::Size(10, 10), cv::Size(-1, -1), this->termCriteria);

    return points;
}
