#include "tracker/opticalflow.hpp"

using namespace visopt;
void Opticalflow::setImage(const cv::Mat& image) {
    cv::cvtColor(image, this->currImage, cv::COLOR_BGR2GRAY);
}

void Opticalflow::track() {
    if( this->prevImage.size() != this->currImage.size() )
        return;
    std::vector<unsigned char> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(this->prevImage, this->currImage,
            this->prevPoints, this->currPoints,
            status, err, cv::Size(31, 31), 3, this->termCriteria, 0, 0.001);
    this->remove( status );

    // calc fundamantal matrix & motion
    cv::Mat fundamental = cv::findFundamentalMat(
            this->prevPoints, this->currPoints,
            cv::FM_RANSAC, 0.1, 0.99, status);
    this->remove( status );
}

void Opticalflow::extract() {
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(this->currImage, points, 500, 0.01, 10, cv::Mat(), 3, 0, 0.04);
    cv::cornerSubPix(this->currImage, points, cv::Size(10, 10), cv::Size(-1, -1), this->termCriteria);

    this->append( points );
}

void Opticalflow::reconstruct() {
}

void Opticalflow::draw(cv::Mat& image) const {
    cv::Scalar colors[2]; colors[0] = cv::Scalar(0,0,255); colors[1] = cv::Scalar(0,255,255);
    for(size_t i=0; i<this->currPoints.size(); i++) {
        cv::circle(image, this->currPoints[i], 3, colors[this->types[i]], -1, 8);
        cv::line(image, this->currPoints[i], this->prevPoints[i], cv::Scalar(0,255,0), 1, 8);
    }
}

void Opticalflow::swap() {
    std::swap( this->prevImage, this->currImage );
    std::swap( this->prevPoints, this->currPoints );
}

void Opticalflow::append(const std::vector<cv::Point2f>& points) {
    for(size_t i=0; i<points.size(); i++) {
        this->prevPoints.push_back( points[i] );
        this->currPoints.push_back( points[i] );
        this->initPoints.push_back( points[i] );
        this->mapPoints.push_back( cv::Point3f(-1, -1, -1) );
        this->types.push_back( 0 );
    }
}

void Opticalflow::remove(const std::vector<unsigned char>& status) {
    size_t k;
    for(size_t i=k=0; i<status.size(); i++) {
        if(!status[i])
            continue;
        this->prevPoints[k] = this->prevPoints[i];
        this->currPoints[k] = this->currPoints[i];
        this->initPoints[k] = this->initPoints[i];
        this->mapPoints[k]  = this->mapPoints[i];
        this->types[k]      = this->types[i];
        k++;
    }
}
