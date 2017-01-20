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

    if( this->trackAble == false ) {
        cv::Mat fundamental = cv::findFundamentalMat(
                this->prevPoints, this->currPoints,
                cv::FM_RANSAC, 1.0, 0.99, status);
        this->remove( status );
    }
}

bool Opticalflow::updatePose() {
    if( this->trackAble == true ) {
        std::vector<cv::Point3f> map;
        std::vector<cv::Point2f> points;
        for(size_t i=0; i<this->types.size(); i++) {
            if(this->types[i] == 1) {
                map.push_back( this->mapPoints[i] );
                points.push_back( this->currPoints[i] );
            }
        }
        std::vector<unsigned char> status;
        cv::Mat rotation, translation;
        cv::solvePnPRansac(map, points, this->intrinsic, cv::noArray(),
                rotation, translation,
                false, 100, 1.0, 0.99, status);
        this->remove( status );

        std::vector<cv::Point2f> reprojected;
        cv::projectPoints(map, rotation, translation, this->intrinsic, cv::Mat(), reprojected);
        float reprejectError = cv::norm(cv::Mat(reprojected), cv::Mat(points), cv::NORM_L2) / reprojected.size();

        std::vector<cv::Mat> measures;
        measures.push_back( translation );
        measures.push_back( rotation );
        this->kalmanFilter.predict();
        cv::Mat measure;
        cv::vconcat(measures, measure); 
        cv::Mat estimated = this->kalmanFilter.correct( measure );
        estimated = estimated.reshape(1, 6);
        translation = estimated.row(0);
        rotation = estimated.row(3);
        
        cv::Rodrigues(rotation, rotation);
        cv::Mat_<double> R = rotation;
        cv::Mat_<double> t = translation;
        cv::Matx34d pose(R(0,0), R(0,1), R(0,2), t(0),
                         R(1,0), R(1,1), R(1,2), t(1),
                         R(2,0), R(2,1), R(2,2), t(2) );
        this->currPose = cv::Mat(pose);
    }
    return this->trackAble;
}

void Opticalflow::extract() {
    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(this->currImage, points, 500, 0.01, 10, cv::Mat(), 3, 0, 0.04);
    cv::cornerSubPix(this->currImage, points, cv::Size(10, 10), cv::Size(-1, -1), this->termCriteria);

    this->append( points );
}

void Opticalflow::reconstruct() {
    cv::Mat p1, p2;
    if(!this->trackAble) { 
        std::vector<unsigned char> status;
        p1 = cv::Mat::eye(3, 4, CV_64F);
        p2 = this->reconstructor->pose( this->initPoints, this->currPoints, status );
        this->remove( status );
    } else {
        p1 = this->initPose;
        p2 = this->currPose;
    }
    std::vector<unsigned char> status;
    this->mapPoints = this->reconstructor->reconstruct( this->initPoints, this->currPoints, p1, p2, status );
    this->remove( status );
    this->initPose = p2;

    for(size_t i=0; i<this->types.size(); i++) {
        this->types[i] = 1;
    }

    this->initPoints = this->currPoints;
    this->trackAble = true;
}

void Opticalflow::draw(cv::Mat& image) const {
    cv::Scalar colors[2]; colors[0] = cv::Scalar(0,0,255); colors[1] = cv::Scalar(0,255,0);
    for(size_t i=0; i<this->currPoints.size(); i++) {
        cv::circle(image, this->currPoints[i], 3, colors[this->types[i]], -1, 8);
        cv::line(image, this->currPoints[i], this->prevPoints[i], cv::Scalar(0,255,255), 1, 8);
        cv::line(image, this->currPoints[i], this->initPoints[i], cv::Scalar(255,0,255), 1, 8);
    }
    if(this->trackAble && this->currPose.size().width * this->currPose.size().height >= 12) {
        cv::Matx44d axis(0.0, 10.0, 0.0, 0.0,
                         0.0, 0.0, 10.0, 0.0,
                         10.0, 10.0, 10.0, 20.0,
                         1.0, 1.0, 1.0, 1.0);
        cv::Mat projected = this->intrinsic * this->currPose * cv::Mat(axis);

        std::vector<cv::Scalar> colors;
        colors.push_back( cv::Scalar(0,0,0) );
        colors.push_back( cv::Scalar(0,0,255) );
        colors.push_back( cv::Scalar(0,255,0) );
        colors.push_back( cv::Scalar(255,0,0) );
        for(size_t j=1; j<4; j++) {
            cv::Point3f _p1( projected.col(0) );
            cv::Point3f _p2( projected.col(j) );
            cv::Point2f p1 = cv::Point2f(_p1.x/_p1.z, _p1.y/_p1.z);
            cv::Point2f p2 = cv::Point2f(_p2.x/_p2.z, _p2.y/_p2.z);

            cv::line(image, p1, p2, colors[j], 3, 8);
        }
    }
}

void Opticalflow::swap() {
    std::swap( this->prevImage, this->currImage );
    std::swap( this->prevPoints, this->currPoints );
}

void Opticalflow::append(const std::vector<cv::Point2f>& points) {
    for(size_t i=0; i<points.size(); i++) {
        bool found = false;
        for(size_t j=0; j<this->currPoints.size(); j++) {
            if( cv::norm(points[i] - this->currPoints[j]) < 3.0 ) {
                found = true;
                break;
            }
        }
        if( found )
            continue;

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
