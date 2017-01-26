#include "tracker/tracker.hpp"

using namespace visopt;

void Tracker::init(const cv::Mat& image, const std::vector<cv::Point2f>& points, const std::vector<int>& indicies) {
    cv::cvtColor(image, this->images[Tracker::prev], cv::COLOR_BGR2GRAY);
    this->setPoints(points, indicies, Tracker::prev);
}

const cv::Mat Tracker::getImage(const Selector index) const {
    return this->images[index];
}
const std::vector<cv::Point2f> Tracker::getPoints(const Selector index) const {
    return this->points[index];
}
const std::vector<int> Tracker::getIndicies(const Selector index) const {
    return this->indicies[index];
}
void Tracker::setPoints(const std::vector<cv::Point2f>& points, const Selector index) {
    this->setPoints(points, std::vector<int>(), index);
}
void Tracker::setPoints(const std::vector<cv::Point2f>& points, const std::vector<int>& indicies, const Selector index) {
    this->points[index] = points;
    if(indicies.size() == 0) {
        this->indicies[index].clear();
        for(size_t i=0; i<points.size(); i++) {
            this->indicies[index].push_back( i );
        }
    } else {
        this->indicies[index] = indicies;
    }
}
void Tracker::append(const std::vector<cv::Point2f>& points, const Selector index, const double epsilon) {
    size_t pointsCount = this->points[index].size();
    for(size_t i=0; i<points.size(); i++) {
        bool found = false;
        for(size_t j=0; j<pointsCount; j++) {
            if( cv::norm(points[i] - this->points[index][j]) < epsilon) {
                found = true;
                break;
            }
        }
        if( found ) continue;
        this->points[index].push_back( points[i] );
        this->indicies[index].push_back( -1 );
    }
}

void Tracker::swap() {
    std::swap( images[0], images[1] );
    std::swap( points[0], points[1] );
    std::swap( indicies[0], indicies[1] );
}

void Tracker::remove(const std::vector<unsigned char>& status) {
    this->indicies[Tracker::prev].resize( status.size() );
    this->indicies[Tracker::curr].resize( status.size() );
    size_t k;
    for(size_t i=k=0; i<status.size(); i++) {
        if(!status[i]) continue;
        this->points[Tracker::prev][k] = this->points[Tracker::prev][i];
        this->points[Tracker::curr][k] = this->points[Tracker::curr][i];
        this->indicies[Tracker::prev][k] = this->indicies[Tracker::prev][i];
        this->indicies[Tracker::curr][k] = this->indicies[Tracker::curr][i];
        k++;
    }
    this->points[Tracker::prev].resize( k );
    this->points[Tracker::curr].resize( k );
    this->indicies[Tracker::prev].resize( k );
    this->indicies[Tracker::curr].resize( k );
}

void Tracker::draw(cv::Mat& image) {
    cv::Scalar color = cv::Scalar(0,0,255);
    for(size_t i=0; i<std::min(this->points[Tracker::prev].size(), this->points[Tracker::curr].size()); i++) {
        cv::circle(image, this->points[Tracker::curr][i], 3, cv::Scalar(0,0,255), -1, 8);
        cv::line(image, this->points[Tracker::prev][i], this->points[Tracker::curr][i], cv::Scalar(0,255,255), 1, 8);
    }
}

