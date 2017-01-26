#include "extractor/goodfeaturetotrack.hpp"

using namespace visopt;
GoodFeatureToTrack::GoodFeatureToTrack() {
    this->termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
}

const std::vector<cv::Point2f> GoodFeatureToTrack::extract(const cv::Mat& image) const {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> points;
    cv::goodFeaturesToTrack(gray, points, 200, 0.01, 10, cv::Mat(), 2, 0, 0.04);
    cv::cornerSubPix(gray, points, cv::Size(10, 10), cv::Size(-1, -1), this->termCriteria);
    return points;
}
