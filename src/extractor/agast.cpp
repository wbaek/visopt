#include "extractor/agast.hpp"

using namespace visopt;
Agast::Agast(const int threshold, const bool nonmaxSupression, const int gaussianBlur) {
    this->threshold = threshold;
    this->nonmaxSupression = nonmaxSupression;
    this->gaussianBlur = gaussianBlur;
}

const std::vector<cv::Point2f> Agast::extract(const cv::Mat& image) const {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    if( this->gaussianBlur > 0 ) {
    	cv::GaussianBlur(gray, gray,
                         /*kernelSize=*/cv::Size(this->gaussianBlur, this->gaussianBlur),
                         /*sigmaX=*/1.0, /*sigmaY=*/1.0);
    }

    std::vector<cv::KeyPoint> points;
    cv::AGAST(gray, points, this->threshold, this->nonmaxSupression);
    //cv::FAST(gray, points, this->threshold, this->nonmaxSupression);

    std::vector<cv::Point2f> rv;
    cv::KeyPoint::convert(points, rv);
    return rv;
}
