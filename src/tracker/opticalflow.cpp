#include "tracker/opticalflow.hpp"

using namespace visopt;
OpticalFlow::OpticalFlow() : Tracker() {
    this->termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
}

const std::vector<unsigned char> OpticalFlow::track(const cv::Mat& image) {
    cv::cvtColor(image, this->images[Tracker::curr], cv::COLOR_BGR2GRAY);
    if(this->images[0].size() != this->images[1].size()) {
        //TODO: throw exception
        return std::vector<unsigned char>();
    }

    std::vector<unsigned char> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(this->images[Tracker::prev], this->images[Tracker::curr],
            this->points[Tracker::prev], this->points[Tracker::curr],
            status, err,
            cv::Size(15, 15), /*maxlevel=*/2,
            this->termCriteria,
            cv::OPTFLOW_LK_GET_MIN_EIGENVALS, /*minEigThreshold=*/0.001);

    return status;
}
