#ifndef __TRACKER_OPTICALFLOW_HPP__
#define __TRACKER_OPTICALFLOW_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "tracker/tracker.hpp"

namespace visopt{
class OpticalFlow : public Tracker {
    public:
        OpticalFlow();
        virtual ~OpticalFlow() {
        }
        virtual const std::vector<unsigned char> track(const cv::Mat& image);

    protected:
        cv::TermCriteria termCriteria;
};
}

#endif //__TRACKER_OPTICALFLOW_HPP__
