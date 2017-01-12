#ifndef __TRACKER_OPTICALFLOW_HPP__
#define __TRACKER_OPTICALFLOW_HPP__

#include "tracker/tracker.hpp"

namespace visopt {
    class Opticalflow : public Tracker {
        public:
            Opticalflow() : Tracker() {
                this->termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
            }
            virtual ~Opticalflow() {
            }

            virtual void init(const cv::Mat& image=cv::Mat());
            virtual void track(const cv::Mat& image);
            virtual void draw(cv::Mat& image) const;

            virtual const std::vector<cv::Point2f> extract(const cv::Mat& image) const;
        protected:
            std::vector<cv::Point2f> points[2];
            cv::TermCriteria termCriteria;
    };
}

#endif //__TRACKER_OPTICALFLOW_HPP__
