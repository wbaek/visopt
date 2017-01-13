#ifndef __TRACKER_OPTICALFLOW_HPP__
#define __TRACKER_OPTICALFLOW_HPP__

#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
    class Opticalflow : public Base {
        public:
            Opticalflow() {
                this->termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);
            }
            virtual ~Opticalflow() {
            }

            virtual void setImage(const cv::Mat& image);
            virtual void track();
            virtual void extract();
            virtual void reconstruct();
            virtual void draw(cv::Mat& image) const;
            virtual void swap();

        protected:
            virtual void remove(const std::vector<unsigned char>& status);
            virtual void append(const std::vector<cv::Point2f>& points);

        protected:
            cv::TermCriteria termCriteria;

            cv::Mat prevImage;
            cv::Mat currImage;

            std::vector<cv::Point2f> prevPoints;
            std::vector<cv::Point2f> currPoints;
            std::vector<cv::Point2f> initPoints;
            std::vector<cv::Point3f> mapPoints;
            std::vector<int> types; // 0=mapping, 1=tracking
    };
}

#endif //__TRACKER_OPTICALFLOW_HPP__
