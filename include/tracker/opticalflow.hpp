#ifndef __TRACKER_OPTICALFLOW_HPP__
#define __TRACKER_OPTICALFLOW_HPP__

#include <opencv2/opencv.hpp>
#include "base.hpp"
#include "reconstructor/triangulator.hpp"

namespace visopt {
    class Opticalflow : public Base {
        public:
            Opticalflow(const cv::Mat& intrinsic) {
                this->termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);

                this->intrinsic = intrinsic;
                this->initPose = cv::Mat::eye(3, 4, CV_64FC1);
                this->trackAble = false;

                this->reconstructor = new Triangulator(intrinsic);
            }
            virtual ~Opticalflow() {
                delete this->reconstructor;
            }

            virtual void setImage(const cv::Mat& image);
            virtual void track();
            virtual bool updatePose();
            virtual void extract();
            virtual void reconstruct();
            virtual void draw(cv::Mat& image) const;
            virtual void swap();

            virtual void remove(const std::vector<unsigned char>& status);
            virtual void append(const std::vector<cv::Point2f>& points);

        protected:
            cv::TermCriteria termCriteria;

            cv::Mat intrinsic;
            cv::Mat prevImage;
            cv::Mat currImage;
            bool trackAble;

            std::vector<cv::Point2f> prevPoints;
            std::vector<cv::Point2f> currPoints;
            std::vector<cv::Point2f> initPoints;
            std::vector<cv::Point3f> mapPoints;
            std::vector<int> types; // 0=mapping, 1=tracking

            cv::Mat initPose;
            cv::Mat currPose;

            Triangulator* reconstructor;
    };
}

#endif //__TRACKER_OPTICALFLOW_HPP__
