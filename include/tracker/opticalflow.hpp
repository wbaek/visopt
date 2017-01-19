#ifndef __TRACKER_OPTICALFLOW_HPP__
#define __TRACKER_OPTICALFLOW_HPP__

#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
    class Opticalflow : public Base {
        public:
            Opticalflow(const cv::Mat& intrinsic) {
                this->termCriteria = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 20, 0.03);

                this->intrinsic = intrinsic;
                this->initPose = cv::Mat::eye(3, 4, CV_64FC1);
                this->trackAble = false;

                this->rotationKF.init(6, 3, 0, CV_64F);
                this->translationKF.init(6, 3, 0, CV_64F);
                this->rotationKF.transitionMatrix = (cv::Mat_<double>(6, 6) << 1,0,0,1,0,0, 0,1,0,0,1,0, 0,0,1,0,0,1, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
                this->translationKF.transitionMatrix = (cv::Mat_<double>(6, 6) << 1,0,0,1,0,0, 0,1,0,0,1,0, 0,0,1,0,0,1, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1);
                this->rotationKF.measurementMatrix = cv::Mat::eye(3, 6, CV_64F);
                this->translationKF.measurementMatrix = cv::Mat::eye(3, 6, CV_64F);

            }
            virtual ~Opticalflow() {
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

            cv::KalmanFilter translationKF;
            cv::KalmanFilter rotationKF;

        public:
            std::vector<cv::Point2f> prevPoints;
            std::vector<cv::Point2f> currPoints;
            std::vector<cv::Point2f> initPoints;
            std::vector<cv::Point3f> mapPoints;
            std::vector<int> types; // 0=mapping, 1=tracking

            cv::Mat initPose;
            cv::Mat currPose;
    };
}

#endif //__TRACKER_OPTICALFLOW_HPP__
