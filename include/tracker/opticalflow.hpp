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

                double dt = 1.0/30.0; // time between measurements (1/FPS)
				this->kalmanFilter.init(18, 6, 0, CV_64F);
                cv::setIdentity(this->kalmanFilter.processNoiseCov, cv::Scalar::all(1e-5)); 
                cv::setIdentity(this->kalmanFilter.measurementNoiseCov, cv::Scalar::all(1e-1));
                cv::setIdentity(this->kalmanFilter.errorCovPost, cv::Scalar::all(1));
                cv::setIdentity(this->kalmanFilter.transitionMatrix);

				/* DYNAMIC MODEL */
				// [1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0 0 0]
				// [0 1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0 0]
				// [0 0 1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0]
				// [0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2 0 0]
				// [0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2 0]
				// [0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2]
				// [0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0]
				// [0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0]
				// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt]
				// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0]
				// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0]
				// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1]
				// position
				this->kalmanFilter.transitionMatrix.at<double>(0,3) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(1,4) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(2,5) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(3,6) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(4,7) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(5,8) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(0,6) = 0.5*std::pow(dt,2);
				this->kalmanFilter.transitionMatrix.at<double>(1,7) = 0.5*std::pow(dt,2);
				this->kalmanFilter.transitionMatrix.at<double>(2,8) = 0.5*std::pow(dt,2);

				// orientation
				this->kalmanFilter.transitionMatrix.at<double>(9,12) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(10,13) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(11,14) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(12,15) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(13,16) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(14,17) = dt;
				this->kalmanFilter.transitionMatrix.at<double>(9,15) = 0.5*std::pow(dt,2);
				this->kalmanFilter.transitionMatrix.at<double>(10,16) = 0.5*std::pow(dt,2);
				this->kalmanFilter.transitionMatrix.at<double>(11,17) = 0.5*std::pow(dt,2);

				/* MEASUREMENT MODEL */
				// [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
				// [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
				// [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
				// [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
				this->kalmanFilter.measurementMatrix.at<double>(0,0) = 1; // x
				this->kalmanFilter.measurementMatrix.at<double>(1,1) = 1; // y
				this->kalmanFilter.measurementMatrix.at<double>(2,2) = 1; // z
				this->kalmanFilter.measurementMatrix.at<double>(3,9) = 1; // roll
				this->kalmanFilter.measurementMatrix.at<double>(4,10) = 1; // pitch
				this->kalmanFilter.measurementMatrix.at<double>(5,11) = 1; // yaw
            }
            virtual ~Opticalflow() {
                delete this->reconstructor;
            }

            virtual void setImage(const cv::Mat& image);
            virtual void track();
            virtual bool updatePose();
            virtual void extract();
            virtual void reconstruct();
            virtual void draw(cv::Mat& image, const bool debug=false) const;
            virtual void swap();

            virtual void remove(const std::vector<unsigned char>& status);
            virtual void append(const std::vector<cv::Point2f>& points);

            virtual const std::vector<cv::Point3f> getMapPoints() const {
                return this->mapPoints;
            }
            virtual const cv::Mat getPose() const {
                return this->currPose;
            }
            virtual const cv::Mat getGLPose() const {
                return this->glPose;
            }

        protected:
            cv::TermCriteria termCriteria;

            cv::Mat intrinsic;
            cv::Mat prevImage;
            cv::Mat currImage;
            bool trackAble;
            Triangulator* reconstructor;
            cv::KalmanFilter kalmanFilter;

            std::vector<cv::Point2f> prevPoints;
            std::vector<cv::Point2f> currPoints;
            std::vector<cv::Point2f> initPoints;
            std::vector<int> types; // 0=mapping, 1=tracking
            std::vector<cv::Point3f> mapPoints;

            cv::Mat initPose;
            cv::Mat currPose;
            cv::Mat glPose;
    };
}

#endif //__TRACKER_OPTICALFLOW_HPP__
