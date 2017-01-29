#ifndef __POSE_KALMAN_FILTER_HPP__
#define __POSE_KALMAN_FILTER_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class KalmanFilter : public Base {
    public:
        KalmanFilter();
        virtual ~KalmanFilter() {
        }

        const cv::Mat predict();
        const cv::Mat correct(const cv::Mat& pose);
 
        static const std::vector<cv::Mat> decompose(const cv::Mat& pose);
        static const cv::Mat pose(const cv::Mat& rvec, const cv::Mat& tvec);

    protected:
        cv::KalmanFilter rotationKF;
        cv::KalmanFilter translationKF;        
};
}

#endif //__POSE_KALMAN_FILTER_HPP__
