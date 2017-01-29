#include "pose/kalman_filter.hpp"

using namespace visopt;
KalmanFilter::KalmanFilter() {
    double dt = 1.0/30.0; //time between measurements (1/FPS)

    // rotation
    this->rotationKF.init(9, 3, 0, CV_64F);
    cv::setIdentity(this->rotationKF.processNoiseCov, cv::Scalar::all(1e-5)); 
    cv::setIdentity(this->rotationKF.measurementNoiseCov, cv::Scalar::all(1e-3));
    cv::setIdentity(this->rotationKF.errorCovPost, cv::Scalar::all(1));
    /* DYNAMIC MODEL */
    // [1 0 0 dt 0 0 dt2 0 0]
    // [0 1 0 0 dt 0 0 dt2 0]
    // [0 0 1 0 0 dt 0 0 dt2]
    // [0 0 0 1 0 0 dt 0 0  ]
    // [0 0 0 0 1 0 0 dt 0  ]
    // [0 0 0 0 0 1 0 0 dt  ]
    cv::setIdentity(this->rotationKF.transitionMatrix);
    this->rotationKF.transitionMatrix.at<double>(0,3) = dt;
    this->rotationKF.transitionMatrix.at<double>(1,4) = dt;
    this->rotationKF.transitionMatrix.at<double>(2,5) = dt;
    this->rotationKF.transitionMatrix.at<double>(3,6) = dt;
    this->rotationKF.transitionMatrix.at<double>(4,7) = dt;
    this->rotationKF.transitionMatrix.at<double>(5,8) = dt;
    this->rotationKF.transitionMatrix.at<double>(0,6) = 0.5*std::pow(dt,2);
    this->rotationKF.transitionMatrix.at<double>(1,7) = 0.5*std::pow(dt,2);
    this->rotationKF.transitionMatrix.at<double>(2,8) = 0.5*std::pow(dt,2);
    /* MEASUREMENT MODEL */
    // [1 0 0 0 0 0 0 0 0]
    // [0 1 0 0 0 0 0 0 0]
    // [0 0 1 0 0 0 0 0 0]
    cv::setIdentity(this->rotationKF.measurementMatrix, cv::Scalar::all(1));
    
    // translation
    this->translationKF.init(9, 3, 0, CV_64F);
    cv::setIdentity(this->translationKF.processNoiseCov, cv::Scalar::all(1e-5)); 
    cv::setIdentity(this->translationKF.measurementNoiseCov, cv::Scalar::all(1e-3));
    cv::setIdentity(this->translationKF.errorCovPost, cv::Scalar::all(1));
    /* DYNAMIC MODEL */
    // [1 0 0 dt 0 0 dt2 0 0]
    // [0 1 0 0 dt 0 0 dt2 0]
    // [0 0 1 0 0 dt 0 0 dt2]
    // [0 0 0 1 0 0 dt 0 0  ]
    // [0 0 0 0 1 0 0 dt 0  ]
    // [0 0 0 0 0 1 0 0 dt  ]
    // [0 0 0 0 0 0 1 0 0   ]
    // [0 0 0 0 0 0 0 1 0   ]
    // [0 0 0 0 0 0 0 0 1   ]
    cv::setIdentity(this->translationKF.transitionMatrix);
    this->translationKF.transitionMatrix.at<double>(0,3) = dt;
    this->translationKF.transitionMatrix.at<double>(1,4) = dt;
    this->translationKF.transitionMatrix.at<double>(2,5) = dt;
    this->translationKF.transitionMatrix.at<double>(3,6) = dt;
    this->translationKF.transitionMatrix.at<double>(4,7) = dt;
    this->translationKF.transitionMatrix.at<double>(5,8) = dt;
    this->translationKF.transitionMatrix.at<double>(0,6) = 0.5*std::pow(dt,2);
    this->translationKF.transitionMatrix.at<double>(1,7) = 0.5*std::pow(dt,2);
    this->translationKF.transitionMatrix.at<double>(2,8) = 0.5*std::pow(dt,2);
    cv::setIdentity(this->translationKF.measurementMatrix, cv::Scalar::all(1));
}

const cv::Mat KalmanFilter::predict() {
    return KalmanFilter::pose( this->rotationKF.predict().reshape(1, 3),
                               this->translationKF.predict().reshape(1, 3) );
}

const cv::Mat KalmanFilter::correct(const cv::Mat& pose) {
    std::vector<cv::Mat> vec = KalmanFilter::decompose( pose );
    return KalmanFilter::pose( this->rotationKF.correct(    vec[0].reshape(1, 3) ).reshape(1,3).row(0),
                               this->translationKF.correct( vec[1].reshape(1, 3) ).reshape(1,3).row(0) );
}

const std::vector<cv::Mat> KalmanFilter::decompose(const cv::Mat& pose) {
    cv::Mat_<double> p = pose;
    cv::Matx33d r( p(0,0), p(0,1), p(0,2),
                   p(1,0), p(1,1), p(1,2),
                   p(2,0), p(2,1), p(2,2) );
    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(r), rvec);
    cv::Matx31d tvec( p(0,3), p(1,3), p(2,3) );

    std::vector<cv::Mat> result;
    result.push_back( rvec );
    result.push_back( cv::Mat(tvec) );
    return result;
}

const cv::Mat KalmanFilter::pose(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Mat rotation;
    cv::Rodrigues( rvec, rotation );

    cv::Mat_<double> R = rotation;
    cv::Mat_<double> t = tvec;
    cv::Matx34d p(R(0,0), R(0,1), R(0,2), t(0),
                  R(1,0), R(1,1), R(1,2), t(1),
                  R(2,0), R(2,1), R(2,2), t(2) );
    return cv::Mat(p);
}

