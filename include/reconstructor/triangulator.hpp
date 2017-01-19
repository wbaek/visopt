#ifndef __RECONSTRUCTOR_TRIANGULATOR_HPP__
#define __RECONSTRUCTOR_TRIANGULATOR_HPP__

#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
    class Triangulator : public Base {
        public:
            Triangulator(const cv::Mat& intrinsic) {
                this->intrinsic = intrinsic;
            }
            virtual ~Triangulator() {
            }

            const cv::Mat pose(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, std::vector<unsigned char>& status);
            const std::vector<cv::Point3f> reconstruct(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2, const cv::Mat& pose1, const cv::Mat& pose2);

        protected:
            cv::Mat intrinsic;
    };
}

#endif //__RECONSTRUCTOR_TRIANGULATOR_HPP__
