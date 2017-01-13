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

            const std::vector<cv::Point3f> reconstruct(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2);

        protected:
            cv::Mat intrinsic;
    };
}

#endif //__RECONSTRUCTOR_TRIANGULATOR_HPP__
