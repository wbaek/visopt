#ifndef __RECONSTRUCTOR_TRIANGULATOR_HPP__
#define __RECONSTRUCTOR_TRIANGULATOR_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "reconstructor/reconstructor.hpp"

namespace visopt{
class Triangulator : public Reconstructor {
    public:
        Triangulator(const cv::Mat& intrinsic) {
            this->intrinsic = intrinsic;
        }
        virtual ~Triangulator() {
        }

        static const std::vector<int> unionIndicies(const std::vector<KeyFrame>& keyframes) ;
        virtual const std::vector<cv::Point3f> calc(const std::vector<KeyFrame>& keyframes) const;
        
    protected:
        cv::Mat intrinsic;
};
}

#endif //__RECONSTRUCTOR_TRIANGULATOR_HPP__
