#ifndef __RECONSTRUCTOR_RECONSTRUCTOR_HPP__
#define __RECONSTRUCTOR_RECONSTRUCTOR_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"
#include "core/keyframe.hpp"

namespace visopt{
class Reconstructor : public Base {
    public:
        virtual ~Reconstructor() {
        }

        virtual const std::vector<cv::Point3f> calc(const std::vector<KeyFrame>& keyframes) const = 0;
};
}

#endif //__RECONSTRUCTOR_RECONSTRUCTOR_HPP__
