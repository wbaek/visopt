#ifndef __RECONSTRUCTOR_RECONSTRUCTOR_HPP__
#define __RECONSTRUCTOR_RECONSTRUCTOR_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class Reconstructor : public Base {
    public:
        virtual ~Reconstructor() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const = 0;
};
}

#endif //__RECONSTRUCTOR_RECONSTRUCTOR_HPP__
