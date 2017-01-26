#ifndef __EXTRACTOR_EXTRACTOR_HPP__
#define __EXTRACTOR_EXTRACTOR_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class Extractor : public Base {
    public:
        virtual ~Extractor() {
        }
        virtual const std::vector<cv::Point2f> extract(const cv::Mat& image) const = 0;
        virtual const std::vector<size_t> buildIndicies(const size_t size, const size_t lastIdx=0) const {
            std::vector<size_t> indicies;
            for(size_t i=0; i<size; i++) {
                indicies.push_back( i + lastIdx ) ;
            }
            return indicies;
        }
};
}

#endif //__EXTRACTOR_EXTRACTOR_HPP__
