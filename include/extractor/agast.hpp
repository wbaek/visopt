#ifndef __EXTRATOR_AGAST_HPP__
#define __EXTRATOR_AGAST_HPP__

#include "extractor/extractor.hpp"

namespace visopt {
class Agast : public Extractor {
    public:
        Agast(const int threshold=15, const bool nonmaxSuppression=true, const int gaussianBlur=7);
        virtual ~Agast() {
        }

        virtual const std::vector<cv::Point2f> extract(const cv::Mat& image) const;

    private:
        int threshold;
        bool nonmaxSupression;
        int gaussianBlur;
};
}

#endif //__EXTRATOR_AGAST_HPP__
