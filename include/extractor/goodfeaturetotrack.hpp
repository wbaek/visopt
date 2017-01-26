#ifndef __EXTRATOR_GFTT_HPP__
#define __EXTRATOR_GFTT_HPP__

#include "extractor/extractor.hpp"

namespace visopt {
class GoodFeatureToTrack : public Extractor {
    public:
        GoodFeatureToTrack();
        virtual ~GoodFeatureToTrack() {
        }

        virtual const std::vector<cv::Point2f> extract(const cv::Mat& image) const;

    private:
        cv::TermCriteria termCriteria;
};
}

#endif //__EXTRATOR_GFTT_HPP__
