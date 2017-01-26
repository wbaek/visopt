#ifndef __CORE_KEYFRAME_HPP__
#define __CORE_KEYFRAME_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
class KeyFrame : public Base {
    public:
        KeyFrame() {
            this->pose = cv::Mat::eye(3, 4, CV_64F);
        }
        static const std::vector<int> unionIndicies(const std::vector<int>& indicies1, const std::vector<int>& indicies2) {
            std::vector<int> indicies;
            for(auto index1 : indicies1) {
                for(auto index2 : indicies2) {
                    if(index1 == index2) {
                        indicies.push_back(index1);
                        break;
                    }
                }
            }
            return indicies;
        }
        const std::vector<cv::Point2f> getPoints(const std::vector<int>& indicies) {
            std::vector<cv::Point2f> selected;
            for(auto index1 : indicies) {
                for(size_t j=0; j<this->indicies.size(); j++) {
                    int index2 = this->indicies[j];
                    if( index1 == index2 ) {
                        selected.push_back( this->points[j] );
                        break;
                    }
                }
            }
            return selected;
        }

    public:
        cv::Mat pose;
        cv::Mat image;
        std::vector<cv::Point2f> points;
        std::vector<int> indicies;
};
}

#endif //__CORE_KEYFRAME_HPP__
