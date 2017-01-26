#ifndef __CORE_MAP_HPP__
#define __CORE_MAP_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
class Map : public Base {
    public:
        enum Status {outlier=-1, initial=0, reconstructed=1};
        virtual const size_t size() const {
            return this->points.size();
        }
        virtual const std::vector<cv::Point3f> getPoints(const std::vector<size_t>& idx_list = std::vector<size_t>()) const {
            if(idx_list.size() == 0) {
                return this->points;
            } else {
                std::vector<cv::Point3f> selected;
                for(auto idx : idx_list) {
                    if(idx < this->points.size())
                        selected.push_back( this->points.at(idx) );
                }
                return selected;
            }
        }

    public:
        std::vector<cv::Point3f> points;
        std::vector<Status> status;
};
}

#endif //__CORE_MAP_HPP__
