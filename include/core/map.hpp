#ifndef __CORE_MAP_HPP__
#define __CORE_MAP_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt {
class Map : public Base {
    public:
        enum Status {unset=0, reconstructed=1, outlier=100, all=1000};
        virtual const size_t size() const {
            return this->points.size();
        }

        virtual const std::vector<int> unionIndicies(const std::vector<int>& idx_list) const {
            std::vector<int> selected;
            for(auto idx : idx_list) {
                if(0 <= idx && idx < this->points.size())
                    if( this->status.at(idx) == Map::reconstructed ) {
                        selected.push_back( idx );
                    }
            }
            return selected;
        }
        virtual const std::vector<cv::Point3f> getPoints(const Status& status = Map::all) {
            if( status == Map::all ) {
                return this->points;
            } else {
                std::vector<cv::Point3f> selected;
                for(size_t i=0; i<this->status.size(); i++) {
                    if(this->status[i] == status)
                        selected.push_back( this->points[i] );
                }
                return selected;
            }
        }
        virtual const std::vector<cv::Point3f> getPoints(const std::vector<int>& idx_list = std::vector<int>()) const {
            if(idx_list.size() == 0) {
                return this->points;
            } else {
                std::vector<cv::Point3f> selected;
                for(auto idx : idx_list) {
                    if(0 <= idx && idx < this->points.size())
                        if( this->status.at(idx) == Map::reconstructed ) {
                            selected.push_back( this->points.at(idx) );
                        }
                }
                return selected;
            }
        }
        virtual void append(const std::vector<cv::Point3f>& points, const std::vector<int>& indicies) {
            auto maxIndex = std::max((int)this->points.size(), *std::max_element(indicies.begin(), indicies.end()));
            this->points.resize( maxIndex );
            this->status.resize( maxIndex );
            
            for(size_t i=0; i<indicies.size(); i++) {
                this->points[ indicies[i] ] = points[i];
                this->status[ indicies[i] ] = Map::reconstructed;
            }
        }

    public:
        std::vector<cv::Point3f> points;
        std::vector<Status> status;
};
}

#endif //__CORE_MAP_HPP__
