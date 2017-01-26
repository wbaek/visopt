#ifndef __TRACKER_TRACKER_HPP__
#define __TRACKER_TRACKER_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class Tracker : public Base {
    public:
        enum Selector { prev=0, curr=1 };
        Tracker() {
            this->lastIdx = 0;
        }
        virtual ~Tracker() {
        }
        virtual void init(const cv::Mat& image, const std::vector<cv::Point2f>& points, const std::vector<int>& indicies=std::vector<int>());
        virtual const std::vector<unsigned char> track(const cv::Mat& image) = 0;

        virtual const cv::Mat getImage(const Selector index = Tracker::curr) const;
        virtual const std::vector<cv::Point2f> getPoints(const Selector index=Tracker::curr) const;
        virtual const std::vector<int> getIndicies(const Selector index=Tracker::curr) const;
        virtual void setPoints(const std::vector<cv::Point2f>& points, const Selector index=Tracker::curr);
        virtual void setPoints(const std::vector<cv::Point2f>& points, const std::vector<int>& indicies, const Selector index = Tracker::curr);
        virtual void append(const std::vector<cv::Point2f>& points, const Selector index=Tracker::curr, const double epsilon=3.0);

        virtual void swap();
        virtual void remove(const std::vector<unsigned char>& status);

        virtual void draw(cv::Mat& image);

    protected:
        cv::Mat images[2];
        std::vector<cv::Point2f> points[2];
        std::vector<int> indicies[2];
        int lastIdx = 0;
};
}

#endif //__TRACKER_TRACKER_HPP__
