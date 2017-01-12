#ifndef __TRACKER_TRACKER_HPP__
#define __TRACKER_TRACKER_HPP__

#include <opencv2/opencv.hpp>
#include <utils/string.hpp>
#include <utils/type.hpp>

namespace visopt {
    class Tracker {
        protected:
            Tracker(double focallength=500.0) {
                this->homography = cv::Mat::eye(4, 4, CV_64F);
            }

        public:
            virtual ~Tracker() {
            }
            virtual const std::string getNmae() const {
                std::string className = instant::Utils::Type::GetTypeName(this);
                return instant::Utils::String::Replace(className, "visopt::", "");
            }
            virtual void init(const cv::Mat& image=cv::Mat()) = 0;
            virtual void track(const cv::Mat& image) = 0;
            virtual void draw(cv::Mat& image) const;
            virtual void setHomography(const cv::Mat& homography) {
                this->homography = homography.clone();
            }
            virtual const cv::Mat getHomography() const {
                return this->homography.clone();
            }

        protected:
            std::vector<cv::Mat> images;
            cv::Mat homography;

            cv::Mat intrinsic;
    };
}

#endif //__TRACKER_TRACKER_HPP__
