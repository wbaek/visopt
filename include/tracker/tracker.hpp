#ifndef __TRACKER_TRACKER_HPP__
#define __TRACKER_TRACKER_HPP__

#include <opencv2/opencv.hpp>
#include <utils/string.hpp>
#include <utils/type.hpp>

namespace visopt {
    class Tracker {
        protected:
            Tracker() {
                this->intrinsic = cv::Mat::eye(3, 3, CV_64F);
                this->rotation = cv::Mat::eye(3, 3, CV_64F);
                this->translation = cv::Mat::zeros(3, 1, CV_64F);
                this->translation.at<double>(0, 2) = 2000.0;
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
            
            virtual std::tuple<cv::Mat, cv::Mat> calcMotion(const cv::Mat& fundamental) const;
            virtual void updateMotion(const cv::Mat& rotation, const cv::Mat& translation);
            virtual std::vector<cv::Point2f> project(const std::vector<cv::Matx31d>& points3D) const;

            virtual void setIntrinsic(const cv::Mat& intrinsic) {
                this->intrinsic = intrinsic;
            }
            virtual const cv::Mat getIntrinsic() const {
                return this->intrinsic.clone();
            }

        protected:
            std::vector<cv::Mat> images;

            cv::Mat intrinsic;
            cv::Mat rotation;
            cv::Mat translation;
    };
}

#endif //__TRACKER_TRACKER_HPP__
