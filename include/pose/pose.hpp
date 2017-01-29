#ifndef __POSE_POSE_HPP__
#define __POSE_POSE_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include "base.hpp"

namespace visopt{
class Pose : public Base {
    public:
        virtual ~Pose() {
        }

        virtual const cv::Mat calc(const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
            throw std::logic_error("Pose calc 2D-to-2D method is not implemented");
        }
        virtual const cv::Mat calc(const std::vector<cv::Point3f>& pt1, const std::vector<cv::Point2f>& pt2, std::vector<unsigned char>& status) const {
            throw std::logic_error("Pose calc 3D-to-2D method is not implemented");
        }
        static const cv::Mat toGL(const cv::Mat& pose) {
            cv::Matx33d converter( 1.0,  0.0,  0.0,
                                   0.0, -1.0,  0.0,
                                   0.0,  0.0, -1.0 );
            return cv::Mat(converter) * pose;
        }

        virtual void draw(cv::Mat& image, const cv::Mat& projection, double scale=100.0) {
            cv::Matx44d axis(0.0, scale,   0.0,   0.0,
                             0.0,   0.0, scale,   0.0,
                             0.0,   0.0,   0.0, scale,
                             1.0,   1.0,   1.0,   1.0);
            cv::Mat projected = projection * cv::Mat(axis);

            std::vector<cv::Scalar> colors;
            colors.push_back( cv::Scalar(0,0,0) );
            colors.push_back( cv::Scalar(0,0,255) );
            colors.push_back( cv::Scalar(0,255,0) );
            colors.push_back( cv::Scalar(255,0,0) );
            for(size_t j=1; j<4; j++) {
                cv::Point3f _p1( projected.col(0) );
                cv::Point3f _p2( projected.col(j) );
                cv::Point2f p1 = cv::Point2f(_p1.x/_p1.z, _p1.y/_p1.z);
                cv::Point2f p2 = cv::Point2f(_p2.x/_p2.z, _p2.y/_p2.z);

                cv::line(image, p1, p2, colors[j], 3, 8);
            }
        }
};
}

#endif //__POSE_POSE_HPP__
