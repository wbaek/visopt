#include "reconstructor/triangulator.hpp"

#include "pose/camera_pose.hpp"

using namespace visopt;
const std::vector<cv::Point3f> Triangulator::calc(const std::vector<KeyFrame>& keyframes) const {
    if( keyframes.size() < 2 ) {
        throw std::logic_error( instant::Utils::String::Format("keyframes(%d) is more than 2", keyframes.size()) );
        return std::vector<cv::Point3f>();
    }

    KeyFrame frame1 = keyframes.front();
    KeyFrame frame2 = keyframes.back();

    std::vector<int> indicies = KeyFrame::unionIndicies( frame1.indicies, frame2.indicies );
    std::vector<cv::Point2f> pt1 = frame1.getPoints( indicies );
    std::vector<cv::Point2f> pt2 = frame2.getPoints( indicies );

    cv::Mat pose1 = frame1.pose;
    cv::Mat pose2 = frame2.pose;
    if( cv::norm(pose1, pose2, cv::NORM_L1) < 1e-5 ) {
        CameraPose pose(this->intrinsic);
        std::vector<unsigned char> status;
        pose2 = pose.calc(pt1, pt2, status);
        
        size_t k;
        for(size_t i=k=0; i<status.size(); i++) {
            if(!status[i]) continue;
            indicies[k] = indicies[i];
            pt1[k] = pt1[i];
            pt2[k] = pt2[i];
            k++;
        }
        indicies.resize(k);
        pt1.resize(k);
        pt2.resize(k);
    }
    
    cv::Mat reconstructedPoints;
    cv::triangulatePoints(this->intrinsic*pose1, this->intrinsic*pose2, pt1, pt2, reconstructedPoints );

    std::vector<cv::Point3f> results;
    for(size_t i=0; i<reconstructedPoints.size().width; i++) {
        cv::Mat_<double> p = reconstructedPoints.col(i);
        results.push_back( cv::Point3f( p(0)/p(3), p(1)/p(3), p(2)/p(3) ) );
    }
    return results;
}

