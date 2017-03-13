#include <iostream>
#include <cstdlib>
#include <getopt.h>
#include <string>
#include <vector>

#include <utils/string.hpp>
#include <utils/filesystem.hpp>
#include <utils/others.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include "extractor/goodfeaturetotrack.hpp"
#include "extractor/agast.hpp"
#include "tracker/opticalflow.hpp"
#include "pose/fundamental.hpp"
#include "pose/camera_pose.hpp"
#include "pose/kalman_filter.hpp"
#include "core/keyframe.hpp"
#include "reconstructor/triangulator.hpp"
#include "core/map.hpp"

typedef std::tuple<std::string, double> ttuple;

void help(char* execute) {
    std::cerr << "usage: " << execute << " [-h] -p IMAGE_PATH [-v]" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "\t-h, --help                           show this help message and exit" << std::endl;
    std::cerr << "\t-p, --path      DATA_PATH            set DATA_PATH" << std::endl;
    std::cerr << "\t-f, --focal     FOCAL_LENGTH         set FOCAL_LENGTH (default:245)" << std::endl;
    std::cerr << "\t-v, --verbose                        verbose" << std::endl;
    exit(-1);
}

int main(int argc, char* argv[]) {
    static struct option longOptions[] = {
        {"help",      no_argument,       0, 'h'},
        {"path",      required_argument, 0, 'p'},
        {"focal",     required_argument, 0, 'f'},
        {"verboase",  no_argument,       0, 'v'},
    };

    std::string dataPath;
    bool verbose = false;
    int argopt, optionIndex=0;
    float focallength = 235;
    while( (argopt = getopt_long(argc, argv, "hp:f:v", longOptions, &optionIndex)) != -1 ) {
        switch( argopt ) {
            case 'p':
                dataPath = std::string(optarg);
                break;
            case 'f':
                instant::Utils::String::ToPrimitive(optarg, focallength);
                break;
            case 'v':
                verbose = true;
                break;
            case 'h':
            default:
                help(argv[0]);
                break;
        }
    }
    if( dataPath.size() == 0 ) {
        help(argv[0]);
    }

    // setup data with visualization
    std::vector<std::string> filelist;
    cv::VideoCapture capture(1);
    capture.isOpened();
    cv::Mat color;
    if (instant::Utils::Filesystem::IsDirectory(dataPath)) {
        instant::Utils::Filesystem::GetFileNames(dataPath, filelist);
        color = cv::imread(filelist[0], CV_LOAD_IMAGE_COLOR);
    } else {
        //capture = cv::VideoCapture(dataPath);
        //capture.set(CV_CAP_PROP_FPS, 60);
        for(int i=0; i<1; i++){
            capture.grab();
            capture.retrieve(color);
        }
        //double scale = 1.0/4.0;
        //cv::Size size(color.size().width * scale, color.size().height * scale);
        //cv::resize(color, color, size);
        cv::Size size = color.size();
        std::cout << size.width << "x" << size.height << std::endl;
    }

    cv::namedWindow("view");
    cv::moveWindow("view", 0, 0);
    cv::resizeWindow("view", color.size().width, color.size().height);
    cv::viz::Viz3d window("Coordinate Frame");
    {
        window.setWindowSize(cv::Size(500,500));
        window.setWindowPosition(cv::Point(color.size().width,500));
        cv::Point3d cam_pos(60.0f, -600.0f, -500.0), cam_focal_point(0.0f,0.0f,100.0f), cam_y_dir(0.0f,1.0f,0.0f);
        cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
        window.setViewerPose(cam_pose);
        window.spinOnce(1, true);
    }

    // setup tracker
    double fx=focallength, fy=focallength, cx=color.size().width/2.0, cy=color.size().height/2.0;
    cv::Mat intrinsic = cv::Mat( cv::Matx33d(fx, 0, cx,
                                             0, fy, cy,
                                             0,  0,  1) );

    visopt::Tracker* tracker = new visopt::OpticalFlow();
    visopt::Extractor* extractor = new visopt::Agast(); //new visopt::GoodFeatureToTrack();
    visopt::Pose* fundamental = new visopt::Fundamental();
    visopt::Pose* camera = new visopt::CameraPose(intrinsic);
    visopt::KalmanFilter* kalmanFilter = new visopt::KalmanFilter();
    visopt::Reconstructor* reconstructor = new visopt::Triangulator(intrinsic);
    std::vector<visopt::KeyFrame> keyframes;
    visopt::Map map;

    std::vector< cv::Affine3d > path;
    std::vector< cv::Affine3d > last(1);
    size_t frame = 0;
    char ch = ' ';
    //cv::waitKey(0);
    while( !(ch == 'q' || ch == 'Q') ) {
        std::vector<ttuple> timestamps;
		timestamps.push_back( ttuple("init", instant::Utils::Others::GetMilliSeconds()) );
        if(filelist.size() > 0) {
            if( frame >= filelist.size() ) break;
            std::string filename = filelist[frame];
		    color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
        } else {
            capture.grab();
            capture.retrieve(color);
            //double scale = 1.0/4.0;
            //cv::Size size(color.size().width * scale, color.size().height * scale);
            //if( size.width <= 0 || size.height <= 0 ) break;
            //cv::resize(color, color, size);
        }
        timestamps.push_back( ttuple("image", instant::Utils::Others::GetMilliSeconds()) );

        std::vector<unsigned char> status = tracker->track(color);
        tracker->remove( status );
        if( status.size() > 0 ) {
            fundamental->calc(tracker->getPoints(visopt::Tracker::prev), tracker->getPoints(visopt::Tracker::curr), status);
            tracker->remove( status );
        }
        timestamps.push_back( ttuple("track", instant::Utils::Others::GetMilliSeconds()) );

        bool estimatedPose = false;
        cv::Mat pose = cv::Mat::eye(3, 4, CV_64F);
        std::vector<int> indicies = map.unionIndicies( tracker->getIndicies() );
        if(indicies.size() > 0) {
            visopt::KeyFrame currentFrame;
            currentFrame.image = color;
            currentFrame.points = tracker->getPoints();
            currentFrame.indicies = tracker->getIndicies();

            std::vector<cv::Point3f> mapPoints = map.getPoints( indicies );
            std::vector<cv::Point2f> imagePoints = currentFrame.getPoints( indicies );
            pose = camera->calc(mapPoints, imagePoints, status);
            kalmanFilter->predict();
            pose = kalmanFilter->correct( pose );
            //*
            for(size_t i=0; i<status.size(); i++) {
                if(status[i]) continue;
                map.status[ indicies[i] ] = visopt::Map::outlier;
            }
            //*/
            
            path.push_back( cv::Affine3d( -pose ) );
            last[0] = cv::Affine3d( -pose );
            estimatedPose = true;
        }
        timestamps.push_back( ttuple("pose", instant::Utils::Others::GetMilliSeconds()) );

        if( frame%30 == 0 ){
            tracker->append( extractor->extract(color) );

            visopt::KeyFrame keyframe;
            keyframe.image = color;
            keyframe.pose = pose;
            keyframe.points = tracker->getPoints();
            keyframe.indicies = tracker->getIndicies();
            keyframes.push_back( keyframe );

            if( frame > 0 ) {
                std::vector<visopt::KeyFrame> last2frames;
                for(size_t i=keyframes.size()-2; i<keyframes.size(); i++) {
                    last2frames.push_back( keyframes[i] );
                }
                std::vector<int> indicies = visopt::Triangulator::unionIndicies(last2frames);
                std::vector<cv::Point3f> mapPoints = reconstructor->calc(last2frames);
                map.append( mapPoints, indicies );
            }
        }
        timestamps.push_back( ttuple("extract", instant::Utils::Others::GetMilliSeconds()) );

        tracker->draw(color);
        if(estimatedPose) {
            camera->draw(color, intrinsic*pose);
        }
        cv::imshow("view", color);
        timestamps.push_back( ttuple("display", instant::Utils::Others::GetMilliSeconds()) );

        int trackedCount = tracker->getPoints().size();
        int lastIdx = tracker->getIndicies().back();
        tracker->swap();
        timestamps.push_back( ttuple("arrage", instant::Utils::Others::GetMilliSeconds()) );

        std::vector<ttuple> elapsedtimes;
        elapsedtimes.push_back( ttuple("total", std::get<1>(timestamps.back()) - std::get<1>(timestamps.front())) );
        for(size_t i=1; i<timestamps.size(); i++) {
            elapsedtimes.push_back( ttuple(std::get<0>(timestamps[i]), std::get<1>(timestamps[i]) - std::get<1>(timestamps[i-1])) );
        }
        int waitTime = 1;//waitTime <= 0 ? 1 : waitTime;
        ch = cv::waitKey(waitTime);
        frame++;

        std::string elapsed_string = "";
        for(auto item : elapsedtimes) {
            elapsed_string += instant::Utils::String::Format("%s:%.1fms ", std::get<0>(item).c_str(), std::get<1>(item));
        }
        std::cout << instant::Utils::String::Format("[#%05d] %s tracked:%05d lastIdx:%05d", frame, elapsed_string.c_str(), trackedCount, lastIdx) << std::endl;

        window.setBackgroundColor(); // black by default
        window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(100.0));
        if(map.getPoints(visopt::Map::reconstructed).size() > 0) {
            window.showWidget("point_cloud", cv::viz::WCloud(map.getPoints(visopt::Map::reconstructed), cv::viz::Color::green()));
        }
        if(path.size() > 0) {
            window.showWidget("path", cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::red()));
            window.showWidget("camera", cv::viz::WTrajectoryFrustums(last, cv::Matx33d(intrinsic), 10.0, cv::viz::Color::yellow()));
        }
        window.spinOnce(1, true);
	}
	return 0;
}
