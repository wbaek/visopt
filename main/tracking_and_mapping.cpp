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

#include "tracker/opticalflow.hpp"

void help(char* execute) {
    std::cerr << "usage: " << execute << " [-h] -p IMAGE_PATH [-v]" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "\t-h, --help                           show this help message and exit" << std::endl;
    std::cerr << "\t-p, --path      DATA_PATH            set DATA_PATH" << std::endl;
    std::cerr << "\t-f, --focal     FOCAL_LENGTH         set FOCAL_LENGTH (default:256)" << std::endl;
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
    float focallength = 256;
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
    cv::VideoCapture capture;
    cv::Mat color;
    if (instant::Utils::Filesystem::IsDirectory(dataPath)) {
        instant::Utils::Filesystem::GetFileNames(dataPath, filelist);
        color = cv::imread(filelist[0], CV_LOAD_IMAGE_COLOR);
    } else {
        capture = cv::VideoCapture(dataPath);
        capture.set(CV_CAP_PROP_FPS, 30);
        capture.grab();
        capture.retrieve(color);
        double scale = 1.0/4.0;
        cv::Size size(color.size().width * scale, color.size().height * scale);
        cv::resize(color, color, size);
    }

    cv::namedWindow("view");
    cv::moveWindow("view", 0, 0);
    cv::resizeWindow("view", color.size().width, color.size().height);
    cv::namedWindow("debug");
    cv::moveWindow("view", 0, color.size().height);
    cv::resizeWindow("debug", color.size().width, color.size().height);
    cv::viz::Viz3d window("Coordinate Frame");
    {
        window.setWindowSize(cv::Size(500,500));
        window.setWindowPosition(cv::Point(color.size().width,0));
        cv::Point3d cam_pos(200.0f,300.0f,600.0f), cam_focal_point(0.0f,0.0f,0.0f), cam_y_dir(0.0f,1.0f,0.0f);
        cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
        window.setViewerPose(cam_pose);
        window.spinOnce(1, true);
    }

    // setup tracker
    double fx=focallength, fy=focallength, cx=color.size().width/2.0, cy=color.size().height/2.0;
    cv::Matx33d intrinsic(fx, 0, cx,
                          0, fy, cy,
                          0, 0, 1);

    std::vector< cv::Affine3d > path;
    std::vector< cv::Affine3d > last(1);
    visopt::Opticalflow* tracker = new visopt::Opticalflow(cv::Mat(intrinsic));
    size_t frames = 0;
    char ch = ' ';
    cv::waitKey(0);
    while( !(ch == 'q' || ch == 'Q') ) {
		double startTime = instant::Utils::Others::GetMilliSeconds();
        if(filelist.size() > 0) {
            std::string filename = filelist[frames];
		    color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
        } else {
            capture.grab();
            capture.retrieve(color);
            double scale = 1.0/4.0;
            cv::Size size(color.size().width * scale, color.size().height * scale);
            cv::resize(color, color, size);
        }
        cv::Mat debug = color.clone();

        tracker->setImage( color );
        tracker->track();
        bool trackable = tracker->updatePose();
        if(frames%30 == 0) {
            if(frames>0) {
                tracker->reconstruct();
                tracker->updatePose();
            }
            tracker->extract();
        }
        if(trackable) {
            path.push_back( cv::Affine3d(tracker->getGLPose()) );
            last[0] = cv::Affine3d(tracker->getGLPose());
        }
        tracker->draw(color);
        tracker->draw(debug, true);
        tracker->swap();
        cv::imshow("view", color);
        cv::imshow("debug", debug);

        window.setBackgroundColor(); // black by default
        window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
        if(tracker->getMapPoints().size() > 0) 
            window.showWidget("point_cloud", cv::viz::WCloud(tracker->getMapPoints(), cv::viz::Color::green()));
        if(path.size() > 0) {
            window.showWidget("path", cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::red()));
            window.showWidget("camera", cv::viz::WTrajectoryFrustums(last, intrinsic, 10.0, cv::viz::Color::yellow()));
        }
        window.spinOnce(1, true);
		double endTime = instant::Utils::Others::GetMilliSeconds();

		int waitTime = 30 - (int)(endTime - startTime);
        waitTime = 1;//waitTime <= 0 ? 1 : waitTime;
        ch = cv::waitKey(waitTime);
        frames++;
	}
	return 0;
}
