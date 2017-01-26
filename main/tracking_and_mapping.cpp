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
#include "tracker/opticalflow.hpp"
#include "pose/fundamental.hpp"
#include "core/keyframe.hpp"

typedef std::tuple<std::string, double> ttuple;

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
        capture.set(CV_CAP_PROP_FPS, 60);
        capture.grab();
        capture.retrieve(color);
        double scale = 1.0/4.0;
        cv::Size size(color.size().width * scale, color.size().height * scale);
        cv::resize(color, color, size);
        std::cout << size.width << "x" << size.height << std::endl;

    }

    cv::namedWindow("view");
    cv::moveWindow("view", 0, 0);
    cv::resizeWindow("view", color.size().width, color.size().height);

    // setup tracker
    double fx=focallength, fy=focallength, cx=color.size().width/2.0, cy=color.size().height/2.0;
    cv::Matx33d intrinsic(fx, 0, cx,
                          0, fy, cy,
                          0, 0, 1);

    visopt::Tracker* tracker = new visopt::OpticalFlow();
    visopt::Extractor* extractor = new visopt::GoodFeatureToTrack();
    visopt::Pose2D* pose2D = new visopt::Fundamental();
    std::vector<visopt::KeyFrame> keyframes;
    size_t frame = 0;
    char ch = ' ';
    //cv::waitKey(0);
    while( !(ch == 'q' || ch == 'Q') ) {
        std::vector<ttuple> timestamps;
		timestamps.push_back( ttuple("init", instant::Utils::Others::GetMilliSeconds()) );
        if(filelist.size() > 0) {
            std::string filename = filelist[frame];
		    color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
        } else {
            capture.grab();
            capture.retrieve(color);
            double scale = 1.0/4.0;
            cv::Size size(color.size().width * scale, color.size().height * scale);
            cv::resize(color, color, size);
        }
        timestamps.push_back( ttuple("image", instant::Utils::Others::GetMilliSeconds()) );

        std::vector<unsigned char> status = tracker->track(color);
        tracker->remove( status );
        if( status.size() > 0 ) {
            pose2D->calc(tracker->getPoints(visopt::Tracker::prev), tracker->getPoints(visopt::Tracker::curr), status);
            tracker->remove( status );
        }
        timestamps.push_back( ttuple("track", instant::Utils::Others::GetMilliSeconds()) );

        if( frame%15 == 0 ){
            tracker->append( extractor->extract(color) );

            visopt::KeyFrame keyframe;
            keyframe.image = color;
            keyframe.points = tracker->getPoints();
            keyframe.indicies = tracker->getIndicies();
            keyframes.push_back( keyframe );
        }
        timestamps.push_back( ttuple("extract", instant::Utils::Others::GetMilliSeconds()) );

        tracker->draw(color);
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
	}
	return 0;
}
