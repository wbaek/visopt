#include <iostream>
#include <cstdlib>
#include <getopt.h>
#include <string>
#include <vector>

#include <utils/string.hpp>
#include <utils/filesystem.hpp>
#include <utils/others.hpp>
#include <opencv2/opencv.hpp>

#include "tracker/opticalflow.hpp"
#include "reconstructor/triangulator.hpp"

void help(char* execute) {
    std::cerr << "usage: " << execute << " [-h] -p IMAGE_PATH [-v]" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "\t-h, --help                           show this help message and exit" << std::endl;
    std::cerr << "\t-p, --path      DATA_PATH            set DATA_PATH" << std::endl;
    std::cerr << "\t-f, --focal     FOCAL_LENGTH         set FOCAL_LENGTH (default:400)" << std::endl;
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
    float focallength = 400;
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

    double fx=focallength, fy=focallength, cx=color.size().width/2.0, cy=color.size().height/2.0;
    cv::Matx33d intrinsic(fx, 0, cx,
                          0, fy, cy,
                          0, 0, 1);

    visopt::Opticalflow* tracker = new visopt::Opticalflow(cv::Mat(intrinsic));
    visopt::Triangulator* reconstructor = new visopt::Triangulator(cv::Mat(intrinsic));
    size_t frames = 0;
    char ch = ' ';
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

        tracker->setImage( color );
        tracker->track();
        bool trackable = tracker->updatePose();
        if(frames%30 == 0) {
            if(frames>0) {
                cv::Mat p1, p2;
                if(!trackable) {
                    std::vector<unsigned char> status;
                    p1 = cv::Mat::eye(3, 4, CV_64FC1);
                    p2 = reconstructor->pose( tracker->initPoints, tracker->currPoints, status);
                    tracker->remove(status);
                } else {
                    p1 = tracker->initPose;
                    p2 = tracker->currPose;
                }
                std::vector<cv::Point3f> map = reconstructor->reconstruct( tracker->initPoints, tracker->currPoints, p1, p2);

                tracker->mapPoints = map;
                tracker->initPose = p2;
                tracker->reconstruct();
                trackable = tracker->updatePose();
            }
            tracker->extract();
        }
        tracker->draw(color);
        tracker->swap();
        cv::imshow("image", color);
		double endTime = instant::Utils::Others::GetMilliSeconds();

		int waitTime = 30 - (int)(endTime - startTime);
        waitTime = 0;//waitTime <= 0 ? 1 : waitTime;
        ch = cv::waitKey(waitTime);
        if( ch == 'q' || ch == 'Q' )
            break;
        frames++;
	}
	return 0;
}
