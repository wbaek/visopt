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

void help(char* execute) {
    std::cerr << "usage: " << execute << " [-h] -p IMAGE_PATH [-v]" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "\t-h, --help                           show this help message and exit" << std::endl;
    std::cerr << "\t-p, --path      DATA_PATH            set DATA_PATH" << std::endl;
    std::cerr << "\t-v, --verbose                        verbose" << std::endl;
    exit(-1);
}

int main(int argc, char* argv[]) {
    static struct option longOptions[] = {
        {"help",      no_argument,       0, 'h'},
        {"path",      required_argument, 0, 'p'},
        {"verboase",  no_argument,       0, 'v'},
    };

	std::string dataPath;
    bool verbose = false;
	int argopt, optionIndex=0;
    while( (argopt = getopt_long(argc, argv, "hp:t:g:e:k:bv", longOptions, &optionIndex)) != -1 ) {
        switch( argopt ) {
            case 'p':
                dataPath = std::string(optarg);
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
    if (instant::Utils::Filesystem::IsDirectory(dataPath)) {
        instant::Utils::Filesystem::GetFileNames(dataPath, filelist);
    } else {
        capture = cv::VideoCapture(dataPath);
        capture.set(CV_CAP_PROP_FPS, 15);
    }

    visopt::Opticalflow* tracker = new visopt::Opticalflow();
    size_t frames = 0;
    char ch = ' ';
    while( !(ch == 'q' || ch == 'Q') ) {
		double startTime = instant::Utils::Others::GetMilliSeconds();
        cv::Mat color;
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
        if(frames%10 == 0) {
            if(frames>0) {
                tracker->reconstruct();
            }
            tracker->extract();
        }
        tracker->draw(color);
        tracker->swap();
		cv::imshow("image", color);
		double endTime = instant::Utils::Others::GetMilliSeconds();

		int waitTime = 30 - (int)(endTime - startTime);
        waitTime = waitTime <= 0 ? 1 : waitTime;
        ch = cv::waitKey(waitTime);
        if( ch == 'q' || ch == 'Q' )
            break;
        frames++;
	}
	return 0;
}
