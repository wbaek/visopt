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
    instant::Utils::Filesystem::GetFileNames(dataPath, filelist);

    std::string filename = filelist[0];
    cv::Mat color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);

    visopt::Opticalflow* tracker = new visopt::Opticalflow();
   
    cv::Size size = color.size();
    double focal = std::max(size.width, size.height);
    tracker->setIntrinsic(cv::Mat(cv::Matx33d( focal,     0,  size.width/2.0,
                                                   0, focal, size.height/2.0,
                                                   0,     0,               1 )));
    tracker->init(color);
    tracker->draw(color);
    cv::imshow("image", color);
    cv::waitKey(0);

    for(size_t i=0; i<filelist.size(); i++) {
        std::string filename = filelist[i];
		double startTime = instant::Utils::Others::GetMilliSeconds();
		cv::Mat color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
        tracker->track(color);
        tracker->draw(color);
		cv::imshow("image", color);
		double endTime = instant::Utils::Others::GetMilliSeconds();

		int waitTime = 30 - (int)(endTime - startTime);
        waitTime = 0;//waitTime <= 0 ? 1 : waitTime;
        char ch = cv::waitKey(waitTime);
        if( ch == 'q' || ch == 'Q' )
            break;
	}
	return 0;
}
