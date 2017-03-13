#include <iostream>
#include <cstdlib>
#include <getopt.h>
#include <string>
#include <vector>

#include <utils/string.hpp>
#include <utils/filesystem.hpp>
#include <utils/others.hpp>
#include <opencv2/opencv.hpp>

#include "extractor/goodfeaturetotrack.hpp"
#include "extractor/agast.hpp"
#include "tracker/opticalflow.hpp"
#include "pose/fundamental.hpp"

typedef std::tuple<std::string, double> ttuple;

void help(char* execute) {
    std::cerr << "usage: " << execute << " [-h] -p IMAGE_PATH [-v]" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "\t-h, --help                           show this help message and exit" << std::endl;
    std::cerr << "\t-v, --verbose                        verbose" << std::endl;
    exit(-1);
}

int main(int argc, char* argv[]) {
    static struct option longOptions[] = {
        {"help",      no_argument,       0, 'h'},
        {"verboase",  no_argument,       0, 'v'},
    };

    bool verbose = false;
    int argopt, optionIndex=0;
    while( (argopt = getopt_long(argc, argv, "hv", longOptions, &optionIndex)) != -1 ) {
        switch( argopt ) {
            case 'v':
                verbose = true;
                break;
            case 'h':
            default:
                help(argv[0]);
                break;
        }
    }

    cv::VideoCapture capture(1);
    cv::namedWindow("view");
    cv::moveWindow("view", 0, 0);

    visopt::Tracker* tracker = new visopt::OpticalFlow();
    visopt::Extractor* extractor = new visopt::Agast(); //new visopt::GoodFeatureToTrack();

    size_t frame = 0;
    char ch = ' ';
    while( !(ch == 'q' || ch == 'Q') ) {
        cv::Mat color;
        std::vector<ttuple> timestamps;
	timestamps.push_back( ttuple("init", instant::Utils::Others::GetMilliSeconds()) );

        capture.grab();
        capture.retrieve(color);
        timestamps.push_back( ttuple("image", instant::Utils::Others::GetMilliSeconds()) );

        std::vector<unsigned char> status = tracker->track(color);
        tracker->remove( status );
        timestamps.push_back( ttuple("track", instant::Utils::Others::GetMilliSeconds()) );

        tracker->append( extractor->extract(color) );
        timestamps.push_back( ttuple("extract", instant::Utils::Others::GetMilliSeconds()) );

        tracker->draw(color);
        cv::imshow("view", color);
        timestamps.push_back( ttuple("display", instant::Utils::Others::GetMilliSeconds()) );

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
            elapsed_string += instant::Utils::String::Format("%s:%02.1fms ", std::get<0>(item).c_str(), std::get<1>(item));
        }
        std::cout << instant::Utils::String::Format("[#%05d] %s", frame, elapsed_string.c_str()) << std::endl;
    }
    return 0;
}
