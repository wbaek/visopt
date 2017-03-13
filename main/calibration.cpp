#include <iostream>
#include <getopt.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <utils/string.hpp>
#include <utils/filesystem.hpp>

void help(char* execute) {
    std::cerr << "usage: " << execute << " [-h] -p IMAGE_PATH [-v]" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "\t-h, --help                           show this help message and exit" << std::endl;
    std::cerr << "\t-c, --camera    CAMERA_INDEX         set CAMERA_INDEX (default:1)" << std::endl;
    std::cerr << "\t-x, --corner-x  CORNER_X             set CORNER_X (default:10)" << std::endl;
    std::cerr << "\t-y, --corner-y  CORNER_Y             set CORNER_Y (default:7)" << std::endl;
    std::cerr << "\t-s, --corner-size CORNER_SIZE        set CORNER_SIZE (default:20.0)" << std::endl;
    std::cerr << "\t-v, --verbose                        verbose" << std::endl;
    exit(-1);
}

int main(int argc, char* argv[]) {
    static struct option longOptions[] = {
        {"help",         no_argument,       0, 'h'},
        {"camera",       required_argument, 0, 'c'},
        {"corner-x",     required_argument, 0, 'x'},
        {"corner-y",     required_argument, 0, 'y'},
        {"corner-size",  required_argument, 0, 's'},
        {"verboase",     no_argument,       0, 'v'},
    };

    int cameraIndex=1;
    int cornerX=10;
    int cornerY=7;
    float cornerSize=20.0;
    int argopt, optionIndex=0;
    while( (argopt = getopt_long(argc, argv, "hc:p:", longOptions, &optionIndex)) != -1 ) {
        switch( argopt ) {
            case 'c':
                instant::Utils::String::ToPrimitive(optarg, cameraIndex);
                break;
            case 'x':
                instant::Utils::String::ToPrimitive(optarg, cornerX);
                break;
            case 'y':
                instant::Utils::String::ToPrimitive(optarg, cornerY);
                break;
            case 's':
                instant::Utils::String::ToPrimitive(optarg, cornerY);
                break;
            case 'h':
            default:
                help(argv[0]);
                break;
        }
    }

    cv::VideoCapture capture(cameraIndex);
    if(!capture.isOpened())  // check if we succeeded
        return -1;
    std::cout << "press c or spacebar to save image in memory for calibration candidates" << std::endl;
    std::cout << "press p calculate camera calibration" << std::endl;

    cv::Size boardSize = cv::Size(cornerX, cornerY);
    std::vector< std::vector<cv::Point3f> > pointsInWorld;
    std::vector< std::vector<cv::Point2f> > pointsInImage;
    std::vector<cv::Point3f> chessboardInWorld;
    for(int y=0; y<cornerY; y++) {
        for(int x=0; x<cornerX; x++) {
            chessboardInWorld.push_back( cv::Point3f(x*cornerSize, y*cornerSize, 0.0) );
        }
    }

    cv::namedWindow("view");
    cv::moveWindow("view", 0, 0);
    char ch = '-';
    int frames = 0;
    while( !(ch == 'q' || ch == 'Q') ) {
        cv::Mat image, gray;
        capture.grab();
        capture.retrieve(image);

        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(image, boardSize, corners,
                                               CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if( found ) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(image, boardSize, corners, found);
        }

        cv::imshow("view", image);
        ch = cv::waitKey(1);
        if( found && (ch == 'c' || ch == 'C' || ch == ' ') ) {
            pointsInWorld.push_back( chessboardInWorld );
            pointsInImage.push_back( corners );
            std::cout << "save frame at #images:" << pointsInImage.size() << std::endl;
        } else if( ch == 'p' || ch == 'P' ) {
            std::cout << "calculate camera calibaration" << std::endl;
            cv::Mat cameraMatrix, distCoeffs;
            std::vector<cv::Mat> rvecs, tvecs;
            cv::calibrateCamera(pointsInWorld, pointsInImage, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
            std::cout << cameraMatrix << std::endl;
            ch = 'q';
        }
        frames += 1;
    }
}
