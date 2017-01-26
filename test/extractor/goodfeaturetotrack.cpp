#include <gtest/gtest.h>

#include "extractor/goodfeaturetotrack.hpp"

TEST(GoodFeatureToTrack, create) {
    visopt::Extractor* extractor = new visopt::GoodFeatureToTrack();
    EXPECT_EQ("GoodFeatureToTrack", extractor->getName());

    delete extractor;
}

TEST(Map, extract) {
    visopt::Extractor* extractor = new visopt::GoodFeatureToTrack();
    cv::Mat image;
    image = cv::imread("datas/im000.png");

    std::vector<cv::Point2f> features = extractor->extract( image );
    EXPECT_EQ(100, features.size());

    delete extractor;
}
