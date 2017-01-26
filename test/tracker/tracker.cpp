#include <gtest/gtest.h>

#include "tracker/tracker.hpp"

namespace visopt {
class TestTracker : public Tracker {
    virtual const std::vector<unsigned char> track(const cv::Mat& image) {
        this->images[Tracker::curr] = image;
        return std::vector<unsigned char>();
    }
};
}

TEST(Tracker, create) {
    visopt::Tracker* tracker = new visopt::TestTracker();
    EXPECT_EQ("TestTracker", tracker->getName());

    delete tracker;
}

TEST(Tracker, get_set_points) {
    visopt::Tracker* tracker = new visopt::TestTracker();

    std::vector<cv::Point2f> points[2];
    points[0].push_back( cv::Point2f(1,0) );
    points[0].push_back( cv::Point2f(2,0) );
    points[1].push_back( cv::Point2f(0,1) );
    points[1].push_back( cv::Point2f(0,2) );

    std::vector<cv::Point2f> out;
    std::vector<int> indicies;
    out = tracker->getPoints();
    EXPECT_EQ(0, out.size());

    tracker->setPoints(points[0]);
    out = tracker->getPoints();
    EXPECT_EQ(2, out.size());
    EXPECT_EQ(cv::Point2f(1,0), out[0]);
    EXPECT_EQ(cv::Point2f(2,0), out[1]);

    indicies = tracker->getIndicies();
    EXPECT_EQ(2, indicies.size());
    EXPECT_EQ(0, indicies[0]);
    EXPECT_EQ(1, indicies[1]);
    indicies[0] = 1;
    indicies[1] = 2;

    out = tracker->getPoints(visopt::Tracker::prev);
    EXPECT_EQ(0, out.size());
    tracker->setPoints(points[1], indicies, visopt::Tracker::prev);

    out = tracker->getPoints(visopt::Tracker::prev);
    EXPECT_EQ(2, out.size());
    EXPECT_EQ(cv::Point2f(0,1), out[0]);
    EXPECT_EQ(cv::Point2f(0,2), out[1]);

    indicies = tracker->getIndicies(visopt::Tracker::prev);
    EXPECT_EQ(2, indicies.size());
    EXPECT_EQ(1, indicies[0]);
    EXPECT_EQ(2, indicies[1]);

    delete tracker;
}

TEST(Tracker, swap) {
    visopt::Tracker* tracker = new visopt::TestTracker();

    std::vector<cv::Point2f> points[2];
    points[0].push_back( cv::Point2f(1,0) );
    points[0].push_back( cv::Point2f(2,0) );
    points[1].push_back( cv::Point2f(0,1) );
    points[1].push_back( cv::Point2f(0,2) );

    EXPECT_EQ(0, tracker->getPoints(visopt::Tracker::prev).size());
    EXPECT_EQ(0, tracker->getPoints(visopt::Tracker::curr).size());

    tracker->setPoints(points[0], visopt::Tracker::prev);
    tracker->setPoints(points[1], visopt::Tracker::curr);

    std::vector<cv::Point2f> out;
    out = tracker->getPoints(visopt::Tracker::prev);
    EXPECT_EQ(2, out.size());
    EXPECT_EQ(cv::Point2f(1,0), out[0]);
    EXPECT_EQ(cv::Point2f(2,0), out[1]);

    out = tracker->getPoints(visopt::Tracker::curr);
    EXPECT_EQ(2, out.size());
    EXPECT_EQ(cv::Point2f(0,1), out[0]);
    EXPECT_EQ(cv::Point2f(0,2), out[1]);

    tracker->swap();

    out = tracker->getPoints(visopt::Tracker::prev);
    EXPECT_EQ(2, out.size());
    EXPECT_EQ(cv::Point2f(0,1), out[0]);
    EXPECT_EQ(cv::Point2f(0,2), out[1]);

    out = tracker->getPoints(visopt::Tracker::curr);
    EXPECT_EQ(2, out.size());
    EXPECT_EQ(cv::Point2f(1,0), out[0]);
    EXPECT_EQ(cv::Point2f(2,0), out[1]);

    delete tracker;
}

