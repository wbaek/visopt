#include <gtest/gtest.h>

#include "core/map.hpp"

TEST(Map, create) {
    visopt::Map map;
    EXPECT_EQ("Map", map.getName());
}

TEST(Map, size) {
    visopt::Map map;
    map.points.push_back( cv::Point3f(1, 0, 0) );
    map.points.push_back( cv::Point3f(2, 0, 0) );
    map.points.push_back( cv::Point3f(3, 0, 0) );

    EXPECT_EQ(3, map.size());
}

TEST(Map, getPoints_all) {
    visopt::Map map;
    map.points.push_back( cv::Point3f(1, 0, 0) );
    map.points.push_back( cv::Point3f(2, 0, 0) );
    map.points.push_back( cv::Point3f(3, 0, 0) );

    std::vector<cv::Point3f> points = map.getPoints();
    EXPECT_EQ(3, points.size());
    EXPECT_EQ(cv::Point3f(1, 0, 0), points[0]);
}

TEST(Map, getPoints_selected) {
    visopt::Map map;
    map.points.push_back( cv::Point3f(1, 0, 0) );
    map.points.push_back( cv::Point3f(2, 0, 0) );
    map.points.push_back( cv::Point3f(3, 0, 0) );

    std::vector<int> idx_list;
    idx_list.push_back(1);
    idx_list.push_back(3);

    std::vector<cv::Point3f> points = map.getPoints(idx_list);
    EXPECT_EQ(1, points.size());
    EXPECT_EQ(cv::Point3f(2, 0, 0), points[0]);
}
