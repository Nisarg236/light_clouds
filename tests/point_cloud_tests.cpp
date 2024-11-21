#include <gtest/gtest.h>
#include "PointCloud.hpp"

// Test adding points to PointCloud
TEST(PointCloudTest, AddPoint) {
    PointCloud cloud;
    cloud.addPoint(Point(1, 2, 3));
    ASSERT_EQ(cloud.size(), 1);
    EXPECT_EQ(cloud.getPoint(0).getX(), 1);
    EXPECT_EQ(cloud.getPoint(0).getY(), 2);
    EXPECT_EQ(cloud.getPoint(0).getZ(), 3);
}

// Test calculating centroid
TEST(PointCloudTest, Centroid) {
    PointCloud cloud({Point(1, 1, 1), Point(2, 2, 2), Point(3, 3, 3)});
    Point centroid = cloud.calculateCentroid();
    EXPECT_EQ(centroid.getX(), 2);
    EXPECT_EQ(centroid.getY(), 2);
    EXPECT_EQ(centroid.getZ(), 2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
