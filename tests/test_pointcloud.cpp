#include <gtest/gtest.h>
#include "Point.hpp"
#include "PointCloud.hpp"

// Test the Point constructor and distance function
TEST(PointTest, ConstructorAndDistance) {
    Point p1(0.0f, 0.0f, 0.0f);
    Point p2(3.0f, 4.0f, 0.0f);

    // Test distance between two points
    EXPECT_FLOAT_EQ(p1.distanceTo(p2), 5.0f);  // sqrt(3^2 + 4^2) = 5
}

// Test the PointCloud class
TEST(PointCloudTest, AddPointAndSize) {
    PointCloud cloud;

    // Add some points
    cloud.addPoint(Point(1.0f, 2.0f, 3.0f));
    cloud.addPoint(Point(4.0f, 5.0f, 6.0f));

    // Test the size of the cloud
    EXPECT_EQ(cloud.size(), 2);  // Should be 2 after adding two points
}

TEST(PointCloudTest, GetPointValidIndex) {
    PointCloud cloud;
    cloud.addPoint(Point(1.0f, 2.0f, 3.0f));
    cloud.addPoint(Point(4.0f, 5.0f, 6.0f));

    // Test getting a point by valid index
    Point p = cloud.getPoint(1);
    EXPECT_FLOAT_EQ(p.getX(), 4.0f);
    EXPECT_FLOAT_EQ(p.getY(), 5.0f);
    EXPECT_FLOAT_EQ(p.getZ(), 6.0f);
}

TEST(PointCloudTest, GetPointInvalidIndex) {
    PointCloud cloud;
    cloud.addPoint(Point(1.0f, 2.0f, 3.0f));

    // Test getting a point by invalid index (out of range)
    EXPECT_THROW(cloud.getPoint(5), std::out_of_range);  // Expect an exception
}

// Test centroid calculation
TEST(PointCloudTest, CalculateCentroid) {
    PointCloud cloud;
    cloud.addPoint(Point(0.0f, 0.0f, 0.0f));
    cloud.addPoint(Point(2.0f, 2.0f, 2.0f));
    cloud.addPoint(Point(4.0f, 4.0f, 4.0f));

    // Calculate centroid (should be (2.0, 2.0, 2.0))
    Point centroid = cloud.calculateCentroid();
    EXPECT_FLOAT_EQ(centroid.getX(), 2.0f);
    EXPECT_FLOAT_EQ(centroid.getY(), 2.0f);
    EXPECT_FLOAT_EQ(centroid.getZ(), 2.0f);
}

