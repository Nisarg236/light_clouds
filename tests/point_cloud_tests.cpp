#include <gtest/gtest.h>
#include "light_clouds/PointCloud.hpp"
#include "light_clouds/Point.hpp"

// Test adding points to PointCloud
TEST(PointCloudTest, AddPoint)
{
    PointCloud cloud;
    cloud.addPoint(Point(1, 2, 3));
    ASSERT_EQ(cloud.size(), 1);
    EXPECT_EQ(cloud.getPoint(0).getX(), 1);
    EXPECT_EQ(cloud.getPoint(0).getY(), 2);
    EXPECT_EQ(cloud.getPoint(0).getZ(), 3);
}

// Test centroid function
TEST(PointCloudTest, Centroid)
{
    PointCloud cloud({Point(1, 1, 1), Point(2, 2, 2), Point(3, 3, 3)});
    Point centroid = cloud.calculateCentroid();
    EXPECT_EQ(centroid.getX(), 2);
    EXPECT_EQ(centroid.getY(), 2);
    EXPECT_EQ(centroid.getZ(), 2);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// Test PointCloud cropping function
TEST(PointCloudTest, Crop)
{
    PointCloud cloud({Point(1, 2, 3), Point(4, 5, 6), Point(7, 8, 9), Point(2, 3, 4)});
    PointCloud croppedCloud = cloud.crop(2, 6, 2, 6, 3, 6);

    EXPECT_EQ(croppedCloud.size(), 2); // Only points (4, 5, 6) and (2, 3, 4) should be in the cropped cloud
}

// Test creating ROI (Region of Interest)
TEST(PointCloudTest, CreateROI)
{
    // Define a set of points
    PointCloud cloud;
    cloud.addPoint(Point(1.0, 2.0, 3.0));
    cloud.addPoint(Point(4.0, 5.0, 6.0));
    cloud.addPoint(Point(7.0, 8.0, 9.0));

    // Define ROI bounds
    float min_x = 2.0, max_x = 8.0;
    float min_y = 3.0, max_y = 10.0;
    float min_z = 2.0, max_z = 7.0;

    // Get ROI object
    PointCloud::ROI roi = cloud.createROI(min_x, max_x, min_y, max_y, min_z, max_z);

    // Validate ROI indices
    ASSERT_EQ(roi.indices.size(), 1);
    EXPECT_EQ(roi.indices[0], 1); // Point (4.0, 5.0, 6.0) should be in the ROI

    // Validate ROI points
    std::vector<Point> roiPoints = roi.getPoints();
    ASSERT_EQ(roiPoints.size(), 1);
    EXPECT_EQ(roiPoints[0].getX(), 4.0);
    EXPECT_EQ(roiPoints[0].getY(), 5.0);
    EXPECT_EQ(roiPoints[0].getZ(), 6.0);
}

// Test PointCloud size and empty check
TEST(PointCloudTest, EmptyCloud)
{
    PointCloud cloud;
    EXPECT_EQ(cloud.size(), 0); // Cloud should be empty initially

    cloud.addPoint(Point(1, 2, 3));
    EXPECT_EQ(cloud.size(), 1); // Cloud should have one point after adding
}

// Test distance between two points
TEST(PointTest, Distance)
{
    Point p1(1.0, 1.0, 1.0);
    Point p2(2.0, 1.0, 1.0);

    float distance = p1.distanceTo(p2);
    EXPECT_FLOAT_EQ(distance, 1.0); // Distance should be sqrt((4-1)^2 + (5-2)^2 + (6-3)^2)
}

// Test addition and multiplication of points
TEST(PointTest, AddAndMultiply)
{
    Point p1(1.0, 2.0, 3.0);
    Point p2(4.0, 5.0, 6.0);

    Point p3 = p1 + p2;
    EXPECT_EQ(p3.getX(), 5.0);
    EXPECT_EQ(p3.getY(), 7.0);
    EXPECT_EQ(p3.getZ(), 9.0);

    Point p4 = p1 * 2.0;
    EXPECT_EQ(p4.getX(), 2.0);
    EXPECT_EQ(p4.getY(), 4.0);
    EXPECT_EQ(p4.getZ(), 6.0);
}