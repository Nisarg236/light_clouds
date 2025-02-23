#include <iostream>
#include "Point.hpp"
#include "PointCloud.hpp"
#include "Filters.hpp"

int PointDemo()
{
    Point p1(1.0f, 2.0f, 3.0f);
    Point p2(4.0f, 5.0f, 6.0f);

    p1.print(1);
    p2.print(0);

    std::cout << "Distance between p1 and p2: " << p1.distanceTo(p2) << std::endl;

    Point p3 = p1 + p2;
    p3.print();

    float dot_product = p1.dot(p2);
    std::cout << "Dot product: " << dot_product << std::endl;

    Point cross_product = p1.cross(p2);
    cross_product.print();

    return 0;
}

int PointCloudDemo()
{

    Point p1(1.0f, 1.0f, 1.0f);
    Point p2(1.0f, 1.1f, 1.4f);
    Point p3(1.0f, 1.2f, 1.3f);
    Point p4(1.0f, 1.3f, 1.2f);
    Point p5(10.0f, 1.4f, 1.1f); // Outlier point far away from the rest
    Point p6(1.0f, 1.0f, 1.0f);

    PointCloud cloud;
    cloud.addPoint(p1);
    cloud.addPoint(p2);
    cloud.addPoint(p3);
    cloud.addPoint(p4);
    cloud.addPoint(p5);
    cloud.addPoint(p6);

    std::cout << "Points in the PointCloud (before SOR filter):" << std::endl;
    cloud.print();

    Point centroid_before = cloud.calculateCentroid();
    std::cout << "Centroid of the PointCloud (before SOR filter): ";
    centroid_before.print();

    size_t k = 2;
    float threshold = 1.0f;
    Filters::applySOR(cloud, k, threshold);

    std::cout << "\nPoints in the PointCloud (after SOR filter):" << std::endl;
    cloud.print();

    Point centroid_after = cloud.calculateCentroid();
    std::cout << "Centroid of the PointCloud (after SOR filter): ";
    centroid_after.print();

    return 0;
}

int fileIODemo()
{

    PointCloud cloud;
    cloud.loadFromPCD("/home/nisarg/light_clouds/files/lamppost.pcd");
    std::cout << "Input cloud has: "<< cloud.size() << "number of points" << std::endl;
    std::cout << "Centroid: ";
    cloud.calculateCentroid().print();
    cloud.dumpToPCD("/home/nisarg/light_clouds/files/output.pcd");
    return 0;
}

int main()
{
    PointDemo();
    PointCloudDemo();
    fileIODemo();
    return 0;
}
