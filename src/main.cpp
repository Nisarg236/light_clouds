#include <iostream>
#include "Point.hpp"
#include "PointCloud.hpp"
#include "Filters.hpp"

int PointDemo() {
    Point p1(1.0f, 2.0f, 3.0f);
    Point p2(4.0f, 5.0f, 6.0f);
    
    p1.print();
    p2.print();
    
    std::cout << "Distance between p1 and p2: " << p1.distanceTo(p2) << std::endl;
    
    Point p3 = p1 + p2;
    p3.print();
    
    float dotProduct = p1.dot(p2);
    std::cout << "Dot product: " << dotProduct << std::endl;
    
    Point crossProduct = p1.cross(p2);
    crossProduct.print();
    
    return 0;
}

int PointCloudDemo() {

    Point p1(1.0f, 1.0f, 1.0f);
    Point p2(1.0f, 1.1f, 1.4f);
    Point p3(1.0f, 1.2f, 1.3f);
    Point p4(1.0f, 1.3f, 1.2f);
    Point p5(10.0f, 1.4f, 1.1f); // Outlier point far away from the rest

    PointCloud cloud;
    cloud.addPoint(p1);
    cloud.addPoint(p2);
    cloud.addPoint(p3);
    cloud.addPoint(p4);
    cloud.addPoint(p5);

    std::cout << "Points in the PointCloud (before SOR filter):" << std::endl;
    cloud.print();

    Point centroidBefore = cloud.calculateCentroid();
    std::cout << "Centroid of the PointCloud (before SOR filter): ";
    centroidBefore.print();
    
    size_t k = 2; 
    float threshold = 1.0f;
    Filters::applySOR(cloud, k, threshold);

    std::cout << "\nPoints in the PointCloud (after SOR filter):" << std::endl;
    cloud.print();

    Point centroidAfter = cloud.calculateCentroid();
    std::cout << "Centroid of the PointCloud (after SOR filter): ";
    centroidAfter.print();

    return 0;
}

int main() {
    PointDemo();
    PointCloudDemo(); 
    return 0;
}
