#include "Point.hpp"
#include "PointCloud.hpp"


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
    
    //  product
    Point crossProduct = p1.cross(p2);
    crossProduct.print();
    
    return 0;
}

int PointCloudDemo() {
    // Create some points
    Point p1(1.0f, 2.0f, 3.0f);
    Point p2(4.0f, 5.0f, 6.0f);
    Point p3(7.0f, 8.0f, 9.0f);
    Point p4(10.0f, 11.0f, 12.0f);

    // Create a PointCloud
    PointCloud cloud;
    cloud.addPoint(p1);
    cloud.addPoint(p2);
    cloud.addPoint(p3);
    cloud.addPoint(p4);

    // Print the points in the cloud
    std::cout << "Points in the PointCloud:" << std::endl;
    cloud.print();

    // Calculate the centroid of the PointCloud
    Point centroid = cloud.calculateCentroid();
    std::cout << "Centroid of the PointCloud: ";
    centroid.print();
}

int main(){
    PointDemo();
    PointCloudDemo();
    return 0;
}
