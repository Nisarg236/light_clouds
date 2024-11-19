#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <vector>
#include "Point.hpp"

class PointCloud {
public:
    std::vector<Point> points; 

    PointCloud() = default;

    PointCloud(const std::vector<Point>& points) : points(points) {}

    ~PointCloud() = default;

    void addPoint(const Point& point) {
        points.push_back(point);
    }

    size_t size() const {
        return points.size();
    }

    Point getPoint(size_t index) const {
        // Check if the index is valid
        if (index < points.size()) {
            return points[index];  
        } else {
            std::cerr << "Error: Attempt to access Point at index " << index
                    << " in a PointCloud with size " << points.size() << std::endl;
            throw std::out_of_range("Index out of range"); 
        }
    }

    void print() const {
        for (const auto& point : points) {
            point.print();
        }
    }

    Point calculateCentroid() const {
        float sumX = 0, sumY = 0, sumZ = 0;
        for (const auto& point : points) {
            sumX += point.getX();
            sumY += point.getY();
            sumZ += point.getZ();
        }
        size_t count = points.size();
        if (count > 0) {
            return Point(sumX / count, sumY / count, sumZ / count);
        } else {
            return Point(); 
        }
    }

    void transform(float scalar) {
        for (auto& point : points) {
            point = point * scalar;
        }
    }

    PointCloud operator+(const PointCloud& other) const {
        PointCloud newCloud = *this; 
        for (const auto& point : other.points) {
            newCloud.addPoint(point);
        }
        return newCloud; 
    }

    PointCloud operator+(const Point& other) const {
        PointCloud newCloud = *this; 
            newCloud.addPoint(other);
        return newCloud;
    }

};

#endif // POINTCLOUD_HPP
