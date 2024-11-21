#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <vector>
#include "Point.hpp"

class PointCloud
{
public:
    std::vector<Point> points;

    PointCloud() = default;

    PointCloud(const std::vector<Point> &points) : points(points) {}

    ~PointCloud() = default;

    void addPoint(const Point &point)
    {
        points.push_back(point);
    }

    size_t size() const
    {
        return points.size();
    }

    Point getPoint(size_t index) const
    {
        if (index < points.size())
        {
            return points[index];
        }
        else
        {
            std::cerr << "Error: Attempt to access Point at index " << index
                      << " in a PointCloud with size " << points.size() << std::endl;
            throw std::out_of_range("Index out of range");
        }
    }

    void print() const
    {
        for (const auto &point : points)
        {
            point.print();
        }
    }

    Point calculateCentroid() const
    {
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto &point : points)
        {
            sum_x += point.getX();
            sum_y += point.getY();
            sum_z += point.getZ();
        }
        size_t count = points.size();
        if (count > 0)
        {
            return Point(sum_x / count, sum_y / count, sum_z / count);
        }
        else
        {
            return Point();
        }
    }

    void transform(float scalar)
    {
        for (auto &point : points)
        {
            point = point * scalar;
        }
    }

    PointCloud operator+(const PointCloud &other) const
    {
        PointCloud new_cloud = *this;
        for (const auto &point : other.points)
        {
            new_cloud.addPoint(point);
        }
        return new_cloud;
    }

    PointCloud operator+(const Point &other) const
    {
        PointCloud new_cloud = *this;
        new_cloud.addPoint(other);
        return new_cloud;
    }

    Point getMaximumPointInAxis(char axis)
    {
        if (points.empty())
        {
            throw std::runtime_error("PointCloud is empty.");
        }

        Point max_point = points[0];
        float max_value;

        switch (axis)
        {
        case 'x':
        {
            max_value = max_point.getX();
            for (const auto &point : points)
            {
                if (point.getX() > max_value)
                {
                    max_value = point.getX();
                    max_point = point;
                }
            }
            break;
        }
        case 'y':
        {
            max_value = max_point.getY();
            for (const auto &point : points)
            {
                if (point.getY() > max_value)
                {
                    max_value = point.getY();
                    max_point = point;
                }
            }
            break;
        }
        case 'z':
        {
            max_value = max_point.getZ();
            for (const auto &point : points)
            {
                if (point.getZ() > max_value)
                {
                    max_value = point.getZ();
                    max_point = point;
                }
            }
            break;
        }
        default:
            throw std::invalid_argument("Invalid axis input. Valid values are 'x', 'y', or 'z'.");
        }
        return max_point;
    }

    Point getMinmumPointInAxis(char axis)
    {
        if (points.empty())
        {
            throw std::runtime_error("PointCloud is empty.");
        }

        Point min_point = points[0];
        float min_value;

        switch (axis)
        {
        case 'x':
        {
            min_value = min_point.getX();
            for (const auto &point : points)
            {
                if (point.getX() < min_value)
                {
                    min_value = point.getX();
                    min_point = point;
                }
            }
            break;
        }
        case 'y':
        {
            min_value = min_point.getY();
            for (const auto &point : points)
            {
                if (point.getY() < min_value)
                {
                    min_value = point.getY();
                    min_point = point;
                }
            }
            break;
        }
        case 'z':
        {
            min_value = min_point.getZ();
            for (const auto &point : points)
            {
                if (point.getZ() > min_value)
                {
                    min_value = point.getZ();
                    min_point = point;
                }
            }
            break;
        }
        default:
            throw std::invalid_argument("Invalid axis input. Valid values are 'x', 'y', or 'z'.");
        }
        return min_point;
    }

    PointCloud crop(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) const
    {
        PointCloud croppedCloud;
        for (const auto &point : points)
        {
            float x = point.getX();
            float y = point.getY();
            float z = point.getZ();

            // Check if the point lies within the bounds
            if (x >= min_x && x <= max_x &&
                y >= min_y && y <= max_y &&
                z >= min_z && z <= max_z)
            {
                croppedCloud.addPoint(point);
            }
        }
        return croppedCloud;
    }

};

#endif // POINTCLOUD_HPP
