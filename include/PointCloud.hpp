#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <vector>
#include "Point.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
class PointCloud
{
public:
    std::vector<Point> points;
    std::unordered_map<std::string, std::vector<size_t>>tagMap;

    PointCloud() = default;

    PointCloud(const std::vector<Point> &points) : points(points) {}

    ~PointCloud() = default;

void addPoint(const Point &point)
{
    points.push_back(point);

    // Add the new point's index to the tagMap
    size_t index = points.size() - 1;
    const auto &tags = point.getTags();
    for (const auto &tag : tags)
    {
        tagMap[tag].push_back(index);
    }
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
            point.print(1);
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

PointCloud getSubsetByTags(const std::set<std::string> &inputTags) const
{
    PointCloud subset;

    for (const auto &tag : inputTags)
    {
        auto it = tagMap.find(tag);
        if (it != tagMap.end())
        {
            for (const auto &index : it->second)
            {
                if (index < points.size()) // Ensure the index is valid
                {
                    subset.addPoint(points[index]);
                }
                else
                {
                    std::cerr << "Warning: Invalid index in tagMap for tag '" << tag << "'." << std::endl;
                }
            }
        }
    }

    return subset;
}

    void loadFromPCD(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + filename);
        }

        size_t point_count = 0, data_start = 0;
        if (!parseHeader(file, point_count, data_start))
        {
            throw std::runtime_error("Invalid PCD header.");
        }

        file.clear();
        file.seekg(0, std::ios::beg);

        std::string line;
        for (size_t i = 0; i < data_start; ++i)
        {
            std::getline(file, line);
        }

        points.clear();
        while (std::getline(file, line))
        {
            if (line.empty())
                continue;

            std::istringstream iss(line);
            float x, y, z;

            if (!(iss >> x >> y >> z))
            {
                throw std::runtime_error("Error parsing point data in PCD file.");
            }

            points.emplace_back(x, y, z);
        }

        if (points.size() != point_count)
        {
            std::cerr << "Warning: Parsed points (" << points.size()
                      << ") do not match the declared count (" << point_count << ")." << std::endl;
        }

        file.close();
    }

    void dumpToPCD(const std::string &filename) const
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + filename);
        }

        file << "# .PCD v0.7 - Point Cloud Data file format\n";
        file << "VERSION 0.7\n";
        file << "FIELDS x y z\n";
        file << "SIZE 4 4 4\n";
        file << "TYPE F F F\n";
        file << "COUNT 1 1 1\n";
        file << "WIDTH " << points.size() << "\n";
        file << "HEIGHT 1\n";
        file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        file << "POINTS " << points.size() << "\n";
        file << "DATA ascii\n";

        for (const auto &point : points)
        {
            file << point.getX() << " " << point.getY() << " " << point.getZ() << "\n";
        }

        file.close();

        std::cout << "PointCloud saved to " << filename << std::endl;
    }

private:
    bool parseHeader(std::ifstream &file, size_t &point_count, size_t &data_start)
    {
        std::string line;
        size_t line_index = 0;
        bool data_ascii = false;

        while (std::getline(file, line))
        {
            ++line_index;
            std::istringstream iss(line);
            std::string key;

            if (line.empty())
                continue;

            if (line.find("DATA") == 0)
            {
                std::string data_type;
                iss >> key >> data_type;
                if (data_type != "ascii")
                {
                    throw std::runtime_error("Only ASCII format is supported.");
                }
                data_ascii = true;
                break;
            }
            else if (line.find("POINTS") == 0)
            {
                iss >> key >> point_count;
            }
        }

        if (!data_ascii)
        {
            throw std::runtime_error("Failed to find ASCII data section in PCD file.");
        }

        data_start = line_index;
        return true;
    }
};

#endif // POINTCLOUD_HPP
