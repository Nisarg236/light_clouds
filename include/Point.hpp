#ifndef POINT_HPP
#define POINT_HPP

#include <iostream>
#include <cmath>
#include <set>
class Point
{
public:
    float x, y, z;
    std::set<std::string> tags; // Set of tags given to a point cloud

    Point() : x(0), y(0), z(0)
    {
        tags.insert("point");
    }


    Point(float x, float y, float z) : x(x), y(y), z(z)
    {
        tags.insert("point");
    }

    bool hasTag(const std::string &tag) const
    {
        return tags.find(tag) != tags.end();
    }

    void removeTag(const std::string &tag)
    {
        tags.erase(tag);
    }

    ~Point() {}

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    void setX(float x) { this->x = x; }
    void setY(float y) { this->y = y; }
    void setZ(float z) { this->z = z; }

    void addTag(const std::string &tag)
    {
        tags.insert(tag);
    }

    const std::set<std::string> &getTags() const
    {
        return tags;
    }

    void print(int mode=0) const
    {
        std::cout << x << ", " << y << ", " << z;
        if (!tags.empty() && mode == 1)
        {
            std::cout << " Tags: {";
            for (const auto &tag : tags)
            {
                std::cout << tag << ", ";
            }
            std::cout << "\b\b}";
        }
        std::cout << std::endl;
    }

    float distanceTo(const Point &other) const
    {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2) + std::pow(other.z - z, 2));
    }

    bool operator==(const Point &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }

    Point operator-(const Point &other) const
    {
        return Point(x - other.x, y - other.y, z - other.z);
    }

    Point operator+(const Point &other) const
    {
        return Point(x + other.x, y + other.y, z + other.z);
    }

    Point operator*(float scalar) const
    {
        return Point(x * scalar, y * scalar, z * scalar);
    }

    Point operator/(float scalar) const
    {
        if (scalar != 0)
        {
            return Point(x / scalar, y / scalar, z / scalar);
        }
        std::cerr << "Error: Division by zero!\n";
        return *this;
    }

    float dot(const Point &other) const
    {
        return x * other.x + y * other.y + z * other.z;
    }

    Point cross(const Point &other) const
    {
        return Point(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x);
    }
};

#endif // POINT_HPP
