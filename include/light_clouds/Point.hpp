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
    }

    Point(float x, float y, float z) : x(x), y(y), z(z)
    {
    }

    ~Point() {}

    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    void setX(float x) { this->x = x; }
    void setY(float y) { this->y = y; }
    void setZ(float z) { this->z = z; }


    void print() const
    {
        std::cout << x << ", " << y << ", " << z << std::endl;
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
