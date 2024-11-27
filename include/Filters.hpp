#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "PointCloud.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <map>

class Filters
{
public:
    static void applySOR(PointCloud &cloud, size_t k, float threshold)
    {
        std::vector<bool> is_outlier(cloud.size(), false);

        for (size_t i = 0; i < cloud.size(); ++i)
        {
            const Point &point = cloud.getPoint(i);
            std::vector<float> distances;

            for (size_t j = 0; j < cloud.size(); ++j)
            {
                if (i != j)
                {
                    const Point &other_point = cloud.getPoint(j);
                    distances.push_back(point.distanceTo(other_point));
                }
            }

            std::nth_element(distances.begin(), distances.begin() + k, distances.end());
            distances.resize(k);

            float mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0f) / k;

            if (mean_distance > threshold)
            {
                is_outlier[i] = true;
            }
        }

        size_t j = 0;
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            if (!is_outlier[i])
            {
                cloud.points[j++] = cloud.getPoint(i);
            }
        }

        cloud.points.resize(j);
    }

    static PointCloud applyVoxelGrid(const PointCloud &cloud, float voxel_size)
    {
        if (voxel_size <= 0)
        {
            throw std::invalid_argument("Voxel size must be greater than 0.");
        }

        std::map<std::tuple<int, int, int>, PointCloud> voxel_map;

        for (const auto &point : cloud.points)
        {
            int voxel_x = static_cast<int>(std::floor(point.getX() / voxel_size));
            int voxel_y = static_cast<int>(std::floor(point.getY() / voxel_size));
            int voxel_z = static_cast<int>(std::floor(point.getZ() / voxel_size));

            std::tuple<int, int, int> voxel_key = std::make_tuple(voxel_x, voxel_y, voxel_z);
            voxel_map[voxel_key].addPoint(point);
        }

        PointCloud filtered_cloud;

        for (const auto &[voxel, voxel_points] : voxel_map)
        {
            filtered_cloud.addPoint(voxel_points.calculateCentroid());
        }

        return filtered_cloud;
    }

    static void applyROR(PointCloud &cloud, float radius, size_t min_neighbors) 
    {
        std::vector<bool> is_outlier(cloud.size(), false);

        for (size_t i = 0; i < cloud.size(); ++i) {
            size_t neighbor_count = 0;

            for (size_t j = 0; j < cloud.size(); ++j) {
                if (i != j && cloud.getPoint(i).distanceTo(cloud.getPoint(j)) < radius) {
                    neighbor_count++;
                }
            }

            if (neighbor_count < min_neighbors) {
                is_outlier[i] = true;
            }
        }

        size_t j = 0;
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (!is_outlier[i]) {
                cloud.points[j++] = cloud.getPoint(i);
            }
        }

        cloud.points.resize(j);
    }

    static void applyPassThrough(PointCloud &cloud, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) 
    {
        size_t j = 0;

        for (size_t i = 0; i < cloud.size(); ++i) {
            const Point &point = cloud.getPoint(i);
            if (point.x >= min_x && point.x <= max_x &&
                point.y >= min_y && point.y <= max_y &&
                point.z >= min_z && point.z <= max_z) {
                cloud.points[j++] = point;
            }
        }

        cloud.points.resize(j);
    }
};

#endif // FILTERS_HPP
