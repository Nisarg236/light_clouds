#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "PointCloud.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>

class Filters {
public:
    static void applySOR(PointCloud& cloud, size_t k, float threshold) {
        std::vector<bool> is_outlier(cloud.size(), false);

        for (size_t i = 0; i < cloud.size(); ++i) {
            const Point& point = cloud.getPoint(i);
            std::vector<float> distances;

            for (size_t j = 0; j < cloud.size(); ++j) {
                if (i != j) {
                    const Point& other_point = cloud.getPoint(j);
                    distances.push_back(point.distanceTo(other_point));
                }
            }

            std::nth_element(distances.begin(), distances.begin() + k, distances.end());
            distances.resize(k);

            float mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0f) / k;

            if (mean_distance > threshold) {
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

    static PointCloud applyVoxelGrid(const PointCloud& cloud, float voxel_size) {
        //todo
    }
};

#endif // FILTERS_HPP
