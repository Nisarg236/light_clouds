#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "PointCloud.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>

class Filters {
public:
    static void applySOR(PointCloud& cloud, size_t k, float threshold) {
        std::vector<bool> isOutlier(cloud.size(), false);
        
        for (size_t i = 0; i < cloud.size(); ++i) {
            const Point& point = cloud.getPoint(i);
            std::vector<float> distances;
            
            for (size_t j = 0; j < cloud.size(); ++j) {
                if (i != j) {
                    const Point& otherPoint = cloud.getPoint(j);
                    distances.push_back(point.distanceTo(otherPoint));
                }
            }

            std::nth_element(distances.begin(), distances.begin() + k, distances.end());
            distances.resize(k); 

            float meanDistance = std::accumulate(distances.begin(), distances.end(), 0.0f) / k;

            if (meanDistance > threshold) {
                isOutlier[i] = true;
            }
        }

        size_t j = 0;
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (!isOutlier[i]) {
                cloud.points[j++] = cloud.getPoint(i);
            }
        }

        cloud.points.resize(j); 
    }
};

#endif // FILTERS_HPP
