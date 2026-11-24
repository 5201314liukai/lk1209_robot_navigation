#pragma once

#include <vector>

#include "obstacle_detection/point_set.hpp"

namespace obstacle_detection
{

class ObstacleGrouper
{
public:

    // 构造函数：设置点分组参数
    ObstacleGrouper(
        double max_group_distance = 0.1,
        double distance_proportion = 0.00628,
        int min_group_points = 5
    );

    // 将扫描点分成不同的点组
    std::vector<PointSet> groupPoints(
        const std::vector<Point>& points
    );

private:

    double max_group_distance_;
    double distance_proportion_;
    int min_group_points_;
};

}

