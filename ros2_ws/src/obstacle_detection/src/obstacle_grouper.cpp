#include "obstacle_detection/obstacle_grouper.hpp"

#include <cmath>

namespace obstacle_detection
{

// 构造函数
ObstacleGrouper::ObstacleGrouper(
    double max_group_distance,
    double distance_proportion,
    int min_group_points
)
    : max_group_distance_(max_group_distance),
      distance_proportion_(distance_proportion),
      min_group_points_(min_group_points)
{
}


// 连续扫描点分组算法
std::vector<PointSet> ObstacleGrouper::groupPoints(
    const std::vector<Point>& points)
{
    std::vector<PointSet> groups;

    // 没有扫描点，直接返回
    if (points.empty())
    {
        return groups;
    }

    // 创建第一个点组
    PointSet current_group;

    current_group.addPoint(points[0]);

    // 遍历后续扫描点
    for (std::size_t i = 1; i < points.size(); ++i)
    {
        const Point& current_point = points[i];

        const Point& previous_point =
            current_group.points.back();

        // 计算相邻点之间的距离
        double distance =
            current_point.distanceTo(previous_point);

        // 当前点到雷达原点的距离
        double range = current_point.length();

        // 计算自适应分组阈值
        double threshold =
            max_group_distance_
            + range * distance_proportion_;

        // 判断是否属于当前点组
        if (distance < threshold)
        {
            current_group.addPoint(current_point);
        }
        else
        {
            // 保存满足最小点数要求的点组
            if (static_cast<int>(current_group.size())
                >= min_group_points_)
            {
                groups.push_back(current_group);
            }

            // 开始新的点组
            current_group.clear();
            current_group.addPoint(current_point);
        }
    }

    // 处理最后一个点组
    if (static_cast<int>(current_group.size())
        >= min_group_points_)
    {
        groups.push_back(current_group);
    }

    return groups;
}

}

