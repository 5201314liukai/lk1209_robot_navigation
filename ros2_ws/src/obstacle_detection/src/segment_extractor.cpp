#include "obstacle_detection/segment_extractor.hpp"

#include <cmath>

namespace obstacle_detection
{

// 构造函数
SegmentExtractor::SegmentExtractor(
    double max_split_distance,
    int min_group_points
)
    : max_split_distance_(max_split_distance),
      min_group_points_(min_group_points)
{
}


// 对点集合执行线段提取
std::vector<Segment> SegmentExtractor::extract(
    const PointSet& point_set)
{
    std::vector<Segment> segments;

    // 点数量不足，不进行提取
    if (static_cast<int>(point_set.size()) < min_group_points_)
    {
        return segments;
    }

    // 开始递归分裂
    split(point_set, segments);

    return segments;
}


// Split and Merge 中的递归分裂部分
void SegmentExtractor::split(
    const PointSet& point_set,
    std::vector<Segment>& segments)
{
    // 点数量不足，停止处理
    if (static_cast<int>(point_set.size()) < min_group_points_)
    {
        return;
    }

    // =====================================
    // 1. 拟合当前点集
    // =====================================

    Segment fitted_segment = fitter_.fit(point_set);

    double max_distance = 0.0;
    std::size_t split_index = 0;

    // =====================================
    // 2. 寻找偏离最远的扫描点
    // =====================================

    for (std::size_t i = 1; i + 1 < point_set.size(); ++i)
    {
        double distance =
            fitted_segment.distanceTo(point_set.points[i]);

        if (distance > max_distance)
        {
            max_distance = distance;
            split_index = i;
        }
    }

    // =====================================
    // 3. 判断是否需要分裂
    // =====================================

    if (max_distance > max_split_distance_ &&
        split_index +1 >= static_cast<std::size_t>(min_group_points_) &&
        point_set.size() - split_index >=
            static_cast<std::size_t>(min_group_points_))
    {
        // 创建左右两个点集合
        PointSet first_group;
        PointSet second_group;

        // 第一组：起点到分裂点
        for (std::size_t i = 0; i <= split_index; ++i)
        {
            first_group.addPoint(point_set.points[i]);
        }

        // 第二组：分裂点到终点
        for (std::size_t i = split_index;
             i < point_set.size(); ++i)
        {
            second_group.addPoint(point_set.points[i]);
        }

        // 递归处理两个点组
        split(first_group, segments);
        split(second_group, segments);
    }
    else
    {
        // =================================
        // 4. 满足条件，保存最终线段
        // =================================

        segments.push_back(fitted_segment);
    }
}

}

