#include "obstacle_detection/obstacle_extractor.hpp"

namespace obstacle_detection
{

// 构造函数
ObstacleExtractor::ObstacleExtractor(
    double max_split_distance,
    double max_merge_separation,
    double max_merge_spread,
    int min_group_points
)
    : segment_extractor_(
        max_split_distance,
        min_group_points
      ),
      segment_merger_(
        max_merge_separation,
        max_merge_spread
      )
{
}


// 完整障碍物线段提取流程
std::vector<Segment> ObstacleExtractor::extract(
    const PointSet& point_set)
{
    // 第一步：Split，提取初始线段
    std::vector<Segment> segments =
        segment_extractor_.extract(point_set);

    // 第二步：Merge，合并相邻线段
    std::vector<Segment> merged_segments =
        segment_merger_.merge(segments);

    return merged_segments;
}

}

