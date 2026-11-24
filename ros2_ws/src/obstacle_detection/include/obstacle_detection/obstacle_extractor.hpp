#pragma once

#include <vector>

#include "obstacle_detection/point_set.hpp"
#include "obstacle_detection/segment.hpp"
#include "obstacle_detection/segment_extractor.hpp"
#include "obstacle_detection/segment_merger.hpp"

namespace obstacle_detection
{

class ObstacleExtractor
{
public:

    ObstacleExtractor(
        double max_split_distance = 0.2,
        double max_merge_separation = 0.2,
        double max_merge_spread = 0.2,
        int min_group_points = 5
    );

    // 完整障碍物线段提取流程
    std::vector<Segment> extract(
        const PointSet& point_set
    );

private:

    SegmentExtractor segment_extractor_;
    SegmentMerger segment_merger_;
};

}

