#pragma once

#include <vector>

#include "obstacle_detection/point_set.hpp"
#include "obstacle_detection/segment.hpp"
#include "obstacle_detection/segment_fitter.hpp"

namespace obstacle_detection
{

class SegmentExtractor
{
public:

    // 构造函数
    explicit SegmentExtractor(
        double max_split_distance = 0.2,
        int min_group_points = 5
    );

    // 对点集合执行 Split and Merge
    std::vector<Segment> extract(
        const PointSet& point_set
    );

private:

    // 递归分裂
    void split(
        const PointSet& point_set,
        std::vector<Segment>& segments
    );

    double max_split_distance_;
    int min_group_points_;

    SegmentFitter fitter_;
};

}

