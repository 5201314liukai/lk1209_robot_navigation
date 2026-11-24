#pragma once

#include <vector>

#include "obstacle_detection/segment.hpp"
#include "obstacle_detection/segment_fitter.hpp"

namespace obstacle_detection
{

class SegmentMerger
{
public:

    explicit SegmentMerger(
        double max_merge_separation = 0.2,
        double max_merge_spread = 0.2
    );

    // 合并相邻且近似共线的线段
    std::vector<Segment> merge(
        const std::vector<Segment>& segments
    );

private:

    bool canMerge(
        const Segment& first,
        const Segment& second
    ) const;

    double max_merge_separation_;
    double max_merge_spread_;
};

}

