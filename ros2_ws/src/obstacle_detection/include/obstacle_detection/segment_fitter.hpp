#pragma once

#include "obstacle_detection/point_set.hpp"
#include "obstacle_detection/segment.hpp"

namespace obstacle_detection
{

class SegmentFitter
{
public:

    // 根据一组扫描点拟合线段
    Segment fit(const PointSet& point_set);

};

}

