#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include "obstacle_detection/point_set.hpp"
#include "obstacle_detection/segment.hpp"

namespace obstacle_detection
{

struct ScanFeatures
{
    std::vector<Segment> segments;
    std::vector<Point> points;
};

class ScanProcessor
{
public:

    // 将 LaserScan 转换成二维扫描点集合
    PointSet process(
        const sensor_msgs::msg::LaserScan& scan
    );

    ScanFeatures features(const sensor_msgs::msg::LaserScan& scan);

};

}

