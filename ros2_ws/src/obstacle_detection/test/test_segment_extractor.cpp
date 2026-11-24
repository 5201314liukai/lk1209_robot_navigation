#include <iostream>
#include <vector>

#include "obstacle_detection/segment_extractor.hpp"

int main()
{
    using namespace obstacle_detection;

    // 创建测试点集合
    PointSet point_set;

    // 第一条线段：竖直方向
    point_set.addPoint(Point(0.0, 0.0));
    point_set.addPoint(Point(0.0, 1.0));
    point_set.addPoint(Point(0.0, 2.0));
    point_set.addPoint(Point(0.0, 3.0));
    point_set.addPoint(Point(0.0, 4.0));

    // 第二条线段：水平方向
    point_set.addPoint(Point(1.0, 4.0));
    point_set.addPoint(Point(2.0, 4.0));
    point_set.addPoint(Point(3.0, 4.0));
    point_set.addPoint(Point(4.0, 4.0));

    // 创建线段提取器
    SegmentExtractor extractor(0.2, 5);

    // 执行分裂算法
    std::vector<Segment> segments =
        extractor.extract(point_set);

    // 输出结果
    std::cout << "\n========== Split and Merge Test ==========\n";

    std::cout << "Input points: "
              << point_set.size() << std::endl;

    std::cout << "Detected segments: "
              << segments.size() << std::endl;

    for (std::size_t i = 0; i < segments.size(); ++i)
    {
        const auto& segment = segments[i];

        std::cout << "\nSegment " << i + 1 << std::endl;

        std::cout << "Start: ("
                  << segment.first_point.x << ", "
                  << segment.first_point.y << ")"
                  << std::endl;

        std::cout << "End: ("
                  << segment.last_point.x << ", "
                  << segment.last_point.y << ")"
                  << std::endl;

        std::cout << "Length: "
                  << segment.length()
                  << std::endl;
    }

    std::cout << "\n==========================================\n";

    return 0;
}

