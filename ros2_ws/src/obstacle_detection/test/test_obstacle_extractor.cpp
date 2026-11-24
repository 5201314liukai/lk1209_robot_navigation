#include <iostream>

#include "obstacle_detection/obstacle_extractor.hpp"

int main()
{
    using namespace obstacle_detection;

    // 创建一个包含 90° 拐角的测试点集
    PointSet point_set;

    // 竖直边缘
    point_set.addPoint(Point(0.0, 0.0));
    point_set.addPoint(Point(0.0, 1.0));
    point_set.addPoint(Point(0.0, 2.0));
    point_set.addPoint(Point(0.0, 3.0));
    point_set.addPoint(Point(0.0, 4.0));

    // 水平边缘
    point_set.addPoint(Point(1.0, 4.0));
    point_set.addPoint(Point(2.0, 4.0));
    point_set.addPoint(Point(3.0, 4.0));
    point_set.addPoint(Point(4.0, 4.0));

    // 创建完整障碍物提取器
    ObstacleExtractor extractor(
        0.2,
        0.2,
        0.2,
        5
    );

    // 执行完整提取流程
    auto segments = extractor.extract(point_set);

    std::cout << "\n========== Obstacle Extractor Test ==========\n";

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
                  << segment.length() << std::endl;
    }

    std::cout << "\n=============================================\n";

    return 0;
}

