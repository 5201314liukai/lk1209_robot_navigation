#include <iostream>
#include <vector>

#include "obstacle_detection/segment_merger.hpp"

int main()
{
    using namespace obstacle_detection;

    std::vector<Segment> segments;

    // 第一条线段：(0,0) 到 (2,0)
    segments.emplace_back(
        Point(0.0, 0.0),
        Point(2.0, 0.0)
    );

    // 第二条线段：(2,0) 到 (4,0)
    segments.emplace_back(
        Point(2.0, 0.0),
        Point(4.0, 0.0)
    );

    // 第三条线段：(4,0) 到 (6,0)
    segments.emplace_back(
        Point(4.0, 0.0),
        Point(6.0, 0.0)
    );

    // 创建线段合并器
    SegmentMerger merger(0.2, 0.2);

    // 执行合并
    auto result = merger.merge(segments);

    std::cout << "\n========== Segment Merge Test ==========\n";

    std::cout << "Input segments: "
              << segments.size() << std::endl;

    std::cout << "Merged segments: "
              << result.size() << std::endl;

    for (std::size_t i = 0; i < result.size(); ++i)
    {
        std::cout << "\nSegment " << i + 1 << std::endl;

        std::cout << "Start: ("
                  << result[i].first_point.x << ", "
                  << result[i].first_point.y << ")"
                  << std::endl;

        std::cout << "End: ("
                  << result[i].last_point.x << ", "
                  << result[i].last_point.y << ")"
                  << std::endl;

        std::cout << "Length: "
                  << result[i].length() << std::endl;
    }

    std::cout << "\n========================================\n";

    return 0;
}

