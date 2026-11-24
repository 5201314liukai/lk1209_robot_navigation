#include <iostream>

#include "obstacle_detection/segment_fitter.hpp"

int main()
{
    using namespace obstacle_detection;

    // 创建一组位于 y=2 的扫描点
    PointSet point_set;

    point_set.addPoint(Point(1.0, 2.0));
    point_set.addPoint(Point(2.0, 2.0));
    point_set.addPoint(Point(3.0, 2.0));
    point_set.addPoint(Point(4.0, 2.0));
    point_set.addPoint(Point(5.0, 2.0));

    // 创建线段拟合器
    SegmentFitter fitter;

    // 执行线段拟合
    Segment segment = fitter.fit(point_set);

    // 输出结果
    std::cout << "========== Segment Fitting =========="
              << std::endl;

    std::cout << "Start: "
              << segment.first_point.x << ", "
              << segment.first_point.y << std::endl;

    std::cout << "End: "
              << segment.last_point.x << ", "
              << segment.last_point.y << std::endl;

    std::cout << "Length: "
              << segment.length() << std::endl;

    std::cout << "====================================="
              << std::endl;

    return 0;
}

