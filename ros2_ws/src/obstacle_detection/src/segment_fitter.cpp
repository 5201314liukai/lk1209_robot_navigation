#include "obstacle_detection/segment_fitter.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
namespace obstacle_detection
{

Segment SegmentFitter::fit(const PointSet& point_set)
{
    // 至少需要两个点
    if (point_set.size() < 2)
    {
        return Segment();
    }

    // =====================================
    // 1. 计算点集中心
    // =====================================

    double mean_x = 0.0;
    double mean_y = 0.0;

    for (const Point& point : point_set.points)
    {
        mean_x += point.x;
        mean_y += point.y;
    }

    mean_x /= point_set.size();
    mean_y /= point_set.size();

    // =====================================
    // 2. 计算协方差矩阵
    // =====================================

    double xx = 0.0;
    double yy = 0.0;
    double xy = 0.0;

    for (const Point& point : point_set.points)
    {
        double dx = point.x - mean_x;
        double dy = point.y - mean_y;

        xx += dx * dx;
        yy += dy * dy;
        xy += dx * dy;
    }

    // =====================================
    // 3. 计算最佳拟合方向
    // =====================================

    double theta = 0.5 * std::atan2(
        2.0 * xy,
        xx - yy
    );

    Point direction(
        std::cos(theta),
        std::sin(theta)
    );

    // =====================================
    // 4. 将所有点投影到拟合方向
    // =====================================

    double min_projection =
        std::numeric_limits<double>::max();

    double max_projection =
        std::numeric_limits<double>::lowest();

    for (const Point& point : point_set.points)
    {
        Point centered(
            point.x - mean_x,
            point.y - mean_y
        );

        double projection = centered.dot(direction);

        min_projection = std::min(
            min_projection,
            projection
        );

        max_projection = std::max(
            max_projection,
            projection
        );
    }

    // =====================================
    // 5. 计算线段起点和终点
    // =====================================

    Point first_point(
        mean_x + min_projection * direction.x,
        mean_y + min_projection * direction.y
    );

    Point last_point(
        mean_x + max_projection * direction.x,
        mean_y + max_projection * direction.y
    );

    return Segment(first_point, last_point);
}

}

