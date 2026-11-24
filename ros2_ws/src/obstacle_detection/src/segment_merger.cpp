#include "obstacle_detection/segment_merger.hpp"

#include <algorithm>
#include <cmath>

namespace obstacle_detection
{

// 构造函数
SegmentMerger::SegmentMerger(
    double max_merge_separation,
    double max_merge_spread
)
    : max_merge_separation_(max_merge_separation),
      max_merge_spread_(max_merge_spread)
{
}


// 判断两条线段是否可以合并
bool SegmentMerger::canMerge(
    const Segment& first,
    const Segment& second) const
{
    // 1. 检查两条线段端点之间的最小距离
    double min_distance = std::min({
        first.distanceTo(second.first_point),
        first.distanceTo(second.last_point),
        second.distanceTo(first.first_point),
        second.distanceTo(first.last_point)
    });

    if (min_distance > max_merge_separation_)
    {
        return false;
    }

    // 2. 找出四个端点中距离最远的两个点
    Point endpoints[4] = {
        first.first_point,
        first.last_point,
        second.first_point,
        second.last_point
    };

    double max_length_squared = 0.0;

    Point start;
    Point end;

    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            double distance_squared =
                endpoints[i].distanceTo(endpoints[j]);

            distance_squared *= distance_squared;

            if (distance_squared > max_length_squared)
            {
                max_length_squared = distance_squared;
                start = endpoints[i];
                end = endpoints[j];
            }
        }
    }

    // 3. 根据最远的两个端点构造候选线段
    Segment candidate(start, end);

    // 4. 检查四个端点是否接近候选线段
    for (const Point& point : endpoints)
    {
        if (candidate.distanceTo(point) > max_merge_spread_)
        {
            return false;
        }
    }

    return true;
}


// 合并线段
std::vector<Segment> SegmentMerger::merge(
    const std::vector<Segment>& input_segments)
{
    std::vector<Segment> segments = input_segments;

    bool merged = true;

    // 持续寻找可以合并的线段
    while (merged)
    {
        merged = false;

        for (std::size_t i = 0; i < segments.size(); ++i)
        {
            for (std::size_t j = i + 1;
                 j < segments.size(); ++j)
            {
                if (canMerge(segments[i], segments[j]))
                {
                    // 找出两个线段中距离最远的端点
                    Point endpoints[4] = {
                        segments[i].first_point,
                        segments[i].last_point,
                        segments[j].first_point,
                        segments[j].last_point
                    };

                    double max_length_squared = 0.0;

                    Point start;
                    Point end;

                    for (int a = 0; a < 4; ++a)
                    {
                        for (int b = a + 1; b < 4; ++b)
                        {
                            double distance_squared =
                                endpoints[a].distanceTo(endpoints[b]);

                            distance_squared *= distance_squared;

                            if (distance_squared > max_length_squared)
                            {
                                max_length_squared = distance_squared;
                                start = endpoints[a];
                                end = endpoints[b];
                            }
                        }
                    }

                    // 用新线段替换原来的两条线段
                    segments[i] = Segment(start, end);

                    segments.erase(
                        segments.begin() +
                        static_cast<std::ptrdiff_t>(j)
                    );

                    merged = true;

                    break;
                }
            }

            if (merged)
            {
                break;
            }
        }
    }

    return segments;
}

}

