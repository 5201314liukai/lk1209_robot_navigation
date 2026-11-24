#pragma once

#include <vector>
#include <cstddef>

#include "obstacle_detection/point.hpp"

namespace obstacle_detection
{

class PointSet
{
public:

    // 保存一组二维扫描点
    std::vector<Point> points;

    // 点集合是否可见
    bool is_visible = true;

    // 获取点的数量
    std::size_t size() const
    {
        return points.size();
    }

    // 判断集合是否为空
    bool empty() const
    {
        return points.empty();
    }

    // 添加一个扫描点
    void addPoint(const Point& point)
    {
        points.push_back(point);
    }

    // 清空点集合
    void clear()
    {
        points.clear();
        is_visible = true;
    }

};

}

