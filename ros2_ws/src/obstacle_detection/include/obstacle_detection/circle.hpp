#pragma once

#include "obstacle_detection/point.hpp"

namespace obstacle_detection
{

class Circle
{
public:

    // 构造函数：设置圆心和半径
    Circle(const Point& center = Point(),
           double radius = 0.0)
        : center(center),
          radius(radius)
    {
    }

    // 计算圆心到指定点的距离
    double centerDistance(const Point& p) const
    {
        return center.distanceTo(p);
    }

    // 计算点到圆形边界的距离
    double distanceTo(const Point& p) const
    {
        return center.distanceTo(p) - radius;
    }

    // 判断指定点是否位于圆形内部
    bool contains(const Point& p) const
    {
        return centerDistance(p) <= radius;
    }

    // 圆心坐标和半径
    Point center;
    double radius;
};

}

