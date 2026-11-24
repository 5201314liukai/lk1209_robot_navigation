#pragma once

#include "obstacle_detection/point.hpp"

namespace obstacle_detection
{

class Segment
{
public:

    // 构造函数：设置线段起点和终点
    Segment(const Point& start = Point(),
            const Point& end = Point())
        : first_point(start),
          last_point(end)
    {
    }

    // 计算线段长度
    double length() const
    {
        return first_point.distanceTo(last_point);
    }

    // 计算线段长度平方
    double lengthSquared() const
    {
        double dx = last_point.x - first_point.x;
        double dy = last_point.y - first_point.y;

        return dx * dx + dy * dy;
    }

    // 计算线段方向向量
    Point direction() const
    {
        return last_point - first_point;
    }

    // 计算单位法向量
    Point normal() const
    {
        Point d = direction();

        double len = d.length();

        if (len < 1e-9)
        {
            return Point();
        }

        return Point(-d.y / len, d.x / len);
    }

    // 计算点在线段所在直线上的投影
    Point projection(const Point& p) const
    {
        Point d = direction();

        double len_squared = d.dot(d);

        if (len_squared < 1e-18)
        {
            return first_point;
        }

        double t = (p - first_point).dot(d) / len_squared;

        return first_point + t * d;
    }

    // 计算点在线段上的最近位置
    Point closestPoint(const Point& p) const
    {
        Point d = direction();

        double len_squared = d.dot(d);

        if (len_squared < 1e-18)
        {
            return first_point;
        }

        double t = (p - first_point).dot(d) / len_squared;

        // 限制投影位置在线段范围内
        if (t < 0.0)
            t = 0.0;

        if (t > 1.0)
            t = 1.0;

        return first_point + t * d;
    }

    // 计算点到线段的最短距离
    double distanceTo(const Point& p) const
    {
        return p.distanceTo(closestPoint(p));
    }

    // 线段起点和终点
    Point first_point;
    Point last_point;
};

}

