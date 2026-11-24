
#pragma once

#include <cmath>

namespace obstacle_detection
{

class Point
{
public:

    // 构造函数
    Point(double x = 0.0, double y = 0.0)
        : x(x), y(y)
    {
    }

    // 计算向量长度
    double length() const
    {
        return std::sqrt(x * x + y * y);
    }

    // 计算方向角
    double angle() const
    {
        return std::atan2(y, x);
    }

    // 计算两点距离
    double distanceTo(const Point& p) const
    {
        double dx = x - p.x;
        double dy = y - p.y;

        return std::sqrt(dx * dx + dy * dy);
    }

    // 两个点相加
    Point operator+(const Point& p) const
    {
        return Point(x + p.x, y + p.y);
    }

    // 两个点相减
    Point operator-(const Point& p) const
    {
        return Point(x - p.x, y - p.y);
    }

    // 点乘标量：Point * double
    Point operator*(double value) const
    {
        return Point(x * value, y * value);
    }

    // 点积
    double dot(const Point& p) const
    {
        return x * p.x + y * p.y;
    }

    // 二维叉积
    double cross(const Point& p) const
    {
        return x * p.y - y * p.x;
    }

    // 归一化
    Point normalized() const
    {
        double len = length();

        if (len < 1e-12)
            return Point();

        return Point(x / len, y / len);
    }

    // 反向标量乘法：double * Point
    friend Point operator*(double value, const Point& p)
    {
        return Point(value * p.x, value * p.y);
    }

    double x;
    double y;
};

}

