#pragma once
// placeholder — filled in next
#include <cmath>

struct Vec2
{
    double x = 0.0;
    double y = 0.0;

    Vec2 operator+(const Vec2 &other) const
    {
        return {x + other.x, y + other.y};
    }

    Vec2 operator-(const Vec2 &other) const
    {
        return {x - other.x, y - other.y};
    }

    Vec2 operator*(double scalar) const
    {
        return {x * scalar, y * scalar};
    }

    Vec2 &operator+=(const Vec2 &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    double norm() const
    {
        return std::sqrt(x * x + y * y);
    }
};
