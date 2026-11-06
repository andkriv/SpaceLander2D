#pragma once
#include <cmath>

struct Vec2
{
    float x;
    float y;
};

inline Vec2  operator+(Vec2 a, Vec2 b)
{
    return { a.x + b.x, a.y + b.y };
}

inline Vec2  operator-(Vec2 a, Vec2 b)
{
    return { a.x - b.x, a.y - b.y };
}

inline Vec2  operator*(Vec2 a, float s)
{
    return { a.x * s,   a.y * s };
}

inline Vec2& operator+=(Vec2& a, Vec2 b)
{
    a.x += b.x; a.y += b.y;
    return a;
}

inline float dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

inline float cross(Vec2 a, Vec2 b)
{
    return a.x * b.y - a.y * b.x;
}