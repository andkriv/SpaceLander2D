#pragma once
#include <string>
#include <vector>

struct Vec2
{
    float x;
    float y;
};

struct Polyline
{
    std::string name;
    std::vector<Vec2> points;
};

struct Terrain
{
    std::vector<Polyline> polylines;
};