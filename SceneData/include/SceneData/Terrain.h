#pragma once
#include <string>
#include <vector>

#include "Vec2.h"

struct Polyline
{
    std::string name;
    std::vector<Vec2> points;
};

struct Terrain
{
    std::vector<Polyline> polylines;
};