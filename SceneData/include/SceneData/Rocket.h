#pragma once
#include <string>

#include "Vec2.h"
#include "RocketSpec.h"

struct Rocket
{
    std::string name;

    Vec2  pos{};
    float angle = 0.0f; // radians, 0 = nose up, positive = screen's clockwise

    RocketSpec spec;
};