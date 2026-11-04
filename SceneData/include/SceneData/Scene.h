#pragma once
#include <vector>

#include "Terrain.h"
#include "Rocket.h"

struct Scene
{
    Terrain             terrain;
    std::vector<Rocket> rockets;
};