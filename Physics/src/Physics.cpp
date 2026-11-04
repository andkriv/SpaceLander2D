#include <algorithm>
#include <cmath>

#include "Physics/Physics.h"

namespace Physics
{
    // ---------- World ----------
    void World::setTerrain(const Terrain& t, float k)
    {
        terrain.clear();
        for (const auto& pl : t.polylines)
        { 
            for (size_t i = 1; i < pl.points.size(); ++i)
            {
                terrain.push_back({ { pl.points[i - 1].x * k, pl.points[i - 1].y * k },
                                    { pl.points[i].x * k,     pl.points[i].y * k     } });
            }
        }
    }

    // ---------- Geometry / mass ----------
    Vec2 rotate(Vec2 v, float a)
    {
        float c = std::cos(a);
        float s = std::sin(a);

        return { v.x * c - v.y * s, v.x * s + v.y * c };
    }

} // namespace Physics