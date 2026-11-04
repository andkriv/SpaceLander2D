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

    std::array<Vec2, 4> hullLocal(const RocketSpec& s)
    {
        return { { { -s.noseHalfWidth, -s.halfHeight }, {  s.noseHalfWidth, -s.halfHeight },
                   {  s.baseHalfWidth,  s.halfHeight }, { -s.baseHalfWidth,  s.halfHeight } } };
    }

    MassProps massProps(const RocketSpec& s, float fuel)
    {
        fuel = std::clamp(fuel, 0.0f, s.fuelCapacity);
        float frac = s.fuelCapacity > 0 ? fuel / s.fuelCapacity : 0.0f;

        // dry body, box with the avg hull width
        float avgW = s.noseHalfWidth + s.baseHalfWidth;
        float H = 2.0f * s.halfHeight;
        // dry I = m*x^2 -> dI = dm*x^2, (rho = m/L -> dm = rho * dx) -> dI = x^2*rho*dx
        // int[-L/2, L/2](x^2*rho*dx) = rho*L^3/12 = m*L^2/12
        float Id = s.dryMass / 12.0f * (avgW * avgW + H * H);

        // Fuel: box filling the tank from the bottom of the hull upward
        float fuelH = s.tankHeight * frac;
        float fuelY = s.halfHeight - fuelH * 0.5f;
        // fuel I
        float If = fuel / 12.0f * (s.tankWidth * s.tankWidth + fuelH * fuelH);

        MassProps m;
        m.mass = s.dryMass + fuel;
        float comY = (s.dryMass * s.dryComY + fuel * fuelY) / m.mass;
        m.com = { 0.0f, comY };

        // dry, fuel COMs to whole COM
        float dd = s.dryComY - comY;
        float df = fuelY - comY;
        // parallel-axis theorem, I = Icom + M*d^2
        //     I = [Idry + Mdry*d_dry^2] + [Ifuel + Mfuel * d_fuel^2]
        // alpha[rad/s^2]=tau[N*m]/I[kg*m^2] of RCS rise as fuel burns
        m.inertia = Id + s.dryMass * dd * dd + If + fuel * df * df;  
        return m;
    }

    Vec2 toWorld(const RocketSpec& s, const BodyState& b, Vec2 local)
    {
        return b.pos + rotate(local - massProps(s, b.fuel).com, b.angle);
    }

    BodyState makeState(const RocketSpec& s, Vec2 center, float angle, float fuelFraction)
    {
        BodyState b;
        b.fuel = s.fuelCapacity * std::clamp(fuelFraction, 0.0f, 1.0f);
        b.angle = angle;
        b.pos = center + rotate(massProps(s, b.fuel).com, angle);   // hull center -> COM
        return b;
    }

} // namespace Physics