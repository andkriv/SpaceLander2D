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

    // ---------- step ----------
    StepInfo step(const RocketSpec& s, BodyState& b, const Input& in, const World& w, float dt)
    {
        StepInfo info;
        const Vec2 v0 = b.vel;
        MassProps mp = massProps(s, b.fuel);

        // 1. thrust and fuel
        float thrust = std::clamp(in.throttle, 0.0f, 1.0f) * s.maxThrust;
        // m[kg/s] = Ft[N] / (Isp[s] * g0[m/s^2])
        float burn = thrust / (s.isp * kG0) * dt;
        if (b.fuel <= 0.0f)
        {
            thrust = 0.0f;
            burn = 0.0f;
        }
        else if (burn > b.fuel) // account for fuel starvation
        {
            thrust *= b.fuel / burn;
            burn = b.fuel;
        }
        b.fuel = std::max(0.0f, b.fuel - burn);

        // 2. torque
        // u vector nose direction
        Vec2 nose{ std::sin(b.angle), -std::cos(b.angle) };
        Vec2 F = nose * thrust;

        Vec2 enginePos{ 0.0f, s.halfHeight };
        Vec2 engineArm = rotate(enginePos - mp.com, b.angle);
        // tau = r x F, and include RCS torque
        float torque = cross(engineArm, F) + std::clamp(in.rotate, -1.0f, 1.0f) * s.maxTorque;

        // 3. forces -> velocities (mass at the start of the step)
        Vec2 acc = F * (1.0f / mp.mass) + w.gravity;
        b.vel += acc * dt;
        b.angVel += torque / mp.inertia * dt;

        info.mass = mp.mass;
        info.inertia = mp.inertia;
        info.thrust = thrust;
        info.fuelFlow = burn / dt;

        mp = massProps(s, b.fuel); // upd after the burn

        info.acc = (b.vel - v0) * (1.0f / dt);
        return info;
    }

} // namespace Physics