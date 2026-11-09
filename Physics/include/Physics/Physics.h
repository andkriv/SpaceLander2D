#pragma once
#include <array>
#include <vector>

#include "SceneData/Terrain.h"
#include "SceneData/RocketSpec.h"

namespace Physics
{

    constexpr float kG0 = 9.80665f;   // standard gravity, for Isp

    struct BodyState
    {
        Vec2  pos = {0.0f, 0.0f};    // center of mass, world
        Vec2  vel = {0.0f, 0.0f};
        float angle = 0.0f;          // [rad]
        float angVel = 0.0f;         // [rad/s]
        float fuel = 0.0f;           // [kg]
    };

    struct Input
    {
        float throttle = 0.0f;       // 0..1
        float rotate = 0.0f;         // -1..1, positive = clockwise
    };

    struct Segment
    {
        Vec2 a;
        Vec2 b;
    };

    struct World
    {
        Vec2  gravity{ 0.0f, 9.81f };// [m/s^2], +y is down
        float restitution = 0.1f;    // 0 = no bounce, 1 = perfectly elastic
        float friction = 0.7f;       // Coulomb friction coefficient

        std::vector<Segment> terrain;

        void setTerrain(const Terrain& t, float metersPerUnit);
    };

    struct MassProps
    {
        float mass = 0.0f;           // [kg]
        Vec2  com;                   // in hull local frame
        float inertia = 0.0f;        // [kg*m^2], about the COM
    };

    struct StepInfo
    {
        float mass = 0;              // [kg]
        float inertia = 0;           // [kg*m^2]
        float thrust = 0;            // [N]
        float fuelFlow = 0;          // [kg/s]

        Vec2  acc;                   // [m/s^2], measured: (vel_after - vel_before) / dt
        bool  touching = false;
        float impactSpeed = 0.0f;    // [m/s], largest closing speed on a contact this step
    };

    Vec2 rotate(Vec2 v, float angle);

    MassProps massProps(const RocketSpec& s, float fuel);

    // hull-local point -> world (accounts for the moving COM), render + sim sync
    Vec2 toWorld(const RocketSpec& s, const BodyState& b, Vec2 local);

    // state from the hull center (editor)
    BodyState makeState(const RocketSpec& s, Vec2 hullCenter, float angle, float fuelFraction = 1.0f);

    StepInfo step(const RocketSpec& s, BodyState& b, const Input& in, const World& w, float dt);

} // namespace Physics