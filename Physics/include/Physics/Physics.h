#pragma once
#include <array>
#include <vector>

#include "SceneData/Terrain.h"
#include "SceneData/Vec2.h"

namespace Physics
{

    constexpr float kG0 = 9.80665f;   // standard gravity, for Isp

    struct RocketSpec                // SI units
    {                
        float dryMass = 800.0f;      // [kg]
        float fuelCapacity = 700.0f; // [kg]
        float maxThrust = 25000.0f;  // [N]
        float isp = 250.0f;          // [s], specific impulse
        float maxTorque = 6000.0f;   // [N*m], rotation control authority

        // Hull trapezoid, local frame centered on the hull, nose toward -y
        // half for collision OOB in SAT, fuel tank bottom
        float noseHalfWidth = 0.8f;  // [m]
        float baseHalfWidth = 1.6f;  // [m]
        float halfHeight = 2.0f;     // [m]

        // fuel tank params
        float dryComY = 0.0f;        // [m], dry-mass COM along the axis (+y = toward engine)
        float tankWidth = 1.4f;      // [m], fuel tank sits at the bottom of the hull
        float tankHeight = 3.0f;     // [m]
    };

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

} // namespace Physics