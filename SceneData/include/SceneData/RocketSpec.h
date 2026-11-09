#pragma once
#include <array>
#include <algorithm>

#include "Vec2.h"

constexpr float kPixelsPerMeter = 10.0f;   // editor pixels <-> physics meters

struct RocketSpec                // SI units
{
    // mass and fuel
    float dryMass      = 800.0f; // [kg]
    float fuelCapacity = 700.0f; // [kg]
    float fuelStart    = 700.0f; // [kg] at beginig

    // main engine
    float maxThrust  = 25000.0f; // [N]
    float isp          = 250.0f; // [s], specific impulse
    float maxTorque   = 6000.0f; // [N*m], rotation control authority

    // rot engines, nozzles +-rcsArm from hull center
    float rcsThrust   = 3000.0f; // [N] per nozzle
    float rcsArm         = 1.8f; // [m]

    // hull trapezoid, local frame centered on the hull, nose toward -y
    // half for collision OOB in SAT, fuel tank bottom
    float noseHalfWidth  = 0.8f; // [m]
    float baseHalfWidth  = 1.6f; // [m]
    float halfHeight     = 2.0f; // [m]

    // fuel tank params
    float dryComY        = 0.0f; // [m], dry-mass COM along the axis (+y = toward engine)
    float tankWidth      = 1.4f; // [m], fuel tank sits at the bottom of the hull
    float tankHeight     = 3.0f; // [m]
};

inline std::array<Vec2, 4> hullLocal(const RocketSpec& s)
{
    return {{ { -s.noseHalfWidth, -s.halfHeight }, {  s.noseHalfWidth, -s.halfHeight },
              {  s.baseHalfWidth,  s.halfHeight }, { -s.baseHalfWidth,  s.halfHeight } }};
}

// clamp fields
inline void sanitize(RocketSpec& s)
{
    s.dryMass       = std::clamp(s.dryMass,      1.0f, 1e7f);       // [kg]
    s.fuelCapacity  = std::clamp(s.fuelCapacity, 0.0f, 1e7f);       // [kg]
    s.fuelStart     = std::clamp(s.fuelStart, 0.0f, s.fuelCapacity);// [kg]

    s.maxThrust     = std::clamp(s.maxThrust,     0.0f, 1e8f);      // [N]
    s.isp           = std::clamp(s.isp,           1.0f, 1e5f);      // [s]

    s.rcsThrust     = std::clamp(s.rcsThrust,     0.0f, 1e7f);      // [N]
    s.rcsArm        = std::clamp(s.rcsArm, 0.0f, s.halfHeight);     // [m]

    s.noseHalfWidth = std::clamp(s.noseHalfWidth, 0.1f, 20.0f);     // [m]
    s.baseHalfWidth = std::clamp(s.baseHalfWidth, 0.1f, 20.0f);     // [m]
    s.halfHeight    = std::clamp(s.halfHeight,    0.5f, 20.0f);     // [m]

    s.dryComY       = std::clamp(s.dryComY, -s.halfHeight, s.halfHeight);
    s.tankWidth     = std::clamp(s.tankWidth,  0.0f, 2.0f * std::max(s.noseHalfWidth, s.baseHalfWidth));
    s.tankHeight    = std::clamp(s.tankHeight, 0.0f, 2.0f * s.halfHeight);
}