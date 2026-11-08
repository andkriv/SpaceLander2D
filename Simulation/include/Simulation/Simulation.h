#pragma once
#include <string>
#include <vector>

#include "Physics/Physics.h"
#include "SceneData/Scene.h"

class Simulation
{
public:
    static constexpr float kPxPerM     = 10.0f;         // 1m = 10px; the editor hull (16/8/20 px) matches the spec
    static constexpr float kStep       = 1.0f / 240.0f; // physics update rate, with kSkin - 0.02*240 = 4.8 m/s, g*dt=0.04 m/s
    static constexpr float kCrashSpeed = 8.0f;          // [m/s] closing speed on contact
    // 20fps, clamp so a stall doesn't cause spiral of death
    static constexpr float kMaxFrame = 0.05f;

    struct RocketData
    {
        std::string        name;
        Physics::BodyState state;
        Physics::StepInfo  info;
        float              throttle = 0.0f;
        bool               is_crashed = false;
    };

    void start(const Scene& s);
    void reset();

    int  step(float frameDt, const Physics::Input& selectedInput); // returns steps run

    int  selectedRocket() const;
    void selectRocket(int i);

    const Physics::RocketSpec& spec() const;
    const std::vector<RocketData>& rockets() const;
    const RocketData& selectedRocketData() const;

    Physics::World& world();
    const Physics::World& world() const;

private:
    Scene                   m_scene;
    Physics::RocketSpec     m_spec; // all same for now
    Physics::World          m_world;
    std::vector<RocketData> m_rockets;

    int    m_selRocket = -1;
    double m_acc = 0.0;
};