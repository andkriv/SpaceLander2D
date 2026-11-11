#pragma once
#include <string>
#include <vector>

#include "Physics/Physics.h"
#include "SceneData/Scene.h"

class Simulation
{
public:
    static constexpr float kPxPerM       = kPixelsPerMeter;// 1m = 10px; the editor hull (16/8/20 px) matches the spec
    static constexpr float kStep         = 1.0f / 240.0f;  // physics update rate, with kSkin - 0.02*240 = 4.8 m/s, g*dt=0.04 m/s
    static constexpr float kCrashSpeed   = 8.0f;           // [m/s] closing speed on contact
    // 20fps, clamp so a stall doesn't cause spiral of death
    static constexpr float kMaxFrame     = 0.05f;
    static constexpr int   kEngineLevels = 4;

    static float engineLevelFraction(int level)
    {   
        // 1..4 -> 25, 50, 75, 100 %
        static constexpr float f[kEngineLevels] = { 0.25f, 0.5f, 0.75f, 1.0f };
        return f[level < 0 ? 0 : (level >= kEngineLevels ? kEngineLevels - 1 : level)];
    }

    struct RocketData
    {
        std::string         name;
        Physics::RocketSpec spec;
        Physics::BodyState  state;
        Physics::StepInfo   info;
        bool                is_crashed = false;
        int                 level = kEngineLevels - 1; // idx 0..3
    };

    void start(const Scene& s);
    void reset();

    int  step(float frameDt, const Physics::Input& selectedInput); // returns steps run

    int  selectedRocket() const;
    void selectRocket(int i);

    const Physics::RocketSpec& spec(int i = 0) const;
    const std::vector<RocketData>& rockets() const;
    const RocketData& selectedRocketData() const;
    void setEngineLevel(int lvl); // selected only
    int  engineLevel() const;

    Physics::World& world();
    const Physics::World& world() const;

private:
    Scene                   m_scene;
    Physics::World          m_world;
    std::vector<RocketData> m_rockets;

    int    m_selRocket = -1;
    double m_acc = 0.0;
};