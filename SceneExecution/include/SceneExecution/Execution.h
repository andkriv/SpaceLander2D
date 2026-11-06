#pragma once
#include <Physics/Physics.h>
#include <Renderer/Renderer.h>
#include <SceneData/Scene.h>
#include <string>
#include <vector>

class Execution
{
public:
    void start(const Scene& s);
    void update(Renderer& r);
    void draw(Renderer& r) const;
    bool wantsStop() const;

private:
    struct RocketData
    {
        std::string        name;
        Physics::BodyState state;
        Physics::StepInfo  info;
        float              throttle = 0.0f;
        bool               is_crashed = false;
    };

    void reset();

    Scene                   m_scene;
    Physics::RocketSpec     m_spec; // all same for now
    Physics::World          m_world;
    std::vector<RocketData> m_rockets;

    int    m_selRocket = -1;
    double m_acc       = 0.0;
    bool   m_paused    = false;
    bool   m_stop      = false;
};