#include "Simulation/Simulation.h"
#include <algorithm>

void Simulation::start(const Scene& s)
{
    m_scene = s; // for reset
    m_world.setTerrain(s.terrain, 1.0f / kPxPerM);
    reset();
}

void Simulation::reset()
{
    m_rockets.clear();
    for (const Rocket& rk : m_scene.rockets)
    {
        RocketData rd;
        rd.name = rk.name;
        rd.state = Physics::makeState(m_spec, { rk.pos.x / kPxPerM, rk.pos.y / kPxPerM }, rk.angle);
        m_rockets.push_back(std::move(rd));
    }
    m_selRocket = m_rockets.empty() ? -1 : 0;
    m_acc = 0.0;
    // don't reset g
}

int Simulation::step(float frameDt, const Physics::Input& selectedInput)
{
    // clamp so a stall doesn't cause spiral of death (sim takes more time then sim step), but lose time
    m_acc += std::min(frameDt, kMaxFrame);
    int n = 0;
    while (m_acc >= kStep)
    {
        for (int i = 0; i < (int)m_rockets.size(); ++i)
        {
            RocketData& rd = m_rockets[i];
            Physics::Input ri = (i == m_selRocket) ? selectedInput : Physics::Input{};
            rd.throttle = ri.throttle;
            rd.info = Physics::step(m_spec, rd.state, ri, m_world, kStep);
            if (rd.info.impactSpeed > kCrashSpeed)
                rd.is_crashed = true;
        }
        m_acc -= kStep;
        ++n;
    }
    return n;
}

const Simulation::RocketData& Simulation::selectedRocketData() const
{
    return m_rockets[m_selRocket];
}

int  Simulation::selectedRocket() const
{
    return m_selRocket;
}

void Simulation::selectRocket(int i)
{
    if (i >= 0 && i < (int)m_rockets.size())
        m_selRocket = i;
}

const std::vector<Simulation::RocketData>& Simulation::rockets() const
{
    return m_rockets;
}

Physics::World& Simulation::world()
{
    return m_world;
}

const Physics::World& Simulation::world() const
{
    return m_world;
}

const RocketSpec& Simulation::spec() const
{
    return m_spec;
}