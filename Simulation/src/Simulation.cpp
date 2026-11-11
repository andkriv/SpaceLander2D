#include <algorithm>

#include "Simulation/Simulation.h"

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
        rd.spec = rk.spec;

        float frac = rd.spec.fuelCapacity > 0 ? rd.spec.fuelStart / rd.spec.fuelCapacity : 0.0f;
        rd.state = Physics::makeState(rd.spec, { rk.pos.x / kPxPerM, rk.pos.y / kPxPerM }, rk.angle, frac);
        m_rockets.push_back(std::move(rd));
    }
    m_selRocket = m_rockets.empty() ? -1 : 0;
    m_acc = 0.0;
    // don't reset g
}

int Simulation::step(float frameDt, const Physics::Input& selectedInput)
{
    std::vector<Physics::RocketRef> rocketRefs(m_rockets.size());
    for (size_t i = 0; i < m_rockets.size(); ++i)
        rocketRefs[i] = { &m_rockets[i].spec, &m_rockets[i].state, {}, {} };

    // clamp so a stall doesn't cause spiral of death (sim takes more time then sim step), but lose time
    m_acc += std::min(frameDt, kMaxFrame);
    int n = 0;
    while (m_acc >= kStep)
    {
        for (size_t i = 0; i < rocketRefs.size(); ++i)
            rocketRefs[i].input = ((int)i == m_selRocket) ? selectedInput : Physics::Input{};

        Physics::stepAll(rocketRefs, m_world, kStep);

        for (size_t i = 0; i < rocketRefs.size(); ++i)
        {
            m_rockets[i].info = rocketRefs[i].info;
            if (rocketRefs[i].info.impactSpeed > kCrashSpeed)
                m_rockets[i].is_crashed = true;
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

const Physics::RocketSpec& Simulation::spec(int i) const
{
    static const Physics::RocketSpec none;
    return (i >= 0 && i < (int)m_rockets.size()) ? m_rockets[i].spec : none;
}

void Simulation::setEngineLevel(int lvl)
{
    if (m_selRocket >= 0)
        m_rockets[m_selRocket].level = std::clamp(lvl, 0, kEngineLevels - 1);
}

int  Simulation::engineLevel() const
{
    return m_selRocket >= 0 ? m_rockets[m_selRocket].level : kEngineLevels - 1;
}