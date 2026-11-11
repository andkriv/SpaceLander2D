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
        rd.trail.push_back(Physics::toWorld(rd.spec, rd.state, { 0.0f, 0.0f }));
        m_rockets.push_back(std::move(rd));
    }
    m_selRocket = m_rockets.empty() ? -1 : 0;
    m_acc = 0.0;
    m_trailAcc = 0.0f;
    // don't reset g
}

void Simulation::clearTrails()
{
    for (RocketData& rd : m_rockets)
    {
        rd.trail.clear();
        rd.trail.push_back(Physics::toWorld(rd.spec, rd.state, { 0.0f, 0.0f }));
    }
    m_trailAcc = 0.0f;
}

void Simulation::recordTrails()
{
    for (RocketData& rd : m_rockets)
    {
        Vec2 p = Physics::toWorld(rd.spec, rd.state, { 0.0f, 0.0f });

        if (!rd.trail.empty())
        {
            Vec2 d = p - rd.trail.back();
            if (d.x * d.x + d.y * d.y < kTrailMinDist * kTrailMinDist)
                continue;
        }
        rd.trail.push_back(p);

        if (rd.trail.size() > kTrailMax)
            rd.trail.erase(rd.trail.begin());
    }
}

std::vector<Vec2> Simulation::predict(int i, float seconds) const
{
    std::vector<Vec2> out;
    if (i < 0 || i >= (int)m_rockets.size())
        return out;

    const Physics::RocketSpec& spec = m_rockets[i].spec;
    Physics::BodyState st = m_rockets[i].state;

    out.push_back(Physics::toWorld(spec, st, { 0.0f, 0.0f }));

    const int steps = (int)(seconds / kStep);
    for (int k = 1; k <= steps; ++k)
    {
        Physics::StepInfo info = Physics::step(spec, st, Physics::Input{}, m_world, kStep);

        bool last = info.touching || k == steps;
        if (k % kPredictEvery == 0 || last)
            out.push_back(Physics::toWorld(spec, st, { 0.0f, 0.0f }));

        if (info.touching)
            break;
    }
    return out;
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

        m_trailAcc += kStep;
        if (m_trailAcc >= kTrailInterval)
        {
            m_trailAcc -= kTrailInterval;
            recordTrails();
        }
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