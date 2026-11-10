#pragma once
#include <string>
#include <vector>

#include "Simulation/Simulation.h"
#include "Renderer/Renderer.h"

class Execution
{
public:
    void start(const Scene& s);
    void update(IDrawTarget& r);
    void draw(IDrawTarget& r) const;
    bool wantsStop() const;

    Simulation& sim();
    bool paused() const;

private:
    Simulation m_sim;

    bool   m_paused = false;
    bool   m_stop = false;
};