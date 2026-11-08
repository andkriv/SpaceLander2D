#pragma once
#include <string>
#include <vector>

#include "Simulation/Simulation.h"
#include "Renderer/Renderer.h"

class Execution
{
public:
    void start(const Scene& s);
    void update(Renderer& r);
    void draw(Renderer& r) const;
    bool wantsStop() const;

private:
    void reset();

private:
    Simulation m_sim;

    bool   m_paused = false;
    bool   m_stop = false;
};