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

    void setShowTrails(bool v);
    bool showTrails() const;

    void setShowPrediction(bool v);
    bool showPrediction() const;


private:
    Simulation m_sim;

    bool   m_paused = false;
    bool   m_stop = false;

    bool m_showTrails = true;
    bool m_showPrediction = true;
};