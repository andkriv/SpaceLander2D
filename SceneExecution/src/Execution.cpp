#include "SceneExecution/Execution.h"
#include <imgui.h>
#include <algorithm>
#include <cmath>

namespace
{
    void drawLine(IDrawTarget& r, Vec2 a, Vec2 b)
    {
        // physics world to sceen
        r.drawLine(a.x * kPixelsPerMeter, a.y * kPixelsPerMeter, b.x * kPixelsPerMeter, b.y * kPixelsPerMeter);
    }
}

Simulation& Execution::sim()
{
    return m_sim;
}

bool Execution::paused() const
{
    return m_paused;
}

bool Execution::wantsStop() const
{
    return m_stop;
}

void Execution::start(const Scene& s)
{
    m_stop = false;
    m_paused = false;
    m_sim.start(s);
}

void Execution::update(IDrawTarget&)
{
    ImGuiIO& io = ImGui::GetIO();

    Physics::Input in;
    if (!io.WantTextInput)
    {
        for (int k = 0; k < Simulation::kEngineLevels; ++k)
            if (ImGui::IsKeyPressed((ImGuiKey)(ImGuiKey_1 + k), false))
                m_sim.setEngineLevel(k);

        float f = Simulation::engineLevelFraction(m_sim.engineLevel());
        in.throttle = ImGui::IsKeyDown(ImGuiKey_W) ? f : 0.0f;
        in.rotate = ((float)ImGui::IsKeyDown(ImGuiKey_E) - (float)ImGui::IsKeyDown(ImGuiKey_Q)) * f;
    }

    // fix-step sim
    if (!m_paused)
    {
        m_sim.step(io.DeltaTime, in);
    }

    // rocket ui
    ImGui::Begin("Simulation");
    if (ImGui::Button("Stop"))
        m_stop = true;

    ImGui::SameLine();
    if (ImGui::Button("Reset"))
        m_sim.reset();

    ImGui::SameLine();
    ImGui::Checkbox("Pause", &m_paused);

    ImGui::Checkbox("Trails", &m_showTrails);
    ImGui::SameLine();
    ImGui::Checkbox("Prediction", &m_showPrediction);
    ImGui::SameLine();
    if (ImGui::Button("Clear trails"))
        m_sim.clearTrails();

    if (io.WantTextInput)
        ImGui::TextColored(ImVec4(1, 0.6f, 0.2f, 1), "text field has the keyboard: click empty space");


    ImGui::SliderFloat("G [m/s^2]", &m_sim.world().gravity.y, 0.0f, 25.0f);

    const auto& rockets = m_sim.rockets();
    const int& selR = m_sim.selectedRocket();
    const char* prev = selR >= 0 ? rockets[selR].name.c_str()
                                 : "(none)";
    if (ImGui::BeginCombo("Rocket", prev))
    {
        for (int i = 0; i < (int)rockets.size(); ++i)
            if (ImGui::Selectable(rockets[i].name.c_str(), i == selR))
                m_sim.selectRocket(i);
        ImGui::EndCombo();
    }

    if (selR >= 0)
    {
        const Simulation::RocketData& rd = m_sim.selectedRocketData();
        ImGui::Separator();
        ImGui::TextDisabled("W thrust, Q/E rotate, 1-4 thrust");
        ImGui::Text(        "Mass    %.1f kg", rd.info.mass);
        ImGui::Text(        "Fuel    %.1f / %.0f kg", rd.state.fuel, m_sim.spec().fuelCapacity);
        ImGui::ProgressBar(rd.spec.fuelCapacity > 0 ? rd.state.fuel / rd.spec.fuelCapacity : 0.0f, ImVec2(-1, 0));
        ImGui::Text(        "Level   %d/4 (%.0f%%)", rd.level + 1, Simulation::engineLevelFraction(rd.level) * 100.0f);
        ImGui::Text(        "Main    %.0f N", rd.info.thrust);
        ImGui::Text(        "Rot     %.0f N x2 (%s)", rd.info.rcsForce, rd.info.rcsDir > 0 ? "CW" : rd.info.rcsDir < 0 ? "CCW" : "off");
        ImGui::Text(        "Vh      %.2f m/s", rd.state.vel.x);
        ImGui::Text(        "Vv      %.2f m/s (up+)", -rd.state.vel.y);
        ImGui::Text(        "Acc     %.2f, %.2f m/s^2", rd.info.acc.x, -rd.info.acc.y);
        ImGui::Text(        "Angle   %.1f deg", rd.state.angle * 57.29578f);
        ImGui::Text(        "Status  %s", rd.is_crashed ? "CRASHED"
                                                        : rd.info.touching ? "on ground" 
                                                                           : "flying");
    }
    ImGui::End();
}

void Execution::draw(IDrawTarget& r) const
{
    // terrain
    r.setColor(220, 220, 220); // light-gray color
    for (const auto& sg : m_sim.world().terrain)
        drawLine(r, sg.a, sg.b);

    // rockets
    const auto& rockets = m_sim.rockets();

    // trail
    if (m_showTrails)
    {
        r.setColor(90, 160, 255, 180); // blue color
        for (const auto& rd : rockets)
            for (size_t k = 1; k < rd.trail.size(); ++k)
                drawLine(r, rd.trail[k - 1], rd.trail[k]);
    }

    // prediction, coasting path, dashed
    if (m_showPrediction && m_sim.selectedRocket() >= 0)
    {
        auto path = m_sim.predict(m_sim.selectedRocket());
        r.setColor(255, 230, 120, 150); // light-yellow color
        for (size_t k = 1; k < path.size(); k += 2)
            drawLine(r, path[k - 1], path[k]);
    }

    for (unsigned i = 0; i < (unsigned)rockets.size(); ++i)
    {
        const Simulation::RocketData& rd = rockets[i];
        if (rd.is_crashed)
            r.setColor(255, 90, 90); // light-red color
        else
            if (i == m_sim.selectedRocket())
                r.setColor(120, 255, 140); // light-green color
        else
                r.setColor(200, 200, 200); // light-gray color

        auto hull = hullLocal(rd.spec); // same as in editor mode
        for (int k = 0; k < 4; ++k)
        {
            drawLine(r, Physics::toWorld(rd.spec, rd.state, hull[k]), Physics::toWorld(rd.spec, rd.state, hull[(k + 1) % 4]));
        }

        // engines plumes, length = force * kPlumeMetersPerNewton, same scale
        auto plume = [&](const Physics::Nozzle& nz, float newtons)
        {
            if (newtons <= 0.0f)
                return;

            Vec2 base = Physics::toWorld(rd.spec, rd.state, nz.pos);
            Vec2 dir = Physics::rotate(nz.exhaust, rd.state.angle);

            drawLine(r, base, base + dir * Physics::plumeLength(newtons));
        };
        r.setColor(255, 170, 40); // light-orange color
        plume(Physics::mainNozzle(rd.spec), rd.info.thrust);
        r.setColor(140, 220, 255); // light-blue color

        if (rd.info.rcsDir != 0)
            for (const auto& nz : Physics::rcsNozzles(rd.spec, rd.info.rcsDir))
                plume(nz, rd.info.rcsForce);
    }
}

void Execution::setShowTrails(bool v)
{
    m_showTrails = v;
}

bool Execution::showTrails() const
{
    return m_showTrails;
}

void Execution::setShowPrediction(bool v)
{
    m_showPrediction = v;
}

bool Execution::showPrediction() const
{
    return m_showPrediction;
}