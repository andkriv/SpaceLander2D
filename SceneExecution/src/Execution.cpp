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
    m_sim.start(s);
    m_stop = false;
    m_paused = false;
}

void Execution::reset()
{
    m_sim.reset();
}

void Execution::update(IDrawTarget&)
{
    ImGuiIO& io = ImGui::GetIO();

    Physics::Input in;
    if (!io.WantTextInput)
    {
        in.throttle = ImGui::IsKeyDown(ImGuiKey_W) ? 1.0f : 0.0f;
        in.rotate   = (float)ImGui::IsKeyDown(ImGuiKey_E) - (float)ImGui::IsKeyDown(ImGuiKey_Q);
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
        reset();

    ImGui::SameLine();
    ImGui::Checkbox("Pause", &m_paused);

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
        ImGui::TextDisabled("W thrust, Q/E rotate");
        ImGui::Text(        "Mass    %.1f kg", rd.info.mass);
        ImGui::Text(        "Fuel    %.1f / %.0f kg", rd.state.fuel, m_sim.spec().fuelCapacity);
        ImGui::ProgressBar(rd.spec.fuelCapacity > 0 ? rd.state.fuel / rd.spec.fuelCapacity : 0.0f, ImVec2(-1, 0));
        ImGui::Text(        "Thrust  %.0f N", rd.info.thrust);
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
            drawLine(r, Physics::toWorld(m_sim.spec(), rd.state, hull[k]), Physics::toWorld(m_sim.spec(), rd.state, hull[(k + 1) % 4]));
        }

        // flame, from the engine
        if (rd.info.thrust > 0.0f)
        {
            Vec2 base = Physics::toWorld(m_sim.spec(), rd.state, { 0.0f, m_sim.spec().halfHeight });
            Vec2 back{ -std::sin(rd.state.angle), std::cos(rd.state.angle) }; // sin, -cos - nose
            float len = 3.0f * rd.info.thrust / m_sim.spec().maxThrust; /// 3m scaled by thrust
            r.setColor(255, 170, 40); // light-orange color
            drawLine(r, base, base + back * len);
        }
    }
}