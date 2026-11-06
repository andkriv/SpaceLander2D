#include "SceneExecution/Execution.h"
#include <imgui.h>
#include <algorithm>
#include <cmath>

namespace
{
    constexpr float kPxPerM     = 10.0f;         // 1m = 10px; the editor hull (16/8/20 px) matches the spec
    constexpr float kStep       = 1.0f / 240.0f; // physics update rate, with kSkin - 0.02*240 = 4.8 m/s, g*dt=0.04 m/s
    constexpr float kCrashSpeed = 8.0f;          // [m/s] closing speed on contact

    void drawLine(Renderer& r, Vec2 a, Vec2 b)
    {
        // physics world to sceen
        r.drawLine(a.x * kPxPerM, a.y * kPxPerM, b.x * kPxPerM, b.y * kPxPerM);
    }
}

bool Execution::wantsStop() const
{
    return m_stop;
}

void Execution::start(const Scene& s)
{
    m_scene = s; // for reset
    m_world.setTerrain(s.terrain, 1.0f / kPxPerM);
    m_stop = false;
    reset();
}

void Execution::reset()
{
    m_rockets.clear();
    for (const Rocket& rk : m_scene.rockets)
    {
        RocketData rd;
        rd.name  = rk.name;
        rd.state = Physics::makeState(m_spec, { rk.pos.x / kPxPerM, rk.pos.y / kPxPerM }, rk.angle);
        m_rockets.push_back(std::move(rd));
    }
    m_selRocket = m_rockets.empty() ? -1 : 0;
    m_acc = 0.0;
    // don't reset g
}

void Execution::update(Renderer&)
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
    { // 20fps, clamp so a stall doesn't cause spiral of death (sim takes more time then sim step), but lose time
        m_acc += std::min(io.DeltaTime, 0.05f);
        while (m_acc >= kStep)
        {
            for (int i = 0; i < (int)m_rockets.size(); ++i)
            {
                RocketData& rd = m_rockets[i];
                Physics::Input ri = (i == m_selRocket) ? in : Physics::Input{};
                rd.throttle = ri.throttle;
                rd.info = Physics::step(m_spec, rd.state, ri, m_world, kStep);
                if (rd.info.impactSpeed > kCrashSpeed)
                    rd.is_crashed = true;
            }
            m_acc -= kStep;
        }
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

    ImGui::SliderFloat("G [m/s^2]", &m_world.gravity.y, 0.0f, 25.0f);

    const char* prev = m_selRocket >= 0 ? m_rockets[m_selRocket].name.c_str()
                                      : "(none)";
    if (ImGui::BeginCombo("Rocket", prev))
    {
        for (int i = 0; i < (int)m_rockets.size(); ++i)
            if (ImGui::Selectable(m_rockets[i].name.c_str(), i == m_selRocket))
                m_selRocket = i;
        ImGui::EndCombo();
    }

    if (m_selRocket >= 0)
    {
        const RocketData& rd = m_rockets[m_selRocket];
        ImGui::Separator();
        ImGui::TextDisabled("W thrust, Q/E rotate");
        ImGui::Text(        "Mass    %.1f kg", rd.info.mass);
        ImGui::Text(        "Fuel    %.1f / %.0f kg", rd.state.fuel, m_spec.fuelCapacity);
        ImGui::ProgressBar(rd.state.fuel / m_spec.fuelCapacity, ImVec2(-1, 0));
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

void Execution::draw(Renderer& r) const
{
    // terrain
    r.setColor(220, 220, 220); // light-gray color
    for (const auto& sg : m_world.terrain)
        drawLine(r, sg.a, sg.b);

    // rockets
    auto hull = Physics::hullLocal(m_spec);
    for (unsigned i = 0; i < (unsigned)m_rockets.size(); ++i)
    {
        const RocketData& rd = m_rockets[i];
        if (rd.is_crashed)
            r.setColor(255, 90, 90); // light-red color
        else
            if (i == m_selRocket)
                r.setColor(120, 255, 140); // light-green color
        else
                r.setColor(200, 200, 200); // light-gray color

        for (int k = 0; k < 4; ++k)
        {
            drawLine(r, Physics::toWorld(m_spec, rd.state, hull[k]), Physics::toWorld(m_spec, rd.state, hull[(k + 1) % 4]));
        }

        // flame, from the engine
        if (rd.info.thrust > 0.0f)
        {
            Vec2 base = Physics::toWorld(m_spec, rd.state, { 0.0f, m_spec.halfHeight });
            Vec2 back{ -std::sin(rd.state.angle), std::cos(rd.state.angle) }; // sin, -cos - nose
            float len = 3.0f * rd.info.thrust / m_spec.maxThrust; /// 3m scaled by thrust
            r.setColor(255, 170, 40); // light-orange color
            drawLine(r, base, base + back * len);
        }
    }
}