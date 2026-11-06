#include <cmath>

#include <imgui.h>

#include "SceneEditor/Editor.h"
#include "SceneIO/SceneIO.h"
#include "Physics/Physics.h"

namespace
{
    constexpr float kMoveSpeed = 200.0f;   // px/s
    constexpr float kTurnSpeed = 2.0f;     // rad/s

    // trapezoid
    constexpr Vec2 kHull[4] = { {-8, -20}, {8, -20}, {16, 20}, {-16, 20} };

    Vec2 toWorld(const Rocket& rk, const Vec2& l)
    {
        float c = std::cos(rk.angle);
        float s = std::sin(rk.angle);

        return { rk.pos.x + l.x * c - l.y * s,
                 rk.pos.y + l.x * s + l.y * c };
    }

    // TODO: physics test, remove later
    constexpr float kPxPerM = 10.0f;       // 1 m = 10 px; the editor hull (16/8/20 px) matches the spec
    constexpr float kStep = 1.0f / 240.0f; // physics update rate

    struct RocketPhysics
    {
        Physics::RocketSpec spec;
        Physics::BodyState body; // from the editor's Rocket { pos (px), angle }:
        Physics::StepInfo info;
    };

    std::vector<RocketPhysics> rkp;
    float acc = 0; // Time Accumulator
    Physics::World      world;
}

void Editor::controlRocket(float dt)
{
    if (m_selectedRocket < 0 || ImGui::GetIO().WantTextInput)
        return;

    Rocket& rk = m_rockets[m_selectedRocket];

    auto axis = [](ImGuiKey pos, ImGuiKey neg)
    {
        return (float)ImGui::IsKeyDown(pos) - (float)ImGui::IsKeyDown(neg);
    };

    //rk.pos.x += axis(ImGuiKey_D, ImGuiKey_A) * kMoveSpeed * dt;
    //rk.pos.y += axis(ImGuiKey_S, ImGuiKey_W) * kMoveSpeed * dt;
    //rk.angle += axis(ImGuiKey_E, ImGuiKey_Q) * kTurnSpeed * dt;

    for (int i = 0; i < (int)m_rockets.size(); i++)
    {
        acc += dt; //frame time accum
        while (acc >= kStep)
        {
            Physics::Input in;
            if (i == m_selectedRocket)
            {
                in.throttle = ImGui::IsKeyDown(ImGuiKey_W) ? 1.0f : 0.0f;
                in.rotate = (float)ImGui::IsKeyDown(ImGuiKey_E) - (float)ImGui::IsKeyDown(ImGuiKey_Q);
            }

            rkp[i].info = Physics::step(rkp[i].spec, rkp[i].body, in, world, kStep);
            acc -= kStep;
        }
    }
}

const Terrain& Editor::terrain() const
{
    return m_terrain;
}

void Editor::setScene(Scene s)
{
    m_terrain = std::move(s.terrain);
    m_rockets = std::move(s.rockets);

    m_current = {};
    m_drawing = false;
    m_selected = m_terrain.polylines.empty() ? -1 : 0;
    m_counter = (int)m_terrain.polylines.size();

    m_selectedRocket = m_rockets.empty() ? -1 : 0;
    m_rocketCounter = (int)m_rockets.size();

    rkp.clear();
    world.setTerrain(m_terrain, 1.0f / kPxPerM);
    for (const auto& rk : m_rockets)
    {
        RocketPhysics rp;
        rp.body = Physics::makeState(rp.spec, { rk.pos.x / kPxPerM, rk.pos.y / kPxPerM }, rk.angle);
        rkp.push_back(std::move(rp));
    }
}

void Editor::pollDialog(Renderer& r)
{
    if (m_pending == Pending::None)
        return;

    char buf[1024] = {};
    DialogStatus s = r.pollFileDialog(buf, sizeof buf);
    if (s == DialogStatus::None)
        return;  // still open

    if (s == DialogStatus::Picked)
    {
        std::string path = buf, err;
        if (m_pending == Pending::Export)
        {
            if (path.size() < 5 || path.compare(path.size() - 5, 5, ".json") != 0)
                path += ".json";

            Scene s{ m_terrain, m_rockets };
            m_status = SceneIO::save(s, path, &err) ? "Exported: " + path
                                                    : "Export failed: " + err;
        }
        else
        {
            Scene s;
            if (SceneIO::load(s, path, &err))
            {
                setScene(std::move(s));
                m_status = "Imported: " + path;
            }
            else
            {
                m_status = "Import failed: " + err;
            }
        }
    }
    m_pending = Pending::None;
}

void Editor::update(Renderer& r)
{
    pollDialog(r);
    controlRocket(ImGui::GetIO().DeltaTime);

    const InputState& in = r.input();

    if (m_drawing && in.leftPressed && !ImGui::GetIO().WantCaptureMouse)
        m_current.points.push_back({ in.mouseX, in.mouseY });

    ImGui::Begin("Terrain Editor");

    if (!m_drawing)
    {
        if (ImGui::Button("Start drawing"))
        {
            m_drawing = true;
            m_current = {};
        }
    }
    else
    {
        ImGui::BeginDisabled(m_current.points.size() < 2);
        if (ImGui::Button("Finish"))
        {
            m_current.name = "Polyline " + std::to_string(++m_counter);
            m_terrain.polylines.push_back(std::move(m_current));
            world.setTerrain(m_terrain, 1.0f / kPxPerM);

            m_selected = (int)m_terrain.polylines.size() - 1;
            m_current = {};
            m_drawing = false;
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::BeginDisabled(m_current.points.empty());
        if (ImGui::Button("Undo point"))
            m_current.points.pop_back();

        ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            m_current = {};
            m_drawing = false;
        }

        ImGui::TextDisabled("Left-click to add points (%d)", (int)m_current.points.size());
    }

    ImGui::Separator();

    // Dropdown polylines selector
    const char* preview = m_selected >= 0 ? m_terrain.polylines[m_selected].name.c_str()
                                          : "(none)";

    if (ImGui::BeginCombo("Polylines", preview))
    {
        for (int i = 0; i < (int)m_terrain.polylines.size(); ++i)
        {
            bool sel = (i == m_selected);
            if (ImGui::Selectable(m_terrain.polylines[i].name.c_str(), sel))
                m_selected = i;

            if (sel)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    // Remove selected polyline
    ImGui::BeginDisabled(m_selected < 0);
    if (ImGui::Button("Remove selected"))
    {
        m_terrain.polylines.erase(m_terrain.polylines.begin() + m_selected);
        m_selected = -1;
        world.setTerrain(m_terrain, 1.0f / kPxPerM);
    }
    ImGui::EndDisabled();

    // rocket part
    ImGui::Separator();
    if (ImGui::Button("Add rocket"))
    {
        Rocket rk;
        rk.name = "Rocket " + std::to_string(++m_rocketCounter);
        rk.pos = { 400.0f + 60.0f * (m_rocketCounter - 1), 200.0f };

        RocketPhysics rp;
        rp.body = Physics::makeState(rp.spec, { rk.pos.x / kPxPerM, rk.pos.y / kPxPerM }, rk.angle);

        rkp.push_back(std::move(rp));
        m_rockets.push_back(std::move(rk));
        m_selectedRocket = (int)m_rockets.size() - 1;
    }

    const char* rpreview = m_selectedRocket >= 0 ? m_rockets[m_selectedRocket].name.c_str()
                                                 : "(none)";

    if (ImGui::BeginCombo("Rockets", rpreview))
    {
        for (int i = 0; i < (int)m_rockets.size(); ++i)
        {
            bool sel = (i == m_selectedRocket);
            if (ImGui::Selectable(m_rockets[i].name.c_str(), sel))
                m_selectedRocket = i;

            if (sel)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::BeginDisabled(m_selectedRocket < 0);
    if (ImGui::Button("Remove selected##rocket"))
    {
        rkp.erase(rkp.begin() + m_selectedRocket);

        m_rockets.erase(m_rockets.begin() + m_selectedRocket);
        m_selectedRocket = -1;
    }
    ImGui::EndDisabled();
    if (m_selectedRocket >= 0)
        ImGui::TextDisabled("WASD move, Q/E rotate");

    // io part
    ImGui::Separator();
    ImGui::BeginDisabled(m_pending != Pending::None);
    if (ImGui::Button("Export..."))
    {
        m_pending = Pending::Export;
        r.showFileDialog(DialogType::Save);
    }

    ImGui::SameLine();
    if (ImGui::Button("Import..."))
    {
        m_pending = Pending::Import;
        r.showFileDialog(DialogType::Open);
    }
    ImGui::EndDisabled();
    if (!m_status.empty())
        ImGui::TextWrapped("%s", m_status.c_str());

    // TODO: rocket physics data, remove later
    const ImGuiViewport* viewport = ImGui::GetMainViewport();

    ImVec2 window_pos;
    window_pos.x = viewport->WorkPos.x + viewport->WorkSize.x - 10.0f;
    window_pos.y = viewport->WorkPos.y + 10.0f;

    ImGui::SetNextWindowPos(window_pos, ImGuiCond_Always, ImVec2(1.0f, 0.0f));

    ImGui::Begin("Rockets Info");
    for (int i = 0; i < (int)m_rockets.size(); i++)
    {
        ImGui::Text("%s telemetry = Vv: %.2f, Vh: %.2f, Av:  %.2f, Ah: %.2f, Mb = %.2f, Mf: %.2f",
            m_rockets[i].name.c_str(),
            -rkp[i].body.vel.y,
            rkp[i].body.vel.x,
            rkp[i].info.acc.y,
            rkp[i].info.acc.x,
            rkp[i].info.mass,
            rkp[i].body.fuel);
    }
    ImGui::End();

    // ImGui::ShowMetricsWindow();
    ImGui::End();
}

static void drawMarker(Renderer& r, const Vec2& p)
{
    r.drawLine(p.x - 4, p.y, p.x + 4, p.y);
    r.drawLine(p.x, p.y - 4, p.x, p.y + 4);
}

void Editor::draw(Renderer& r) const
{
    // Finished polylines (selected one highlighted)
    for (int i = 0; i < (int)m_terrain.polylines.size(); ++i)
    {
        const auto& pts = m_terrain.polylines[i].points;
        if (i == m_selected)
            r.setColor(255, 200, 60); // light-orange color
        else
            r.setColor(220, 220, 220); // light-gray color

        for (size_t k = 1; k < pts.size(); ++k)
            r.drawLine(pts[k - 1].x, pts[k - 1].y, pts[k].x, pts[k].y);
    }

    // Polyline draw, preview segment to the cursor
    if (m_drawing)
    {
        r.setColor(90, 200, 255); // light-blue color

        const auto& pts = m_current.points;
        for (size_t k = 1; k < pts.size(); ++k)
            r.drawLine(pts[k - 1].x, pts[k - 1].y, pts[k].x, pts[k].y);

        for (const auto& p : pts)
            drawMarker(r, p);

        if (!pts.empty())
        {
            r.setColor(90, 200, 255, 120); // light-blue color, transp

            const InputState& in = r.input();
            r.drawLine(pts.back().x, pts.back().y, in.mouseX, in.mouseY);
        }
    }

    // Rockets (selected one highlighted)
    for (int i = 0; i < (int)m_rockets.size(); ++i)
    {
        const Rocket& rk = m_rockets[i];
        if (i == m_selectedRocket)
            r.setColor(120, 255, 140); // light-green color
        else
            r.setColor(200, 200, 200); // light-gray color

        // convert rocket body point with rocket's pos, angle
        for (int k = 0; k < 4; ++k)
        {
            Vec2 a = Physics::toWorld(rkp[i].spec, rkp[i].body, hullLocal(rkp[i].spec)[k]) * kPxPerM;
            Vec2 b = Physics::toWorld(rkp[i].spec, rkp[i].body, hullLocal(rkp[i].spec)[(k + 1) % 4]) * kPxPerM;
            r.drawLine(a.x, a.y, b.x, b.y);
        }
    }
}