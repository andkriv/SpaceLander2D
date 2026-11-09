#include <cmath>

#include <imgui.h>

#include "SceneEditor/Editor.h"
#include "SceneIO/SceneIO.h"

namespace
{
    constexpr float kMoveSpeed = 200.0f;   // px/s
    constexpr float kTurnSpeed = 2.0f;     // rad/s

    Vec2 toPixels(const Rocket& rk, Vec2 localMeters)
    {
        Vec2 l = localMeters * kPixelsPerMeter;
        float c = std::cos(rk.angle);
        float s = std::sin(rk.angle);

        return { rk.pos.x + l.x * c - l.y * s,
                 rk.pos.y + l.x * s + l.y * c };
    }
}

void Editor::controlRocket(float dt)
{
    if (m_model.selectedRocket() < 0 || ImGui::GetIO().WantTextInput)
        return;

    auto axis = [](ImGuiKey pos, ImGuiKey neg)
    {
        return (float)ImGui::IsKeyDown(pos) - (float)ImGui::IsKeyDown(neg);
    };

    Vec2 dS{ axis(ImGuiKey_D, ImGuiKey_A) * kMoveSpeed * dt, axis(ImGuiKey_S, ImGuiKey_W) * kMoveSpeed * dt };
    float dA { axis(ImGuiKey_E, ImGuiKey_Q) * kTurnSpeed * dt };
    m_model.moveSelectedRocket(dS, dA);
}

const Terrain& Editor::terrain() const
{
    return m_model.terrain();
}

Scene Editor::scene() const
{
    return m_model.scene();
}

void Editor::setScene(Scene s)
{
    m_model.setScene(s);
}

EditorModel& Editor::model()
{
    return m_model;
}

const EditorModel& Editor::model() const
{
    return m_model;
}

void Editor::update(IDrawTarget& r, IFileDialog& dialog)
{
    Scene imported;
    if (m_files.poll(dialog, m_model.scene(), imported) == SceneFileFlow::Result::Imported)
        m_model.setScene(std::move(imported));

    controlRocket(ImGui::GetIO().DeltaTime);

    const InputState& in = r.input();

    if (in.leftPressed && !ImGui::GetIO().WantCaptureMouse)
        m_model.addPoint({ in.mouseX, in.mouseY });

    ImGui::Begin("Terrain Editor");

    if (!m_model.isDrawing())
    {
        if (ImGui::Button("Start drawing"))
        {
            m_model.startDrawing();
        }
    }
    else
    {
        ImGui::BeginDisabled(m_model.current().points.size() < 2);
        if (ImGui::Button("Finish"))
        {
            m_model.finish();
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::BeginDisabled(m_model.current().points.empty());
        if (ImGui::Button("Undo point"))
            m_model.undoPoint();

        ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            m_model.cancel();
        }

        ImGui::TextDisabled("Left-click to add points (%d)", (int)m_model.current().points.size());
    }

    ImGui::Separator();

    // Dropdown polylines selector
    const std::string preview = m_model.selectedTerrainName();

    if (ImGui::BeginCombo("Polylines", preview.c_str()))
    {
        for (int i = 0; i < (int)m_model.terrain().polylines.size(); ++i)
        {
            bool sel = (i == m_model.selectedPolyline());
            if (ImGui::Selectable(m_model.terrain().polylines[i].name.c_str(), sel))
                m_model.selectPolyline(i);

            if (sel)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    // Remove selected polyline
    ImGui::BeginDisabled(m_model.selectedPolyline() < 0);
    if (ImGui::Button("Remove selected"))
    {
        m_model.removePolyline(m_model.selectedPolyline());
    }
    ImGui::EndDisabled();

    // rocket part
    ImGui::Separator();
    if (ImGui::Button("Add rocket"))
    {
        m_model.addRocket();
    }

    const std::string rpreview = m_model.selectedRocketName();

    if (ImGui::BeginCombo("Rockets", rpreview.c_str()))
    {
        for (int i = 0; i < (int)m_model.rockets().size(); ++i)
        {
            bool sel = (i == m_model.selectedRocket());
            if (ImGui::Selectable(m_model.rockets()[i].name.c_str(), sel))
                m_model.selectRocket(i);

            if (sel)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::BeginDisabled(m_model.selectedRocket() < 0);
    if (ImGui::Button("Remove selected##rocket"))
    {
        m_model.removeRocket(m_model.selectedRocket());
    }
    ImGui::EndDisabled();
    if (m_model.selectedRocket() >= 0)
        ImGui::TextDisabled("WASD move, Q/E rotate");

    // io part
    ImGui::Separator();

    ImGui::BeginDisabled(m_files.busy());
    if (ImGui::Button("Export..."))
        m_files.requestExport(dialog);

    ImGui::SameLine();

    if (ImGui::Button("Import..."))
        m_files.requestImport(dialog);

    ImGui::EndDisabled();
    if (!m_files.status().empty())
        ImGui::TextWrapped("%s", m_files.status().c_str());

    ImGui::End();

    specPanel();
}

void Editor::specPanel()
{
    const RocketSpec* cur = m_model.selectedSpec();
    if (!cur)
        return;

    RocketSpec s = *cur;
    bool ch = false;

    ImGui::Begin("Rocket Settings");
    ImGui::TextDisabled("%s", m_model.rockets()[m_model.selectedRocket()].name.c_str());

    ImGui::SeparatorText("Mass and fuel [kg]");
    ch |= ImGui::DragFloat("Dry mass",           &s.dryMass,        5.0f,   1.0f, 1e6f, "%.0f");
    ch |= ImGui::DragFloat("Fuel capacity",      &s.fuelCapacity,   5.0f,   0.0f, 1e6f, "%.0f");
    ch |= ImGui::DragFloat("Fuel at start",      &s.fuelStart,      5.0f,   0.0f, s.fuelCapacity, "%.0f");

    ImGui::SeparatorText("Main engine");
    ch |= ImGui::DragFloat("Max thrust (N)",     &s.maxThrust,    100.0f,   0.0f, 1e7f, "%.0f");
    ch |= ImGui::DragFloat("Isp (s)",            &s.isp,            1.0f,   1.0f, 1000.0f, "%.0f");

    ImGui::SeparatorText("Rotation thrusters");
    ch |= ImGui::DragFloat("Thrust, nozzle [N]", &s.rcsThrust,     10.0f,   0.0f, 1e6f, "%.0f");
    ch |= ImGui::DragFloat("Arm [m]",            &s.rcsArm,        0.05f,   0.0f, s.halfHeight, "%.2f");

    ImGui::SeparatorText("Shape [m]");
    ch |= ImGui::DragFloat("Half height",        &s.halfHeight,    0.05f,   0.5f, 20.0f, "%.2f");
    ch |= ImGui::DragFloat("Nose half width",    &s.noseHalfWidth, 0.05f,   0.1f, 20.0f, "%.2f");
    ch |= ImGui::DragFloat("Base half width",    &s.baseHalfWidth, 0.05f,   0.1f, 20.0f, "%.2f");

    ImGui::SeparatorText("Internals (m)");
    ch |= ImGui::DragFloat("Tank width",         &s.tankWidth,     0.05f,   0.0f, 40.0f, "%.2f");
    ch |= ImGui::DragFloat("Tank height",        &s.tankHeight,    0.05f,   0.0f, 40.0f, "%.2f");
    ch |= ImGui::DragFloat("Dry COM Y",          &s.dryComY,       0.05f, -20.0f, 20.0f, "%.2f");

    if (ch)
        m_model.setSelectedSpec(s); // clamps, redraws next frame

    if (ImGui::Button("Reset to defaults"))
        m_model.setSelectedSpec(RocketSpec{});

    float mass = s.dryMass + s.fuelStart;
    ImGui::Separator();
    ImGui::Text("Start mass %.0f kg, thrust/weight %.2f", mass, s.maxThrust / (mass * 9.81f));
    ImGui::End();
}

static void drawMarker(IDrawTarget& r, const Vec2& p)
{
    r.drawLine(p.x - 4, p.y, p.x + 4, p.y);
    r.drawLine(p.x, p.y - 4, p.x, p.y + 4);
}

void Editor::draw(IDrawTarget& r) const
{
    // Finished polylines (selected one highlighted)
    for (int i = 0; i < (int)m_model.terrain().polylines.size(); ++i)
    {
        const auto& pts = m_model.terrain().polylines[i].points;
        if (i == m_model.selectedPolyline())
            r.setColor(255, 200, 60); // light-orange color
        else
            r.setColor(220, 220, 220); // light-gray color

        for (size_t k = 1; k < pts.size(); ++k)
            r.drawLine(pts[k - 1].x, pts[k - 1].y, pts[k].x, pts[k].y);
    }

    // Polyline draw, preview segment to the cursor
    if (m_model.isDrawing())
    {
        r.setColor(90, 200, 255); // light-blue color

        const auto& pts = m_model.current().points;
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
    for (int i = 0; i < (int)m_model.rockets().size(); ++i)
    {
        const Rocket& rk = m_model.rockets()[i];
        if (i == m_model.selectedRocket())
            r.setColor(120, 255, 140); // light-green color
        else
            r.setColor(200, 200, 200); // light-gray color

        // convert rocket body point with rocket's pos, angle
        auto hull = hullLocal(rk.spec);
        for (int k = 0; k < 4; ++k)
        {
            Vec2 a = toPixels(rk, hull[k]);
            Vec2 b = toPixels(rk, hull[(k + 1) % 4]);
            r.drawLine(a.x, a.y, b.x, b.y);
        }
    }
}