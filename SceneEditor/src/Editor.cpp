#include <imgui.h>

#include "SceneEditor/Editor.h"
#include "SceneIO/SceneIO.h"

const Terrain& Editor::terrain() const
{
    return m_terrain;
}

void Editor::setTerrain(Terrain t)
{
    m_terrain = std::move(t);
    m_current = {};
    m_drawing = false;
    m_selected = m_terrain.polylines.empty() ? -1 : 0;
    m_counter = (int)m_terrain.polylines.size();
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
            m_status = SceneIO::save(m_terrain, path, &err) ? "Exported: " + path
                                                            : "Export failed: " + err;
        }
        else
        {
            Terrain t;
            if (SceneIO::load(t, path, &err))
            {
                setTerrain(std::move(t));
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
    }
    ImGui::EndDisabled();

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
            r.setColor(255, 200, 60);
        else
            r.setColor(220, 220, 220);

        for (size_t k = 1; k < pts.size(); ++k)
            r.drawLine(pts[k-1].x, pts[k-1].y, pts[k].x, pts[k].y);
    }

    // Polyline draw, preview segment to the cursor
    if (m_drawing)
    {
        r.setColor(90, 200, 255);

        const auto& pts = m_current.points;
        for (size_t k = 1; k < pts.size(); ++k)
            r.drawLine(pts[k-1].x, pts[k-1].y, pts[k].x, pts[k].y);

        for (const auto& p : pts)
            drawMarker(r, p);

        if (!pts.empty())
        {
            r.setColor(90, 200, 255, 120);

            const InputState& in = r.input();
            r.drawLine(pts.back().x, pts.back().y, in.mouseX, in.mouseY);
        }
    }
}