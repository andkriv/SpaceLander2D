#include <string>

#include "EditorModel/EditorModel.h"

bool EditorModel::startDrawing()
{
    if (m_drawing)
        return false;
    m_drawing = true;
    m_current = {};
    return true;
}

void EditorModel::addPoint(Vec2 p)
{
    if (m_drawing)
        m_current.points.push_back(p);
}

bool EditorModel::undoPoint()
{
    if (!m_drawing || m_current.points.empty())
        return false;
    m_current.points.pop_back();
    return true;
}

bool EditorModel::finish()
{
    if (!m_drawing || m_current.points.size() < 2)
        return false;
    m_current.name = "Polyline " + std::to_string(++m_counter);
    m_terrain.polylines.push_back(std::move(m_current));
    m_selected = (int)m_terrain.polylines.size() - 1;
    m_current = {};
    m_drawing = false;
    return true;
}

void EditorModel::cancel()
{
    m_current = {};
    m_drawing = false;
}

bool EditorModel::removePolyline(int i)
{
    if (i < 0 || i >= (int)m_terrain.polylines.size())
        return false;
    m_terrain.polylines.erase(m_terrain.polylines.begin() + i);
    m_selected = -1;
    return true;
}

void EditorModel::selectPolyline(int i)
{
    if (i >= 0 && i < (int)m_terrain.polylines.size())
        m_selected = i;
}

int EditorModel::addRocket()
{
    Rocket rk;
    rk.name = "Rocket " + std::to_string(++m_rocketCounter);
    rk.pos = { 400.0f + 60.0f * (m_rocketCounter - 1), 200.0f };
    m_rockets.push_back(std::move(rk));
    m_selectedRocket = (int)m_rockets.size() - 1;
    return m_selectedRocket;
}

bool EditorModel::removeRocket(int i)
{
    if (i < 0 || i >= (int)m_rockets.size())
        return false;
    m_rockets.erase(m_rockets.begin() + i);
    m_selectedRocket = -1;
    return true;
}

void EditorModel::selectRocket(int i)
{
    if (i >= 0 && i < (int)m_rockets.size())
        m_selectedRocket = i;
}

std::string EditorModel::selectedRocketName() const
{
    return m_selectedRocket >= 0 ? m_rockets[m_selectedRocket].name
                                 : "(none)";
}

bool EditorModel::moveSelectedRocket(Vec2 d, float dAngle)
{
    if (m_selectedRocket < 0)
        return false;
    Rocket& rk = m_rockets[m_selectedRocket];
    rk.pos += d;
    rk.angle += dAngle;
    return true;
}

void EditorModel::setScene(Scene s)
{
    m_terrain = std::move(s.terrain);
    m_rockets = std::move(s.rockets);

    m_current = {};
    m_drawing = false;
    m_selected = m_terrain.polylines.empty() ? -1 : 0;
    m_counter = (int)m_terrain.polylines.size();

    m_selectedRocket = m_rockets.empty() ? -1 : 0;
    m_rocketCounter = (int)m_rockets.size();
}

std::string EditorModel::selectedTerrainName() const
{
    return m_selected >= 0 ? m_terrain.polylines[m_selected].name
                           : "(none)";
}

Scene EditorModel::scene() const
{
    return Scene{ m_terrain, m_rockets };
}

bool EditorModel::isDrawing() const
{
    return m_drawing;
}

const Terrain& EditorModel::terrain() const
{
    return m_terrain;
}

const Polyline& EditorModel::current() const
{
    return m_current;
}

const std::vector<Rocket>& EditorModel::rockets() const
{
    return m_rockets;
}

int EditorModel::selectedPolyline() const
{
    return m_selected;
}

int EditorModel::selectedRocket() const
{
    return m_selectedRocket;
}