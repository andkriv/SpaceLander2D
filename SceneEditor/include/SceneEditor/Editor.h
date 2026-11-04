#pragma once
#include <string>

#include "Renderer/Renderer.h"
#include "SceneData/Terrain.h"

class Editor
{
public:
    void update(Renderer& r);
    void draw(Renderer& r) const;

    const Terrain& terrain() const;
    void setTerrain(Terrain t);

private:
    enum class Pending
    {
        None,
        Import,
        Export
    };

private:
    void pollDialog(Renderer& r);

private:
    bool     m_drawing = false;

    Terrain  m_terrain;
    Polyline m_current;
    int      m_selected = -1;
    int      m_counter  = 0;

    Pending     m_pending = Pending::None;
    std::string m_status;
};