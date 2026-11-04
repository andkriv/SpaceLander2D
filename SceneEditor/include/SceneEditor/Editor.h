#pragma once
#include "Renderer/Renderer.h"

#include "Terrain.h"

class Editor
{
public:
    void update(Renderer& r);
    void draw(Renderer& r) const;
    const Terrain& terrain() const;

private:
    bool     m_drawing = false;

    Terrain  m_terrain;
    Polyline m_current;
    int      m_selected = -1;
    int      m_counter  = 0;
};