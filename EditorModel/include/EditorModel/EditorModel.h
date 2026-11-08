#pragma once
#include "SceneData/Scene.h"

class EditorModel
{
public:
    // polylines
    bool startDrawing();

    void addPoint(Vec2 p);
    bool undoPoint();

    bool finish();
    void cancel();

    bool removePolyline(int i);
    void selectPolyline(int i);

    // rockets
    int  addRocket();
    bool removeRocket(int i);
    void selectRocket(int i);
    bool moveSelectedRocket(Vec2 delta, float dAngle);

    // scene
    bool isDrawing() const;

    void  setScene(Scene s);
    Scene scene() const;

    const Terrain& terrain() const;
    const Polyline& current() const;
    int selectedPolyline() const;
    std::string selectedTerrainName() const;

    const std::vector<Rocket>& rockets() const;
    std::string selectedRocketName() const;
    int selectedRocket() const;

private:
    bool     m_drawing = false;

    Terrain  m_terrain;
    Polyline m_current;
    int      m_selected = -1;
    int      m_counter  = 0;

    std::vector<Rocket> m_rockets;
    int m_selectedRocket = -1;
    int m_rocketCounter  = 0;
};