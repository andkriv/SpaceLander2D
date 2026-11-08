#pragma once
#include <string>
#include <vector>

#include "Renderer/Renderer.h"
#include "EditorModel/EditorModel.h"
#include "SceneData/Rocket.h"

class Editor
{
public:
    void update(Renderer& r);
    void draw(Renderer& r) const;

    const Terrain& terrain() const;
    void setScene(Scene s);

    Scene scene() const;

private:
    enum class Pending
    {
        None,
        Import,
        Export
    };

private:
    void pollDialog(Renderer& r);

    void controlRocket(float dt);

private:

    Pending     m_pending = Pending::None;
    std::string m_status;

    EditorModel m_model;
};