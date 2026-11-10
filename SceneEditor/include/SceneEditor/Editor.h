#pragma once
#include <string>
#include <vector>

#include "RenderApi/RenderApi.h"
#include "EditorModel/EditorModel.h"
#include "EditorModel/SceneFileFlow.h"

class Editor
{
public:
    void update(IDrawTarget& view, IFileDialog& dialog);
    void draw(IDrawTarget& view) const;

    const Terrain& terrain() const;
    void setScene(Scene s);

    Scene scene() const;
    EditorModel& model();
    const EditorModel& model() const;

private:
    void controlRocket(float dt);
    void specPanel();

private:
    EditorModel m_model;
    SceneFileFlow m_files;
};