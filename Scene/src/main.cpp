#include <imgui.h>

#include "Renderer/Renderer.h"
#include "SceneEditor/Editor.h"
#include "SceneExecution/Execution.h"

enum class Mode
{
    Edit,
    Run
};

int main(int, char**) {
    Renderer r;
    if (!r.init("SpaceLander2D", 1280, 720))
        return 1;

    Editor    editor;
    Execution exec;
    Mode      mode = Mode::Edit;

    while (r.pollEvents())
    {
        r.beginFrame();

        if (mode == Mode::Edit)
        {
            editor.update(r);
            editor.draw(r);

            ImGui::Begin("Mode");
            if (ImGui::Button("Run simulation"))
            {
                exec.start(editor.scene());
                mode = Mode::Run;
            }
            ImGui::End();
        }
        else
        {
            exec.update(r);
            exec.draw(r);

            if (exec.wantsStop())
                mode = Mode::Edit;
        }

        r.endFrame();
    }
    return 0;
}