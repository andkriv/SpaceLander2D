#include <imgui.h>

#include "Renderer/Renderer.h"
#include "SceneEditor/Editor.h"

int main(int, char**) {
    Renderer r;
    if (!r.init("SpaceLander2D", 1280, 720))
        return 1;

    Editor editor;

    while (r.pollEvents())
    {
        r.beginFrame();

        editor.update(r);
        editor.draw(r);

        r.endFrame();
    }
    return 0;
}