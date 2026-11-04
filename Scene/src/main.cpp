#include <imgui.h>

#include "Renderer/Renderer.h"

int main(int, char**) {
    Renderer r;
    if (!r.init("SpaceLander2D", 1280, 720))
        return 1;

    while (r.pollEvents())
    {
        r.beginFrame();

        ImGui::Begin("debug");
        ImGui::Text("fps: %.1f", ImGui::GetIO().Framerate);
        ImGui::End();

        r.setColor(255, 255, 255);
        r.drawLine(100, 600, 1100, 500);   // terrain placeholder

        r.endFrame();
    }
    return 0;
}