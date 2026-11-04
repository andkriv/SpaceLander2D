#include <imgui.h>

#include "Renderer/Renderer.h"

int main(int, char**) {
    Renderer r;
    if (!r.init("SpaceLander2D", 1280, 720))
        return 1;

    while (r.pollEvents())
    {
        r.beginFrame();
        r.endFrame();
    }
    return 0;
}