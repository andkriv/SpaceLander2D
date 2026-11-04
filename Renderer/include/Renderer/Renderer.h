#pragma once
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <SDL3/SDL.h>

#include "Export.h"

class RENDERER_API Renderer
{
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool init(const char* title, int width, int height);
    void shutdown();

private:
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
};