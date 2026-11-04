#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <SDL3/SDL.h>

#include "Renderer/Renderer.h"

Renderer::Renderer()
{}

Renderer::~Renderer()
{
    shutdown();
}

bool Renderer::init(const char* title, int w, int h)
{
    if (!SDL_Init(SDL_INIT_VIDEO))
        return false;

    window = SDL_CreateWindow(title, w, h, SDL_WINDOW_RESIZABLE);
    if (!window)
        return false;

    renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer)
        return false;

    ImGui::CreateContext();
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);
    return true;
}

void Renderer::shutdown()
{
    if (!renderer)
        return;

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();

    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    renderer = nullptr;
    window = nullptr;

    SDL_Quit();
}