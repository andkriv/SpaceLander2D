#include <mutex>
#include <string>
#include <cstring>

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <SDL3/SDL.h>

#include "Renderer/Renderer.h"

struct Renderer::Impl
{
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;

    InputState input;

    std::mutex  dlgMutex;
    std::string dlgPath;
    int         dlgState = 0;   // 0 none, 1 picked, 2 cancelled
};

Renderer::Renderer() : m_impl(new Impl)
{}
Renderer::~Renderer()
{
    shutdown();
    delete m_impl;
}

bool Renderer::init(const char* title, int w, int h)
{
    if (!SDL_Init(SDL_INIT_VIDEO))
        return false;

    m_impl->window = SDL_CreateWindow(title, w, h, SDL_WINDOW_RESIZABLE);
    if (!m_impl->window)
        return false;

    m_impl->renderer = SDL_CreateRenderer(m_impl->window, nullptr);
    if (!m_impl->renderer)
        return false;

    SDL_SetRenderVSync(m_impl->renderer, 1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForSDLRenderer(m_impl->window, m_impl->renderer);
    ImGui_ImplSDLRenderer3_Init(m_impl->renderer);
    return true;
}

void Renderer::shutdown()
{
    if (!m_impl->renderer)
        return;

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();

    ImGui::DestroyContext();
    SDL_DestroyRenderer(m_impl->renderer);
    SDL_DestroyWindow(m_impl->window);

    m_impl->renderer = nullptr;
    m_impl->window = nullptr;

    SDL_Quit();
}

bool Renderer::pollEvents()
{
    bool running = true;
    m_impl->input.leftPressed = false;
    m_impl->input.rightPressed = false;

    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
        ImGui_ImplSDL3_ProcessEvent(&e);
        if (e.type == SDL_EVENT_QUIT)
            running = false;

        if (e.type == SDL_EVENT_MOUSE_MOTION)
        {
            m_impl->input.mouseX = e.motion.x;
            m_impl->input.mouseY = e.motion.y;
        }

        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            m_impl->input.mouseX = e.button.x;
            m_impl->input.mouseY = e.button.y;

            if (e.button.button == SDL_BUTTON_LEFT)
                m_impl->input.leftPressed = true;

            if (e.button.button == SDL_BUTTON_RIGHT)
                m_impl->input.rightPressed = true;
        }
    }

    return running;
}

void Renderer::beginFrame()
{
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();

    SDL_SetRenderDrawColor(m_impl->renderer, 20, 22, 28, 255);
    SDL_RenderClear(m_impl->renderer);
}

void Renderer::endFrame()
{
    ImGui::Render();

    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), m_impl->renderer);
    SDL_RenderPresent(m_impl->renderer);
}

const InputState& Renderer::input() const
{
    return m_impl->input;
}

void Renderer::setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
    SDL_SetRenderDrawColor(m_impl->renderer, r, g, b, a);
}

void Renderer::drawLine(float x0, float y0, float x1, float y1)
{
    SDL_RenderLine(m_impl->renderer, x0, y0, x1, y1);
}

void Renderer::showFileDialog(DialogType type)
{
    static const SDL_DialogFileFilter filters[] = { { "Scene (*.json)", "json" } };

    auto cb = [](void* ud, const char* const* files, int /*filter*/)
    {
        auto* im = static_cast<Impl*>(ud);
        std::lock_guard<std::mutex> lk(im->dlgMutex);

        if (files && files[0])
        {
            im->dlgPath = files[0];
            im->dlgState = 1;
        }
        else
        {
            im->dlgState = 2; // cancelled or error
        }
    };

    if (type == DialogType::Save)
        SDL_ShowSaveFileDialog(cb, m_impl, m_impl->window, filters, 1, nullptr);
    else
        SDL_ShowOpenFileDialog(cb, m_impl, m_impl->window, filters, 1, nullptr, false);
}

DialogStatus Renderer::pollFileDialog(char* out, int cap)
{
    std::lock_guard<std::mutex> lk(m_impl->dlgMutex);
    if (m_impl->dlgState == 0) return DialogStatus::None;

    DialogStatus s = (m_impl->dlgState == 1) ? DialogStatus::Picked
                                             : DialogStatus::Cancelled;
    if (s == DialogStatus::Picked && cap > 0)
    {
        std::strncpy(out, m_impl->dlgPath.c_str(), cap - 1);
        out[cap - 1] = '\0';
    }
    m_impl->dlgState = 0;
    return s;
}

bool Renderer::readPixel(int x, int y, unsigned char out[4]) const
{
    SDL_Rect rc{ x, y, 1, 1 };
    SDL_Surface* s = SDL_RenderReadPixels(m_impl->renderer, &rc);

    if (!s)
        return false;

    Uint8 r, g, b, a;
    bool ok = SDL_ReadSurfacePixel(s, 0, 0, &r, &g, &b, &a);
    SDL_DestroySurface(s);

    if (ok)
    {
        out[0] = r;
        out[1] = g;
        out[2] = b;
        out[3] = a;
    }
    return ok;
}