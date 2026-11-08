#include <imgui.h>
#include <SDL3/SDL.h>

#include <catch2/catch_test_macros.hpp>

#include "Renderer/Renderer.h"

namespace
{
    // SDL_Quit clears hints -> set them before every init
    struct Headless
    {
        Renderer r;
        Headless()
        {
            SDL_SetHint(SDL_HINT_VIDEO_DRIVER, "dummy");
            SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
            REQUIRE(r.init("test", 200, 100));
        }
    };

    SDL_Event mouseDown(Uint8 button, float x, float y)
    {
        SDL_Event e{};
        e.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        e.button.button = button;
        e.button.down = true;

        e.button.x = x;
        e.button.y = y;
        return e;
    }
    bool px(const Renderer& r, int x, int y, unsigned char R, unsigned char G, unsigned char B)
    {
        unsigned char c[4];
        return r.readPixel(x, y, c) && c[0] == R && c[1] == G && c[2] == B;
    }
}

TEST_CASE("init creates a window and the loop keeps running", "[renderer]")
{
    Headless h;
    CHECK(h.r.pollEvents());
}

TEST_CASE("idempotent shutdown, renderer can be re-run", "[renderer]")
{
    Headless h;
    h.r.shutdown();
    h.r.shutdown();

    SDL_SetHint(SDL_HINT_VIDEO_DRIVER,  "dummy");
    SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
    CHECK(h.r.init("again", 100, 100));
}

TEST_CASE("quit event ends the loop", "[renderer]")
{
    Headless h;
    SDL_Event e{};
    e.type = SDL_EVENT_QUIT;

    REQUIRE(SDL_PushEvent(&e));
    CHECK_FALSE(h.r.pollEvents());
}

TEST_CASE("left click is a one-frame edge with its position", "[renderer][input]")
{
    Headless h;
    SDL_Event e = mouseDown(SDL_BUTTON_LEFT, 12.0f, 34.0f);
    REQUIRE(SDL_PushEvent(&e));
    h.r.pollEvents();
    CHECK(h.r.input().leftPressed);
    CHECK_FALSE(h.r.input().rightPressed);
    CHECK(h.r.input().mouseX == 12.0f);
    CHECK(h.r.input().mouseY == 34.0f);

    h.r.pollEvents(); // no new event
    CHECK_FALSE(h.r.input().leftPressed);
    CHECK(h.r.input().mouseX == 12.0f); // same pos
}

TEST_CASE("right click sets only the right flag", "[renderer][input]")
{
    Headless h;
    SDL_Event e = mouseDown(SDL_BUTTON_RIGHT, 5.0f, 6.0f);
    REQUIRE(SDL_PushEvent(&e));
    h.r.pollEvents();
    CHECK(h.r.input().rightPressed);
    CHECK_FALSE(h.r.input().leftPressed);
}

TEST_CASE("mouse motion updates the position without clicking", "[renderer][input]")
{
    Headless h;
    SDL_Event e{};

    e.type = SDL_EVENT_MOUSE_MOTION;
    e.motion.x = 70;
    e.motion.y = 80;

    REQUIRE(SDL_PushEvent(&e));
    h.r.pollEvents();

    CHECK(h.r.input().mouseX == 70.0f);
    CHECK(h.r.input().mouseY == 80.0f);
    CHECK_FALSE(h.r.input().leftPressed);
}

TEST_CASE("drawLine and setColor correct on screen", "[renderer][draw]")
{
    Headless h;
    h.r.beginFrame();
    h.r.setColor(255, 0, 0);
    h.r.drawLine(10, 10, 100, 10);

    CHECK(px(h.r, 50, 10, 255, 0, 0));  // line
    CHECK(px(h.r, 50, 50, 20, 22, 28)); // bg from beginFrame
    h.r.endFrame();
}

TEST_CASE("each frame starts from clean bg", "[renderer][draw]")
{
    Headless h;
    h.r.beginFrame();
    h.r.setColor(0, 255, 0);
    h.r.drawLine(10, 10, 100, 10);
    h.r.endFrame();

    h.r.beginFrame();
    CHECK(px(h.r, 50, 10, 20, 22, 28));
    h.r.endFrame();
}

TEST_CASE("many frames with an ImGui window run cleanly", "[renderer][imgui]")
{
    Headless h;
    for (int i = 0; i < 30; ++i)
    {
        h.r.pollEvents();
        h.r.beginFrame();

        ImGui::Begin("Probe");
        ImGui::Text("frame %d", i);
        ImGui::End();

        h.r.endFrame();
    }
    SUCCEED();
}

TEST_CASE("polling the dialog result with none open returns None", "[renderer][dialog]")
{
    Headless h;
    char buf[16] = {};

    CHECK(h.r.pollFileDialog(buf, sizeof buf) == DialogStatus::None);
}