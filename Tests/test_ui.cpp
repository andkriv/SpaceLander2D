#include <cfloat>
#include <filesystem>

#include <imgui.h>
#include <imgui_internal.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "SceneEditor/Editor.h"
#include "SceneExecution/Execution.h"
#include "SceneIO/SceneIO.h"

#include "ui_placeholder.h"

using Catch::Approx;
namespace fs = std::filesystem;

namespace
{
    // ImGui context, no backend (widgets, their results)
    struct Ui
    {
        Ui()
        {
            IMGUI_CHECKVERSION();
            ctx = ImGui::CreateContext();
            ImGuiIO& io = ImGui::GetIO();
            io.IniFilename = nullptr;
            io.DisplaySize = { 1280, 720 };
            io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
            unsigned char* pixels; int w, h;
            io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h); // atlas for NewFrame
        }
        ~Ui()
        {
            ImGui::DestroyContext(ctx);
        }

        template <class F> void frame(F&& body)
        {
            ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
            ImGui::NewFrame();
            body();
            ImGui::Render();
        }
        template <class F> void frames(int n, F&& body)
        {
            for (int i = 0; i < n; ++i)
                frame(body);
        }

        // activate a btn/checkbox by label, hover, click. disabled never hovered
        template <class F> void press(F&& body, const char* window, const char* label)
        {
            ImGuiWindow* w = ImGui::FindWindowByName(window);
            REQUIRE(w != nullptr);

            ImGuiContext& g = *ImGui::GetCurrentContext();
            ImGuiIO& io = ImGui::GetIO();
            const ImGuiID id = w->GetID(label);

            bool found = false;
            for (float y = w->Pos.y + 2; y < w->Pos.y + w->Size.y && !found; y += 3.0f)
            {
                for (float x = w->Pos.x + 4; x < w->Pos.x + w->Size.x; x += 6.0f)
                {
                    io.AddMousePosEvent(x, y);
                    frame(body);

                    if (g.HoveredId == id)
                    {
                        found = true;
                        break;
                    }
                }
            }
            if (!found)
                return;

            io.AddMouseButtonEvent(0, true);
            frames(2, body);

            io.AddMouseButtonEvent(0, false);
            frames(2, body);

            io.AddMousePosEvent(-FLT_MAX, -FLT_MAX); frames(2, body); // move, next press clean
        }

        void hover(float x, float y)
        {
            ImGui::GetIO().AddMousePosEvent(x, y);
        }

        void key(ImGuiKey k, bool down)
        {
            ImGui::GetIO().AddKeyEvent(k, down);
        }

        ImGuiContext* ctx;
    };

    Scene testScene()
    {
        Scene s;

        Rocket r;
        r.name = "R1";
        r.pos = { 400, 200 };

        s.rockets.push_back(r);
        return s;
    }
}

// ---------------- editor panel ----------------

TEST_CASE("the Terrain Editor window is created", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
            ed.update(view, dlg);
    };
    ui.frames(2, body);

    CHECK(ImGui::FindWindowByName("Terrain Editor") != nullptr);
}

TEST_CASE("buttons start, finish, add, remove", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ui.frames(2, body);

    ui.press(body, "Terrain Editor", "Start drawing");
    REQUIRE(ed.model().isDrawing());

    ui.press(body, "Terrain Editor", "Finish"); // no points = disabled
    CHECK(ed.model().isDrawing());

    ed.model().addPoint({ 1, 1 });
    ed.model().addPoint({ 2, 2 });
    ui.frames(2, body);
    ui.press(body, "Terrain Editor", "Finish");
    CHECK_FALSE(ed.model().isDrawing());
    REQUIRE(ed.model().terrain().polylines.size() == 1);

    ui.press(body, "Terrain Editor", "Remove selected");
    CHECK(ed.model().terrain().polylines.empty());

    ui.press(body, "Terrain Editor", "Add rocket");
    REQUIRE(ed.model().rockets().size() == 1);

    ui.press(body, "Terrain Editor", "Remove selected##rocket");
    CHECK(ed.model().rockets().empty());
}

TEST_CASE("Cancel and Undo point work from the panel", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ui.frames(2, body);

    ed.model().startDrawing();
    ed.model().addPoint({ 1, 1 });
    ed.model().addPoint({ 2, 2 });
    ui.frames(2, body);
    ui.press(body, "Terrain Editor", "Undo point");
    CHECK(ed.model().current().points.size() == 1);

    ui.press(body, "Terrain Editor", "Cancel");
    CHECK_FALSE(ed.model().isDrawing());
}

TEST_CASE("viewport click adds point only while drawing", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ui.frames(2, body);

    ui.hover(1200, 700); // empty canvas
    ui.frames(2, body);
    view.in = { 1200, 700, true, false };
    ui.frame(body);
    CHECK(ed.model().current().points.empty()); // no drawing

    ed.model().startDrawing();
    ui.frame(body);
    CHECK(ed.model().current().points.size() == 1);
    CHECK(ed.model().current().points[0].x == 1200.0f);

    view.in.leftPressed = false;
    ui.frame(body);
    CHECK(ed.model().current().points.size() == 1); // edge, not level
}

TEST_CASE("click on the panel does not add a terrain point", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ed.model().startDrawing();
    ui.frames(2, body);

    ui.hover(80, 100); // inside the panel
    ui.frames(2, body);
    view.in = { 80, 100, true, false };
    ui.frame(body);

    CHECK(ed.model().current().points.empty());
}

TEST_CASE("WASD moves, Q/E rotates sel rocket", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ed.model().addRocket();
    ed.model().addRocket();
    ed.model().selectRocket(0);
    ui.frames(2, body);

    ui.key(ImGuiKey_D, true); 
    ui.frames(60, body); 
    ui.key(ImGuiKey_D, false);
    CHECK(ed.model().rockets()[0].pos.x == Approx(600.0f).margin(10.0f)); // 200 px/s for 1s
    CHECK(ed.model().rockets()[1].pos.x == Approx(460.0f));              // other untouched

    ui.key(ImGuiKey_W, true); 
    ui.frames(30, body); 
    ui.key(ImGuiKey_W, false);
    CHECK(ed.model().rockets()[0].pos.y == Approx(100.0f).margin(10.0f));// -y=up

    ui.key(ImGuiKey_E, true);  ui.frames(30, body);  ui.key(ImGuiKey_E, false);
    CHECK(ed.model().rockets()[0].angle == Approx(1.0f).margin(0.1f));   // 2 rad/s for 0.5s

    ui.key(ImGuiKey_Q, true);  ui.frames(30, body);  ui.key(ImGuiKey_Q, false);
    CHECK(ed.model().rockets()[0].angle == Approx(0.0f).margin(0.1f));
}

TEST_CASE("movement keys with no sel rocket selected = nothing", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ui.key(ImGuiKey_D, true);
    ui.frames(10, body);

    CHECK(ed.model().rockets().empty());
}

TEST_CASE("import/export btns open the matching dialog, lock while busy", "[ui][editor]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ui.frames(2, body);

    ui.press(body, "Terrain Editor", "Export...");
    CHECK(dlg.shown == 1);
    CHECK(dlg.lastType == DialogType::Save);

    ui.press(body, "Terrain Editor", "Import..."); // first is open, disabled
    CHECK(dlg.shown == 1);
}

TEST_CASE("export then import through the panel restores the scene", "[ui][editor]")
{
    fs::path file = fs::temp_directory_path() / "rocketsim_ui_roundtrip.json";
    fs::remove(file);

    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };
    ui.frames(2, body);
    ed.model().startDrawing();
    ed.model().addPoint({ 5, 5 });
    ed.model().addPoint({ 9, 9 });
    ed.model().finish();

    ed.model().addRocket();
    ui.frames(2, body);

    ui.press(body, "Terrain Editor", "Export...");
    dlg.next = DialogStatus::Picked;
    dlg.path = file.string();
    ui.frame(body);
    REQUIRE(fs::exists(file));

    ed.model().setScene(Scene{}); // clr editor
    ui.frames(2, body);
    ui.press(body, "Terrain Editor", "Import...");
    dlg.next = DialogStatus::Picked; dlg.path = file.string();
    ui.frame(body);

    CHECK(ed.model().terrain().polylines.size() == 1);
    CHECK(ed.model().rockets().size() == 1);
    fs::remove(file);
}

// ---------------- run (sim) panel ----------------

TEST_CASE("Stop asks the app to leave run mode", "[ui][execution]")
{
    Ui ui;
    Execution ex;
    Rec view;

    auto body = [&]
    {
        ex.update(view);
    };
    ex.start(testScene());
    ui.frames(2, body);

    CHECK_FALSE(ex.wantsStop());
    ui.press(body, "Simulation", "Stop");
    CHECK(ex.wantsStop());
}

TEST_CASE("simulation steps with frame time", "[ui][execution]")
{
    Ui ui;
    Execution ex;
    Rec view;

    auto body = [&]
    {
        ex.update(view);
    };
    ex.start(testScene());
    float y0 = ex.sim().rockets()[0].state.pos.y;
    ui.frames(60, body); // g for 1s

    CHECK(ex.sim().rockets()[0].state.vel.y == Approx(9.81f).epsilon(0.05));
    CHECK(ex.sim().rockets()[0].state.pos.y > y0 + 4.0f);
}

TEST_CASE("Pause freezes the simulation", "[ui][execution]")
{
    Ui ui;
    Execution ex;
    Rec view;
    auto body = [&]
    {
        ex.update(view);
    };

    ex.start(testScene());
    ui.frames(10, body);
    ui.press(body, "Simulation", "Pause");

    REQUIRE(ex.paused());

    float y = ex.sim().rockets()[0].state.pos.y;
    ui.frames(30, body);
    CHECK(ex.sim().rockets()[0].state.pos.y == y);
}

TEST_CASE("Reset puts the rockets back", "[ui][execution]")
{
    Ui ui;
    Execution ex;
    Rec view;

    auto body = [&]
    {
        ex.update(view);
    };
    ex.start(testScene());

    float y0 = ex.sim().rockets()[0].state.pos.y;
    ui.frames(60, body);

    REQUIRE(ex.sim().rockets()[0].state.pos.y > y0 + 1.0f);

    ui.press(body, "Simulation", "Reset");
    CHECK(ex.sim().rockets()[0].state.pos.y == Approx(y0).margin(0.2f)); // 2 frames may pass after reset
    CHECK(ex.sim().rockets()[0].state.fuel == Approx(ex.sim().spec().fuelCapacity));
}

TEST_CASE("W applies throttle and burns fuel; Q/E rotates", "[ui][execution]")
{
    Ui ui;
    Execution ex;
    Rec view;
    auto body = [&]
    {
        ex.update(view);
    };
    ex.start(testScene());
    ui.frames(2, body);

    ui.key(ImGuiKey_W, true);
    ui.frames(60, body);
    ui.key(ImGuiKey_W, false);
    const auto& b = ex.sim().rockets()[0];

    CHECK(b.state.vel.y < -5.0f); // 16.7 up, 9.81 gravity
    CHECK(b.state.fuel < ex.sim().spec().fuelCapacity);

    ui.key(ImGuiKey_E, true);  ui.frames(30, body);  ui.key(ImGuiKey_E, false);
    CHECK(b.state.angVel > 0.1f);
}

// ---------------- edit -> sim ----------------

TEST_CASE("scene built in the editor runs in the sim", "[ui][integration]")
{
    Editor ed;
    ed.model().startDrawing();
    ed.model().addPoint({ 0, 600 });
    ed.model().addPoint({ 2000, 600 });
    ed.model().finish();
    ed.model().addRocket(); // 40m above the floor

    Execution ex;
    ex.start(ed.model().scene());
    for (int i = 0; i < 200; ++i)
        ex.sim().step(0.05f, {});

    const auto& b = ex.sim().rockets()[0];
    CHECK(b.is_crashed); // ~27m/s impact

    Vec2 c = Physics::toWorld(ex.sim().spec(), b.state, { 0, 0 });
    CHECK(c.y < 60.0f);
    CHECK(c.y > 57.0f); // resting on the floor, not through it
}

TEST_CASE("apply thrust level", "[ui][execution][level]")
{
    Ui ui;
    Execution ex;
    Rec view;

    auto body = [&]
    {
        ex.update(view);
    };

    ex.start(testScene());
    ex.sim().world().gravity = { 0, 0 };
    ui.frames(2, body);

    ui.key(ImGuiKey_1, true);
    ui.frames(1, body);

    ui.key(ImGuiKey_1, false);
    ui.frames(1, body);
    CHECK(ex.sim().engineLevel() == 0);

    ui.key(ImGuiKey_W, true);
    ui.frames(10, body);
    CHECK(ex.sim().rockets()[0].info.thrust == Approx(6250.0f));  // 25% of 25 [kN]
    ui.key(ImGuiKey_W, false);

    ui.key(ImGuiKey_3, true);
    ui.frames(1, body);

    ui.key(ImGuiKey_3, false);
    ui.frames(1, body);

    ui.key(ImGuiKey_W, true);
    ui.frames(10, body);
    CHECK(ex.sim().rockets()[0].info.thrust == Approx(18750.0f));

    ui.key(ImGuiKey_W, false);
}

TEST_CASE("levels effects rcs, for now", "[ui][execution][level]")
{
    Ui ui;
    Execution ex;
    Rec view;

    auto body = [&]
    {
        ex.update(view);
    };

    ex.start(testScene());
    ui.frames(2, body);

    ui.key(ImGuiKey_2, true);
    ui.frames(1, body);

    ui.key(ImGuiKey_2, false);
    ui.frames(1, body);

    ui.key(ImGuiKey_E, true);
    ui.frames(10, body);
    CHECK(ex.sim().rockets()[0].info.rcsForce == Approx(1500.0f)); // 50% of 3 [kN]

    ui.key(ImGuiKey_E, false);
}

TEST_CASE("editing spec changes the rocket", "[ui][editor][spec]")
{
    Ui ui;
    Editor ed;
    Rec view;
    TestDialog dlg;

    auto body = [&]
    {
        ed.update(view, dlg);
    };

    ed.model().addRocket();
    ui.frames(2, body);
    CHECK(ImGui::FindWindowByName("Rocket Settings") != nullptr);

    ui.press(body, "Rocket Settings", "Reset to defaults");
    CHECK(ed.model().rockets()[0].spec.dryMass == Approx(800.0f));
}