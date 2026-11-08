#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "SceneEditor/Editor.h"
#include "SceneExecution/Execution.h"

#include "ui_placeholder.h"

using Catch::Approx;
using C = Rec::Color;

namespace
{
    constexpr C   kGrayPoly { 220, 220, 220 },
                   kSelPoly { 255, 200,  60 },
                      kBlue {  90, 200, 255 },
                kGrayRocket { 200, 200, 200 },
                     kGreen { 120, 255, 140 },
                       kRed { 255,  90,  90 },
                     kFlame { 255, 170,  40 };

    constexpr float kPi = 3.14159265f; // pi

    void addLine(EditorModel& m, Vec2 a, Vec2 b, Vec2 c = { -1, -1 })
    {
        m.startDrawing();
        m.addPoint(a);
        m.addPoint(b);

        if (c.x >= 0)
            m.addPoint(c);

        m.finish();
    }

    Scene sceneOf(std::vector<Vec2> rocketsPx, bool ground)
    {
        Scene s;
        if (ground)
        {
            Polyline pl;
            pl.name = "ground";
            pl.points = { { 0, 600 }, { 2000, 600 } };
            s.terrain.polylines.push_back(pl);
        }

        int n = 0;
        for (Vec2 p : rocketsPx)
        {
            Rocket r;
            r.name = "R" + std::to_string(++n);
            r.pos = p;
            s.rockets.push_back(r);
        }
        return s;
    }
    float len(const Rec::Line& l) { return std::hypot(l.x1 - l.x0, l.y1 - l.y0); }
}

// ---------------- Editor::draw ----------------

TEST_CASE("empty editor draws nothing", "[draw][editor]")
{
    Editor e; Rec r;
    e.draw(r);

    CHECK(r.lines.empty());
}

TEST_CASE("finished polylines are drawn segment by segment, selected highlighted", "[draw][editor]")
{
    Editor e; Rec r;
    addLine(e.model(), { 0, 0 }, { 10, 0 }, { 10, 10 }); // 2 seg
    addLine(e.model(), { 50, 50 }, { 60, 60 });          // 1 seg, selected
    e.draw(r);

    REQUIRE(r.lines.size() == 3);
    CHECK(r.count(kGrayPoly) == 2);
    CHECK(r.count(kSelPoly) == 1);

    CHECK(r.lines[0].x0 == 0.0f);
    CHECK(r.lines[0].x1 == 10.0f);

    CHECK(r.lines[2].x0 == 50.0f);
    CHECK(r.lines[2].y1 == 60.0f);

    Rec r2;
    e.model().selectPolyline(0);
    e.draw(r2);

    CHECK(r2.count(kSelPoly) == 2);
}

TEST_CASE("polyline in progress shows segments, point markers and a cursor preview", "[draw][editor]")
{
    Editor e; Rec r;
    e.model().startDrawing();
    e.model().addPoint({ 10, 10 });
    e.model().addPoint({ 40, 10 });

    r.in.mouseX = 77;
    r.in.mouseY = 88;
    e.draw(r);

    REQUIRE(r.lines.size() == 6); // 1 segment + 2 markers x 2 lines + preview
    CHECK(r.count(kBlue) == 5);

    const auto& preview = r.lines.back();
    CHECK(preview.c.a == 120);

    CHECK(preview.x0 == 40.0f); 
    CHECK(preview.y0 == 10.0f);

    CHECK(preview.x1 == 77.0f); 
    CHECK(preview.y1 == 88.0f);
}

TEST_CASE("no preview line before the first point", "[draw][editor]")
{
    Editor e; Rec r;
    e.model().startDrawing();
    e.draw(r);

    CHECK(r.lines.empty());
}

TEST_CASE("rocket is a four-edge trapezoid at pos", "[draw][editor]")
{
    Editor e; Rec r;
    e.model().addRocket(); // at (400, 200)
    e.draw(r);

    REQUIRE(r.lines.size() == 4);
    CHECK(r.count(kGreen) == 4);

    CHECK(r.lines[0].x0 == Approx(292.0f));
    CHECK(r.lines[0].y0 == Approx(180.0f)); // narrow nose

    CHECK(r.lines[0].x1 == Approx(308.0f));

    CHECK(r.lines[2].x0 == Approx(316.0f));
    CHECK(r.lines[2].y0 == Approx(220.0f)); // wide base
}

TEST_CASE("only selected rocket is highlighted", "[draw][editor]")
{
    Editor e; Rec r;
    e.model().addRocket(); e.model().addRocket(); // second is selected
    e.draw(r);

    REQUIRE(r.lines.size() == 8);
    CHECK(r.count(kGrayRocket) == 4);
    CHECK(r.count(kGreen) == 4);
    CHECK(r.lines[0].c == kGrayRocket);
}

TEST_CASE("rocket drawing follows rotation", "[draw][editor]")
{
    Editor e; Rec r;
    e.model().addRocket();
    e.model().moveSelectedRocket({ 0, 0 }, kPi / 2); // nose points right
    e.draw(r);

    CHECK(r.lines[0].x0 == Approx(320.0f).margin(1e-3));
    CHECK(r.lines[0].y0 == Approx(192.0f).margin(1e-3));
}

// ---------------- Execution::draw ----------------

TEST_CASE("run mode draws terrain back in editor pixels", "[draw][execution]")
{
    Execution ex; Rec r;
    ex.start(sceneOf({}, true));
    ex.draw(r);

    REQUIRE(r.lines.size() == 1);
    CHECK(r.lines[0].c == kGrayPoly);

    CHECK(r.lines[0].x0 == Approx(0.0f));
    CHECK(r.lines[0].y0 == Approx(600.0f));

    CHECK(r.lines[0].x1 == Approx(2000.0f));
    CHECK(r.lines[0].y1 == Approx(600.0f));
}

TEST_CASE("each rocket adds four hull edges, selected green", "[draw][execution]")
{
    Execution ex; Rec r;
    ex.start(sceneOf({ { 300, 200 }, { 700, 200 } }, false));
    ex.draw(r);

    CHECK(r.lines.size() == 8);
    CHECK(r.count(kGreen) == 4);
    CHECK(r.count(kGrayRocket) == 4);
}

TEST_CASE("rocket under thrust draws a flame scaled by throttle", "[draw][execution]")
{
    Execution ex; Rec r;
    ex.start(sceneOf({ { 300, 200 } }, false));
    ex.draw(r);
    CHECK(r.count(kFlame) == 0); // idle, no flame

    Physics::Input full;
    full.throttle = 1.0f;
    ex.sim().step(Simulation::kStep * 1.5f, full);
    Rec r2;
    ex.draw(r2);

    REQUIRE(r2.count(kFlame) == 1);
    const auto& f = r2.lines.back();
    CHECK(len(f) == Approx(30.0f).margin(0.5f)); // 3m full thrust, 10 px/m
    CHECK(f.y1 > f.y0);                          // exhaust points down for a rocket pointing up

    Physics::Input half;
    half.throttle = 0.5f;
    ex.sim().step(Simulation::kStep * 1.5f, half);
    Rec r3;
    ex.draw(r3);

    CHECK(len(r3.lines.back()) == Approx(15.0f).margin(0.5f));
}

TEST_CASE("crashed rocket is red", "[draw][execution]")
{
    Execution ex; Rec r;
    ex.start(sceneOf({ { 100, 300 } }, true)); // ~28m drop
    for (int i = 0; i < 160; ++i)
        ex.sim().step(0.05f, {});

    REQUIRE(ex.sim().rockets()[0].is_crashed);
    ex.draw(r);
    CHECK(r.count(kRed) == 4);
    CHECK(r.count(kGreen) == 0);
}