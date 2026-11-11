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
                     kFlame { 255, 170,  40 },      // light-orange color
                       kRcs { 140, 220, 255 },      // light-blue color
                     kTrail {  90, 160, 255, 180 }, // blue color
                   kPredict { 255, 230, 120, 150 }; // light-yellow color

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
    Rec r;
    Editor e;
    e.draw(r);

    CHECK(r.lines.empty());
}

TEST_CASE("finished polylines are drawn segment by segment, selected highlighted", "[draw][editor]")
{
    Rec r;
    Editor e;
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
    Rec r;
    Editor e;
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
    Rec r;
    Editor e;
    e.model().startDrawing();
    e.draw(r);

    CHECK(r.lines.empty());
}

TEST_CASE("rocket is a four-edge trapezoid at pos", "[draw][editor]")
{
    Rec r;
    Editor e;
    e.model().addRocket(); // at (400, 200)
    e.draw(r);

    REQUIRE(r.lines.size() == 4);
    CHECK(r.count(kGreen) == 4);

    CHECK(r.lines[0].x0 == Approx(392.0f));
    CHECK(r.lines[0].y0 == Approx(180.0f)); // narrow nose

    CHECK(r.lines[0].x1 == Approx(408.0f));

    CHECK(r.lines[2].x0 == Approx(416.0f));
    CHECK(r.lines[2].y0 == Approx(220.0f)); // wide base
}

TEST_CASE("only selected rocket is highlighted", "[draw][editor]")
{
    Rec r;
    Editor e;
    e.model().addRocket(); e.model().addRocket(); // second is selected
    e.draw(r);

    REQUIRE(r.lines.size() == 8);
    CHECK(r.count(kGrayRocket) == 4);
    CHECK(r.count(kGreen) == 4);
    CHECK(r.lines[0].c == kGrayRocket);
}

TEST_CASE("rocket drawing follows rotation", "[draw][editor]")
{
    Rec r;
    Editor e;
    e.model().addRocket();
    e.model().moveSelectedRocket({ 0, 0 }, kPi / 2); // nose points right
    e.draw(r);

    CHECK(r.lines[0].x0 == Approx(420.0f).margin(1e-3));
    CHECK(r.lines[0].y0 == Approx(192.0f).margin(1e-3));
}

// ---------------- Execution::draw ----------------

TEST_CASE("run mode draws terrain back in editor pixels", "[draw][execution]")
{
    Rec r;
    Execution ex;
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
    Rec r;
    Execution ex;
    ex.start(sceneOf({ { 300, 200 }, { 700, 200 } }, false));
    ex.setShowPrediction(false);
    ex.draw(r);

    CHECK(r.lines.size() == 8);
    CHECK(r.count(kGreen) == 4);
    CHECK(r.count(kGrayRocket) == 4);
}

TEST_CASE("rocket under thrust draws a flame scaled by throttle", "[draw][execution]")
{
    Rec r;
    Execution ex;
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
    const float fullLenPx = Physics::kPlumeMetersPerNewton * 25000.0f * Simulation::kPxPerM;
    CHECK(len(f) == Approx(fullLenPx).margin(0.5f)); // 3.75m full thrust, 10 px/m
    CHECK(f.y1 > f.y0);                              // exhaust points down for a rocket pointing up

    Physics::Input half;
    half.throttle = 0.5f;
    ex.sim().step(Simulation::kStep * 1.5f, half);
    Rec r3;
    ex.draw(r3);

    CHECK(len(r3.lines.back()) == Approx(0.5f * fullLenPx).margin(0.5f));
}

TEST_CASE("crashed rocket is red", "[draw][execution]")
{
    Rec r;
    Execution ex;
    ex.start(sceneOf({ { 100, 300 } }, true)); // ~28m drop
    for (int i = 0; i < 160; ++i)
        ex.sim().step(0.05f, {});

    REQUIRE(ex.sim().rockets()[0].is_crashed);
    ex.draw(r);
    CHECK(r.count(kRed) == 4);
    CHECK(r.count(kGreen) == 0);
}

TEST_CASE("main and rotation plumes share one force-to-length scale, for now", "[draw][execution][plume]")
{
    Execution ex;
    ex.start(sceneOf({ { 300, 200 } }, false));
    ex.sim().world().gravity = { 0, 0 };

    Physics::Input main;
    main.throttle = 1.0f;

    ex.sim().step(Simulation::kStep * 1.5f, main);

    Rec r1;
    ex.draw(r1);
    float mainLen = 0;
    for (auto& l : r1.lines)
        if (l.c == kFlame)
            mainLen = len(l);

    CHECK(mainLen == Approx(37.5f).margin(0.5f));

    ex.start(sceneOf({ { 300, 200 } }, false));
    ex.sim().world().gravity = { 0, 0 };

    Physics::Input rot;
    rot.rotate = 1.0f;

    ex.sim().step(Simulation::kStep * 1.5f, rot);

    Rec r2; ex.draw(r2);
    std::vector<Rec::Line> rc;
    for (auto& l : r2.lines)
        if (l.c == kRcs)
            rc.push_back(l);

    REQUIRE(rc.size() == 2);
    CHECK(r2.count(kFlame) == 0);
    CHECK(len(rc[0]) == Approx(4.5f).margin(0.1f));
    CHECK(mainLen / len(rc[0]) == Approx(25000.0f / 3000.0f).epsilon(0.03));
    CHECK(rc[0].y0 < rc[1].y0); // nose nozzle above the tail nozzle
    CHECK(rc[0].x1 < rc[0].x0); // clockwise, nose nozzle exhausts left
    CHECK(rc[1].x1 > rc[1].x0); // tail nozzle exhausts right
}

TEST_CASE("plume length follows the thrust level", "[draw][execution][plume]")
{
    Execution ex;
    ex.start(sceneOf({ { 300, 200 } }, false));
    ex.sim().world().gravity = { 0, 0 };

    float lens[4];
    for (int lvl = 0; lvl < 4; ++lvl)
    {
        Physics::Input in;
        in.throttle = Simulation::engineLevelFraction(lvl);
        ex.sim().step(Simulation::kStep * 1.5f, in);

        Rec r;
        ex.draw(r);
        for (auto& l : r.lines)
            if (l.c == kFlame)
                lens[lvl] = len(l);
    }
    CHECK(lens[1] == Approx(2 * lens[0]).epsilon(0.02));
    CHECK(lens[3] == Approx(4 * lens[0]).epsilon(0.02));
}

TEST_CASE("editor and simulation draw the same hull", "[draw][shape]")
{
    Editor ed;
    ed.model().addRocket();

    RocketSpec s;
    s.halfHeight = 3.0f;
    s.noseHalfWidth = 0.4f;
    s.baseHalfWidth = 2.5f;

    ed.model().setSelectedSpec(s);
    ed.model().moveSelectedRocket({ 0, 0 }, 0.6f);

    Rec re;
    ed.draw(re);

    Execution ex;
    ex.start(ed.model().scene());
    ex.setShowPrediction(false);

    Rec rs;
    ex.draw(rs);

    REQUIRE(re.lines.size() == 4);
    REQUIRE(rs.lines.size() == 4); // t = 0, no plumes, no terrain
    for (int i = 0; i < 4; ++i)
    {
        CHECK(re.lines[i].x0 == Approx(rs.lines[i].x0).margin(0.05f));
        CHECK(re.lines[i].y0 == Approx(rs.lines[i].y0).margin(0.05f));
        CHECK(re.lines[i].x1 == Approx(rs.lines[i].x1).margin(0.05f));
        CHECK(re.lines[i].y1 == Approx(rs.lines[i].y1).margin(0.05f));
    }
}

// ---------------- trajectories ----------------

TEST_CASE("flying rocket leaves a trail", "[draw][execution][trajectory]")
{
    Execution ex;
    ex.start(sceneOf({ { 400, 200 } }, false));
    ex.setShowPrediction(false);

    Rec r0;
    ex.draw(r0);
    CHECK(r0.count(kTrail) == 0); // start point

    for (int i = 0; i < 20; ++i)
        ex.sim().step(0.05f, {}); // 1s fall

    Rec r;
    ex.draw(r);

    int pts = (int)ex.sim().rockets()[0].trail.size();
    REQUIRE(pts > 5);
    CHECK(r.count(kTrail) == pts - 1);

    for (auto& l : r.lines)
    {   // trail is in editor pixels, x same
        if (l.c == kTrail)
        {
            CHECK(l.x0 == Approx(400.0f).margin(0.5f));
        }
    }
}

TEST_CASE("trails switch off", "[draw][execution][trajectory]")
{
    Execution ex;
    ex.start(sceneOf({ { 400, 200 } }, false));
    ex.setShowPrediction(false);

    for (int i = 0; i < 20; ++i)
        ex.sim().step(0.05f, {});

    ex.setShowTrails(false);

    Rec r;
    ex.draw(r);
    CHECK(r.count(kTrail) == 0);
}

TEST_CASE("prediction is drawn dashed for the selected rocket", "[draw][execution][trajectory]")
{
    Execution ex;
    ex.start(sceneOf({ { 400, 200 }, { 700, 200 } }, true));
    ex.setShowTrails(false);

    Rec r;
    ex.draw(r);
    int n = r.count(kPredict);
    CHECK(n > 3);

    auto path = ex.sim().predict(0);
    CHECK(n == (int)(path.size() / 2)); // every other segment
    for (auto& l : r.lines)
        if (l.c == kPredict)
            CHECK(l.x0 == Approx(400.0f).margin(1.0f)); // rocket 0, not rocket 1

    ex.setShowPrediction(false);

    Rec r2;
    ex.draw(r2);
    CHECK(r2.count(kPredict) == 0);
}