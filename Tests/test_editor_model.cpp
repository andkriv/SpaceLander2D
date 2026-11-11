#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "EditorModel/EditorModel.h"

using Catch::Approx;

namespace
{
    void drawLine(EditorModel& m, Vec2 a = { 0, 0 }, Vec2 b = { 10, 0 })
    {
        REQUIRE(m.startDrawing());
        m.addPoint(a);
        m.addPoint(b);
        REQUIRE(m.finish());
    }
}

TEST_CASE("empty", "[editor]")
{
    EditorModel m;

    CHECK_FALSE(m.isDrawing());
    CHECK(m.terrain().polylines.empty());
    CHECK(m.rockets().empty());
    CHECK(m.selectedPolyline() == -1);
    CHECK(m.selectedRocket() == -1);
}

TEST_CASE("points are only added while drawing", "[editor][polyline]")
{
    EditorModel m;

    m.addPoint({ 1, 1 });
    CHECK(m.current().points.empty());

    m.startDrawing();
    m.addPoint({ 1, 1 });
    CHECK(m.current().points.size() == 1);
}

TEST_CASE("start, add points, finish -> named polyline", "[editor][polyline]")
{
    EditorModel m;

    drawLine(m, { 5, 6 }, { 7, 8 });
    REQUIRE(m.terrain().polylines.size() == 1);
    CHECK(m.terrain().polylines[0].name == "Polyline 1");
    CHECK(m.terrain().polylines[0].points[1].y == Approx(8.0f));
    CHECK(m.selectedPolyline() == 0);
    CHECK_FALSE(m.isDrawing());
    CHECK(m.current().points.empty());
}

TEST_CASE("finish needs at least two points", "[editor][polyline]")
{
    EditorModel m;

    CHECK_FALSE(m.finish()); // nothing

    m.startDrawing();
    CHECK_FALSE(m.finish()); // zero points

    m.addPoint({ 1, 1 });
    CHECK_FALSE(m.finish());  // one point

    CHECK(m.isDrawing());
    CHECK(m.terrain().polylines.empty());

    m.addPoint({ 2, 2 });
    CHECK(m.finish());
}

TEST_CASE("startDrawing while drawing keeps the points", "[editor][polyline]")
{
    EditorModel m;

    m.startDrawing();
    m.addPoint({ 1, 1 });
    CHECK_FALSE(m.startDrawing());
    CHECK(m.current().points.size() == 1);
}

TEST_CASE("undo removes last point", "[editor][polyline]")
{
    EditorModel m;

    CHECK_FALSE(m.undoPoint()); // nothing

    m.startDrawing();
    CHECK_FALSE(m.undoPoint()); // nothing to undo

    m.addPoint({ 1, 1 });
    m.addPoint({ 2, 2 });
    CHECK(m.undoPoint());
    REQUIRE(m.current().points.size() == 1);
    CHECK(m.current().points[0].x == 1.0f);
}

TEST_CASE("cancel discards the polyline in progress", "[editor][polyline]")
{
    EditorModel m;

    m.startDrawing();
    m.addPoint({ 1, 1 });
    m.addPoint({ 2, 2 });
    m.cancel();

    CHECK_FALSE(m.isDrawing());
    CHECK(m.current().points.empty());
    CHECK(m.terrain().polylines.empty());
}

TEST_CASE("removing a polyline removes it as a whole", "[editor][polyline]")
{
    EditorModel m;

    drawLine(m);
    drawLine(m);
    drawLine(m);

    CHECK(m.removePolyline(1));
    REQUIRE(m.terrain().polylines.size() == 2);
    CHECK(m.terrain().polylines[0].name == "Polyline 1");
    CHECK(m.terrain().polylines[1].name == "Polyline 3");
    CHECK(m.selectedPolyline() == -1);
    CHECK_FALSE(m.removePolyline(5));
    CHECK_FALSE(m.removePolyline(-1));
    CHECK(m.terrain().polylines.size() == 2);
}

TEST_CASE("polyline names keep counting after removal", "[editor][polyline]")
{
    EditorModel m;

    drawLine(m); drawLine(m);
    m.removePolyline(0);
    drawLine(m);
    CHECK(m.terrain().polylines.back().name == "Polyline 3");
}

TEST_CASE("polyline selection ignores invalid idx", "[editor][polyline]")
{
    EditorModel m;

    drawLine(m); drawLine(m);

    m.selectPolyline(0);
    CHECK(m.selectedPolyline() == 0);
    m.selectPolyline(7);
    m.selectPolyline(-3);
    CHECK(m.selectedPolyline() == 0);
}

TEST_CASE("adding rockets", "[editor][rocket]")
{
    EditorModel m;

    CHECK(m.addRocket() == 0);
    CHECK(m.addRocket() == 1);

    REQUIRE(m.rockets().size() == 2);
    CHECK(m.rockets()[0].name == "Rocket 1");
    CHECK(m.rockets()[1].name == "Rocket 2");
    CHECK(m.rockets()[0].pos.x == Approx(400.0f));
    CHECK(m.rockets()[1].pos.x == Approx(460.0f)); // shift, not overlaping
    CHECK(m.rockets()[0].pos.y == Approx(200.0f));
    CHECK(m.selectedRocket() == 1); // newest selected
}

TEST_CASE("removing rockets", "[editor][rocket]")
{
    EditorModel m;

    m.addRocket();
    m.addRocket();
    m.addRocket();

    CHECK(m.removeRocket(1));
    REQUIRE(m.rockets().size() == 2);
    CHECK(m.rockets()[1].name == "Rocket 3");
    CHECK(m.selectedRocket() == -1);
    CHECK_FALSE(m.removeRocket(9));
    CHECK_FALSE(m.removeRocket(-1));
    m.addRocket();
    CHECK(m.rockets().back().name == "Rocket 4");
}

TEST_CASE("rocket selection ignores invalid indices", "[editor][rocket]")
{
    EditorModel m;

    m.addRocket();
    m.addRocket();

    m.selectRocket(0);
    CHECK(m.selectedRocket() == 0);
    m.selectRocket(5);
    CHECK(m.selectedRocket() == 0);
}

TEST_CASE("moving the selected rocket", "[editor][rocket]")
{
    EditorModel m;

    CHECK_FALSE(m.moveSelectedRocket({ 1, 1 }, 0.1f)); // nothing selected

    m.addRocket();
    m.addRocket();

    m.selectRocket(0);
    CHECK(m.moveSelectedRocket({ 10, -5 }, 0.5f));
    CHECK(m.rockets()[0].pos.x == Approx(410.0f));
    CHECK(m.rockets()[0].pos.y == Approx(195.0f));
    CHECK(m.rockets()[0].angle == Approx(0.5f));
    CHECK(m.rockets()[1].pos.x == Approx(460.0f)); // others untouched
    CHECK(m.rockets()[1].angle == 0.0f);
}

TEST_CASE("setScene replaces everything and resets state", "[editor][scene]")
{
    EditorModel src;
    drawLine(src); drawLine(src);
    src.addRocket();
    Scene s = src.scene();

    EditorModel m;
    m.startDrawing();
    m.addPoint({ 1, 1 }); // in-progress dropped
    m.addRocket(); m.addRocket(); m.addRocket();
    m.setScene(s);

    CHECK_FALSE(m.isDrawing());
    CHECK(m.current().points.empty());
    CHECK(m.terrain().polylines.size() == 2);
    CHECK(m.rockets().size() == 1);
    CHECK(m.selectedPolyline() == 0);
    CHECK(m.selectedRocket() == 0);

    drawLine(m); // counters continue after import
    CHECK(m.terrain().polylines.back().name == "Polyline 3");
    m.addRocket();
    CHECK(m.rockets().back().name == "Rocket 2");
}

TEST_CASE("setScene with an empty scene clears selection", "[editor][scene]")
{
    EditorModel m;

    drawLine(m);
    m.addRocket();
    m.setScene(Scene{});

    CHECK(m.selectedPolyline() == -1);
    CHECK(m.selectedRocket() == -1);
    CHECK(m.terrain().polylines.empty());
    CHECK(m.rockets().empty());
}

TEST_CASE("scene() snapshots terrain and rockets", "[editor][scene]")
{
    EditorModel m;

    drawLine(m, { 1, 2 }, { 3, 4 });
    m.addRocket();

    Scene s = m.scene();
    m.removePolyline(0); // later edits don't change the snapshot

    CHECK(s.terrain.polylines.size() == 1);
    CHECK(s.rockets.size() == 1);
}

TEST_CASE("each rocket has its own spec", "[editor][spec]")
{
    EditorModel m;
    m.addRocket();
    m.addRocket();
    m.selectRocket(0);

    RocketSpec s;
    s.dryMass = 1200.0f;
    s.maxThrust = 40000.0f;

    CHECK(m.setSelectedSpec(s));
    CHECK(m.rockets()[0].spec.dryMass == Approx(1200.0f));
    CHECK(m.rockets()[1].spec.dryMass == Approx(800.0f)); // untouched
    CHECK(m.selectedSpec() == &m.rockets()[0].spec);
}

TEST_CASE("setSelectedSpec needs a selection and clamps bad values", "[editor][spec]")
{
    EditorModel m;
    CHECK_FALSE(m.setSelectedSpec(RocketSpec{}));
    CHECK(m.selectedSpec() == nullptr);

    m.addRocket();
    RocketSpec s;
    s.dryMass = -5;
    s.fuelCapacity = 100;
    s.fuelStart = 900;

    s.halfHeight = 1.0f;
    s.tankHeight = 9.0f;
    s.rcsArm = 7.0f;
    s.isp = 0;
    m.setSelectedSpec(s);

    const RocketSpec& r = *m.selectedSpec();
    CHECK(r.dryMass >= 1.0f);
    CHECK(r.fuelStart == Approx(100.0f));
    CHECK(r.tankHeight <= 2.0f * r.halfHeight);
    CHECK(r.rcsArm <= r.halfHeight);
    CHECK(r.isp >= 1.0f);
}