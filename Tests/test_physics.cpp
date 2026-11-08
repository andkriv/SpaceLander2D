#include <cmath>
#include <algorithm>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Physics/Physics.h"

using Catch::Approx;
using namespace Physics;

namespace
{
    constexpr float kPi = 3.14159265f;
}

// ---------------- mass, fuel, inertia ----------------

TEST_CASE("massProps: empty tank", "[mass]")
{
    RocketSpec s;
    MassProps m = massProps(s, 0.0f);

    CHECK(m.mass == Approx(800.0f));
    CHECK(m.com.y == Approx(0.0f).margin(1e-6));
    // dry I: m/12 * ((nose+base)^2 + H^2) = 800/12 * (2.4^2 + 4^2)
    CHECK(m.inertia == Approx(1450.67f).epsilon(0.001));
}

TEST_CASE("massProps: half tank", "[mass]")
{
    RocketSpec s;
    MassProps m = massProps(s, 350.0f);

    CHECK(m.mass == Approx(1150.0f));
    CHECK(m.com.y == Approx(0.380435f).epsilon(0.001));
}

TEST_CASE("massProps: mass and inertia up with fuel", "[mass]")
{
    RocketSpec s;
    float prevM = -1;
    float prevI = -1;

    for (float f = 0; f <= s.fuelCapacity; f += 50.0f)
    {
        MassProps m = massProps(s, f);

        CHECK(m.mass > prevM);
        CHECK(m.inertia > prevI);

        prevM = m.mass;
        prevI = m.inertia;
    }
}

TEST_CASE("massProps clamps fuel to [0, capacity]", "[mass]")
{
    RocketSpec s;

    CHECK(massProps(s, -50.0f).mass == Approx(s.dryMass));
    CHECK(massProps(s, 1e6f).mass == Approx(s.dryMass + s.fuelCapacity));
}

TEST_CASE("hullLocal is the trapezoid", "[geometry]")
{
    RocketSpec s;
    auto h = hullLocal(s);

    CHECK(h[0].x == Approx(-0.8f)); CHECK(h[0].y == Approx(-2.0f));
    CHECK(h[1].x == Approx(0.8f));  CHECK(h[1].y == Approx(-2.0f));
    CHECK(h[2].x == Approx(1.6f));  CHECK(h[2].y == Approx(2.0f));
    CHECK(h[3].x == Approx(-1.6f)); CHECK(h[3].y == Approx(2.0f));
}

TEST_CASE("rotate turns cw on screen", "[geometry]")
{
    Vec2 r = rotate({ 0, -1 }, kPi / 2); // nose: up -> right

    CHECK(r.x == Approx(1.0f).margin(1e-5));
    CHECK(r.y == Approx(0.0f).margin(1e-5));
}

TEST_CASE("makeState: hull center through toWorld", "[state]")
{
    RocketSpec s;

    for (float angle : { 0.0f, 0.7f, -2.0f })
    {
        for (float frac : { 0.0f, 0.5f, 1.0f })
        {
            BodyState b = makeState(s, { 12, -5 }, angle, frac);
            Vec2 c = toWorld(s, b, { 0, 0 });

            CHECK(c.x == Approx(12.0f).margin(1e-4));
            CHECK(c.y == Approx(-5.0f).margin(1e-4));
            CHECK(b.fuel == Approx(frac * s.fuelCapacity));
        }
    }
}

// ---------------- terrain conversion ----------------

TEST_CASE("World::setTerrain builds scaled segments", "[terrain]")
{
    Terrain t;
    Polyline a;
    a.points = { { 0, 0 }, { 100, 0 }, { 100, 50 } };

    Polyline b;
    b.points = { { 10, 10 } }; // one point, no segment

    Polyline c;
    c.points = { { 0, 100 }, { 200, 100 } };

    t.polylines = { a, b, c };

    World w;
    w.setTerrain(t, 0.1f);

    REQUIRE(w.terrain.size() == 3);
    CHECK(w.terrain[1].b.x == Approx(10.0f));
    CHECK(w.terrain[1].b.y == Approx(5.0f));
    CHECK(w.terrain[2].a.y == Approx(10.0f));
    CHECK(w.terrain[2].b.x == Approx(20.0f));

    w.setTerrain(Terrain{}, 0.1f);
    CHECK(w.terrain.empty());
}