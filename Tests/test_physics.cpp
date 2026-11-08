#include <cmath>
#include <algorithm>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Physics/Physics.h"

using Catch::Approx;
using namespace Physics;

namespace
{
    constexpr float kDt = 1.0f / 240.0f; // physics update rate, with kSkin - 0.02*240 = 4.8 m/s, g*dt=0.04 m/s
    constexpr float kPi = 3.14159265f;   // pi

    World zeroG()
    {
        World w;
        w.gravity = { 0, 0 };
        return w;
    }
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

// ---------------- gravity ----------------

TEST_CASE("free fall matches kinematics", "[gravity]")
{
    RocketSpec s;
    World w;
    Input in;
    BodyState b = makeState(s, { 0, 0 }, 0);

    float y0 = b.pos.y;
    for (int i = 0; i < 240; ++i)
        step(s, b, in, w, kDt);

    CHECK(b.vel.y == Approx(9.81f).epsilon(0.001));
    CHECK(b.pos.y - y0 == Approx(0.5f * 9.81f).epsilon(0.01));
    CHECK(b.vel.x == 0.0f);
}

TEST_CASE("gravity's vector", "[gravity]")
{
    RocketSpec s;
    Input in;

    SECTION("sideways")
    {
        World w;
        w.gravity = { 5, 0 };
        BodyState b = makeState(s, { 0, 0 }, 0);

        for (int i = 0; i < 240; ++i)
            step(s, b, in, w, kDt);

        CHECK(b.vel.x == Approx(5.0f).epsilon(0.001));
        CHECK(b.vel.y == Approx(0.0f).margin(1e-4));
    }
    SECTION("upward")
    {
        World w;
        w.gravity = { 0, -9.81f };
        BodyState b = makeState(s, { 0, 0 }, 0);

        for (int i = 0; i < 240; ++i)
            step(s, b, in, w, kDt);

        CHECK(b.vel.y < -9.0f);
    }
    SECTION("diagonal keeps the ratio")
    {
        World w;
        w.gravity = { 3, 4 };
        BodyState b = makeState(s, { 0, 0 }, 0);

        for (int i = 0; i < 240; ++i)
            step(s, b, in, w, kDt);

        CHECK(b.vel.x / b.vel.y == Approx(0.75f).epsilon(0.001));
    }
}

TEST_CASE("gravity acceleration is independent of mass", "[gravity]")
{
    RocketSpec s;
    World w;
    Input in;
    BodyState light = makeState(s, { 0, 0 }, 0, 0.0f);
    BodyState heavy = makeState(s, { 0, 0 }, 0, 1.0f);

    for (int i = 0; i < 120; ++i)
    {
        step(s, light, in, w, kDt);
        step(s, heavy, in, w, kDt);
    }

    CHECK(light.vel.y == Approx(heavy.vel.y).epsilon(1e-4));
}

TEST_CASE("free flight -> no contact", "[gravity]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    BodyState b = makeState(s, { 0, 0 }, 0);
    StepInfo info = step(s, b, in, w, kDt);

    CHECK_FALSE(info.touching);
    CHECK(info.impactSpeed == 0.0f);
}

// ---------------- thrust ----------------

TEST_CASE("thrust acceleration is thrust / mass, measured in info.acc", "[thrust]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState b = makeState(s, { 0, 0 }, 0);
    StepInfo info = step(s, b, in, w, kDt);

    CHECK(info.thrust == Approx(25000.0f));
    CHECK(info.mass == Approx(1500.0f));
    CHECK(info.acc.y == Approx(-25000.0f / 1500.0f).epsilon(0.001)); // nose up = -y
    CHECK(info.acc.x == Approx(0.0f).margin(1e-3));
}

TEST_CASE("thrust follows the nose direction", "[thrust]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;

    SECTION("pointing right")
    {
        BodyState b = makeState(s, { 0, 0 }, kPi / 2);
        for (int i = 0; i < 60; ++i)
            step(s, b, in, w, kDt);

        CHECK(b.vel.x > 3.0f);
        CHECK(std::fabs(b.vel.y) < 0.01f);
    }
    SECTION("pointing down")
    {
        BodyState b = makeState(s, { 0, 0 }, kPi);
        for (int i = 0; i < 60; ++i)
            step(s, b, in, w, kDt);

        CHECK(b.vel.y > 3.0f);
    }
    SECTION("45 degrees splits into components")
    {
        BodyState b = makeState(s, { 0, 0 }, kPi / 4);
        for (int i = 0; i < 60; ++i)
            step(s, b, in, w, kDt);

        CHECK(b.vel.x > 0.0f);
        CHECK(b.vel.y < 0.0f);
        CHECK(b.vel.x == Approx(-b.vel.y).epsilon(0.01));
    }
}

TEST_CASE("throttle is clamped to [0, 1]", "[thrust]")
{
    RocketSpec s;
    World w = zeroG();
    BodyState b = makeState(s, { 0, 0 }, 0);
    Input in;
    in.throttle = 5.0f;

    CHECK(step(s, b, in, w, kDt).thrust == Approx(s.maxThrust));
    in.throttle = -1.0f;
    CHECK(step(s, b, in, w, kDt).thrust == 0.0f);
}

TEST_CASE("main engine on the axis produces no torque", "[thrust]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState b = makeState(s, { 0, 0 }, 0.4f);
    for (int i = 0; i < 240; ++i)
        step(s, b, in, w, kDt);

    CHECK(b.angVel == Approx(0.0f).margin(1e-4));
}

// -------------- fuel burn ---------------

TEST_CASE("fuel burn rate is thrust / (Isp * g0)", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 0.5f;
    BodyState b = makeState(s, { 0, 0 }, 0);

    float f0 = b.fuel;
    for (int i = 0; i < 240; ++i) // 1s
        step(s, b, in, w, kDt); 
    float expected = 0.5f * s.maxThrust / (s.isp * kG0);

    CHECK(f0 - b.fuel == Approx(expected).epsilon(0.001));
}

TEST_CASE("no fuel is burned at zero throttle", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    BodyState b = makeState(s, { 0, 0 }, 0);

    float f0 = b.fuel;
    for (int i = 0; i < 100; ++i)
        step(s, b, in, w, kDt);

    CHECK(b.fuel == f0);
}

TEST_CASE("mass decreases while burning", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState b = makeState(s, { 0, 0 }, 0);

    float prev = 1e30f;
    for (int i = 0; i < 200; ++i)
    {
        StepInfo info = step(s, b, in, w, kDt);
        CHECK(info.mass <= prev);
        prev = info.mass;
    }

    CHECK(prev < 1500.0f);
}

TEST_CASE("same thrust accelerates a lighter rocket more", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState full = makeState(s, { 0, 0 }, 0, 1.0f);
    BodyState empty = makeState(s, { 0, 0 }, 0, 0.1f);
    StepInfo a = step(s, full, in, w, kDt);
    StepInfo e = step(s, empty, in, w, kDt);

    CHECK(std::fabs(e.acc.y) > std::fabs(a.acc.y));
}

TEST_CASE("dV = Tsiolkovsky eq", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState b = makeState(s, { 0, 0 }, 0);

    float m0 = s.dryMass + b.fuel;
    for (int i = 0; i < 2400; ++i) // 10s
        step(s, b, in, w, kDt);
    float m1 = s.dryMass + b.fuel;
    float dV = s.isp * kG0 * std::log(m0 / m1);

    CHECK(-b.vel.y == Approx(dV).epsilon(0.005));
}

TEST_CASE("empty tank -> no thrust", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState b = makeState(s, { 0, 0 }, 0, 0.0f);
    StepInfo info = step(s, b, in, w, kDt);

    CHECK(info.thrust == 0.0f);
    CHECK(b.vel.y == 0.0f);
    CHECK(b.fuel == 0.0f);
}

TEST_CASE("thrust is scaled down (tank runs dry)", "[fuel]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.throttle = 1.0f;
    BodyState b = makeState(s, { 0, 0 }, 0);
    b.fuel = 0.01f;

    float fullBurn = s.maxThrust / (s.isp * kG0) * kDt;
    StepInfo info = step(s, b, in, w, kDt);

    CHECK(b.fuel == 0.0f);
    CHECK(info.thrust > 0.0f);
    CHECK(info.thrust < s.maxThrust);
    CHECK(info.thrust == Approx(s.maxThrust * 0.01f / fullBurn).epsilon(0.001));
}

// ---------------- rot, I ----------------

TEST_CASE("rotation input applies torque / inertia", "[rotation]")
{
    RocketSpec s;
    World w = zeroG();
    BodyState b = makeState(s, { 0, 0 }, 0);
    Input in;
    in.rotate = 1.0f;

    step(s, b, in, w, kDt);
    CHECK(b.angVel == Approx(s.maxTorque / 2183.33f * kDt).epsilon(0.001));
    CHECK(b.angVel > 0.0f);

    BodyState c = makeState(s, { 0, 0 }, 0);
    in.rotate = -1.0f;
    step(s, c, in, w, kDt);

    CHECK(c.angVel == Approx(-b.angVel).epsilon(1e-4));
}

TEST_CASE("rotation input is clamped, zero = no torque", "[rotation]")
{
    RocketSpec s;
    World w = zeroG();
    BodyState a = makeState(s, { 0, 0 }, 0), b = makeState(s, { 0, 0 }, 0);
    Input big;
    big.rotate = 3.0f;

    Input one;
    one.rotate = 1.0f;

    step(s, a, big, w, kDt);
    step(s, b, one, w, kDt);

    CHECK(a.angVel == Approx(b.angVel));

    BodyState c = makeState(s, { 0, 0 }, 0);
    step(s, c, Input{}, w, kDt);

    CHECK(c.angVel == 0.0f);
}

TEST_CASE("heavier rocket turns slower (inertia)", "[rotation]")
{
    RocketSpec s;
    World w = zeroG();
    Input in;
    in.rotate = 1.0f;

    BodyState full = makeState(s, { 0, 0 }, 0, 1.0f);
    BodyState empty = makeState(s, { 0, 0 }, 0, 0.0f);

    step(s, full, in, w, kDt);
    step(s, empty, in, w, kDt);

    CHECK(full.angVel < empty.angVel);
}

TEST_CASE("angular velocity persists with no torque", "[rotation]")
{
    RocketSpec s;
    World w = zeroG();
    BodyState b = makeState(s, { 0, 0 }, 0);
    b.angVel = 0.5f;

    for (int i = 0; i < 240; ++i)
        step(s, b, Input{}, w, kDt);

    CHECK(b.angVel == Approx(0.5f));
}

TEST_CASE("angle is wrapped to [-pi, pi]", "[rotation]")
{
    RocketSpec s;
    World w = zeroG();
    BodyState b = makeState(s, { 0, 0 }, 0);
    b.angVel = 10.0f;

    for (int i = 0; i < 480; ++i)
    {
        step(s, b, Input{}, w, kDt);
        REQUIRE(std::fabs(b.angle) <= kPi + 1e-4f);
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