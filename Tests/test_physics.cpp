#include <cmath>
#include <array>
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

    World flatGround(float y = 10.0f)
    {
        World w;
        w.terrain.push_back({ { -50, y }, { 50, y } });
        return w;
    }

    float lowestY(const RocketSpec& s, const BodyState& b)
    {
        float m = -1e30f;
        for (Vec2 p : hullLocal(s))
            m = std::max(m, toWorld(s, b, p).y);
        return m;
    }

    float rightmostX(const RocketSpec& s, const BodyState& b)
    {
        float m = -1e30f;
        for (Vec2 p : hullLocal(s))
            m = std::max(m, toWorld(s, b, p).x);
        return m;
    }

    float speed(const BodyState& b)
    {
        return std::sqrt(b.vel.x * b.vel.x + b.vel.y * b.vel.y);
    }

    std::array<Vec2, 4> worldPoly(const RocketSpec& s, const BodyState& b)
    {
        std::array<Vec2, 4> out; auto h = hullLocal(s);
        for (int i = 0; i < 4; ++i)
            out[i] = toWorld(s, b, h[i]);

        return out;
    }
    // >0 when p is inside the convex polygon
    float insideDepth(const std::array<Vec2, 4>& P, Vec2 p)
    {
        float area = 0;
        for (int i = 0; i < 4; ++i)
            area += cross(P[i], P[(i + 1) % 4]);

        float sg = area > 0 ? 1.0f : -1.0f;
        float d = 1e30f;
        for (int i = 0; i < 4; ++i)
        {
            Vec2 a = P[i];
            Vec2 e = P[(i + 1) % 4] - a;
            d = std::min(d, sg * cross(e, p - a) / std::sqrt(dot(e, e)));
        }
        return d;
    }
    float deepestVertex(const std::array<Vec2, 4>& P, const std::array<Vec2, 4>& Q)
    {
        float m = -1e30f;
        for (Vec2 p : P)
            m = std::max(m, insideDepth(Q, p));

        return m;
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

TEST_CASE("massProps: full tank", "[mass]")
{
    RocketSpec s;
    MassProps m = massProps(s, s.fuelCapacity);

    CHECK(m.mass == Approx(1500.0f));
    CHECK(m.com.y == Approx(0.23333f).epsilon(0.001));   // fuel sits low, COM moves toward the engine
    CHECK(m.inertia == Approx(2183.33f).epsilon(0.001)); // incl. parallel-axis terms
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
    BodyState empty = makeState(s, { 0, 0 }, 0, 0.5f);

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

TEST_CASE("rot thrusters are couple: torque = 2 F arm", "[rcs]")
{
    RocketSpec s;
    World w = zeroG();

    Input in;
    in.rotate = 0.5f;

    BodyState b = makeState(s, { 0, 0 }, 0);
    StepInfo i = step(s, b, in, w, kDt);

    float torque = 2.0f * (0.5f * s.rcsThrust) * s.rcsArm; // 5400 [N*m]

    CHECK(b.angVel == Approx(torque / 2183.33f * kDt).epsilon(0.002));
    CHECK(i.rcsForce == Approx(1500.0f));
    CHECK(i.rcsDir == 1);
    CHECK(i.thrust == 0.0f);

    CHECK(i.acc.x == Approx(0.0f).margin(1e-3)); // no net force
    CHECK(i.acc.y == Approx(0.0f).margin(1e-3));
}

TEST_CASE("rot dir follows the sign", "[rcs]")
{
    RocketSpec s;
    World w = zeroG();

    BodyState cw = makeState(s, { 0, 0 }, 0);
    BodyState ccw = cw;

    Input in;
    in.rotate = 1.0f;
    step(s, cw, in, w, kDt);

    in.rotate = -1.0f;
    StepInfo i = step(s, ccw, in, w, kDt);

    CHECK(cw.angVel > 0.0f);
    CHECK(ccw.angVel == Approx(-cw.angVel).epsilon(1e-3));
    CHECK(i.rcsDir == -1);
}

TEST_CASE("rot thrusters burn fuel and stop when no fuel", "[rcs][fuel]")
{
    RocketSpec s;
    World w = zeroG();

    Input in;
    in.rotate = 1.0f;

    BodyState b = makeState(s, { 0, 0 }, 0);

    float f0 = b.fuel;
    for (int i = 0; i < 240; ++i)
        step(s, b, in, w, kDt);

    CHECK(f0 - b.fuel == Approx(2.0f * s.rcsThrust / (s.isp * kG0)).epsilon(0.01));

    BodyState e = makeState(s, { 0, 0 }, 0, 0.0f);
    StepInfo i = step(s, e, in, w, kDt);

    CHECK(i.rcsForce == 0.0f);
    CHECK(e.angVel == 0.0f);
}

TEST_CASE("nozzle geometry and plume scale", "[rcs]")
{
    RocketSpec s;
    auto cw = rcsNozzles(s, 1);

    CHECK(cw[0].pos.y == Approx(-s.rcsArm));  CHECK(cw[0].exhaust.x == -1.0f); // nose, exhaust left
    CHECK(cw[1].pos.y == Approx(s.rcsArm));   CHECK(cw[1].exhaust.x == 1.0f);  // tail, exhaust right
    CHECK(mainNozzle(s).pos.y == Approx(s.halfHeight));
    CHECK(plumeLength(25000.0f) / plumeLength(3000.0f) == Approx(25000.0f / 3000.0f));
}

TEST_CASE("each spec changes the dynamics", "[spec]")
{
    World w = zeroG();

    Input in;
    in.throttle = 1.0f;

    RocketSpec heavy;
    RocketSpec strong;
    heavy.dryMass = 2400.0f;
    strong.maxThrust = 50000.0f;

    BodyState a = makeState(heavy, { 0, 0 }, 0);
    BodyState b = makeState(RocketSpec{}, { 0, 0 }, 0);
    BodyState c = makeState(strong, { 0, 0 }, 0);

    StepInfo ia = step(heavy, a, in, w, kDt);
    StepInfo ib = step(RocketSpec{}, b, in, w, kDt);
    StepInfo ic = step(strong, c, in, w, kDt);

    CHECK(std::fabs(ia.acc.y) < std::fabs(ib.acc.y));
    CHECK(std::fabs(ic.acc.y) > std::fabs(ib.acc.y));
}

TEST_CASE("rockets exchange momentum when collide", "[collision][rockets]")
{
    RocketSpec a;
    RocketSpec b;
    b.fuelStart = 0.0f; // b is lighter, 800 [kg] vs 1500 [kg]

    World w = zeroG();

    BodyState A = makeState(a, { 0, 0 }, 0);
    BodyState B = makeState(b, { 5, 0 }, 0, 0.0f);
    A.vel = { 5, 0 };

    float mA = 1500.0f, mB = 800.0f;
    std::vector<RocketRef> refs{ RocketRef{ &a, &A, {}, {} }, RocketRef{ &b, &B, {}, {} } };
    for (int i = 0; i < 480; ++i)
        stepAll(refs, w, kDt);

    CHECK(mA * A.vel.x + mB * B.vel.x == Approx(mA * 5.0f).epsilon(0.01)); // momentum conserved
    CHECK(B.vel.x > 2.0f);                                                 // B was pushed
    CHECK(A.vel.x < 4.5f);                                                 // A was slowed
}

TEST_CASE("rockets never overlap in head-on collision", "[collision][rockets]")
{
    RocketSpec s;
    World w = zeroG();

    BodyState A = makeState(s, { 0, 0 }, 0);
    BodyState B = makeState(s, { 8, 0 }, 0);
    A.vel = { 5, 0 };
    B.vel = { -5, 0 };

    std::vector<RocketRef> refs{ RocketRef{ &s, &A, {}, {} }, RocketRef{ &s, &B, {}, {} } };
    float worst = -1e30f;
    for (int i = 0; i < 240 * 3; ++i)
    {
        stepAll(refs, w, kDt);
        auto PA = worldPoly(s, A);
        auto PB = worldPoly(s, B);

        worst = std::max({ worst, deepestVertex(PA, PB), deepestVertex(PB, PA) });
    }
    CHECK(worst < 0.1f);
    CHECK(A.vel.x < 1.0f); // bounce apart
    CHECK(B.vel.x > -1.0f);
}

TEST_CASE("thrusting rocket pushes another one", "[collision][rockets][thrust]")
{
    RocketSpec s;
    World w = zeroG();

    BodyState A = makeState(s, { 0, 0 }, kPi / 2);
    BodyState B = makeState(s, { 10, 0 }, 0); // A nose points at B

    Input push;
    push.throttle = 1.0f;

    std::vector<RocketRef> refs{ RocketRef{ &s, &A, push, {} }, RocketRef{ &s, &B, {}, {} } };
    for (int i = 0; i < 240 * 3; ++i)
        stepAll(refs, w, kDt);

    CHECK(B.vel.x > 3.0f);
    CHECK(B.pos.x > 10.0f);
}

TEST_CASE("rockets still rest on the ground next to each other", "[collision][rockets]")
{
    RocketSpec s;
    World w = flatGround(10);

    BodyState A = makeState(s, { 0, 7.5f }, 0);
    BodyState B = makeState(s, { 3.6f, 7.5f }, 0);

    std::vector<RocketRef> refs{ RocketRef{ &s, &A, {}, {} }, RocketRef{ &s, &B, {}, {} } };
    for (int i = 0; i < 240 * 5; ++i)
        stepAll(refs, w, kDt);

    CHECK(lowestY(s, A) - 10.0f < 0.2f);
    CHECK(lowestY(s, B) - 10.0f < 0.2f);
    CHECK(std::max(deepestVertex(worldPoly(s, A), worldPoly(s, B)),
                   deepestVertex(worldPoly(s, B), worldPoly(s, A))) < 0.1f);
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

// ---------------- collision ----------------

TEST_CASE("rocket lands and rests on flat ground", "[collision]")
{
    RocketSpec s;
    World w = flatGround(10);
    BodyState b = makeState(s, { 0, 7.5f }, 0);
    StepInfo info;

    float worst = -1e30f;
    for (int i = 0; i < 240 * 5; ++i)
    {
        info = step(s, b, Input{}, w, kDt);
        worst = std::max(worst, lowestY(s, b) - 10.0f);
    }

    CHECK(worst < 0.15f);                 // not sinks in
    CHECK(lowestY(s, b) - 10.0f < 0.02f); // settled at ~kSlop
    CHECK(lowestY(s, b) > 9.95f);         // not hovering
    CHECK(speed(b) < 0.1f);
    CHECK(std::fabs(b.angVel) < 0.1f);
    CHECK(std::fabs(b.angle) < 0.02f);
    CHECK(info.touching);
}

TEST_CASE("fast impact never tunnels through the terrain", "[collision]")
{
    RocketSpec s;
    World w = flatGround(10);
    BodyState b = makeState(s, { 0, -11.8f }, 0); // ~20m drop

    float worst = -1e30f;
    float maxImpact = 0.0f;
    for (int i = 0; i < 240 * 6; ++i)
    {
        StepInfo info = step(s, b, Input{}, w, kDt);
        worst = std::max(worst, lowestY(s, b) - 10.0f);
        maxImpact = std::max(maxImpact, info.impactSpeed);
    }

    CHECK(worst < 0.15f);
    CHECK(maxImpact > 15.0f); // ~19.8m/s expected
    CHECK(maxImpact < 25.0f);
    CHECK(speed(b) < 0.2f);
}

TEST_CASE("restitution controls the bounce", "[collision]")
{
    RocketSpec s;
    auto run = [&](float e, float& maxImpact)
    {
        World w = flatGround(10);
        w.restitution = e;
        BodyState b = makeState(s, { 0, 3.2f }, 0); // ~5 m drop

        float maxUp = 0.0f;
        maxImpact = 0.0f;
        for (int i = 0; i < 240 * 2; ++i)
        {
            StepInfo info = step(s, b, Input{}, w, kDt);
            maxImpact = std::max(maxImpact, info.impactSpeed);
            maxUp = std::max(maxUp, -b.vel.y);
        }
        return maxUp;
    };
    float impact = 0;
    float bouncy = run(0.5f, impact);

    CHECK(bouncy > 0.3f * impact);
    CHECK(bouncy < 0.7f * impact);
    CHECK(run(0.0f, impact) < 0.1f);
}

TEST_CASE("tilted rocket settles flat on its base", "[collision]")
{
    RocketSpec s;
    World w = flatGround(10);
    BodyState b = makeState(s, { 0, 7.0f }, 0.3f);

    float worst = -1e30f;
    for (int i = 0; i < 240 * 15; ++i)
    {
        step(s, b, Input{}, w, kDt);
        worst = std::max(worst, lowestY(s, b) - 10.0f);
    }

    CHECK(worst < 0.2f);
    CHECK(std::fabs(b.angle) < 0.05f);
    CHECK(speed(b) < 0.1f);
}

TEST_CASE("vertical wall stops sideways motion", "[collision]")
{
    RocketSpec s;
    World w = zeroG();
    w.terrain.push_back({ { 10, -100 }, { 10, 100 } });
    BodyState b = makeState(s, { 5, 0 }, 0);
    b.vel = { 10, 0 };

    float worst = -1e30f;
    for (int i = 0; i < 240 * 3; ++i)
    {
        step(s, b, Input{}, w, kDt);
        worst = std::max(worst, rightmostX(s, b) - 10.0f);
    }
    CHECK(worst < 0.15f);
}

TEST_CASE("friction holds the rocket on a gentle slope, not a steep one", "[collision]")
{
    RocketSpec s;
    SECTION("gentle slope (tan = 0.1 < mu)")
    {
        World w;
        w.terrain.push_back({ { -50, 5 }, { 50, 15 } });
        BodyState b = makeState(s, { 0, 3 }, 0);

        for (int i = 0; i < 240 * 10; ++i)
            step(s, b, Input{}, w, kDt);

        CHECK(speed(b) < 0.5f);
    }
    SECTION("45 degree slope (tan = 1 > mu) slides downhill")
    {
        World w;
        w.terrain.push_back({ { -50, -50 }, { 50, 50 } });
        BodyState b = makeState(s, { 0, -6 }, 0);

        for (int i = 0; i < 240 * 4; ++i)
            step(s, b, Input{}, w, kDt);

        CHECK(b.pos.x > 3.0f);
    }
}

TEST_CASE("thrust lifts a resting rocket off the ground", "[collision]")
{
    RocketSpec s;
    World w = flatGround(10);
    BodyState b = makeState(s, { 0, 8.0f }, 0);

    for (int i = 0; i < 240 * 3; ++i)
        step(s, b, Input{}, w, kDt);
    REQUIRE(speed(b) < 0.1f);

    Input in; in.throttle = 1.0f;
    StepInfo info;
    for (int i = 0; i < 240; ++i)
        info = step(s, b, in, w, kDt);

    CHECK(b.vel.y < -3.0f); // net upward: 16.7 - 9.81 m/s^2
    CHECK_FALSE(info.touching);
}

TEST_CASE("degenerate terrain does not break the step", "[collision]")
{
    RocketSpec s;
    World w;
    w.terrain.push_back({ { 3, 3 }, { 3, 3 } }); // zero length
    BodyState b = makeState(s, { 3, 3 }, 0);

    step(s, b, Input{}, w, kDt);

    CHECK(std::isfinite(b.pos.x));
    CHECK(std::isfinite(b.pos.y));
}

// ---------------- robustness ----------------

TEST_CASE("long run with mixed input stays finite and bounded", "[robustness]")
{
    RocketSpec s;
    World w = flatGround(10);
    BodyState b = makeState(s, { 0, 7.0f }, 0);

    bool ok = true;
    for (int i = 0; i < 20000; ++i)
    {
        Input in;
        in.throttle = ((i / 300) % 2) ? 1.0f : 0.0f;
        in.rotate = std::sin(i * 0.01f);

        step(s, b, in, w, kDt);
        ok = ok && std::isfinite(b.pos.x) && std::isfinite(b.pos.y)
                && std::isfinite(b.vel.x) && std::isfinite(b.vel.y)
                && std::isfinite(b.angle) && std::isfinite(b.angVel)
                && b.fuel >= 0.0f         && b.fuel <= s.fuelCapacity
                && std::fabs(b.angle) <= kPi + 1e-4f;
    }
    CHECK(ok);
}

TEST_CASE("the step is deterministic", "[robustness]")
{
    RocketSpec s;
    World w = flatGround(10);
    BodyState a = makeState(s, { 0, 2 }, 0.2f), b = a;

    for (int i = 0; i < 2000; ++i)
    {
        Input in;
        in.throttle = (i % 400 < 100) ? 1.0f : 0.0f;
        in.rotate = 0.3f;

        step(s, a, in, w, kDt);
        step(s, b, in, w, kDt);
    }

    CHECK(a.pos.x == b.pos.x);  CHECK(a.pos.y == b.pos.y);
    CHECK(a.vel.x == b.vel.x);  CHECK(a.vel.y == b.vel.y);
    CHECK(a.angle == b.angle);  CHECK(a.fuel == b.fuel);
}