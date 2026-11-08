#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "Simulation/Simulation.h"

using Catch::Approx;

namespace
{
    Scene scene(std::vector<Vec2> rocketsPx, bool floor)
    {
        Scene s;
        if (floor)
        {
            Polyline pl;
            pl.name = "floor";
            pl.points = { { 0, 600 }, { 2000, 600 } }; // 10px/m, y = 60m 
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

    // hull center (px) -> base drop meters above the floor
    Vec2 dropAt(float xPx, float drop)
    {
        return { xPx, (60.0f - 1.7667f - drop) * 10.0f };
    }

    void run(Simulation& sim, float seconds, Physics::Input in = {})
    {
        int frames = (int)std::lround(seconds / 0.05f);
        for (int i = 0; i < frames; ++i)
            sim.step(0.05f, in);
    }
}

TEST_CASE("two rockets", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 }, { 500, 200 } }, false));
    REQUIRE(sim.rockets().size() == 2);
    CHECK(sim.rockets()[0].name == "R1");
    CHECK(sim.rockets()[1].name == "R2");
    CHECK(sim.selectedRocket() == 0);
    CHECK_FALSE(sim.rockets()[0].is_crashed);
}

TEST_CASE("empty sim", "[simulation]")
{
    Simulation sim;
    sim.start(scene({}, true));
    CHECK(sim.rockets().empty());
    CHECK(sim.selectedRocket() == -1);
    CHECK(sim.step(0.05f, {}) > 0);
}

TEST_CASE("editor pixels -> physics meters", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 } }, true));
    Vec2 c = Physics::toWorld(sim.spec(), sim.rockets()[0].state, { 0, 0 });
    CHECK(c.x == Approx(30.0f).epsilon(1e-4));
    CHECK(c.y == Approx(20.0f).epsilon(1e-4));
    REQUIRE(sim.world().terrain.size() == 1);
    CHECK(sim.world().terrain[0].b.x == Approx(200.0f));
    CHECK(sim.world().terrain[0].a.y == Approx(60.0f));
}

TEST_CASE("step runs whole fixed steps, keeps the remainder", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 } }, false));
    CHECK(sim.step(Simulation::kStep * 3.5f, {}) == 3);
    CHECK(sim.step(Simulation::kStep * 0.6f, {}) == 1);   // 0.5 + 0.6 carries over
    CHECK(sim.step(Simulation::kStep * 0.2f, {}) == 0);
}

TEST_CASE("long frames are clamped", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 } }, false));
    int n = sim.step(10.0f, {});
    CHECK(n <= 12);
    CHECK(n >= 11);
}

TEST_CASE("g acts on every rocket", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 }, { 500, 200 } }, false));
    run(sim, 1.0f);
    for (const auto& b : sim.rockets())
        CHECK(b.state.vel.y == Approx(9.81f).epsilon(0.02));
}

TEST_CASE("g change at runtime takes effect", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 } }, false));
    sim.world().gravity = { 0, 0 };
    run(sim, 1.0f);
    CHECK(sim.rockets()[0].state.vel.y == Approx(0.0f).margin(1e-5));
}

TEST_CASE("input only for selected rocket", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 }, { 700, 200 } }, false));
    sim.world().gravity = { 0, 0 };
    Physics::Input thrust;
    thrust.throttle = 1.0f;

    run(sim, 1.0f, thrust);
    CHECK(sim.rockets()[0].state.vel.y < -5.0f);
    CHECK(sim.rockets()[1].state.vel.y == 0.0f);
    CHECK(sim.rockets()[1].state.fuel == Approx(sim.spec().fuelCapacity));

    sim.selectRocket(1);
    run(sim, 1.0f, thrust);
    CHECK(sim.rockets()[1].state.vel.y < -5.0f);
}

TEST_CASE("ignore invalid select", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ { 300, 200 } }, false));
    sim.selectRocket(4);
    sim.selectRocket(-1);
    CHECK(sim.selectedRocket() == 0);
}

TEST_CASE("hard landing -> crashed, soft -> rests", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ dropAt(100, 20.0f), dropAt(300, 1.0f) }, true));
    run(sim, 8.0f);
    CHECK(sim.rockets()[0].is_crashed);       // ~19.8 m/s impact
    CHECK_FALSE(sim.rockets()[1].is_crashed); //  ~4.4 m/s impact
    CHECK(sim.rockets()[1].info.touching);
}

TEST_CASE("rockets on the floor, not falling through", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ dropAt(100, 2.0f) }, true));
    run(sim, 6.0f);
    Vec2 c = Physics::toWorld(sim.spec(), sim.rockets()[0].state, { 0, 0 });
    CHECK(c.y < 60.0f);
    CHECK(c.y > 57.0f);
}

TEST_CASE("reset restores init scene", "[simulation]")
{
    Simulation sim;
    sim.start(scene({ dropAt(100, 20.0f), dropAt(300, 5.0f) }, true));
    Vec2 start = sim.rockets()[0].state.pos;
    run(sim, 6.0f);
    REQUIRE(sim.rockets()[0].is_crashed);
    sim.selectRocket(1);

    sim.reset();
    CHECK_FALSE(sim.rockets()[0].is_crashed);
    CHECK(sim.rockets()[0].state.pos.y == Approx(start.y));
    CHECK(sim.rockets()[0].state.vel.y == 0.0f);
    CHECK(sim.rockets()[0].state.fuel == Approx(sim.spec().fuelCapacity));
    CHECK(sim.selectedRocket() == 0);
    CHECK(sim.step(Simulation::kStep * 0.5f, {}) == 0); // accumulator cleared
}

TEST_CASE("same scene, inputs -> identical results", "[simulation]")
{
    Scene s = scene({ dropAt(100, 5.0f) }, true);
    Simulation a;
    Simulation b;
    a.start(s);
    b.start(s);

    Physics::Input in;
    in.throttle = 0.4f;
    in.rotate = 0.2f;

    for (int i = 0; i < 200; ++i)
    {
        a.step(0.016f, in);
        b.step(0.016f, in);
    }

    CHECK(a.rockets()[0].state.pos.x == b.rockets()[0].state.pos.x);
    CHECK(a.rockets()[0].state.pos.y == b.rockets()[0].state.pos.y);
    CHECK(a.rockets()[0].state.angle == b.rockets()[0].state.angle);
}