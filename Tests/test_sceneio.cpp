#include <fstream>
#include <sstream>
#include <filesystem>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "SceneIO/SceneIO.h"

// Save/load, versioning, error safety, unicode path

using Catch::Approx;
namespace fs = std::filesystem;

namespace
{
    struct TempFile
    {
        explicit TempFile(const fs::path& name) : p(fs::temp_directory_path() / name)
        {
            fs::remove(p);
        }

        ~TempFile()
        {
            std::error_code ec;
            fs::remove(p, ec);
        }

        std::string str() const
        { // utf-8
            auto u = p.u8string();
            return std::string(u.begin(), u.end());
        }

        fs::path p;
    };

    void write(const fs::path& p, const std::string& text)
    {
        std::ofstream(p) << text;
    }

    std::string read(const fs::path& p)
    {
        std::ifstream f(p);
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
    }

    Scene test_scene()
    {
        Scene s;

        Polyline a;
        a.name = "Polyline 1";
        a.points = { { 100, 600 }, { 400.5f, 580.25f }, { 900, 640 } };

        Polyline b;
        b.name = "quote \" and \\ slash";
        b.points = { { 0, 0 }, { 1, 1 } };

        s.terrain.polylines = { a, b };

        Rocket r1;
        r1.name = "Rocket 1";
        r1.pos = { 300, 200 };
        r1.angle = 0.35f;

        Rocket r2;
        r2.name = "Rocket 2";
        r2.pos = { 360.5f, 200 };
        r2.angle = -1.25f;

        s.rockets = { r1, r2 };
        return s;
    }
}

TEST_CASE("scene file full cyrcle", "[sceneio]")
{
    TempFile f("rocketsim_roundtrip.json");
    Scene in = test_scene();
    Scene out;

    std::string err;
    REQUIRE(SceneIO::save(in, f.str(), &err));
    REQUIRE(SceneIO::load(out, f.str(), &err));

    REQUIRE(out.terrain.polylines.size() == 2);

    for (size_t i = 0; i < 2; ++i)
    {
        CHECK(out.terrain.polylines[i].name == in.terrain.polylines[i].name);
        REQUIRE(out.terrain.polylines[i].points.size() == in.terrain.polylines[i].points.size());
        for (size_t k = 0; k < in.terrain.polylines[i].points.size(); ++k)
        {
            CHECK(out.terrain.polylines[i].points[k].x == Approx(in.terrain.polylines[i].points[k].x));
            CHECK(out.terrain.polylines[i].points[k].y == Approx(in.terrain.polylines[i].points[k].y));
        }
    }

    REQUIRE(out.rockets.size() == 2);
    for (size_t i = 0; i < 2; ++i)
    {
        CHECK(out.rockets[i].name == in.rockets[i].name);
        CHECK(out.rockets[i].pos.x == Approx(in.rockets[i].pos.x));
        CHECK(out.rockets[i].pos.y == Approx(in.rockets[i].pos.y));
        CHECK(out.rockets[i].angle == Approx(in.rockets[i].angle));
    }
}

TEST_CASE("empty scene", "[sceneio]") {
    TempFile f("rocketsim_empty.json");
    Scene out = test_scene();

    REQUIRE(SceneIO::save(Scene{}, f.str()));
    REQUIRE(SceneIO::load(out, f.str()));

    CHECK(out.terrain.polylines.empty());
    CHECK(out.rockets.empty());
}

TEST_CASE("saved file version", "[sceneio]")
{
    TempFile f("rocketsim_version.json");

    REQUIRE(SceneIO::save(test_scene(), f.str()));
    CHECK(read(f.p).find("\"version\": 3") != std::string::npos);
}

TEST_CASE("version 1 (no rockets) still loads", "[sceneio]")
{
    Scene out;

    TempFile f("rocketsim_v1.json");
    write(f.p, R"({"version":1,"terrain":{"polylines":[{"name":"A","points":[[1,2],[3,4]]}]}})");

    REQUIRE(SceneIO::load(out, f.str()));
    REQUIRE(out.terrain.polylines.size() == 1);
    CHECK(out.terrain.polylines[0].points[1].y == Approx(4.0f));
    CHECK(out.rockets.empty());
}

TEST_CASE("wrong (newer) version rejected", "[sceneio]")
{
    TempFile f("rocketsim_future.json");
    write(f.p, R"({"version":99,"terrain":{"polylines":[]},"rockets":[]})");

    Scene out = test_scene();
    std::string err;

    CHECK_FALSE(SceneIO::load(out, f.str(), &err));
    CHECK_FALSE(err.empty());
    CHECK(out.rockets.size() == 2); // untouched
}

TEST_CASE("bad input fails cleanly and leaves the target untouched", "[sceneio]")
{
    TempFile f("rocketsim_bad.json");
    Scene out = test_scene();
    std::string err;

    SECTION("not JSON")
    {
        write(f.p, "this is not json");
        CHECK_FALSE(SceneIO::load(out, f.str(), &err));
    }
    SECTION("missing terrain section")
    {
        write(f.p, R"({"version":2,"rockets":[]})");
        CHECK_FALSE(SceneIO::load(out, f.str(), &err));
    }
    SECTION("a later rocket is invalid: nothing is half-loaded")
    {
        write(f.p, R"({"version":2,"terrain":{"polylines":[]},
                                   "rockets":[{"name":"ok","pos":[1,2],"angle":0},{"name":"bad","pos":[1,2]}]})");
        CHECK_FALSE(SceneIO::load(out, f.str(), &err));
    }
    SECTION("wrong point shape")
    {
        write(f.p, R"({"version":2,"terrain":{"polylines":[{"name":"A","points":[[1]]}]}})");
        CHECK_FALSE(SceneIO::load(out, f.str(), &err));
    }
    CHECK_FALSE(err.empty());
    CHECK(out.terrain.polylines.size() == 2);
    CHECK(out.rockets.size() == 2);
}

TEST_CASE("missing file and unwritable path fail", "[sceneio]")
{
    Scene out;
    std::string err;
    CHECK_FALSE(SceneIO::load(out, (fs::temp_directory_path() / "rocketsim_nope.json").string(), &err));
    CHECK_FALSE(err.empty());

    err.clear();
    fs::path bad = fs::temp_directory_path() / "rocketsim_no_such_dir" / "x.json";
    CHECK_FALSE(SceneIO::save(test_scene(), bad.string(), &err));
    CHECK_FALSE(err.empty());
}

TEST_CASE("error pointer is optional", "[sceneio]")
{
    Scene out;
    CHECK_FALSE(SceneIO::load(out, (fs::temp_directory_path() / "rocketsim_nope2.json").string()));
}

TEST_CASE("non-ASCII file names work", "[sceneio]")
{
    TempFile f(fs::path(u8"\u0441\u0446\u0435\u043d\u0430_\u0442\u0435\u0441\u0442.json")); // "сцена_тест.json"
    Scene out;

    REQUIRE(SceneIO::save(test_scene(), f.str()));
    CHECK(fs::exists(f.p));
    REQUIRE(SceneIO::load(out, f.str()));
    CHECK(out.rockets.size() == 2);
}

TEST_CASE("rocket specs round-trip", "[sceneio][spec]")
{
    TempFile f("rocketsim_spec.json");
    Scene in = test_scene();
    in.rockets[1].spec.dryMass = 1234.0f;
    in.rockets[1].spec.fuelStart = 321.0f;
    in.rockets[1].spec.rcsThrust = 777.0f;
    in.rockets[1].spec.baseHalfWidth = 2.5f;

    Scene out;
    REQUIRE(SceneIO::save(in, f.str()));
    REQUIRE(SceneIO::load(out, f.str()));
    CHECK(out.rockets[1].spec.dryMass == Approx(1234.0f));
    CHECK(out.rockets[1].spec.fuelStart == Approx(321.0f));
    CHECK(out.rockets[1].spec.rcsThrust == Approx(777.0f));
    CHECK(out.rockets[1].spec.baseHalfWidth == Approx(2.5f));
    CHECK(out.rockets[0].spec.dryMass == Approx(800.0f));
}

TEST_CASE("version 2 files load with default specs; bad specs are clamped", "[sceneio][spec]")
{
    TempFile f("rocketsim_v2.json");
    write(f.p, R"({"version":2,"terrain":{"polylines":[]},
                               "rockets":[{"name":"R","pos":[1,2],
                               "angle":0}]})");

    Scene out;
    REQUIRE(SceneIO::load(out, f.str()));
    CHECK(out.rockets[0].spec.dryMass == Approx(800.0f));

    write(f.p, R"({"version":3,"terrain":{"polylines":[]},
                               "rockets":[{"name":"R","pos":[1,2],
                               "angle":0,
                               "spec":{"dryMass":-10,"isp":0}}]})");

    REQUIRE(SceneIO::load(out, f.str()));
    CHECK(out.rockets[0].spec.dryMass >= 1.0f);
    CHECK(out.rockets[0].spec.isp >= 1.0f);
}