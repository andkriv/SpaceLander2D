#include <filesystem>
#include <fstream>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "EditorModel/EditorModel.h">
#include "EditorModel/SceneFileFlow.h"
#include "SceneIO/SceneIO.h"

#include "ui_placeholder.h"

namespace fs = std::filesystem;
using Catch::Approx;
using Result = SceneFileFlow::Result;

namespace
{
    Scene sample()
    {
        EditorModel m;

        m.startDrawing();
        m.addPoint({ 1, 2 });
        m.addPoint({ 3, 4 });
        m.finish();

        m.addRocket();
        return m.scene();
    }
    struct Tmp
    {
        explicit Tmp(const char* name) : p(fs::temp_directory_path() / name)
        {
            fs::remove(p);
        }

        ~Tmp()
        {
            std::error_code ec;
            fs::remove(p, ec);
        }

        fs::path p;
    };
}

TEST_CASE("requesting a dialog opens it and marks the flow busy", "[files]")
{
    SceneFileFlow f;
    TestDialog d;

    CHECK_FALSE(f.busy());
    f.requestExport(d);

    CHECK(f.busy());
    CHECK(d.shown == 1);
    CHECK(d.lastType == DialogType::Save);
}

TEST_CASE("import opens an Open dialog; second request is ignored", "[files]")
{
    SceneFileFlow f;
    TestDialog d;

    f.requestImport(d);
    CHECK(d.lastType == DialogType::Open);

    f.requestExport(d);
    f.requestImport(d);
    CHECK(d.shown == 1);
}

TEST_CASE("polling does nothing when no dialog is pending", "[files]")
{
    SceneFileFlow f;
    TestDialog d;
    Scene out;

    CHECK(f.poll(d, sample(), out) == Result::None);
    CHECK(d.polls == 0);
}

TEST_CASE("open dialog keeps the flow waiting", "[files]")
{
    SceneFileFlow f;
    TestDialog d;
    Scene out;

    f.requestExport(d);
    CHECK(f.poll(d, sample(), out) == Result::None);
    CHECK(f.busy());
}

TEST_CASE("export writes the file and appends .json", "[files]")
{
    Tmp t("rocketsim_flow_export.json");
    fs::path noExt = t.p;
    noExt.replace_extension("");

    SceneFileFlow f;
    TestDialog d;
    Scene out;

    f.requestExport(d);
    d.next = DialogStatus::Picked;
    d.path = noExt.string();

    CHECK(f.poll(d, sample(), out) == Result::Exported);
    CHECK(fs::exists(t.p));
    CHECK_FALSE(f.busy());
    CHECK(f.status().find("Exported") == 0);
}

TEST_CASE("export does not double the extension", "[files]")
{
    Tmp lower("rocketsim_flow_lower.json");
    Tmp upper("rocketsim_flow_upper.JSON");

    for (Tmp* t : { &lower, &upper })
    {
        SceneFileFlow f;
        TestDialog d;
        Scene out;

        f.requestExport(d);
        d.next = DialogStatus::Picked;
        d.path = t->p.string();

        REQUIRE(f.poll(d, sample(), out) == Result::Exported);
        CHECK(fs::exists(t->p));
    }
    CHECK_FALSE(fs::exists(upper.p.string() + ".json"));
}

TEST_CASE("cancelling changes nothing", "[files]")
{
    SceneFileFlow f;
    TestDialog d;
    Scene out;

    f.requestImport(d);
    d.next = DialogStatus::Cancelled;

    CHECK(f.poll(d, sample(), out) == Result::None);
    CHECK_FALSE(f.busy());
    CHECK(f.status().empty());
    CHECK(out.rockets.empty());
}

TEST_CASE("import loads sel file", "[files]")
{
    Tmp t("rocketsim_flow_import.json");
    REQUIRE(SceneIO::save(sample(), t.p.string()));

    SceneFileFlow f;
    TestDialog d;
    Scene out;

    f.requestImport(d);
    d.next = DialogStatus::Picked; d.path = t.p.string();
    CHECK(f.poll(d, Scene{}, out) == Result::Imported);
    CHECK(out.terrain.polylines.size() == 1);
    CHECK(out.rockets.size() == 1);
    CHECK(f.status().find("Imported") == 0);
}

TEST_CASE("bad import fails, output untouched", "[files]")
{
    Tmp t("rocketsim_flow_bad.json");
    std::ofstream(t.p) << "not json";

    SceneFileFlow f;
    TestDialog d;
    Scene out = sample();

    f.requestImport(d);
    d.next = DialogStatus::Picked; d.path = t.p.string();
    CHECK(f.poll(d, Scene{}, out) == Result::Failed);
    CHECK(f.status().find("Import failed") == 0);
    CHECK(out.rockets.size() == 1);
    CHECK_FALSE(f.busy());
}

TEST_CASE("export to unwritable path = failure", "[files]")
{
    SceneFileFlow f;
    TestDialog d;
    Scene out;

    f.requestExport(d);
    d.next = DialogStatus::Picked;
    d.path = (fs::temp_directory_path() / "rocketsim_no_such_dir" / "x").string();
    CHECK(f.poll(d, sample(), out) == Result::Failed);
    CHECK(f.status().find("Export failed") == 0);
}

TEST_CASE("dialog result is consumed once", "[files]")
{
    Tmp t("rocketsim_flow_once.json");
    SceneFileFlow f;
    TestDialog d;
    Scene out;

    f.requestExport(d);
    d.next = DialogStatus::Picked; d.path = t.p.string();
    REQUIRE(f.poll(d, sample(), out) == Result::Exported);
    CHECK(f.poll(d, sample(), out) == Result::None);
    CHECK(d.polls == 1);
}