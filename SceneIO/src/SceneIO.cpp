#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include "SceneIO/SceneIO.h"

using json = nlohmann::json;

namespace
{
    constexpr int kVersion = 2;

    std::filesystem::path toPath(const std::string& s)
    {
        return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(s.c_str())));
    }

    void fail(std::string* err, const std::string& msg)
    {
        if (err)
            *err = msg;
    }
}

namespace SceneIO
{

    bool save(const Scene& s, const std::string& path, std::string* err)
    {
        json j;

        j["version"] = kVersion;
        json lines = json::array();
        for (const auto& pl : s.terrain.polylines)
        {
            json pts = json::array();
            for (const auto& p : pl.points)
                pts.push_back({ p.x, p.y });

            lines.push_back({ {"name", pl.name}, {"points", pts} });
        }
        j["terrain"] = { {"polylines", lines} };

        json rockets = json::array();
        for (const auto& r : s.rockets)
            rockets.push_back({ {"name", r.name},
                                {"pos", { r.pos.x, r.pos.y }},
                                {"angle", r.angle} });
        j["rockets"] = rockets;

        std::ofstream f(toPath(path));
        if (!f)
        {
            fail(err, "Cannot open file for writing");
            return false;
        }
        f << j.dump(2);
        return f.good();
    }

    bool load(Scene& out, const std::string& path, std::string* err)
    {
        std::ifstream f(toPath(path));
        if (!f)
        {
            fail(err, "Cannot open file");
            return false;
        }

        try
        {
            json j = json::parse(f);

            if (j.value("version", 0) > kVersion)
            {
                fail(err, "File was made by a newer version");
                return false;
            }

            Scene s;
            for (const auto& jl : j.at("terrain").at("polylines"))
            {
                Polyline pl;
                pl.name = jl.at("name").get<std::string>();
                for (const auto& jp : jl.at("points"))
                    pl.points.push_back({ jp.at(0).get<float>(), jp.at(1).get<float>() });
                s.terrain.polylines.push_back(std::move(pl));
            }
            if (j.contains("rockets"))
            {
                for (const auto& jr : j["rockets"])
                {
                    Rocket r;
                    r.name = jr.at("name").get<std::string>();
                    r.pos = { jr.at("pos").at(0).get<float>(), jr.at("pos").at(1).get<float>() };
                    r.angle = jr.at("angle").get<float>();
                    s.rockets.push_back(std::move(r));
                }
            }
            out = std::move(s);   // only replace on full success
            return true;
        }
        catch (const std::exception& e)
        {
            fail(err, e.what());
            return false;
        }
    }

} // namespace SceneIO