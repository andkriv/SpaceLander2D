#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include "SceneIO/SceneIO.h"

using json = nlohmann::json;

namespace
{
    constexpr int kVersion = 3;

    std::filesystem::path toPath(const std::string& s)
    {
        return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(s.c_str())));
    }

    void fail(std::string* err, const std::string& msg)
    {
        if (err)
            *err = msg;
    }

    json specToJson(const RocketSpec& s)
    {
        return {{"dryMass",       s.dryMass},
                {"fuelCapacity",  s.fuelCapacity},
                {"fuelStart",     s.fuelStart},
                {"maxThrust",     s.maxThrust},
                {"isp",           s.isp},
                {"rcsThrust",     s.rcsThrust},
                {"rcsArm",        s.rcsArm},
                {"noseHalfWidth", s.noseHalfWidth},
                {"baseHalfWidth", s.baseHalfWidth},
                {"halfHeight",    s.halfHeight},
                {"dryComY",       s.dryComY},
                {"tankWidth",     s.tankWidth},
                {"tankHeight",    s.tankHeight} };
    }

    RocketSpec specFromJson(const json& j)
    {
        RocketSpec s;
        auto get = [&](const char* k, float& v)
        {
            v = j.value(k, v);
        };

        get("dryMass",       s.dryMass);
        get("fuelCapacity",  s.fuelCapacity);
        get("fuelStart",     s.fuelStart);
        get("maxThrust",     s.maxThrust);
        get("isp",           s.isp);
        get("rcsThrust",     s.rcsThrust);
        get("rcsArm",        s.rcsArm);
        get("noseHalfWidth", s.noseHalfWidth);
        get("baseHalfWidth", s.baseHalfWidth);
        get("halfHeight",    s.halfHeight);
        get("dryComY",       s.dryComY);
        get("tankWidth",     s.tankWidth);
        get("tankHeight",    s.tankHeight);

        sanitize(s);
        return s;
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
                                {"angle", r.angle},
                                {"spec", specToJson(r.spec)} });
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
                    r.spec = specFromJson(jr.value("spec", json::object()));

                    s.rockets.push_back(std::move(r));
                }
            }
            out = std::move(s); // only replace on full success
            return true;
        }
        catch (const std::exception& e)
        {
            fail(err, e.what());
            return false;
        }
    }

} // namespace SceneIO