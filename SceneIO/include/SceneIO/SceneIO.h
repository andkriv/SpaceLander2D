#pragma once
#include <string>

#include "SceneData/Terrain.h"

namespace SceneIO
{
    bool save(const Terrain& t,   const std::string& path, std::string* err = nullptr);
    bool load(      Terrain& out, const std::string& path, std::string* err = nullptr);
}