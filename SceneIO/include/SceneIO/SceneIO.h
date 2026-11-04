#pragma once
#include <string>

#include "SceneData/Scene.h"

namespace SceneIO
{
    bool save(const Scene& s,   const std::string& path, std::string* err = nullptr);
    bool load(      Scene& out, const std::string& path, std::string* err = nullptr);
}