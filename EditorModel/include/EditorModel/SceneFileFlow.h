#pragma once
#include <string>

#include "RenderApi/RenderApi.h"
#include "SceneData/Scene.h"

class SceneFileFlow
{
public:
    enum class Result
    {
        None,
        Exported,
        Imported,
        Failed
    };

    bool busy() const;

    void requestImport(IFileDialog& d);
    void requestExport(IFileDialog& d);

    // call every frame, imported is filled only when Result::Imported ret
    Result poll(IFileDialog& d, const Scene& current, Scene& imported);

    const std::string& status() const;

private:
    enum class Pending
    {
        None,
        Import,
        Export
    };

private:
    Pending     m_pending = Pending::None;
    std::string m_status;
};