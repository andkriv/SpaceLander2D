#include "EditorModel/SceneFileFlow.h"
#include <SceneIO/SceneIO.h>
#include <cctype>

namespace
{
    bool hasJsonExt(const std::string& p)
    {
        if (p.size() < 5)
            return false;

        for (int i = 0; i < 5; ++i)
            if (std::tolower((unsigned char)p[p.size() - 5 + i]) != ".json"[i])
                return false;
        return true;
    }
}

void SceneFileFlow::requestExport(IFileDialog& d)
{
    if (busy())
        return;

    m_pending = Pending::Export;
    d.showFileDialog(DialogType::Save);
}

void SceneFileFlow::requestImport(IFileDialog& d)
{
    if (busy())
        return;

    m_pending = Pending::Import;
    d.showFileDialog(DialogType::Open);
}

SceneFileFlow::Result SceneFileFlow::poll(IFileDialog& d, const Scene& current, Scene& imported)
{
    if (m_pending == Pending::None)
        return Result::None;

    char buf[1024] = {};
    DialogStatus s = d.pollFileDialog(buf, sizeof buf);
    if (s == DialogStatus::None)
        return Result::None; // still open

    Pending p = m_pending;
    m_pending = Pending::None;
    if (s == DialogStatus::Cancelled)
        return Result::None;

    std::string path = buf, err;
    if (p == Pending::Export)
    {
        if (!hasJsonExt(path))
            path += ".json";

        if (SceneIO::save(current, path, &err))
        {
            m_status = "Exported: " + path;
            return Result::Exported;
        }

        m_status = "Export failed: " + err;
        return Result::Failed;
    }

    Scene tmp;
    if (SceneIO::load(tmp, path, &err))
    {
        imported = std::move(tmp);
        m_status = "Imported: " + path;
        return Result::Imported;
    }
    m_status = "Import failed: " + err;
    return Result::Failed;
}

const std::string& SceneFileFlow::status() const
{
    return m_status;
}

bool SceneFileFlow::busy() const
{
    return m_pending != Pending::None;
}