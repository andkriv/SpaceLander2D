#pragma once
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "RenderApi/RenderApi.h"

// rec every call, check draw() result
struct Rec : IDrawTarget
{
    struct Color
    {
        unsigned char r = 0, g = 0, b = 0, a = 255;
        bool operator==(const Color&) const = default;
    };
    struct Line
    {
        float x0;
        float y0;

        float x1;
        float y1;

        Color c;
    };

    std::vector<Line> lines;
    Color      clr;
    InputState in;

    void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) override
    {
        clr = { r, g, b, a };
    }

    void drawLine(float x0, float y0, float x1, float y1) override
    {
        lines.push_back({ x0, y0, x1, y1, clr });
    }

    const InputState& input() const override
    {
        return in;
    }

    int count(Color c) const
    {
        int n = 0;
        for (auto& l : lines)
        {
            if (l.c == c)
                ++n;
        }
        return n;
    }
};

// native dialogs
struct TestDialog : IFileDialog
{
    int shown = 0;
    int polls = 0;

    DialogType   lastKind = DialogType::Open;
    DialogStatus next = DialogStatus::None; // user acts consumed by one poll
    std::string  path;

    void showFileDialog(DialogType t) override
    {
        ++shown;
        lastKind = t;
    }

    DialogStatus pollFileDialog(char* out, int cap) override
    {
        ++polls;
        DialogStatus s = next;
        next = DialogStatus::None;
        if (s == DialogStatus::Picked && cap > 0)
        {
            std::strncpy(out, path.c_str(), cap - 1);
            out[cap - 1] = 0;
        }
        return s;
    }
};