#pragma once

#include "RenderApi/RenderApi.h"

#include "Export.h"

// no STL or SDL types for interface, keeps DLL clean
class RENDERER_API Renderer : public IDrawTarget, public IFileDialog
{
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool init(const char* title, unsigned width, unsigned height);
    void shutdown();

    bool pollEvents(); // window closed = false
    void beginFrame(); // clear and start ImGui frame
    void endFrame();   // draw ImGui on top + presents

    const InputState& input() const override;

    void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) override;
    void drawLine(float x0, float y0, float x1, float y1) override;

    void showFileDialog(DialogType type) override;
    DialogStatus pollFileDialog(char* outPath, int capacity) override; // consumes the result

    bool readPixel(int x, int y, unsigned char rgba[4]) const;

private:
    struct Impl;
    Impl* m_impl;
};