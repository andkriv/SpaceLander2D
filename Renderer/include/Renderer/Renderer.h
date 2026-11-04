#pragma once
#include "Export.h"

// no STL or SDL types for interface, keeps DLL clean
class RENDERER_API Renderer
{
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool init(const char* title, int width, int height);
    void shutdown();

    bool pollEvents(); // window closed = false
    void beginFrame(); // clear and start ImGui frame
    void endFrame();   // draw ImGui on top + presents

    void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255);
    void drawLine(float x0, float y0, float x1, float y1);

private:
    struct Impl;
    Impl* m_impl;
};