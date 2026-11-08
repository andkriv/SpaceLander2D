#pragma once

struct InputState
{
    float mouseX = 0;
    float mouseY = 0;

    bool  leftPressed = false;   // true only on the frame btn fired
    bool  rightPressed = false;
};

enum class DialogType
{
    Open,
    Save
};
enum class DialogStatus
{
    None,
    Picked,
    Cancelled
};

class IDrawTarget
{
public:
    virtual ~IDrawTarget() = default;
    virtual void setColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) = 0;
    virtual void drawLine(float x0, float y0, float x1, float y1) = 0;
    virtual const InputState& input() const = 0;
};

class IFileDialog
{
public:
    virtual ~IFileDialog() = default;
    virtual void showFileDialog(DialogType kind) = 0;
    virtual DialogStatus pollFileDialog(char* outPath, int capacity) = 0;
};