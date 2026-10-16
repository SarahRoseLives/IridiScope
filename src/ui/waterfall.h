// OpenGL scrolling waterfall widget (newest row at top).
// Adapted from AirScope (github.com/...) for IridiScope.
#pragma once

#include "imgui.h"

#include <cstdint>
#include <vector>

class Waterfall {
public:
    Waterfall() = default;
    ~Waterfall();

    // Push one FFT row (magnitude in dB, length = number of bins).
    void addRow(const float *db, int n, float dbMin, float dbMax);

    // Draw filling the given size.
    void draw(const ImVec2 &size, float uMin = 0.0f, float uMax = 1.0f);

    void clear();

private:
    void ensureTexture(int bins);
    static void colormap(float t, uint8_t &r, uint8_t &g, uint8_t &b);

    unsigned int tex_ = 0; // GLuint
    int texW_ = 0;
    int texH_ = 1024;
    int writePtr_ = 0;
    bool filled_ = false;

    std::vector<uint8_t> rowBuf_;
};
