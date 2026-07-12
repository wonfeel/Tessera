// engine/core/HexGrid.h
#pragma once
#include <glm/glm.hpp>

// Pointy-top, odd-r offset hex grid geometry - pure math, no physics/state.
// Extracted out of demo/light/LightField so any demo can use the same
// layout without duplicating the formulas. See redblobgames "Hexagonal
// Grids" for the reference layout (odd-r horizontal).
namespace HexGrid {

constexpr float kSqrt3 = 1.7320508075688772f;   // compile-time, no runtime std::sqrt

inline float horizSpacing(float spacing) { return kSqrt3 * spacing; }
inline float vertSpacing(float spacing) { return 1.5f * spacing; }

// World position of hex (col,row) - odd rows are shifted half a cell right.
inline glm::vec2 worldPos(int col, int row, float spacing) {
    float horiz = horizSpacing(spacing);
    float vert = vertSpacing(spacing);
    float x = horiz * (static_cast<float>(col) + ((row & 1) ? 0.5f : 0.0f));
    float y = vert * static_cast<float>(row);
    return glm::vec2(x, y);
}

// 6 neighbor offsets (dcol,drow), row-parity dependent since odd rows are
// shifted in world space but not in the (col,row) index itself.
constexpr int kOffsetsEven[6][2] = {{+1,0}, {0,-1}, {-1,-1}, {-1,0}, {-1,+1}, {0,+1}};
constexpr int kOffsetsOdd[6][2]  = {{+1,0}, {+1,-1}, {0,-1}, {-1,0}, {0,+1}, {+1,+1}};

inline const int (*neighborOffsets(int row))[2] {
    return (row & 1) ? kOffsetsOdd : kOffsetsEven;
}

}   // namespace HexGrid
