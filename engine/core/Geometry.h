// engine/core/Geometry.h
#pragma once
#include <vector>
#include <algorithm>
#include <limits>
#include <glm/glm.hpp>

// Small 2D geometry helpers, no state, shared across demos.
namespace Geometry {

inline float pointSegmentDist(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
    glm::vec2 ab = b - a;
    float len2 = glm::dot(ab, ab);
    float t = len2 > 1e-8f ? std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
    return glm::length(p - (a + t * ab));
}

// Even-odd ray-cast point-in-polygon test; also returns the distance to the
// nearest edge (useful for feathering a polygon's boundary). Polygon need
// not be convex.
inline bool pointInPolygon(glm::vec2 p, const std::vector<glm::vec2>& polygon, float& outMinEdgeDist) {
    bool inside = false;
    float minEdgeDist = std::numeric_limits<float>::max();
    int vertCount = static_cast<int>(polygon.size());
    for (int a = 0, b = vertCount - 1; a < vertCount; b = a++) {
        const glm::vec2& pa = polygon[static_cast<size_t>(a)];
        const glm::vec2& pb = polygon[static_cast<size_t>(b)];
        bool crosses = ((pa.y > p.y) != (pb.y > p.y)) &&
            (p.x < (pb.x - pa.x) * (p.y - pa.y) / (pb.y - pa.y) + pa.x);
        if (crosses) inside = !inside;
        minEdgeDist = std::min(minEdgeDist, pointSegmentDist(p, pa, pb));
    }
    outMinEdgeDist = minEdgeDist;
    return inside;
}

}   // namespace Geometry
