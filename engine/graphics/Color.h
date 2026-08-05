// engine/graphics/Color.h
#pragma once
#include <algorithm>
#include <glm/glm.hpp>

namespace Color {

// Pushes a color away from its own luma toward its hue - boost > 1
// increases the difference between channels ("more vivid"), boost == 1 is
// a no-op. Clamped to [0,1] - a boosted channel can go negative (e.g. a dim
// blue with little red/green) or above 1 even if the input was already
// clamped to 1 (a channel far from luma gets pushed past it) - both are
// undefined under additive blending.
inline glm::vec3 boostSaturation(glm::vec3 c, float boost) {
    float luma = 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
    return glm::vec3(
        std::clamp(luma + (c.r - luma) * boost, 0.0f, 1.0f),
        std::clamp(luma + (c.g - luma) * boost, 0.0f, 1.0f),
        std::clamp(luma + (c.b - luma) * boost, 0.0f, 1.0f));
}

}   // namespace Color
