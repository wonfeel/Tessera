#pragma once
#include <glm/glm.hpp>

struct AABB {
    glm::vec2 min, max;
    bool overlaps(const AABB& other) const {
        return min.x < other.max.x && max.x > other.min.x &&
               min.y < other.max.y && max.y > other.min.y;
    }
};