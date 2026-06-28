#pragma once
#include <cstdint>
#include <functional>

struct ChunkCoord {
    uint64_t packed;

    ChunkCoord() : packed(0) {}
    ChunkCoord(int x, int y) : packed((static_cast<uint64_t>(x) << 32) | (static_cast<uint32_t>(y))) {}

    int x() const { return static_cast<int>(packed >> 32); }
    int y() const { return static_cast<int>(packed & 0xFFFFFFFF); }

    bool operator==(const ChunkCoord& o) const { return packed == o.packed; }
};

namespace std {
    template<>
    struct hash<ChunkCoord> {
        size_t operator()(const ChunkCoord& c) const noexcept {
            return std::hash<uint64_t>()(c.packed);
        }
    };
}