// engine/chunk/Chunk.h
#pragma once
#include "ChunkCoord.h"
#include "ChunkRenderer.h"
#include "AABB.h"
#include <glm/glm.hpp>
#include <vector>
#include <atomic>
#include <memory>
#include <mutex>

class ChunkedTileMap;

class Chunk {
public:
    const int chunkSize;

    Chunk(const ChunkCoord& coord, const glm::ivec2& worldOffset, float tileSize, ChunkedTileMap* map, int chunkSize);
    virtual ~Chunk() = default;

    glm::ivec2 worldOffset;
    AABB bounds;

    std::vector<uint8_t> simBuffer;
    std::vector<uint8_t> renderBuffer;

    std::unique_ptr<ChunkRenderer> renderer;

    alignas(64) std::atomic<bool> dirty{ true };
    alignas(64) std::atomic<bool> active{ false };
    std::atomic<int> liveCells{ 0 };

    std::mutex chunkMutex;

    uint64_t lastSimGeneration = 0;   // опционально, для отладки

    void updateRenderData();
    void commitChanges();
    void simulate(ChunkedTileMap& map);
    void recalcLiveCells();   // пересчитывает liveCells по renderBuffer

    ChunkedTileMap* getMap() const { return m_map; }

private:
    ChunkedTileMap* m_map;
};