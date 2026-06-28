#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkedTileMap.h"
#include <algorithm>

Chunk::Chunk(const ChunkCoord& coord, const glm::ivec2& worldOff,
    float tileSize, ChunkedTileMap* map, int size)
    : worldOffset(worldOff), chunkSize(size), m_map(map)
{
    renderer = std::make_unique<ChunkRenderer>(chunkSize);

    int total = chunkSize * chunkSize;
    simBuffer.resize(total, 0);
    renderBuffer.resize(total, 0);

    bounds.min = glm::vec2(worldOff) * tileSize;
    bounds.max = glm::vec2(worldOff + glm::ivec2(chunkSize)) * tileSize;
}

void Chunk::updateRenderData() {
    renderer->updateIndices(renderBuffer.data());
    dirty.store(false, std::memory_order_release);
}

void Chunk::commitChanges() {
    std::swap(simBuffer, renderBuffer);
    recalcLiveCells();
    dirty.store(true, std::memory_order_release);
}

void Chunk::simulate(ChunkedTileMap& map) {
    map.simulateChunk(*this);
}

void Chunk::recalcLiveCells() {
    int cnt = 0;
    for (uint8_t v : renderBuffer)
        if (v != 0) ++cnt;
    liveCells.store(cnt, std::memory_order_release);
}