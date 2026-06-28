#pragma once
#include "engine/chunk/ChunkedTileMap.h"

class CellularAutomaton : public ChunkedTileMap {
public:
    CellularAutomaton(int totalWidth, int totalHeight, int chunkSize, float tileSize);
protected:
    virtual uint8_t transition(const uint8_t neighbors[9]) const = 0;
    void simulateChunk(Chunk& chunk) override final;
};