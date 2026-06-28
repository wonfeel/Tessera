#include "CellularAutomaton.h"
#include "engine/chunk/Chunk.h"
#include "engine/graphics/Palette.h"

CellularAutomaton::CellularAutomaton(int totalWidth, int totalHeight, int chunkSize, float tileSize)
    : ChunkedTileMap(totalWidth, totalHeight, chunkSize, tileSize) {
    setPalette(Palette::DefaultRainbow());   // по умолчанию — радуга
}

void CellularAutomaton::simulateChunk(Chunk& chunk) {
    const int S = chunk.chunkSize;
    auto ext = getExtendedNeighborhood(chunk, 1);
    std::vector<uint8_t> next(S * S, 0);
    const int extW = S + 2;
    for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
            uint8_t n[9];
            int idx = 0;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    n[idx++] = ext[(y + 1 + dy) * extW + (x + 1 + dx)];
            next[y * S + x] = transition(n);
        }
    }
    {
        std::lock_guard<std::mutex> lock(chunk.chunkMutex);
        std::copy(next.begin(), next.end(), chunk.simBuffer.begin());
        chunk.dirty.store(true, std::memory_order_release);
    }
}