// engine/automaton/LifeLikeAutomaton.cpp
#include "engine/automaton/LifeLikeAutomaton.h"
#include "engine/chunk/Chunk.h"
#include "engine/graphics/Palette.h"
#include "engine/simulation/SimulationBackendFactory.h"
#include <algorithm>
#include <vector>

LifeLikeAutomaton::LifeLikeAutomaton(int totalWidth, int totalHeight, int chunkSize,
                                     float tileSize, const LifeRule& rule)
    : ChunkedTileMap(totalWidth, totalHeight, chunkSize, tileSize)
    , m_rule(rule)
    , m_backend(MakeSimulationBackend())
{
    setPalette(Palette::DefaultRainbow());
}

void LifeLikeAutomaton::simulateChunk(Chunk& chunk) {
    const int S = chunk.chunkSize;

    // Расширенная окрестность (S+2)x(S+2) с данными соседних чанков. Эту часть
    // (сбор окрестности из renderBuffer соседей) считает движок одинаково для
    // CPU и GPU — на устройство уходит уже готовый буфер.
    std::vector<uint8_t> ext = getExtendedNeighborhood(chunk, 1);
    const int extW = S + 2;

    std::vector<uint8_t> next(static_cast<size_t>(S) * S, 0);
    m_backend->simulate(ext.data(), extW, next.data(), S, m_rule);

    std::lock_guard<std::mutex> lock(chunk.chunkMutex);
    std::copy(next.begin(), next.end(), chunk.simBuffer.begin());
    chunk.dirty.store(true, std::memory_order_release);
}
