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

    std::vector<uint8_t> ext = getExtendedNeighborhood(chunk, 1);
    const int extW = S + 2;

    std::vector<uint8_t> next(static_cast<size_t>(S) * S, 0);

    // Если бэкенд поддерживает CUDA-GL interop — передаём VBO чанка.
    // Тогда GPU пишет результат напрямую в GL VBO (D2D), минуя glBufferSubData.
    // renderBuffer по-прежнему нужен для border exchange с соседями, поэтому
    // D2H в next делается в любом случае.
    unsigned int vbo = m_backend->supportsGLInterop()
                       ? chunk.renderer->getInstanceVBO()
                       : 0u;
    m_backend->simulateDirect(ext.data(), extW, next.data(), S, m_rule, vbo);

    std::lock_guard<std::mutex> lock(chunk.chunkMutex);
    std::copy(next.begin(), next.end(), chunk.simBuffer.begin());
    chunk.dirty.store(true, std::memory_order_release);
}
