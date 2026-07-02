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

void LifeLikeAutomaton::simulateChunkBatch(const std::vector<std::shared_ptr<Chunk>>& chunks) {
    if (chunks.empty()) return;

    // GL-interop чанки пишут результат прямо в свой собственный VBO (D2D) —
    // это неотъемлемо по одному на чанк, батчить нечего. Остальные — кандидаты
    // на общий batched-вызов бэкенда.
    std::vector<std::shared_ptr<Chunk>> interopChunks;
    std::vector<std::shared_ptr<Chunk>> plainChunks;
    for (auto& c : chunks) {
        if (!c) continue;
        unsigned int vbo = m_backend->supportsGLInterop() ? c->renderer->getInstanceVBO() : 0u;
        (vbo != 0 ? interopChunks : plainChunks).push_back(c);
    }

    for (auto& c : interopChunks) simulateChunk(*c);

    if (plainChunks.empty()) return;

    if (!m_backend->preferBatch() || plainChunks.size() == 1) {
        for (auto& c : plainChunks) simulateChunk(*c);
        return;
    }

    const int S = plainChunks.front()->chunkSize;
    const int extW = S + 2;
    const size_t n = plainChunks.size();

    // Живут до конца функции — simulateBatch получает только сырые указатели.
    std::vector<std::vector<uint8_t>> exts(n);
    std::vector<std::vector<uint8_t>> outs(n, std::vector<uint8_t>(static_cast<size_t>(S) * S, 0));
    std::vector<const uint8_t*> extPtrs(n);
    std::vector<uint8_t*> outPtrs(n);
    for (size_t i = 0; i < n; ++i) {
        exts[i] = getExtendedNeighborhood(*plainChunks[i], 1);
        extPtrs[i] = exts[i].data();
        outPtrs[i] = outs[i].data();
    }

    m_backend->simulateBatch(extPtrs, extW, outPtrs, S, m_rule);

    for (size_t i = 0; i < n; ++i) {
        Chunk& chunk = *plainChunks[i];
        std::lock_guard<std::mutex> lock(chunk.chunkMutex);
        std::copy(outs[i].begin(), outs[i].end(), chunk.simBuffer.begin());
        chunk.dirty.store(true, std::memory_order_release);
    }
}
