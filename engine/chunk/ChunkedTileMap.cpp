// engine/chunk/ChunkedTileMap.cpp
#include "engine/chunk/ChunkedTileMap.h"
#include "engine/core/TaskScheduler.h"
#include <algorithm>
#include <sstream>
#include <glm/glm.hpp>

ChunkedTileMap::ChunkedTileMap(int totalWidth, int totalHeight, int chunkSize, float tileSize)
    : m_totalWidth(totalWidth), m_totalHeight(totalHeight), m_chunkSize(chunkSize), m_tileSize(tileSize)
{
}

Chunk* ChunkedTileMap::getOrCreateChunk(ChunkCoord coord) {
    auto it = m_chunks.find(coord);
    if (it != m_chunks.end()) return it->second.get();

    glm::ivec2 worldOffset(coord.x() * m_chunkSize, coord.y() * m_chunkSize);
    auto chunk = createChunk(coord, worldOffset, m_chunkSize, this);
    Chunk* ptr = chunk.get();
    m_chunks[coord] = std::move(chunk);
    return ptr;
}

Chunk* ChunkedTileMap::getChunk(ChunkCoord coord) {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_chunks.find(coord);
    return (it != m_chunks.end()) ? it->second.get() : nullptr;
}

std::shared_ptr<Chunk> ChunkedTileMap::getChunkShared(ChunkCoord coord) const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_chunks.find(coord);
    return (it != m_chunks.end()) ? it->second : nullptr;
}

std::string ChunkedTileMap::getTileInfo(int x, int y) const {
    if (x < 0 || x >= m_totalWidth || y < 0 || y >= m_totalHeight) return "";
    int state = getTileState(x, y);
    std::ostringstream oss;
    oss << "Cell(" << x << "," << y << ") State=" << state;
    return oss.str();
}

std::shared_ptr<Chunk> ChunkedTileMap::createChunk(const ChunkCoord& coord,
    const glm::ivec2& worldOffset, int chunkSize, ChunkedTileMap* map) {
    return std::make_shared<Chunk>(coord, worldOffset, m_tileSize, this, chunkSize);
}

void ChunkedTileMap::setTile(int x, int y, int state) {
    if (x < 0 || x >= m_totalWidth || y < 0 || y >= m_totalHeight) return;
    int cx = x / m_chunkSize, cy = y / m_chunkSize;
    int lx = x % m_chunkSize, ly = y % m_chunkSize;
    ChunkCoord coord(cx, cy);

    Chunk* chunk = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_mutex);
        chunk = getOrCreateChunk(coord);
    }
    std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
    int idx = ly * chunk->chunkSize + lx;
    chunk->simBuffer[idx] = static_cast<uint8_t>(state);
    chunk->dirty = true;
    if (!chunk->active) {
        chunk->active = true;
        addToActiveIfMissing(coord);
    }
}

void ChunkedTileMap::setTileDirect(int x, int y, uint8_t state) {
    if (x < 0 || x >= m_totalWidth || y < 0 || y >= m_totalHeight) return;
    int cx = x / m_chunkSize, cy = y / m_chunkSize;
    int lx = x % m_chunkSize, ly = y % m_chunkSize;
    ChunkCoord coord(cx, cy);

    Chunk* chunk = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_mutex);
        chunk = getOrCreateChunk(coord);
    }
    std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
    int idx = ly * chunk->chunkSize + lx;
    chunk->renderBuffer[idx] = state;
    chunk->dirty = true;
}

void ChunkedTileMap::applyDefaultPalette() {
    std::vector<glm::vec3> pal(256);
    pal[0] = glm::vec3(0.0f); // мёртвая клетка — чёрный
    for (int i = 1; i < 256; ++i) {
        float h = (i - 1) / 255.0f;          // 0..1
        // Простой переход: красный → жёлтый → зелёный → циан → синий → малиновый → красный
        float r, g, b;
        if (h < 0.1667f) {
            r = 1.0f; g = h * 6.0f; b = 0.0f;
        }
        else if (h < 0.3333f) {
            r = 1.0f - (h - 0.1667f) * 6.0f; g = 1.0f; b = 0.0f;
        }
        else if (h < 0.5f) {
            r = 0.0f; g = 1.0f; b = (h - 0.3333f) * 6.0f;
        }
        else if (h < 0.6667f) {
            r = 0.0f; g = 1.0f - (h - 0.5f) * 6.0f; b = 1.0f;
        }
        else if (h < 0.8333f) {
            r = (h - 0.6667f) * 6.0f; g = 0.0f; b = 1.0f;
        }
        else {
            r = 1.0f; g = 0.0f; b = 1.0f - (h - 0.8333f) * 6.0f;
        }
        pal[i] = glm::vec3(r, g, b);
    }
    setPalette(pal);
}

int ChunkedTileMap::getTileState(int x, int y) const {
    if (x < 0 || x >= m_totalWidth || y < 0 || y >= m_totalHeight) return 0;
    int cx = x / m_chunkSize, cy = y / m_chunkSize;
    int lx = x % m_chunkSize, ly = y % m_chunkSize;
    ChunkCoord coord(cx, cy);
    auto it = m_chunks.find(coord);
    if (it == m_chunks.end()) return 0;
    const auto& chunk = *it->second;
    int idx = ly * chunk.chunkSize + lx;
    return static_cast<int>(chunk.renderBuffer[idx]);
}

int ChunkedTileMap::getSimTileState(int x, int y) const {
    if (x < 0 || x >= m_totalWidth || y < 0 || y >= m_totalHeight) return 0;
    int cx = x / m_chunkSize, cy = y / m_chunkSize;
    int lx = x % m_chunkSize, ly = y % m_chunkSize;
    ChunkCoord coord(cx, cy);
    auto it = m_chunks.find(coord);
    if (it == m_chunks.end()) return 0;
    const auto& chunk = *it->second;
    int idx = ly * chunk.chunkSize + lx;
    return static_cast<int>(chunk.simBuffer[idx]);
}

void ChunkedTileMap::render(const Camera2D& camera) {
    glm::vec2 viewMin, viewMax;
    camera.getVisibleAABB(viewMin, viewMax);
    float margin = m_chunkSize * m_tileSize * 2.0f;
    viewMin -= glm::vec2(margin, margin);
    viewMax += glm::vec2(margin, margin);
    AABB viewBounds{ viewMin, viewMax };

    std::shared_lock<std::shared_mutex> lock(m_mutex);
    for (auto& [coord, chunk] : m_chunks) {
        if (!chunk || !chunk->renderer) continue;
        if (!chunk->bounds.overlaps(viewBounds)) continue;

        if (chunk->dirty.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
            chunk->updateRenderData();
        }
        chunk->renderer->render(camera, chunk->worldOffset, m_tileSize);
    }
}

void ChunkedTileMap::setPalette(const std::vector<glm::vec3>& palette) {
    ChunkRenderer::setGlobalPalette(palette);
}

void ChunkedTileMap::enqueueReadyChunk(std::shared_ptr<Chunk> chunk, uint64_t generation) {
    std::lock_guard<std::mutex> lock(m_readyMutex);
    m_readyChunks.emplace(std::move(chunk), generation);
}

void ChunkedTileMap::commitReadyChunks(const Camera2D& camera) {
    std::vector<std::pair<std::shared_ptr<Chunk>, uint64_t>> readyList;
    {
        std::lock_guard<std::mutex> lock(m_readyMutex);
        while (!m_readyChunks.empty()) {
            readyList.push_back(std::move(m_readyChunks.front()));
            m_readyChunks.pop();
        }
    }
    readyList.insert(readyList.end(),
        std::make_move_iterator(m_pendingReady.begin()),
        std::make_move_iterator(m_pendingReady.end()));
    m_pendingReady.clear();

    if (readyList.empty()) return;

    std::unordered_map<uint64_t, std::vector<std::shared_ptr<Chunk>>> groups;
    for (auto& [chunk, gen] : readyList) {
        groups[gen].push_back(std::move(chunk));
    }

    glm::vec2 viewMin, viewMax;
    camera.getVisibleAABB(viewMin, viewMax);
    float margin = m_chunkSize * m_tileSize * 2.0f;
    viewMin -= glm::vec2(margin, margin);
    viewMax += glm::vec2(margin, margin);
    AABB viewBounds{ viewMin, viewMax };

    std::vector<ChunkCoord> emptyCoords;

    for (auto& [gen, chunks] : groups) {
        int expected = 0;
        bool known = false;
        {
            std::lock_guard<std::mutex> lock(m_genMutex);
            auto it = m_generationExpectedCount.find(gen);
            if (it != m_generationExpectedCount.end()) {
                known = true;
                expected = it->second;
            }
        }

        if (known && static_cast<int>(chunks.size()) < expected) {
            for (auto& c : chunks) {
                m_pendingReady.emplace_back(std::move(c), gen);
            }
            continue;
        }

        for (auto& c : chunks) {
            bool visible = c->bounds.overlaps(viewBounds);
            {
                std::lock_guard<std::mutex> lock(c->chunkMutex);
                std::swap(c->simBuffer, c->renderBuffer);
                if (visible) {
                    c->recalcLiveCells();
                    c->updateRenderData();
                    if (c->liveCells.load(std::memory_order_acquire) == 0)
                        emptyCoords.push_back(ChunkCoord(c->worldOffset.x / m_chunkSize, c->worldOffset.y / m_chunkSize));
                }
                else {
                    c->dirty.store(true, std::memory_order_release);
                }
            }
        }

        if (known) {
            std::lock_guard<std::mutex> lock(m_genMutex);
            m_generationExpectedCount.erase(gen);
        }
    }

    for (auto& coord : emptyCoords) {
        std::unique_lock<std::shared_mutex> lock(m_mutex);
        auto it = std::find(m_activeChunks.begin(), m_activeChunks.end(), coord);
        if (it != m_activeChunks.end()) m_activeChunks.erase(it);
        m_chunks.erase(coord);
    }
}

void ChunkedTileMap::simulateActiveChunks() {
    bool expected = false;
    if (!m_simulating.compare_exchange_strong(expected, true)) {
        return;
    }

    auto activeCopy = std::make_shared<std::vector<std::shared_ptr<Chunk>>>();
    {
        std::shared_lock<std::shared_mutex> lock(m_mutex);
        activeCopy->reserve(m_activeChunks.size());
        for (const auto& coord : m_activeChunks) {
            auto it = m_chunks.find(coord);
            if (it != m_chunks.end()) {
                activeCopy->push_back(it->second);
            }
        }
    }

    if (activeCopy->empty()) {
        m_simulating.store(false, std::memory_order_release);
        return;
    }

    uint64_t gen;
    {
        std::lock_guard<std::mutex> lock(m_genMutex);
        gen = ++m_currentGeneration;
        m_generationExpectedCount[gen] = static_cast<int>(activeCopy->size());
    }

    size_t total = activeCopy->size();
    size_t nw = std::max<size_t>(1, TaskScheduler::instance().thread_count());
    size_t batch = std::max<size_t>(1, std::min<size_t>(256, total / nw));

    auto tasksRemaining = std::make_shared<std::atomic<int>>(
        static_cast<int>((total + batch - 1) / batch));

    for (size_t i = 0; i < total; i += batch) {
        size_t end = std::min(i + batch, total);
        TaskScheduler::instance().schedule([this, activeCopy, i, end, gen, tasksRemaining]() {
            for (size_t j = i; j < end; ++j) {
                auto& chunk = (*activeCopy)[j];
                if (chunk) {
                    simulateChunk(*chunk);
                    enqueueReadyChunk(chunk, gen);
                }
            }
            if (tasksRemaining->fetch_sub(1) == 1) {
                m_simulating.store(false, std::memory_order_release);
            }
            });
    }
}

void ChunkedTileMap::commitOverlayRender() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    for (auto& [coord, chunk] : m_chunks) {
        if (chunk->dirty.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
            chunk->updateRenderData();
        }
    }
}

void ChunkedTileMap::restoreAllRenderBuffers() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    for (auto& [coord, chunk] : m_chunks) {
        std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
        chunk->commitChanges();
        chunk->updateRenderData();
    }
}

void ChunkedTileMap::clearAllRenderTiles(uint8_t value) {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    for (auto& [coord, chunk] : m_chunks) {
        std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
        auto& buf = chunk->renderBuffer;
        for (auto& v : buf) v = value;
        chunk->dirty = true;
        chunk->updateRenderData();
    }
}

std::vector<ChunkCoord> ChunkedTileMap::snapshotActiveChunks() const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    return m_activeChunks;
}

void ChunkedTileMap::commitInitialState() {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    for (auto& [coord, chunk] : m_chunks) {
        chunk->commitChanges();
        chunk->updateRenderData();
    }
}

std::vector<ChunkCoord> ChunkedTileMap::updateActiveStatus(const std::vector<ChunkCoord>& coords) {
    std::vector<ChunkCoord> becameEmpty;
    for (const auto& coord : coords) {
        auto it = m_chunks.find(coord);
        if (it == m_chunks.end()) continue;
        Chunk& chunk = *it->second;
        bool nowActive = isChunkActive(chunk);
        if (nowActive != chunk.active) {
            chunk.active = nowActive;
            if (nowActive) addToActiveIfMissing(coord);
            else {
                removeFromActive(coord);
                becameEmpty.push_back(coord);
            }
        }
    }
    return becameEmpty;
}

bool ChunkedTileMap::isChunkActive(const Chunk& chunk) const {
    for (uint8_t v : chunk.renderBuffer)
        if (v != 0) return true;
    return false;
}

void ChunkedTileMap::addToActiveIfMissing(const ChunkCoord& coord) {
    auto it = std::find(m_activeChunks.begin(), m_activeChunks.end(), coord);
    if (it == m_activeChunks.end()) m_activeChunks.push_back(coord);
}

void ChunkedTileMap::removeFromActive(const ChunkCoord& coord) {
    auto it = std::find(m_activeChunks.begin(), m_activeChunks.end(), coord);
    if (it != m_activeChunks.end()) m_activeChunks.erase(it);
}

void ChunkedTileMap::removeEmptyChunks(const std::vector<ChunkCoord>& coords) {
    for (const auto& coord : coords) {
        auto it = m_chunks.find(coord);
        if (it != m_chunks.end() && isChunkActive(*it->second) == false) {
            m_chunks.erase(it);
        }
    }
}

ChunkCoord ChunkedTileMap::getChunkCoord(const glm::ivec2& worldOffset) const {
    return ChunkCoord(worldOffset.x / m_chunkSize, worldOffset.y / m_chunkSize);
}

std::vector<uint8_t> ChunkedTileMap::getExtendedNeighborhood(const Chunk& chunk, int border) const {
    const int S = chunk.chunkSize;
    const int extSize = S + 2 * border;
    std::vector<uint8_t> ext(extSize * extSize, 0);

    int cx = chunk.worldOffset.x / S;
    int cy = chunk.worldOffset.y / S;

    // Копируем центральный чанк (под мьютексом, чтобы избежать гонки)
    {
        std::lock_guard<std::mutex> lock(const_cast<Chunk&>(chunk).chunkMutex);
        for (int y = 0; y < S; ++y) {
            const uint8_t* src = &chunk.renderBuffer[y * S];
            uint8_t* dst = &ext[(y + border) * extSize + border];
            std::copy(src, src + S, dst);
        }
    }

    // Проходим по соседям
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            ChunkCoord ncoord(cx + dx, cy + dy);
            auto neighbor = getChunkShared(ncoord);
            if (!neighbor) continue;

            int srcXStart, srcYStart, dstXStart, dstYStart, copyW, copyH;
            if (dx == -1) { srcXStart = S - border; dstXStart = 0; copyW = border; }
            else if (dx == 1) { srcXStart = 0; dstXStart = S + border; copyW = border; }
            else { srcXStart = 0; dstXStart = border; copyW = S; }

            if (dy == -1) { srcYStart = S - border; dstYStart = 0; copyH = border; }
            else if (dy == 1) { srcYStart = 0; dstYStart = S + border; copyH = border; }
            else { srcYStart = 0; dstYStart = border; copyH = S; }

            const auto& srcBuf = neighbor->renderBuffer;
            for (int row = 0; row < copyH; ++row) {
                for (int col = 0; col < copyW; ++col) {
                    ext[(dstYStart + row) * extSize + (dstXStart + col)] =
                        srcBuf[(srcYStart + row) * S + (srcXStart + col)];
                }
            }
        }
    }
    return ext;
}