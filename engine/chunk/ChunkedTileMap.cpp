// engine/chunk/ChunkedTileMap.cpp
#include "engine/chunk/ChunkedTileMap.h"
#include "engine/core/TaskScheduler.h"
#include <algorithm>
#include <sstream>
#include <thread>
#include <glm/glm.hpp>

ChunkedTileMap::ChunkedTileMap(int totalWidth, int totalHeight, int chunkSize, float tileSize)
    : m_grid(totalWidth, totalHeight, chunkSize, tileSize),
      m_sim(m_store, chunkSize, tileSize,
            [this](Chunk& c) { simulateChunk(c); },
            [this](const std::vector<ChunkCoord>& coords) { ensureActiveChunks(coords); }),
      m_renderer(m_store, chunkSize, tileSize)
{
}

Chunk* ChunkedTileMap::getOrCreateChunk(ChunkCoord coord) {
    return m_store.getOrCreateLocked(coord, [this](ChunkCoord c) {
        int cs = m_grid.chunkSize();
        glm::ivec2 worldOffset(c.x() * cs, c.y() * cs);
        return createChunk(c, worldOffset, cs, this);
    });
}

Chunk* ChunkedTileMap::getChunk(ChunkCoord coord) {
    return m_store.get(coord);
}

std::shared_ptr<Chunk> ChunkedTileMap::getChunkShared(ChunkCoord coord) const {
    return m_store.getShared(coord);
}

std::string ChunkedTileMap::getTileInfo(int x, int y) const {
    if (!m_grid.inBounds(x, y)) return "";
    int state = getTileState(x, y);
    std::ostringstream oss;
    oss << "Cell(" << x << "," << y << ") State=" << state;
    return oss.str();
}

std::shared_ptr<Chunk> ChunkedTileMap::createChunk(const ChunkCoord& coord,
    const glm::ivec2& worldOffset, int chunkSize, ChunkedTileMap* map) {
    return std::make_shared<Chunk>(coord, worldOffset, m_grid.tileSize(), this, chunkSize);
}

void ChunkedTileMap::setTile(int x, int y, int state) {
    if (!m_grid.inBounds(x, y)) return;
    CellLocation loc = m_grid.locate(x, y);

    Chunk* chunk = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_store.mutex());
        chunk = getOrCreateChunk(loc.coord);
    }
    std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
    chunk->simBuffer[loc.index(chunk->chunkSize)] = static_cast<uint8_t>(state);
    chunk->dirty = true;
    if (!chunk->active) {
        chunk->active = true;
        m_store.addActiveIfMissing(loc.coord);
    }
}

void ChunkedTileMap::setTileDirect(int x, int y, uint8_t state) {
    if (!m_grid.inBounds(x, y)) return;
    CellLocation loc = m_grid.locate(x, y);

    Chunk* chunk = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_store.mutex());
        chunk = getOrCreateChunk(loc.coord);
    }
    std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
    chunk->renderBuffer[loc.index(chunk->chunkSize)] = state;
    chunk->dirty = true;
}

void ChunkedTileMap::ensureActiveChunks(const std::vector<ChunkCoord>& coords) {
    const int cs = m_grid.chunkSize();
    std::unique_lock<std::shared_mutex> lock(m_store.mutex());
    for (const ChunkCoord& coord : coords) {
        // Не выходим за границы мира.
        int ox = coord.x() * cs, oy = coord.y() * cs;
        if (ox < 0 || oy < 0 || ox >= m_grid.width() || oy >= m_grid.height())
            continue;
        Chunk* c = getOrCreateChunk(coord);
        if (!c->active) {
            c->active = true;
            m_store.addActiveIfMissing(coord);
        }
    }
}

void ChunkedTileMap::paintBrush(int tileX, int tileY, int size, uint8_t state) {
    int half = size / 2;
    for (int dy = -half; dy <= half; ++dy)
        for (int dx = -half; dx <= half; ++dx)
            paintTile(tileX + dx, tileY + dy, state);
}

void ChunkedTileMap::paintTile(int x, int y, uint8_t state) {
    if (!m_grid.inBounds(x, y)) return;
    CellLocation loc = m_grid.locate(x, y);

    Chunk* chunk = nullptr;
    {
        std::shared_lock<std::shared_mutex> lock(m_store.mutex());
        chunk = getOrCreateChunk(loc.coord);
    }
    std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
    chunk->renderBuffer[loc.index(chunk->chunkSize)] = state;
    chunk->recalcLiveCells();   // держим liveCells в согласии с renderBuffer
    chunk->dirty = true;
    if (!chunk->active) {
        chunk->active = true;
        m_store.addActiveIfMissing(loc.coord);
    }
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
    if (!m_grid.inBounds(x, y)) return 0;
    CellLocation loc = m_grid.locate(x, y);
    auto it = m_store.map().find(loc.coord);
    if (it == m_store.map().end()) return 0;
    const auto& chunk = *it->second;
    return static_cast<int>(chunk.renderBuffer[loc.index(chunk.chunkSize)]);
}

int ChunkedTileMap::getSimTileState(int x, int y) const {
    if (!m_grid.inBounds(x, y)) return 0;
    CellLocation loc = m_grid.locate(x, y);
    auto it = m_store.map().find(loc.coord);
    if (it == m_store.map().end()) return 0;
    const auto& chunk = *it->second;
    return static_cast<int>(chunk.simBuffer[loc.index(chunk.chunkSize)]);
}

void ChunkedTileMap::render(const Camera2D& camera) {
    m_renderer.render(camera);
}

void ChunkedTileMap::setPalette(const std::vector<glm::vec3>& palette) {
    ChunkMapRenderer::setPalette(palette);
}

void ChunkedTileMap::commitReadyChunks(const Camera2D& camera) {
    m_sim.commitReady(camera);
}

void ChunkedTileMap::stampPattern(const RlePattern& pattern, int worldX, int worldY) {
    if (pattern.width <= 0 || pattern.height <= 0) return;
    for (int py = 0; py < pattern.height; ++py) {
        for (int px = 0; px < pattern.width; ++px) {
            uint8_t cell = pattern.cells[py * pattern.width + px];
            if (cell != 0)
                setTile(worldX + px, worldY + py, cell);
        }
    }
}

bool ChunkedTileMap::simulateActiveChunks() {
    return m_sim.simulateActive();
}

void ChunkedTileMap::simulateAndWait() {
    m_sim.simulateAndWait();
}

void ChunkedTileMap::commitOverlayRender() {
    std::shared_lock<std::shared_mutex> lock(m_store.mutex());
    for (auto& [coord, chunk] : m_store.map()) {
        if (chunk->dirty.load(std::memory_order_acquire)) {
            std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
            chunk->updateRenderData();
        }
    }
}

void ChunkedTileMap::restoreAllRenderBuffers() {
    std::shared_lock<std::shared_mutex> lock(m_store.mutex());
    for (auto& [coord, chunk] : m_store.map()) {
        std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
        chunk->commitChanges();
        chunk->updateRenderData();
    }
}

void ChunkedTileMap::clearAll() {
    // Сначала погасить асинхронный пайплайн: иначе уже просчитанные чанки из
    // очереди закоммитятся после очистки и вернут старое поле.
    m_sim.reset();
    std::unique_lock<std::shared_mutex> lock(m_store.mutex());
    for (auto& [coord, chunk] : m_store.map()) {
        std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
        std::fill(chunk->simBuffer.begin(), chunk->simBuffer.end(), uint8_t{0});
        std::fill(chunk->renderBuffer.begin(), chunk->renderBuffer.end(), uint8_t{0});
        chunk->liveCells.store(0, std::memory_order_release);
        chunk->active = false;
        chunk->dirty.store(true, std::memory_order_release);  // рендер обновит GL
    }
    m_store.active().clear();
}

void ChunkedTileMap::clearAllRenderTiles(uint8_t value) {
    std::shared_lock<std::shared_mutex> lock(m_store.mutex());
    for (auto& [coord, chunk] : m_store.map()) {
        std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
        auto& buf = chunk->renderBuffer;
        for (auto& v : buf) v = value;
        chunk->dirty = true;
        chunk->updateRenderData();
    }
}

std::vector<ChunkCoord> ChunkedTileMap::snapshotActiveChunks() const {
    std::shared_lock<std::shared_mutex> lock(m_store.mutex());
    return m_store.active();
}

void ChunkedTileMap::commitInitialState() {
    std::shared_lock<std::shared_mutex> lock(m_store.mutex());
    for (auto& [coord, chunk] : m_store.map()) {
        std::lock_guard<std::mutex> cellLock(chunk->chunkMutex);
        // commitChanges() ставит dirty=true; саму загрузку в GL делает рендер-поток
        // (он владеет контекстом). Вызывать updateRenderData() здесь нельзя — этот
        // метод дёргается и из update-потока (randomize/load scene), где GL-контекста нет.
        chunk->commitChanges();
    }
}

std::vector<ChunkCoord> ChunkedTileMap::updateActiveStatus(const std::vector<ChunkCoord>& coords) {
    std::vector<ChunkCoord> becameEmpty;
    for (const auto& coord : coords) {
        auto it = m_store.map().find(coord);
        if (it == m_store.map().end()) continue;
        Chunk& chunk = *it->second;
        bool nowActive = isChunkActive(chunk);
        if (nowActive != chunk.active) {
            chunk.active = nowActive;
            if (nowActive) m_store.addActiveIfMissing(coord);
            else {
                m_store.removeActive(coord);
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

void ChunkedTileMap::removeEmptyChunks(const std::vector<ChunkCoord>& coords) {
    for (const auto& coord : coords) {
        auto it = m_store.map().find(coord);
        if (it != m_store.map().end() && isChunkActive(*it->second) == false) {
            m_store.map().erase(it);
        }
    }
}

ChunkCoord ChunkedTileMap::getChunkCoord(const glm::ivec2& worldOffset) const {
    return m_grid.chunkOfWorldOffset(worldOffset);
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