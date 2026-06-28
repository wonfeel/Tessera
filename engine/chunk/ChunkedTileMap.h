// engine/chunk/ChunkedTileMap.h
#pragma once
#include "engine/tilemap/IndexedTileMap.h"
#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkCoord.h"
#include "engine/graphics/Camera2D.h"
#include <unordered_map>
#include <shared_mutex>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <memory>

class ChunkedTileMap : public IIndexedTileMap {
public:
    Chunk* getChunk(ChunkCoord coord);
    std::shared_ptr<Chunk> getChunkShared(ChunkCoord coord) const;

    ChunkedTileMap(int totalWidth, int totalHeight, int chunkSize, float tileSize);
    virtual ~ChunkedTileMap() = default;

    int getWidth() const override { return m_totalWidth; }
    int getHeight() const override { return m_totalHeight; }
    float getTileSize() const override { return m_tileSize; }
    int getChunkSize() const { return m_chunkSize; }

    const std::vector<uint8_t>& getTileIndices() const override {
        static std::vector<uint8_t> dummy;
        return dummy;
    }
    bool isDirty() const override { return false; }
    void clearDirty() override {}

    std::string getTileInfo(int x, int y) const override;
    void setTile(int x, int y, int state) override;
    int getTileState(int x, int y) const override;
    int getSimTileState(int x, int y) const;

    void render(const Camera2D& camera);
    void commitReadyChunks(const Camera2D& camera);
    virtual void simulateActiveChunks();

    const std::vector<ChunkCoord>& getActiveChunks() const { return m_activeChunks; }
    virtual void simulateChunk(Chunk& chunk) {}

    Chunk* getOrCreateChunk(ChunkCoord coord);
    void setPalette(const std::vector<glm::vec3>& palette);
    void applyDefaultPalette();
    virtual void commitInitialState();

    void setTileDirect(int x, int y, uint8_t state);
    void commitOverlayRender();
    void restoreAllRenderBuffers();
    void clearAllRenderTiles(uint8_t value = 0);

    virtual std::shared_ptr<Chunk> createChunk(const ChunkCoord& coord,
        const glm::ivec2& worldOffset,
        int chunkSize,
        ChunkedTileMap* map);
    ChunkCoord getChunkCoord(const glm::ivec2& worldOffset) const;
    std::unordered_map<ChunkCoord, std::shared_ptr<Chunk>> m_chunks;

    void enqueueReadyChunk(std::shared_ptr<Chunk> chunk, uint64_t generation);

    // Автоматическое построение расширенного буфера с учётом соседей.
    // Возвращает буфер размером (chunkSize + 2*border) ^ 2, заполненный данными
    // из renderBuffer чанка и его соседей. Отсутствующие чанки считаются пустыми.
    std::vector<uint8_t> getExtendedNeighborhood(const Chunk& chunk, int border = 1) const;

protected:
    int m_totalWidth, m_totalHeight;
    int m_chunkSize;
    float m_tileSize;

    std::vector<ChunkCoord> m_activeChunks;
    mutable std::shared_mutex m_mutex;

    bool isChunkActive(const Chunk& chunk) const;
    std::vector<ChunkCoord> snapshotActiveChunks() const;

    std::vector<ChunkCoord> updateActiveStatus(const std::vector<ChunkCoord>& coords);
    void addToActiveIfMissing(const ChunkCoord& coord);
    void removeFromActive(const ChunkCoord& coord);
    void removeEmptyChunks(const std::vector<ChunkCoord>& coords);

    // Адаптивная симуляция с поколениями
    std::atomic<bool> m_simulating{ false };
    uint64_t m_currentGeneration = 0;
    std::unordered_map<uint64_t, int> m_generationExpectedCount;
    std::mutex m_genMutex;

private:
    std::queue<std::pair<std::shared_ptr<Chunk>, uint64_t>> m_readyChunks;
    std::mutex m_readyMutex;
    std::vector<std::pair<std::shared_ptr<Chunk>, uint64_t>> m_pendingReady;
};