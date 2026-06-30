// engine/chunk/ChunkedTileMap.h
#pragma once
#include "engine/tilemap/IndexedTileMap.h"
#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkStore.h"
#include "engine/chunk/ChunkGrid.h"
#include "engine/chunk/ChunkCoord.h"
#include "engine/chunk/ChunkMapRenderer.h"
#include "engine/simulation/SimulationCoordinator.h"
#include "engine/graphics/Camera2D.h"
#include "engine/utils/RleLoader.h"
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

    int getWidth() const override { return m_grid.width(); }
    int getHeight() const override { return m_grid.height(); }
    float getTileSize() const override { return m_grid.tileSize(); }
    int getChunkSize() const { return m_grid.chunkSize(); }

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

    // Синхронный шаг: запускает симуляцию одного поколения и блокируется,
    // пока все задачи не завершатся. Удобно для детерминированного оффлайн-
    // прогона (захват кадров, тесты), где нужен предсказуемый порядок шагов.
    void simulateAndWait();

    const std::vector<ChunkCoord>& getActiveChunks() const { return m_store.active(); }
    virtual void simulateChunk(Chunk& chunk) {}

    Chunk* getOrCreateChunk(ChunkCoord coord);
    void setPalette(const std::vector<glm::vec3>& palette);
    void applyDefaultPalette();
    virtual void commitInitialState();

    void setTileDirect(int x, int y, uint8_t state);

    // Рисование "живой" клетки во время работы/паузы: пишет напрямую в renderBuffer,
    // поэтому видно сразу (даже на паузе, без шага симуляции), и помечает чанк
    // активным, чтобы клетка участвовала в следующем шаге. В отличие от setTile,
    // который пишет в simBuffer (вход следующего поколения) и проявляется только
    // после commit, т.е. на следующем шаге.
    void paintTile(int x, int y, uint8_t state);

    // Создать (если нужно) и активировать перечисленные чанки в пределах мира.
    // Использует координатор для активации соседей, в которые перетекает жизнь.
    void ensureActiveChunks(const std::vector<ChunkCoord>& coords);

    // Рисование кистью size×size клеток с центром в тайле (tileX,tileY).
    void paintBrush(int tileX, int tileY, int size, uint8_t state);

    // Перевод мировой позиции (в пикселях) в координаты тайла.
    glm::ivec2 worldToTile(const glm::vec2& world) const {
        float ts = m_grid.tileSize();
        return glm::ivec2(static_cast<int>(world.x / ts), static_cast<int>(world.y / ts));
    }

    // Штамповать RLE-паттерн с верхним левым углом в (worldX, worldY).
    // Клетки вне границ мира молча пропускаются.
    void stampPattern(const RlePattern& pattern, int worldX, int worldY);
    void commitOverlayRender();
    void restoreAllRenderBuffers();
    void clearAllRenderTiles(uint8_t value = 0);

    // Полностью очищает поле: оба буфера всех чанков -> 0, список активных
    // чанков очищается. Без GL-вызовов (только помечает dirty), поэтому
    // безопасно звать из update-потока; рендер обновит GL по dirty.
    void clearAll();

    virtual std::shared_ptr<Chunk> createChunk(const ChunkCoord& coord,
        const glm::ivec2& worldOffset,
        int chunkSize,
        ChunkedTileMap* map);
    ChunkCoord getChunkCoord(const glm::ivec2& worldOffset) const;

    // Автоматическое построение расширенного буфера с учётом соседей.
    // Возвращает буфер размером (chunkSize + 2*border) ^ 2, заполненный данными
    // из renderBuffer чанка и его соседей. Отсутствующие чанки считаются пустыми.
    std::vector<uint8_t> getExtendedNeighborhood(const Chunk& chunk, int border = 1) const;

protected:
    // Геометрия сетки: размеры мира, размер чанка/тайла и перевод координат.
    ChunkGrid m_grid;

    // Хранилище чанков: владеет картой чанков, защищающим её мьютексом и списком
    // активных координат. Доступ к мьютексу — через m_store.mutex().
    ChunkStore m_store;

    // Оркестратор шага симуляции (поколения, планирование, commit). Объявлен
    // после m_store, т.к. конструируется ссылкой на него.
    SimulationCoordinator m_sim;

    // Рисование видимых чанков. Тоже держит ссылку на m_store.
    ChunkMapRenderer m_renderer;

    bool isChunkActive(const Chunk& chunk) const;
    std::vector<ChunkCoord> snapshotActiveChunks() const;

    std::vector<ChunkCoord> updateActiveStatus(const std::vector<ChunkCoord>& coords);
    void removeEmptyChunks(const std::vector<ChunkCoord>& coords);
};