// engine/chunk/ChunkStore.h
#pragma once
#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkCoord.h"
#include <unordered_map>
#include <shared_mutex>
#include <vector>
#include <memory>
#include <functional>

// Владеет набором живых чанков, мьютексом, который их защищает, и списком
// "активных" (непустых, симулируемых) координат. Выделено из ChunkedTileMap,
// чтобы хранение и поиск чанков были одной понятной ответственностью.
//
// Локинг намеренно отдан наружу: коллабораторы (рендерер, координатор симуляции)
// берут mutex() сами и теми же типами блокировок, что и раньше, — это сохраняет
// прежнюю семантику синхронизации после разбиения god-класса.
class ChunkStore {
public:
    using ChunkPtr = std::shared_ptr<Chunk>;
    using Factory  = std::function<ChunkPtr(ChunkCoord)>;

    std::shared_mutex& mutex() const { return m_mutex; }

    // --- Поиск (берут shared-блокировку сами) ---
    Chunk*   get(ChunkCoord coord);
    ChunkPtr getShared(ChunkCoord coord) const;

    // Найти или создать чанк через фабрику. Вызывающий ДОЛЖЕН уже держать
    // mutex() (как это делал прежний getOrCreateChunk под shared-блокировкой).
    Chunk* getOrCreateLocked(ChunkCoord coord, const Factory& factory);

    // Прямой доступ к контейнеру для итерации — вызывающий сам берёт mutex().
    std::unordered_map<ChunkCoord, ChunkPtr>&       map()       { return m_chunks; }
    const std::unordered_map<ChunkCoord, ChunkPtr>& map() const { return m_chunks; }

    // --- Список активных чанков (вызывающий держит подходящую блокировку) ---
    std::vector<ChunkCoord>&       active()       { return m_activeChunks; }
    const std::vector<ChunkCoord>& active() const { return m_activeChunks; }
    void addActiveIfMissing(const ChunkCoord& coord);
    void removeActive(const ChunkCoord& coord);

    // Удалить чанк и его запись в списке активных (вызывающий держит unique-блокировку).
    void eraseLocked(const ChunkCoord& coord);

private:
    std::unordered_map<ChunkCoord, ChunkPtr> m_chunks;
    mutable std::shared_mutex                m_mutex;
    std::vector<ChunkCoord>                  m_activeChunks;
};
