// engine/chunk/ChunkStore.h
#pragma once
#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkCoord.h"
#include <unordered_map>
#include <unordered_set>
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

    // Найти или создать чанк через фабрику, взяв блокировку САМОСТОЯТЕЛЬНО.
    //
    // Это замена прежней схемы, где вызывающий брал shared_lock и звал
    // getOrCreateLocked() под ней. Схема была сломана: getOrCreateLocked()
    // ВСТАВЛЯЕТ в m_chunks, а shared_lock эксклюзивности не даёт - несколько
    // потоков (UI-рисование, ensureActiveChunks) могли вставлять одновременно,
    // и любая из вставок могла вызвать рехеш unordered_map прямо под ногами
    // у параллельного find() из симуляции, то есть порчу памяти, а не просто
    // устаревшее чтение. Хуже того, прежний комментарий здесь закреплял этот
    // контракт как намеренный ("вызывающий ДОЛЖЕН держать mutex()"), из-за
    // чего баг выглядел как решение.
    //
    // Внутри - обычный double-checked путь: сначала дёшево пробуем найти под
    // shared-блокировкой (типичный случай - чанк уже есть), и только если
    // его нет, берём unique и проверяем ПОВТОРНО, потому что между снятием
    // shared и захватом unique чанк мог создать другой поток (std::shared_mutex
    // не умеет атомарно повышать блокировку).
    Chunk* getOrCreate(ChunkCoord coord, const Factory& factory);

    // Вариант для вызывающего, который УЖЕ держит unique-блокировку mutex()
    // (например, ensureActiveChunks обрабатывает пачку координат под одним
    // локом). Именно unique, не shared: метод пишет в m_chunks.
    Chunk* getOrCreateLocked(ChunkCoord coord, const Factory& factory);

    // Прямой доступ к контейнеру для итерации — вызывающий сам берёт mutex().
    std::unordered_map<ChunkCoord, ChunkPtr>&       map()       { return m_chunks; }
    const std::unordered_map<ChunkCoord, ChunkPtr>& map() const { return m_chunks; }

    // --- Список активных чанков (вызывающий держит подходящую блокировку) ---
    // unordered_set вместо vector: активация/деактивация чанка — O(1) в среднем,
    // а не линейный поиск по всему списку на каждый шаг симуляции.
    std::unordered_set<ChunkCoord>&       active()       { return m_activeChunks; }
    const std::unordered_set<ChunkCoord>& active() const { return m_activeChunks; }
    // Требуют, чтобы вызывающий держал unique-блокировку: обе пишут в
    // m_activeChunks.
    void addActiveIfMissing(const ChunkCoord& coord);
    void removeActive(const ChunkCoord& coord);

    // То же, но берёт unique-блокировку само - для вызывающих, которые в этот
    // момент никакой блокировки store не держат (setTile/paintTile: они
    // работают под mutex'ом КОНКРЕТНОГО чанка, что множество m_activeChunks
    // никак не защищает - раньше вставка в него шла вообще без блокировки).
    void markActive(const ChunkCoord& coord);

    // Удалить чанк и его запись в списке активных (вызывающий держит unique-блокировку).
    void eraseLocked(const ChunkCoord& coord);

private:
    std::unordered_map<ChunkCoord, ChunkPtr> m_chunks;
    mutable std::shared_mutex                m_mutex;
    std::unordered_set<ChunkCoord>           m_activeChunks;
};
