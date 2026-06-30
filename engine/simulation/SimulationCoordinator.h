// engine/simulation/SimulationCoordinator.h
#pragma once
#include "engine/chunk/Chunk.h"
#include "engine/chunk/ChunkStore.h"
#include "engine/graphics/Camera2D.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Оркестратор одного шага клеточного автомата над набором активных чанков.
// Вынесено из ChunkedTileMap: здесь живут поколения (generations), раскидывание
// задач по TaskScheduler, очередь "готовых" чанков и их commit в рендер-буфер.
//
// Координатор НЕ знает правило автомата — он принимает функтор simulateOne,
// который считает один чанк (это виртуальный simulateChunk у карты). Так
// сохраняется инверсия зависимостей: оркестрация отделена от вычисления правила.
class SimulationCoordinator {
public:
    using SimulateOne = std::function<void(Chunk&)>;
    // Создать (если нужно) и активировать перечисленные чанки. Через колбэк, т.к.
    // создание чанка — забота карты (виртуальный createChunk + проверка границ мира).
    using EnsureActive = std::function<void(const std::vector<ChunkCoord>&)>;

    SimulationCoordinator(ChunkStore& store, int chunkSize, float tileSize,
                          SimulateOne simulateOne, EnsureActive ensureActive)
        : m_store(store), m_chunkSize(chunkSize), m_tileSize(tileSize),
          m_simulateOne(std::move(simulateOne)),
          m_ensureActive(std::move(ensureActive)) {}

    // Асинхронный шаг: ставит флаг "идёт симуляция" и раскидывает чанки по пулу.
    // Повторный вызов во время незавершённой симуляции — no-op.
    void simulateActive();

    // Синхронный шаг: запускает один шаг и блокируется до завершения всех задач.
    void simulateAndWait();

    // Переносит готовые (просчитанные) чанки в рендер-буфер с учётом видимости
    // камерой. Пустые чанки удаляются из хранилища.
    void commitReady(const Camera2D& camera);

    // Жёсткий сброс пайплайна: дождаться завершения текущего шага и выбросить все
    // ещё не закоммиченные чанки и учёт поколений. Нужен перед очисткой/перегенерацией
    // поля, иначе "старые" готовые чанки откатят новое состояние при следующем commit.
    void reset();

    bool isSimulating() const { return m_phase.load(std::memory_order_acquire) != Phase::Idle; }

private:
    void enqueueReady(std::shared_ptr<Chunk> chunk, uint64_t generation);
    void collectBorderNeighbors(const Chunk& c, std::unordered_set<ChunkCoord>& out) const;

    // Фазы шага. Барьер: новый расчёт начинается только из Idle, а commit — только
    // из ReadyToCommit. Это разводит вычисление и перенос буферов во времени, чтобы
    // сбор соседей никогда не читал чанки, часть которых уже переключена на новое
    // поколение, а часть — нет (иначе рваная, неравномерная симуляция на границах).
    enum class Phase { Idle, Computing, ReadyToCommit, Committing };

    ChunkStore& m_store;
    int         m_chunkSize;
    float       m_tileSize;
    SimulateOne m_simulateOne;
    EnsureActive m_ensureActive;

    std::atomic<Phase> m_phase{ Phase::Idle };
    uint64_t          m_currentGeneration = 0;
    std::unordered_map<uint64_t, int> m_generationExpectedCount;
    std::mutex        m_genMutex;

    std::queue<std::pair<std::shared_ptr<Chunk>, uint64_t>> m_readyChunks;
    std::mutex m_readyMutex;
    std::vector<std::pair<std::shared_ptr<Chunk>, uint64_t>> m_pendingReady;
};
