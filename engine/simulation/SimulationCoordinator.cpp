// engine/simulation/SimulationCoordinator.cpp
#include "engine/simulation/SimulationCoordinator.h"
#include "engine/core/TaskScheduler.h"
#include <algorithm>
#include <thread>
#include <glm/glm.hpp>

void SimulationCoordinator::enqueueReady(std::shared_ptr<Chunk> chunk, uint64_t generation) {
    std::lock_guard<std::mutex> lock(m_readyMutex);
    m_readyChunks.emplace(std::move(chunk), generation);
}

void SimulationCoordinator::simulateActive() {
    // Стартуем новое поколение только если предыдущее полностью закоммичено (Idle).
    Phase expected = Phase::Idle;
    if (!m_phase.compare_exchange_strong(expected, Phase::Computing)) {
        return;
    }

    auto activeCopy = std::make_shared<std::vector<std::shared_ptr<Chunk>>>();
    {
        std::shared_lock<std::shared_mutex> lock(m_store.mutex());
        activeCopy->reserve(m_store.active().size());
        for (const auto& coord : m_store.active()) {
            auto it = m_store.map().find(coord);
            if (it != m_store.map().end()) {
                activeCopy->push_back(it->second);
            }
        }
    }

    if (activeCopy->empty()) {
        m_phase.store(Phase::Idle, std::memory_order_release);
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
                    m_simulateOne(*chunk);
                    enqueueReady(chunk, gen);
                }
            }
            if (tasksRemaining->fetch_sub(1) == 1) {
                // Все чанки посчитаны и поставлены в очередь — поколение готово к commit.
                m_phase.store(Phase::ReadyToCommit, std::memory_order_release);
            }
            });
    }
}

void SimulationCoordinator::simulateAndWait() {
    // simulateActive() переводит фазу в Computing и раскидывает задачи; последняя
    // завершившаяся задача выставляет ReadyToCommit. Если активных чанков нет —
    // фаза вернётся в Idle сразу. Ждём, пока идёт расчёт.
    simulateActive();
    while (m_phase.load(std::memory_order_acquire) == Phase::Computing)
        std::this_thread::yield();
}

void SimulationCoordinator::reset() {
    // Дождаться, пока фаза не выйдет из активных стадий (расчёт/commit), иначе
    // параллельные задачи до-enqueue'ят чанки уже после дренажа.
    Phase p = m_phase.load(std::memory_order_acquire);
    while (p == Phase::Computing || p == Phase::Committing) {
        std::this_thread::yield();
        p = m_phase.load(std::memory_order_acquire);
    }

    {
        std::lock_guard<std::mutex> lock(m_readyMutex);
        std::queue<std::pair<std::shared_ptr<Chunk>, uint64_t>> empty;
        std::swap(m_readyChunks, empty);
        m_pendingReady.clear();
    }
    {
        std::lock_guard<std::mutex> lock(m_genMutex);
        m_generationExpectedCount.clear();
    }
    // Вернуть в Idle (могли быть в ReadyToCommit с выброшенными чанками).
    m_phase.store(Phase::Idle, std::memory_order_release);
}

void SimulationCoordinator::commitReady(const Camera2D& camera) {
    // Коммитим только полностью посчитанное поколение. Пока идёт расчёт (Computing)
    // или нечего коммитить (Idle) — выходим, не трогая буферы. Это и есть вторая
    // половина барьера: commit и compute не пересекаются во времени.
    Phase expected = Phase::ReadyToCommit;
    if (!m_phase.compare_exchange_strong(expected, Phase::Committing)) {
        return;
    }

    std::vector<std::pair<std::shared_ptr<Chunk>, uint64_t>> readyList;
    {
        // m_pendingReady тоже под m_readyMutex: иначе reset() из update-потока
        // гоняется с этим доступом из рендер-потока.
        std::lock_guard<std::mutex> lock(m_readyMutex);
        while (!m_readyChunks.empty()) {
            readyList.push_back(std::move(m_readyChunks.front()));
            m_readyChunks.pop();
        }
        readyList.insert(readyList.end(),
            std::make_move_iterator(m_pendingReady.begin()),
            std::make_move_iterator(m_pendingReady.end()));
        m_pendingReady.clear();
    }

    if (readyList.empty()) {
        m_phase.store(Phase::Idle, std::memory_order_release);
        return;
    }

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
    // Координаты соседних чанков, в которые перетекает жизнь с границы (их нужно
    // создать/активировать, чтобы на следующем шаге они приняли клетки).
    std::unordered_set<ChunkCoord> activateNeighbors;

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
            std::lock_guard<std::mutex> lock(m_readyMutex);
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
                // Жизнь, вышедшая на границу чанка, должна перетечь в соседей —
                // собираем их координаты для активации (для всех чанков, не только
                // видимых, иначе off-screen эволюция замирает на границах).
                collectBorderNeighbors(*c, activateNeighbors);
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
        // Не удалять чанк, в который как раз перетекает жизнь с соседней границы.
        if (activateNeighbors.count(coord)) continue;
        std::unique_lock<std::shared_mutex> lock(m_store.mutex());
        m_store.eraseLocked(coord);
    }

    // Создать/активировать соседей, принимающих перетекающую жизнь, — чтобы они
    // участвовали в следующем шаге. Фильтрацию по границам мира делает карта.
    if (m_ensureActive && !activateNeighbors.empty()) {
        std::vector<ChunkCoord> coords(activateNeighbors.begin(), activateNeighbors.end());
        m_ensureActive(coords);
    }

    // Поколение закоммичено — снова можно считать следующее.
    m_phase.store(Phase::Idle, std::memory_order_release);
}

// Сканирует границу renderBuffer чанка; для каждой стороны/угла с живой клеткой
// добавляет координату соответствующего соседнего чанка. Вызывается под chunkMutex.
void SimulationCoordinator::collectBorderNeighbors(const Chunk& c,
                                                   std::unordered_set<ChunkCoord>& out) const {
    const int S = c.chunkSize;
    const auto& rb = c.renderBuffer;
    const int cx = c.worldOffset.x / m_chunkSize;
    const int cy = c.worldOffset.y / m_chunkSize;

    bool left = false, right = false, top = false, bottom = false;
    for (int i = 0; i < S; ++i) {
        if (rb[i * S + 0])         left = true;
        if (rb[i * S + (S - 1)])   right = true;
        if (rb[0 * S + i])         top = true;
        if (rb[(S - 1) * S + i])   bottom = true;
    }
    if (left)   out.insert(ChunkCoord(cx - 1, cy));
    if (right)  out.insert(ChunkCoord(cx + 1, cy));
    if (top)    out.insert(ChunkCoord(cx, cy - 1));
    if (bottom) out.insert(ChunkCoord(cx, cy + 1));

    // Углы — для диагональных соседей (рождение может произойти по диагонали).
    if (rb[0])                       out.insert(ChunkCoord(cx - 1, cy - 1));
    if (rb[S - 1])                   out.insert(ChunkCoord(cx + 1, cy - 1));
    if (rb[(S - 1) * S])             out.insert(ChunkCoord(cx - 1, cy + 1));
    if (rb[(S - 1) * S + (S - 1)])   out.insert(ChunkCoord(cx + 1, cy + 1));
}
