// demo/cloth/SpringNetwork.cpp
#include "demo/cloth/SpringNetwork.h"
#include "engine/core/ParallelFor.h"
#include "demo/cloth/SpringBackendFactory.h"
#include "engine/simulation/CudaSpringBackend.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    // Затухание пика яркости за кадр: glow = max(норм.значение, glow*decay).
    // Само "норм.значение" — не домноженная на константу сырая величина, а
    // результат нормализации относительно текущих min/max по сетке (см.
    // step()), поэтому здесь остаётся только скорость затухания следа —
    // подобрана на глаз, даёт заметный, но не "дёрганый" пик, гаснущий за
    // десяток-другой кадров.
    constexpr float kNodeGlowDecay = 0.90f;
    constexpr float kEdgeGlowDecay = 0.92f;

    // Reinhard-нормализация (v/(v+avg)) даёт 0.5 ровно на среднем значении —
    // если энергия поднялась по всему полю разом (широкое натяжение), у
    // среднего узла/ребра всё ещё будет norm~0.5, и всё поле светится
    // наполовину, а не только самые "горячие" точки. Возводим в степень > 1,
    // чтобы прижать середину к нулю (0.5^3 = 0.125), оставив ярким только то,
    // что заметно выше среднего — так поле в целом остаётся тёмным даже при
    // высокой общей энергии, и виден именно пик/фронт волны, а не вся сетка.
    constexpr float kGlowContrast = 3.0f;

    // Нижние пороги L_avg (см. step()) и одновременно порог "энергичности"
    // чанка для activity-пересчёта в конце step() — тот же смысл, не разные
    // магические числа. Вынесены на уровень файла (а не локальные внутри
    // step()), потому что нужны и в CPU-, и в GPU-ветке step(), а activity-
    // пересчёт в конце функции остаётся общим для обоих путей.
    constexpr float kMinAvgSpeed = 0.05f;    // доля от spacing/сек
    constexpr float kMinAvgStretch = 0.01f;  // ~1% растяжения
    constexpr float kLogEps = 1e-4f;
}

SpringNetwork::SpringNetwork(int cols, int rows, float spacing)
    : m_cols(cols), m_rows(rows), m_spacing(spacing)
{
#ifdef FE_CUDA_ENABLED
    m_gpuBackend = MakeSpringGpuBackend();
#endif
    reset();
}

SpringNetwork::~SpringNetwork() = default;   // здесь CudaSpringBackend уже полный тип (см. .h, pImpl)

void SpringNetwork::addEdge(int a, int b) {
    float restLen = glm::length(m_restPos[a] - m_restPos[b]);
    m_edges.push_back({a, b, restLen});
}

void SpringNetwork::reset() {
    std::lock_guard<std::mutex> lock(m_mutex);
    int n = m_cols * m_rows;
    m_pos.assign(n, glm::vec2(0));
    m_vel.assign(n, glm::vec2(0));
    m_restPos.assign(n, glm::vec2(0));
    m_pinned.assign(n, false);
    m_nodeGlow.assign(n, 0.0f);
    m_edges.clear();

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            int i = index(c, r);
            glm::vec2 p(c * m_spacing, r * m_spacing);
            m_pos[i] = p;
            m_restPos[i] = p;
            // Края закреплены — импульс отражается от границы вместо того,
            // чтобы утащить всю сетку за собой.
            m_pinned[i] = (r == 0 || r == m_rows - 1 || c == 0 || c == m_cols - 1);
        }
    }

    // Structural — только рёбра решётки (право/вниз), добавляются первыми
    // одним куском. m_renderEdgeCount фиксирует границу: рендер рисует только
    // [0, m_renderEdgeCount) — это то, что визуально выглядит как сетка
    // квадратов. Shear/bend ниже участвуют в физике (жёсткость на сдвиг и
    // изгиб), но НЕ рендерятся: диагонали поверх решётки читаются глазом как
    // ромбы, а не как квадратная сетка, и это вдобавок вдвое-втрое больше
    // вершинных данных без визуальной пользы.
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            int i = index(c, r);
            if (c + 1 < m_cols) addEdge(i, index(c + 1, r));               // structural →
            if (r + 1 < m_rows) addEdge(i, index(c, r + 1));               // structural ↓
        }
    }
    m_renderEdgeCount = m_edges.size();   // structural-only — граница рендерящегося блока

    // Shear-диагонали — сопротивление сдвигу, только физика.
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            int i = index(c, r);
            if (c + 1 < m_cols && r + 1 < m_rows) {
                addEdge(i, index(c + 1, r + 1));                           // shear ↘
                addEdge(index(c + 1, r), index(c, r + 1));                 // shear ↙
            }
        }
    }

    // Bend-пружины через один узел — сопротивление изгибу, только физика.
    size_t shearEnd = m_edges.size();
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            int i = index(c, r);
            if (c + 2 < m_cols) addEdge(i, index(c + 2, r));               // bend →→
            if (r + 2 < m_rows) addEdge(i, index(c, r + 2));               // bend ↓↓
        }
    }
    size_t bendEnd = m_edges.size();

    // Геометрия чанков — см. kChunkSize (.h). Бакетинг ниже переупорядочивает
    // рёбра ВНУТРИ каждого из трёх типовых блоков (границы m_renderEdgeCount/
    // shearEnd/bendEnd не меняются, порядок внутри — да) по чанку узла a, для
    // chunk-based диспетчеризации в step() (см. bucketEdgeBlock()) и
    // chunk-based отбраковки при рендере (см. chunkTopology(), main.cpp).
    m_chunksX = (m_cols + kChunkSize - 1) / kChunkSize;
    m_chunksY = (m_rows + kChunkSize - 1) / kChunkSize;
    m_numChunks = m_chunksX * m_chunksY;
    bucketEdgeBlock(0, m_renderEdgeCount, m_chunkStructOffset, m_chunkStructInteriorEnd);
    bucketEdgeBlock(m_renderEdgeCount, shearEnd - m_renderEdgeCount, m_chunkShearOffset, m_chunkShearInteriorEnd);
    bucketEdgeBlock(shearEnd, bendEnd - shearEnd, m_chunkBendOffset, m_chunkBendInteriorEnd);

    m_edgeGlow.assign(m_edges.size(), 0.0f);
    m_force.assign(n, glm::vec2(0.0f));
    m_chunkActive.assign(m_numChunks, 0);
    m_chunkIdleFrames.assign(m_numChunks, 0);

    // GPU-топология — не меняется до следующего reset(), грузим один раз
    // здесь (см. CudaSpringBackend::uploadTopology). EdgeGpu — POD-зеркало
    // Edge, конвертируем явным циклом (безопаснее reinterpret_cast, дёшево —
    // это только reset(), не step()). Вызовы m_gpuBackend->... гейтятся
    // #ifdef FE_CUDA_ENABLED (не только рантайм-проверкой if(m_gpuBackend))
    // — методы CudaSpringBackend физически не слинкованы в CPU-only сборке
    // (CudaSpringBackend.cu туда не компилируется), а m_gpuBackend в такой
    // сборке всё равно всегда nullptr (см. SpringBackendFactory.cpp).
#ifdef FE_CUDA_ENABLED
    if (m_gpuBackend) {
        std::vector<CudaSpringBackend::EdgeGpu> gpuEdges(m_edges.size());
        for (size_t i = 0; i < m_edges.size(); ++i) {
            gpuEdges[i] = {m_edges[i].a, m_edges[i].b, m_edges[i].restLen};
        }
        m_gpuBackend->uploadTopology(gpuEdges, m_restPos,
                                     m_chunkStructOffset, m_chunkShearOffset, m_chunkBendOffset,
                                     m_numChunks);
    }
#endif
}

void SpringNetwork::bucketEdgeBlock(size_t base, size_t count,
                                     std::vector<uint32_t>& outOffset,
                                     std::vector<uint32_t>& outInteriorEnd) {
    outOffset.assign(static_cast<size_t>(m_numChunks) + 1, 0);
    outInteriorEnd.assign(static_cast<size_t>(m_numChunks), 0);
    if (count == 0) { std::fill(outOffset.begin(), outOffset.end(), static_cast<uint32_t>(base)); return; }

    // 1) какому чанку принадлежит каждое ребро (по узлу a) + гистограмма.
    std::vector<int> edgeChunk(count);
    for (size_t i = 0; i < count; ++i) {
        const Edge& e = m_edges[base + i];
        int c = chunkIndexOf(e.a % m_cols, e.a / m_cols);
        edgeChunk[i] = c;
        outOffset[static_cast<size_t>(c) + 1]++;
    }
    // 2) префиксная сумма → offset (локально в блоке, затем сдвигаем на base).
    for (int c = 0; c < m_numChunks; ++c) outOffset[c + 1] += outOffset[c];
    for (auto& v : outOffset) v += static_cast<uint32_t>(base);

    // 3) scatter по чанку (порядок внутри чанка ещё не важен — упорядочим
    // internal/boundary отдельным проходом ниже).
    std::vector<Edge> bucketed(count);
    std::vector<uint32_t> cursor(outOffset.begin(), outOffset.end() - 1);
    for (size_t i = 0; i < count; ++i) {
        int c = edgeChunk[i];
        bucketed[cursor[static_cast<size_t>(c)] - static_cast<uint32_t>(base)] = m_edges[base + i];
        cursor[static_cast<size_t>(c)]++;
    }
    std::copy(bucketed.begin(), bucketed.end(), m_edges.begin() + static_cast<ptrdiff_t>(base));

    // 4) внутри каждого чанка — stable_partition: сначала "внутренние" рёбра
    // (оба конца в этом чанке — их можно параллелить по чанкам без гонок на
    // запись силы), потом "граничные" (конец b в соседнем чанке — пишут в
    // узел ЧУЖОГО чанка, обрабатываются последовательно в step()).
    for (int c = 0; c < m_numChunks; ++c) {
        auto beginIt = m_edges.begin() + static_cast<ptrdiff_t>(outOffset[c]);
        auto endIt = m_edges.begin() + static_cast<ptrdiff_t>(outOffset[c + 1]);
        auto mid = std::stable_partition(beginIt, endIt, [&](const Edge& e) {
            return chunkIndexOf(e.b % m_cols, e.b / m_cols) == c;   // true = internal
        });
        outInteriorEnd[static_cast<size_t>(c)] = static_cast<uint32_t>(mid - m_edges.begin());
    }
}

int SpringNetwork::chunkNeighbors(int chunkCol, int chunkRow, int out[8]) const {
    int n = 0;
    for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            int cc = chunkCol + dc, cr = chunkRow + dr;
            if (cc < 0 || cc >= m_chunksX || cr < 0 || cr >= m_chunksY) continue;
            out[n++] = cr * m_chunksX + cc;
        }
    }
    return n;
}

void SpringNetwork::chunkTopology(std::vector<uint32_t>& outStructOffset) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    outStructOffset = m_chunkStructOffset;
}

void SpringNetwork::step(float dt, float stiffness, float dampingRate) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (dt <= 0.0f) return;

    const int n = static_cast<int>(m_pos.size());
    const int m = static_cast<int>(m_edges.size());
    const size_t numThreads = std::max<size_t>(1, TaskScheduler::instance().thread_count());
    if (m_force.size() != static_cast<size_t>(n)) m_force.resize(n);

    // Критерий устойчивости явного Эйлера для пружин: stiffness*dtSub^2
    // должно быть заметно меньше 4 (иначе даже линейный осциллятор
    // расходится). При сильном локальном натяжении (pluck / резкий drag)
    // энергия перекачивается в соседние рёбра за один шаг с перелётом
    // (overshoot) — соседние узлы начинают колебаться в противофазе
    // (zigzag/hourglass mode), и именно это, а не просто "медленно гаснущая
    // осцилляция", даёт видимые "замятины". Дробим шаг на substeps так,
    // чтобы stiffness*dtSub^2 оставался в безопасных пределах — итоговое
    // смещение за кадр по-прежнему управляется dt/stiffness так же, как
    // раньше (Time scale не меняется), просто сам вызов интегратора мельче.
    // m_force переиспользуется между под-шагами — не переаллоцируется,
    // только перезаполняется нулями каждый под-шаг.
    constexpr float kStabilityLimit = 2.0f;   // запас вместо теоретических 4.0
    constexpr int kMaxSubsteps = 16;
    int substeps = 1;
    if (stiffness > 0.0f) {
        float need = dt * std::sqrt(stiffness / kStabilityLimit);
        substeps = std::clamp(static_cast<int>(std::ceil(need)), 1, kMaxSubsteps);
    }
    const float subDt = dt / static_cast<float>(substeps);
    const float dampFactor = std::exp(-dampingRate * subDt);
    m_lastSubsteps.store(substeps, std::memory_order_relaxed);   // диагностика — см. .h

    // Мягкий предел скорости вместо резкого teleport-reset позиции при уходе
    // в NaN/Inf: раньше узел, разлетевшийся в inf, телепортировался обратно
    // в rest-позицию, а соседи — нет; их пружины к этому узлу резко меняли
    // растяжение и порождали НОВЫЙ импульс — самоподдерживающаяся цепная
    // реакция, которая и была настоящей причиной "замятин" (не просто плохо
    // гаснущая осцилляция, а активно генерируемый заново источник энергии
    // прямо посреди поля). Ограничение скорости не даёт узлу вообще долететь
    // до inf/NaN, поэтому позиция никогда не скачет разрывно — 2 клетки
    // сетки за под-шаг с большим запасом покрывает любую нормальную волну.
    constexpr float kMaxDisplacementPerSubstep = 2.0f;   // в единицах m_spacing
    const float maxSpeed = kMaxDisplacementPerSubstep * m_spacing / subDt;

    // Список чанков к обработке в этом step() — активные (энергичные сами
    // по себе, см. пересчёт в конце функции, или только что тронутые
    // взаимодействием — activateChunkAt/activateChunksInWindow) плюс их
    // непосредственные соседи (halo — чтобы сила из активного чанка дошла
    // на границу соседа В ЭТОМ ЖЕ кадре, а не с задержкой в кадр). Считается
    // один раз на весь step() (не на под-шаг — активность не меняется
    // внутри под-шагов одного кадра).
    //
    // Известный краевой случай: halo — только 1 кольцо соседей (64 клетки),
    // а при большом stiffness substeps может доходить до kMaxSubsteps=16, и
    // клэмп в 2 клетки/под-шаг теоретически допускает смещение узла до ~32
    // клеток за один вызов step() — на самой экстремальной энергии узел у
    // края halo-зоны в принципе может "перепрыгнуть" её до того, как список
    // пересчитается на следующий кадр. Не крэш и не NaN (то отдельно ловится
    // isfinite-клэмпом ниже) — тихий визуальный артефакт на один кадр,
    // предположительно редкий (нужны одновременно stiffness у самого
    // предела и узел у самого края halo). Не исправлено — если станет
    // заметно, обновлять m_processChunks на каждый под-шаг, а не раз на кадр.
    {
        std::vector<uint8_t> include(static_cast<size_t>(m_numChunks), 0);
        for (int c = 0; c < m_numChunks; ++c) {
            if (!m_chunkActive[static_cast<size_t>(c)]) continue;
            include[static_cast<size_t>(c)] = 1;
            int nb[8];
            int cnt = chunkNeighbors(c % m_chunksX, c / m_chunksX, nb);
            for (int k = 0; k < cnt; ++k) include[static_cast<size_t>(nb[k])] = 1;
        }
        m_processChunks.clear();
        for (int c = 0; c < m_numChunks; ++c) if (include[static_cast<size_t>(c)]) m_processChunks.push_back(c);
    }
    const int numProcessChunks = static_cast<int>(m_processChunks.size());

    bool usedGpu = false;
#ifdef FE_CUDA_ENABLED
    // Вызовы m_gpuBackend->... гейтятся #ifdef FE_CUDA_ENABLED (не только
    // рантайм-проверкой if(m_gpuBackend)) — методы CudaSpringBackend
    // физически не слинкованы в CPU-only сборке (CudaSpringBackend.cu туда
    // не компилируется), а m_gpuBackend в такой сборке всё равно всегда
    // nullptr (см. SpringBackendFactory.cpp) — но САМ ВЫЗОВ метода должен
    // не существовать в объектном файле, а не просто не выполняться.
    if (m_gpuBackend) {
        // GPU-путь (Stage 2/3): force(atomicAdd)+integrate+glow целиком на
        // устройстве, см. CudaSpringBackend::step(). Единственное, что
        // остаётся здесь — конвертация m_pinned (vector<bool>, не грузится
        // напрямую в device-память) и диагностический пересчёт avgSpeed/
        // avgStretch (для watchdog-панели, не влияет на glow — GPU уже
        // нормализовал c теми же порогами/decay внутри себя).
        std::vector<uint8_t> pinnedBytes(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) pinnedBytes[static_cast<size_t>(i)] = m_pinned[i] ? 1 : 0;
        if (m_speedBuf.size() != static_cast<size_t>(n)) m_speedBuf.resize(n);
        if (m_stretchBuf.size() != static_cast<size_t>(m)) m_stretchBuf.resize(m);

        m_gpuBackend->step(m_pos, m_vel, pinnedBytes,
                          m_nodeGlow, m_nodeStretchGlow, m_edgeGlow,
                          m_speedBuf, m_stretchBuf, m_processChunks,
                          substeps, subDt, dampFactor, maxSpeed, stiffness,
                          kNodeGlowDecay, kEdgeGlowDecay, kGlowContrast,
                          kMinAvgSpeed * m_spacing, kMinAvgStretch);

        float totalLogSpeed = 0.0f;
        for (int i = 0; i < n; ++i) totalLogSpeed += std::log(m_speedBuf[static_cast<size_t>(i)] + kLogEps);
        float avgSpeed = n > 0
            ? std::max(std::exp(totalLogSpeed / static_cast<float>(n)), kMinAvgSpeed * m_spacing)
            : kMinAvgSpeed * m_spacing;
        m_lastAvgSpeed.store(avgSpeed, std::memory_order_relaxed);   // диагностика — см. .h

        float totalLogStretch = 0.0f;
        for (int e = 0; e < m; ++e) totalLogStretch += std::log(m_stretchBuf[static_cast<size_t>(e)] + kLogEps);
        float avgStretch = m > 0
            ? std::max(std::exp(totalLogStretch / static_cast<float>(m)), kMinAvgStretch)
            : kMinAvgStretch;
        m_lastAvgStretch.store(avgStretch, std::memory_order_relaxed);   // диагностика — см. .h
        usedGpu = true;
    }
#endif
    if (!usedGpu) {
    for (int s = 0; s < substeps; ++s) {
        // m_force — общий (не per-thread) аккумулятор, обнуляется целиком
        // одним O(n) проходом каждый под-шаг (дёшево — экономия чанков в
        // пропуске самого РАСЧЁТА рёбер неактивных чанков, не в обнулении).
        parallelFor(n, [&](int begin, int end, int) {
            std::fill(m_force.begin() + begin, m_force.begin() + end, glm::vec2(0.0f));
        });

        // Фаза 1a — "внутренние" рёбра (оба конца в одном чанке) обрабатываемых
        // чанков, параллельно ПО ЧАНКАМ: разные чанки из m_processChunks не
        // пересекаются по узлам для internal-рёбер (bucketEdgeBlock()
        // гарантирует это при бакетинге в reset()), поэтому пишут в общий
        // m_force без атомиков и без гонок — раньше для этого нужны были
        // per-thread буферы на ЛЮБОЙ диапазон рёбер, здесь диапазон уже
        // пространственно разделён самими чанками.
        auto applyEdge = [&](const Edge& edge) {
            glm::vec2 delta = m_pos[static_cast<size_t>(edge.b)] - m_pos[static_cast<size_t>(edge.a)];
            float dist = glm::length(delta);
            // Раньше dist~0 означало continue — сила вообще не применялась
            // именно там, где растяжение (относительно restLen) максимально
            // (узлы схлопнулись). Подставляем произвольное направление
            // вместо пропуска — как уже делает pluck() для того же
            // вырожденного случая.
            glm::vec2 dir = (dist > 1e-5f) ? (delta / dist) : glm::vec2(0.0f, -1.0f);
            glm::vec2 f = dir * (stiffness * (dist - edge.restLen));
            m_force[static_cast<size_t>(edge.a)] += f;
            m_force[static_cast<size_t>(edge.b)] -= f;
        };
        parallelFor(numProcessChunks, [&](int begin, int end, int) {
            for (int idx = begin; idx < end; ++idx) {
                int c = m_processChunks[static_cast<size_t>(idx)];
                for (uint32_t e = m_chunkStructOffset[c]; e < m_chunkStructInteriorEnd[c]; ++e) applyEdge(m_edges[e]);
                for (uint32_t e = m_chunkShearOffset[c]; e < m_chunkShearInteriorEnd[c]; ++e) applyEdge(m_edges[e]);
                for (uint32_t e = m_chunkBendOffset[c]; e < m_chunkBendInteriorEnd[c]; ++e) applyEdge(m_edges[e]);
            }
        });

        // Фаза 1b — "граничные" рёбра (конец b — в СОСЕДНЕМ чанке, который
        // в этом же кадре может параллельно обрабатываться другим worker'ом
        // из фазы 1a) — последовательно, без атомиков, единственный
        // корректный способ не гоняться за conflict-free раскраской графа
        // чанков ради небольшой (по числу рёбер — периметр, не площадь)
        // доли работы.
        for (int c : m_processChunks) {
            for (uint32_t e = m_chunkStructInteriorEnd[c]; e < m_chunkStructOffset[c + 1]; ++e) applyEdge(m_edges[e]);
            for (uint32_t e = m_chunkShearInteriorEnd[c]; e < m_chunkShearOffset[c + 1]; ++e) applyEdge(m_edges[e]);
            for (uint32_t e = m_chunkBendInteriorEnd[c]; e < m_chunkBendOffset[c + 1]; ++e) applyEdge(m_edges[e]);
        }

        // Фаза 2b — интегрирование, параллельно по узлам (безусловно, для
        // ВСЕХ n узлов, а не только активных чанков — узел на границе
        // активного/неактивного чанка мог получить ненулевую силу из фазы
        // 1b выше, и она обязана быть проинтегрирована в этом же кадре;
        // см. design-заметки в плане про "почему интеграция/glow остаются
        // безусловными в Stage 1"), читает силы из m_force.
        parallelFor(n, [&](int begin, int end, int) {
            for (int i = begin; i < end; ++i) {
                if (m_pinned[i]) {
                    // Позицию ведёт drag напрямую (updateDrag), физика её не
                    // трогает. Скорость драг сам обновляет в updateDrag() —
                    // не обнуляем её здесь, иначе при отпускании (endDrag)
                    // узел падает без "броска", хотя его реально тащили.
                    continue;
                }
                m_vel[i] += m_force[i] * subDt;
                m_vel[i] *= dampFactor;

                float speed = glm::length(m_vel[i]);
                if (speed > maxSpeed) m_vel[i] *= (maxSpeed / speed);

                m_pos[i] += m_vel[i] * subDt;

                // Остаточная защита: если что-то всё же проскочило клэмп
                // выше (например, NaN пришёл уже в m_force с другой
                // стороны), не даём этому течь в GL-буферы каждый кадр.
                if (!std::isfinite(m_pos[i].x) || !std::isfinite(m_pos[i].y)) {
                    m_pos[i] = m_restPos[i];
                    m_vel[i] = glm::vec2(0.0f);
                }
            }
        });
    }

    // Фаза 3/4 — яркость для рендера, один раз за кадр целиком (не за
    // под-шаг — это визуальная надстройка над финальным состоянием, гонять
    // её substeps раз незачем). Нормализация — геометрическое (лог-)среднее
    // вместо арифметического или min/max: L_avg = exp(mean(log(v+eps))),
    // затем Reinhard-подобное сжатие norm = v/(v+L_avg). В отличие от min/max
    // — устойчиво к единичным выбросам (одна аномально энергичная точка не
    // топит всю остальную сетку в темноту) и не требует явного "потолка":
    // L_avg сама служит "серой точкой" экспозиции, а norm всегда плавно
    // лежит в [0,1) независимо от абсолютной величины. Сумма логарифмов —
    // per-thread частичные суммы (без атомиков), как и раньше; эта фаза, в
    // отличие от расчёта сил, безусловна для ВСЕХ n узлов / m рёбер, не
    // только активных чанков (см. комментарий у фазы агрегации натяжения
    // ниже).
    if (m_speedBuf.size() != static_cast<size_t>(n)) m_speedBuf.resize(n);
    std::vector<float> sumLogSpeedT(numThreads, 0.0f);
    parallelFor(n, [&](int begin, int end, int t) {
        float sumLog = 0.0f;
        for (int i = begin; i < end; ++i) {
            float speed = glm::length(m_vel[i]);
            m_speedBuf[i] = speed;
            sumLog += std::log(speed + kLogEps);
        }
        sumLogSpeedT[static_cast<size_t>(t)] = sumLog;
    });
    float totalLogSpeed = 0.0f;
    for (float v : sumLogSpeedT) totalLogSpeed += v;
    // Нижний порог на L_avg: без него, когда вся сетка тихая, L_avg сама
    // уходит к ~0 — и тогда даже ничтожная абсолютная скорость (шум/лёгкая
    // рябь) делит "почти на ноль" и норм. значение раздувается к 1.0, хотя
    // светиться там особо нечему. С порогом L_avg не может провалиться ниже
    // "типичной тихой" скорости, поэтому малые абсолютные значения остаются
    // малыми и в тихой сцене, а не только относительно текущего минимума.
    float avgSpeed = n > 0
        ? std::max(std::exp(totalLogSpeed / static_cast<float>(n)), kMinAvgSpeed * m_spacing)
        : kMinAvgSpeed * m_spacing;
    m_lastAvgSpeed.store(avgSpeed, std::memory_order_relaxed);   // диагностика — см. .h

    parallelFor(n, [&](int begin, int end, int) {
        for (int i = begin; i < end; ++i) {
            float norm = m_speedBuf[i] / (m_speedBuf[i] + avgSpeed);
            norm = std::pow(norm, kGlowContrast);
            m_nodeGlow[i] = std::max(norm, m_nodeGlow[i] * kNodeGlowDecay);
        }
    });

    if (m_stretchBuf.size() != static_cast<size_t>(m)) m_stretchBuf.resize(m);
    std::vector<float> sumLogStretchT(numThreads, 0.0f);
    parallelFor(m, [&](int begin, int end, int t) {
        float sumLog = 0.0f;
        for (int e = begin; e < end; ++e) {
            const Edge& edge = m_edges[e];
            float len = glm::length(m_pos[edge.b] - m_pos[edge.a]);
            float stretch = std::fabs(len - edge.restLen) / edge.restLen;
            m_stretchBuf[e] = stretch;
            sumLog += std::log(stretch + kLogEps);
        }
        sumLogStretchT[static_cast<size_t>(t)] = sumLog;
    });
    float totalLogStretch = 0.0f;
    for (float v : sumLogStretchT) totalLogStretch += v;
    float avgStretch = m > 0
        ? std::max(std::exp(totalLogStretch / static_cast<float>(m)), kMinAvgStretch)
        : kMinAvgStretch;
    m_lastAvgStretch.store(avgStretch, std::memory_order_relaxed);   // диагностика — см. .h

    parallelFor(m, [&](int begin, int end, int) {
        for (int e = begin; e < end; ++e) {
            float norm = m_stretchBuf[e] / (m_stretchBuf[e] + avgStretch);
            norm = std::pow(norm, kGlowContrast);
            m_edgeGlow[e] = std::max(norm, m_edgeGlow[e] * kEdgeGlowDecay);
        }
    });

    // Агрегируем натяжение на уровень узла (max среди инцидентных рёбер) —
    // для Points/Cubes, когда хотим видеть натяжение, а не скорость узла
    // (см. main.cpp, переключатель "What to display"). Переиспользуем уже
    // нормализованный/контрастный m_edgeGlow — decay уже применён на уровне
    // ребра, отдельного геометрического среднего/decay тут не нужно. Эта
    // фаза, в отличие от расчёта сил, безусловна для ВСЕХ чанков (не только
    // m_processChunks) — glow/интеграция в Stage 1 намеренно остаются
    // безусловными (см. design-заметки в плане), поэтому переиспользуем тот
    // же приём "internal-рёбра чанка параллельно, boundary — последовательно"
    // (см. bucketEdgeBlock()/фазу 1 выше), но по ВСЕМ m_numChunks чанкам.
    if (m_nodeStretchGlow.size() != static_cast<size_t>(n)) m_nodeStretchGlow.resize(n);
    parallelFor(n, [&](int begin, int end, int) {
        std::fill(m_nodeStretchGlow.begin() + begin, m_nodeStretchGlow.begin() + end, 0.0f);
    });
    auto applyStretch = [&](const Edge& edge, uint32_t e) {
        float g = m_edgeGlow[e];
        if (g > m_nodeStretchGlow[static_cast<size_t>(edge.a)]) m_nodeStretchGlow[static_cast<size_t>(edge.a)] = g;
        if (g > m_nodeStretchGlow[static_cast<size_t>(edge.b)]) m_nodeStretchGlow[static_cast<size_t>(edge.b)] = g;
    };
    parallelFor(m_numChunks, [&](int begin, int end, int) {
        for (int c = begin; c < end; ++c) {
            for (uint32_t e = m_chunkStructOffset[c]; e < m_chunkStructInteriorEnd[c]; ++e) applyStretch(m_edges[e], e);
            for (uint32_t e = m_chunkShearOffset[c]; e < m_chunkShearInteriorEnd[c]; ++e) applyStretch(m_edges[e], e);
            for (uint32_t e = m_chunkBendOffset[c]; e < m_chunkBendInteriorEnd[c]; ++e) applyStretch(m_edges[e], e);
        }
    });
    for (int c = 0; c < m_numChunks; ++c) {
        for (uint32_t e = m_chunkStructInteriorEnd[c]; e < m_chunkStructOffset[c + 1]; ++e) applyStretch(m_edges[e], e);
        for (uint32_t e = m_chunkShearInteriorEnd[c]; e < m_chunkShearOffset[c + 1]; ++e) applyStretch(m_edges[e], e);
        for (uint32_t e = m_chunkBendInteriorEnd[c]; e < m_chunkBendOffset[c + 1]; ++e) applyStretch(m_edges[e], e);
    }
    }   // !usedGpu (CPU path)

    // Пересчёт активности чанков — для физики СЛЕДУЮЩЕГО step() (задержка в
    // кадр для "пассивного" пробуждения энергией; немедленная активация от
    // взаимодействия — см. activateChunkAt/activateChunksInWindow, отдельно
    // от этого пересчёта). Чанк "энергичный", если у него самого (не у
    // соседей) есть узел с сырой скоростью выше kMinAvgSpeed*spacing ИЛИ
    // ребро с сырым растяжением выше kMinAvgStretch — те же пороги "почти
    // тишина", что уже используются как пол L_avg выше, а не новые
    // магические числа. Гистерезис (kIdleFramesToDeactivate кадров подряд
    // ниже порога) — чтобы не мигать активностью на маргинальных значениях
    // энергии прямо у порога.
    constexpr int kIdleFramesToDeactivate = 12;
    const float wakeSpeed = kMinAvgSpeed * m_spacing;
    const float wakeStretch = kMinAvgStretch;
    for (int cy = 0; cy < m_chunksY; ++cy) {
        for (int cx = 0; cx < m_chunksX; ++cx) {
            int c = cy * m_chunksX + cx;
            int rowLo = cy * kChunkSize, rowHi = std::min(rowLo + kChunkSize, m_rows);
            int colLo = cx * kChunkSize, colHi = std::min(colLo + kChunkSize, m_cols);
            bool energetic = false;
            for (int r = rowLo; r < rowHi && !energetic; ++r) {
                for (int cc = colLo; cc < colHi; ++cc) {
                    if (m_speedBuf[static_cast<size_t>(index(cc, r))] > wakeSpeed) { energetic = true; break; }
                }
            }
            if (!energetic) {
                auto anyStretchAbove = [&](uint32_t lo, uint32_t hi) {
                    for (uint32_t e = lo; e < hi; ++e) if (m_stretchBuf[e] > wakeStretch) return true;
                    return false;
                };
                energetic = anyStretchAbove(m_chunkStructOffset[c], m_chunkStructOffset[c + 1])
                         || anyStretchAbove(m_chunkShearOffset[c], m_chunkShearOffset[c + 1])
                         || anyStretchAbove(m_chunkBendOffset[c], m_chunkBendOffset[c + 1]);
            }
            if (energetic) {
                m_chunkActive[c] = 1;
                m_chunkIdleFrames[c] = 0;
            } else if (m_chunkActive[c] && ++m_chunkIdleFrames[c] >= kIdleFramesToDeactivate) {
                m_chunkActive[c] = 0;
                m_chunkIdleFrames[c] = 0;
            }
        }
    }
}

void SpringNetwork::activateChunksInWindow(int colLo, int colHi, int rowLo, int rowHi) {
    int chunkColLo = colLo / kChunkSize, chunkColHi = colHi / kChunkSize;
    int chunkRowLo = rowLo / kChunkSize, chunkRowHi = rowHi / kChunkSize;
    for (int cy = chunkRowLo; cy <= chunkRowHi; ++cy) {
        for (int cx = chunkColLo; cx <= chunkColHi; ++cx) {
            int c = cy * m_chunksX + cx;
            m_chunkActive[c] = 1;
            m_chunkIdleFrames[c] = 0;
        }
    }
}

void SpringNetwork::activateChunkAt(int nodeIndex) {
    int col = nodeIndex % m_cols, row = nodeIndex / m_cols;
    activateChunksInWindow(col, col, row, row);
}

void SpringNetwork::windowAround(glm::vec2 worldPos, float radiusCells,
                                  int& colLo, int& colHi, int& rowLo, int& rowHi) const {
    int colC = static_cast<int>(std::round(worldPos.x / m_spacing));
    int rowC = static_cast<int>(std::round(worldPos.y / m_spacing));
    int half = static_cast<int>(std::ceil(radiusCells));
    colLo = std::clamp(colC - half, 0, m_cols - 1);
    colHi = std::clamp(colC + half, 0, m_cols - 1);
    rowLo = std::clamp(rowC - half, 0, m_rows - 1);
    rowHi = std::clamp(rowC + half, 0, m_rows - 1);
}

int SpringNetwork::beginDrag(glm::vec2 worldPos) {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Растущее окно вместо обхода всех n узлов (см. windowAround() в .h) —
    // в норме первая попытка уже находит узел, т.к. клик всегда попадает
    // рядом с видимой (примерно rest) позицией. Не гарантирует АБСОЛЮТНО
    // точный глобальный ближайший при большом уходе узла от rest (тот же
    // компромисс, что и clamp смещения в step()), но для интерактивного
    // "хватания" мышью это неотличимо от точного поиска.
    constexpr float kInitialSearchCells = 3.0f;
    float searchCells = kInitialSearchCells;
    int best = -1;
    for (;;) {
        int colLo, colHi, rowLo, rowHi;
        windowAround(worldPos, searchCells, colLo, colHi, rowLo, rowHi);
        best = -1;
        float bestDist = std::numeric_limits<float>::max();
        for (int r = rowLo; r <= rowHi; ++r) {
            for (int c = colLo; c <= colHi; ++c) {
                int i = index(c, r);
                if (m_pinned[i]) continue;   // не хватаем закреплённые по краю узлы
                float d = glm::length(m_pos[i] - worldPos);
                if (d < bestDist) { bestDist = d; best = i; }
            }
        }
        bool coversAll = (colLo == 0 && colHi == m_cols - 1 && rowLo == 0 && rowHi == m_rows - 1);
        if (best >= 0 || coversAll) break;
        searchCells *= 2.0f;
    }
    if (best >= 0) {
        m_pinned[best] = true;   // временно кинематический, пока держат
        activateChunkAt(best);   // немедленно, не ждать energetic-пересчёта в step()
    }
    return best;
}

void SpringNetwork::updateDrag(int idx, glm::vec2 worldPos, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (idx < 0 || idx >= static_cast<int>(m_pos.size())) return;
    if (dt > 1e-5f) {
        // Скорость протаскивания — step() больше не обнуляет её для
        // закреплённых узлов (см. комментарий там), поэтому при отпускании
        // (endDrag) узел полетит с реальной скоростью, а не с нулевой.
        m_vel[idx] = (worldPos - m_pos[idx]) / dt;
    }
    m_pos[idx] = worldPos;
    activateChunkAt(idx);   // каждый кадр протаскивания — растянутые пружины к соседям должны считаться
}

void SpringNetwork::endDrag(int idx) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (idx < 0 || idx >= static_cast<int>(m_pos.size())) return;
    m_pinned[idx] = false;   // отпускаем — летит со скоростью, заданной updateDrag() (throw)
}

void SpringNetwork::pluck(glm::vec2 worldPos, float strength) {
    std::lock_guard<std::mutex> lock(m_mutex);
    // См. beginDrag() — растущее окно вместо обхода всех n узлов.
    constexpr float kInitialSearchCells = 3.0f;
    float searchCells = kInitialSearchCells;
    int best = -1;
    for (;;) {
        int colLo, colHi, rowLo, rowHi;
        windowAround(worldPos, searchCells, colLo, colHi, rowLo, rowHi);
        best = -1;
        float bestDist = std::numeric_limits<float>::max();
        for (int r = rowLo; r <= rowHi; ++r) {
            for (int c = colLo; c <= colHi; ++c) {
                int i = index(c, r);
                if (m_pinned[i]) continue;
                float d = glm::length(m_pos[i] - worldPos);
                if (d < bestDist) { bestDist = d; best = i; }
            }
        }
        bool coversAll = (colLo == 0 && colHi == m_cols - 1 && rowLo == 0 && rowHi == m_rows - 1);
        if (best >= 0 || coversAll) break;
        searchCells *= 2.0f;
    }
    if (best < 0) return;
    activateChunkAt(best);   // немедленно, не ждать energetic-пересчёта в step()
    glm::vec2 dir = m_pos[best] - worldPos;
    float len = glm::length(dir);
    dir = (len > 1e-4f) ? dir / len : glm::vec2(0.0f, -1.0f);
    // Растягиваем пружину смещением позиции — force-интегратор на следующем
    // step() сам разошлёт импульс от натяжения. strength клэмпится: без
    // предела при strength~200 и spacing~32 узел мог мгновенно прыгнуть на
    // 6+ клеток за кадр, разрывая топологию раньше, чем сработает clamp
    // скорости в step() (тот клэмпит уже РЕЗУЛЬТАТ огромной силы, но не сам
    // мгновенный скачок позиции, из которого эта сила берётся).
    constexpr float kMaxPluckDisplacement = 3.0f;   // в единицах m_spacing
    float clampedStrength = std::min(strength, kMaxPluckDisplacement * m_spacing);
    m_pos[best] += dir * clampedStrength;
}

namespace {
    // Запас в клетках поверх radius/spacing для окна кисти — узел может
    // отклониться от rest-позиции на несколько клеток (см. clamp смещения в
    // step()), а окно строится по rest, не по текущей позиции. Сама
    // проверка dist>=radius внутри цикла всё равно точна (по факт. m_pos) —
    // margin влияет только на то, что окно не потеряет узел, реально
    // попадающий в радиус кисти, из-за такого отклонения.
    constexpr float kBrushWindowMarginCells = 4.0f;
}

void SpringNetwork::brush(glm::vec2 worldPos, float radius, float strength, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (radius <= 0.0f || dt <= 0.0f) return;
    // Окно по rest-позиции вместо обхода всех n узлов (см. windowAround() в
    // .h) — brush() зовётся каждый кадр, пока кисть зажата, поэтому на
    // большой сетке (сотни тысяч+ узлов) это самый горячий путь взаимодействия.
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, radius / m_spacing + kBrushWindowMarginCells, colLo, colHi, rowLo, rowHi);
    activateChunksInWindow(colLo, colHi, rowLo, rowHi);   // немедленно, не ждать energetic-пересчёта в step()
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[i]) continue;
            glm::vec2 delta = m_pos[i] - worldPos;
            float dist = glm::length(delta);
            if (dist >= radius) continue;
            float falloff = 1.0f - (dist / radius);   // 1 в центре, 0 на краю
            glm::vec2 dir = (dist > 1e-4f) ? (delta / dist) : glm::vec2(0.0f, -1.0f);
            m_vel[i] += dir * (strength * falloff * dt);
        }
    }
}

void SpringNetwork::brushDamp(glm::vec2 worldPos, float radius, float strength, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (radius <= 0.0f || dt <= 0.0f) return;
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, radius / m_spacing + kBrushWindowMarginCells, colLo, colHi, rowLo, rowHi);
    activateChunksInWindow(colLo, colHi, rowLo, rowHi);   // см. brush() — тот же принцип
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[i]) continue;
            float dist = glm::length(m_pos[i] - worldPos);
            if (dist >= radius) continue;
            float falloff = 1.0f - (dist / radius);
            float damp = std::clamp(strength * falloff * dt, 0.0f, 1.0f);
            m_vel[i] *= (1.0f - damp);
        }
    }
}

void SpringNetwork::snapshot(std::vector<glm::vec2>& outPos,
                             std::vector<float>& outNodeGlow,
                             std::vector<float>& outNodeStretchGlow,
                             std::vector<float>& outEdgeGlow) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    outPos = m_pos;
    outNodeGlow = m_nodeGlow;
    outNodeStretchGlow = m_nodeStretchGlow;
    outEdgeGlow = m_edgeGlow;
}

void SpringNetwork::topology(std::vector<Edge>& outEdges, size_t& outRenderEdgeCount) const {
    // Топология (a,b,restLen) не меняется после reset() — вызывать один раз
    // при старте, а не каждый кадр вместе со snapshot(). На большой сетке
    // рёбра — самый тяжёлый кусок состояния (Edge = 12 байт, их миллионы),
    // копировать их 60 раз в секунду просто незачем.
    std::lock_guard<std::mutex> lock(m_mutex);
    outEdges = m_edges;
    outRenderEdgeCount = m_renderEdgeCount;
}

void SpringNetwork::restPositions(std::vector<glm::vec2>& outRestPos) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    outRestPos = m_restPos;
}
