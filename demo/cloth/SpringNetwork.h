// demo/cloth/SpringNetwork.h
#pragma once
#include <vector>
#include <mutex>
#include <atomic>
#include <utility>
#include <cstdint>
#include <memory>
#include <glm/glm.hpp>

// Под #ifdef FE_CUDA_ENABLED целиком (не только тело): unique_ptr<T> требует
// ПОЛНЫЙ тип T в каждом месте, где объект такого unique_ptr создаётся/
// уничтожается — а ~CudaSpringBackend() определён только в
// CudaSpringBackend.cu, которая в CPU-only сборке (FE_USE_CUDA=OFF) не
// компилируется вообще. Поэтому в такой сборке ни объявления класса, ни
// члена m_gpuBackend здесь просто нет — SpringNetwork в CPU-only сборке
// не содержит вообще ничего, ссылающегося на CudaSpringBackend.
#ifdef FE_CUDA_ENABLED
class CudaSpringBackend;
#endif

// Сетка масс, связанных пружинами (structural + shear + bend), явное
// (semi-implicit Euler) интегрирование по Гуку: F = -k*(len-restLen)*dir,
// v += F*dt, x += v*dt. dt здесь реально масштабирует величину шага (не
// количество шагов за кадр) — замедлить время значит уменьшить смещение за
// вызов step(), а не звать step() реже. Отдельного расчёта "света" нет —
// яркость узла/ребра в рендере это его текущая энергия (скорость узла /
// растяжение ребра относительно длины покоя), расходящаяся волной от места
// возбуждения и затухающая демпфированием.
//
// Параллелизм и sleep: сетка разбита на квадратные чанки фиксированного
// размера (kChunkSize, см. ниже) — единица и параллелизма, и "сна" для
// физики. Рёбра бакетируются по чанку узла a при reset() (см.
// bucketEdgeBlock() в .cpp) на "внутренние" (оба конца в одном чанке —
// параллелятся по чанкам без атомиков, т.к. чанки не пересекаются по узлам)
// и "граничные" (конец в соседнем чанке — считаются отдельным
// последовательным проходом, тоже без атомиков). Чанк, где энергия (сырая
// скорость узла / растяжение ребра) давно ниже порога, помечается
// неактивным и его рёбра не участвуют в расчёте силы в step() — но узлы
// всё равно интегрируются каждый кадр безусловно (см. step()), поэтому
// halo от соседнего активного чанка всё ещё доходит корректно. См.
// snapshot()/step() ниже и design-план сессии, где это было введено.
class SpringNetwork {
public:
    struct Edge { int a, b; float restLen; };

    SpringNetwork(int cols, int rows, float spacing);
    ~SpringNetwork();   // определён в .cpp — там, где CudaSpringBackend уже полный тип (pImpl)

    // Один физический шаг. stiffness — коэффициент жёсткости пружин (сила на
    // единицу растяжения), dampingRate — экспоненциальная скорость затухания
    // скорости в 1/сек (v *= exp(-dampingRate*dt)). Оба параметра действуют
    // через dt, поэтому замедление времени (меньший dt) само по себе даёт
    // меньшее смещение за вызов — количество вызовов step() за кадр менять
    // не нужно. Вызывать только из update-потока.
    void step(float dt, float stiffness, float dampingRate);

    // Схватить узел, ближайший к worldPos (кроме закреплённых по краю поля),
    // и вести его к worldPos, пока держат — кинематически, как будто тянешь
    // рукой. Возвращает индекс схваченного узла или -1, если рядом нет
    // свободного (не бывает на практике, но на всякий случай).
    int beginDrag(glm::vec2 worldPos);
    // dt — оценить скорость протаскивания (worldPos-pos)/dt и запомнить в
    // m_vel, иначе при отпускании (endDrag) узел падает с нулевой скоростью
    // вместо "броска" (step() больше не обнуляет скорость закреплённых узлов).
    void updateDrag(int idx, glm::vec2 worldPos, float dt);
    void endDrag(int idx);

    // Мгновенный "щипок": сдвигает ближайший свободный узел к worldPos, не
    // трогая его скорость — следующий step() увидит растянутые пружины и
    // разошлёт импульс бегущим пиком, как будто дёрнули струну.
    void pluck(glm::vec2 worldPos, float strength);

    // Кисть: впрыскивает скорость (не позицию — в отличие от drag/pluck, узлы
    // физически не двигает) во все свободные узлы в радиусе от worldPos,
    // радиально от центра кисти, с линейным затуханием к краю. Вызывать
    // каждый кадр, пока кисть "зажата" — накапливается сама, дольше держишь =
    // ярче светится. dt масштабирует впрыск, чтобы не зависеть от FPS.
    void brush(glm::vec2 worldPos, float radius, float strength, float dt);
    // То же самое, но гасит |v| (умножает на (1 - clamp(strength*falloff*dt)))
    // вместо впрыска — "ластик" энергии, а не источник нового импульса.
    void brushDamp(glm::vec2 worldPos, float radius, float strength, float dt);

    void reset();

    // Копия того, что меняется каждый кадр (позиции + яркость), для
    // рендер-потока, под мьютексом. Топология (рёбра) сюда сознательно не
    // входит — см. topology(). step() сам параллелится по TaskScheduler, но
    // это внутренние рабочие потоки — снаружи (для рендер-потока) вся сетка
    // всё ещё выглядит как одна атомарная операция, поэтому один mutex на
    // снимок остаётся достаточным и проще двойной буферизации, как у чанков.
    // outNodeStretchGlow — натяжение, агрегированное на узел (max среди
    // инцидентных рёбер), альтернатива outNodeGlow (скорость узла) для
    // Points/Cubes, когда хочется видеть натяжение, а не энергию движения —
    // см. main.cpp, переключатель "What to display".
    void snapshot(std::vector<glm::vec2>& outPos,
                  std::vector<float>& outNodeGlow,
                  std::vector<float>& outNodeStretchGlow,
                  std::vector<float>& outEdgeGlow) const;

    // Список рёбер (a,b,restLen) — не меняется после reset(), вызывать один
    // раз при старте и закэшировать у себя, а не гонять в snapshot() каждый
    // кадр (на большой сетке Edge-массив — самый тяжёлый кусок состояния).
    // outRenderEdgeCount — сколько первых рёбер в outEdges стоит рисовать
    // (structural+shear); остаток — bend-пружины, они только для физики.
    void topology(std::vector<Edge>& outEdges, size_t& outRenderEdgeCount) const;

    // Позиции покоя (m_restPos) — как topology(), не меняются после reset()
    // (сетка cols/rows/spacing фиксирована), кэшировать один раз при старте.
    // Нужны для "статичного" режима рендера (кубики): узел физически может
    // сместиться, а кубик остаётся на исходном месте сетки и меняет только
    // яркость — см. main.cpp, Cube mode.
    void restPositions(std::vector<glm::vec2>& outRestPos) const;

    int cols() const { return m_cols; }
    int rows() const { return m_rows; }
    float spacing() const { return m_spacing; }

    // Геометрия чанков — фиксированный квадратный размер (см. kChunkSize),
    // не зависящий от размера сетки. Нужна снаружи (main.cpp) для
    // чанк-выровненной отбраковки вне-экранных рёбер при рендере — см.
    // chunkTopology().
    static constexpr int kChunkSize = 64;
    int chunksX() const { return m_chunksX; }
    int chunksY() const { return m_chunksY; }

    // CSR-таблицы structural-рёбер по чанкам: для чанка c (индекс —
    // row*chunksX+col в координатах ЧАНКОВ, не узлов) диапазон
    // [outStructOffset[c], outStructOffset[c+1)) в массиве из topology() —
    // рёбра сетки (только structural, как и раньше рендерится только этот
    // тип), "принадлежащие" чанку c. Не меняется после reset() — кэшировать
    // один раз, как и topology()/restPositions().
    void chunkTopology(std::vector<uint32_t>& outStructOffset) const;

    // Диагностика: последнее вычисленное L_avg растяжения/скорости (см.
    // step()) — НЕ под m_mutex, читается lock-free через atomic. Специально
    // для watchdog-потока в main.cpp: если step() реально завис, держа
    // m_mutex (например, внутри самого TaskScheduler), обычный геттер под
    // локом завис бы вместе с ним — тогда как раз в момент зависания и нужно
    // прочитать последнее известное состояние, а не блокироваться.
    float lastAvgStretch() const { return m_lastAvgStretch.load(std::memory_order_relaxed); }
    float lastAvgSpeed() const { return m_lastAvgSpeed.load(std::memory_order_relaxed); }
    int lastSubsteps() const { return m_lastSubsteps.load(std::memory_order_relaxed); }

    // Диагностика (Stage 5): какой бэкенд физики реально используется —
    // "CUDA", если GPU-бэкенд создан (SpringBackendFactory нашёл рабочую
    // CUDA-карту), иначе "CPU". Решается один раз в конструкторе, не меняется
    // в рантайме.
    const char* backendName() const {
#ifdef FE_CUDA_ENABLED
        return m_gpuBackend ? "CUDA" : "CPU";
#else
        return "CPU";
#endif
    }

private:
    int index(int col, int row) const { return row * m_cols + col; }
    void addEdge(int a, int b);

    // Окно [colLo,colHi]x[rowLo,rowHi] (в клетках, clamped к границам сетки)
    // вокруг worldPos, посчитанное по РЕЖ-позиции (r*spacing, c*spacing), а
    // не обходом всех узлов — на большой сетке (сотни тысяч+ узлов) линейный
    // скан на каждый вызов brush()/beginDrag()/pluck() (а brush — КАЖДЫЙ
    // кадр, пока кисть зажата) ощутимо тормозит. radiusCells — половина
    // стороны окна в клетках; т.к. окно строится по rest-позиции, а не по
    // текущей (узел может отклониться от rest на несколько клеток — см.
    // clamp смещения в step()), вызывающий обязан либо сам проверить
    // фактическую дистанцию (brush/brushDamp), либо взять запас и при
    // необходимости расширить окно (beginDrag/pluck) — см. .cpp.
    void windowAround(glm::vec2 worldPos, float radiusCells,
                       int& colLo, int& colHi, int& rowLo, int& rowHi) const;

    // Индекс чанка (row/kChunkSize, col/kChunkSize) в координатах чанков.
    int chunkIndexOf(int col, int row) const {
        return (row / kChunkSize) * m_chunksX + (col / kChunkSize);
    }
    // До 8 соседей (включая диагональные — нужны хотя бы для shear, которая
    // цепляет диагональных соседей узла) чанка (chunkCol,chunkRow) в out[8],
    // -1 для несуществующих (за краем сетки чанков). Возвращает число
    // валидных записей в начале out (не обязательно все 8 заняты).
    int chunkNeighbors(int chunkCol, int chunkRow, int out[8]) const;

    // Бакетинг ОДНОГО типового под-диапазона m_edges[base, base+count) по
    // чанку узла a — переупорядочивает эти рёбра IN PLACE (порядок внутри
    // диапазона до вызова не важен, как и раньше — только a монотонно
    // росло, что здесь не требуется) и заполняет outOffset (size
    // m_numChunks+1, ГЛОБАЛЬНЫЕ индексы в m_edges) и outInteriorEnd (size
    // m_numChunks) — граница между "внутренними" рёбрами чанка c (оба конца
    // в c — их можно параллелить по чанкам без гонок на запись силы) и
    // "граничными" (конец b в соседнем чанке — пишут в узел ЧУЖОГО чанка,
    // поэтому в step() обрабатываются отдельным последовательным проходом,
    // без атомиков). Вызывается один раз в reset() для structural/shear/bend
    // по отдельности — учитывает разные "радиусы" рёбер разных типов
    // естественно (никакого maxOffset не нужно, разбиение чисто по
    // фактическому чанку обоих концов).
    void bucketEdgeBlock(size_t base, size_t count,
                          std::vector<uint32_t>& outOffset,
                          std::vector<uint32_t>& outInteriorEnd);

    // Немедленно (в этом же кадре, до следующего step()) помечает чанк(и)
    // активными и сбрасывает их idle-счётчик — вызывается из
    // brush/brushDamp/beginDrag/updateDrag/pluck в момент взаимодействия, а
    // не ждёт обычного energetic-пересчёта в конце step() (у того — задержка
    // в кадр, см. step()), чтобы отклик на ввод не запаздывал. Активация по
    // ИНДЕКСУ узла в регулярной сетке (его "слот", неизменный), а не по
    // текущей world-позиции — согласовано с тем, как вообще определена
    // принадлежность узла чанку (см. bucketEdgeBlock): она фиксируется в
    // reset() раз и навсегда по слоту, физическое смещение узла (drag) на
    // неё не влияет.
    void activateChunkAt(int nodeIndex);
    void activateChunksInWindow(int colLo, int colHi, int rowLo, int rowHi);

    int m_cols, m_rows;
    float m_spacing;

    mutable std::mutex m_mutex;
    std::vector<glm::vec2> m_pos, m_vel, m_restPos;
    std::vector<bool> m_pinned;
    std::vector<float> m_nodeGlow;
    std::vector<float> m_nodeStretchGlow;   // см. snapshot() — натяжение, агрегированное на узел
    std::vector<Edge> m_edges;
    // [0, m_renderEdgeCount) — structural (рендерится и как физика); дальше
    // до m_edges.size() — shear, потом bend (только физика, не рендерятся).
    // Порядок ВНУТРИ каждого блока — по чанкам (см. bucketEdgeBlock), не по
    // строкам, как было раньше.
    size_t m_renderEdgeCount = 0;
    std::vector<float> m_edgeGlow;

    // Геометрия чанков — см. kChunkSize (.h, public). Фиксируется в reset().
    int m_chunksX = 0, m_chunksY = 0, m_numChunks = 0;
    // CSR-таблицы по чанкам для каждого из трёх типов рёбер — см.
    // bucketEdgeBlock(). offset/interiorEnd — ГЛОБАЛЬНЫЕ индексы в m_edges.
    std::vector<uint32_t> m_chunkStructOffset, m_chunkStructInteriorEnd;
    std::vector<uint32_t> m_chunkShearOffset, m_chunkShearInteriorEnd;
    std::vector<uint32_t> m_chunkBendOffset, m_chunkBendInteriorEnd;

    // Активность чанка для физики этого кадра: 1 — чанк сам "живой" (энергия
    // выше порога) или сосед живого (halo — чтобы сила из живого чанка
    // корректно доходила на границу соседа в тот же кадр, а не с задержкой
    // в кадр). 0 — рёбра чанка в этом step() не считаются (но узлы всё
    // равно интегрируются — см. step()). Пишется точечно (разные индексы
    // разными потоками/фазами) — обычный vector, не atomic, гонок нет.
    std::vector<uint8_t> m_chunkActive;
    // Счётчик кадров подряд ниже порога энергии — гистерезис деактивации
    // (kIdleFramesToDeactivate в .cpp), чтобы не мигать активностью на
    // маргинальных значениях энергии прямо у порога.
    std::vector<uint8_t> m_chunkIdleFrames;
    // Компактный список чанков к обработке в текущем step() (active ∪ их
    // соседи) — пересчитывается раз в step(), не за под-шаг (активность не
    // меняется внутри под-шагов одного кадра).
    std::vector<int> m_processChunks;

    // Общий (не per-thread) аккумулятор силы — заменяет прошлую схему
    // "N per-thread буферов + слияние" (была нужна, когда параллелили по
    // ПРОИЗВОЛЬНОМУ диапазону рёбер). Теперь параллелим по ЦЕЛЫМ чанкам:
    // internal-рёбра разных чанков не пересекаются по узлам (безопасно
    // писать из разных потоков без атомиков), boundary-рёбра — отдельным
    // последовательным проходом (тоже без атомиков, просто не параллельным).
    // Обнуляется целиком раз в под-шаг — O(n), дёшево (сама экономия
    // чанков — в пропуске РАСЧЁТА рёбер неактивных чанков, не в обнулении).
    std::vector<glm::vec2> m_force;

    // Scratch-буферы для нормализации яркости (геометрическое среднее, см.
    // step()) — переиспользуются между кадрами, а не аллоцируются заново.
    std::vector<float> m_speedBuf;    // сырая скорость узла до нормализации
    std::vector<float> m_stretchBuf;  // сырое растяжение ребра до нормализации

    // См. lastAvgStretch()/lastAvgSpeed()/lastSubsteps() выше — обновляются
    // обычным (не под-локом) atomic store в конце/начале step().
    std::atomic<float> m_lastAvgStretch{0.0f};
    std::atomic<float> m_lastAvgSpeed{0.0f};
    std::atomic<int> m_lastSubsteps{1};

    // GPU-бэкенд физики — nullptr, если GPU в рантайме не найден (см.
    // SpringBackendFactory::MakeSpringGpuBackend()); член вообще отсутствует
    // в CPU-only сборке (FE_USE_CUDA=OFF, см. #ifdef выше). step() при
    // наличии бэкенда шлёт на него force+integrate+glow, оставляя на CPU
    // только activity-пересчёт чанков (единый источник правды для обоих
    // путей — см. design-план сессии и комментарий у CudaSpringBackend).
#ifdef FE_CUDA_ENABLED
    std::unique_ptr<CudaSpringBackend> m_gpuBackend;
#endif
};
