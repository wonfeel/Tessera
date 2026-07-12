// demo/light/LightField.cpp
#include "demo/light/LightField.h"
#include "engine/core/ParallelFor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    constexpr float kGlowDecay = 0.90f;
    constexpr float kGlowContrast = 3.0f;   // см. пояснение в cloth/SpringNetwork.cpp - тот же приём
    constexpr float kLogEps = 1e-4f;
    constexpr float kMinAvgSpeed = 0.05f;   // доля от spacing/сек - тот же смысл, что в cloth-модели
    constexpr float kMinAvgHeight = 0.01f;  // доля от spacing - "почти покой" по амплитуде

    // Долгая выдержка - в отличие от фиксированного ACCUMULATED_EXPOSURE в
    // Light-Simulation-JS (см. README.md), здесь адаптивная: копится не сырой
    // |height|, а та же нормализованная относительно avgSpeed доля энергии,
    // что идёт в m_glow (см. её вычисление ниже) - kAccumRate это скорость
    // насыщения в 1/сек при МАКСИМАЛЬНОЙ относительной энергии узла. Без
    // этого накопление либо мгновенно уходило в белое при большом
    // waveSpeedSq/pluckStrength, либо было еле заметно при маленьких - тюнинг
    // физики и тюнинг скорости накопления были жёстко связаны через одну
    // константу.
    constexpr float kAccumRate = 0.06f;
}

// Геометрия гекс-решётки живёт в engine/core/HexGrid.h (общая для всех
// демок), эти обёртки просто держат публичный API LightField неизменным.
float LightField::hexHorizSpacing() const { return HexGrid::horizSpacing(m_spacing); }
float LightField::hexVertSpacing() const { return HexGrid::vertSpacing(m_spacing); }

glm::vec2 LightField::worldPos(int col, int row) const {
    return HexGrid::worldPos(col, row, m_spacing);
}

LightField::LightField(int cols, int rows, float spacing)
    : m_cols(cols), m_rows(rows), m_spacing(spacing)
{
    reset();
}

void LightField::reset() {
    std::lock_guard<std::mutex> lock(m_mutex);
    int n = m_cols * m_rows;
    m_height.assign(n, 0.0f);
    m_velocity.assign(n, 0.0f);
    m_force.assign(n, 0.0f);
    m_mediumMask.assign(n, 0.0f);
    m_pinned.assign(n, 0);
    m_glow.assign(n, 0.0f);
    m_accum.assign(n, 0.0f);

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            // Край сетки закреплён на h=0 - отражающая граница (та же роль,
            // что pinned-края в cloth-модели), не пропускает волну наружу.
            if (r == 0 || r == m_rows - 1 || c == 0 || c == m_cols - 1) {
                m_pinned[static_cast<size_t>(index(c, r))] = 1;
            }
        }
    }

    m_chunksX = (m_cols + kChunkSize - 1) / kChunkSize;
    m_chunksY = (m_rows + kChunkSize - 1) / kChunkSize;
    m_numChunks = m_chunksX * m_chunksY;
    m_chunkActive.assign(m_numChunks, 0);
    m_chunkIdleFrames.assign(m_numChunks, 0);
}

void LightField::resetAccumulation() {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::fill(m_accum.begin(), m_accum.end(), 0.0f);
}

int LightField::chunkNeighbors(int chunkCol, int chunkRow, int out[8]) const {
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

void LightField::step(float dt, float waveSpeedSq, float dampingRate, float dispersion) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (dt < 0.0f) return;

    const int n = static_cast<int>(m_height.size());

    // dt==0 (пауза) раньше делало ранний return - pluck()/brush() пишут в
    // m_height/m_velocity напрямую, но m_glow пересчитывается только здесь,
    // так что взаимодействие на паузе оставалось невидимым до следующего
    // живого шага. Интегратор всё ещё пропускаем при dt==0, но яркость и
    // активность чанков теперь считаются безусловно, ниже.
    const bool doPhysics = dt > 0.0f;

    // Устойчивость - тот же приём, что в cloth-модели (SpringNetwork::step):
    // дробим шаг на substeps так, чтобы waveSpeedSq*dtSub^2 оставался в
    // безопасных пределах явного интегратора.
    constexpr float kStabilityLimit = 2.0f;
    constexpr int kMaxSubsteps = 16;
    int substeps = 1;
    if (doPhysics && waveSpeedSq > 0.0f) {
        float need = dt * std::sqrt(waveSpeedSq / kStabilityLimit);
        substeps = std::clamp(static_cast<int>(std::ceil(need)), 1, kMaxSubsteps);
    }
    if (doPhysics) m_lastSubsteps.store(substeps, std::memory_order_relaxed);

    if (doPhysics) {
    const float subDt = dt / static_cast<float>(substeps);
    const float dampFactor = std::exp(-dampingRate * subDt);

    // Клэмп скорости вместо телепорта на NaN/Inf - тот же трюк, что решил
    // "замятины" в cloth-модели (см. её step()).
    constexpr float kMaxDisplacementPerSubstep = 2.0f;
    const float maxSpeed = kMaxDisplacementPerSubstep * m_spacing / subDt;

    // Список чанков к обработке - активные + их halo-соседи (см. .h). Без
    // CSR/рёбер: "обработать чанк" значит просто пройти его прямоугольник
    // узлов напрямую индексами.
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

    for (int s = 0; s < substeps; ++s) {
        // Фаза 1 - сила на каждый узел процессируемых чанков. Каждый узел
        // ЧИТАЕТ 6 гекс-соседей (общий, не мутируемый в этой фазе m_height) и
        // ПИШЕТ только в свой собственный m_force[i] - race-free при любом
        // разбиении на потоки/чанки, без атомиков и без CSR (в отличие от
        // cloth-модели, где пружина связывала ДВА узла и писала в оба -
        // здесь запись всегда в "себя", соседи только читаются).
        parallelFor(numProcessChunks, [&](int begin, int end, int) {
            for (int idx = begin; idx < end; ++idx) {
                int c = m_processChunks[static_cast<size_t>(idx)];
                int cx = c % m_chunksX, cy = c / m_chunksX;
                int rowLo = std::max(1, cy * kChunkSize);
                int rowHi = std::min((cy + 1) * kChunkSize, m_rows - 1);   // исключая границу
                int colLo = std::max(1, cx * kChunkSize);
                int colHi = std::min((cx + 1) * kChunkSize, m_cols - 1);
                for (int r = rowLo; r < rowHi; ++r) {
                    for (int col = colLo; col < colHi; ++col) {
                        int i = index(col, r);
                        if (m_pinned[static_cast<size_t>(i)]) { m_force[static_cast<size_t>(i)] = 0.0f; continue; }

                        // Гекс-лапласиан: 6 равноудалённых соседей с
                        // единичными весами - sum(6 соседей) - 6*center, без
                        // деления на осевые/диагональные классы (см. .h).
                        // Смещения из HexGrid::neighborOffsets() всегда в
                        // пределах {-1,0,+1} по col и row, для внутреннего
                        // узла (rowLo/rowHi/colLo/colHi выше уже исключают
                        // границу) индексы соседей гарантированно валидны.
                        float c0 = m_height[static_cast<size_t>(i)];
                        const auto* offs = HexGrid::neighborOffsets(r);
                        float sum = 0.0f;
                        for (int k = 0; k < 6; ++k) {
                            sum += m_height[static_cast<size_t>(index(col + offs[k][0], r + offs[k][1]))];
                        }
                        float lap = (sum - 6.0f * c0) / 6.0f;

                        float medium = m_mediumMask[static_cast<size_t>(i)];
                        float speedSq = std::max(0.0f, waveSpeedSq * (1.0f - medium * dispersion));
                        m_force[static_cast<size_t>(i)] = speedSq * lap;
                    }
                }
            }
        });

        // Фаза 2 - интегрирование, безусловно по ВСЕМ n узлам (та же
        // причина, что в cloth-модели: узел на границе активного/неактивного
        // чанка мог получить ненулевую силу - граница обработки чанка выше
        // (rowLo/rowHi) НЕ пересекает саму себя между чанками, но halo уже
        // включил соседей, так что это просто безопасный процесс поверх
        // финального m_force).
        parallelFor(n, [&](int begin, int end, int) {
            for (int i = begin; i < end; ++i) {
                if (m_pinned[static_cast<size_t>(i)]) continue;
                float v = m_velocity[static_cast<size_t>(i)];
                v += m_force[static_cast<size_t>(i)] * subDt;
                v *= dampFactor;
                if (std::fabs(v) > maxSpeed) v = (v > 0 ? maxSpeed : -maxSpeed);

                float h = m_height[static_cast<size_t>(i)] + v * subDt;
                if (!std::isfinite(h) || !std::isfinite(v)) { h = 0.0f; v = 0.0f; }

                m_velocity[static_cast<size_t>(i)] = v;
                m_height[static_cast<size_t>(i)] = h;
            }
        });
    }
    }   // doPhysics

    // Яркость - геометрическая (лог-)нормализация Reinhard-типа, как в
    // cloth-модели, но на max(|velocity|, |height|), не только скорость.
    // pluck() двигает только m_height - на паузе (doPhysics==false) скорость
    // ещё 0, интегратор её не превратил в движение, чисто по |velocity| щипок
    // был бы невидим до следующего живого шага. |height| - потенциальная
    // энергия, натянутая, но ещё не высвобожденная, тоже должна светиться.
    // Считается безусловно, даже на паузе.
    std::vector<float> speedBuf(static_cast<size_t>(n));
    const size_t numThreads = std::max<size_t>(1, TaskScheduler::instance().thread_count());
    std::vector<float> sumLogT(numThreads, 0.0f);
    parallelFor(n, [&](int begin, int end, int t) {
        float sumLog = 0.0f;
        for (int i = begin; i < end; ++i) {
            float speed = std::max(std::fabs(m_velocity[static_cast<size_t>(i)]),
                                    std::fabs(m_height[static_cast<size_t>(i)]));
            speedBuf[static_cast<size_t>(i)] = speed;
            sumLog += std::log(speed + kLogEps);
        }
        sumLogT[static_cast<size_t>(t)] = sumLog;
    });
    float totalLog = 0.0f;
    for (float v : sumLogT) totalLog += v;
    float avgSpeed = n > 0
        ? std::max(std::exp(totalLog / static_cast<float>(n)), kMinAvgSpeed * m_spacing)
        : kMinAvgSpeed * m_spacing;
    m_lastAvgSpeed.store(avgSpeed, std::memory_order_relaxed);

    parallelFor(n, [&](int begin, int end, int) {
        for (int i = begin; i < end; ++i) {
            float normLinear = speedBuf[static_cast<size_t>(i)] / (speedBuf[static_cast<size_t>(i)] + avgSpeed);
            float norm = std::pow(normLinear, kGlowContrast);
            m_glow[static_cast<size_t>(i)] = std::max(norm, m_glow[static_cast<size_t>(i)] * kGlowDecay);
            // Долгая выдержка - монотонно растёт, никогда не затухает сама
            // (в отличие от m_glow выше), main.cpp клэмпит/возводит в
            // степень при отображении, см. её описание в .h. Копится
            // normLinear (та же адаптивная относительно avgSpeed доля, что и
            // у glow до контраста), не сырой |height| - см. пояснение у
            // kAccumRate выше.
            m_accum[static_cast<size_t>(i)] += normLinear * kAccumRate * dt;
        }
    });

    // Пересчёт активности чанков - та же гистерезис-схема, что в
    // cloth-модели, но проверяем И скорость, И "сырую" амплитуду (|h|), а не
    // только скорость: узел в момент разворота колебания (v проходит через
    // ноль дважды за период) всё ещё несёт энергию в h - проверка только по
    // |velocity| рисковала бы усыпить чанк ровно в этот момент.
    constexpr int kIdleFramesToDeactivate = 12;
    const float wakeSpeed = kMinAvgSpeed * m_spacing;
    const float wakeHeight = kMinAvgHeight * m_spacing;
    for (int cy = 0; cy < m_chunksY; ++cy) {
        for (int cx = 0; cx < m_chunksX; ++cx) {
            int c = cy * m_chunksX + cx;
            int rowLo = cy * kChunkSize, rowHi = std::min(rowLo + kChunkSize, m_rows);
            int colLo = cx * kChunkSize, colHi = std::min(colLo + kChunkSize, m_cols);
            bool energetic = false;
            for (int r = rowLo; r < rowHi && !energetic; ++r) {
                for (int cc = colLo; cc < colHi; ++cc) {
                    size_t i = static_cast<size_t>(index(cc, r));
                    if (speedBuf[i] > wakeSpeed || std::fabs(m_height[i]) > wakeHeight) { energetic = true; break; }
                }
            }
            if (energetic) {
                m_chunkActive[static_cast<size_t>(c)] = 1;
                m_chunkIdleFrames[static_cast<size_t>(c)] = 0;
            } else if (m_chunkActive[static_cast<size_t>(c)] && ++m_chunkIdleFrames[static_cast<size_t>(c)] >= kIdleFramesToDeactivate) {
                m_chunkActive[static_cast<size_t>(c)] = 0;
                m_chunkIdleFrames[static_cast<size_t>(c)] = 0;
            }
        }
    }
}

void LightField::activateChunksInWindow(int colLo, int colHi, int rowLo, int rowHi) {
    int chunkColLo = colLo / kChunkSize, chunkColHi = colHi / kChunkSize;
    int chunkRowLo = rowLo / kChunkSize, chunkRowHi = rowHi / kChunkSize;
    for (int cy = chunkRowLo; cy <= chunkRowHi; ++cy) {
        for (int cx = chunkColLo; cx <= chunkColHi; ++cx) {
            int c = cy * m_chunksX + cx;
            m_chunkActive[static_cast<size_t>(c)] = 1;
            m_chunkIdleFrames[static_cast<size_t>(c)] = 0;
        }
    }
}

void LightField::activateChunkAt(int nodeIndex) {
    int col = nodeIndex % m_cols, row = nodeIndex / m_cols;
    activateChunksInWindow(col, col, row, row);
}

void LightField::windowAround(glm::vec2 worldPos, float radiusCells,
                               int& colLo, int& colHi, int& rowLo, int& rowHi) const {
    // Обратное преобразование мировой позиции в (col,row) - только ПРИБЛИЖЁННОЕ
    // (для оценки строки игнорируется полуклеточный сдвиг нечётных строк по
    // col, учитывается только после того, как rowC уже известен), этого
    // достаточно - вызывающие (pluck/brush/...) всё равно потом ищут ближайший
    // узел ПЕРЕБОРОМ по фактическому расстоянию внутри окна (см. их код), а
    // не полагаются на точность (colC,rowC) как на финальный ответ.
    float horiz = HexGrid::horizSpacing(m_spacing);
    float vert = HexGrid::vertSpacing(m_spacing);
    int rowC = static_cast<int>(std::round(worldPos.y / vert));
    int colC = static_cast<int>(std::round(worldPos.x / horiz - ((rowC & 1) ? 0.5f : 0.0f)));
    int half = static_cast<int>(std::ceil(radiusCells));
    colLo = std::clamp(colC - half, 0, m_cols - 1);
    colHi = std::clamp(colC + half, 0, m_cols - 1);
    rowLo = std::clamp(rowC - half, 0, m_rows - 1);
    rowHi = std::clamp(rowC + half, 0, m_rows - 1);
}

void LightField::pluck(glm::vec2 worldPos, float strength) {
    std::lock_guard<std::mutex> lock(m_mutex);
    constexpr float kSearchCells = 3.0f;
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, kSearchCells, colLo, colHi, rowLo, rowHi);
    int best = -1;
    float bestDist = std::numeric_limits<float>::max();
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            float d = glm::length(p - worldPos);
            if (d < bestDist) { bestDist = d; best = i; }
        }
    }
    if (best < 0) return;
    activateChunkAt(best);
    // Клэмп как в cloth-модели (SpringNetwork::pluck()) - без него абсурдный
    // strength из текстового поля (sliderWithInput() не обрезает ручной ввод)
    // даёт высоту на порядки больше нормы. Клэмп скорости в step() абсолютный,
    // не пропорциональный высоте - без этого клэмпа стравливание такой
    // аномалии заняло бы миллиарды кадров, выглядело бы как замёрзшая
    // картинка, не как волна.
    constexpr float kMaxPluckDisplacement = 3.0f;   // в единицах m_spacing
    float clamped = std::clamp(strength, -kMaxPluckDisplacement * m_spacing, kMaxPluckDisplacement * m_spacing);
    m_height[static_cast<size_t>(best)] += clamped;
}

void LightField::brush(glm::vec2 worldPos, float radius, float strength, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (radius <= 0.0f || dt <= 0.0f) return;
    constexpr float kMarginCells = 4.0f;
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, radius / HexGrid::horizSpacing(m_spacing) + kMarginCells, colLo, colHi, rowLo, rowHi);
    activateChunksInWindow(colLo, colHi, rowLo, rowHi);
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            float dist = glm::length(p - worldPos);
            if (dist >= radius) continue;
            float falloff = 1.0f - (dist / radius);
            m_velocity[static_cast<size_t>(i)] += strength * falloff * dt;
        }
    }
}

void LightField::brushDamp(glm::vec2 worldPos, float radius, float strength, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (radius <= 0.0f || dt <= 0.0f) return;
    constexpr float kMarginCells = 4.0f;
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, radius / HexGrid::horizSpacing(m_spacing) + kMarginCells, colLo, colHi, rowLo, rowHi);
    activateChunksInWindow(colLo, colHi, rowLo, rowHi);
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            float dist = glm::length(p - worldPos);
            if (dist >= radius) continue;
            float falloff = 1.0f - (dist / radius);
            float damp = std::clamp(strength * falloff * dt, 0.0f, 1.0f);
            m_velocity[static_cast<size_t>(i)] *= (1.0f - damp);
        }
    }
}

void LightField::paintMedium(glm::vec2 worldPos, float radius, float strength, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (radius <= 0.0f || dt <= 0.0f) return;
    constexpr float kMarginCells = 4.0f;
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, radius / HexGrid::horizSpacing(m_spacing) + kMarginCells, colLo, colHi, rowLo, rowHi);
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            float dist = glm::length(p - worldPos);
            if (dist >= radius) continue;
            float falloff = 1.0f - (dist / radius);
            float& mask = m_mediumMask[static_cast<size_t>(i)];
            mask = std::clamp(mask + strength * falloff * dt, 0.0f, 1.0f);
        }
    }
}

void LightField::eraseMedium(glm::vec2 worldPos, float radius, float strength, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (radius <= 0.0f || dt <= 0.0f) return;
    constexpr float kMarginCells = 4.0f;
    int colLo, colHi, rowLo, rowHi;
    windowAround(worldPos, radius / HexGrid::horizSpacing(m_spacing) + kMarginCells, colLo, colHi, rowLo, rowHi);
    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            float dist = glm::length(p - worldPos);
            if (dist >= radius) continue;
            float falloff = 1.0f - (dist / radius);
            float& mask = m_mediumMask[static_cast<size_t>(i)];
            mask = std::clamp(mask - strength * falloff * dt, 0.0f, 1.0f);
        }
    }
}

void LightField::paintMediumPolygon(const std::vector<glm::vec2>& polygonWorld, float value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (polygonWorld.size() < 3) return;
    const float clampedValue = std::clamp(value, 0.0f, 1.0f);
    const int vertCount = static_cast<int>(polygonWorld.size());

    // Чётно-нечётное правило (ray casting) - стандартный point-in-polygon
    // тест, полигон не обязан быть выпуклым. Полный проход по сетке -
    // вызывается по кнопке пресета, не каждый кадр, оптимизировать нечего.
    for (int r = 1; r < m_rows - 1; ++r) {
        for (int c = 1; c < m_cols - 1; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            bool inside = false;
            for (int a = 0, b = vertCount - 1; a < vertCount; b = a++) {
                const glm::vec2& pa = polygonWorld[static_cast<size_t>(a)];
                const glm::vec2& pb = polygonWorld[static_cast<size_t>(b)];
                bool crosses = ((pa.y > p.y) != (pb.y > p.y)) &&
                    (p.x < (pb.x - pa.x) * (p.y - pa.y) / (pb.y - pa.y) + pa.x);
                if (crosses) inside = !inside;
            }
            if (inside) m_mediumMask[static_cast<size_t>(i)] = clampedValue;
        }
    }
}

void LightField::beam(glm::vec2 origin, glm::vec2 direction, float aperture,
                       float frequency, float strength, float time, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (dt <= 0.0f || aperture <= 0.0f) return;
    float dirLen = glm::length(direction);
    if (dirLen < 1e-5f) return;
    glm::vec2 dir = direction / dirLen;
    glm::vec2 perp(-dir.y, dir.x);   // вдоль perp располагаются узлы-излучатели

    constexpr float kTwoPi = 6.283185307179586f;
    // Толщина полосы вдоль direction - геометрически линия излучателей
    // бесконечно тонкая, но сетка дискретна: берём пару клеток запаса, чтобы
    // хотя бы один ряд узлов гарантированно попал в полосу на любой
    // ориентации direction.
    const float thickness = hexHorizSpacing() * 1.5f;
    const float halfAperture = aperture * 0.5f;

    int colLo, colHi, rowLo, rowHi;
    windowAround(origin, (halfAperture + thickness) / hexHorizSpacing() + 4.0f,
                 colLo, colHi, rowLo, rowHi);

    // Одна и та же фаза для ВСЕХ излучателей линии (нулевой сдвиг фазы вдоль
    // perp) - интерференция сама выстраивает направленность вдоль direction
    // без явного управления фазой на узел, та же физика, что у синфазной
    // антенной решётки.
    float injection = strength * std::sin(kTwoPi * frequency * time) * dt;

    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 rel = LightField::worldPos(c, r) - origin;
            float along = glm::dot(rel, perp);
            float across = glm::dot(rel, dir);
            if (std::fabs(along) > halfAperture || std::fabs(across) > thickness) continue;
            m_velocity[static_cast<size_t>(i)] += injection;
        }
    }
    activateChunksInWindow(colLo, colHi, rowLo, rowHi);
}

void LightField::snapshot(std::vector<float>& outGlow, std::vector<float>& outMediumMask,
                           std::vector<float>& outAccum) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    outGlow = m_glow;
    outMediumMask = m_mediumMask;
    outAccum = m_accum;
}
