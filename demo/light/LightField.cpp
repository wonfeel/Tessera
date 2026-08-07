// demo/light/LightField.cpp
#include "demo/light/LightField.h"
#include "engine/core/ParallelFor.h"
#include "engine/core/Geometry.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
    constexpr float kGlowDecay = 0.90f;
    constexpr float kLogEps = 1e-4f;
    constexpr float kMinAvgSpeed = 0.05f;
    constexpr float kMinAvgHeight = 0.01f;

    // m_accum - EMA к текущему normLinear (dA/dt = kAccumRate*(target-A)):
    // при constexpr rate решение за подшаг - accum + (target-accum)*(1-exp(-rate*dt)),
    // строго ограничено диапазоном [min(accum,target), max(accum,target)] - в
    // отличие от раздельных rate/decay не может уйти выше 1 под непрерывным
    // источником (авто-луч), только к нему сходится.
    constexpr float kAccumRate = 0.15f;   // ~6.7с до 63% скачка
}

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
    for (int s = 0; s < kSlots; ++s) {
        m_glowSlot[s].assign(n, 0.0f);
        m_accumSlot[s].assign(n, 0.0f);
        m_maskSlot[s].assign(n, 0.0f);
    }
    m_writeSlot = 0;
    m_readySlot.store(0, std::memory_order_release);
    m_maskDirtyFrames = kSlots;

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
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
    // Чистим все слоты: следующий шаг читает предыдущий как основу EMA, так
    // что уцелевший в нём хвост протёк бы обратно в свежий кадр.
    for (int s = 0; s < kSlots; ++s)
        std::fill(m_accumSlot[s].begin(), m_accumSlot[s].end(), 0.0f);
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

    // dt==0 (пауза) - интегратор пропускаем, но glow/активность чанков всё
    // равно считаются ниже: pluck()/brush() пишут в m_height/m_velocity
    // напрямую, а m_glow пересчитывается только здесь.
    const bool doPhysics = dt > 0.0f;

    constexpr float kStabilityLimit = 2.0f;
    constexpr int kMaxSubsteps = 16;
    int substeps = 1;
    if (doPhysics && waveSpeedSq > 0.0f) {
        float need = dt * std::sqrt(waveSpeedSq / kStabilityLimit);
        substeps = std::clamp(static_cast<int>(std::ceil(need)), 1, kMaxSubsteps);
    }
    if (doPhysics) m_lastSubsteps.store(substeps, std::memory_order_relaxed);

    // Footprint активных чанков (+соседи) - нужен физике и яркости
    // одинаково, считаем один раз вне doPhysics, чтобы работал и на паузе
    // (pluck() на паузе тоже будит чанк, glow должен подсветиться).
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

    if (doPhysics) {
    const float subDt = dt / static_cast<float>(substeps);
    const float dampFactor = std::exp(-dampingRate * subDt);

    // Поглощающий слой у края - m_pinned-граница сама по себе отражает
    // энергию полностью, превращая конечное поле в резонатор вместо
    // открытого пространства. Затухание растёт к краю квадратично, не
    // ступенькой (резкий скачок сам даёт отражение). dampingRate в толще
    // поля может быть 0 - у света нет потерь на трение о вакуум/воздух,
    // именно граница должна быть агрессивно поглощающей.
    constexpr int kAbsorbLayerCells = 40;
    constexpr float kAbsorbExtraDamping = 400.0f;
    float dampLookup[kAbsorbLayerCells + 1];
    for (int d = 0; d <= kAbsorbLayerCells; ++d) {
        float t = 1.0f - static_cast<float>(d) / static_cast<float>(kAbsorbLayerCells);
        dampLookup[d] = std::exp(-(dampingRate + t * t * kAbsorbExtraDamping) * subDt);
    }

    constexpr float kMaxDisplacementPerSubstep = 2.0f;
    const float maxSpeed = kMaxDisplacementPerSubstep * m_spacing / subDt;

    for (int s = 0; s < substeps; ++s) {
        // Каждый узел читает 6 гекс-соседей и пишет только в свой m_force[i] -
        // race-free без атомиков и без CSR.
        parallelFor(numProcessChunks, [&](int begin, int end, int) {
            for (int idx = begin; idx < end; ++idx) {
                int c = m_processChunks[static_cast<size_t>(idx)];
                int cx = c % m_chunksX, cy = c / m_chunksX;
                int rowLo = std::max(1, cy * kChunkSize);
                int rowHi = std::min((cy + 1) * kChunkSize, m_rows - 1);
                int colLo = std::max(1, cx * kChunkSize);
                int colHi = std::min((cx + 1) * kChunkSize, m_cols - 1);
                for (int r = rowLo; r < rowHi; ++r) {
                    for (int col = colLo; col < colHi; ++col) {
                        int i = index(col, r);
                        if (m_pinned[static_cast<size_t>(i)]) { m_force[static_cast<size_t>(i)] = 0.0f; continue; }

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

        // Тот же footprint, что и у силы выше - иначе на спящих узлах
        // копилось бы протухшее m_force с момента, когда чанк ещё не спал
        // (сила там больше не пересчитывается). Чанк засыпает только когда
        // v и h уже ниже wake-порогов, так что заморозка их тут незаметна.
        parallelFor(numProcessChunks, [&](int begin, int end, int) {
            for (int idx = begin; idx < end; ++idx) {
                int c = m_processChunks[static_cast<size_t>(idx)];
                int cx = c % m_chunksX, cy = c / m_chunksX;
                int rowLo = std::max(1, cy * kChunkSize);
                int rowHi = std::min((cy + 1) * kChunkSize, m_rows - 1);
                int colLo = std::max(1, cx * kChunkSize);
                int colHi = std::min((cx + 1) * kChunkSize, m_cols - 1);
                for (int r = rowLo; r < rowHi; ++r) {
                    for (int col = colLo; col < colHi; ++col) {
                        int i = index(col, r);
                        if (m_pinned[static_cast<size_t>(i)]) continue;
                        float v = m_velocity[static_cast<size_t>(i)];
                        v += m_force[static_cast<size_t>(i)] * subDt;

                        int distToEdge = std::min(std::min(col, m_cols - 1 - col), std::min(r, m_rows - 1 - r));
                        v *= (distToEdge < kAbsorbLayerCells) ? dampLookup[distToEdge] : dampFactor;
                        if (std::fabs(v) > maxSpeed) v = (v > 0 ? maxSpeed : -maxSpeed);

                        float h = m_height[static_cast<size_t>(i)] + v * subDt;
                        if (!std::isfinite(h) || !std::isfinite(v)) { h = 0.0f; v = 0.0f; }

                        m_velocity[static_cast<size_t>(i)] = v;
                        m_height[static_cast<size_t>(i)] = h;
                    }
                }
            }
        });
    }
    }   // doPhysics

    // Яркость - Reinhard-подобная нормализация на max(|velocity|, |height|):
    // pluck() двигает только m_height, на паузе интегратор ещё не превратил
    // это в скорость, чисто по |velocity| щипок был бы не виден.
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

    // alpha не зависит от узла (dt один на весь step()) - считаем раз, не
    // на каждый из 160к*3 узлов. norm - куб вместо std::pow(x,3), тот же
    // контраст в разы дешевле (нет общего показателя степени, он и не нужен -
    // экспонента всегда 3).
    const float accumAlpha = 1.0f - std::exp(-kAccumRate * dt);
    // Пишем в текущий слот, читая предыдущий - затухание glow и EMA accum и
    // так опираются на прошлый кадр, так что ротация им ничего не стоит.
    const int prevSlot = (m_writeSlot + kSlots - 1) % kSlots;
    float* glowCur = m_glowSlot[m_writeSlot].data();
    const float* glowPrev = m_glowSlot[prevSlot].data();
    float* accumCur = m_accumSlot[m_writeSlot].data();
    const float* accumPrev = m_accumSlot[prevSlot].data();

    // Тот же footprint активных чанков - у спящего узла normLinear уже
    // ниже wake-порога, его glow/accum и так должны быть у нуля; не
    // пересчитывать их каждый кадр на всём поле безопасно и на порядок
    // дешевле для большого в основном спящего поля.
    parallelFor(numProcessChunks, [&](int begin, int end, int) {
        for (int idx = begin; idx < end; ++idx) {
            int c = m_processChunks[static_cast<size_t>(idx)];
            int cx = c % m_chunksX, cy = c / m_chunksX;
            int rowLo = std::max(1, cy * kChunkSize);
            int rowHi = std::min((cy + 1) * kChunkSize, m_rows - 1);
            int colLo = std::max(1, cx * kChunkSize);
            int colHi = std::min((cx + 1) * kChunkSize, m_cols - 1);
            for (int r = rowLo; r < rowHi; ++r) {
                for (int col = colLo; col < colHi; ++col) {
                    int i = index(col, r);
                    float normLinear = speedBuf[static_cast<size_t>(i)] / (speedBuf[static_cast<size_t>(i)] + avgSpeed);
                    float norm = normLinear * normLinear * normLinear;
                    float g = std::max(norm, glowPrev[i] * kGlowDecay);
                    glowCur[i] = g;
                    // Копим уже контрастный glow, не сырой normLinear - в
                    // разреженном (по большей части спящем) поле avgSpeed
                    // крошечный, и даже слабая рябь получает normLinear~1;
                    // долгая выдержка такое усредняет в сплошной белый. glow
                    // уже давит слабое кубом.
                    float a = accumPrev[i];
                    accumCur[i] = a + (g - a) * accumAlpha;
                }
            }
        }
    });

    // Активность чанков - гистерезис по скорости И по |height| (узел в
    // момент разворота колебания всё ещё несёт энергию в h).
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
                // Уснувший чанк больше не переписывается каждый кадр, а слоты
                // чередуются - без этого рендер увидел бы, как его последние
                // значения мигают по кругу из трёх устаревших копий. Ровняем
                // слоты один раз, на переходе; пока чанк спит, они совпадают.
                syncChunkAcrossSlots(rowLo, rowHi, colLo, colHi);
            }
        }
    }

    publish();
}

void LightField::syncChunkAcrossSlots(int rowLo, int rowHi, int colLo, int colHi) {
    for (int s = 0; s < kSlots; ++s) {
        if (s == m_writeSlot) continue;
        for (int r = rowLo; r < rowHi; ++r) {
            size_t from = static_cast<size_t>(index(colLo, r));
            size_t count = static_cast<size_t>(colHi - colLo);
            std::copy_n(m_glowSlot[m_writeSlot].begin() + from, count, m_glowSlot[s].begin() + from);
            std::copy_n(m_accumSlot[m_writeSlot].begin() + from, count, m_accumSlot[s].begin() + from);
        }
    }
}

// Публикация без копирования: маска переливается в слоты только когда её
// реально меняли (кисть/карта), а сам кадр отдаётся одним atomic-store
// индекса. Дальше физика уходит на следующий слот.
void LightField::publish() {
    // Маску переливаем ТОЛЬКО в свой слот и только пока счётчик не обнулится:
    // разом во все слоты нельзя - один из них в этот момент читает рендер.
    // За kSlots шагов новая маска доходит до каждого, и каждый раз запись
    // идёт в слот, который ещё не опубликован.
    if (m_maskDirtyFrames > 0) {
        m_maskSlot[m_writeSlot] = m_mediumMask;
        --m_maskDirtyFrames;
    }
    m_readySlot.store(m_writeSlot, std::memory_order_release);
    m_writeSlot = (m_writeSlot + 1) % kSlots;
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
    // Приближённое обратное преобразование - вызывающие всё равно ищут
    // ближайший узел перебором по факту, точность (colC,rowC) не критична.
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
    // Клэмп - неограниченный strength из текстового поля (слайдер не
    // обрезает ручной ввод) даёт высоту на порядки больше нормы, а клэмп
    // скорости в step() абсолютный - стравливание такой аномалии заняло бы
    // миллиарды кадров.
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
    m_maskDirtyFrames = kSlots;   // разольётся по слотам за следующие шаги
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
    m_maskDirtyFrames = kSlots;
}

void LightField::paintMediumPolygon(const std::vector<glm::vec2>& polygonWorld, float value) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (polygonWorld.size() < 3) return;
    const float clampedValue = std::clamp(value, 0.0f, 1.0f);

    // Резкая граница 0->1 - для FDTD-сетки это скачок импеданса, волна
    // отражается вместо честного преломления. featherWidth растягивает
    // переход на несколько клеток.
    const float featherWidth = HexGrid::horizSpacing(m_spacing) * 1.5f;

    for (int r = 1; r < m_rows - 1; ++r) {
        for (int c = 1; c < m_cols - 1; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 p = LightField::worldPos(c, r);
            float minEdgeDist;
            if (Geometry::pointInPolygon(p, polygonWorld, minEdgeDist)) {
                float t = std::clamp(minEdgeDist / featherWidth, 0.0f, 1.0f);
                m_mediumMask[static_cast<size_t>(i)] = clampedValue * t;
            }
        }
    }
    m_maskDirtyFrames = kSlots;
}

void LightField::beam(glm::vec2 origin, glm::vec2 direction, float aperture,
                       float frequency, float strength, float time, float dt) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (dt <= 0.0f || aperture <= 0.0f) return;
    float dirLen = glm::length(direction);
    if (dirLen < 1e-5f) return;
    glm::vec2 dir = direction / dirLen;
    glm::vec2 perp(-dir.y, dir.x);

    constexpr float kTwoPi = 6.283185307179586f;
    constexpr float kPi = 3.14159265358979323846f;
    const float thickness = hexHorizSpacing() * 1.5f;
    const float halfAperture = aperture * 0.5f;

    int colLo, colHi, rowLo, rowHi;
    windowAround(origin, (halfAperture + thickness) / hexHorizSpacing() + 4.0f,
                 colLo, colHi, rowLo, rowHi);

    // Одна фаза для всех излучателей линии - направленность вдоль direction
    // строится интерференцией, та же физика, что у фазированной решётки.
    float injection = strength * std::sin(kTwoPi * frequency * time) * dt;

    for (int r = rowLo; r <= rowHi; ++r) {
        for (int c = colLo; c <= colHi; ++c) {
            int i = index(c, r);
            if (m_pinned[static_cast<size_t>(i)]) continue;
            glm::vec2 rel = LightField::worldPos(c, r) - origin;
            float along = glm::dot(rel, perp);
            float across = glm::dot(rel, dir);
            if (std::fabs(along) > halfAperture || std::fabs(across) > thickness) continue;
            // Окно Ханна вдоль апертуры - резкий обрыв на краях линии даёт
            // sinc-паттерн (дифракция Фраунгофера) с медленно затухающими
            // боковыми лепестками - плавный спад до нуля их гасит.
            float taper = 0.5f * (1.0f + std::cos(kPi * along / halfAperture));
            m_velocity[static_cast<size_t>(i)] += injection * taper;
        }
    }
    activateChunksInWindow(colLo, colHi, rowLo, rowHi);
}

LightField::View LightField::acquireView() const {
    int s = m_readySlot.load(std::memory_order_acquire);
    return View{ m_glowSlot[s].data(), m_maskSlot[s].data(), m_accumSlot[s].data() };
}

// Совместимость для тех, кому нужна именно копия. Локов не берёт вообще -
// читает опубликованный слот, в который физика не пишет.
void LightField::snapshot(std::vector<float>& outGlow, std::vector<float>& outMediumMask,
                           std::vector<float>& outAccum) const {
    int s = m_readySlot.load(std::memory_order_acquire);
    outGlow = m_glowSlot[s];
    outMediumMask = m_maskSlot[s];
    outAccum = m_accumSlot[s];
}
