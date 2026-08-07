// demo/light/LightField.h
#pragma once
#include "engine/core/HexGrid.h"
#include <vector>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <glm/glm.hpp>

// Скалярное волновое поле на фиксированной гекс-решётке (d^2h/dt^2 =
// c^2*lap(h)) - одна честная скорость волны на экземпляр, в отличие от
// demo/cloth (узел двигался в плоскости, продольные/поперечные моды
// расходятся по скорости). Нет списка рёбер - каждый узел пишет только
// свой m_force[i], race-free без атомиков и CSR.
class LightField {
public:
    LightField(int cols, int rows, float spacing);

    glm::vec2 worldPos(int col, int row) const;

    // dispersion - насколько цвет тормозится в призме (m_mediumMask):
    // эффективная скорость = waveSpeedSq * (1 - mediumMask[i]*dispersion).
    void step(float dt, float waveSpeedSq, float dampingRate, float dispersion);

    void pluck(glm::vec2 worldPos, float strength);
    void brush(glm::vec2 worldPos, float radius, float strength, float dt);
    void brushDamp(glm::vec2 worldPos, float radius, float strength, float dt);

    // Среда ("призма") - свойство поля, отдельное от амплитуды.
    void paintMedium(glm::vec2 worldPos, float radius, float strength, float dt);
    void eraseMedium(glm::vec2 worldPos, float radius, float strength, float dt);
    // Заливает m_mediumMask значением (не приращением) внутри многоугольника -
    // для готовых карт (main.cpp), не для рисования мышью.
    void paintMediumPolygon(const std::vector<glm::vec2>& polygonWorld, float value);

    // Фазированная линия излучателей (окно Ханна) перпендикулярно direction -
    // направленность строится интерференцией, не "стеной"/каналом.
    void beam(glm::vec2 origin, glm::vec2 direction, float aperture,
              float frequency, float strength, float time, float dt);

    void reset();
    void resetAccumulation();   // обнуляет только m_accum, не волну/среду

    void snapshot(std::vector<float>& outGlow, std::vector<float>& outMediumMask,
                  std::vector<float>& outAccum) const;

    // Чтение готового кадра БЕЗ копирования - указатели живут в слоте, куда
    // физика не пишет (см. kSlots). Дешевле snapshot() ровно на стоимость
    // копии ~5.7МБ, которая и была основной платой за кадр.
    struct View {
        const float* glow = nullptr;
        const float* mediumMask = nullptr;
        const float* accum = nullptr;
    };
    View acquireView() const;

    int cols() const { return m_cols; }
    int rows() const { return m_rows; }
    float spacing() const { return m_spacing; }
    float hexHorizSpacing() const;
    float hexVertSpacing() const;

    static constexpr int kChunkSize = 64;
    int chunksX() const { return m_chunksX; }
    int chunksY() const { return m_chunksY; }

private:
    int index(int col, int row) const { return row * m_cols + col; }

    void windowAround(glm::vec2 worldPos, float radiusCells,
                       int& colLo, int& colHi, int& rowLo, int& rowHi) const;

    int chunkIndexOf(int col, int row) const {
        return (row / kChunkSize) * m_chunksX + (col / kChunkSize);
    }
    int chunkNeighbors(int chunkCol, int chunkRow, int out[8]) const;
    void activateChunkAt(int nodeIndex);
    void activateChunksInWindow(int colLo, int colHi, int rowLo, int rowHi);
    void publish();   // вызывать под m_mutex, см. слоты ниже
    void syncChunkAcrossSlots(int rowLo, int rowHi, int colLo, int colHi);

    int m_cols, m_rows;
    float m_spacing;

    mutable std::mutex m_mutex;
    std::vector<float> m_height, m_velocity, m_force;
    std::vector<float> m_mediumMask;   // рабочая копия, в неё пишет кисть
    std::vector<uint8_t> m_pinned;

    // Публикация для рендера - ротация слотов без копирования.
    //
    // Сначала snapshot() брал m_mutex, тот же, что step() держит весь шаг:
    // рендер ждал шаг целиком, физика на следующем шаге ждала уже рендер.
    // Промежуточный вариант (копия в отдельные буферы под коротким локом)
    // рендер разблокировал, но физике стало ХУЖЕ: копия ~5.7МБ переехала на
    // неё и продолжила вымывать кэш - платой был не лок, а сам трафик.
    //
    // Теперь копий нет вовсе. Физика пишет в слот m_writeSlot, читая
    // предыдущий (затухание glow и EMA accum и так смотрят на прошлый кадр),
    // затем публикует индекс одним atomic-store. Рендер берёт этот индекс и
    // читает слот напрямую. Слотов три, а не два: физика уходит на два кадра
    // вперёд, прежде чем вернуться к слоту, который сейчас читает рендер.
    static constexpr int kSlots = 3;
    std::vector<float> m_glowSlot[kSlots], m_accumSlot[kSlots], m_maskSlot[kSlots];
    int m_writeSlot = 0;                  // только поток физики
    std::atomic<int> m_readySlot{0};      // последний дописанный слот
    int m_maskDirtyFrames = kSlots;       // маска меняется редко - разливаем по слотам

    int m_chunksX = 0, m_chunksY = 0, m_numChunks = 0;
    std::vector<uint8_t> m_chunkActive, m_chunkIdleFrames;
    std::vector<int> m_processChunks;

    std::atomic<float> m_lastAvgSpeed{0.0f};
    std::atomic<int> m_lastSubsteps{1};
public:
    float lastAvgSpeed() const { return m_lastAvgSpeed.load(std::memory_order_relaxed); }
    int lastSubsteps() const { return m_lastSubsteps.load(std::memory_order_relaxed); }
};
