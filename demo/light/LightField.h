// demo/light/LightField.h
#pragma once
#include "engine/core/HexGrid.h"
#include <vector>
#include <mutex>
#include <atomic>
#include <cstdint>
#include <glm/glm.hpp>

// Скалярное волновое поле, замена пружинной 2D-сетки (demo/cloth) для волн
// света. Позиция узла (x,y) зафиксирована как решётка, единственная степень
// свободы - скалярная амплитуда h, эволюционирует по дискретному лапласиану
// (d²h/dt² = c²∇²h). В cloth-модели узел двигался в плоскости - продольные и
// поперечные упругие моды там расходятся по скорости, честной RGB-дисперсии
// на этом не построить. Здесь одна скорость на поле, main.cpp держит три
// экземпляра (R/G/B) с разной waveSpeedSq.
//
// Нет списка рёбер - сосед адресуется напрямую по индексу (index(c±1,r),
// index(c±1,r±1)). Каждый узел читает соседей (m_height, не мутируется в
// этой фазе) и пишет только в свой m_force[i] - race-free при любом
// разбиении на потоки, без атомиков и без CSR (в cloth-модели пружина
// связывала два узла и писала в оба, здесь только чтение соседа).
//
// Гекс-решётка (pointy-top, odd-r offset, см. engine/core/HexGrid.h) вместо
// квадратной: все 6 соседей равноудалены (sqrt(3)*spacing по всем
// направлениям), поэтому лапласиан - просто sum(6 соседей) - 6*center с
// единичными весами, без деления на осевые/диагональные классы, как на
// квадратной сетке (9-точечный стенсил, веса 4/1). Меньше соседей (6 вместо
// 8) и честно изотропнее - круглый фронт от точечного источника без
// квадратных артефактов.
class LightField {
public:
    LightField(int cols, int rows, float spacing);

    // Мировая позиция узла (col,row) - pointy-top гекс-раскладка (нечётные
    // строки сдвинуты по горизонтали на полклетки). Публичный, т.к. нужен и
    // рендеру (main.cpp собирает позицию вершины отсюда, не по формуле
    // "col*spacing" - так и физика, и картинка используют ОДНУ и ту же
    // геометрию, не две параллельные копии формулы).
    glm::vec2 worldPos(int col, int row) const;

    // waveSpeedSq - квадрат скорости волны (аналог stiffness в cloth-модели,
    // тот же смысл "тюнинг-константа", не привязанная к физическим единицам
    // - как и раньше, замедление времени должно идти через dt, а не через
    // количество вызовов step()). dispersion - насколько СИЛЬНО этот
    // конкретный цвет тормозится в области "призмы" (m_mediumMask) -
    // эффективная скорость на узле = waveSpeedSq * (1 - mediumMask[i]*dispersion),
    // клэмпится снизу нулём. Разные dispersion на R/G/B дают хроматическое
    // разделение при прохождении через призму.
    //
    // Пробовал делать число колец соседей настраиваемым (1-3) против
    // 6-кратной анизотропии решётки - не помогло на резком точечном щипке
    // (звезда там от высокочастотного содержимого источника, расширенный
    // стенсил лечит только низкие/средние частоты). Убрано, всегда 1 кольцо
    // (6 соседей).
    void step(float dt, float waveSpeedSq, float dampingRate, float dispersion);

    // Щипок - разовый импульс АМПЛИТУДЫ (не позиции, как в cloth-модели) в
    // ближайший свободный (не граничный) узел.
    void pluck(glm::vec2 worldPos, float strength);
    // Кисть - впрыск СКОРОСТИ (dh/dt) в радиусе, накапливается, пока
    // зажато - прямой аналог cloth-brush()/brushDamp(), только по амплитуде.
    void brush(glm::vec2 worldPos, float radius, float strength, float dt);
    void brushDamp(glm::vec2 worldPos, float radius, float strength, float dt);

    // Рисование "призмы": повышает m_mediumMask в радиусе (насыщается к
    // 1.0), НЕ трогает текущую волну - это свойство СРЕДЫ, отдельное от
    // амплитуды. eraseMedium - обратный ластик (понижает к 0.0).
    void paintMedium(glm::vec2 worldPos, float radius, float strength, float dt);
    void eraseMedium(glm::vec2 worldPos, float radius, float strength, float dt);

    // Заливает m_mediumMask ЗНАЧЕНИЕМ (не приращением, как paintMedium) во
    // всех узлах внутри многоугольника (мировые координаты, четно-нечетное
    // правило) - для готовых карт/пресетов (main.cpp), где форма призмы
    // задана заранее, не рисуется мышью. Полный проход по сетке, не через
    // windowAround - вызывается редко (по кнопке), не каждый кадр.
    void paintMediumPolygon(const std::vector<glm::vec2>& polygonWorld, float value);

    // Направленный пучок - фазированная линия излучателей, не "стена"/канал.
    // Узлы вдоль отрезка длиной aperture, идущего через origin перпендикулярно
    // direction, колеблются синхронно (одна фаза, sin(2π*frequency*time)) -
    // волны складываются конструктивно вдоль direction и гасят друг друга в
    // стороны, та же физика, что у фазированной антенной решётки. aperture
    // управляет шириной пучка (диффракционный предел: шире апертура в длинах
    // волн - уже и направленнее пучок, реальный компромисс, не баг). time -
    // накопленное время эмиссии (main.cpp считает, пока луч зажат), нужен для
    // непрерывной фазы между кадрами.
    void beam(glm::vec2 origin, glm::vec2 direction, float aperture,
              float frequency, float strength, float time, float dt);

    void reset();
    // Только обнуляет m_accum - не трогает волну/среду, в отличие от
    // полного reset(). Долгая выдержка монотонно растёт (см. .cpp), без
    // отдельного сброса со временем насытилась бы в белое.
    void resetAccumulation();

    // outGlow - нормализованная (Reinhard-подобная) яркость для рендера,
    // затухает со временем (kGlowDecay). outMediumMask - для затемнения
    // области призмы на экране (main.cpp). outAccum - копия накопительного
    // буфера m_accum (см. его описание у поля) для режима "накопление"
    // (main.cpp решает, каким буфером красить узел - glow или accum).
    void snapshot(std::vector<float>& outGlow, std::vector<float>& outMediumMask,
                  std::vector<float>& outAccum) const;

    int cols() const { return m_cols; }
    int rows() const { return m_rows; }
    float spacing() const { return m_spacing; }
    // Мировое расстояние между соседними узлами по горизонтали/вертикали -
    // нужно снаружи (main.cpp) для отбраковки вне-экранного диапазона
    // строк/столбцов по AABB камеры (см. worldPos() в .cpp - та же формула,
    // не дублируем её тут отдельной константой).
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

    int m_cols, m_rows;
    float m_spacing;

    mutable std::mutex m_mutex;
    std::vector<float> m_height, m_velocity, m_force;
    std::vector<float> m_mediumMask;      // 0..1, "плотность" призмы на узле
    std::vector<uint8_t> m_pinned;        // край сетки - h всегда 0 (отражение)
    std::vector<float> m_glow;
    // Долгая выдержка: копит нормализованную (адаптивную к avgSpeed, см.
    // .cpp) долю энергии узла каждый step(), никогда не затухает сама - тот
    // же приём, что accumulatedLight в reference-проекте (Light-Simulation-JS,
    // см. demo/light/README.md), но не завязана на сырую |height| напрямую,
    // чтобы скорость насыщения не зависела от текущих значений
    // waveSpeedSq/pluckStrength. Растёт монотонно, клэмпится при отображении
    // (main.cpp), не здесь - reset() обнуляет.
    std::vector<float> m_accum;

    int m_chunksX = 0, m_chunksY = 0, m_numChunks = 0;
    std::vector<uint8_t> m_chunkActive, m_chunkIdleFrames;
    std::vector<int> m_processChunks;

    // Диагностика - та же схема, что в cloth-модели (SpringNetwork), см. её .h.
    std::atomic<float> m_lastAvgSpeed{0.0f};
    std::atomic<int> m_lastSubsteps{1};
public:
    float lastAvgSpeed() const { return m_lastAvgSpeed.load(std::memory_order_relaxed); }
    int lastSubsteps() const { return m_lastSubsteps.load(std::memory_order_relaxed); }
};
