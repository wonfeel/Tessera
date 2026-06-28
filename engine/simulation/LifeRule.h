// engine/simulation/LifeRule.h
#pragma once
#include <cstdint>

// Описание тотализирующего ("life-like") правила клеточного автомата в виде данных,
// а не кода. Это нужно, чтобы одно и то же правило могли исполнять и CPU-, и
// CUDA-бэкенд: виртуальную функцию transition() с GPU вызвать нельзя, а таблицу —
// можно (она крошечная и кладётся в constant memory).
//
// table[centerAlive][aliveNeighbors] -> новое состояние клетки.
//   centerAlive    : 0 — центральная клетка мертва (==0), 1 — жива (!=0)
//   aliveNeighbors : число живых соседей из 8, диапазон 0..8
//
// "Жива" = любое ненулевое значение тайла.
struct LifeRule {
    uint8_t table[2][9];
};

// Классическая Conway "Жизнь" (B3/S23), результат — 255 для живых.
inline LifeRule MakeConwayRule() {
    LifeRule r{};
    // мёртвая клетка: рождается при ровно 3 соседях
    r.table[0][3] = 255;
    // живая клетка: выживает при 2 или 3 соседях
    r.table[1][2] = 255;
    r.table[1][3] = 255;
    return r;
}
