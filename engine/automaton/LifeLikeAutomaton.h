// engine/automaton/LifeLikeAutomaton.h
#pragma once
#include "engine/chunk/ChunkedTileMap.h"
#include "engine/simulation/ISimulationBackend.h"
#include "engine/simulation/LifeRule.h"
#include <memory>

// Клеточный автомат "life-like" (тотализирующее правило: исход зависит от состояния
// клетки и числа живых соседей). В отличие от CellularAutomaton, правило задаётся
// таблицей-данными (LifeRule), а шаг считается через подменяемый бэкенд
// (CPU или CUDA) — см. ISimulationBackend.
//
// Это и есть точка, где живёт CPU/GPU-форк: какой именно бэкенд создан, решает
// фабрика MakeSimulationBackend() в зависимости от того, собран ли проект с CUDA
// и есть ли GPU.
class LifeLikeAutomaton : public ChunkedTileMap {
public:
    LifeLikeAutomaton(int totalWidth, int totalHeight, int chunkSize,
                      float tileSize, const LifeRule& rule);

    // Имя активного бэкенда ("CPU"/"CUDA") — удобно для заголовка окна/бенчмарка.
    const char* backendName() const { return m_backend->name(); }

protected:
    void simulateChunk(Chunk& chunk) override;

    LifeRule m_rule;
    std::unique_ptr<ISimulationBackend> m_backend;
};
