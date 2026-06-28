// engine/simulation/ISimulationBackend.h
#pragma once
#include <cstdint>
#include "engine/simulation/LifeRule.h"

// Абстракция вычислителя одного шага клеточного автомата над одним чанком.
// Реализации: CpuLifeBackend (эталон на CPU) и CudaLifeBackend (GPU).
// Это точка применения принципа Dependency Inversion: автомат зависит от этого
// интерфейса, а не от конкретного CPU/GPU-класса.
//
// Контракт simulate():
//   ext  — расширенная окрестность чанка, буфер (S+2*border)x(S+2*border),
//          row-major, с границей border=1 (соседи чанка). Центр чанка — со сдвигом 1.
//   extW — ширина ext (== S + 2)
//   out  — выходной буфер SxS (row-major), заполняется новым состоянием чанка
//   S    — размер чанка (сторона)
//   rule — правило в виде таблицы
//
// Реализация ДОЛЖНА быть потокобезопасной: simulate() вызывается параллельно
// из нескольких потоков TaskScheduler для разных чанков.
class ISimulationBackend {
public:
    virtual ~ISimulationBackend() = default;

    virtual void simulate(const uint8_t* ext, int extW,
                          uint8_t* out, int S,
                          const LifeRule& rule) = 0;

    // Человекочитаемое имя бэкенда ("CPU" / "CUDA") — для логов и бенчмарков.
    virtual const char* name() const = 0;
};
