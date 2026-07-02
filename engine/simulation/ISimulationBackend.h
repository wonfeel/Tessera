// engine/simulation/ISimulationBackend.h
#pragma once
#include <cstdint>
#include <vector>
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

    // Версия с CUDA-OpenGL interop: результат пишется параллельно в out (CPU)
    // и напрямую в GL VBO (glVBO) через D2D-копию, минуя шину PCI-E.
    // Базовая реализация — просто вызывает simulate() без interop.
    // glVBO == 0 означает "interop не нужен", поведение идентично simulate().
    virtual void simulateDirect(const uint8_t* ext, int extW,
                                uint8_t* out, int S,
                                const LifeRule& rule,
                                unsigned int glVBO) {
        simulate(ext, extW, out, S, rule);
    }

    // true если бэкенд поддерживает CUDA-GL interop (только CudaLifeBackend).
    virtual bool supportsGLInterop() const { return false; }

    // true если бэкенду выгоднее считать много чанков одним вызовом
    // simulateBatch(), а не по одному через simulate(). На GPU каждый вызов
    // simulate() — это H2D + kernel launch + D2H под мьютексом; при десятках-
    // сотнях мелких активных чанков (обычный chunkSize=64) overhead запуска
    // ядра и синхронных копий доминирует над самим счётом. CPU-бэкенду батчинг
    // не нужен — параллелизм там уже даёт TaskScheduler, гоняя simulate()
    // одновременно из разных потоков.
    virtual bool preferBatch() const { return false; }

    // Считает N независимых чанков одним вызовом (для GPU — один kernel launch
    // на все N вместо N отдельных). exts[i]/outs[i] — те же буферы, что и в
    // simulate(), просто по одному на чанк; все extW одинаковые (общий chunkSize).
    // Реализация по умолчанию — просто цикл по simulate(), корректна для любого
    // бэкенда и служит safety-net, если конкретная реализация не переопределила.
    virtual void simulateBatch(const std::vector<const uint8_t*>& exts, int extW,
                               const std::vector<uint8_t*>& outs, int S,
                               const LifeRule& rule) {
        for (size_t i = 0; i < exts.size(); ++i)
            simulate(exts[i], extW, outs[i], S, rule);
    }

    // Человекочитаемое имя бэкенда ("CPU" / "CUDA") — для логов и бенчмарков.
    virtual const char* name() const = 0;
};
