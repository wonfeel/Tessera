// engine/simulation/CudaLifeBackend.h
#pragma once
#include "engine/simulation/ISimulationBackend.h"
#include <mutex>
#include <cstddef>

// GPU-реализация шага автомата на CUDA.
//
// Важно: этот заголовок включается и обычным C++-компилятором (cl/g++),
// поэтому НИ ОДНОГО CUDA-типа здесь быть не должно. Указатели на память
// устройства держим как void*, чтобы заголовок оставался "чистым" C++.
//
// Реализация (.cu) кэширует device-буферы между вызовами: размер чанка
// постоянен, поэтому аллокация происходит один раз. Доступ к кэшу
// сериализуется мьютексом — simulate() вызывается из нескольких потоков.
class CudaLifeBackend : public ISimulationBackend {
public:
    CudaLifeBackend();
    ~CudaLifeBackend() override;

    void simulate(const uint8_t* ext, int extW,
                  uint8_t* out, int S,
                  const LifeRule& rule) override;

    const char* name() const override { return "CUDA"; }

    // true, если на машине есть видимое CUDA-устройство. Используется фабрикой
    // для выбора бэкенда. Безопасно вызывать всегда.
    static bool isAvailable();

private:
    void ensureBuffers(size_t extBytes, size_t outBytes);

    std::mutex m_mutex;          // сериализует доступ к device-буферам
    void* m_dExt = nullptr;      // device: расширенная окрестность
    void* m_dOut = nullptr;      // device: результат
    size_t m_extCapacity = 0;
    size_t m_outCapacity = 0;
};
