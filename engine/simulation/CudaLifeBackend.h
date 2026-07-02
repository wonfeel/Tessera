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

    void simulateDirect(const uint8_t* ext, int extW,
                        uint8_t* out, int S,
                        const LifeRule& rule,
                        unsigned int glVBO) override;

    // Один kernel launch на весь батч вместо одного на чанк — см. комментарий
    // у ISimulationBackend::preferBatch(). H2D складывает все ext подряд в один
    // device-буфер, kernel индексирует чанк по blockIdx.z, D2H разбирает
    // результат обратно по outs[i]. Отдельные буферы от simulate()/simulateDirect(),
    // чтобы не путать capacity с per-chunk путём (в т.ч. GL-interop).
    bool preferBatch() const override { return true; }
    void simulateBatch(const std::vector<const uint8_t*>& exts, int extW,
                       const std::vector<uint8_t*>& outs, int S,
                       const LifeRule& rule) override;

    bool supportsGLInterop() const override { return true; }
    const char* name() const override { return "CUDA"; }

    static bool isAvailable();

private:
    void ensureBuffers(size_t extBytes, size_t outBytes);
    bool runKernel(const uint8_t* ext, int extW, int S, const LifeRule& rule);
    void ensureBatchBuffers(size_t extBytesTotal, size_t outBytesTotal);

    std::mutex m_mutex;
    void* m_dExt = nullptr;
    void* m_dOut = nullptr;
    size_t m_extCapacity = 0;
    size_t m_outCapacity = 0;

    void* m_dExtBatch = nullptr;
    void* m_dOutBatch = nullptr;
    size_t m_extBatchCapacity = 0;
    size_t m_outBatchCapacity = 0;

    // CUDA Graphics Resource для GL interop (регистрируется лениво по VBO ID).
    // На Windows WDDM cudaGraphicsGLRegisterBuffer требует GL-контекст на
    // вызывающем потоке — если регистрация провалилась, interop отключается.
    void* m_glResource = nullptr;   // cudaGraphicsResource* — храним как void*
    unsigned int m_registeredVBO = 0;
    bool m_interopFailed = false;   // после первой ошибки больше не пробуем
};
