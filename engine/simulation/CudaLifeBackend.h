// engine/simulation/CudaLifeBackend.h
#pragma once
#include "engine/simulation/ISimulationBackend.h"
#include <mutex>
#include <atomic>
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
    // atomic, а не обычный bool: simulateDirect() читает этот флаг ДО захвата
    // m_mutex (быстрый выход на уже отключённый interop), а пишется он под
    // локом - обычный bool дал бы неатомарное чтение параллельно с записью,
    // то есть формальный data race (UB, ThreadSanitizer это ловит). Запись
    // по-прежнему только под m_mutex, атомик тут нужен ровно для законности
    // этого одного чтения снаружи лока. Порядок оставлен по умолчанию
    // (seq_cst): флаг проверяется раз на кадр, не в горячем цикле, и
    // выигрывать на нём ослаблением порядка нечего, а повторная проверка под
    // локом ниже всё равно остаётся единственной, по которой принимается
    // решение.
    std::atomic<bool> m_interopFailed{false};   // после первой ошибки больше не пробуем
};
