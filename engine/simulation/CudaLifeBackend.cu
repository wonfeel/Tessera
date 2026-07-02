// engine/simulation/CudaLifeBackend.cu
// cuda_gl_interop.h требует GL-типы (GLuint/GLenum) — glad даёт их.
#include <glad/glad.h>
#include "engine/simulation/CudaLifeBackend.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <cstdio>

// Правило в constant memory: broadcast на варп бесплатный, идеально для lookup.
__constant__ uint8_t c_ruleTable[2][9];

// Размеры блока — одно место, всё завязано на них.
static constexpr int BW = 16;
static constexpr int BH = 16;
static constexpr int TW = BW + 2;
static constexpr int TH = BH + 2;

// ---------------------------------------------------------------------------
// Ядро с shared memory тайлингом.
// Каждый блок BW×BH грузит тайл TW×TH из ext в shared memory — один раз
// на блок вместо 9-10 обращений в global на поток.
// ---------------------------------------------------------------------------
__global__ void lifeKernelShared(const uint8_t* __restrict__ ext, int extW,
                                  uint8_t* __restrict__ out, int S) {
    __shared__ uint8_t tile[TH][TW];

    const int tx = threadIdx.x;
    const int ty = threadIdx.y;

    const int tileRow0 = blockIdx.y * BH;
    const int tileCol0 = blockIdx.x * BW;

    const int tid      = ty * BW + tx;
    const int tileSize = TH * TW;

    for (int i = tid; i < tileSize; i += BW * BH) {
        const int tr = i / TW;
        const int tc = i % TW;
        const int er = tileRow0 + tr;
        const int ec = tileCol0 + tc;
        tile[tr][tc] = (er < extW && ec < extW) ? ext[er * extW + ec] : 0;
    }

    __syncthreads();

    const int x = blockIdx.x * BW + tx;
    const int y = blockIdx.y * BH + ty;
    if (x >= S || y >= S) return;

    int alive = 0;
    #pragma unroll
    for (int dy = -1; dy <= 1; ++dy) {
        #pragma unroll
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (tile[ty + dy + 1][tx + dx + 1] != 0) ++alive;
        }
    }

    const int center = (tile[ty + 1][tx + 1] != 0) ? 1 : 0;
    out[y * S + x] = c_ruleTable[center][alive];
}

// ---------------------------------------------------------------------------
// Батчевая версия того же ядра: доп. измерение grid.z выбирает чанк внутри
// одного launch'а. Тело идентично lifeKernelShared, просто ext/out со сдвигом
// на chunkIdx * extW*extW / chunkIdx * S*S — все чанки в батче одного размера.
// ---------------------------------------------------------------------------
__global__ void lifeKernelSharedBatched(const uint8_t* __restrict__ extBatch, int extW,
                                        uint8_t* __restrict__ outBatch, int S) {
    __shared__ uint8_t tile[TH][TW];

    const int chunkIdx = blockIdx.z;
    const uint8_t* ext = extBatch + static_cast<size_t>(chunkIdx) * extW * extW;
    uint8_t* out = outBatch + static_cast<size_t>(chunkIdx) * S * S;

    const int tx = threadIdx.x;
    const int ty = threadIdx.y;

    const int tileRow0 = blockIdx.y * BH;
    const int tileCol0 = blockIdx.x * BW;

    const int tid      = ty * BW + tx;
    const int tileSize = TH * TW;

    for (int i = tid; i < tileSize; i += BW * BH) {
        const int tr = i / TW;
        const int tc = i % TW;
        const int er = tileRow0 + tr;
        const int ec = tileCol0 + tc;
        tile[tr][tc] = (er < extW && ec < extW) ? ext[er * extW + ec] : 0;
    }

    __syncthreads();

    const int x = blockIdx.x * BW + tx;
    const int y = blockIdx.y * BH + ty;
    if (x >= S || y >= S) return;

    int alive = 0;
    #pragma unroll
    for (int dy = -1; dy <= 1; ++dy) {
        #pragma unroll
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (tile[ty + dy + 1][tx + dx + 1] != 0) ++alive;
        }
    }

    const int center = (tile[ty + 1][tx + 1] != 0) ? 1 : 0;
    out[y * S + x] = c_ruleTable[center][alive];
}

static bool cudaOk(cudaError_t e, const char* what) {
    if (e != cudaSuccess) {
        std::fprintf(stderr, "[CudaLifeBackend] %s failed: %s\n",
                     what, cudaGetErrorString(e));
        return false;
    }
    return true;
}

bool CudaLifeBackend::isAvailable() {
    int count = 0;
    return (cudaGetDeviceCount(&count) == cudaSuccess && count > 0);
}

CudaLifeBackend::CudaLifeBackend() = default;

CudaLifeBackend::~CudaLifeBackend() {
    if (m_glResource) {
        cudaGraphicsUnregisterResource(
            static_cast<cudaGraphicsResource_t>(m_glResource));
    }
    if (m_dExt) cudaFree(m_dExt);
    if (m_dOut) cudaFree(m_dOut);
    if (m_dExtBatch) cudaFree(m_dExtBatch);
    if (m_dOutBatch) cudaFree(m_dOutBatch);
}

void CudaLifeBackend::ensureBatchBuffers(size_t extBytesTotal, size_t outBytesTotal) {
    if (extBytesTotal > m_extBatchCapacity) {
        if (m_dExtBatch) cudaFree(m_dExtBatch);
        m_dExtBatch = nullptr;
        if (cudaOk(cudaMalloc(&m_dExtBatch, extBytesTotal), "cudaMalloc(extBatch)"))
            m_extBatchCapacity = extBytesTotal;
        else
            m_extBatchCapacity = 0;
    }
    if (outBytesTotal > m_outBatchCapacity) {
        if (m_dOutBatch) cudaFree(m_dOutBatch);
        m_dOutBatch = nullptr;
        if (cudaOk(cudaMalloc(&m_dOutBatch, outBytesTotal), "cudaMalloc(outBatch)"))
            m_outBatchCapacity = outBytesTotal;
        else
            m_outBatchCapacity = 0;
    }
}

void CudaLifeBackend::simulateBatch(const std::vector<const uint8_t*>& exts, int extW,
                                    const std::vector<uint8_t*>& outs, int S,
                                    const LifeRule& rule) {
    const size_t n = exts.size();
    if (n == 0) return;
    if (n == 1) { simulate(exts[0], extW, outs[0], S, rule); return; }

    const size_t extBytes = static_cast<size_t>(extW) * extW;
    const size_t outBytes = static_cast<size_t>(S) * S;

    std::lock_guard<std::mutex> lock(m_mutex);

    ensureBatchBuffers(extBytes * n, outBytes * n);
    if (!m_dExtBatch || !m_dOutBatch) {
        // Батч не влез в память — считаем как раньше, по одному чанку.
        for (size_t i = 0; i < n; ++i) simulate(exts[i], extW, outs[i], S, rule);
        return;
    }

    if (!cudaOk(cudaMemcpyToSymbol(c_ruleTable, rule.table, sizeof(rule.table)),
                "cudaMemcpyToSymbol(rule)")) return;

    // H2D по одному на чанк — exts[i] не смежны в host-памяти (отдельные
    // std::vector), склеивать их в один host-буфер перед копированием того же
    // порядка сложности, что и просто скопировать N раз, зато без лишней
    // аллокации. Выигрыш батчинга — не в этих копиях, а в том, что kernel
    // launch и синхронизация теперь одни на весь батч, а не на каждый чанк.
    uint8_t* dExt = static_cast<uint8_t*>(m_dExtBatch);
    for (size_t i = 0; i < n; ++i) {
        if (!cudaOk(cudaMemcpy(dExt + i * extBytes, exts[i], extBytes, cudaMemcpyHostToDevice),
                    "cudaMemcpy(H2D ext batch)")) return;
    }

    dim3 block(BW, BH);
    dim3 grid((S + BW - 1) / BW, (S + BH - 1) / BH, static_cast<unsigned int>(n));
    lifeKernelSharedBatched<<<grid, block>>>(dExt, extW,
                                             static_cast<uint8_t*>(m_dOutBatch), S);
    if (!cudaOk(cudaGetLastError(), "lifeKernelSharedBatched launch")) return;

    uint8_t* dOut = static_cast<uint8_t*>(m_dOutBatch);
    for (size_t i = 0; i < n; ++i) {
        cudaOk(cudaMemcpy(outs[i], dOut + i * outBytes, outBytes, cudaMemcpyDeviceToHost),
               "cudaMemcpy(D2H out batch)");
    }
}

void CudaLifeBackend::ensureBuffers(size_t extBytes, size_t outBytes) {
    if (extBytes > m_extCapacity) {
        if (m_dExt) cudaFree(m_dExt);
        m_dExt = nullptr;
        if (cudaOk(cudaMalloc(&m_dExt, extBytes), "cudaMalloc(ext)"))
            m_extCapacity = extBytes;
        else
            m_extCapacity = 0;
    }
    if (outBytes > m_outCapacity) {
        if (m_dOut) cudaFree(m_dOut);
        m_dOut = nullptr;
        if (cudaOk(cudaMalloc(&m_dOut, outBytes), "cudaMalloc(out)"))
            m_outCapacity = outBytes;
        else
            m_outCapacity = 0;
    }
}

// Общая часть: H2D ext → ядро → результат в m_dOut.
// Вызывается под m_mutex. Возвращает false при ошибке.
bool CudaLifeBackend::runKernel(const uint8_t* ext, int extW,
                                 int S, const LifeRule& rule) {
    const size_t extBytes = static_cast<size_t>(extW) * extW;
    const size_t outBytes = static_cast<size_t>(S) * S;

    ensureBuffers(extBytes, outBytes);
    if (!m_dExt || !m_dOut) return false;

    if (!cudaOk(cudaMemcpyToSymbol(c_ruleTable, rule.table, sizeof(rule.table)),
                "cudaMemcpyToSymbol(rule)")) return false;
    if (!cudaOk(cudaMemcpy(m_dExt, ext, extBytes, cudaMemcpyHostToDevice),
                "cudaMemcpy(H2D ext)")) return false;

    dim3 block(BW, BH);
    dim3 grid((S + BW - 1) / BW, (S + BH - 1) / BH);
    lifeKernelShared<<<grid, block>>>(static_cast<const uint8_t*>(m_dExt),
                                      extW,
                                      static_cast<uint8_t*>(m_dOut), S);
    return cudaOk(cudaGetLastError(), "lifeKernelShared launch");
}

void CudaLifeBackend::simulate(const uint8_t* ext, int extW,
                               uint8_t* out, int S,
                               const LifeRule& rule) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!runKernel(ext, extW, S, rule)) return;
    cudaOk(cudaMemcpy(out, m_dOut, static_cast<size_t>(S) * S,
                      cudaMemcpyDeviceToHost), "cudaMemcpy(D2H out)");
}

void CudaLifeBackend::simulateDirect(const uint8_t* ext, int extW,
                                     uint8_t* out, int S,
                                     const LifeRule& rule,
                                     unsigned int glVBO) {
    if (glVBO == 0 || m_interopFailed) {
        simulate(ext, extW, out, S, rule);
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    // Повторная проверка под локом: пока этот поток ждал мьютекса, другой
    // поток мог уже установить m_interopFailed = true.
    if (m_interopFailed) {
        if (runKernel(ext, extW, S, rule))
            cudaOk(cudaMemcpy(out, m_dOut, static_cast<size_t>(S) * S,
                              cudaMemcpyDeviceToHost), "cudaMemcpy(D2H out)");
        return;
    }

    // Ленивая регистрация GL буфера. Если VBO сменился — перерегистрируем.
    if (m_registeredVBO != glVBO) {
        if (m_glResource)
            cudaGraphicsUnregisterResource(
                static_cast<cudaGraphicsResource_t>(m_glResource));
        m_glResource    = nullptr;
        m_registeredVBO = 0;

        // Не вызываем cudaSetDevice здесь: на WDDM с рабочими потоками он сбрасывает
        // контекст текущего потока, после чего ранее аллоцированные m_dExt/m_dOut
        // становятся невалидными и последующий runKernel падает с "OS call failed".
        // На однопроцессорной системе (RTX 3060 Ti) device 0 всегда единственный.
        cudaGraphicsResource_t res = nullptr;
        cudaError_t regErr = cudaGraphicsGLRegisterBuffer(
            &res, glVBO, cudaGraphicsRegisterFlagsWriteDiscard);
        if (regErr == cudaSuccess) {
            m_glResource    = res;
            m_registeredVBO = glVBO;
        } else {
            // cudaGraphicsGLRegisterBuffer требует GL-контекст на вызывающем потоке.
            // На Windows WDDM с рабочими потоками это не работает — отключаем навсегда.
            std::fprintf(stderr,
                "[CudaLifeBackend] GL interop unavailable (%s) — using CPU→GPU path\n",
                cudaGetErrorString(regErr));
            // Провальный вызов cudaGraphicsGLRegisterBuffer оставляет "sticky error"
            // в контексте CUDA: все последующие вызовы тоже вернут ту же ошибку,
            // пока мы не сбросим её через cudaGetLastError().
            cudaGetLastError();
            m_interopFailed = true;
            // ВНИМАНИЕ: мы держим m_mutex, поэтому НЕ вызываем simulate()
            // (она лочит тот же нерекурсивный мьютекс → дедлок). Делаем kernel
            // и D2H напрямую — runKernel рассчитан на вызов под локом.
            if (runKernel(ext, extW, S, rule))
                cudaOk(cudaMemcpy(out, m_dOut, static_cast<size_t>(S) * S,
                                  cudaMemcpyDeviceToHost), "cudaMemcpy(D2H out)");
            return;
        }
    }

    if (!runKernel(ext, extW, S, rule)) return;

    const size_t outBytes = static_cast<size_t>(S) * S;

    // Параллельно: D2H для renderBuffer (нужен border exchange) +
    //              D2D → GL VBO (минуем PCI-E на upload).
    cudaOk(cudaMemcpy(out, m_dOut, outBytes, cudaMemcpyDeviceToHost),
           "cudaMemcpy(D2H out)");

    // Map GL VBO как CUDA-указатель и скопируем D2D (GPU → GPU, без CPU).
    auto res = static_cast<cudaGraphicsResource_t>(m_glResource);
    if (!cudaOk(cudaGraphicsMapResources(1, &res), "cudaGraphicsMapResources"))
        return;

    void*  glPtr  = nullptr;
    size_t glSize = 0;
    if (cudaOk(cudaGraphicsResourceGetMappedPointer(&glPtr, &glSize, res),
               "cudaGraphicsResourceGetMappedPointer")) {
        if (glSize >= outBytes)
            cudaOk(cudaMemcpy(glPtr, m_dOut, outBytes, cudaMemcpyDeviceToDevice),
                   "cudaMemcpy(D2D → GL VBO)");
    }

    cudaOk(cudaGraphicsUnmapResources(1, &res), "cudaGraphicsUnmapResources");
}
