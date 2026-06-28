// engine/simulation/CudaLifeBackend.cu
#include "engine/simulation/CudaLifeBackend.h"
#include <cuda_runtime.h>
#include <cstdio>

// ---------------------------------------------------------------------------
// Правило кладём в constant memory: оно одно на все потоки и читается часто,
// constant memory кэшируется и broadcast'ится на варп — идеально для таблицы.
// 2*9 = 18 байт.
// ---------------------------------------------------------------------------
__constant__ uint8_t c_ruleTable[2][9];

// Ядро: один поток = одна клетка выходного чанка.
//   ext  — расширенная окрестность (extW x extW), border = 1
//   out  — результат (S x S)
__global__ void lifeKernel(const uint8_t* __restrict__ ext, int extW,
                           uint8_t* __restrict__ out, int S) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= S || y >= S) return;

    const int cx = x + 1;   // сдвиг на border
    const int cy = y + 1;

    int alive = 0;
    #pragma unroll
    for (int dy = -1; dy <= 1; ++dy) {
        #pragma unroll
        for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            if (ext[(cy + dy) * extW + (cx + dx)] != 0) ++alive;
        }
    }

    const int center = (ext[cy * extW + cx] != 0) ? 1 : 0;
    out[y * S + x] = c_ruleTable[center][alive];
}

// Маленький помощник: проверка кода возврата CUDA с печатью в stderr.
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
    cudaError_t e = cudaGetDeviceCount(&count);
    return (e == cudaSuccess && count > 0);
}

CudaLifeBackend::CudaLifeBackend() = default;

CudaLifeBackend::~CudaLifeBackend() {
    if (m_dExt) cudaFree(m_dExt);
    if (m_dOut) cudaFree(m_dOut);
}

void CudaLifeBackend::ensureBuffers(size_t extBytes, size_t outBytes) {
    // Буферы переиспользуются между вызовами; реаллокация только при росте размера.
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

void CudaLifeBackend::simulate(const uint8_t* ext, int extW,
                               uint8_t* out, int S,
                               const LifeRule& rule) {
    const size_t extBytes = static_cast<size_t>(extW) * extW * sizeof(uint8_t);
    const size_t outBytes = static_cast<size_t>(S) * S * sizeof(uint8_t);

    // Device-буферы общие для всех чанков, поэтому весь вызов под мьютексом.
    // Тяжёлая работа всё равно на GPU; чанки сериализуются, но каждый — это
    // десятки тысяч параллельных потоков внутри ядра.
    std::lock_guard<std::mutex> lock(m_mutex);

    ensureBuffers(extBytes, outBytes);
    if (!m_dExt || !m_dOut) return;   // аллокация не удалась — тихо выходим

    if (!cudaOk(cudaMemcpyToSymbol(c_ruleTable, rule.table, sizeof(rule.table)),
                "cudaMemcpyToSymbol(rule)")) return;
    if (!cudaOk(cudaMemcpy(m_dExt, ext, extBytes, cudaMemcpyHostToDevice),
                "cudaMemcpy(H2D ext)")) return;

    dim3 block(16, 16);
    dim3 grid((S + block.x - 1) / block.x,
              (S + block.y - 1) / block.y);
    lifeKernel<<<grid, block>>>(static_cast<const uint8_t*>(m_dExt), extW,
                                static_cast<uint8_t*>(m_dOut), S);
    if (!cudaOk(cudaGetLastError(), "lifeKernel launch")) return;

    if (!cudaOk(cudaMemcpy(out, m_dOut, outBytes, cudaMemcpyDeviceToHost),
                "cudaMemcpy(D2H out)")) return;
}
