// engine/simulation/LightWaveCuda.cu
#include "engine/simulation/LightWaveCuda.h"
#include <cuda_runtime.h>
#include <cstdio>
#include <algorithm>

namespace {

// odd-r offset - те же таблицы, что и engine/core/HexGrid.h (kOffsetsEven/kOffsetsOdd).
__constant__ int c_offEven[6][2] = {{1,0}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}, {0,1}};
__constant__ int c_offOdd[6][2]  = {{1,0}, {1,-1}, {0,-1}, {-1,0}, {0,1}, {1,1}};

static constexpr int BW = 16;
static constexpr int BH = 16;

// Копия граничных констант из LightField.cpp (kAbsorbLayerCells/kAbsorbExtraDamping) -
// держим синхронно с CPU-версией, иначе сравнение нечестное.
static constexpr int   kAbsorbLayerCells = 40;
static constexpr float kAbsorbExtraDamping = 400.0f;
static constexpr float kMaxDisplacementPerSubstep = 2.0f;

__global__ void hexForceKernel(const float* __restrict__ height,
                                const float* __restrict__ mediumMask,
                                float* __restrict__ force,
                                int cols, int rows,
                                float waveSpeedSq, float dispersion) {
    int col = blockIdx.x * BW + threadIdx.x;
    int row = blockIdx.y * BH + threadIdx.y;
    if (col >= cols || row >= rows) return;
    int i = row * cols + col;

    bool pinned = (col == 0 || col == cols - 1 || row == 0 || row == rows - 1);
    if (pinned) { force[i] = 0.0f; return; }

    const int (*offs)[2] = (row & 1) ? c_offOdd : c_offEven;
    float c0 = height[i];
    float sum = 0.0f;
    #pragma unroll
    for (int k = 0; k < 6; ++k) {
        int nc = col + offs[k][0], nr = row + offs[k][1];
        sum += height[nr * cols + nc];
    }
    float lap = (sum - 6.0f * c0) / 6.0f;
    float medium = mediumMask[i];
    float speedSq = fmaxf(0.0f, waveSpeedSq * (1.0f - medium * dispersion));
    force[i] = speedSq * lap;
}

__global__ void integrateKernel(float* __restrict__ height,
                                 float* __restrict__ velocity,
                                 const float* __restrict__ force,
                                 int cols, int rows,
                                 float dt, float dampingRate, float maxSpeed) {
    int col = blockIdx.x * BW + threadIdx.x;
    int row = blockIdx.y * BH + threadIdx.y;
    if (col >= cols || row >= rows) return;
    int i = row * cols + col;

    bool pinned = (col == 0 || col == cols - 1 || row == 0 || row == rows - 1);
    if (pinned) return;

    float v = velocity[i] + force[i] * dt;

    int distToEdge = min(min(col, cols - 1 - col), min(row, rows - 1 - row));
    float damp;
    if (distToEdge < kAbsorbLayerCells) {
        float t = 1.0f - static_cast<float>(distToEdge) / static_cast<float>(kAbsorbLayerCells);
        damp = expf(-(dampingRate + t * t * kAbsorbExtraDamping) * dt);
    } else {
        damp = expf(-dampingRate * dt);
    }
    v *= damp;
    if (fabsf(v) > maxSpeed) v = (v > 0.0f) ? maxSpeed : -maxSpeed;

    float h = height[i] + v * dt;
    if (!isfinite(h) || !isfinite(v)) { h = 0.0f; v = 0.0f; }

    velocity[i] = v;
    height[i] = h;
}

// Копии констант яркости из LightField.cpp - должны совпадать, иначе
// сравнение CPU/GPU считает разную работу.
static constexpr float kGlowDecay = 0.90f;
static constexpr float kLogEps = 1e-4f;
static constexpr float kMinAvgSpeed = 0.05f;
static constexpr float kAccumRate = 0.15f;

// speed = max(|v|,|h|) + блочная редукция sum(log(speed+eps)). Блочный
// результат добавляем одним atomicAdd на блок (625 атомиков на 160к узлов -
// контенции нет), чтобы не гонять отдельный проход редукции.
__global__ void speedAndSumLogKernel(const float* __restrict__ height,
                                      const float* __restrict__ velocity,
                                      float* __restrict__ speedBuf,
                                      float* __restrict__ sumLogOut,
                                      int n) {
    extern __shared__ float sdata[];
    int tid = threadIdx.x;
    int i = blockIdx.x * blockDim.x + tid;

    float local = 0.0f;
    if (i < n) {
        float speed = fmaxf(fabsf(velocity[i]), fabsf(height[i]));
        speedBuf[i] = speed;
        local = logf(speed + kLogEps);
    }
    sdata[tid] = local;
    __syncthreads();

    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) sdata[tid] += sdata[tid + s];
        __syncthreads();
    }
    if (tid == 0) atomicAdd(sumLogOut, sdata[0]);
}

__global__ void glowAccumKernel(const float* __restrict__ speedBuf,
                                 const float* __restrict__ sumLog,
                                 float* __restrict__ glow,
                                 float* __restrict__ accum,
                                 int n, float spacing, float accumAlpha) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float avgSpeed = fmaxf(__expf(sumLog[0] / static_cast<float>(n)), kMinAvgSpeed * spacing);
    float s = speedBuf[i];
    float normLinear = s / (s + avgSpeed);
    float norm = normLinear * normLinear * normLinear;
    float g = fmaxf(norm, glow[i] * kGlowDecay);
    glow[i] = g;
    accum[i] += (g - accum[i]) * accumAlpha;
}

bool cudaOk(cudaError_t e, const char* what) {
    if (e != cudaSuccess) {
        std::fprintf(stderr, "[LightWaveCuda] %s failed: %s\n", what, cudaGetErrorString(e));
        return false;
    }
    return true;
}

}   // namespace

bool LightWaveCuda::isAvailable() {
    int count = 0;
    return (cudaGetDeviceCount(&count) == cudaSuccess && count > 0);
}

LightWaveCuda::LightWaveCuda() = default;

void LightWaveCuda::freeBuffers() {
    if (m_dHeight) cudaFree(m_dHeight);
    if (m_dVelocity) cudaFree(m_dVelocity);
    if (m_dForce) cudaFree(m_dForce);
    if (m_dMediumMask) cudaFree(m_dMediumMask);
    if (m_dGlow) cudaFree(m_dGlow);
    if (m_dAccum) cudaFree(m_dAccum);
    if (m_dSpeedBuf) cudaFree(m_dSpeedBuf);
    if (m_dSumLog) cudaFree(m_dSumLog);
    m_dHeight = m_dVelocity = m_dForce = m_dMediumMask = nullptr;
    m_dGlow = m_dAccum = m_dSpeedBuf = m_dSumLog = nullptr;
    m_bytesCapacity = 0;
}

LightWaveCuda::~LightWaveCuda() {
    freeBuffers();
}

bool LightWaveCuda::upload(int cols, int rows, float spacing,
                            const float* height, const float* velocity, const float* mediumMask) {
    m_cols = cols; m_rows = rows; m_spacing = spacing;
    size_t bytes = static_cast<size_t>(cols) * rows * sizeof(float);

    // Аллоцируем только если буферов ещё нет или поле изменило размер. Без
    // этой проверки повторный upload() (смена размера поля, reset, второй
    // прогон в бенчмарке) заново звал восемь cudaMalloc поверх старых
    // указателей, теряя всю ранее выделенную device-память.
    if (bytes != m_bytesCapacity) {
        freeBuffers();
        if (!cudaOk(cudaMalloc(&m_dHeight, bytes), "cudaMalloc(height)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dVelocity, bytes), "cudaMalloc(velocity)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dForce, bytes), "cudaMalloc(force)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dMediumMask, bytes), "cudaMalloc(mediumMask)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dGlow, bytes), "cudaMalloc(glow)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dAccum, bytes), "cudaMalloc(accum)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dSpeedBuf, bytes), "cudaMalloc(speedBuf)")) { freeBuffers(); return false; }
        if (!cudaOk(cudaMalloc(&m_dSumLog, sizeof(float)), "cudaMalloc(sumLog)")) { freeBuffers(); return false; }
        m_bytesCapacity = bytes;
    }

    if (!cudaOk(cudaMemcpy(m_dHeight, height, bytes, cudaMemcpyHostToDevice), "H2D height")) return false;
    if (!cudaOk(cudaMemcpy(m_dVelocity, velocity, bytes, cudaMemcpyHostToDevice), "H2D velocity")) return false;
    if (!cudaOk(cudaMemcpy(m_dMediumMask, mediumMask, bytes, cudaMemcpyHostToDevice), "H2D mediumMask")) return false;
    if (!cudaOk(cudaMemset(m_dGlow, 0, bytes), "memset(glow)")) return false;
    if (!cudaOk(cudaMemset(m_dAccum, 0, bytes), "memset(accum)")) return false;
    return true;
}

void LightWaveCuda::step(float dt, float waveSpeedSq, float dampingRate, float dispersion) {
    dim3 block(BW, BH);
    dim3 grid((m_cols + BW - 1) / BW, (m_rows + BH - 1) / BH);

    hexForceKernel<<<grid, block>>>(
        static_cast<const float*>(m_dHeight), static_cast<const float*>(m_dMediumMask),
        static_cast<float*>(m_dForce), m_cols, m_rows, waveSpeedSq, dispersion);
    if (!cudaOk(cudaGetLastError(), "hexForceKernel launch")) return;

    float maxSpeed = kMaxDisplacementPerSubstep * m_spacing / dt;

    integrateKernel<<<grid, block>>>(
        static_cast<float*>(m_dHeight), static_cast<float*>(m_dVelocity),
        static_cast<const float*>(m_dForce), m_cols, m_rows, dt, dampingRate, maxSpeed);
    if (!cudaOk(cudaGetLastError(), "integrateKernel launch")) return;

    // Пасс яркости - то же, что делает CPU-версия в конце step().
    const int n = m_cols * m_rows;
    constexpr int kThreads = 256;
    int blocks = (n + kThreads - 1) / kThreads;

    if (!cudaOk(cudaMemsetAsync(m_dSumLog, 0, sizeof(float)), "memset(sumLog)")) return;
    speedAndSumLogKernel<<<blocks, kThreads, kThreads * sizeof(float)>>>(
        static_cast<const float*>(m_dHeight), static_cast<const float*>(m_dVelocity),
        static_cast<float*>(m_dSpeedBuf), static_cast<float*>(m_dSumLog), n);
    if (!cudaOk(cudaGetLastError(), "speedAndSumLogKernel launch")) return;

    float accumAlpha = 1.0f - expf(-kAccumRate * dt);
    glowAccumKernel<<<blocks, kThreads>>>(
        static_cast<const float*>(m_dSpeedBuf), static_cast<const float*>(m_dSumLog),
        static_cast<float*>(m_dGlow), static_cast<float*>(m_dAccum),
        n, m_spacing, accumAlpha);
    cudaOk(cudaGetLastError(), "glowAccumKernel launch");
}

void LightWaveCuda::downloadHeight(float* outHeight) const {
    size_t bytes = static_cast<size_t>(m_cols) * m_rows * sizeof(float);
    cudaOk(cudaMemcpy(outHeight, m_dHeight, bytes, cudaMemcpyDeviceToHost), "D2H height");
}

void LightWaveCuda::downloadGlow(float* outGlow) const {
    size_t bytes = static_cast<size_t>(m_cols) * m_rows * sizeof(float);
    cudaOk(cudaMemcpy(outGlow, m_dGlow, bytes, cudaMemcpyDeviceToHost), "D2H glow");
}
