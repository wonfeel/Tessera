// engine/simulation/CudaSpringBackend.cu
#include "engine/simulation/CudaSpringBackend.h"
#include <cuda_runtime.h>
#include <cub/cub.cuh>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace {
    constexpr int kBlockSize = 128;

    static bool cudaOk(cudaError_t e, const char* what) {
        if (e != cudaSuccess) {
            std::fprintf(stderr, "[CudaSpringBackend] %s failed: %s\n",
                         what, cudaGetErrorString(e));
            return false;
        }
        return true;
    }

    // ---- device-типы (POD, layout совпадает с CudaSpringBackend::EdgeGpu) ----
    struct EdgeD { int a, b; float restLen; };

    __global__ void zeroForceKernel(float2* force, int n) {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i < n) force[i] = make_float2(0.0f, 0.0f);
    }

    // Один блок на элемент processChunks[blockIdx.x]; grid-stride по CSR-
    // диапазону чанка. atomicAdd — см. комментарий класса в .h: GPU не делит
    // рёбра на internal/boundary, оба конца могут принадлежать РАЗНЫМ
    // обрабатываемым чанкам, atomicAdd делает совместную запись безопасной.
    __global__ void springForceKernel(const int* __restrict__ processChunks,
                                       const EdgeD* __restrict__ edges,
                                       const uint32_t* __restrict__ offset,
                                       const float2* __restrict__ pos,
                                       float2* force, float stiffness) {
        int chunkId = processChunks[blockIdx.x];
        uint32_t begin = offset[chunkId];
        uint32_t end = offset[chunkId + 1];
        for (uint32_t e = begin + threadIdx.x; e < end; e += blockDim.x) {
            EdgeD edge = edges[e];
            float2 pa = pos[edge.a];
            float2 pb = pos[edge.b];
            float dx = pb.x - pa.x, dy = pb.y - pa.y;
            float dist = sqrtf(dx * dx + dy * dy);
            float dirx, diry;
            if (dist > 1e-5f) {
                float inv = 1.0f / dist;
                dirx = dx * inv; diry = dy * inv;
            } else {
                // Тот же вырожденный случай, что и CPU applyEdge(): узлы
                // схлопнулись — подставляем произвольное направление вместо
                // пропуска силы (растяжение относительно restLen тут
                // максимально, пропускать нельзя).
                dirx = 0.0f; diry = -1.0f;
            }
            float mag = stiffness * (dist - edge.restLen);
            float fx = dirx * mag, fy = diry * mag;
            atomicAdd(&force[edge.a].x, fx);
            atomicAdd(&force[edge.a].y, fy);
            atomicAdd(&force[edge.b].x, -fx);
            atomicAdd(&force[edge.b].y, -fy);
        }
    }

    __global__ void integrateKernel(float2* pos, float2* vel, const float2* force,
                                     const uint8_t* pinned, const float2* restPos, int n,
                                     float subDt, float dampFactor, float maxSpeed) {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= n) return;
        if (pinned[i]) return;   // ведёт drag на хосте — физика не трогает (см. SpringNetwork)

        float2 v = vel[i];
        v.x = (v.x + force[i].x * subDt) * dampFactor;
        v.y = (v.y + force[i].y * subDt) * dampFactor;

        float speed = sqrtf(v.x * v.x + v.y * v.y);
        if (speed > maxSpeed && speed > 0.0f) {
            float scale = maxSpeed / speed;
            v.x *= scale; v.y *= scale;
        }

        float2 p = pos[i];
        p.x += v.x * subDt;
        p.y += v.y * subDt;

        if (!isfinite(p.x) || !isfinite(p.y)) {
            p = restPos[i];
            v = make_float2(0.0f, 0.0f);
        }

        vel[i] = v;
        pos[i] = p;
    }

    __global__ void speedLogKernel(const float2* vel, float* speedBuf, float* logBuf,
                                    int n, float eps) {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= n) return;
        float s = sqrtf(vel[i].x * vel[i].x + vel[i].y * vel[i].y);
        speedBuf[i] = s;
        logBuf[i] = logf(s + eps);
    }

    __global__ void nodeGlowKernel(const float* speedBuf, float* nodeGlow, int n,
                                    float avgSpeed, float contrast, float decay) {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= n) return;
        float norm = speedBuf[i] / (speedBuf[i] + avgSpeed);
        norm = powf(norm, contrast);
        nodeGlow[i] = fmaxf(norm, nodeGlow[i] * decay);
    }

    __global__ void stretchLogKernel(const float2* pos, const EdgeD* edges,
                                      float* stretchBuf, float* logBuf, int m, float eps) {
        int e = blockIdx.x * blockDim.x + threadIdx.x;
        if (e >= m) return;
        EdgeD edge = edges[e];
        float2 pa = pos[edge.a], pb = pos[edge.b];
        float dx = pb.x - pa.x, dy = pb.y - pa.y;
        float len = sqrtf(dx * dx + dy * dy);
        float stretch = fabsf(len - edge.restLen) / edge.restLen;
        stretchBuf[e] = stretch;
        logBuf[e] = logf(stretch + eps);
    }

    __global__ void edgeGlowKernel(const float* stretchBuf, float* edgeGlow, int m,
                                    float avgStretch, float contrast, float decay) {
        int e = blockIdx.x * blockDim.x + threadIdx.x;
        if (e >= m) return;
        float norm = stretchBuf[e] / (stretchBuf[e] + avgStretch);
        norm = powf(norm, contrast);
        edgeGlow[e] = fmaxf(norm, edgeGlow[e] * decay);
    }

    __global__ void resetNodeStretchKernel(float* nodeStretchGlow, int n) {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i < n) nodeStretchGlow[i] = 0.0f;
    }

    // atomicMax по float-битам корректен для НЕотрицательных значений (glow
    // всегда в [0,1) — Reinhard-нормализация не даёт отрицательных): для
    // неотрицательных float битовый порядок совпадает со знаковым int
    // порядком, поэтому обычный atomicMax(int*) даёт верный float-max без
    // отдельной CAS-петли. Безусловно по ВСЕМ m рёбрам (не только
    // processChunks) — та же семантика, что и CPU-агрегация натяжения на
    // узел (см. SpringNetwork::step(), фаза после edgeGlow).
    __global__ void nodeStretchAggKernel(const EdgeD* __restrict__ edges,
                                          const float* __restrict__ edgeGlow,
                                          float* nodeStretchGlow, int m) {
        int e = blockIdx.x * blockDim.x + threadIdx.x;
        if (e >= m) return;
        EdgeD edge = edges[e];
        float g = edgeGlow[e];
        int gi = __float_as_int(g);
        atomicMax(reinterpret_cast<int*>(&nodeStretchGlow[edge.a]), gi);
        atomicMax(reinterpret_cast<int*>(&nodeStretchGlow[edge.b]), gi);
    }
}

CudaSpringBackend::CudaSpringBackend() = default;

CudaSpringBackend::~CudaSpringBackend() {
    if (m_dEdges) cudaFree(m_dEdges);
    if (m_dRestPos) cudaFree(m_dRestPos);
    if (m_dStructOffset) cudaFree(m_dStructOffset);
    if (m_dShearOffset) cudaFree(m_dShearOffset);
    if (m_dBendOffset) cudaFree(m_dBendOffset);
    if (m_dPos) cudaFree(m_dPos);
    if (m_dVel) cudaFree(m_dVel);
    if (m_dForce) cudaFree(m_dForce);
    if (m_dPinned) cudaFree(m_dPinned);
    if (m_dNodeGlow) cudaFree(m_dNodeGlow);
    if (m_dNodeStretchGlow) cudaFree(m_dNodeStretchGlow);
    if (m_dSpeedBuf) cudaFree(m_dSpeedBuf);
    if (m_dLogBufN) cudaFree(m_dLogBufN);
    if (m_dEdgeGlow) cudaFree(m_dEdgeGlow);
    if (m_dStretchBuf) cudaFree(m_dStretchBuf);
    if (m_dLogBufM) cudaFree(m_dLogBufM);
    if (m_dProcessChunks) cudaFree(m_dProcessChunks);
    if (m_dReduceTmp) cudaFree(m_dReduceTmp);
    if (m_dSumOut) cudaFree(m_dSumOut);
}

bool CudaSpringBackend::isAvailable() {
    int count = 0;
    return (cudaGetDeviceCount(&count) == cudaSuccess && count > 0);
}

void CudaSpringBackend::ensureNodeBuffers(size_t n) {
    if (n <= m_nodeCapacity) return;
    if (m_dPos) cudaFree(m_dPos);
    if (m_dVel) cudaFree(m_dVel);
    if (m_dForce) cudaFree(m_dForce);
    if (m_dPinned) cudaFree(m_dPinned);
    if (m_dNodeGlow) cudaFree(m_dNodeGlow);
    if (m_dNodeStretchGlow) cudaFree(m_dNodeStretchGlow);
    if (m_dSpeedBuf) cudaFree(m_dSpeedBuf);
    if (m_dLogBufN) cudaFree(m_dLogBufN);
    if (m_dRestPos) cudaFree(m_dRestPos);

    cudaMalloc(&m_dPos, n * sizeof(float2));
    cudaMalloc(&m_dVel, n * sizeof(float2));
    cudaMalloc(&m_dForce, n * sizeof(float2));
    cudaMalloc(&m_dPinned, n * sizeof(uint8_t));
    cudaMalloc(&m_dNodeGlow, n * sizeof(float));
    cudaMalloc(&m_dNodeStretchGlow, n * sizeof(float));
    cudaMalloc(&m_dSpeedBuf, n * sizeof(float));
    cudaMalloc(&m_dLogBufN, n * sizeof(float));
    cudaMalloc(&m_dRestPos, n * sizeof(float2));
    m_nodeCapacity = n;
}

void CudaSpringBackend::ensureEdgeBuffers(size_t m) {
    if (m <= m_edgeCapacity) return;
    if (m_dEdges) cudaFree(m_dEdges);
    if (m_dEdgeGlow) cudaFree(m_dEdgeGlow);
    if (m_dStretchBuf) cudaFree(m_dStretchBuf);
    if (m_dLogBufM) cudaFree(m_dLogBufM);

    cudaMalloc(&m_dEdges, m * sizeof(EdgeD));
    cudaMalloc(&m_dEdgeGlow, m * sizeof(float));
    cudaMalloc(&m_dStretchBuf, m * sizeof(float));
    cudaMalloc(&m_dLogBufM, m * sizeof(float));
    m_edgeCapacity = m;
}

void CudaSpringBackend::ensureProcessBuffer(size_t count) {
    if (count <= m_processCapacity) return;
    if (m_dProcessChunks) cudaFree(m_dProcessChunks);
    cudaMalloc(&m_dProcessChunks, count * sizeof(int));
    m_processCapacity = count;
}

void CudaSpringBackend::ensureReduceBuffer(size_t maxItems) {
    size_t bytes = 0;
    cub::DeviceReduce::Sum(nullptr, bytes,
                           static_cast<float*>(nullptr), static_cast<float*>(nullptr),
                           static_cast<int>(maxItems));
    if (bytes > m_reduceTmpBytes) {
        if (m_dReduceTmp) cudaFree(m_dReduceTmp);
        cudaMalloc(&m_dReduceTmp, bytes);
        m_reduceTmpBytes = bytes;
    }
    if (!m_dSumOut) cudaMalloc(&m_dSumOut, sizeof(float));
}

void CudaSpringBackend::uploadTopology(const std::vector<EdgeGpu>& edges,
                                       const std::vector<glm::vec2>& restPos,
                                       const std::vector<uint32_t>& structOffset,
                                       const std::vector<uint32_t>& shearOffset,
                                       const std::vector<uint32_t>& bendOffset,
                                       int numChunks) {
    m_numNodes = static_cast<int>(restPos.size());
    m_numEdges = static_cast<int>(edges.size());
    m_numChunks = numChunks;

    ensureNodeBuffers(static_cast<size_t>(m_numNodes));
    ensureEdgeBuffers(static_cast<size_t>(m_numEdges));
    ensureReduceBuffer(static_cast<size_t>(std::max(m_numNodes, m_numEdges)));

    if (m_dStructOffset) cudaFree(m_dStructOffset);
    if (m_dShearOffset) cudaFree(m_dShearOffset);
    if (m_dBendOffset) cudaFree(m_dBendOffset);
    cudaMalloc(&m_dStructOffset, structOffset.size() * sizeof(uint32_t));
    cudaMalloc(&m_dShearOffset, shearOffset.size() * sizeof(uint32_t));
    cudaMalloc(&m_dBendOffset, bendOffset.size() * sizeof(uint32_t));

    cudaMemcpy(m_dEdges, edges.data(), edges.size() * sizeof(EdgeGpu), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dRestPos, restPos.data(), restPos.size() * sizeof(glm::vec2), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dStructOffset, structOffset.data(), structOffset.size() * sizeof(uint32_t), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dShearOffset, shearOffset.data(), shearOffset.size() * sizeof(uint32_t), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dBendOffset, bendOffset.data(), bendOffset.size() * sizeof(uint32_t), cudaMemcpyHostToDevice);

    // nodeGlow/nodeStretchGlow/edgeGlow должны стартовать с нуля — то же,
    // что SpringNetwork::reset() делает для m_nodeGlow/m_edgeGlow.
    cudaMemset(m_dNodeGlow, 0, static_cast<size_t>(m_numNodes) * sizeof(float));
    cudaMemset(m_dNodeStretchGlow, 0, static_cast<size_t>(m_numNodes) * sizeof(float));
    cudaMemset(m_dEdgeGlow, 0, static_cast<size_t>(m_numEdges) * sizeof(float));

    m_topologyUploaded = true;
}

void CudaSpringBackend::step(std::vector<glm::vec2>& pos, std::vector<glm::vec2>& vel,
                             const std::vector<uint8_t>& pinned,
                             std::vector<float>& nodeGlow, std::vector<float>& nodeStretchGlow,
                             std::vector<float>& edgeGlow,
                             std::vector<float>& outSpeedBuf, std::vector<float>& outStretchBuf,
                             const std::vector<int>& processChunks,
                             int substeps, float subDt, float dampFactor, float maxSpeed,
                             float stiffness,
                             float nodeGlowDecay, float edgeGlowDecay, float glowContrast,
                             float avgSpeedFloor, float avgStretchFloor) {
    if (!m_topologyUploaded) return;
    const int n = m_numNodes;
    const int m = m_numEdges;
    if (n == 0) return;

    // pos/vel/pinned/glow-состояние меняются на хосте между кадрами (drag,
    // pluck, brush мутируют напрямую) — грузим целиком каждый вызов, не
    // кэшируем (см. design-план, Stage 3: это честная цена GPU-пути, не
    // устраняется chunk-sleep'ом, тот экономит только compute).
    cudaMemcpy(m_dPos, pos.data(), static_cast<size_t>(n) * sizeof(float2), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dVel, vel.data(), static_cast<size_t>(n) * sizeof(float2), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dPinned, pinned.data(), static_cast<size_t>(n) * sizeof(uint8_t), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dNodeGlow, nodeGlow.data(), static_cast<size_t>(n) * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(m_dEdgeGlow, edgeGlow.data(), static_cast<size_t>(m) * sizeof(float), cudaMemcpyHostToDevice);

    ensureProcessBuffer(processChunks.empty() ? 1 : processChunks.size());
    const int numProcessChunks = static_cast<int>(processChunks.size());
    if (numProcessChunks > 0) {
        cudaMemcpy(m_dProcessChunks, processChunks.data(),
                  processChunks.size() * sizeof(int), cudaMemcpyHostToDevice);
    }

    const int nBlocks = (n + kBlockSize - 1) / kBlockSize;
    const int mBlocks = (m + kBlockSize - 1) / kBlockSize;

    const uint32_t* structOff = static_cast<uint32_t*>(m_dStructOffset);
    const uint32_t* shearOff = static_cast<uint32_t*>(m_dShearOffset);
    const uint32_t* bendOff = static_cast<uint32_t*>(m_dBendOffset);
    const EdgeD* edges = static_cast<EdgeD*>(m_dEdges);
    float2* dPos = static_cast<float2*>(m_dPos);
    float2* dVel = static_cast<float2*>(m_dVel);
    float2* dForce = static_cast<float2*>(m_dForce);
    const uint8_t* dPinned = static_cast<uint8_t*>(m_dPinned);
    const float2* dRestPos = static_cast<float2*>(m_dRestPos);
    const int* dProcessChunks = static_cast<int*>(m_dProcessChunks);

    for (int s = 0; s < substeps; ++s) {
        zeroForceKernel<<<nBlocks, kBlockSize>>>(dForce, n);
        if (numProcessChunks > 0) {
            springForceKernel<<<numProcessChunks, kBlockSize>>>(dProcessChunks, edges, structOff, dPos, dForce, stiffness);
            springForceKernel<<<numProcessChunks, kBlockSize>>>(dProcessChunks, edges, shearOff, dPos, dForce, stiffness);
            springForceKernel<<<numProcessChunks, kBlockSize>>>(dProcessChunks, edges, bendOff, dPos, dForce, stiffness);
        }
        integrateKernel<<<nBlocks, kBlockSize>>>(dPos, dVel, dForce, dPinned, dRestPos, n, subDt, dampFactor, maxSpeed);
    }
    cudaOk(cudaGetLastError(), "substep kernels");

    // ---- Glow (раз за кадр, безусловно для всех n/m — см. .h) ----
    constexpr float kLogEps = 1e-4f;
    float* speedBuf = static_cast<float*>(m_dSpeedBuf);
    float* logBufN = static_cast<float*>(m_dLogBufN);
    speedLogKernel<<<nBlocks, kBlockSize>>>(dVel, speedBuf, logBufN, n, kLogEps);

    size_t reduceBytes = m_reduceTmpBytes;
    cub::DeviceReduce::Sum(m_dReduceTmp, reduceBytes, logBufN, static_cast<float*>(m_dSumOut), n);
    float sumLogSpeed = 0.0f;
    cudaMemcpy(&sumLogSpeed, m_dSumOut, sizeof(float), cudaMemcpyDeviceToHost);
    float avgSpeed = std::max(std::exp(sumLogSpeed / static_cast<float>(n)), avgSpeedFloor);

    float* nodeGlowDev = static_cast<float*>(m_dNodeGlow);
    nodeGlowKernel<<<nBlocks, kBlockSize>>>(speedBuf, nodeGlowDev, n, avgSpeed, glowContrast, nodeGlowDecay);

    float* stretchBuf = static_cast<float*>(m_dStretchBuf);
    float* logBufM = static_cast<float*>(m_dLogBufM);
    float* edgeGlowDev = static_cast<float*>(m_dEdgeGlow);
    float avgStretch = avgStretchFloor;
    if (m > 0) {
        stretchLogKernel<<<mBlocks, kBlockSize>>>(dPos, edges, stretchBuf, logBufM, m, kLogEps);
        reduceBytes = m_reduceTmpBytes;
        cub::DeviceReduce::Sum(m_dReduceTmp, reduceBytes, logBufM, static_cast<float*>(m_dSumOut), m);
        float sumLogStretch = 0.0f;
        cudaMemcpy(&sumLogStretch, m_dSumOut, sizeof(float), cudaMemcpyDeviceToHost);
        avgStretch = std::max(std::exp(sumLogStretch / static_cast<float>(m)), avgStretchFloor);
        edgeGlowKernel<<<mBlocks, kBlockSize>>>(stretchBuf, edgeGlowDev, m, avgStretch, glowContrast, edgeGlowDecay);
    }

    float* nodeStretchGlowDev = static_cast<float*>(m_dNodeStretchGlow);
    resetNodeStretchKernel<<<nBlocks, kBlockSize>>>(nodeStretchGlowDev, n);
    if (m > 0) {
        nodeStretchAggKernel<<<mBlocks, kBlockSize>>>(edges, edgeGlowDev, nodeStretchGlowDev, m);
    }
    cudaOk(cudaGetLastError(), "glow kernels");

    // ---- Скачать всё, что нужно хосту: pos/vel (drag читает их со следующего
    // кадра), glow-буферы (рендер), сырые speed/stretch (energetic-пересчёт
    // m_chunkActive на CPU, см. design-план: единый источник правды по
    // активности остаётся на хосте). ----
    pos.resize(static_cast<size_t>(n));
    vel.resize(static_cast<size_t>(n));
    nodeGlow.resize(static_cast<size_t>(n));
    nodeStretchGlow.resize(static_cast<size_t>(n));
    edgeGlow.resize(static_cast<size_t>(m));
    outSpeedBuf.resize(static_cast<size_t>(n));
    outStretchBuf.resize(static_cast<size_t>(m));

    cudaMemcpy(pos.data(), dPos, static_cast<size_t>(n) * sizeof(float2), cudaMemcpyDeviceToHost);
    cudaMemcpy(vel.data(), dVel, static_cast<size_t>(n) * sizeof(float2), cudaMemcpyDeviceToHost);
    cudaMemcpy(nodeGlow.data(), nodeGlowDev, static_cast<size_t>(n) * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(nodeStretchGlow.data(), nodeStretchGlowDev, static_cast<size_t>(n) * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(outSpeedBuf.data(), speedBuf, static_cast<size_t>(n) * sizeof(float), cudaMemcpyDeviceToHost);
    if (m > 0) {
        cudaMemcpy(edgeGlow.data(), edgeGlowDev, static_cast<size_t>(m) * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(outStretchBuf.data(), stretchBuf, static_cast<size_t>(m) * sizeof(float), cudaMemcpyDeviceToHost);
    }
}
