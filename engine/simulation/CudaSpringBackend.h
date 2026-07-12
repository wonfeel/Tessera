// engine/simulation/CudaSpringBackend.h
#pragma once
#include <vector>
#include <cstdint>
#include <cstddef>
#include <glm/glm.hpp>

// GPU-бэкенд физики для demo/springs::SpringNetwork — зеркалит паттерн
// CudaLifeBackend (h/.cu разделены так, что этот заголовок не содержит НИ
// ОДНОГО CUDA-типа, только void* на device-память, чтобы его мог включать
// обычный C++-компилятор).
//
// В отличие от CPU-пути (SpringNetwork::step()), GPU-путь НЕ делит рёбра
// чанка на internal/boundary: atomicAdd в dForce — стандартный, дешёвый на
// масштабе GPU способ (тысячи потоков, сотни-тысячи рёбер на чанк), тогда как
// на CPU он был не нужен только потому что для 16-32 потоков boundary-проход
// проще писать последовательным, чем городить lock-free аккумуляцию для
// каждого потока. Здесь просто идём по ПОЛНОМУ CSR-диапазону чанка (тот же
// offset-массив, что и CPU, тот же порядок edges) — оба конца ребра могут
// принадлежать разным обрабатываемым чанкам, atomicAdd делает это безопасным
// без разделения на internal/boundary вообще.
//
// Активность чанков (какие чанки "спят") остаётся единственным источником
// правды на CPU (см. SpringNetwork::m_chunkActive/m_processChunks) — GPU
// только ИСПОЛНЯЕТ переданный список processChunks, не решает сам, кто спит.
class CudaSpringBackend {
public:
    // POD-зеркало SpringNetwork::Edge — тот же layout (int,int,float), нужен
    // отдельный тип, чтобы .h не тянул demo/springs/SpringNetwork.h (у демок
    // нет обратной зависимости на engine/simulation, а не наоборот).
    struct EdgeGpu { int a, b; float restLen; };

    CudaSpringBackend();
    ~CudaSpringBackend();

    static bool isAvailable();

    // Топология + rest-позиции — не меняются после SpringNetwork::reset(),
    // загружать один раз. edges — ВСЕ рёбра (structural+shear+bend, тот же
    // порядок, что SpringNetwork хранит внутри) — bend тоже нужен для
    // физики, в отличие от topology()/outRenderEdgeCount, который отсекает
    // bend только для рендера. offset* — те же CSR-таблицы, что
    // SpringNetwork::bucketEdgeBlock() строит для CPU (interior/boundary
    // здесь не нужны — см. комментарий класса, поэтому только offset, без
    // interiorEnd).
    void uploadTopology(const std::vector<EdgeGpu>& edges,
                        const std::vector<glm::vec2>& restPos,
                        const std::vector<uint32_t>& structOffset,
                        const std::vector<uint32_t>& shearOffset,
                        const std::vector<uint32_t>& bendOffset,
                        int numChunks);

    // Один вызов — это один SpringNetwork::step(): substeps под-шагов
    // force(atomicAdd)+integrate, затем один проход glow (как и в CPU-версии,
    // glow считается раз за кадр, не за под-шаг). pos/vel/pinned — вход
    // (текущее состояние, могло измениться drag/pluck на хосте с прошлого
    // кадра — поэтому грузим КАЖДЫЙ кадр, не кэшируем) и выход
    // (перезаписываются результатом на месте). nodeGlow/nodeStretchGlow/
    // edgeGlow — вход (нужен decay = max(norm, prev*decayRate)) и выход.
    // outSpeedBuf/outStretchBuf — сырые (до нормализации) скорость/растяжение,
    // нужны хосту для energetic-пересчёта m_chunkActive (та же логика, что
    // CPU, остаётся на хосте — единый источник правды по активности, см.
    // комментарий класса).
    void step(std::vector<glm::vec2>& pos, std::vector<glm::vec2>& vel,
              const std::vector<uint8_t>& pinned,
              std::vector<float>& nodeGlow, std::vector<float>& nodeStretchGlow,
              std::vector<float>& edgeGlow,
              std::vector<float>& outSpeedBuf, std::vector<float>& outStretchBuf,
              const std::vector<int>& processChunks,
              int substeps, float subDt, float dampFactor, float maxSpeed,
              float stiffness,
              float nodeGlowDecay, float edgeGlowDecay, float glowContrast,
              float avgSpeedFloor, float avgStretchFloor);

private:
    void ensureNodeBuffers(size_t n);
    void ensureEdgeBuffers(size_t m);
    void ensureProcessBuffer(size_t count);
    void ensureReduceBuffer(size_t maxItems);

    void* m_dEdges = nullptr;
    void* m_dRestPos = nullptr;
    void* m_dStructOffset = nullptr;
    void* m_dShearOffset = nullptr;
    void* m_dBendOffset = nullptr;

    void* m_dPos = nullptr;
    void* m_dVel = nullptr;
    void* m_dForce = nullptr;
    void* m_dPinned = nullptr;
    void* m_dNodeGlow = nullptr;
    void* m_dNodeStretchGlow = nullptr;
    void* m_dSpeedBuf = nullptr;
    void* m_dLogBufN = nullptr;
    size_t m_nodeCapacity = 0;

    void* m_dEdgeGlow = nullptr;
    void* m_dStretchBuf = nullptr;
    void* m_dLogBufM = nullptr;
    size_t m_edgeCapacity = 0;

    void* m_dProcessChunks = nullptr;
    size_t m_processCapacity = 0;

    void* m_dReduceTmp = nullptr;
    size_t m_reduceTmpBytes = 0;
    void* m_dSumOut = nullptr;   // одно float-значение, переиспользуется для speed/stretch

    int m_numNodes = 0;
    int m_numEdges = 0;
    int m_numChunks = 0;
    bool m_topologyUploaded = false;
};
