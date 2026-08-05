// engine/simulation/LightWaveCuda.h
#pragma once
#include <cstddef>

// GPU-версия ядра шага LightField (demo/light) - честный full-grid шаг
// (Laplacian + поглощающая граница + интеграция), без chunk-sleep: на GPU
// сон чанков не даёт выигрыша (планировщик и так параллелит весь грид), так
// что сравнивать нужно full-active CPU против full-grid GPU - это и есть
// честный "worst case vs worst case".
//
// Тот же void*-паттерн, что у CudaLifeBackend: заголовок компилируется и
// обычным C++, ни одного CUDA-типа тут быть не должно.
class LightWaveCuda {
public:
    LightWaveCuda();
    ~LightWaveCuda();
    LightWaveCuda(const LightWaveCuda&) = delete;
    LightWaveCuda& operator=(const LightWaveCuda&) = delete;

    // Заливает высоту/скорость/mediumMask на device один раз. cols/rows -
    // odd-r гекс-решётка, spacing как в LightField.
    bool upload(int cols, int rows, float spacing,
                const float* height, const float* velocity, const float* mediumMask);

    // Один шаг прямо на GPU, без H2D/D2H между кадрами. Считает то же, что и
    // LightField::step() при substeps==1: сила -> интеграция -> яркость
    // (glow/accum), включая log-average нормировку по всему полю.
    void step(float dt, float waveSpeedSq, float dampingRate, float dispersion);

    void downloadHeight(float* outHeight) const;
    void downloadGlow(float* outGlow) const;

    static bool isAvailable();

private:
    int m_cols = 0, m_rows = 0;
    float m_spacing = 0.0f;

    void* m_dHeight = nullptr;
    void* m_dVelocity = nullptr;
    void* m_dForce = nullptr;
    void* m_dMediumMask = nullptr;
    void* m_dGlow = nullptr;
    void* m_dAccum = nullptr;
    void* m_dSpeedBuf = nullptr;
    void* m_dSumLog = nullptr;      // одно float-значение, аккумулятор редукции
    // Pinned-граница - фиксированное кольцо по краю поля (см. LightField::reset()),
    // не отдельный буфер: дешевле проверить col/row в ядре, чем гонять байты.
};
