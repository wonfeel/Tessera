// tests/light_field_cuda_benchmark/main.cpp
//
// CPU vs CUDA throughput для hex-wave ядра LightField (demo/light) - честное
// сравнение "worst case vs worst case": оба прогоняют full grid каждый шаг.
// GPU-путь (LightWaveCuda) не имеет chunk-sleep - на GPU он и не даёт
// выигрыша (варпы и так параллелят весь грид), так что сравнивать CPU
// chunk-sleep (см. tests/light_field_benchmark, sparse-сценарий) против
// full-grid GPU было бы нечестно. Здесь оба - full-active.
//
// Запуск: Test_light_field_cuda_benchmark.exe [iterations]

#include "demo/light/LightField.h"
// LightField.cpp лежит в demo/light/ - fe_add_app() (CMakeLists.txt) глобит
// исходники только рядом с main.cpp, для tests/ цели не подхватится
// автоматически. Инклюдим напрямую (тот же приём, что и в tests/light_field_wave).
#include "demo/light/LightField.cpp"
#include "engine/core/TaskScheduler.h"

#ifdef FE_CUDA_ENABLED
#include "engine/simulation/LightWaveCuda.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

namespace {

constexpr int kCols = 400, kRows = 400;
constexpr float kSpacing = 32.0f;
constexpr float dt = 1.0f / 60.0f;
constexpr float kWaveSpeedSq = 400.0f;
constexpr float kDispersion = 0.4f;

// Держим ВСЁ поле разбуженным каждый кадр (brush по всем чанкам) - без
// этого CPU быстро уснул бы по chunk-sleep, и сравнение с GPU (у которого
// сна нет) было бы нечестным в пользу CPU.
double benchCpu(int iterations) {
    LightField field(kCols, kRows, kSpacing);

    constexpr int kChunk = LightField::kChunkSize;
    std::vector<glm::vec2> wakePoints;
    for (int row = kChunk / 2; row < kRows; row += kChunk)
        for (int col = kChunk / 2; col < kCols; col += kChunk)
            wakePoints.push_back(field.worldPos(col, row));
    auto keepAwake = [&] {
        for (auto& p : wakePoints) field.brush(p, kSpacing * (kChunk * 0.75f), 40.0f, dt);
    };

    constexpr int kWarmupFrames = 60;
    for (int i = 0; i < kWarmupFrames; ++i) {
        keepAwake();
        field.step(dt, kWaveSpeedSq, 0.0f, kDispersion);
    }

    // keepAwake() - скаффолдинг (не даём chunk-sleep усыпить поле), у GPU
    // аналога нет, поэтому в замер он попадать не должен: таймер только
    // вокруг самого step().
    double totalMs = 0.0;
    for (int i = 0; i < iterations; ++i) {
        keepAwake();
        auto t0 = std::chrono::steady_clock::now();
        field.step(dt, kWaveSpeedSq, 0.0f, kDispersion);
        auto t1 = std::chrono::steady_clock::now();
        totalMs += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    return totalMs / iterations;
}

#ifdef FE_CUDA_ENABLED
// Быстрый бенчмарк неверного ядра ничего не стоит - сверяем, что GPU считает
// ту же физику. Одинаковый старт (одиночный pluck), N шагов, сравнение height.
// Спящие у CPU области тут не мешают: там всё нулевое, GPU на нулях тоже даёт
// нули, так что поля обязаны совпасть везде.
bool verifyAgainstCpu() {
    constexpr int kVerifySteps = 40;
    constexpr float kPluck = 20.0f;

    LightField cpu(kCols, kRows, kSpacing);
    glm::vec2 p = cpu.worldPos(kCols / 2, kRows / 2);
    cpu.pluck(p, kPluck);
    for (int i = 0; i < kVerifySteps; ++i) cpu.step(dt, kWaveSpeedSq, 0.0f, kDispersion);

    std::vector<float> cpuGlow, cpuMask, cpuAccum;
    cpu.snapshot(cpuGlow, cpuMask, cpuAccum);

    // Тот же одиночный спайк на GPU. pluck() кладёт добавку в ближайший узел -
    // для (kCols/2, kRows/2) это ровно он сам.
    size_t n = static_cast<size_t>(kCols) * kRows;
    std::vector<float> height(n, 0.0f), velocity(n, 0.0f), mask(n, 0.0f);
    height[static_cast<size_t>(kRows / 2) * kCols + kCols / 2] = kPluck;

    LightWaveCuda gpu;
    if (!gpu.upload(kCols, kRows, kSpacing, height.data(), velocity.data(), mask.data()))
        return false;
    for (int i = 0; i < kVerifySteps; ++i) gpu.step(dt, kWaveSpeedSq, 0.0f, kDispersion);

    std::vector<float> gpuGlow(n);
    gpu.downloadGlow(gpuGlow.data());

    double maxAbsDiff = 0.0;
    double sumCpu = 0.0, sumGpu = 0.0;
    for (size_t i = 0; i < n; ++i) {
        maxAbsDiff = std::max(maxAbsDiff, static_cast<double>(std::fabs(gpuGlow[i] - cpuGlow[i])));
        sumCpu += cpuGlow[i];
        sumGpu += gpuGlow[i];
    }

    // Порог не машинный эпсилон: порядок сложения в редукции log-average на
    // GPU другой, плюс __expf/expf - разные приближения, так что расхождение
    // в младших разрядах glow ожидаемо и физику не меняет.
    constexpr double kTol = 2e-3;
    bool ok = maxAbsDiff < kTol;
    std::printf("  verify: max|glow_gpu - glow_cpu| = %.2e, sum cpu=%.3f gpu=%.3f -> %s\n\n",
                maxAbsDiff, sumCpu, sumGpu, ok ? "OK" : "РАСХОЖДЕНИЕ");
    return ok;
}
#endif

}   // namespace

int main(int argc, char** argv) {
    int iterations = argc > 1 ? std::atoi(argv[1]) : 300;

    TaskScheduler::instance().initialize();
    std::printf("Tessera LightField CPU vs CUDA - %dx%d grid, full-active каждый шаг, 1 канал\n", kCols, kRows);
    std::printf("Обе стороны считают одно и то же: сила -> интеграция -> яркость (glow/accum).\n\n");

#ifdef FE_CUDA_ENABLED
    bool verified = LightWaveCuda::isAvailable() ? verifyAgainstCpu() : false;
#endif

    double cpuMs = benchCpu(iterations);
    std::printf("  CPU  : %8.4f ms/step   %8.0f steps/s   (%zu потоков)\n",
                cpuMs, 1000.0 / cpuMs, TaskScheduler::instance().thread_count());

#ifdef FE_CUDA_ENABLED
    if (LightWaveCuda::isAvailable()) {
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> distH(-0.05f, 0.05f), distV(-0.5f, 0.5f);
        std::vector<float> height(static_cast<size_t>(kCols) * kRows);
        std::vector<float> velocity(static_cast<size_t>(kCols) * kRows);
        std::vector<float> mediumMask(static_cast<size_t>(kCols) * kRows, 0.0f);
        for (auto& h : height) h = distH(rng);
        for (auto& v : velocity) v = distV(rng);

        LightWaveCuda gpu;
        if (gpu.upload(kCols, kRows, kSpacing, height.data(), velocity.data(), mediumMask.data())) {
            constexpr int kWarmupFrames = 60;
            for (int i = 0; i < kWarmupFrames; ++i) gpu.step(dt, kWaveSpeedSq, 0.0f, kDispersion);
            gpu.downloadHeight(height.data());   // синхронизация перед замером

            // Запуски ядер асинхронные - D2H в конце блокирует, пока вся
            // очередь не отработает, так что замер честно покрывает всю
            // работу (это throughput с полным перекрытием, не latency
            // отдельного шага).
            auto t0 = std::chrono::steady_clock::now();
            for (int i = 0; i < iterations; ++i) gpu.step(dt, kWaveSpeedSq, 0.0f, kDispersion);
            gpu.downloadHeight(height.data());
            auto t1 = std::chrono::steady_clock::now();

            double gpuMs = std::chrono::duration<double, std::milli>(t1 - t0).count() / iterations;
            std::printf("  CUDA : %8.4f ms/step   %8.0f steps/s\n", gpuMs, 1000.0 / gpuMs);
            std::printf("\n  speedup (CUDA vs CPU): %.1fx%s\n", cpuMs / gpuMs,
                        verified ? "" : "   [!] ЯДРО НЕ СВЕРЕНО - число некорректно");
        } else {
            std::printf("\n  (CUDA upload failed - см. stderr)\n");
        }
    } else {
        std::printf("\n  (CUDA собрана, но устройство не найдено)\n");
    }
#else
    std::printf("\n  (CUDA backend unavailable - build with CUDA Toolkit to compare GPU)\n");
#endif

    TaskScheduler::instance().shutdown();
    return 0;
}
