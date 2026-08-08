// demo/benchmark_demo/main.cpp
//
// Микробенчмарк шага клеточного автомата: CPU-бэкенд против активного
// (CUDA, если проект собран с ней и есть GPU). Считает throughput (клеток/сек)
// и коэффициент ускорения. Окно не открывается — это headless-замер.
//
// ВАЖНО про конфигурацию сборки: CpuLifeBackend - обычный C++ цикл, и разница
// между Debug и Release для него семикратная (37 против ~280 Mcells/s на этой
// машине). CUDA-часть при этом почти не меняется: ядро исполняется на
// устройстве, а host-код вокруг него в общем времени почти не виден. Поэтому
// Debug-прогон завышает "ускорение GPU" примерно в семь раз, сравнивая
// заторможенный CPU с полноценным GPU. Один раз этим уже была испорчена
// таблица в README (см. docs/BENCHMARKS.md, раздел про происхождение 109x) -
// теперь конфигурация печатается в шапке вывода, чтобы перепутать было нельзя.

#include "engine/simulation/CpuLifeBackend.h"
#include "engine/simulation/SimulationBackendFactory.h"
#include "engine/simulation/LifeRule.h"

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

double benchBackend(ISimulationBackend& backend,
                    const std::vector<uint8_t>& ext, int extW,
                    std::vector<uint8_t>& out, int S,
                    const LifeRule& rule, int iterations) {
    // Прогрев (первый запуск CUDA включает инициализацию контекста/аллокацию).
    backend.simulate(ext.data(), extW, out.data(), S, rule);

    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i)
        backend.simulate(ext.data(), extW, out.data(), S, rule);
    auto t1 = std::chrono::steady_clock::now();

    return std::chrono::duration<double>(t1 - t0).count();
}

} // namespace

int main(int argc, char** argv) {
    const int S = (argc > 1) ? std::atoi(argv[1]) : 1024;   // сторона чанка
    const int iterations = (argc > 2) ? std::atoi(argv[2]) : 100;
    const int extW = S + 2;

    // Конфигурация сборки - первой строкой, до всяких чисел. NDEBUG - это то,
    // что реально определяет, оптимизирован ли CPU-цикл, а не имя пресета:
    // пресет можно переопределить из командной строки, макрос соврать не может.
    // Любая цифра из этого стенда без этой строки рядом не значит ничего.
#ifdef NDEBUG
    const char* kBuildConfig = "Release/NDEBUG (оптимизированный)";
#else
    const char* kBuildConfig = "Debug (БЕЗ ОПТИМИЗАЦИЙ - CPU-число занижено примерно в 7 раз!)";
#endif

    std::printf("Tessera simulation benchmark\n");
    std::printf("  build: %s\n", kBuildConfig);
    std::printf("  chunk: %dx%d (%lld cells), iterations: %d\n\n",
                S, S, static_cast<long long>(S) * S, iterations);

    // Случайное стартовое поле в расширенном буфере (с границей).
    std::vector<uint8_t> ext(static_cast<size_t>(extW) * extW, 0);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 99);
    for (auto& v : ext) v = (dist(rng) < 30) ? 255 : 0;

    std::vector<uint8_t> out(static_cast<size_t>(S) * S, 0);
    const LifeRule rule = MakeConwayRule();
    const double cells = static_cast<double>(S) * S * iterations;

    // --- CPU (эталон) ---
    CpuLifeBackend cpu;
    double cpuSec = benchBackend(cpu, ext, extW, out, S, rule, iterations);
    std::printf("  %-5s : %8.3f s   %8.2f Mcells/s\n",
                cpu.name(), cpuSec, cells / cpuSec / 1e6);

    // --- Активный бэкенд (CUDA, если доступна) ---
    std::unique_ptr<ISimulationBackend> active = MakeSimulationBackend();
    if (std::string(active->name()) != "CPU") {
        double accSec = benchBackend(*active, ext, extW, out, S, rule, iterations);
        std::printf("  %-5s : %8.3f s   %8.2f Mcells/s\n",
                    active->name(), accSec, cells / accSec / 1e6);
        std::printf("\n  speedup (%s vs CPU): %.1fx\n", active->name(), cpuSec / accSec);
    } else {
        std::printf("\n  (CUDA backend unavailable — build with CUDA Toolkit to compare GPU)\n");
    }

    return 0;
}
