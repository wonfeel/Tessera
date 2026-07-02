// tests/batch_benchmark/main.cpp
//
// Полный тест производительности симуляции: CPU в 1 поток, CPU через
// TaskScheduler на всех ядрах (как в реальном движке), CUDA по одному чанку
// за launch (старая схема) и CUDA батчем (см. ISimulationBackend::preferBatch,
// SimulationCoordinator::simulateActive). Прогон по нескольким размерам поля —
// от одного чанка до сотен активных.
//
// Важно про CPU-числа: CpuLifeBackend::simulate() сам по себе однопоточный,
// без внутреннего параллелизма (SIMD/OpenMP) — считает один чанк одним потоком
// от начала до конца. Многопоточность CPU-пути в реальном движке целиком
// заслуга TaskScheduler: он раздаёт отдельные чанки по std::thread::
// hardware_concurrency() воркерам, каждый из которых просто вызывает
// simulate() на своих чанках последовательно. Раздел "CPU x1 поток" ниже —
// сырая скорость одного вызова; "CPU xN потоков" — то, что реально видит
// игрок, когда активных чанков много.
//
// Headless, окно не открывается.

#include "engine/simulation/CpuLifeBackend.h"
#include "engine/simulation/SimulationBackendFactory.h"
#include "engine/simulation/LifeRule.h"
#include "engine/core/TaskScheduler.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

// Ширины столбцов — единственное место, где их нужно менять; разделитель
// и строки таблицы всегда совпадут, потому что оба строятся из этих же чисел.
constexpr int kColGroup  = 22;   // "N chunks x SxS"
constexpr int kColMethod = 18;
constexpr int kColTime   = 10;
constexpr int kColRate   = 12;
constexpr int kColSpeed  = 10;

void printRule(char fill = '-') {
    std::printf("+");
    for (int i = 0; i < kColGroup + 2; ++i) std::putchar(fill);
    std::putchar('+');
    for (int i = 0; i < kColMethod + 2; ++i) std::putchar(fill);
    std::putchar('+');
    for (int i = 0; i < kColTime + 2; ++i) std::putchar(fill);
    std::putchar('+');
    for (int i = 0; i < kColRate + 2; ++i) std::putchar(fill);
    std::putchar('+');
    for (int i = 0; i < kColSpeed + 2; ++i) std::putchar(fill);
    std::printf("+\n");
}

void printRow(const std::string& group, const std::string& method, const std::string& time,
              const std::string& rate, const std::string& speed) {
    std::printf("| %-*s | %-*s | %*s | %*s | %*s |\n",
                kColGroup, group.c_str(),
                kColMethod, method.c_str(),
                kColTime, time.c_str(),
                kColRate, rate.c_str(),
                kColSpeed, speed.c_str());
}

std::string fmt(const char* pattern, double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), pattern, v);
    return buf;
}

// Одна строка итоговой таблицы. group пустой у строк 2-4 внутри сценария —
// печатается только у первой строки, чтобы не повторять то же самое 4 раза.
struct Row { std::string group, method, time, rate, speed; };

// Раздаёт chunkCount чанков по TaskScheduler ровно так же, как
// SimulationCoordinator::simulateActive(): батчи по min(256, total/threads),
// одна задача на батч, барьер (Latch) в конце поколения.
double benchCpuMultiThread(CpuLifeBackend& cpu,
                           const std::vector<const uint8_t*>& exts, int extW,
                           const std::vector<uint8_t*>& outs, int S,
                           const LifeRule& rule, int iterations, size_t numThreads) {
    const size_t total = exts.size();
    const size_t batchSize = std::max<size_t>(1,
        std::min<size_t>(256, total / std::max<size_t>(1, numThreads)));

    auto t0 = std::chrono::steady_clock::now();
    for (int it = 0; it < iterations; ++it) {
        size_t numTasks = (total + batchSize - 1) / batchSize;
        TaskScheduler::Latch latch(static_cast<int>(numTasks));
        for (size_t i = 0; i < total; i += batchSize) {
            size_t end = std::min(i + batchSize, total);
            TaskScheduler::instance().schedule([&, i, end]() {
                for (size_t j = i; j < end; ++j)
                    cpu.simulate(exts[j], extW, outs[j], S, rule);
                latch.count_down();
            });
        }
        latch.wait();
    }
    auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t1 - t0).count();
}

double benchSequential(ISimulationBackend& backend,
                       const std::vector<const uint8_t*>& exts, int extW,
                       const std::vector<uint8_t*>& outs, int S,
                       const LifeRule& rule, int iterations) {
    auto t0 = std::chrono::steady_clock::now();
    for (int it = 0; it < iterations; ++it)
        for (size_t i = 0; i < exts.size(); ++i)
            backend.simulate(exts[i], extW, outs[i], S, rule);
    auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t1 - t0).count();
}

double benchBatch(ISimulationBackend& backend,
                  const std::vector<const uint8_t*>& exts, int extW,
                  const std::vector<uint8_t*>& outs, int S,
                  const LifeRule& rule, int iterations) {
    auto t0 = std::chrono::steady_clock::now();
    for (int it = 0; it < iterations; ++it)
        backend.simulateBatch(exts, extW, outs, S, rule);
    auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(t1 - t0).count();
}

struct Scenario { int chunkCount; int chunkSize; int iterations; };

// Гоняет один сценарий и добавляет его строки в общий список rows (без
// печати — печатаем всё одной таблицей в конце main()).
void runScenario(const Scenario& sc, CpuLifeBackend& cpu, ISimulationBackend& active,
                 bool haveCuda, size_t hwThreads, const LifeRule& rule,
                 std::vector<Row>& rows) {
    const int extW = sc.chunkSize + 2;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coin(0, 1);

    std::vector<std::vector<uint8_t>> exts(sc.chunkCount,
        std::vector<uint8_t>(static_cast<size_t>(extW) * extW));
    for (auto& e : exts) for (auto& v : e) v = coin(rng) ? 255 : 0;

    std::vector<std::vector<uint8_t>> outs(sc.chunkCount,
        std::vector<uint8_t>(static_cast<size_t>(sc.chunkSize) * sc.chunkSize, 0));

    std::vector<const uint8_t*> extPtrs(sc.chunkCount);
    std::vector<uint8_t*> outPtrs(sc.chunkCount);
    for (int i = 0; i < sc.chunkCount; ++i) {
        extPtrs[i] = exts[i].data();
        outPtrs[i] = outs[i].data();
    }

    // Прогрев — первый CUDA-вызов включает инициализацию контекста.
    active.simulate(extPtrs[0], extW, outPtrs[0], sc.chunkSize, rule);

    const double totalCells =
        static_cast<double>(sc.chunkCount) * sc.chunkSize * sc.chunkSize * sc.iterations;

    // "^2", не "²" — printf-паддинг считает байты, а "²" в UTF-8 занимает 2,
    // из-за чего рамка таблицы съезжает на терминалах, не схлопывающих её сами.
    const std::string group = std::to_string(sc.chunkCount) + " x " +
        std::to_string(sc.chunkSize) + "^2";

    const double cpu1Sec = benchSequential(cpu, extPtrs, extW, outPtrs, sc.chunkSize, rule, sc.iterations);
    const double cpu1Rate = totalCells / cpu1Sec / 1e6;
    rows.push_back({group, "CPU x1 thread", fmt("%.4fs", cpu1Sec), fmt("%.1f", cpu1Rate), "1.00x"});

    const double cpuNSec = benchCpuMultiThread(cpu, extPtrs, extW, outPtrs, sc.chunkSize, rule,
                                                sc.iterations, hwThreads);
    const double cpuNRate = totalCells / cpuNSec / 1e6;
    rows.push_back({"", "CPU x" + std::to_string(hwThreads) + " threads",
                    fmt("%.4fs", cpuNSec), fmt("%.1f", cpuNRate), fmt("%.2fx", cpu1Sec / cpuNSec)});

    if (haveCuda) {
        const double cudaSeqSec = benchSequential(active, extPtrs, extW, outPtrs, sc.chunkSize, rule, sc.iterations);
        const double cudaSeqRate = totalCells / cudaSeqSec / 1e6;
        rows.push_back({"", "CUDA per-chunk", fmt("%.4fs", cudaSeqSec), fmt("%.1f", cudaSeqRate),
                        fmt("%.2fx", cpu1Sec / cudaSeqSec)});

        const double cudaBatchSec = benchBatch(active, extPtrs, extW, outPtrs, sc.chunkSize, rule, sc.iterations);
        const double cudaBatchRate = totalCells / cudaBatchSec / 1e6;
        rows.push_back({"", "CUDA batched", fmt("%.4fs", cudaBatchSec), fmt("%.1f", cudaBatchRate),
                        fmt("%.2fx", cpu1Sec / cudaBatchSec)});
    }
}

} // namespace

int main() {
    const size_t hwThreads = std::thread::hardware_concurrency();

    std::printf("Tessera performance test\n\n");
    std::printf("  CPU threads (hardware_concurrency): %zu\n", hwThreads);
    std::printf("  CpuLifeBackend::simulate() is single-threaded per call —\n");
    std::printf("  parallelism across chunks comes entirely from TaskScheduler\n");
    std::printf("  fanning simulate() out to all %zu worker threads.\n\n", hwThreads);

    TaskScheduler::instance().initialize();

    std::unique_ptr<ISimulationBackend> active = MakeSimulationBackend();
    const bool haveCuda = std::string(active->name()) == "CUDA";
    std::printf("  active backend: %s\n\n", active->name());

    CpuLifeBackend cpu;
    LifeRule rule = MakeConwayRule();

    if (!haveCuda) {
        std::printf("  (CUDA backend unavailable — build with CUDA Toolkit to compare GPU)\n\n");
    }

    // Три группы сценариев: (1) разное число активных чанков при дефолтном
    // chunkSize=64, (2) один чанк растущего размера — сырой per-launch
    // throughput, (3) 8 чанков растущего размера — реалистичный случай, где
    // видно, при каком chunkSize CUDA обгоняет CPU x12 потоков, а не только
    // однопоточный CPU.
    const std::vector<Scenario> byCount = {
        {  1, 64, 2000},
        { 20, 64,  200},
        {100, 64,   80},
        {300, 64,   30},
        {800, 64,   12},
    };
    const std::vector<Scenario> bySize = {
        {1,   64, 400},
        {1,  256, 100},
        {1,  512,  40},
        {1, 1024,  15},
        {1, 2048,   6},
    };
    const std::vector<Scenario> bySizeMulti = {
        {8,   64, 200},
        {8,  256,  40},
        {8,  512,  15},
        {8, 1024,   6},
    };

    std::vector<Row> rows;
    std::vector<size_t> groupBoundaries;   // индекс первой строки каждой группы (для разделителей)

    for (const auto& sc : byCount) {
        groupBoundaries.push_back(rows.size());
        runScenario(sc, cpu, *active, haveCuda, hwThreads, rule, rows);
    }
    for (const auto& sc : bySize) {
        groupBoundaries.push_back(rows.size());
        runScenario(sc, cpu, *active, haveCuda, hwThreads, rule, rows);
    }
    for (const auto& sc : bySizeMulti) {
        groupBoundaries.push_back(rows.size());
        runScenario(sc, cpu, *active, haveCuda, hwThreads, rule, rows);
    }

    std::printf("chunk count sweep (chunkSize=64) -> chunk size sweep (1 chunk) -> "
                "chunk size sweep (8 chunks)\n\n");
    printRule();
    printRow("Scenario", "Method", "Time", "Mcells/s", "Speedup");
    printRule();
    for (size_t i = 0; i < rows.size(); ++i) {
        // Разделитель перед каждой новой группой (кроме самой первой строки).
        if (i > 0 && std::find(groupBoundaries.begin(), groupBoundaries.end(), i) != groupBoundaries.end())
            printRule();
        const auto& r = rows[i];
        printRow(r.group, r.method, r.time, r.rate, r.speed);
    }
    printRule();

    TaskScheduler::instance().shutdown();
    return 0;
}
