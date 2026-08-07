// tests/light_field_benchmark/main.cpp
//
// Headless-бенчмарк LightField::step() (demo/light) - не regression-тест,
// печатает тайминги. Два сценария:
//   sparse - типичный случай (как Dispersion demo): один непрерывный луч,
//            остальное поле спит по chunk-sleep.
//   full   - худший случай: всё поле искусственно держится разбуженным.
// Разница между ними - прямая мера пользы от chunk-scoped обработки в step().
//
// Запуск: Test_light_field_benchmark.exe [iterations]

#include "demo/light/LightField.h"
// LightField.cpp лежит в demo/light/ - fe_add_app() (CMakeLists.txt) глобит
// исходники только рядом с main.cpp, так что для tests/ цели он не
// подхватится автоматически. Инклюдим напрямую (тот же приём, что и в
// tests/light_field_wave).
#include "demo/light/LightField.cpp"
#include "engine/core/TaskScheduler.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

constexpr int kCols = 400, kRows = 400;
constexpr float kSpacing = 32.0f;
constexpr float dt = 1.0f / 60.0f;
constexpr float kWaveSpeedSq = 400.0f;

double stepAllMs(LightField& r, LightField& g, LightField& b, float dispersion) {
    auto t0 = std::chrono::steady_clock::now();
    r.step(dt, kWaveSpeedSq * 0.92f, 0.0f, dispersion);
    g.step(dt, kWaveSpeedSq * 1.00f, 0.0f, dispersion);
    b.step(dt, kWaveSpeedSq * 1.10f, 0.0f, dispersion);
    auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// Сценарий из Dispersion demo: призма + один непрерывный луч. Большая часть
// 400x400 поля остаётся спящей всю дорогу.
void benchSparse(int iterations) {
    LightField r(kCols, kRows, kSpacing), g(kCols, kRows, kSpacing), b(kCols, kRows, kSpacing);

    glm::vec2 center = r.worldPos(kCols / 2, kRows / 2);
    std::vector<glm::vec2> triangle = {
        {center.x, center.y - 300.0f},
        {center.x + 260.0f, center.y + 150.0f},
        {center.x - 260.0f, center.y + 150.0f},
    };
    r.paintMediumPolygon(triangle, 1.0f);
    g.paintMediumPolygon(triangle, 1.0f);
    b.paintMediumPolygon(triangle, 1.0f);

    glm::vec2 origin = r.worldPos(60, kRows / 2);
    glm::vec2 dir(1.0f, 0.0f);

    auto fireBeam = [&](float time) {
        r.beam(origin, dir, 250.0f, 2.0f, 600.0f, time, dt);
        g.beam(origin, dir, 250.0f, 2.0f, 600.0f, time, dt);
        b.beam(origin, dir, 250.0f, 2.0f, 600.0f, time, dt);
    };

    // Прогрев - даём chunk-активности выйти на устойчивое состояние
    // (сколько чанков реально держит луч+рассеяние в прогретом поле).
    constexpr int kWarmupFrames = 180;   // 3с
    float time = 0.0f;
    for (int i = 0; i < kWarmupFrames; ++i) {
        fireBeam(time);
        stepAllMs(r, g, b, 0.4f);
        time += dt;
    }

    constexpr int defaultIters = 300;
    int iters = iterations > 0 ? iterations : defaultIters;
    double totalMs = 0.0;
    for (int i = 0; i < iters; ++i) {
        fireBeam(time);
        totalMs += stepAllMs(r, g, b, 0.4f);
        time += dt;
    }

    double avgMs = totalMs / iters;
    std::printf("[sparse]  %d кадров, avg %.4f ms/step(R+G+B), %.0f FPS-эквивалент, поле %dx%d\n",
                iters, avgMs, avgMs > 0.0 ? 1000.0 / avgMs : 0.0, kCols, kRows);
}

// Худший случай: держим ВСЁ поле разбуженным каждый кадр (сетка точек-brush
// по всем чанкам), чтобы измерить верхнюю границу без пользы от chunk-sleep.
void benchFullyActive(int iterations) {
    LightField r(kCols, kRows, kSpacing), g(kCols, kRows, kSpacing), b(kCols, kRows, kSpacing);

    constexpr int kChunk = LightField::kChunkSize;
    std::vector<glm::vec2> wakePoints;
    for (int row = kChunk / 2; row < kRows; row += kChunk)
        for (int col = kChunk / 2; col < kCols; col += kChunk)
            wakePoints.push_back(r.worldPos(col, row));

    auto keepAwake = [&](LightField& f) {
        for (auto& p : wakePoints) f.brush(p, kSpacing * (kChunk * 0.75f), 40.0f, dt);
    };

    constexpr int kWarmupFrames = 60;
    for (int i = 0; i < kWarmupFrames; ++i) {
        keepAwake(r); keepAwake(g); keepAwake(b);
        stepAllMs(r, g, b, 0.4f);
    }

    constexpr int defaultIters = 300;
    int iters = iterations > 0 ? iterations : defaultIters;
    double totalMs = 0.0;
    for (int i = 0; i < iters; ++i) {
        keepAwake(r); keepAwake(g); keepAwake(b);
        totalMs += stepAllMs(r, g, b, 0.4f);
    }

    double avgMs = totalMs / iters;
    std::printf("[full]    %d кадров, avg %.4f ms/step(R+G+B), %.0f FPS-эквивалент, поле %dx%d\n",
                iters, avgMs, avgMs > 0.0 ? 1000.0 / avgMs : 0.0, kCols, kRows);
}

}   // namespace

int main(int argc, char** argv) {
    int iterations = argc > 1 ? std::atoi(argv[1]) : 0;

    TaskScheduler::instance().initialize();
    std::printf("Tessera LightField benchmark - %dx%d, %zu потоков\n\n",
                kCols, kRows, TaskScheduler::instance().thread_count());

    benchSparse(iterations);
    benchFullyActive(iterations);

    TaskScheduler::instance().shutdown();
    return 0;
}
