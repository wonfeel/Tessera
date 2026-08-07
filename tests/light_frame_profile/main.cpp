// tests/light_frame_profile/main.cpp
//
// Куда уходит время кадра в demo/light: физика, снимок полей или сборка
// вершин. Плюс отдельно - сколько поток физики простаивает на мьютексе,
// который рендер держит во время snapshot().
//
// Headless: GL-загрузку (glBufferData) не меряем, только CPU-часть. Пишет
// отчёт в light_frame_profile.txt рядом с exe.

#include "demo/light/LightField.h"
#include "demo/light/LightField.cpp"
#include "engine/core/TaskScheduler.h"
#include "engine/core/ParallelFor.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

namespace {

constexpr int kCols = 400, kRows = 400;
constexpr float kSpacing = 32.0f;
constexpr float dt = 1.0f / 60.0f;
constexpr float kWaveSpeedSq = 400.0f;
constexpr float kDispersion = 0.4f;

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct Fields {
    LightField r{kCols, kRows, kSpacing};
    LightField g{kCols, kRows, kSpacing};
    LightField b{kCols, kRows, kSpacing};
};

// Сцена Dispersion demo: призма + непрерывный луч, остальное поле спит.
void setupScene(Fields& f) {
    glm::vec2 center = f.r.worldPos(kCols / 2, kRows / 2);
    std::vector<glm::vec2> tri = {
        {center.x, center.y - 300.0f},
        {center.x + 260.0f, center.y + 150.0f},
        {center.x - 260.0f, center.y + 150.0f},
    };
    f.r.paintMediumPolygon(tri, 1.0f);
    f.g.paintMediumPolygon(tri, 1.0f);
    f.b.paintMediumPolygon(tri, 1.0f);
}

void fireBeam(Fields& f, float time) {
    glm::vec2 origin = f.r.worldPos(60, kRows / 2);
    glm::vec2 dir(1.0f, 0.0f);
    f.r.beam(origin, dir, 250.0f, 2.0f, 600.0f, time, dt);
    f.g.beam(origin, dir, 250.0f, 2.0f, 600.0f, time, dt);
    f.b.beam(origin, dir, 250.0f, 2.0f, 600.0f, time, dt);
}

void stepAll(Fields& f) {
    f.r.step(dt, kWaveSpeedSq * 0.92f, 0.0f, kDispersion);
    f.g.step(dt, kWaveSpeedSq * 1.00f, 0.0f, kDispersion);
    f.b.step(dt, kWaveSpeedSq * 1.10f, 0.0f, kDispersion);
}

struct RenderBuffers {
    std::vector<float> glowR, glowG, glowB;
    std::vector<float> maskR, maskG, maskB;
    std::vector<float> accumR, accumG, accumB;
    std::vector<float> vertexData;
};

void snapshotAll(Fields& f, RenderBuffers& rb) {
    f.r.snapshot(rb.glowR, rb.maskR, rb.accumR);
    f.g.snapshot(rb.glowG, rb.maskG, rb.accumG);
    f.b.snapshot(rb.glowB, rb.maskB, rb.accumB);
}

// Модель горячего цикла renderPointLayer при полном отдалении (видны все
// узлы) - 5 float на узел, цветовая математика ветки Accumulate.
void buildVerticesFrom(const float* accR, const float* accG, const float* accB,
                        const float* maskR, std::vector<float>& vertexData) {
    const int visCount = kCols * kRows;
    vertexData.resize(static_cast<size_t>(visCount) * 5);
    float* out = vertexData.data();
    parallelFor(visCount, [&](int begin, int end, int) {
        for (int idx = begin; idx < end; ++idx) {
            size_t i = static_cast<size_t>(idx);
            float mask = maskR[i];
            float aR = accR[i] < 0.0f ? 0.0f : (accR[i] > 1.0f ? 1.0f : accR[i]);
            float aG = accG[i] < 0.0f ? 0.0f : (accG[i] > 1.0f ? 1.0f : accG[i]);
            float aB = accB[i] < 0.0f ? 0.0f : (accB[i] > 1.0f ? 1.0f : accB[i]);
            float shade = 1.0f - 0.7f * mask;
            if (shade < 0.15f) shade = 0.15f;
            size_t base = i * 5;
            out[base + 0] = static_cast<float>(idx % kCols);
            out[base + 1] = static_cast<float>(idx / kCols);
            out[base + 2] = std::pow(aR, 0.6f) * shade;
            out[base + 3] = std::pow(aG, 0.6f) * shade;
            out[base + 4] = std::pow(aB, 0.6f) * shade;
        }
    });
}

void buildVertices(RenderBuffers& rb) {
    buildVerticesFrom(rb.accumR.data(), rb.accumG.data(), rb.accumB.data(),
                      rb.maskR.data(), rb.vertexData);
}

// Zero-copy путь: никаких snapshot(), читаем опубликованный слот напрямую.
void renderFrameZeroCopy(Fields& f, std::vector<float>& vertexData) {
    LightField::View vr = f.r.acquireView();
    LightField::View vg = f.g.acquireView();
    LightField::View vb = f.b.acquireView();
    buildVerticesFrom(vr.accum, vg.accum, vb.accum, vr.mediumMask, vertexData);
}

void warmup(Fields& f, float& time, int frames) {
    for (int i = 0; i < frames; ++i) {
        fireBeam(f, time);
        stepAll(f);
        time += dt;
    }
}

}   // namespace

int main() {
    TaskScheduler::instance().initialize();

    Fields fields;
    RenderBuffers rb;
    setupScene(fields);
    float time = 0.0f;
    warmup(fields, time, 180);

    constexpr int kIters = 300;

    // --- 1. Физика без рендера (базовая линия, мьютекс свободен) ---
    double physicsSoloMs = 0.0;
    for (int i = 0; i < kIters; ++i) {
        fireBeam(fields, time);
        auto t0 = Clock::now();
        stepAll(fields);
        physicsSoloMs += msSince(t0);
        time += dt;
    }
    physicsSoloMs /= kIters;

    // --- 2. Рендер-путь без физики ---
    double snapshotMs = 0.0, vertexMs = 0.0;
    for (int i = 0; i < kIters; ++i) {
        auto t0 = Clock::now();
        snapshotAll(fields, rb);
        snapshotMs += msSince(t0);

        auto t1 = Clock::now();
        buildVertices(rb);
        vertexMs += msSince(t1);
    }
    snapshotMs /= kIters;
    vertexMs /= kIters;

    // --- 3. Оба потока разом, как в реальном приложении ---
    // (Application: updateLoop на вызывающем потоке, renderLoop на своём.)
    //
    // Два прогона, чтобы не свалить в одну кучу две разные причины замедления:
    //   3a - рендер только снимает поля  -> чистая борьба за мьютекс поля;
    //   3b - рендер ещё и собирает вершины -> сверху борьба за TaskScheduler
    //        (buildVertices гоняет parallelFor по тому же пулу, что и step()).
    std::atomic<int> renderFrames{0};
    double renderSnapshotMs = 0.0;

    // --- 2b. Тот же кадр рендера, но zero-copy (без snapshot) ---
    double zeroCopyFrameMs = 0.0;
    {
        std::vector<float> vd;
        for (int i = 0; i < kIters; ++i) {
            auto t0 = Clock::now();
            renderFrameZeroCopy(fields, vd);
            zeroCopyFrameMs += msSince(t0);
        }
        zeroCopyFrameMs /= kIters;
    }

    auto measurePhysicsWithRender = [&](bool renderBuildsVertices) {
        std::atomic<bool> running{true};
        renderFrames = 0;
        renderSnapshotMs = 0.0;

        std::thread renderThread([&] {
            RenderBuffers localRb;
            while (running.load(std::memory_order_relaxed)) {
                auto t0 = Clock::now();
                snapshotAll(fields, localRb);
                renderSnapshotMs += msSince(t0);
                if (renderBuildsVertices) buildVertices(localRb);
                renderFrames.fetch_add(1, std::memory_order_relaxed);
                std::this_thread::sleep_for(std::chrono::milliseconds(16));   // ~60 Гц
            }
        });

        double ms = 0.0;
        for (int i = 0; i < kIters; ++i) {
            fireBeam(fields, time);
            auto t0 = Clock::now();
            stepAll(fields);
            ms += msSince(t0);
            time += dt;
            std::this_thread::sleep_for(std::chrono::milliseconds(8));        // ~120 Гц
        }

        running = false;
        renderThread.join();
        int rf = renderFrames.load();
        if (rf > 0) renderSnapshotMs /= rf;
        return ms / kIters;
    };

    const double physicsSnapshotOnlyMs = measurePhysicsWithRender(false);
    const double snapshotOnlyBlockedMs = renderSnapshotMs;
    const double physicsContendedMs = measurePhysicsWithRender(true);

    // --- 4. Реальный режим: физика + zero-copy рендер ---
    double physicsZeroCopyMs = 0.0;
    {
        std::atomic<bool> running{true};
        std::thread renderThread([&] {
            std::vector<float> vd;
            while (running.load(std::memory_order_relaxed)) {
                renderFrameZeroCopy(fields, vd);
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
        });
        for (int i = 0; i < kIters; ++i) {
            fireBeam(fields, time);
            auto t0 = Clock::now();
            stepAll(fields);
            physicsZeroCopyMs += msSince(t0);
            time += dt;
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }
        running = false;
        renderThread.join();
        physicsZeroCopyMs /= kIters;
    }

    const double mutexMs = physicsSnapshotOnlyMs - physicsSoloMs;
    const double poolMs = physicsContendedMs - physicsSnapshotOnlyMs;
    const double contentionMs = physicsContendedMs - physicsSoloMs;
    const double renderTotalMs = snapshotMs + vertexMs;

    FILE* out = std::fopen("light_frame_profile.txt", "w");
    auto emit = [&](const char* fmt, auto... args) {
        std::printf(fmt, args...);
        if (out) std::fprintf(out, fmt, args...);
    };

    emit("Tessera demo/light - профиль кадра\n");
    emit("поле %dx%d, 3 канала (R/G/B), %zu потоков, сцена Dispersion demo\n",
         kCols, kRows, TaskScheduler::instance().thread_count());
    emit("рендер меряется при ПОЛНОМ отдалении (видны все %d узлов) - худший случай\n\n",
         kCols * kRows);

    emit("-- поток физики (update) --\n");
    emit("  step() x3, рендер спит           : %7.3f ms   (базовая линия)\n", physicsSoloMs);
    emit("  step() x3, рендер только snapshot: %7.3f ms\n", physicsSnapshotOnlyMs);
    emit("  step() x3, рендер полный         : %7.3f ms   (как в демке)\n", physicsContendedMs);
    emit("  накладные расходы, разложение:\n");
    emit("    борьба за мьютекс поля         : %7.3f ms   %5.1f%% от накладных\n",
         mutexMs, contentionMs > 0.0 ? 100.0 * mutexMs / contentionMs : 0.0);
    emit("    борьба за пул потоков          : %7.3f ms   %5.1f%% от накладных\n",
         poolMs, contentionMs > 0.0 ? 100.0 * poolMs / contentionMs : 0.0);
    emit("    итого сверху                   : %7.3f ms   (+%.0f%% к шагу)\n\n",
         contentionMs, physicsSoloMs > 0.0 ? 100.0 * contentionMs / physicsSoloMs : 0.0);

    emit("-- поток рендера --\n");
    emit("  snapshot() x3 (копия 9 полей)    : %7.3f ms   %5.1f%% CPU-кадра\n",
         snapshotMs, renderTotalMs > 0.0 ? 100.0 * snapshotMs / renderTotalMs : 0.0);
    emit("  сборка вершин (160k узлов)       : %7.3f ms   %5.1f%% CPU-кадра\n",
         vertexMs, renderTotalMs > 0.0 ? 100.0 * vertexMs / renderTotalMs : 0.0);
    emit("  итого CPU на кадр (без GL)       : %7.3f ms\n", renderTotalMs);
    emit("  snapshot() в ожидании физики     : %7.3f ms   (против %.3f ms без неё)\n\n",
         snapshotOnlyBlockedMs, snapshotMs);

    emit("-- zero-copy (acquireView, без snapshot) --\n");
    emit("  кадр рендера целиком             : %7.3f ms   (против %.3f ms через snapshot)\n",
         zeroCopyFrameMs, renderTotalMs);
    emit("  step() x3 при таком рендере      : %7.3f ms   (против %.3f ms)\n",
         physicsZeroCopyMs, physicsContendedMs);
    emit("  выигрыш физики                   : %7.3f ms   (%.0f%%)\n\n",
         physicsContendedMs - physicsZeroCopyMs,
         physicsContendedMs > 0.0 ? 100.0 * (physicsContendedMs - physicsZeroCopyMs) / physicsContendedMs : 0.0);

    emit("-- бюджеты --\n");
    emit("  кадр при vsync 60 FPS            :  16.667 ms  -> рендер занимает %.0f%%\n",
         100.0 * renderTotalMs / 16.667);
    emit("  шаг при kMaxStepHz=120           :   8.333 ms  -> физика занимает %.0f%%\n",
         100.0 * physicsContendedMs / 8.333);

    if (out) std::fclose(out);
    TaskScheduler::instance().shutdown();
    return 0;
}
