// tests/light_field_wave/main.cpp
//
// Regression test for LightField (demo/light): does energy injected right at
// a chunk seam actually propagate INTO the neighboring chunk (not frozen by
// chunk-sleep, same failure mode already checked for demo/cloth in
// tests/cloth_chunk_seam), and does a field with a higher waveSpeedSq
// actually propagate its wavefront FARTHER than a slower one after the same
// number of steps (the whole point of per-color dispersion in demo/light -
// if this doesn't hold, chromatic separation in the demo would be an
// illusion).
//
// Exit 0 = pass. Exit 1 = at least one failure.

#include "demo/light/LightField.h"
// LightField.cpp lives under demo/light/ - fe_add_app() (CMakeLists.txt)
// only globs sources next to main.cpp, so it isn't picked up automatically
// for a tests/ target. Including it directly keeps this self-contained (same
// convention as tests/cloth_chunk_seam).
#include "demo/light/LightField.cpp"
#include "engine/core/TaskScheduler.h"

#include <cstdio>
#include <cmath>

namespace {
bool testSeamCrossing() {
    constexpr int kCols = 128, kRows = 128;
    constexpr float kSpacing = 32.0f;
    LightField field(kCols, kRows, kSpacing);

    glm::vec2 seamWorld = field.worldPos(64, 64);
    field.pluck(seamWorld, 20.0f);

    constexpr float dt = 1.0f / 60.0f;
    constexpr float waveSpeedSq = 100.0f;
    constexpr float dampingRate = 2.0f;
    constexpr int steps = 30;
    for (int i = 0; i < steps; ++i) field.step(dt, waveSpeedSq, dampingRate, 0.0f);

    std::vector<float> glow, mask, accum;
    field.snapshot(glow, mask, accum);
    auto glowAt = [&](int col, int row) { return glow[static_cast<size_t>(row) * kCols + col]; };

    float leftGlow = glowAt(60, 64);
    float rightGlow = glowAt(68, 64);
    float controlGlow = glowAt(10, 10);

    constexpr float kThreshold = 0.01f;
    bool pass = true;

    std::printf("[seam] leftGlow(col=60)=%.4f rightGlow(col=68)=%.4f controlGlow(col=10)=%.4f\n",
                leftGlow, rightGlow, controlGlow);

    if (leftGlow <= kThreshold) {
        std::printf("FAIL[seam]: left-of-seam node did not receive energy from pluck at all\n");
        pass = false;
    }
    if (rightGlow <= kThreshold) {
        std::printf("FAIL[seam]: right-of-seam node (neighbor chunk) never became energetic - "
                    "energy is NOT crossing the chunk boundary\n");
        pass = false;
    }
    if (controlGlow > kThreshold) {
        std::printf("FAIL[seam]: distant control node became energetic - chunk activity/halo "
                    "inclusion is too broad\n");
        pass = false;
    }
    if (pass) std::printf("PASS[seam]: energy crosses the chunk seam, distant chunks stay asleep\n");
    return pass;
}

bool testDispersion() {
    constexpr int kCols = 128, kRows = 128;
    constexpr float kSpacing = 32.0f;
    LightField fast(kCols, kRows, kSpacing);
    LightField slow(kCols, kRows, kSpacing);

    glm::vec2 center = fast.worldPos(64, 64);   // fast/slow одинаковой геометрии - позиция совпадает
    fast.pluck(center, 30.0f);
    slow.pluck(center, 30.0f);

    constexpr float dt = 1.0f / 60.0f;
    constexpr float dampingRate = 0.0f;   // без затухания - амплитуда фронта сравнима
    constexpr int steps = 40;
    for (int i = 0; i < steps; ++i) {
        fast.step(dt, 300.0f, dampingRate, 0.0f);
        slow.step(dt, 60.0f, dampingRate, 0.0f);
    }

    std::vector<float> glowFast, maskFast, accumFast, glowSlow, maskSlow, accumSlow;
    fast.snapshot(glowFast, maskFast, accumFast);
    slow.snapshot(glowSlow, maskSlow, accumSlow);

    constexpr float kThreshold = 0.02f;
    // Идём вдоль +x от центра, ищем самый дальний узел, где фронт ещё
    // "энергичен" - это и есть текущее положение волнового фронта.
    auto frontRadius = [&](const std::vector<float>& glow) {
        int front = 0;
        for (int col = 64; col < kCols; ++col) {
            float g = glow[static_cast<size_t>(64) * kCols + col];
            if (g > kThreshold) front = col - 64;
        }
        return front;
    };

    int fastFront = frontRadius(glowFast);
    int slowFront = frontRadius(glowSlow);

    std::printf("[dispersion] fastFront=%d slowFront=%d\n", fastFront, slowFront);

    bool pass = true;
    if (fastFront <= slowFront) {
        std::printf("FAIL[dispersion]: field with higher waveSpeedSq did not propagate farther - "
                    "per-color dispersion in demo/light would be a no-op\n");
        pass = false;
    }
    if (pass) std::printf("PASS[dispersion]: higher waveSpeedSq propagates its front farther, as expected\n");
    return pass;
}

// Sanity check for LightField::paintMediumPolygon() (demo/light's map
// presets): a node at the triangle's centroid should end up inside the
// medium, a node clearly outside the bounding box should not.
bool testMediumPolygon() {
    constexpr int kCols = 64, kRows = 64;
    constexpr float kSpacing = 32.0f;
    LightField field(kCols, kRows, kSpacing);

    glm::vec2 center = field.worldPos(32, 32);
    std::vector<glm::vec2> triangle = {
        {center.x, center.y - 200.0f},
        {center.x + 173.0f, center.y + 100.0f},
        {center.x - 173.0f, center.y + 100.0f},
    };
    field.paintMediumPolygon(triangle, 1.0f);

    std::vector<float> glow, mask, accum;
    field.snapshot(glow, mask, accum);
    auto maskAt = [&](int col, int row) { return mask[static_cast<size_t>(row) * kCols + col]; };

    float insideMask = maskAt(32, 32);
    float outsideMask = maskAt(5, 5);

    std::printf("[polygon] insideMask=%.2f outsideMask=%.2f\n", insideMask, outsideMask);

    bool pass = true;
    if (insideMask < 0.99f) {
        std::printf("FAIL[polygon]: centroid node was not painted as medium\n");
        pass = false;
    }
    if (outsideMask > 0.01f) {
        std::printf("FAIL[polygon]: node clearly outside the triangle got painted anyway\n");
        pass = false;
    }
    if (pass) std::printf("PASS[polygon]: point-in-polygon paint only covers the triangle interior\n");
    return pass;
}
}

int main() {
    TaskScheduler::instance().initialize();

    bool pass = true;
    pass &= testSeamCrossing();
    pass &= testDispersion();
    pass &= testMediumPolygon();

    TaskScheduler::instance().shutdown();
    return pass ? 0 : 1;
}
