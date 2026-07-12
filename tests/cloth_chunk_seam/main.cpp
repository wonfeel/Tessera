// tests/spring_chunk_seam/main.cpp
//
// Headless regression test: does energy injected right at a chunk seam
// (SpringNetwork::kChunkSize boundary) actually propagate INTO the
// neighboring chunk, or does chunk-based activity skipping (see
// SpringNetwork::step()) artificially freeze it at the boundary?
//
// This is exactly the failure mode flagged during design review: a narrow
// standing pattern ("замятина") sitting right on a chunk seam could, in a
// buggy chunk-skip implementation, only ever update on one side if the
// other side's chunk never gets included in force computation. See the
// halo/interior-boundary split in SpringNetwork::step() and
// bucketEdgeBlock() — this test exercises that mechanism directly rather
// than relying on eyeballing a screenshot.
//
// Exit 0 = pass. Exit 1 = at least one failure.

#include "demo/cloth/SpringNetwork.h"
// SpringNetwork.cpp lives under demo/cloth/, outside this test's own
// directory — fe_add_app() (CMakeLists.txt) only globs sources next to
// main.cpp, so it isn't picked up automatically. Including it directly
// keeps this a self-contained single-file test without touching the
// generic CMake app-discovery glob (which would affect every demo/test).
#include "demo/cloth/SpringNetwork.cpp"
// Same reasoning as above: SpringBackendFactory.cpp lives in demo/cloth/
// too and isn't globbed into this test's own directory.
#include "demo/cloth/SpringBackendFactory.cpp"
#include "engine/core/TaskScheduler.h"

#include <cstdio>
#include <cmath>

int main() {
    TaskScheduler::instance().initialize();

    // 128x128 grid, chunkSize=64 -> a clean 2x2 chunk grid with a seam
    // running through col=64 and row=64.
    constexpr int kCols = 128, kRows = 128;
    constexpr float kSpacing = 32.0f;
    SpringNetwork net(kCols, kRows, kSpacing);

    // Pluck exactly at the seam (col=64,row=64 world position) with a
    // strong displacement so the resulting wave is unambiguous.
    glm::vec2 seamWorld(64.0f * kSpacing, 64.0f * kSpacing);
    net.pluck(seamWorld, 200.0f);

    constexpr float dt = 1.0f / 60.0f;
    constexpr float stiffness = 80.0f;
    constexpr float dampingRate = 4.0f;
    constexpr int steps = 30;
    for (int i = 0; i < steps; ++i) net.step(dt, stiffness, dampingRate);

    std::vector<glm::vec2> pos;
    std::vector<float> nodeGlow, nodeStretchGlow, edgeGlow;
    net.snapshot(pos, nodeGlow, nodeStretchGlow, edgeGlow);

    auto glowAt = [&](int col, int row) {
        return nodeGlow[static_cast<size_t>(row) * kCols + col];
    };

    // Just left of the seam (chunk col 0, the "owning" chunk of the pluck).
    float leftGlow = glowAt(60, 64);
    // Just right of the seam (chunk col 1 — the neighbor that must NOT be
    // frozen out).
    float rightGlow = glowAt(68, 64);
    // Distant control point, far from the pluck and its chunk's neighbors —
    // should stay essentially asleep/dark, confirming chunk-skip is actually
    // skipping something (not just "everything happens to be active").
    float controlGlow = glowAt(10, 10);

    constexpr float kEnergeticThreshold = 0.01f;
    bool pass = true;

    std::printf("leftGlow(col=60)=%.4f rightGlow(col=68)=%.4f controlGlow(col=10)=%.4f\n",
                leftGlow, rightGlow, controlGlow);

    if (leftGlow <= kEnergeticThreshold) {
        std::printf("FAIL: left-of-seam node did not receive energy from pluck at all "
                     "(sanity check failed — pluck itself may be broken, not a chunk bug)\n");
        pass = false;
    }
    if (rightGlow <= kEnergeticThreshold) {
        std::printf("FAIL: right-of-seam node (neighbor chunk) never became energetic — "
                     "energy is NOT crossing the chunk boundary (this is the exact bug "
                     "the halo/interior-boundary split in step() is supposed to prevent)\n");
        pass = false;
    }
    if (controlGlow > kEnergeticThreshold) {
        std::printf("FAIL: distant control node became energetic — chunk activity/halo "
                     "inclusion is too broad (effectively not skipping anything)\n");
        pass = false;
    }

    if (pass) std::printf("PASS: energy crosses the chunk seam, distant chunks stay asleep\n");
    return pass ? 0 : 1;
}
