// tests/spring_reset_repro/main.cpp
//
// Repro attempt for an access-violation crash observed in the interactive
// demo while dragging/plucking a node (drag/pluck mode, not brush). Mimics
// the exact frame-by-frame call pattern from demo/cloth/main.cpp:
// handleDragPluckInput() (update thread: beginDrag/updateDrag/endDrag/pluck
// interleaved with step()) plus a background thread hammering snapshot()
// (render thread's real workload), at the SAME 300x200 grid size currently
// configured in demo/cloth/main.cpp — ragged (non-multiple-of-64) chunk
// edges, unlike the clean 128x128 seam test.
//
// Exit 0 = no crash / no corruption detected. An abnormal process exit
// (not via return) is itself the repro signal.

#include "demo/cloth/SpringNetwork.h"
#include "demo/cloth/SpringNetwork.cpp"
#include "demo/cloth/SpringBackendFactory.cpp"
#include "engine/core/TaskScheduler.h"

#include <atomic>
#include <cstdio>
#include <random>
#include <thread>

int main() {
    TaskScheduler::instance().initialize();

    constexpr int kCols = 300, kRows = 200;
    constexpr float kSpacing = 32.0f;
    SpringNetwork net(kCols, kRows, kSpacing);

    std::atomic<bool> running{true};
    std::atomic<uint64_t> snapshotCount{0};
    std::thread renderThread([&] {
        std::vector<glm::vec2> pos;
        std::vector<float> nodeGlow, nodeStretchGlow, edgeGlow;
        while (running.load()) {
            net.snapshot(pos, nodeGlow, nodeStretchGlow, edgeGlow);
            snapshotCount.fetch_add(1);
        }
    });

    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> colDist(0.0f, static_cast<float>(kCols - 1));
    std::uniform_real_distribution<float> rowDist(0.0f, static_cast<float>(kRows - 1));
    constexpr float dt = 1.0f / 60.0f;

    for (int cycle = 0; cycle < 200; ++cycle) {
        glm::vec2 startWorld(colDist(rng) * kSpacing, rowDist(rng) * kSpacing);
        int grabbed = net.beginDrag(startWorld);

        // Drag across a random path for a random number of frames, stepping
        // physics every frame — exactly main.cpp's onUpdate() order
        // (handlePointerInput before stepPhysics).
        int dragFrames = 5 + (cycle % 20);
        for (int f = 0; f < dragFrames; ++f) {
            glm::vec2 target(colDist(rng) * kSpacing, rowDist(rng) * kSpacing);
            if (grabbed >= 0) net.updateDrag(grabbed, target, dt);
            net.step(dt, 80.0f, 4.0f);
        }
        if (grabbed >= 0) net.endDrag(grabbed);

        // Interleave a pluck too (RMB in the real demo), at another random
        // position, then a few more steps.
        glm::vec2 pluckWorld(colDist(rng) * kSpacing, rowDist(rng) * kSpacing);
        net.pluck(pluckWorld, 60.0f);
        for (int f = 0; f < 5; ++f) net.step(dt, 80.0f, 4.0f);

        if (cycle % 50 == 0) std::printf("cycle %d ok (snapshots so far: %llu)\n",
                                          cycle, (unsigned long long)snapshotCount.load());
    }

    running.store(false);
    renderThread.join();

    std::printf("PASS: no crash across 200 drag/pluck cycles with concurrent snapshot() "
                "(%llu total snapshots)\n", (unsigned long long)snapshotCount.load());
    return 0;
}
