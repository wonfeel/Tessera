// tests/spring_cuda_smoke/main.cpp
//
// Standalone correctness check for CudaSpringBackend (Stage 2 of the
// chunk+CUDA plan) — does NOT touch SpringNetwork/main.cpp. Uploads a tiny
// synthetic 2-node/1-edge "grid" with a known stretch, runs one substep with
// damping=0, and checks the resulting velocity of the free node against the
// closed-form expected force: F = stiffness * (dist - restLen), applied for
// one subDt with unit mass (matches SpringNetwork's semi-implicit Euler:
// v += F*dt, no separate mass term).
//
// This only builds/runs when FE_CUDA_ENABLED (CMake skips discovering it
// into a target if the CUDA toolkit isn't found — see fe_discover_apps).
//
// Exit 0 = pass. Exit 1 = at least one failure. Skips (exit 0) if no CUDA
// device is available at runtime (e.g. compiled with CUDA but run on a
// machine without an NVIDIA GPU).

#include <cstdio>
#include <cmath>
#include <vector>

// CudaSpringBackend's methods are only DEFINED (linked into EngineLib) when
// the project was configured with CUDA found — see CMakeLists.txt's
// FE_CUDA_ENABLED block. On a CPU-only build the class declaration is still
// visible (the header has no CUDA types, see CudaSpringBackend.h), but
// calling any of its methods would be a link error — so the entire body is
// gated behind the same macro the engine already uses for this fork.
#ifdef FE_CUDA_ENABLED
#include "engine/simulation/CudaSpringBackend.h"

int main() {
    if (!CudaSpringBackend::isAvailable()) {
        std::printf("SKIP: no CUDA device available at runtime\n");
        return 0;
    }

    CudaSpringBackend backend;

    // Node 0 pinned at origin, node 1 free, stretched to 1.5x rest length
    // along +x. One chunk (numChunks=1), one structural edge, no shear/bend.
    constexpr float kSpacing = 32.0f;
    constexpr float kRestLen = kSpacing;
    constexpr float kStretchFactor = 1.5f;

    std::vector<CudaSpringBackend::EdgeGpu> edges = { {0, 1, kRestLen} };
    std::vector<glm::vec2> restPos = { glm::vec2(0.0f, 0.0f), glm::vec2(kRestLen, 0.0f) };
    std::vector<uint32_t> structOffset = { 0, 1 };   // chunk 0 owns edge [0,1)
    std::vector<uint32_t> shearOffset = { 0, 0 };
    std::vector<uint32_t> bendOffset = { 0, 0 };

    backend.uploadTopology(edges, restPos, structOffset, shearOffset, bendOffset, /*numChunks=*/1);

    std::vector<glm::vec2> pos = { glm::vec2(0.0f, 0.0f), glm::vec2(kRestLen * kStretchFactor, 0.0f) };
    std::vector<glm::vec2> vel = { glm::vec2(0.0f, 0.0f), glm::vec2(0.0f, 0.0f) };
    std::vector<uint8_t> pinned = { 1, 0 };   // node 0 fixed, node 1 free
    std::vector<float> nodeGlow(2, 0.0f), nodeStretchGlow(2, 0.0f), edgeGlow(1, 0.0f);
    std::vector<float> speedBuf, stretchBuf;
    std::vector<int> processChunks = { 0 };

    constexpr float kStiffness = 100.0f;
    constexpr float kSubDt = 1.0f / 600.0f;   // small, single substep — no integration drift to worry about
    constexpr float kDampFactor = 1.0f;       // damping=0
    constexpr float kMaxSpeed = 1e6f;         // effectively unclamped for this check

    backend.step(pos, vel, pinned, nodeGlow, nodeStretchGlow, edgeGlow,
                speedBuf, stretchBuf, processChunks,
                /*substeps=*/1, kSubDt, kDampFactor, kMaxSpeed, kStiffness,
                /*nodeGlowDecay=*/0.9f, /*edgeGlowDecay=*/0.9f, /*glowContrast=*/3.0f,
                /*avgSpeedFloor=*/1e-3f, /*avgStretchFloor=*/1e-3f);

    const float dist = kRestLen * kStretchFactor;
    const float expectedMag = kStiffness * (dist - kRestLen);       // force magnitude, +x direction on the spring
    const float expectedVelX = -expectedMag * kSubDt;                // node 1 is pulled back toward node 0 (-x)

    std::printf("vel1 = (%.6f, %.6f), expected vx = %.6f\n", vel[1].x, vel[1].y, expectedVelX);
    std::printf("stretchBuf[0] = %.6f, expected = %.6f\n", stretchBuf.empty() ? -1.0f : stretchBuf[0],
                std::fabs(dist - kRestLen) / kRestLen);

    constexpr float kTol = 1e-3f;
    bool pass = true;
    if (std::fabs(vel[1].x - expectedVelX) > kTol) {
        std::printf("FAIL: node 1 velocity.x mismatch — force computation on GPU doesn't match "
                    "stiffness*(dist-restLen)\n");
        pass = false;
    }
    if (std::fabs(vel[1].y) > kTol) {
        std::printf("FAIL: node 1 picked up unexpected y-velocity (direction bug)\n");
        pass = false;
    }
    if (pinned[0] && (vel[0].x != 0.0f || vel[0].y != 0.0f)) {
        std::printf("FAIL: pinned node 0 moved (integration kernel should skip pinned nodes)\n");
        pass = false;
    }

    if (pass) std::printf("PASS: CudaSpringBackend force/integration matches closed-form expectation\n");
    return pass ? 0 : 1;
}
#else
int main() {
    std::printf("SKIP: built without FE_CUDA_ENABLED\n");
    return 0;
}
#endif
