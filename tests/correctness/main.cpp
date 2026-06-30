// demo/life_test/main.cpp
//
// Headless correctness test for Tessera simulation backends.
// Exit 0 = all tests passed. Exit 1 = at least one failure.
//
// Tests:
//   1. Block (still life)     — must not change after 1 step
//   2. Blinker H→V            — 3 cells horizontal must become vertical after 1 step
//   3. Blinker V→H (period 2) — after 2 steps must be back to horizontal
//   4. Glider position        — after 4 steps glider must shift by (+1,+1)
//   5. CPU vs CUDA agreement  — random board, run both backends N steps, output identical

#include "engine/simulation/CpuLifeBackend.h"
#include "engine/simulation/SimulationBackendFactory.h"
#include "engine/simulation/LifeRule.h"
#include "engine/utils/RleLoader.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <random>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr int S = 32;      // chunk side for pattern tests
static constexpr int EXTW = S + 2; // ext buffer width (1-cell border)

// Build a fresh ext buffer (all dead).
static std::vector<uint8_t> makeExt() {
    return std::vector<uint8_t>(static_cast<size_t>(EXTW) * EXTW, 0);
}

// Set cell (x,y) alive in ext buffer (x,y are chunk-local coords, 0-based).
static void set(std::vector<uint8_t>& ext, int x, int y) {
    ext[static_cast<size_t>(y + 1) * EXTW + (x + 1)] = 255;
}

// Read cell (x,y) from out buffer (SxS, row-major).
static bool alive(const std::vector<uint8_t>& out, int x, int y) {
    return out[static_cast<size_t>(y) * S + x] != 0;
}

// Run one simulation step: ext → out.
static void step(ISimulationBackend& backend,
                 const std::vector<uint8_t>& ext,
                 std::vector<uint8_t>& out,
                 const LifeRule& rule) {
    backend.simulate(ext.data(), EXTW, out.data(), S, rule);
}

// Run one step and rebuild ext from out (copy interior, zero border).
// This lets us run multiple steps in sequence.
static void stepInPlace(ISimulationBackend& backend,
                        std::vector<uint8_t>& ext,
                        std::vector<uint8_t>& out,
                        const LifeRule& rule) {
    backend.simulate(ext.data(), EXTW, out.data(), S, rule);
    // Rebuild ext from out (interior only; border stays 0 — open boundary).
    std::fill(ext.begin(), ext.end(), uint8_t{0});
    for (int y = 0; y < S; ++y)
        for (int x = 0; x < S; ++x)
            if (alive(out, x, y))
                set(ext, x, y);
}

// ---------------------------------------------------------------------------
// Individual test cases
// ---------------------------------------------------------------------------

static bool testBlock(ISimulationBackend& backend, const LifeRule& rule) {
    // 2x2 block centred at (14,14)...(15,15): still life, must not change.
    auto ext = makeExt();
    set(ext, 14, 14); set(ext, 15, 14);
    set(ext, 14, 15); set(ext, 15, 15);
    std::vector<uint8_t> out(S * S, 0);
    step(backend, ext, out, rule);

    bool ok = alive(out, 14, 14) && alive(out, 15, 14)
           && alive(out, 14, 15) && alive(out, 15, 15);
    // No extra cells should appear.
    int liveCount = 0;
    for (auto v : out) if (v) ++liveCount;
    ok = ok && (liveCount == 4);
    std::printf("  [%s] Block still-life (%s)  backend=%s\n",
                ok ? "PASS" : "FAIL", ok ? "unchanged" : "changed", backend.name());
    return ok;
}

static bool testBlinkerHV(ISimulationBackend& backend, const LifeRule& rule) {
    // Horizontal blinker at row 15, cols 14-16 → after 1 step must be vertical.
    auto ext = makeExt();
    set(ext, 14, 15); set(ext, 15, 15); set(ext, 16, 15);
    std::vector<uint8_t> out(S * S, 0);
    step(backend, ext, out, rule);

    // Expected: vertical — col 15, rows 14-16.
    bool ok = !alive(out, 14, 15) && !alive(out, 16, 15)  // side cells gone
           &&  alive(out, 15, 14) &&  alive(out, 15, 15) &&  alive(out, 15, 16);
    int liveCount = 0;
    for (auto v : out) if (v) ++liveCount;
    ok = ok && (liveCount == 3);
    std::printf("  [%s] Blinker H→V         (%s)  backend=%s\n",
                ok ? "PASS" : "FAIL", ok ? "rotated" : "wrong", backend.name());
    return ok;
}

static bool testBlinkerPeriod2(ISimulationBackend& backend, const LifeRule& rule) {
    // After 2 steps the blinker must be back to horizontal.
    auto ext = makeExt();
    set(ext, 14, 15); set(ext, 15, 15); set(ext, 16, 15);
    std::vector<uint8_t> out(S * S, 0);
    stepInPlace(backend, ext, out, rule); // step 1 → vertical
    stepInPlace(backend, ext, out, rule); // step 2 → horizontal again

    bool ok = alive(out, 14, 15) && alive(out, 15, 15) && alive(out, 16, 15)
           && !alive(out, 15, 14) && !alive(out, 15, 16);
    int liveCount = 0;
    for (auto v : out) if (v) ++liveCount;
    ok = ok && (liveCount == 3);
    std::printf("  [%s] Blinker period-2     (%s)  backend=%s\n",
                ok ? "PASS" : "FAIL", ok ? "back to H" : "wrong", backend.name());
    return ok;
}

static bool testGlider(ISimulationBackend& backend, const LifeRule& rule) {
    // Canonical glider (NW corner, offset so it doesn't hit boundary in 4 steps):
    //  . # .          offset by (5,5)
    //  . . #
    //  # # #
    // After exactly 4 steps it moves +1 right +1 down (south-east glider).
    auto ext = makeExt();
    const int ox = 5, oy = 5;
    set(ext, ox + 1, oy);
    set(ext, ox + 2, oy + 1);
    set(ext, ox,     oy + 2);
    set(ext, ox + 1, oy + 2);
    set(ext, ox + 2, oy + 2);

    std::vector<uint8_t> out(S * S, 0);
    for (int i = 0; i < 4; ++i)
        stepInPlace(backend, ext, out, rule);

    // Same shape at offset (+1,+1) from original.
    bool ok = alive(out, ox + 2, oy + 1)
           && alive(out, ox + 3, oy + 2)
           && alive(out, ox + 1, oy + 3)
           && alive(out, ox + 2, oy + 3)
           && alive(out, ox + 3, oy + 3);
    int liveCount = 0;
    for (auto v : out) if (v) ++liveCount;
    ok = ok && (liveCount == 5);
    std::printf("  [%s] Glider 4-step        (%s)  backend=%s\n",
                ok ? "PASS" : "FAIL", ok ? "shifted (+1,+1)" : "wrong pos", backend.name());
    return ok;
}

// ---------------------------------------------------------------------------
// CPU vs CUDA agreement test
// ---------------------------------------------------------------------------

static bool testCpuVsGpu(ISimulationBackend& cpu, ISimulationBackend& gpu,
                          const LifeRule& rule) {
    constexpr int STEPS = 100;
    constexpr int BIG   = 256;
    constexpr int BIGEXTW = BIG + 2;

    // Shared random initial state (seed fixed → reproducible).
    std::vector<uint8_t> ext(static_cast<size_t>(BIGEXTW) * BIGEXTW, 0);
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> dist(0, 99);
    for (auto& v : ext) v = (dist(rng) < 30) ? 255 : 0;

    std::vector<uint8_t> cpuOut(static_cast<size_t>(BIG) * BIG, 0);
    std::vector<uint8_t> gpuOut(static_cast<size_t>(BIG) * BIG, 0);
    std::vector<uint8_t> cpuExt = ext;
    std::vector<uint8_t> gpuExt = ext;

    auto stepBig = [&](ISimulationBackend& b,
                        std::vector<uint8_t>& e,
                        std::vector<uint8_t>& o) {
        b.simulate(e.data(), BIGEXTW, o.data(), BIG, rule);
        std::fill(e.begin(), e.end(), uint8_t{0});
        for (int y = 0; y < BIG; ++y)
            for (int x = 0; x < BIG; ++x)
                if (o[static_cast<size_t>(y) * BIG + x])
                    e[static_cast<size_t>(y + 1) * BIGEXTW + (x + 1)] = 255;
    };

    for (int i = 0; i < STEPS; ++i) {
        stepBig(cpu, cpuExt, cpuOut);
        stepBig(gpu, gpuExt, gpuOut);
    }

    bool ok = (cpuOut == gpuOut);
    if (!ok) {
        int diff = 0;
        for (size_t i = 0; i < cpuOut.size(); ++i)
            if (cpuOut[i] != gpuOut[i]) ++diff;
        std::printf("  [FAIL] CPU vs %s agreement — %d cells differ after %d steps "
                    "(grid %dx%d)\n", gpu.name(), diff, STEPS, BIG, BIG);
    } else {
        std::printf("  [PASS] CPU vs %s agreement — %d steps on %dx%d grid, "
                    "output identical\n", gpu.name(), STEPS, BIG, BIG);
    }
    return ok;
}

// ---------------------------------------------------------------------------
// RLE parser tests (pure string parsing, no GL needed)
// ---------------------------------------------------------------------------

static bool rleEq(const RlePattern& p, int w, int h, const char* art) {
    if (p.width != w || p.height != h) {
        std::printf("    size %dx%d, expected %dx%d\n", p.width, p.height, w, h);
        return false;
    }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            bool want = art[y * w + x] == '#';
            bool got  = p.cells[y * w + x] != 0;
            if (want != got) {
                std::printf("    mismatch at (%d,%d): got %d want %d\n", x, y, got, want);
                return false;
            }
        }
    return true;
}

static bool testRleParser() {
    bool ok = true;

    // R-pentomino: .## / ##. / .#.
    {
        RlePattern p = RleLoader::parse("x = 3, y = 3, rule = B3/S23\nb2o$2ob$bo!\n");
        bool t = rleEq(p, 3, 3, ".####..#.");
        std::printf("  [%s] R-pentomino parse\n", t ? "PASS" : "FAIL");
        ok &= t;
    }
    // Run-length + blank-row skip: 3o$3b3o!  -> ###... / ...### (2 rows? no)
    {
        RlePattern p = RleLoader::parse("x = 3, y = 2\n3o$3o!\n");
        bool t = rleEq(p, 3, 2, "######");
        std::printf("  [%s] run-length rows\n", t ? "PASS" : "FAIL");
        ok &= t;
    }
    // Robustness: whitespace between count and tag must NOT drop the count.
    {
        RlePattern p = RleLoader::parse("x = 3, y = 1\n3 o!\n");
        bool t = rleEq(p, 3, 1, "###");
        std::printf("  [%s] whitespace between count and tag\n", t ? "PASS" : "FAIL");
        ok &= t;
    }
    // Multi-row jump with $ count: o2$o!  -> #.. / ... / #..
    {
        RlePattern p = RleLoader::parse("x = 1, y = 3\no2$o!\n");
        bool t = rleEq(p, 1, 3, "# #");  // (#, blank, #)
        std::printf("  [%s] $N multi-row jump\n", t ? "PASS" : "FAIL");
        ok &= t;
    }
    return ok;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("Tessera simulation correctness tests\n\n");

    const LifeRule rule = MakeConwayRule();
    CpuLifeBackend cpu;

    bool allPass = true;

    std::printf("=== CPU backend ===\n");
    allPass &= testBlock(cpu, rule);
    allPass &= testBlinkerHV(cpu, rule);
    allPass &= testBlinkerPeriod2(cpu, rule);
    allPass &= testGlider(cpu, rule);

    // CUDA backend (if available).
    std::unique_ptr<ISimulationBackend> active = MakeSimulationBackend();
    if (std::string(active->name()) != "CPU") {
        std::printf("\n=== %s backend ===\n", active->name());
        allPass &= testBlock(*active, rule);
        allPass &= testBlinkerHV(*active, rule);
        allPass &= testBlinkerPeriod2(*active, rule);
        allPass &= testGlider(*active, rule);

        std::printf("\n=== CPU vs %s (determinism check) ===\n", active->name());
        allPass &= testCpuVsGpu(cpu, *active, rule);
    } else {
        std::printf("\n(CUDA backend not available — skipping GPU tests)\n");
    }

    std::printf("\n=== RLE parser ===\n");
    allPass &= testRleParser();

    std::printf("\n%s\n", allPass ? "ALL TESTS PASSED" : "*** SOME TESTS FAILED ***");
    return allPass ? 0 : 1;
}
