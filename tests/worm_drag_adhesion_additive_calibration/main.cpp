// tests/worm_drag_adhesion_additive_calibration/main.cpp
//
// Calibration for Params::dragAdhesionAdditiveGain (WormSim.h/body.hpp/
// body.cpp) - a second, ADDITIVE variant of the same force-law fix tested by
// tests/worm_drag_adhesion_calibration, added after that file's MULTIPLICATIVE
// mechanism (c_n_k = c_n*(1+gain*|u_k|)) was found to have a genuine
// MATHEMATICAL ceiling at exactly water/agar=1.0 (never higher, confirmed to
// gain=5000) and a joint search with the full CPG parameter set (tests/worm_
// drag_adhesion_cpg_calibration) did not meaningfully move it either (0/300,
// best ratio 0.983).
//
// WHY the multiplicative form has that ceiling: c_n_k = c_n*(1+gain*|u_k|)
// gives agar and water the SAME proportional boost regardless of their very
// different base c_n (40 vs 1.7) - as gain->infinity, both media's
// force-balance solution converges to the SAME infinite-anisotropy kinematic
// limit (a well-known RFT property: at infinite anisotropy the solution
// depends only on the shape-change pattern, not the drag magnitude) - since
// |u_k| (shape-change-only velocity, computed before drag ever enters, see
// body.cpp) is essentially identical between media whenever nothing feeds
// load back into the network's own drive, BOTH media's speed converges to
// the identical asymptotic value. This is a structural property of the
// PROPORTIONAL scaling, not a tuning problem - no gain value fixes it.
//
// The fix tested here: c_n_k = c_n + gain*|u_k| (ADDITIVE, not proportional
// to c_n). At any FINITE gain, the SAME absolute boost gain*|u_k| is a small
// relative perturbation on agar's large base (40) but a large relative
// perturbation on water's small base (1.7) - e.g. at gain*|u_k|=10: agar's
// c_n_k=50 (+25%), water's c_n_k=11.7 (+588%). This breaks the proportional-
// scaling symmetry that causes the multiplicative ceiling, at finite gain -
// though at gain->infinity this form ALSO converges to the same shared limit
// (the additive term eventually dominates c_n for both media too), so the
// interesting regime is specifically FINITE, moderate gain, not "turn it up
// as far as possible" the way the multiplicative screen behaved.
//
// This is also arguably MORE physically apt than the multiplicative form:
// capillary pinning (Rabets, Backholm, Dalnoki-Veress & Ryu 2014, Biophysical
// J. 107:1980; Sauvage et al. 2011) is a force from surface tension and
// contact geometry - not inherently a percentage of whatever the surrounding
// medium's background viscosity happens to be. An additive force contribution
// is a closer match to that physical picture than a multiplier on c_n.
//
// Search scope: dragAdhesionAdditiveGain ALONE first (dragAdhesionGain=0,
// cpgGain=0, everything else at production defaults) - this project's
// "isolate before joint" discipline. Range is NOT reused from the
// multiplicative axis (different units/scale entirely - gain*|u_k| here is
// compared against c_n's own scale of 1.7-40, not against "1") - calibrated
// instead via this file's own 'trace' mode before picking search bounds, same
// discipline tests/worm_cpg_calibration adopted after getting
// cpgLoadSensitivity's scale wrong by analogy once already this session.
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried (efficiency>=0.40, coiledRatio>=0.30, freqHz>=
// kMinFreqHz, maxAbsHeadingDelta<=kMaxHeadingDeltaRad). Fitness/reporting
// tracks the ACTUAL water/agar ratio (not just a >1 boolean) - the target is
// 2-3x (Fang-Yen et al. 2010 / Vidal-Gadea et al. 2011's real crawl/swim
// ratio), not merely crossing 1.0.
//
// RESULT: THE FIRST HEALTHY WATER>AGAR RESULT IN THIS PROJECT'S ENTIRE
// HISTORY. Screen (1D, gain in {0,1,3,10,30,100,300,1000}, one base): ratio
// rises monotonically from baseline 0.234 (gain=0) through 0.707 (g=1), 0.871
// (g=3), 0.969 (g=10), CROSSES 1.0 at g=30 (ratio=1.006), peaks around
// g=100 (1.018), then stays flat/slightly declines through g=300 (1.013) and
// g=1000 (1.005) - health (efficiency, coiled ratio, heading delta) stays
// good across the ENTIRE range, never collapsing the way the multiplicative
// mechanism's precedent (tests/worm_speed_calibration's reverted bend-rate-
// drag attempt) or this file's own later CPG-combination attempts did.
// CONFIRMED via distribution mode at g=100: 16 independent bases x 8 seeds -
// water>agar in 16/16 bases, ratio 1.017-1.019 (remarkably tight, not a
// single-base illusion), all 16 healthy on both media. CONFIRMED further via
// a 60-trial fine sweep of gain in [20,250]: 60/60 healthy with ratio>1.0,
// but capped in the same tight plateau (~1.014-1.019, best 1.019 at
// g=94.8) - this is a genuine, real, robust ceiling of ITS OWN for this
// single-parameter axis, not noise and not a still-rising curve. Falls far
// short of the real animal's 2-3x (Fang-Yen et al. 2010 / Vidal-Gadea et al.
// 2011) - progress, not a solved bug.
//
// Two follow-up combination attempts, both NEGATIVE (this mechanism works
// BEST ALONE, not combined):
//  - Joint with the full CPG parameter set (tests/worm_drag_adhesion_
//    additive_cpg_calibration, 8 params, 300 trials): 0/300 with ratio>1.0;
//    best healthy candidate found only 0.424 - dramatically WORSE than this
//    mechanism alone. Screen showed why: adhesion>0 combined with cpgGain>0
//    mostly collapses the gait entirely (freq=0, maxHeadDelta up to 3.1 rad)
//    rather than combining productively.
//  - Joint with dragAdhesionGain (the multiplicative form, tests/worm_drag_
//    adhesion_calibration): additive=100 + mult=20, 8 bases x 6 seeds -
//    ratio=0.986, all healthy but WORSE than additive alone (1.017-1.019).
//    The two force-law variants do not stack.
//
// Conclusion: dragAdhesionAdditiveGain is real, honest, healthy progress -
// genuinely breaks the mathematical ceiling that trapped the multiplicative
// mechanism at exactly 1.0 (see that file's RESULT for why) - but has its
// own, lower ceiling around ratio~1.02, and neither of the two most obvious
// combinations (CPG, multiplicative) improves on it; both make it worse.
// NOT shipped as a changed default (stays at Params default 0.0) pending the
// project owner's decision - a confirmed-real ~1.8% water>agar margin is a
// genuinely different kind of result than every prior "0 healthy candidates"
// finding in this project and deserves a human look before flipping a
// production default, not an autonomous change. Untried, honest next steps:
// (a) a different saturating functional form for the additive term itself
// (e.g. a shifted/scaled sigmoid instead of a bare linear gain*|u_k|, to see
// if the ~1.02 plateau has room past its current shape); (b) accept this as
// the ceiling of body-physics-only, load-oblivious mechanisms and pursue the
// dopamine/serotonin-style adaptive-gain-state mechanism (WORM.md section 6)
// as a genuinely different lever - unlike everything tried so far, it is not
// a fixed constant searched to work identically in both media, but a state
// that itself adapts to the sensed environment, closer to how the real
// animal actually solves this exact problem.
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/body.cpp"
#include "demo/worm/connectome/loader.cpp"
#include "demo/worm/WormSim.cpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr float kBodyLength = 576.0f;
constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kHexSpacing = 36.0f;
constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMaxHeadingDeltaRad = 0.5f;

using Candidate = std::array<float, 1>;  // {dragAdhesionAdditiveGain}
enum { kGain = 0 };
const Candidate kIdentity = {0.0f};

void applyCalibration(WormSim& sim, const Candidate& c) { sim.params.dragAdhesionAdditiveGain = c[kGain]; }

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;
    float minCoiledRatio = 1e9f;
    float freqHz = 0.0f;
    float maxAbsHeadingDelta = 0.0f;
    bool healthy = true;
};

Measurement runTrial(const Candidate& cand, int seed, float dragNormal, int warmupSteps, int measureSteps,
                      int freqPosition = 12) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    sim.params.dragTangent = 1.0f;
    sim.params.dragNormal = dragNormal;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
    const float dt = sim.params.dt.load();

    for (int i = 0; i < warmupSteps; ++i) sim.step();

    WormSim::Snapshot snap;
    sim.snapshot(snap);
    auto centroid = [](const WormSim::Snapshot& s, float& cx, float& cy) {
        cx = 0.0f; cy = 0.0f;
        for (std::size_t i = 0; i < s.pointsX.size(); ++i) { cx += s.pointsX[i]; cy += s.pointsY[i]; }
        cx /= static_cast<float>(s.pointsX.size());
        cy /= static_cast<float>(s.pointsY.size());
    };
    float startX, startY;
    centroid(snap, startX, startY);
    float prevX = startX, prevY = startY;
    double pathLen = 0.0;

    float prevDeviation = 0.0f;
    bool havePrevDeviation = false;
    int zeroCrossings = 0;
    float prevHeading = 0.0f;
    bool havePrevHeading = false;
    for (int i = 0; i < measureSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        float x, y;
        centroid(snap, x, y);
        pathLen += std::sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
        prevX = x; prevY = y;

        const auto& dev = sim.lastCurvatureDeviation();
        if (freqPosition >= 0 && freqPosition < static_cast<int>(dev.size())) {
            const float d = dev[static_cast<std::size_t>(freqPosition)];
            if (havePrevDeviation && ((d > 0.0f) != (prevDeviation > 0.0f))) ++zeroCrossings;
            prevDeviation = d;
            havePrevDeviation = true;
        }

        const float headX = snap.pointsX[0], headY = snap.pointsY[0];
        const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            m.maxAbsHeadingDelta = std::max(m.maxAbsHeadingDelta, std::fabs(hd));
        }
        prevHeading = heading;
        havePrevHeading = true;

        if (std::isnan(x) || std::isnan(y)) { m.healthy = false; return m; }
        if (i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
            for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
            const float diagNow = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            m.minCoiledRatio = std::min(m.minCoiledRatio, diagNow / kBodyLength);
        }
    }

    if (m.minCoiledRatio < 0.30f) m.healthy = false;
    if (m.maxAbsHeadingDelta > kMaxHeadingDeltaRad) m.healthy = false;

    const float netDisp = std::sqrt((prevX - startX) * (prevX - startX) + (prevY - startY) * (prevY - startY));
    const float measureSeconds = static_cast<float>(measureSteps) * dt;
    m.efficiency = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
    m.bodyLengthsPerSec = static_cast<float>(pathLen) / measureSeconds / kBodyLength;
    m.freqHz = static_cast<float>(zeroCrossings) / 2.0f / measureSeconds;
    return m;
}

struct AggregateResult {
    float meanBLps = 0.0f, stderrBLps = 0.0f, meanFreqHz = 0.0f, meanEfficiency = 0.0f, minCoiledRatio = 1e9f;
    float maxHeadingDelta = 0.0f;
    bool allHealthy = true;
};

AggregateResult evaluate(const Candidate& cand, int numSeeds, int seedBase, float dragNormal, int warmupSteps,
                          int measureSteps, int freqPosition = 12) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int s = 0; s < numSeeds; ++s) {
        const Measurement m = runTrial(cand, seedBase + s, dragNormal, warmupSteps, measureSteps, freqPosition);
        ar.minCoiledRatio = std::min(ar.minCoiledRatio, m.minCoiledRatio);
        ar.maxHeadingDelta = std::max(ar.maxHeadingDelta, m.maxAbsHeadingDelta);
        if (!m.healthy) { ar.allHealthy = false; continue; }
        blSamples.push_back(m.bodyLengthsPerSec);
        sumFreq += m.freqHz;
        sumEff += m.efficiency;
    }
    if (!blSamples.empty()) {
        double sum = 0.0;
        for (float v : blSamples) sum += v;
        const float mean = static_cast<float>(sum / blSamples.size());
        double sq = 0.0;
        for (float v : blSamples) sq += (v - mean) * (v - mean);
        const float stddev = blSamples.size() > 1 ? std::sqrt(static_cast<float>(sq / (blSamples.size() - 1))) : 0.0f;
        ar.meanBLps = mean;
        ar.stderrBLps = stddev / std::sqrt(static_cast<float>(blSamples.size()));
        ar.meanFreqHz = static_cast<float>(sumFreq / blSamples.size());
        ar.meanEfficiency = static_cast<float>(sumEff / blSamples.size());
    }
    return ar;
}

constexpr float kMinEfficiency = 0.40f;
constexpr float kMinCoiledRatio = 0.30f;
constexpr float kMinFreqHz = 0.001f;

bool isHealthy(const AggregateResult& ar) {
    return ar.allHealthy && ar.minCoiledRatio >= kMinCoiledRatio && ar.meanEfficiency >= kMinEfficiency &&
           ar.meanFreqHz >= kMinFreqHz && ar.maxHeadingDelta <= kMaxHeadingDeltaRad;
}

void printAgg(const char* label, const AggregateResult& ar) {
    std::printf("  %-6s speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f maxHeadDelta=%.4f healthy=%s\n",
                label, ar.meanBLps, ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                ar.maxHeadingDelta, isHealthy(ar) ? "yes" : "NO");
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution gain [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        const int numBases = argc > 3 ? std::atoi(argv[3]) : 12;
        const int seedsPerBase = argc > 4 ? std::atoi(argv[4]) : 8;
        const int warmupSteps = argc > 5 ? std::atoi(argv[5]) : 300;
        const int measureSteps = argc > 6 ? std::atoi(argv[6]) : 2500;
        std::printf("cand=[dragAdhesionAdditiveGain=%.4f] - %d bases x %d seeds\n", cand[kGain], numBases,
                    seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumRatio = 0.0;
        int bothHealthyCount = 0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(cand, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(cand, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool bothOk = agarOk && waterOk;
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  ratio water/agar: %s\n", bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (bothOk) {
                ++bothHealthyCount;
                sumRatio += ratio;
                if (ratio > 1.0f) ++waterBeatsAgarCount;
            }
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s), water healthy=%d/%d (mean "
                    "%.5f BL/s), both-healthy=%d, mean ratio=%.3f, water>agar in %d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    bothHealthyCount, bothHealthyCount ? sumRatio / bothHealthyCount : 0.0, waterBeatsAgarCount);
        return 0;
    }

    // ./exe random <trials> <gainLo> <gainHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float gainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const float gainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 2000.0f;
        const int seedsPerTrial = argc > 5 ? std::atoi(argv[5]) : 6;
        const unsigned rngSeed = argc > 6 ? static_cast<unsigned>(std::atoi(argv[6])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, dragAdhesionAdditiveGain linear [%.2f,%.2f], %d seeds/trial, "
                    "rngSeed=%u\n",
                    trials, gainLo, gainHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> gainDist(gainLo, gainHi);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0;
        float bestRatio = -1.0f, bestGain = 0.0f;
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kGain] = gainDist(rng);
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio > bestRatio) { bestRatio = ratio; bestGain = cand[kGain]; bestBase = base; }
            if (ratio > 1.0f) {
                ++foundOver1;
                if (ratio > 1.5f) ++foundOver15;
                if (ratio > 2.0f) ++foundOver2;
                std::printf("ratio=%.3f gain=%.3f base=%d\n", ratio, cand[kGain], base);
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0; %d with ratio>1.5; %d with ratio>2.0. Best "
                    "ratio: %.3f at gain=%.3f base=%d\n",
                    foundOver1, trials, foundOver15, foundOver2, bestRatio < 0.0f ? 0.0f : bestRatio, bestGain,
                    bestBase);
        std::printf("Each ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // ./exe trace [dragNormal] [gain] [steps] [seed]
    if (argc >= 2 && std::string(argv[1]) == "trace") {
        const float dragNormal = argc > 2 ? static_cast<float>(std::atof(argv[2])) : kDragAgar;
        const float gain = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const int steps = argc > 4 ? std::atoi(argv[4]) : 200;
        const int seed = argc > 5 ? std::atoi(argv[5]) : 42;
        WormSim sim("worm_data/celegans_herm.connectome");
        sim.params.dragAdhesionAdditiveGain = gain;
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = dragNormal;
        std::srand(static_cast<unsigned>(seed));
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        for (int i = 0; i < 300; ++i) sim.step();
        float sumU = 0.0f, maxU = 0.0f;
        int nSamples = 0;
        static float prevCx = 0.0f, prevCy = 0.0f;
        bool havePrev = false;
        for (int i = 0; i < steps; ++i) {
            sim.step();
            WormSim::Snapshot snap;
            sim.snapshot(snap);
            float cx = 0.0f, cy = 0.0f;
            for (std::size_t k = 0; k < snap.pointsX.size(); ++k) { cx += snap.pointsX[k]; cy += snap.pointsY[k]; }
            cx /= static_cast<float>(snap.pointsX.size());
            cy /= static_cast<float>(snap.pointsY.size());
            if (havePrev) {
                const float dt = sim.params.dt.load();
                const float u = std::sqrt((cx - prevCx) * (cx - prevCx) + (cy - prevCy) * (cy - prevCy)) / dt;
                sumU += u;
                maxU = std::max(maxU, u);
                ++nSamples;
            }
            prevCx = cx; prevCy = cy;
            havePrev = true;
        }
        std::printf("dragNormal=%.2f gain=%.4f: centroid-speed proxy mean=%.5f max=%.5f units/s over %d steps\n",
                    dragNormal, gain, nSamples ? sumU / nSamples : 0.0f, maxU, nSamples);
        return 0;
    }

    // Default: quick screen - dragAdhesionAdditiveGain 1D, wide range since
    // this axis's scale was NOT established before this file (unlike the
    // multiplicative axis) - one seed base (shortlist only).
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kGains[] = {0.0f, 1.0f, 3.0f, 10.0f, 30.0f, 100.0f, 300.0f, 1000.0f};
    std::printf("=== Screen (dragAdhesionAdditiveGain 1D, wide range, %d seeds/point, base=%d) ===\n", kScreenSeeds,
                kSeedBase);
    for (float g : kGains) {
        const Candidate cand = {g};
        const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
        const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
        const bool bothOk = isHealthy(agar) && isHealthy(water);
        const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
        std::printf("dragAdhesionAdditiveGain=%.1f:\n", g);
        printAgg("agar", agar);
        printAgg("water", water);
        std::printf("  ratio water/agar: %s\n", bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)");
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising point with "
                "'distribution' or search jointly with 'random' across many seed bases before believing it (see "
                "header).\n");
    return 0;
}
