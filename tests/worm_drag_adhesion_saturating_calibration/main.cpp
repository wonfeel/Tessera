// tests/worm_drag_adhesion_saturating_calibration/main.cpp
//
// Calibration for Params::dragAdhesionSatCap/dragAdhesionSatRate (WormSim.h/
// body.hpp/body.cpp) - a THIRD variant of the force-law fix, added after
// tests/worm_drag_adhesion_additive_calibration confirmed the additive form
// (c_n += gain*|u_k|) genuinely breaks past ratio=1.0 (first ever water>agar
// in this project) but plateaus at ~1.02 and does not rise further even at
// gain up to 1000.
//
// WHY the additive form plateaus: at gain*|u_k| >> c_n, the UNBOUNDED
// additive term itself becomes the dominant contributor to c_n_k for BOTH
// media equally - the same "converges to a shared infinite-anisotropy limit"
// mechanism that caps the multiplicative form at exactly 1.0, just reached
// at a higher absolute level because the additive term's growth is not tied
// to c_n's own magnitude the way the multiplicative term's is. Either way,
// an UNBOUNDED per-|u_k| boost eventually washes out the base c_n difference
// between agar (40) and water (1.7) that the whole mechanism depends on.
//
// The fix tested here: cap the boost. c_n_k += dragAdhesionSatCap*(1 -
// exp(-dragAdhesionSatRate*|u_k|)) saturates toward dragAdhesionSatCap as
// |u_k| grows, rather than growing without bound. At saturation, agar's
// c_n_k approaches (40+cap) and water's approaches (1.7+cap) - two DIFFERENT
// constants, not a shared limit - so there is no a priori mathematical
// reason (unlike the other two forms) for both media to be forced toward
// the same asymptotic speed. Whether this actually lets the ratio exceed
// ~1.02 in practice is an open empirical question, not something argued
// into being true beforehand - the bounded-target argument only shows the
// SPECIFIC convergence mechanism that capped the other two forms doesn't
// apply the same way here, not that no other ceiling exists.
//
// Search scope: {dragAdhesionSatCap, dragAdhesionSatRate} alone (dragAdhesion
// Gain, dragAdhesionAdditiveGain, cpgGain, dopamine* all stay at 0 - isolate
// this new mechanism first, matching this project's discipline throughout).
// Neither parameter's scale was measured beforehand via a dedicated 'trace'
// mode (unlike dragAdhesionGain's own calibration) - ranges below are a
// generous first screen, anchored loosely on dragAdhesionAdditiveGain's own
// useful range (its best point was gain~100, and |u_k| is the same physical
// quantity here), not a precise pre-measurement. Confirm/narrow empirically.
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried. Fitness/reporting tracks the ACTUAL water/agar ratio -
// the target is 2-3x (Fang-Yen et al. 2010 / Vidal-Gadea et al. 2011), not
// merely crossing 1.0 (this project's own past mistake of stopping at "just
// past 100%" - see tests/worm_drag_adhesion_additive_calibration).
//
// RESULT: the bounded-cap hypothesis does NOT beat the unbounded additive
// form's ceiling - it re-discovers essentially the SAME ~1.02 ceiling via a
// different route, and a REFINED version of the mathematical argument
// explains why. First screen (satRate in {5,20,80}) found satRate had almost
// NO effect at any tested value - a red flag that these rates never actually
// reached saturation (1-exp(-rate*|u_k|) stayed in its small-argument,
// approximately-LINEAR regime for the |u_k| scale actually present, so this
// screen was unknowingly re-testing a rescaled version of the already-
// confirmed linear additive mechanism, not the genuinely bounded regime).
// Forcing TRUE saturation (satRate=5000, guaranteeing the exponential term
// sits at its plateau) at three cap values, 8 independent bases each:
// cap=150 -> ratio=1.020 (8/8 healthy, water>agar 8/8); cap=500 -> ratio=
// 1.020 (identical); cap=1500 -> ratio=1.011 - WORSE, not better. This is
// the key finding: the original hypothesis reasoned "agar's c_n_k
// approaches (40+cap), water's approaches (1.7+cap) - different constants,
// so no forced convergence" - true as stated, but incomplete: as cap grows
// large, the RATIO (40+cap)/(1.7+cap) itself approaches 1 (both media's
// saturated c_n_k become dominated by the same large cap, washing out the
// original 40-vs-1.7 difference in RELATIVE terms even though the absolute
// values differ) - the same fundamental "large boost erases the base c_n
// asymmetry" mechanism that capped the multiplicative and linear-additive
// forms, just manifesting through a different piece of the formula. Small-
// to-moderate cap (150-500) best preserves the original asymmetry and gives
// the best ratio found (1.020) - but that is barely different from the
// simple linear additive form's own best (1.019) at gain~100, because
// small-to-moderate cap combined with any satRate high enough to matter is,
// empirically, close enough to the linear form's own effective behavior to
// not be a materially different regime in practice.
//
// CONCLUSION: three structurally different functional forms for "boost c_n
// based on local shape-change velocity" - multiplicative (hard ceiling at
// EXACTLY 1.0, proven), linear additive (empirical ceiling ~1.02), bounded/
// saturating additive (same ~1.02 ceiling, confirmed via a different
// mechanism) - all three converge on essentially the same result. This is
// now good evidence that ~1.0-1.02 is a genuine structural ceiling for this
// entire CLASS of mechanism (locally modulating the RFT normal-drag
// coefficient as any function of instantaneous shape-change speed), not an
// artifact of one specific formula choice. Ships at dragAdhesionSatCap=0.0/
// dragAdhesionSatRate=0.0 (unchanged Params defaults). Reaching the real
// target (2-3x, Fang-Yen et al. 2010 / Vidal-Gadea et al. 2011) likely
// requires leaving this mechanism class entirely - see WORM.md section 6 for
// the dopamine/serotonin-style adaptive-state direction (also tried this
// session, also negative in its first implementation - modulating
// motorLeakScale - but not yet tried modulating the adhesion mechanisms
// themselves, with the explicit caveat there that this may not be free
// either).
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

using Candidate = std::array<float, 2>;  // {dragAdhesionSatCap, dragAdhesionSatRate}
enum { kCap = 0, kRate = 1 };
const Candidate kIdentity = {0.0f, 0.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.dragAdhesionSatCap = c[kCap];
    sim.params.dragAdhesionSatRate = c[kRate];
}

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
    // ./exe distribution cap rate [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kCap] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kRate] = static_cast<float>(std::atof(argv[3]));
        const int numBases = argc > 4 ? std::atoi(argv[4]) : 12;
        const int seedsPerBase = argc > 5 ? std::atoi(argv[5]) : 8;
        const int warmupSteps = argc > 6 ? std::atoi(argv[6]) : 300;
        const int measureSteps = argc > 7 ? std::atoi(argv[7]) : 2500;
        std::printf("cand=[satCap=%.4f satRate=%.4f] - %d bases x %d seeds\n", cand[kCap], cand[kRate], numBases,
                    seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0, bothHealthyCount = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumRatio = 0.0;
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

    // ./exe random <trials> <capLo> <capHi> <rateLo> <rateHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float capLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 1.0f;
        const float capHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 500.0f;
        const float rateLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 1.0f;
        const float rateHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 200.0f;
        const int seedsPerTrial = argc > 7 ? std::atoi(argv[7]) : 6;
        const unsigned rngSeed = argc > 8 ? static_cast<unsigned>(std::atoi(argv[8])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, satCap log-uniform [%.2f,%.2f], satRate log-uniform [%.2f,%.2f], "
                    "%d seeds/trial, rngSeed=%u\n",
                    trials, capLo, capHi, rateLo, rateHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> capLogDist(std::log(capLo), std::log(capHi));
        std::uniform_real_distribution<float> rateLogDist(std::log(rateLo), std::log(rateHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0;
        float bestRatio = -1.0f;
        Candidate bestCand{};
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kCap] = std::exp(capLogDist(rng));
            cand[kRate] = std::exp(rateLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio > bestRatio) { bestRatio = ratio; bestCand = cand; bestBase = base; }
            if (ratio > 1.0f) {
                ++foundOver1;
                if (ratio > 1.5f) ++foundOver15;
                if (ratio > 2.0f) ++foundOver2;
                std::printf("ratio=%.3f cap=%.3f rate=%.3f base=%d\n", ratio, cand[kCap], cand[kRate], base);
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0; %d with ratio>1.5; %d with ratio>2.0. Best "
                    "ratio: %.3f at cap=%.3f rate=%.3f base=%d\n",
                    foundOver1, trials, foundOver15, foundOver2, bestRatio < 0.0f ? 0.0f : bestRatio,
                    bestCand[kCap], bestCand[kRate], bestBase);
        std::printf("Each ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // Default: quick screen - satCap x satRate grid, one seed base.
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kCaps[] = {10.0f, 50.0f, 150.0f, 500.0f};
    const float kRates[] = {5.0f, 20.0f, 80.0f};
    std::printf("=== Screen (dragAdhesionSatCap x dragAdhesionSatRate grid, %d seeds/point, base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float cap : kCaps) {
        for (float rate : kRates) {
            const Candidate cand = {cap, rate};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("satCap=%.1f satRate=%.1f:\n", cap, rate);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  ratio water/agar: %s\n", bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)");
            std::fflush(stdout);
        }
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising point with "
                "'distribution' or search jointly with 'random' across many seed bases before believing it (see "
                "header).\n");
    return 0;
}
