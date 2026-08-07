// tests/worm_dopamine_tone_calibration/main.cpp
//
// Calibration for Params::dopamineToneTau/dopamineMotorLeakGain (WormSim.h/
// .cpp) - the SECOND deliberate exception to "behavior only from the real
// connectome network" in this project (after cpgGain, tests/worm_cpg_
// calibration), added at the user's explicit request after the dedicated
// force-law axis (tests/worm_drag_adhesion_additive_calibration) found a
// genuine, confirmed, but small (~1.8%) water>agar margin and hit its own
// plateau - the user's direct instruction was "push for 200-300%, not just
// 100%, and do it as honestly as possible."
//
// Motivation: FIVE independent axes this project tried (DVA-global
// mechanosensation, local per-segment mechanosensation, per-class leak/
// capacitance, synapse-sign, proprioceptiveGain itself - see WORM.md section
// 6) all searched for ONE FIXED CONSTANT that must work equally well on both
// agar and water simultaneously, and all failed identically. Real
// C. elegans does not solve this with a fixed constant either - Vidal-Gadea
// et al. 2011 (PNAS 108:17504) found substrate contact activates
// dopaminergic ADE/PDE neurons (via D1-like receptors), promoting the crawl
// gait; this is a STATE, not a constant - a slow-acting neuromodulator whose
// level itself tracks the sensed environment, unlike a fixed synaptic
// weight. This file tests whether letting motorLeakScale be modulated by
// such a slow, load-tracking state (rather than searched as one fixed value
// for both media) can do better than any of the five failed fixed-constant
// axes, or the CPG (fixed-shape external rhythm), or the drag-adhesion
// mechanisms (fixed-value force-law tweaks).
//
// Mechanism (WormSim.cpp::applyDopamineDrive, called after m_body.step(),
// same one-step-lagged feedback convention as applyProprioception/
// applyMechanosensation/applyRhythmGenerator):
//   normalizedLoad = m_body.mechanical_load() / dragNormal   (same signal
//     already driving DVA/mechanoGain/CPG)
//   m_dopamineTone += (normalizedLoad - m_dopamineTone) * (dt/dopamineToneTau)
//   effectiveMotorLeak = motorLeakScale * (1 + dopamineMotorLeakGain * tone)
// dopamineGain (real signal into ADEL/ADER/PDEL/PDER) is DELIBERATELY NOT
// part of this search - it only affects connectome-fidelity/visualization
// (those neurons are not wired further into the motor path in this
// reduction), not the measured outcome - see Params::dopamineGain's own
// comment for the honest caveat.
//
// Search scope: {dopamineToneTau, dopamineMotorLeakGain, motorLeakScale} -
// motorLeakScale is included as a free BASE value (not fixed at its 1.0
// identity) because the modulation multiplies it - a base far from 1.0 changes
// what range of effective leak the tone modulation can reach. cpgGain and
// both drag-adhesion gains stay at 0 - isolate this NEW mechanism first,
// same "isolate before joint" discipline as every other axis (tests/worm_
// bclass_oscillator_calibration before tests/worm_bclass_body_joint_
// calibration, tests/worm_cpg_calibration's own added companions, etc.).
//   dopamineToneTau in [1, 200]s log-uniform - NOT calibrated against a
//     measured scale beforehand (unlike dragAdhesionGain, which got a
//     'trace' mode first) - the natural timescale for this state is
//     unknown a priori, this range spans "barely slower than a bend cycle"
//     to "much slower than the whole measurement window" and lets the
//     search itself find where (if anywhere) it matters.
//   dopamineMotorLeakGain in [-5, 5] linear (signed - direction not
//     predicted by literature for this specific parameterization; the real
//     biology says HIGH load -> MORE dopamine tone -> promotes crawl mode,
//     which by analogy suggests gain>0 should MATCH agar's own already-high
//     leak needs and gain<0 would fight it, but this reduced model's
//     motorLeakScale is not a literal 1:1 map to "crawl-promoting", so the
//     sign is left for the search, same discipline as cpgAmpLoadSensitivity
///    and dragAdhesionAdditiveGain's sign before it).
//   motorLeakScale in [0.1, 50] log-uniform - centered around its own
//     established useful range from tests/worm_cpg_calibration (that file's
//     screen used 20.0 as a companion value with real effect).
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried. Fitness/reporting tracks the ACTUAL water/agar ratio,
// same as every axis since the user's "push for 200-300%" instruction - not
// merely a >1 boolean.
//
// RESULT: NEGATIVE. Screen (dopamineMotorLeakGain x motorLeakScale grid,
// tau=20.0 fixed) - even the baseline dopGain=0/motorLeak=1.0 point is
// healthy (ratio=0.234, matching every other file's baseline), but almost
// every OTHER grid point is unhealthy: motorLeak=5 or 20 (away from the 1.0
// identity) freezes the gait (freq=0.0000Hz) even at dopGain=0; any nonzero
// dopGain combined with motorLeak=1.0 ALSO freezes it; dopGain<0 is
// catastrophic (speed=0, large single-step heading jumps even very early in
// the run). 300-trial random search across the full range (tau log-uniform
// [1,200]s, dopGain linear [-5,5], motorLeak log-uniform [0.1,50]): 0/300
// healthy-on-both with ratio>1.0 - the single best HEALTHY candidate found
// was only 0.175 (tau=1.85s, dopGain=0.199, motorLeak=0.130), WORSE than the
// do-nothing baseline (0.234). Conclusion: modulating motorLeakScale by a
// slow load-tracking state does not help, and the search couldn't even find
// a healthy region that matches doing nothing - motorLeakScale away from its
// 1.0 identity destabilizes this connectome's already-fragile intrinsic
// oscillation (same "rigid regime" property documented in tests/worm_local_
// mechanosensation_calibration) UNLESS it's specifically paired with the CPG
// actually driving a rhythm through it (as in tests/worm_cpg_calibration,
// where motorLeakScale~20 was a necessary but not sufficient companion) -
// modulating it in isolation, even slowly/adaptively, does not sidestep that
// fragility. Ships at dopamineToneTau=20.0/dopamineMotorLeakGain=0.0/
// dopamineGain=0.0 (all Params defaults, unchanged). The ADEL/ADER/PDEL/PDER
// signal injection (dopamineGain) itself is untested as a driver of
// anything - it was deliberately scoped out of this search (see header) -
// so this result specifically falsifies "slow-adaptive motorLeakScale
// modulation", not the broader idea of a dopamine-like state per se. An
// honest, not-yet-tried variant: apply the same slow tone to
// dragAdhesionAdditiveGain (the one mechanism that IS confirmed to help)
// instead of motorLeakScale - though note this has its own tension worth
// stating up front: real capillary adhesion should physically be STRONGEST
// on agar (high load) and near-absent in bulk water, but tests/worm_drag_
// adhesion_additive_calibration's benefit specifically comes from applying
// the SAME finite additive term to BOTH media (water's small c_n amplifies
// it disproportionately) - a state that raises gain on agar and lowers it in
// water would likely undo the very asymmetry that made the mechanism work,
// so this is not a free win to assume, only an untested one.
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

using Candidate = std::array<float, 3>;  // {dopamineToneTau, dopamineMotorLeakGain, motorLeakScale}
enum { kTau = 0, kDopGain = 1, kMotorLeak = 2 };
const Candidate kIdentity = {20.0f, 0.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.dopamineToneTau = c[kTau];
    sim.params.dopamineMotorLeakGain = c[kDopGain];
    sim.params.motorLeakScale = c[kMotorLeak];
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
    // ./exe distribution tau dopGain motorLeak [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kTau] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kDopGain] = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) cand[kMotorLeak] = static_cast<float>(std::atof(argv[4]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 12;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 8;
        const int warmupSteps = argc > 7 ? std::atoi(argv[7]) : 300;
        const int measureSteps = argc > 8 ? std::atoi(argv[8]) : 2500;
        std::printf("cand=[tau=%.3f dopGain=%.3f motorLeak=%.3f] - %d bases x %d seeds\n", cand[kTau], cand[kDopGain],
                    cand[kMotorLeak], numBases, seedsPerBase);
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

    // ./exe random <trials> <tauLo> <tauHi> <dopGainLo> <dopGainHi> <leakLo> <leakHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float tauLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 1.0f;
        const float tauHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 200.0f;
        const float dopGainLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : -5.0f;
        const float dopGainHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 5.0f;
        const float leakLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.1f;
        const float leakHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 50.0f;
        const int seedsPerTrial = argc > 9 ? std::atoi(argv[9]) : 6;
        const unsigned rngSeed = argc > 10 ? static_cast<unsigned>(std::atoi(argv[10])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, tau log-uniform [%.2f,%.2f]s, dopGain linear [%.2f,%.2f], "
                    "motorLeak log-uniform [%.2f,%.2f], %d seeds/trial, rngSeed=%u\n",
                    trials, tauLo, tauHi, dopGainLo, dopGainHi, leakLo, leakHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> tauLogDist(std::log(tauLo), std::log(tauHi));
        std::uniform_real_distribution<float> dopGainDist(dopGainLo, dopGainHi);
        std::uniform_real_distribution<float> leakLogDist(std::log(leakLo), std::log(leakHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0;
        float bestRatio = -1.0f;
        Candidate bestCand{};
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kTau] = std::exp(tauLogDist(rng));
            cand[kDopGain] = dopGainDist(rng);
            cand[kMotorLeak] = std::exp(leakLogDist(rng));
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
                std::printf("ratio=%.3f tau=%.3f dopGain=%.3f motorLeak=%.3f base=%d\n", ratio, cand[kTau],
                            cand[kDopGain], cand[kMotorLeak], base);
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0; %d with ratio>1.5; %d with ratio>2.0. Best "
                    "ratio: %.3f at tau=%.3f dopGain=%.3f motorLeak=%.3f base=%d\n",
                    foundOver1, trials, foundOver15, foundOver2, bestRatio < 0.0f ? 0.0f : bestRatio,
                    bestCand[kTau], bestCand[kDopGain], bestCand[kMotorLeak], bestBase);
        std::printf("Each ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // Default: quick screen - dopamineMotorLeakGain x motorLeakScale grid,
    // tau fixed at identity default (20s), one seed base (shortlist only).
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kDopGains[] = {0.0f, 1.0f, -1.0f, 3.0f, -3.0f};
    const float kMotorLeaks[] = {1.0f, 5.0f, 20.0f};
    std::printf("=== Screen (dopamineMotorLeakGain x motorLeakScale grid, tau=20.0 fixed, %d seeds/point, "
                "base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float dg : kDopGains) {
        for (float ml : kMotorLeaks) {
            const Candidate cand = {20.0f, dg, ml};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("dopGain=%.1f motorLeak=%.1f:\n", dg, ml);
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
