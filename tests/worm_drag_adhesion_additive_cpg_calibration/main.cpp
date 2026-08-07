// tests/worm_drag_adhesion_additive_cpg_calibration/main.cpp
//
// Joint search of Params::dragAdhesionAdditiveGain x the full CPG parameter
// set. Follows two confirmed findings, not a blind guess:
//
// 1. tests/worm_drag_adhesion_additive_calibration found the ADDITIVE force-
//    law fix (c_n_k = c_n + gain*|u_k|, vs the earlier MULTIPLICATIVE
//    c_n_k = c_n*(1+gain*|u_k|)) is the FIRST mechanism in this project's
//    entire history to get water/agar speed ratio PAST 1.0 - confirmed
//    robust across 16 independent bases (ratio 1.017-1.019, water>agar in
//    16/16) and across a 60-trial fine sweep of gain in [20,250] (60/60
//    ratio>1.0, but capped in a tight plateau ~1.014-1.019 - a real, if
//    small, ceiling of its own).
// 2. tests/worm_drag_adhesion_cpg_calibration combined the OLDER
//    multiplicative mechanism with CPG and found it did NOT meaningfully
//    move that mechanism's own ~0.98 ceiling (0/300, best 0.983) - because
//    the multiplicative form's ceiling comes from BOTH media converging to
//    the same infinite-anisotropy kinematic limit regardless of gain, and
//    (per that file's own screen) most adhesion+cpgGain combinations
//    together collapsed the gait entirely (freq=0, maxHeadDelta 1.5-3.1 rad)
//    rather than producing a healthy intermediate result.
//
// The hypothesis this file tests: the ADDITIVE mechanism's ceiling has a
// DIFFERENT mathematical origin worth re-examining with CPG in play. Unlike
// the multiplicative form, the additive form's finite-gain behavior does NOT
// depend on proportional scaling of the EXISTING c_n - it depends on how
// gain*|u_k| compares to c_n's absolute scale (1.7 water, 40 agar). CPG
// (tests/worm_cpg_calibration) makes |u_k| itself a genuine, causally real
// function of m_body.mechanical_load() (higher frequency + load-dependent
// amplitude in water vs agar, ~23x apart in raw load) - this could shift
// WHERE on the additive gain curve each medium sits, not just rescale both
// proportionally the way the multiplicative form's combination with CPG did.
// Whether this actually breaks the plateau or not is an open empirical
// question - the point of this file, not something argued into being true
// beforehand.
//
// Search scope: dragAdhesionAdditiveGain (this file's new axis, range from
// tests/worm_drag_adhesion_additive_calibration's own established plateau
// region, extended somewhat in case CPG shifts the optimum) jointly with the
// same 7 CPG-family parameters tests/worm_cpg_calibration and tests/worm_
// drag_adhesion_cpg_calibration both searched (cpgGain, cpgBaseFreqHz,
// cpgLoadSensitivity, cpgAmpLoadSensitivity, bodyPoseDecayRate,
// muscleLeakScale, motorLeakScale) - 8 total, ranges inherited from those
// two predecessor files' already-validated bounds. dragAdhesionGain
// (multiplicative) is NOT included here - tests/worm_drag_adhesion_cpg_
// calibration already showed it doesn't help combined with CPG and its own
// solo ceiling is lower than the additive form's, so including it would only
// add a dimension without a live hypothesis behind it.
//
// Fitness/reporting: same as every axis this session - the target is 2-3x
// (Fang-Yen et al. 2010 / Vidal-Gadea et al. 2011), not merely >1.0. Tracks
// the actual ratio for every healthy candidate and the single best found.
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried.
//
// RESULT: NEGATIVE - the hypothesis did not hold. Screen (additive x cpgGain
// grid, muscleLeak=0.2/motorLeak=20.0 fixed companions): same pathology as
// the multiplicative+CPG combination (tests/worm_drag_adhesion_cpg_
// calibration) - any nonzero additive combined with any nonzero cpgGain
// (5/15/30) collapsed the gait entirely (freq=0.0000Hz, maxHeadDelta up to
// 3.1 rad). 300-trial random search across the full 8-parameter space
// (additive linear [0,400], cpgGain linear [0,50], other 6 CPG-family params
// on established ranges): 0/300 healthy-on-both with ratio>1.0. Best ratio
// found: only 0.424 (cpgGain=2.14, so CPG genuinely active, not a gain~=0
// corner) - dramatically WORSE than tests/worm_drag_adhesion_additive_
// calibration's own solo result (1.017-1.019, confirmed on 16 bases).
// Conclusion: CPG does not merely fail to help the additive mechanism - it
// actively interferes with it, consistent with the same interference pattern
// seen combining CPG with the multiplicative mechanism. The load-responsive
// |u_k| asymmetry CPG introduces (this file's original hypothesis) is
// real, but its net effect on this specific force-law fix is destructive,
// not constructive - the hypothesis is falsified by this search, not merely
// unconfirmed. Ships at dragAdhesionAdditiveGain=0.0/cpgGain=0.0 (unchanged
// Params defaults). The additive mechanism should be used ALONE
// (cpgGain=0) if used at all - see tests/worm_drag_adhesion_additive_
// calibration for its solo result and honest remaining next steps.
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
constexpr float kFixedWavelengths = 1.0f;

// {dragAdhesionAdditiveGain, cpgGain, cpgBaseFreqHz, cpgLoadSensitivity, cpgAmpLoadSensitivity, bodyPoseDecayRate, muscleLeakScale, motorLeakScale}
using Candidate = std::array<float, 8>;
enum { kAdditive = 0, kCpgGain = 1, kCpgFreq = 2, kCpgSens = 3, kCpgAmpSens = 4, kPoseDecay = 5, kMuscleLeak = 6, kMotorLeak = 7 };
const Candidate kIdentity = {0.0f, 0.0f, 2.0f, 0.05f, 0.0f, 0.5f, 0.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.dragAdhesionAdditiveGain = c[kAdditive];
    sim.params.cpgGain = c[kCpgGain];
    sim.params.cpgBaseFreqHz = c[kCpgFreq];
    sim.params.cpgLoadSensitivity = c[kCpgSens];
    sim.params.cpgAmpLoadSensitivity = c[kCpgAmpSens];
    sim.params.cpgWavelengths = kFixedWavelengths;
    sim.params.bodyPoseDecayRate = c[kPoseDecay];
    sim.params.muscleLeakScale = c[kMuscleLeak];
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

void printCand(const Candidate& c) {
    std::printf("[additive=%.3f cpgGain=%.3f cpgFreq=%.3f cpgSens=%.3f cpgAmpSens=%.4f poseDecay=%.3f "
                "muscleLeak=%.3f motorLeak=%.3f]",
                c[kAdditive], c[kCpgGain], c[kCpgFreq], c[kCpgSens], c[kCpgAmpSens], c[kPoseDecay], c[kMuscleLeak],
                c[kMotorLeak]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution additive cpgGain cpgFreq cpgSens cpgAmpSens poseDecay muscleLeak motorLeak [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        for (int k = 0; k < 8 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 10 ? std::atoi(argv[10]) : 12;
        const int seedsPerBase = argc > 11 ? std::atoi(argv[11]) : 8;
        const int warmupSteps = argc > 12 ? std::atoi(argv[12]) : 300;
        const int measureSteps = argc > 13 ? std::atoi(argv[13]) : 2500;
        std::printf("cand="); printCand(cand);
        std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
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

    // ./exe random <trials> <addLo> <addHi> <cpgGainLo> <cpgGainHi> <freqLo> <freqHi> <sensLo> <sensHi> <ampSensLo> <ampSensHi> <decayLo> <decayHi> <leakLo> <leakHi> <motorLo> <motorHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float addLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const float addHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 400.0f;
        const float cpgGainLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.0f;
        const float cpgGainHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 50.0f;
        const float freqLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.5f;
        const float freqHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 4.0f;
        const float sensLo = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.005f;
        const float sensHi = argc > 10 ? static_cast<float>(std::atof(argv[10])) : 2.0f;
        const float ampSensLo = argc > 11 ? static_cast<float>(std::atof(argv[11])) : 0.001f;
        const float ampSensHi = argc > 12 ? static_cast<float>(std::atof(argv[12])) : 0.2f;
        const float decayLo = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 0.1f;
        const float decayHi = argc > 14 ? static_cast<float>(std::atof(argv[14])) : 2.0f;
        const float leakLo = argc > 15 ? static_cast<float>(std::atof(argv[15])) : 0.02f;
        const float leakHi = argc > 16 ? static_cast<float>(std::atof(argv[16])) : 3.0f;
        const float motorLo = argc > 17 ? static_cast<float>(std::atof(argv[17])) : 1.0f;
        const float motorHi = argc > 18 ? static_cast<float>(std::atof(argv[18])) : 100.0f;
        const int seedsPerTrial = argc > 19 ? std::atoi(argv[19]) : 6;
        const unsigned rngSeed = argc > 20 ? static_cast<unsigned>(std::atoi(argv[20])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, additive linear [%.2f,%.2f], cpgGain linear [%.2f,%.2f], freq "
                    "log-uniform [%.2f,%.2f]Hz, sens log-uniform [%.3f,%.3f], ampSens log-uniform [%.3f,%.3f], "
                    "poseDecay log-uniform [%.2f,%.2f], muscleLeak log-uniform [%.2f,%.2f], motorLeak log-uniform "
                    "[%.2f,%.2f], %d seeds/trial, rngSeed=%u\n",
                    trials, addLo, addHi, cpgGainLo, cpgGainHi, freqLo, freqHi, sensLo, sensHi, ampSensLo, ampSensHi,
                    decayLo, decayHi, leakLo, leakHi, motorLo, motorHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> addDist(addLo, addHi);
        std::uniform_real_distribution<float> cpgGainDist(cpgGainLo, cpgGainHi);
        std::uniform_real_distribution<float> freqLogDist(std::log(freqLo), std::log(freqHi));
        std::uniform_real_distribution<float> sensLogDist(std::log(sensLo), std::log(sensHi));
        std::uniform_real_distribution<float> ampSensLogDist(std::log(ampSensLo), std::log(ampSensHi));
        std::uniform_real_distribution<float> decayLogDist(std::log(decayLo), std::log(decayHi));
        std::uniform_real_distribution<float> leakLogDist(std::log(leakLo), std::log(leakHi));
        std::uniform_real_distribution<float> motorLogDist(std::log(motorLo), std::log(motorHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0;
        float bestRatio = -1.0f;
        Candidate bestCand{};
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kAdditive] = addDist(rng);
            cand[kCpgGain] = cpgGainDist(rng);
            cand[kCpgFreq] = std::exp(freqLogDist(rng));
            cand[kCpgSens] = std::exp(sensLogDist(rng));
            cand[kCpgAmpSens] = std::exp(ampSensLogDist(rng));
            cand[kPoseDecay] = std::exp(decayLogDist(rng));
            cand[kMuscleLeak] = std::exp(leakLogDist(rng));
            cand[kMotorLeak] = std::exp(motorLogDist(rng));
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
                std::printf("ratio=%.3f base=%d ", ratio, base);
                printCand(cand);
                std::printf("\n");
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0; %d with ratio>1.5; %d with ratio>2.0. Best "
                    "ratio found: %.3f",
                    foundOver1, trials, foundOver15, foundOver2, bestRatio < 0.0f ? 0.0f : bestRatio);
        if (bestRatio > 0.0f) {
            std::printf(" at base=%d ", bestBase);
            printCand(bestCand);
        }
        std::printf("\nEach ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // Default: quick screen - additive x cpgGain grid.
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kAdditives[] = {0.0f, 50.0f, 100.0f, 200.0f};
    const float kCpgGains[] = {0.0f, 5.0f, 15.0f, 30.0f};
    std::printf("=== Screen (additive x cpgGain grid, freq=2.0/sens=0.05/ampSens=0.02/poseDecay=0.5/"
                "muscleLeak=0.2/motorLeak=20.0 fixed, %d seeds/point, base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float a : kAdditives) {
        for (float g : kCpgGains) {
            const Candidate cand = {a, g, 2.0f, 0.05f, 0.02f, 0.5f, 0.2f, 20.0f};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("additive=%.1f cpgGain=%.1f:\n", a, g);
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
