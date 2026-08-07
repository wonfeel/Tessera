// tests/worm_speed_efficiency_calibration/main.cpp
//
// This session's calibration work so far (leak/capacitance recalibration,
// CPG+muscleBandwidthGain, proprioceptiveDelaySeconds) all targeted BEND
// FREQUENCY - and closed part of that gap (agar 0.0192->0.0283->0.0637Hz
// against a real ~6.3x-remaining gap to ~0.4Hz). But a direct comparison
// this session found the ABSOLUTE SPEED gap is much larger than the
// frequency gap: agar speed 0.00233 BL/s vs real ~0.1-0.2 BL/s is ~50-86x
// off, water 0.00805 BL/s vs real ~0.35-0.45 BL/s is ~44-56x off - i.e. even
// dividing out the ~6-17x frequency gap, there is a REMAINING ~4-10x deficit
// in distance covered PER BEND CYCLE (speed/freq): agar ~0.0366 BL/cycle
// here vs a real ~0.375 BL/cycle estimate (~10x short), water ~0.0722 vs
// ~0.216 (~3x short). This is a propulsion-EFFICIENCY problem, not a tempo
// problem - a different subsystem (WormBody::solve_propulsion's RFT force
// law), not the neural network.
//
// This file jointly searches the RFT force-law adhesion axis
// (dragAdhesionGain multiplicative, dragAdhesionAdditiveGain additive - both
// already implemented in WormBody, see body.hpp/.cpp, both extensively
// calibrated ALONE and against the OLD production stack earlier this
// project's history, never against TODAY's stack) TOGETHER with the already-
// shipped neural stack (cpgGain, muscleBandwidthGain, motorBandwidthGain,
// proprioceptiveDelaySeconds) - not holding the neural side fixed, per this
// session's now twice-confirmed lesson that fixed-companion screens miss
// real wins that only appear when everything is free to move together.
//
// Historical context on adhesion alone (all against the OLD, pre-this-
// session production point): dragAdhesionGain (multiplicative) raises
// ABSOLUTE water speed substantially (0.00054->0.00210 BL/s by gain=2) even
// though its ratio caps at exactly 1.0 as gain->infinity (both media
// converge to the same infinite-anisotropy kinematic limit - a proven, not
// empirical, ceiling). dragAdhesionAdditiveGain breaks that ceiling (ratio
// 1.017-1.019) but plateaus quickly. Today's shipped ratio (3.456) is well
// above the user's originally-cited 200-300% target, meaning there is now
// real headroom to trade SOME ratio margin for absolute-speed gains via the
// multiplicative form specifically - untested until now because it was
// never worth trying while ratio was still below target.
//
// Scoring: reward closeness to REAL ABSOLUTE SPEED targets (agar ~0.15 BL/s,
// water ~0.4 BL/s - literature midpoints, see tests/worm_speed_calibration's
// header for citations), not frequency this time. Health gate requires
// ratio>=1.3 (a real, deliberately-loosened-from-3.46 floor - enough margin
// below today's shipped ratio to allow trading some of it away, while still
// solidly inside the original 200-300% target and nowhere near 1.0).
//
// Health gate: efficiency>=0.40, coiledRatio>=0.30, freqHz>=0.001,
// maxAbsHeadingDelta<=0.5 rad - identical structure to every other axis this
// session. Multi-independent-seed-base distribution confirmation required
// before trusting any single-base hit.
//
// RESULT: NEGATIVE, and unusually decisive - 0/1600 across two independent
// search attempts, both run to completion cleanly (no infra failures).
//
// Broad attempt (1000 trials, cpgGain in the wide [0.5,8] range, other
// neural params similarly wide, adhesion params [0,20]/[0,300]): 800 trials
// completed cleanly (1 shard timed out, excluded, not counted as a
// negative), 0/800 passed health+ratio>=1.30.
//
// Tight-bracket retry (800 trials, ALL 4 shards completed cleanly): neural
// params held close to the ACTUAL shipped point (cpgGain [2.0,3.3],
// muscleBandwidthGain [0.0001,0.00025], motorBandwidthGain [0.012,0.028],
// delaySec [1.0,2.3] - all bracketing 2.621/0.000153/0.0194/1.596), adhesion
// params [0,15]/[0,250] - specifically testing "can adhesion be added to
// what's really in production," not a wandering neighborhood. STILL 0/800.
//
// Combined: 0/1600 clean trials, not a single healthy+ratio>=1.30 candidate
// found with ANY nonzero dragAdhesionGain or dragAdhesionAdditiveGain
// alongside the shipped neural stack, in either a wide or a point-anchored
// search. This is now the SIXTH data point on this project's "which
// mechanism pairs are compatible" question, and it lands cleanly on one
// side: EVERY attempt combining a force-law/physics-level mechanism
// (dragAdhesionGain/dragAdhesionAdditiveGain) with ANY neural-level tempo
// mechanism has failed (cpgGain+multiplicative, cpgGain+additive,
// muscleBandwidthGain+additive, and now this session's full stack+either
// form) - 4/4 attempts. Meanwhile EVERY attempt combining two NEURAL-level
// mechanisms with each other has succeeded when properly joint-searched
// (cpgGain+muscleBandwidthGain, then +proprioceptiveDelaySeconds) - 2/2.
// This is a cleaner, more specific pattern than this project's earlier,
// broader "any two mechanisms touching the loop destabilize it" hypothesis -
// the dividing line looks like NEURAL+NEURAL (compatible, when searched
// jointly) vs NEURAL+PHYSICS (never yet compatible, 4/4 failures).
//
// Consequence for the absolute-speed gap this file was built to attack: it
// remains unclosed by this specific route. The RFT force-law adhesion
// mechanisms, whatever their solo merit (dragAdhesionAdditiveGain alone was
// this project's first-ever healthy water>agar mechanism, see tests/worm_
// drag_adhesion_additive_calibration), cannot currently be combined with the
// neural stack that carries today's frequency gains. Closing the remaining
// ~4-10x per-cycle-efficiency deficit (see header) would need either a
// still-untried mechanism, or resolving WHY force-law and neural-tempo
// mechanisms specifically conflict (not attempted here - would likely need
// direct tracing of a failing combined trial, the same technique tests/
// worm_jerkiness_diagnostic used for the unrelated stability issue).
//
// NOT shipped - dragAdhesionGain/dragAdhesionAdditiveGain stay at their
// Params defaults (0.0/0.0).
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

constexpr float kCpgFreq = 1.108f, kCpgSens = 0.02f, kCpgAmpSens = 0.0342f, kPoseDecay = 1.051f;

// {cpgGain, muscleBandwidthGain, motorBandwidthGain, proprioceptiveDelaySeconds, dragAdhesionGain, dragAdhesionAdditiveGain}
using Candidate = std::array<float, 6>;
enum { kCpgGain = 0, kMuscleBw = 1, kMotorBw = 2, kDelay = 3, kAdhesionMult = 4, kAdhesionAdd = 5 };
const Candidate kShipped = {2.621f, 0.000153f, 0.0194f, 1.596f, 0.0f, 0.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.cpgGain = c[kCpgGain];
    sim.params.cpgBaseFreqHz = kCpgFreq;
    sim.params.cpgLoadSensitivity = kCpgSens;
    sim.params.cpgAmpLoadSensitivity = kCpgAmpSens;
    sim.params.cpgWavelengths = 1.0f;
    sim.params.bodyPoseDecayRate = kPoseDecay;
    sim.params.muscleBandwidthGain = c[kMuscleBw];
    sim.params.motorBandwidthGain = c[kMotorBw];
    sim.params.proprioceptiveDelaySeconds = c[kDelay];
    sim.params.dragAdhesionGain = c[kAdhesionMult];
    sim.params.dragAdhesionAdditiveGain = c[kAdhesionAdd];
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
constexpr float kMinRatioMargin = 1.3f;  // deliberately loosened from today's shipped 3.456 - real headroom to trade

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
    std::printf("[cpgGain=%.3f muscleBw=%.6f motorBw=%.5f delaySec=%.3f adhMult=%.3f adhAdd=%.2f]", c[kCpgGain],
                c[kMuscleBw], c[kMotorBw], c[kDelay], c[kAdhesionMult], c[kAdhesionAdd]);
}

// Real absolute-speed targets: agar ~0.15 BL/s, water ~0.4 BL/s.
float speedScore(const AggregateResult& agar, const AggregateResult& water) {
    return std::min(agar.meanBLps / 0.15f, 1.5f) * 10.0f + std::min(water.meanBLps / 0.4f, 1.5f) * 10.0f;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution cpgGain muscleBw motorBw delaySec adhMult adhAdd [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kShipped;
        for (int k = 0; k < 6 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 8 ? std::atoi(argv[8]) : 12;
        const int seedsPerBase = argc > 9 ? std::atoi(argv[9]) : 8;
        const int warmupSteps = argc > 10 ? std::atoi(argv[10]) : 300;
        const int measureSteps = argc > 11 ? std::atoi(argv[11]) : 2500;
        std::printf("cand="); printCand(cand);
        std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0, bothHealthyCount = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumRatio = 0.0, sumFreqAgar = 0.0, sumFreqWater = 0.0;
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
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; sumFreqAgar += agar.meanFreqHz; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; sumFreqWater += water.meanFreqHz; }
            if (bothOk) {
                ++bothHealthyCount;
                sumRatio += ratio;
                if (ratio > 1.0f) ++waterBeatsAgarCount;
            }
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s, %.4fHz), water healthy=%d/%d "
                    "(mean %.5f BL/s, %.4fHz), both-healthy=%d, mean ratio=%.3f, water>agar in %d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    agarHealthyCount ? sumFreqAgar / agarHealthyCount : 0.0, waterHealthyCount, numBases,
                    waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    waterHealthyCount ? sumFreqWater / waterHealthyCount : 0.0, bothHealthyCount,
                    bothHealthyCount ? sumRatio / bothHealthyCount : 0.0, waterBeatsAgarCount);
        return 0;
    }

    // ./exe random <trials> <cpgGainLo> <cpgGainHi> <muscleBwLo> <muscleBwHi> <motorBwLo> <motorBwHi> <delayLo> <delayHi> <adhMultLo> <adhMultHi> <adhAddLo> <adhAddHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float cpgGainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.5f;
        const float cpgGainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 8.0f;
        const float muscleBwLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.00005f;
        const float muscleBwHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 0.002f;
        const float motorBwLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.002f;
        const float motorBwHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 0.04f;
        const float delayLo = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.1f;
        const float delayHi = argc > 10 ? static_cast<float>(std::atof(argv[10])) : 3.0f;
        const float adhMultLo = argc > 11 ? static_cast<float>(std::atof(argv[11])) : 0.0f;
        const float adhMultHi = argc > 12 ? static_cast<float>(std::atof(argv[12])) : 20.0f;
        const float adhAddLo = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 0.0f;
        const float adhAddHi = argc > 14 ? static_cast<float>(std::atof(argv[14])) : 300.0f;
        const int seedsPerTrial = argc > 15 ? std::atoi(argv[15]) : 6;
        const unsigned rngSeed = argc > 16 ? static_cast<unsigned>(std::atoi(argv[16])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, cpgGain linear [%.2f,%.2f], muscleBw log-uniform [%.6f,%.6f], "
                    "motorBw log-uniform [%.5f,%.5f], delaySec linear [%.2f,%.2f], adhMult linear [%.2f,%.2f], "
                    "adhAdd linear [%.1f,%.1f], ratio>=%.2f required, %d seeds/trial, rngSeed=%u\n",
                    trials, cpgGainLo, cpgGainHi, muscleBwLo, muscleBwHi, motorBwLo, motorBwHi, delayLo, delayHi,
                    adhMultLo, adhMultHi, adhAddLo, adhAddHi, kMinRatioMargin, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> cpgGainDist(cpgGainLo, cpgGainHi);
        std::uniform_real_distribution<float> muscleBwLogDist(std::log(muscleBwLo), std::log(muscleBwHi));
        std::uniform_real_distribution<float> motorBwLogDist(std::log(motorBwLo), std::log(motorBwHi));
        std::uniform_real_distribution<float> delayDist(delayLo, delayHi);
        std::uniform_real_distribution<float> adhMultDist(adhMultLo, adhMultHi);
        std::uniform_real_distribution<float> adhAddDist(adhAddLo, adhAddHi);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundPassingGate = 0;
        float bestScore = -1e9f;
        Candidate bestCand{};
        int bestBase = 0;
        AggregateResult bestAgar, bestWater;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kCpgGain] = cpgGainDist(rng);
            cand[kMuscleBw] = std::exp(muscleBwLogDist(rng));
            cand[kMotorBw] = std::exp(motorBwLogDist(rng));
            cand[kDelay] = delayDist(rng);
            cand[kAdhesionMult] = adhMultDist(rng);
            cand[kAdhesionAdd] = adhAddDist(rng);
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio < kMinRatioMargin) continue;
            ++foundPassingGate;
            const float score = speedScore(agar, water);
            std::printf("PASS score=%.3f ratio=%.3f base=%d ", score, ratio, base);
            printCand(cand);
            std::printf("\n");
            printAgg("  agar", agar);
            printAgg("  water", water);
            std::fflush(stdout);
            if (score > bestScore) {
                bestScore = score; bestCand = cand; bestBase = base; bestAgar = agar; bestWater = water;
            }
        }
        std::printf("\n%d/%d trials passed health+ratio>=%.2f gate. Best score: %.3f", foundPassingGate, trials,
                    kMinRatioMargin, bestScore < -1e8f ? 0.0f : bestScore);
        if (foundPassingGate > 0) {
            std::printf(" at base=%d ratio=%.3f agarSpeed=%.5fBL/s waterSpeed=%.5fBL/s ", bestBase,
                        bestWater.meanBLps / bestAgar.meanBLps, bestAgar.meanBLps, bestWater.meanBLps);
            printCand(bestCand);
        }
        std::printf(
            "\nNOTHING above is trustworthy on a single base - EVERY passing candidate needs 'distribution' "
            "confirmation (16+ independent bases) before it means anything.\n");
        return 0;
    }

    std::printf("Usage:\n  %s random <trials> ...\n  %s distribution <cpgGain> <muscleBw> <motorBw> <delaySec> "
                "<adhMult> <adhAdd> ...\n", argv[0], argv[0]);
    return 0;
}
