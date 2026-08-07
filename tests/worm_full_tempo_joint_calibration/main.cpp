// tests/worm_full_tempo_joint_calibration/main.cpp
//
// FULL joint search across all seven tempo-related Params established this
// session: {cpgGain, cpgBaseFreqHz, cpgLoadSensitivity, cpgAmpLoadSensitivity,
// bodyPoseDecayRate, muscleBandwidthGain, motorBandwidthGain,
// proprioceptiveDelaySeconds}. By direct user request ("продолжай исправлять
// другие проблемы" after the jerkiness root-cause fix shipped) - the
// remaining known gap is absolute tempo (agar ~0.064Hz vs real ~0.3-0.5Hz,
// water ~0.112Hz vs real ~1.7-2Hz).
//
// Why run this NOW, given each of these seven was already searched at least
// once: every one of them was added SEQUENTIALLY on top of an anchor fixed
// from a PREVIOUS search - tests/worm_cpg_calibration found cpgGain/
// cpgBaseFreqHz/cpgLoadSensitivity/cpgAmpLoadSensitivity/bodyPoseDecayRate
// first (muscleBandwidthGain/motorBandwidthGain held at old low values);
// tests/worm_cpg_muscle_bandwidth_calibration then searched bandwidth
// jointly with CPG but held delaySec at 0; tests/worm_proprioceptive_delay_
// calibration then searched delaySec jointly with bandwidth but held
// cpgBaseFreqHz/cpgLoadSensitivity/cpgAmpLoadSensitivity fixed at their
// OLD values from the first search. This project's own established lesson,
// confirmed TWICE this session (leak/capacitance and proprioceptive delay
// both had null fixed-companion screens that hid a real win only visible
// once a companion was freed) - a genuinely joint search over ALL of them at
// once has never actually been run. This file is that search.
//
// propulsionVelocityClamp is NOT part of this search - it defaults to the
// now-shipped 500.0 (WormSim::Params default) via ordinary construction,
// unchanged for every trial - this file is purely about the neural/CPG
// tempo stack, not the jerkiness fix.
//
// Health gate identical to every other axis this session: efficiency>=0.40,
// coiledRatio>=0.30, freqHz>=0.001, maxAbsHeadingDelta<=0.5 rad, and
// ratio>=1.15 for the random-search gate. Score = frequency proximity to
// real targets (agar 0.4Hz cap, water 1.85Hz cap), same formula as tests/
// worm_proprioceptive_delay_calibration's freqScore.
//
// RESULT: NEGATIVE. Despite genuinely freeing all 8 parameters jointly
// (unlike every prior search on this general axis), no candidate found beats
// the sequentially-built shipped point.
//
// 8 parallel shards x 200 trials (1600 total, log/linear-uniform ranges
// centered on the shipped point but wide - see the random-mode distributions
// above): 5-20/200 candidates passed the health+ratio>=1.15 gate per shard
// (97 total) - a healthy hit rate, this was not a search that came up
// structurally empty. Top 3 candidates by score, ALL given full 16-base
// distribution confirmation:
//   - [cpgGain=3.733/cpgFreq=1.4/cpgSens=0.01/cpgAmpSens=0.1415/poseDecay=
//     1.248/muscleBw=0.000043/motorBw=0.04896/delaySec=0.345]: agar
//     healthy=16/16 (0.0671Hz, +5.3% vs shipped 0.0637Hz), water
//     healthy=15/16 (0.1315Hz, +17.4% vs shipped 0.1120Hz - but ONE base
//     unhealthy, not a clean win), ratio=1.633 (well below shipped's 3.523 -
//     and below the ~4-5x realistic range this session's research
//     established as the actual biological target). Both raw frequencies
//     improved, but at the cost of health margin and a much less realistic
//     ratio - not a clean improvement.
//   - [cpgGain=3.322/cpgFreq=1.239/cpgSens=0.011/cpgAmpSens=0.0435/
//     poseDecay=1.126/muscleBw=0.000157/motorBw=0.00295/delaySec=0.122]:
//     16/16 both media, but agar 0.0505Hz (-20.7%) AND water 0.0950Hz
//     (-15.2%) - strictly worse than shipped on both frequencies.
//   - [cpgGain=5.513/cpgFreq=1.699/cpgSens=0.0174/cpgAmpSens=0.0052/
//     poseDecay=1.639/muscleBw=0.00044/motorBw=0.01316/delaySec=0.053]:
//     16/16 both media, agar 0.0721Hz (+13.2%, a real agar-only gain) but
//     water 0.0828Hz (-26.1%) - trades water tempo for agar tempo, no net
//     win.
//
// None of the three confirmed candidates beats the shipped point on BOTH
// media's frequency while staying fully healthy at a realistic ratio - each
// either sacrifices health/ratio for raw Hz, or is a lateral trade (better
// on one medium, worse on the other), or is strictly worse on both. Unlike
// the leak/capacitance and proprioceptive-delay axes (where a genuinely
// joint search DID find a real win after fixed-companion screens looked
// null), this joint search's own null result is not itself suspicious - it
// explored a wide range around the shipped point (a healthy 6% pass rate
// found real, distinct local optima nearby) and none of them dominate.
// Honest interpretation: the current shipped point (built sequentially, one
// axis added at a time) appears to already sit at or near a real joint
// local optimum of this 8-parameter tempo family, at least within the
// ranges searched here - this is NOT proof no better point exists anywhere
// (ranges were centered on the shipped point, not a from-scratch global
// search), but it does mean the specific "sequential search missed a joint
// win" concern that motivated this file is NOT what's happening on this
// axis. NOT SHIPPED - Params unchanged.
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

// {cpgGain, cpgBaseFreqHz, cpgLoadSensitivity, cpgAmpLoadSensitivity, bodyPoseDecayRate, muscleBandwidthGain, motorBandwidthGain, proprioceptiveDelaySeconds}
using Candidate = std::array<float, 8>;
enum { kCpgGain = 0, kCpgFreq = 1, kCpgSens = 2, kCpgAmpSens = 3, kPoseDecay = 4, kMuscleBw = 5, kMotorBw = 6, kDelay = 7 };

const Candidate kShipped = {2.621f, 1.108f, 0.02f, 0.0342f, 1.051f, 0.000153f, 0.0194f, 1.596f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.cpgGain = c[kCpgGain];
    sim.params.cpgBaseFreqHz = c[kCpgFreq];
    sim.params.cpgLoadSensitivity = c[kCpgSens];
    sim.params.cpgAmpLoadSensitivity = c[kCpgAmpSens];
    sim.params.cpgWavelengths = 1.0f;
    sim.params.bodyPoseDecayRate = c[kPoseDecay];
    sim.params.muscleBandwidthGain = c[kMuscleBw];
    sim.params.motorBandwidthGain = c[kMotorBw];
    sim.params.proprioceptiveDelaySeconds = c[kDelay];
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
constexpr float kMinRatioMargin = 1.15f;

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
    std::printf("[cpgGain=%.3f cpgFreq=%.3f cpgSens=%.4f cpgAmpSens=%.4f poseDecay=%.3f muscleBw=%.6f motorBw=%.5f delaySec=%.3f]",
                c[kCpgGain], c[kCpgFreq], c[kCpgSens], c[kCpgAmpSens], c[kPoseDecay], c[kMuscleBw], c[kMotorBw], c[kDelay]);
}

float freqScore(const AggregateResult& agar, const AggregateResult& water) {
    return std::min(agar.meanFreqHz / 0.4f, 1.5f) * 10.0f + std::min(water.meanFreqHz / 1.85f, 1.5f) * 10.0f;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution cpgGain cpgFreq cpgSens cpgAmpSens poseDecay muscleBw motorBw delaySec [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kShipped;
        for (int k = 0; k < 8 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 10 ? std::atoi(argv[10]) : 12;
        const int seedsPerBase = argc > 11 ? std::atoi(argv[11]) : 8;
        const int warmupSteps = argc > 12 ? std::atoi(argv[12]) : 300;
        const int measureSteps = argc > 13 ? std::atoi(argv[13]) : 2500;
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
            if (bothOk) { ++bothHealthyCount; sumRatio += ratio; if (ratio > 1.0f) ++waterBeatsAgarCount; }
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

    // ./exe random <trials> [seedsPerTrial] [rngSeed]
    // Ranges centered on the shipped point, wide enough to let the search
    // move away from it if a better joint combination exists.
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const int seedsPerTrial = argc > 3 ? std::atoi(argv[3]) : 6;
        const unsigned rngSeed = argc > 4 ? static_cast<unsigned>(std::atoi(argv[4])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random joint search: %d trials, %d seeds/trial, rngSeed=%u\n", trials, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> cpgGainDist(0.5f, 6.0f);
        std::uniform_real_distribution<float> cpgFreqDist(0.3f, 2.0f);
        std::uniform_real_distribution<float> cpgSensLogDist(std::log(0.005f), std::log(0.1f));
        std::uniform_real_distribution<float> cpgAmpSensDist(0.0f, 0.15f);
        std::uniform_real_distribution<float> poseDecayDist(0.3f, 3.0f);
        std::uniform_real_distribution<float> muscleBwLogDist(std::log(0.00002f), std::log(0.002f));
        std::uniform_real_distribution<float> motorBwLogDist(std::log(0.002f), std::log(0.06f));
        std::uniform_real_distribution<float> delayLogDist(std::log(0.05f), std::log(4.0f));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundPassingGate = 0;
        float bestScore = -1e9f;
        Candidate bestCand{};
        int bestBase = 0;
        AggregateResult bestAgar, bestWater;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kCpgGain] = cpgGainDist(rng);
            cand[kCpgFreq] = cpgFreqDist(rng);
            cand[kCpgSens] = std::exp(cpgSensLogDist(rng));
            cand[kCpgAmpSens] = cpgAmpSensDist(rng);
            cand[kPoseDecay] = poseDecayDist(rng);
            cand[kMuscleBw] = std::exp(muscleBwLogDist(rng));
            cand[kMotorBw] = std::exp(motorBwLogDist(rng));
            cand[kDelay] = std::exp(delayLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio < kMinRatioMargin) continue;
            ++foundPassingGate;
            const float score = freqScore(agar, water);
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
            std::printf(" at base=%d ratio=%.3f agarFreq=%.4fHz waterFreq=%.4fHz ", bestBase,
                        bestWater.meanBLps / bestAgar.meanBLps, bestAgar.meanFreqHz, bestWater.meanFreqHz);
            printCand(bestCand);
        }
        std::printf(
            "\nNOTHING above is trustworthy on a single base - EVERY passing candidate needs 'distribution' "
            "confirmation (16+ independent bases) before it means anything.\n");
        return 0;
    }

    std::printf("Usage: %s [distribution|random] ...\n", argv[0]);
    return 0;
}
