// tests/worm_speed_leak_calibration/main.cpp
//
// Calibration search for per-architectural-type leak/capacitance scaling
// (Network::scale_type_params, same lever tests/worm_chemotaxis_calibration
// already uses for a DIFFERENT objective - see that file's header for the
// full rationale of why this is the right axis: all 401 neurons load with
// identical NeuronParams{leak=1, rest=0, capacitance=1}, no real per-type
// time constants, 7 free parameters not 401) - this time targeting the
// crawl/swim SPEED bug, not chemotaxis.
//
// Why THIS axis, after four other axes (direct bend-rate drag resistance,
// global DVA mechanoGain, local per-motor-neuron mechanosensation - see
// tests/worm_local_mechanosensation_calibration's RESULT section) all hit
// the same wall: none of those four touch the network's own intrinsic
// dynamics, only add forcing on top of it. The measured baseline oscillation
// period is ~125s (freqHz~0.008) on BOTH presets - a ~40-60x mismatch from
// real C. elegans (~2-3s crawl, ~0.5-0.6s swim), present even with zero
// external perturbation. Weak forcing gets absorbed by a strongly stable
// limit cycle (frequency doesn't move) or collapses it outright (freq->0);
// only changing the network's actual vector field (leak/capacitance, or the
// synapse-sign attempt that got reverted for a different reason - heading
// instability) can plausibly move the intrinsic period itself.
//
// THIS WAS ALREADY TRIED ONCE (see tests/worm_speed_calibration's "ROUND 1
// RESULT - REVERTED"): same 7-param lever, evolutionary search, single seed
// base per candidate during search - found a candidate that looked like a
// 4.85x speedup, reverted when 20 fresh independent bases showed identity
// and the "winning" candidate have IDENTICAL population-mean speed. That
// does NOT mean this axis is dead - it means (a) the search needs many
// independent bases DURING screening, not just at final confirmation, and
// (b) this file adds the heading-delta gate that Round 1 never had at all
// (found AFTER Round 1, from the separate synapse-sign incident - see
// WormSim.cpp ctor comment) - Round 1's "winning" candidate was never even
// checked for heading-reversal pathology.
//
// Health gate: efficiency, coiled ratio, freqHz (reject frozen-arc mode) -
// same as every calibration file in this project - PLUS max |heading delta|
// in one step, gated at kMaxHeadingDeltaRad (see below), same discipline as
// tests/worm_local_mechanosensation_calibration.
//
// Search strategy: bounded random search within a range, NOT the unbounded
// evolutionary walk Round 1 used (that walk reached capIP=20.0, capP=0.05,
// leakP past 15 - values with no grounding, chasing single-base fitness
// noise into an implausible corner). "random" mode below takes explicit
// min/max bounds from the caller (meant to be informed by real C. elegans
// time-constant literature/prior computational models, gathered separately -
// this file does not hardcode a "correct" range, since that number needs
// real research, not a guess baked into source).
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
constexpr float kMaxHeadingDeltaRad = 0.5f;  // see tests/worm_local_mechanosensation_calibration for rationale

constexpr int kNumParams = 7;
enum ParamIdx { kLeakIP = 0, kLeakP = 1, kLeakPO = 2, kCapIP = 3, kCapP = 4, kCapPO = 5, kCapO = 6 };
using Candidate = std::array<float, kNumParams>;
const Candidate kIdentity = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    connectome::Network& net = sim.network();
    net.scale_type_params(connectome::NeuronType::InputProcessing, c[kLeakIP], c[kCapIP]);
    net.scale_type_params(connectome::NeuronType::Processing, c[kLeakP], c[kCapP]);
    net.scale_type_params(connectome::NeuronType::ProcessingOutput, c[kLeakPO], c[kCapPO]);
    net.scale_type_params(connectome::NeuronType::Output, 1.0f, c[kCapO]);
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
                      int freqPosition) {
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
    std::printf("[leakIP=%.3f leakP=%.3f leakPO=%.3f capIP=%.3f capP=%.3f capPO=%.3f capO=%.3f]", c[0], c[1], c[2],
                c[3], c[4], c[5], c[6]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution leakIP leakP leakPO capIP capP capPO capO [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        for (int i = 0; i < kNumParams && argc > 2 + i; ++i) cand[i] = static_cast<float>(std::atof(argv[2 + i]));
        const int numBases = argc > 9 ? std::atoi(argv[9]) : 12;
        const int seedsPerBase = argc > 10 ? std::atoi(argv[10]) : 8;
        const int warmupSteps = argc > 11 ? std::atoi(argv[11]) : 300;
        const int measureSteps = argc > 12 ? std::atoi(argv[12]) : 2500;
        std::printf("cand="); printCand(cand); std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0;
        double sumAgar = 0.0, sumWater = 0.0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(cand, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(cand, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool waterFaster = waterOk && agarOk && water.meanBLps > agar.meanBLps;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  water>agar: %s\n", waterFaster ? "YES" : "no");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (waterFaster) ++waterBeatsAgarCount;
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s), water healthy=%d/%d (mean "
                    "%.5f BL/s), water>agar in %d/%d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    waterBeatsAgarCount, numBases);
        return 0;
    }

    // ./exe random <trials> <lo> <hi> [seedBase] [numSeeds]
    // Bounded random search: EVERY one of the 7 params drawn log-uniformly
    // from [lo, hi] (lo/hi given as a ratio around 1.0 = today's baseline,
    // e.g. "0.1 10" searches a 100x range symmetric in log-space). Multiple
    // INDEPENDENT seed bases per trial from the start (not one base like
    // Round 1) - numSeeds is seeds-per-base, and each trial draws a FRESH
    // base, so consecutive trials are never comparable on the same base
    // (deliberately - a candidate that only looks good on one lucky base
    // should not survive to be reported at all).
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float lo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.2f;
        const float hi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 5.0f;
        const int seedsPerTrial = argc > 5 ? std::atoi(argv[5]) : 6;
        const unsigned rngSeed = argc > 6 ? static_cast<unsigned>(std::atoi(argv[6])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, log-uniform [%.3f, %.3f], %d seeds/trial, rngSeed=%u\n", trials, lo,
                    hi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> logDist(std::log(lo), std::log(hi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            for (int i = 0; i < kNumParams; ++i) cand[i] = std::exp(logDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            const bool agarOk = isHealthy(agar);
            if (!agarOk) continue;  // cheap reject before spending the water run
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            const bool waterOk = isHealthy(water);
            const bool waterFaster = waterOk && water.meanBLps > agar.meanBLps;
            if (waterFaster) {
                ++found;
                printCand(cand);
                std::printf(" base=%d ", base);
                printAgg(" agar", agar);
                printAgg(" water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials had a healthy candidate with water>agar on its OWN (single, fresh) base - each "
                    "one still needs 'distribution' confirmation across many MORE independent bases before it "
                    "means anything (see header).\n",
                    found, trials);
        return 0;
    }

    std::printf("Usage:\n  %s random <trials> <lo> <hi> [seedsPerTrial] [rngSeed]\n  %s distribution leakIP leakP "
                "leakPO capIP capP capPO capO [numBases] [seedsPerBase] [warmup] [measure]\n",
                argv[0], argv[0]);
    return 0;
}
