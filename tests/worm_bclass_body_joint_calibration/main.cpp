// tests/worm_bclass_body_joint_calibration/main.cpp
//
// Follow-up to tests/worm_bclass_oscillator_calibration (axis 6, isolated):
// that search found the B-class active current (Network::set_active_current)
// is either too weak to move anything measurably, or - past a fairly sharp
// threshold - collapses the gait into a frozen static arc (freqHz->0), with
// NO accessible middle ground across 600 bounded-random trials + manual
// grid confirmation. This is not surprising in isolation - the axis's own
// design proposal explicitly flagged, as the single most likely failure
// mode, that the rest of the pipeline (WormBody's own persistence/decay
// constants) is tuned against the CURRENT slow regime and might not
// tolerate a faster upstream drive, citing a direct precedent already in
// this codebase (network.cpp's Output-leak=0 experiment: raised network
// frequency 3-6x as predicted, but crawling speed got ~28x WORSE, not
// better, because a faster neural signal alone doesn't help if the body
// can't turn it into coherent propulsive work).
//
// THIS FILE adds the third parameter that isolated search deliberately left
// out: WormBody::pose_decay_rate_ (body.hpp/.cpp - see set_pose_decay_rate),
// the rate at which a commanded bend relaxes back to neutral pose absent
// further drive. Previously a hardcoded 0.5 (per dt) constant; exposed here
// ONLY for this joint search, default unchanged. WARNING already on record
// (network.cpp's own history): naively speeding this up (shorter
// persistence) alongside a different frequency-raising experiment made
// things MUCH worse (efficiency 0.52 -> 0.06) - a commanded bend needs to
// PERSIST long enough for solve_propulsion to have something to push
// against. This search does not assume a direction (faster or slower decay)
// is correct - it searches both sides of the current 0.5 baseline.
//
// This is the most invasive axis attempted this session: it is the first
// search since the very first "TRIED AND REVERTED" experiment (see
// body.cpp's own header comment on WormBody::step) to touch WormBody's
// mechanics directly, rather than staying entirely within the neural
// network side. Extra scrutiny is warranted precisely because this is new,
// previously off-limits territory - see the health gate below, unchanged
// in strictness from every other axis, plus the same freqHz-separation
// diagnostic as the isolated B-class search.
//
// 3 free parameters: gain, tau_w (Network::set_active_current, same as the
// isolated search) + poseDecayRate (WormBody::set_pose_decay_rate, NEW).
// Search ranges:
//   gain          [0, 20]      - same as isolated search (see that file)
//   tau_w         [0.2, 40]s   - same as isolated search (see that file)
//   poseDecayRate [0.1, 2.0]   - bounded around today's hardcoded 0.5
//                                baseline (5x slower to 4x faster), NOT an
//                                open-ended search - no literature value
//                                exists for this constant (it has no direct
//                                biological referent, it's a numerical
//                                property of this specific reduced-body
//                                model), so the bound is an engineering
//                                judgment call, not a citation. Kept modest
//                                given the documented history of this
//                                exact constant misbehaving when pushed.
//
// Health gate: identical to every other axis this session - efficiency,
// coiledRatio, freqHz>0 (reject frozen arc), max heading-delta <=0.5 rad
// internal (tighter than tests/worm_locomotion's 3.2 rad hard limit) - baked
// in from the start of THIS search too, not assumed inherited from the
// isolated search's own validation.
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

using Candidate = std::array<float, 3>;  // {gain, tau_w, poseDecayRate}
enum { kGain = 0, kTauW = 1, kPoseDecay = 2 };
const Candidate kIdentity = {0.0f, 4.0f, 0.5f};  // gain=0 -> tau_w irrelevant; poseDecay=0.5 is today's hardcode

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.bClassOscillatorGain = c[kGain];
    sim.params.bClassOscillatorTauW = c[kTauW];
    sim.params.bodyPoseDecayRate = c[kPoseDecay];
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
    std::printf("[gain=%.4f tau_w=%.4f poseDecay=%.4f]", c[kGain], c[kTauW], c[kPoseDecay]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution gain tau_w poseDecay [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kTauW] = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) cand[kPoseDecay] = static_cast<float>(std::atof(argv[4]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 12;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 8;
        const int warmupSteps = argc > 7 ? std::atoi(argv[7]) : 300;
        const int measureSteps = argc > 8 ? std::atoi(argv[8]) : 2500;
        std::printf("cand="); printCand(cand); std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0, freqSeparatedCount = 0;
        double sumAgar = 0.0, sumWater = 0.0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(cand, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(cand, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool waterFaster = waterOk && agarOk && water.meanBLps > agar.meanBLps;
            const bool freqSeparated = waterOk && agarOk && water.meanFreqHz > agar.meanFreqHz;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  water>agar: %s   freq(water)>freq(agar): %s%s\n", waterFaster ? "YES" : "no",
                        freqSeparated ? "YES" : "no",
                        (waterFaster && !freqSeparated) ? "  <-- FLAG: speed up without freq separating" : "");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (waterFaster) ++waterBeatsAgarCount;
            if (freqSeparated) ++freqSeparatedCount;
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s), water healthy=%d/%d (mean "
                    "%.5f BL/s), water>agar in %d/%d bases, freq(water)>freq(agar) in %d/%d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    waterBeatsAgarCount, numBases, freqSeparatedCount, numBases);
        return 0;
    }

    // ./exe random <trials> <gainLo> <gainHi> <tauWLo> <tauWHi> <poseDecayLo> <poseDecayHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float gainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.01f;
        const float gainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 20.0f;
        const float tauWLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.2f;
        const float tauWHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 40.0f;
        const float poseDecayLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.1f;
        const float poseDecayHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 2.0f;
        const int seedsPerTrial = argc > 9 ? std::atoi(argv[9]) : 6;
        const unsigned rngSeed = argc > 10 ? static_cast<unsigned>(std::atoi(argv[10])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, gain[%.3f,%.3f] tau_w[%.3f,%.3f]s poseDecay[%.3f,%.3f], %d "
                    "seeds/trial, rngSeed=%u\n",
                    trials, gainLo, gainHi, tauWLo, tauWHi, poseDecayLo, poseDecayHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> gainLogDist(std::log(gainLo), std::log(gainHi));
        std::uniform_real_distribution<float> tauWLogDist(std::log(tauWLo), std::log(tauWHi));
        std::uniform_real_distribution<float> poseDecayLogDist(std::log(poseDecayLo), std::log(poseDecayHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kGain] = std::exp(gainLogDist(rng));
            cand[kTauW] = std::exp(tauWLogDist(rng));
            cand[kPoseDecay] = std::exp(poseDecayLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            const bool agarOk = isHealthy(agar);
            if (!agarOk) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            const bool waterOk = isHealthy(water);
            const bool waterFaster = waterOk && water.meanBLps > agar.meanBLps;
            if (waterFaster) {
                ++found;
                const bool freqSeparated = water.meanFreqHz > agar.meanFreqHz;
                printCand(cand);
                std::printf(" base=%d ", base);
                printAgg(" agar", agar);
                printAgg(" water", water);
                std::printf("  freqSeparated=%s\n", freqSeparated ? "yes" : "NO (flag)");
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials had a healthy candidate with water>agar on its OWN (single, fresh) base - each "
                    "one still needs 'distribution' confirmation across many MORE independent bases before it "
                    "means anything (see header).\n",
                    found, trials);
        return 0;
    }

    std::printf("Usage:\n  %s random <trials> <gainLo> <gainHi> <tauWLo> <tauWHi> <poseDecayLo> <poseDecayHi> "
                "[seedsPerTrial] [rngSeed]\n  %s distribution gain tau_w poseDecay [numBases] [seedsPerBase] "
                "[warmup] [measure]\n",
                argv[0], argv[0]);
    return 0;
}
