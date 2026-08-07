// tests/worm_muscle_body_joint_calibration/main.cpp
//
// Direct retry of the crawl/swim speed bug (see tests/worm_speed_calibration,
// worm_speed_leak_calibration, worm_synapse_speed_calibration,
// worm_bclass_oscillator_calibration, worm_bclass_body_joint_calibration for
// axes 1-6, all either insufficient or reverted - 0/480, 0/400, 0/600, 0/600
// respectively across those four alone). This axis is different from all of
// them: it is not a new additive term (active current, mechanosensation) or
// a rescaling of an already-reachable coefficient (leak_scale_, synapse
// sign) - it targets the ARCHITECTURAL leak=0 on Output/muscle neurons that
// tests/worm_network_eigenmodes identified as the cause of the network's
// slowest collective mode (~755s), which no prior leak/capacitance axis
// could touch (Network::step forced Output's leak to exactly 0 regardless
// of leak_scale_/scale_type_params - see network.hpp's class comment).
//
// THE HISTORICAL PRECEDENT THIS FILE RETRIES, PROPERLY: an earlier,
// ISOLATED experiment (see "TRIED CHANGING, REVERTED" at Network::step in
// network.cpp) removed Output's leak=0 special case entirely (equivalent to
// muscleLeakScale=1.0 here) with NOTHING else changed. Result: the network's
// oscillation frequency rose 3-4x (~0.03-0.05Hz -> ~0.15-0.18Hz) exactly as
// the eigenmode analysis predicted, but real crawling speed collapsed ~28x
// (0.0108 -> 0.00039 BL/s, confirmed across 15 independent seed bases) - the
// body could not turn a faster neural signal into a coherent propulsive
// wave. A FOLLOW-UP attempt to compensate by ALSO speeding up WormBody's own
// angle-decay time constant (bodyPoseDecayRate here, network.cpp's own
// account) made it MUCH worse still (efficiency 0.52 -> 0.06) - tried as a
// reactive patch AFTER the network side had already been changed and found
// broken, not as part of one joint search from the start. The explicit
// conclusion at the time: "a real fix would need to understand and preserve
// [the network-body] coupling while speeding both sides up together, not
// tested here." This file is that untested attempt - muscleLeakScale
// (Network::set_muscle_leak) and bodyPoseDecayRate (WormBody::
// set_pose_decay_rate, already exposed for the bClass joint search) are
// BOTH free parameters of ONE bounded, multi-seed-base search from the very
// first fitness call - not a network change followed by a reactive body
// patch.
//
// Isolated from every other experimental axis this session (bClassOscillator,
// pdf1, localMechanoGain, serotonin all left at their shipped 0.0/inert
// defaults) - any effect found here is attributable to this axis alone.
//
// Search ranges (bounded, literature/precedent-grounded, not an unconstrained
// walk):
//   muscleLeakScale   in [0.02, 3.0]   - no literature unit (this reduced
//                        model's own leak scale, not a real membrane time
//                        constant); upper bound anchored to the historical
//                        precedent's own value (1.0, i.e. "same treatment as
//                        every other neuron type") - search a bit past it in
//                        case joint tuning with the body pushes the healthy
//                        ceiling further, without wandering unboundedly.
//   bodyPoseDecayRate in [0.1, 2.0]    - identical range already used and
//                        justified in tests/worm_bclass_body_joint_
//                        calibration for the same physical parameter.
//
// Health gate: efficiency>=0.40, coiledRatio>=0.30, freqHz>kMinFreqHz (reject
// frozen-static-arc), maxAbsHeadingDelta<=kMaxHeadingDeltaRad - identical
// structure to worm_bclass_oscillator_calibration, baked in from the START.
// Primary fitness: waterBLps > agarBLps with BOTH healthy, not raw speed on
// either alone. freqHz tracked separately for agar/water too (freqSeparated
// flag) - the mechanistic claim is frequency itself should rise in the
// lower-drag medium (Fang-Yen et al. 2010), so a candidate that speeds up
// water without frequency separating the same way is flagged as more likely
// a geometry/coiling artifact than a real reproduction of the mechanism.
//
// Multi-independent-seed-base evaluation from the first fitness call, this
// project's repeatedly-learned lesson, not bolted on at confirmation.
//
// RESULT: 0/400 - the joint retry does NOT fix the crawl/swim ordering
// either. Three separate checks, all negative:
//   (a) muscleLeakScale ALONE (bodyPoseDecayRate held at its old fixed 0.5,
//       9-point grid, one seed base): frequency rises exactly as the
//       eigenmode analysis and the historical isolated experiment predicted
//       - 0.008Hz at leak=0.02 up to 0.309Hz at leak=3.0, a ~40x range,
//       cleanly monotonic. But water>agar: "no" at EVERY single point
//       tested, no exceptions - and water's absolute speed actively
//       DECLINES as leak rises past ~0.2 (0.00096 -> 0.00003 BL/s) while
//       agar's peaks around leak=0.2 (0.00970 BL/s, healthiest point in the
//       whole grid) then also declines. Water breaks health first (leak>=1.0)
//       while agar stays healthy across the entire range - the same
//       asymmetric water-side fragility seen on other axes this session.
//   (b) FULL joint bounded random search, muscleLeakScale x
//       bodyPoseDecayRate log-uniform over [0.02,3.0]x[0.1,2.0], 400 trials,
//       fresh single seed base + 6 seeds each: 0/400 healthy water>agar
//       candidates.
//   (c) The single most mechanistically-motivated corner - HIGH muscleLeak
//       (2.5, fast network) paired with LOW bodyPoseDecayRate (0.12, bends
//       persist ~4x longer than default, on the theory that slower decay
//       might let a faster network's commands still do propulsive work
//       before being replaced) - checked directly across 10 independent
//       bases. Result: NOT merely "not water>agar" but unhealthy on BOTH
//       media simultaneously, consistently (efficiency 0.18-0.27 agar,
//       0.06-0.09 water, both far under the 0.40 gate, all 10 bases). This
//       directly REFUTES the "let bends persist longer to match a faster
//       network" hypothesis, rather than just failing to confirm it - slow
//       decay + fast leak produces confused, low-efficiency motion on BOTH
//       media about equally, not a fix skewed toward water.
// CONCLUSION: this was the biggest still-untried lever from this session's
// entire investigation of the crawl/swim bug (the first PROPER joint
// network+body retiming, as opposed to the historical sequential/reactive
// attempt) and it does not work either. muscleLeakScale ships at 0.0
// (confirmed bitwise-identical via tests/worm_locomotion). Combined with
// every other axis this session and before (leak/capacitance 0/480,
// leak/capacitance+synapse-sign 0/400, bClassOscillator alone 0/600,
// bClassOscillator+bodyPoseDecayRate 0/600, localMechanoGain real-but-
// inert-for-this-purpose, synapse-sign-alone reverted for heading
// pathology), the crawl/swim speed-parity problem has now been attacked
// from essentially every honest-emergent-dynamics angle this project's own
// architecture affords, without a working candidate. This is not a
// weaker attempt than the others - it targeted the SPECIFIC, independently
// diagnosed (eigenmode analysis) bottleneck, tested properly (jointly, not
// reactively), and still came back empty. Whoever continues this should
// treat the honest, no-hand-fed-rhythm version of this problem as
// extremely likely to require either accepting the current behavior as a
// documented limitation (consistent with the state of the published
// literature - see WORM.md's status section) or relaxing the
// full-connectome-emergence constraint itself for the rhythm-generating
// part specifically (a real trade-off, not something to decide unilaterally
// here).
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

using Candidate = std::array<float, 2>;  // {muscleLeakScale, bodyPoseDecayRate}
enum { kMuscleLeak = 0, kPoseDecay = 1 };
const Candidate kIdentity = {0.0f, 0.5f};  // bitwise-identical to shipped defaults

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.muscleLeakScale = c[kMuscleLeak];
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
    // ./exe distribution muscleLeakScale bodyPoseDecayRate [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kMuscleLeak] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kPoseDecay] = static_cast<float>(std::atof(argv[3]));
        const int numBases = argc > 4 ? std::atoi(argv[4]) : 12;
        const int seedsPerBase = argc > 5 ? std::atoi(argv[5]) : 8;
        const int warmupSteps = argc > 6 ? std::atoi(argv[6]) : 300;
        const int measureSteps = argc > 7 ? std::atoi(argv[7]) : 2500;
        std::printf("cand=[muscleLeak=%.4f poseDecay=%.4f] - %d bases x %d seeds\n", cand[kMuscleLeak],
                    cand[kPoseDecay], numBases, seedsPerBase);
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

    // ./exe random <trials> <muscleLeakLo> <muscleLeakHi> <poseDecayLo> <poseDecayHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float leakLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.02f;
        const float leakHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 3.0f;
        const float decayLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.1f;
        const float decayHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 2.0f;
        const int seedsPerTrial = argc > 7 ? std::atoi(argv[7]) : 6;
        const unsigned rngSeed = argc > 8 ? static_cast<unsigned>(std::atoi(argv[8])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, muscleLeak log-uniform [%.3f,%.3f], poseDecay log-uniform "
                    "[%.3f,%.3f], %d seeds/trial, rngSeed=%u\n",
                    trials, leakLo, leakHi, decayLo, decayHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> leakLogDist(std::log(leakLo), std::log(leakHi));
        std::uniform_real_distribution<float> decayLogDist(std::log(decayLo), std::log(decayHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kMuscleLeak] = std::exp(leakLogDist(rng));
            cand[kPoseDecay] = std::exp(decayLogDist(rng));
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
                std::printf("[muscleLeak=%.4f poseDecay=%.4f] base=%d ", cand[kMuscleLeak], cand[kPoseDecay], base);
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

    // Default: screen a grid on one seed base (shortlist only - confirm
    // anything promising with 'distribution'/'random' across many bases
    // before trusting it, per the header and this project's own repeated
    // lesson on that). bodyPoseDecayRate held at its shipped default (0.5)
    // for this first pass - just characterizing muscleLeakScale's own effect
    // before searching the joint space.
    const float kLeakScales[] = {0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f};
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    std::printf("=== Screen (muscleLeakScale grid, bodyPoseDecayRate=0.5 fixed, %d seeds/point, base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float leak : kLeakScales) {
        const Candidate cand = {leak, 0.5f};
        const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
        const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
        std::printf("muscleLeak=%.3f:\n", leak);
        printAgg("agar", agar);
        printAgg("water", water);
        const bool waterFaster = isHealthy(agar) && isHealthy(water) && water.meanBLps > agar.meanBLps;
        std::printf("  water>agar: %s\n", waterFaster ? "YES" : "no");
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising point with "
                "'distribution <muscleLeak> <poseDecay>' or search jointly with 'random' across many seed bases "
                "before believing it (see header).\n");
    return 0;
}
