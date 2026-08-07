// tests/worm_muscle_bandwidth_calibration/main.cpp
//
// Calibration for Params::muscleBandwidthGain/motorBandwidthGain (WormSim.h/
// .cpp) - a mechanism designed to address BOTH open problems this project's
// history has treated as separate axes: absolute tempo (bend frequency) and
// water/agar propulsion-ratio, at once, by targeting what tests/worm_
// network_eigenmodes actually diagnosed as the root cause of the tempo
// problem - not a missing feedback loop, but a bandwidth bottleneck in an
// EXISTING one.
//
// Diagnosis (already established, not new to this file): the network's
// slowest collective eigenmode is ~755s, 100% muscle (Output) energy -
// architectural leak=0 there means the ONLY restoring force is weak gap-
// junction diffusion between muscles. Directly raising muscle leak to a
// FIXED value (long before this session) DID raise frequency (~0.03-0.05Hz)
// but made crawling speed 28x WORSE, because the leak was raised uniformly
// for both media with no corresponding body-side retiming. cpgGain (tests/
// worm_cpg_calibration) sidesteps this with an EXTERNAL, body-state-blind
// forced rhythm injected via add_input directly into motor neurons - gets
// realistic frequency, but 0/300 on the water>agar ratio, AND collapses the
// gait catastrophically when combined with either drag-adhesion mechanism
// (tests/worm_drag_adhesion_cpg_calibration, tests/worm_drag_adhesion_
// additive_cpg_calibration) - hypothesis: two independent, body-state-blind
// signal sources fighting over the same motor-neuron input channel.
//
// This mechanism is deliberately NOT a third blind injection. It widens the
// bandwidth of the EXISTING, already-honest proprioceptive feedback channel
// (applyProprioception, Boyle/Berri/Cohen 2012 - already generates a
// traveling wave emergently) in proportion to REALLY sensed load
// (normalizedLoad, same signal as mechanoGain/CPG), INSTANTANEOUSLY (not via
// a slow EMA tone - tests/worm_dopamine_tone_calibration's dopamine
// MotorLeakGain used a 1-200s tau and got 0/300, worse than baseline,
// plausibly because the tone was an order of magnitude too slow to track the
// bend cycle it was meant to help). No new signal is injected into any
// neuron - only how much of the EXISTING signal survives the muscle/motor
// relay changes. Because it never writes to the motor-neuron input channel
// (unlike CPG) and never touches the RFT force law (unlike either drag-
// adhesion mechanism), it is architecturally orthogonal to both -
// interference is not guaranteed the way it apparently was for CPG+adhesion,
// though this is a hypothesis this file's later joint search (if warranted)
// would need to test, not something assumed true up front.
//
// Mechanism (WormSim.cpp::step, right before net.set_muscle_leak/
// set_motor_leak_scale): normalizedLoad read from m_body.mechanical_load()
// BEFORE this step's m_body.step() call (one-step-lagged, same convention as
// every other feedback path in this file):
//   effectiveMuscleLeak = muscleLeakScale + muscleBandwidthGain*normalizedLoad
//   effectiveMotorLeak  = motorLeakScale * (1 + motorBandwidthGain*normalizedLoad)
// muscleLeakScale/motorLeakScale stay at their PRODUCTION defaults (0.0/1.0)
// in this search - at exactly zero load the effective values are therefore
// unchanged from today's shipped behavior; bandwidth only grows with sensed
// load, which is the entire point.
//
// Search scope: {muscleBandwidthGain, motorBandwidthGain, bodyPoseDecayRate}
// - bodyPoseDecayRate is included from the FIRST search, not added later
// after a failure - the 28x-worse precedent this file's header cites is
// specifically "raised leak without a body-side companion", and this project
// already learned (tests/worm_cpg_calibration) not to repeat that mistake a
// third time. cpgGain, both drag-adhesion gains, and dopamine* all stay at
// their Params defaults (0) - isolate this new mechanism first.
//   muscleBandwidthGain in [0.0001, 0.1] log-uniform - normalizedLoad is
//     ~70-80 on agar, ~3 in water (established this session via cpgLoad
//     Sensitivity's own calibration) - this range lets muscleLeakScale reach
//     roughly [0.007,8] on agar and [0.0003,0.3] in water, spanning well
//     below and above the ~0.2 value tests/worm_cpg_calibration's own screen
//     found necessary for a signal to pass through the muscle relay at all.
//   motorBandwidthGain in [0.001, 2.0] log-uniform - anchored so the
//     (1+gain*load) multiplier can reach roughly motorLeakScale's own
//     established useful value (~20) at agar's load scale (motorBandwidthGain
//     ~0.25 gives ~20x at normalizedLoad~76).
//   bodyPoseDecayRate in [0.1, 2.0] log-uniform - identical range already
//     used and justified in every joint-timing search this project has run.
//
// Health gate: identical structure/thresholds to every other axis. Tracks
// the ACTUAL water/agar ratio AND frequency (this mechanism's whole point is
// tempo, not just ratio) - the target is BOTH a realistic frequency (Fang-
// Yen et al. 2010: crawl ~0.5Hz, swim ~1.7-2Hz) AND ratio 2-3x, not either
// alone.
//
// RESULT: BEST RATIO OF THIS PROJECT'S ENTIRE HISTORY, confirmed robust -
// but the frequency/tempo half of the goal this mechanism was designed for
// was NOT achieved, and it does NOT combine with the drag-adhesion
// mechanism (third independent confirmation of the same "two feedback-loop-
// touching mechanisms fight" pathology already seen twice with cpgGain).
//
// Screen (poseDecay fixed at 0.5): frequency DID rise substantially (0.012-
// 0.054Hz vs the ~0.005-0.01Hz baseline - the mechanism demonstrably widens
// bandwidth as designed) but almost the entire grid was unhealthy, in TWO
// distinct failure modes: low muscleBandwidthGain (0.001-0.02) produced wild
// single-step heading swings (maxHeadDelta 0.3-0.85 rad, some past this
// project's own 0.5 gate); high muscleBandwidthGain (0.05) produced the
// opposite pathology - good coiled ratio and low heading delta, but
// efficiency collapsed to 0.05-0.15 (worm circling in place, not making net
// progress). poseDecay was fixed at 0.5 for this screen - not yet part of
// the joint search.
//
// Random search #1 (300 trials, wide ranges, poseDecay in [0.1,2.0]): only
// 1/300 healthy with ratio>1.0 (1.044), at muscleBandwidthGain=0.00013 - near
// the very FLOOR of the tested range - and poseDecay=1.911, near the CEILING
// of the tested range. CONFIRMED via distribution (16 independent bases x 8
// seeds): 16/16 healthy, water>agar in ALL 16, mean ratio=1.139 - real and
// robust, not a single-base illusion, but frequency stayed low (agar
// 0.0033Hz, water 0.0114Hz) - this specific point does NOT solve tempo,
// despite the mechanism's own screen showing it CAN raise frequency
// elsewhere in the space (just not healthily, at this poseDecay).
//
// Random search #2 (300 trials, poseDecay extended to [0.5,8.0] to test
// whether even higher pose-decay unlocks more of the space): did NOT
// improve on search #1 - again only 1/300, best ratio 1.023 - WORSE than
// search #1's finding. The healthy-and-improved region is a narrow, hard-to-
// randomly-hit pocket, not something a wider blind range search finds more
// of.
//
// Random search #3 (150 trials, TIGHT range bracketing search #1's finding -
// muscleBandwidthGain in [0.00008,0.0003], motorBandwidthGain in
// [0.006,0.015], poseDecay in [1.3,2.8]): dramatically higher hit rate -
// 12/150 (vs 1/300 and 1/300 for the wider searches) - confirming this is a
// real, correctly-located pocket, not noise. Best ratio 1.269. CONFIRMED via
// distribution (16 independent bases x 8 seeds) at muscleBandwidthGain=
// 0.00012, motorBandwidthGain=0.0064, poseDecay=2.198: 16/16 healthy,
// water>agar in ALL 16, ratios tightly clustered 1.15-1.29, MEAN RATIO=1.232
// (23.2% water>agar margin) - more than 12x tests/worm_drag_adhesion_
// additive_calibration's confirmed 1.8% margin, and the best confirmed
// result in this project's entire history. Frequency at this point: agar
// 0.0024Hz, water 0.0120Hz - still far from realistic (Fang-Yen et al. 2010:
// ~0.5Hz/~1.7-2Hz) - the ratio improvement did NOT come with a tempo fix,
// contrary to this mechanism's original design goal of solving both at once.
//
// Combination attempt with dragAdhesionAdditiveGain (both mechanisms at
// their own independently-confirmed-best point, 8 independent bases): 0/8
// HEALTHY - complete collapse on every single base, the same catastrophic
// pattern already seen twice combining cpgGain with either drag-adhesion
// form. This DISPROVES the specific hypothesis stated in this file's header
// and in Params::muscleBandwidthGain's comment - that avoiding a shared
// motor-neuron WRITE channel (unlike cpgGain, which writes there directly)
// would make this mechanism architecturally orthogonal to drag-adhesion.
// It does not: both mechanisms sit inside the SAME closed loop
// (proprioception -> curvature -> body physics -> motion -> proprioception),
// and perturbing that loop from two independently-tuned directions at once
// is apparently enough to destabilize it regardless of which specific state
// variables each mechanism directly writes. Three independent pairings now
// show this same pattern (cpgGain+dragAdhesionGain; cpgGain+
// dragAdhesionAdditiveGain; muscleBandwidthGain/motorBandwidthGain+
// dragAdhesionAdditiveGain) - this looks like a genuine structural property
// of this specific connectome's narrow healthy operating window, not
// something specific to any one pairing.
//
// CONCLUSION: ships at muscleBandwidthGain=0.0/motorBandwidthGain=0.0
// (unchanged Params defaults) despite the strong, confirmed, healthy solo
// result - shipping a changed default is a decision left to the project
// owner (see WORM.md section 6), not made autonomously, consistent with
// every other confirmed-but-unshipped finding this session. If pursued
// further: the mechanism as designed was meant to solve tempo AND ratio
// together, and the confirmed-best point solves only ratio - a finer local
// search specifically biased toward higher frequency (rather than best raw
// ratio) at nearby points in this same pocket is untried and might reveal a
// point that does both, given the screen already showed frequency CAN rise
// substantially somewhere in this parameter family.
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

using Candidate = std::array<float, 3>;  // {muscleBandwidthGain, motorBandwidthGain, bodyPoseDecayRate}
enum { kMuscleBw = 0, kMotorBw = 1, kPoseDecay = 2 };
const Candidate kIdentity = {0.0f, 0.0f, 0.5f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.muscleBandwidthGain = c[kMuscleBw];
    sim.params.motorBandwidthGain = c[kMotorBw];
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
    // ./exe distribution muscleBw motorBw poseDecay [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kMuscleBw] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kMotorBw] = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) cand[kPoseDecay] = static_cast<float>(std::atof(argv[4]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 12;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 8;
        const int warmupSteps = argc > 7 ? std::atoi(argv[7]) : 300;
        const int measureSteps = argc > 8 ? std::atoi(argv[8]) : 2500;
        std::printf("cand=[muscleBw=%.5f motorBw=%.4f poseDecay=%.3f] - %d bases x %d seeds\n", cand[kMuscleBw],
                    cand[kMotorBw], cand[kPoseDecay], numBases, seedsPerBase);
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

    // ./exe random <trials> <muscleBwLo> <muscleBwHi> <motorBwLo> <motorBwHi> <decayLo> <decayHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float muscleBwLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0001f;
        const float muscleBwHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 0.1f;
        const float motorBwLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.001f;
        const float motorBwHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 2.0f;
        const float decayLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.1f;
        const float decayHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 2.0f;
        const int seedsPerTrial = argc > 9 ? std::atoi(argv[9]) : 6;
        const unsigned rngSeed = argc > 10 ? static_cast<unsigned>(std::atoi(argv[10])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, muscleBw log-uniform [%.5f,%.5f], motorBw log-uniform [%.4f,%.4f], "
                    "poseDecay log-uniform [%.2f,%.2f], %d seeds/trial, rngSeed=%u\n",
                    trials, muscleBwLo, muscleBwHi, motorBwLo, motorBwHi, decayLo, decayHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> muscleBwLogDist(std::log(muscleBwLo), std::log(muscleBwHi));
        std::uniform_real_distribution<float> motorBwLogDist(std::log(motorBwLo), std::log(motorBwHi));
        std::uniform_real_distribution<float> decayLogDist(std::log(decayLo), std::log(decayHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0, foundRealisticFreq = 0;
        float bestRatio = -1.0f;
        Candidate bestCand{};
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kMuscleBw] = std::exp(muscleBwLogDist(rng));
            cand[kMotorBw] = std::exp(motorBwLogDist(rng));
            cand[kPoseDecay] = std::exp(decayLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio > bestRatio) { bestRatio = ratio; bestCand = cand; bestBase = base; }
            const bool realisticFreq = agar.meanFreqHz > 0.05f && water.meanFreqHz > 0.05f;
            if (realisticFreq) ++foundRealisticFreq;
            if (ratio > 1.0f) {
                ++foundOver1;
                if (ratio > 1.5f) ++foundOver15;
                if (ratio > 2.0f) ++foundOver2;
                std::printf("ratio=%.3f muscleBw=%.5f motorBw=%.4f poseDecay=%.3f base=%d\n", ratio, cand[kMuscleBw],
                            cand[kMotorBw], cand[kPoseDecay], base);
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0; %d with ratio>1.5; %d with ratio>2.0; %d with "
                    "BOTH media freq>0.05Hz (realistic-ish tempo). Best ratio: %.3f at muscleBw=%.5f motorBw=%.4f "
                    "poseDecay=%.3f base=%d\n",
                    foundOver1, trials, foundOver15, foundOver2, foundRealisticFreq, bestRatio < 0.0f ? 0.0f : bestRatio,
                    bestCand[kMuscleBw], bestCand[kMotorBw], bestCand[kPoseDecay], bestBase);
        std::printf("Each ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // Default: quick screen - muscleBandwidthGain x motorBandwidthGain grid,
    // poseDecay fixed at identity default (0.5).
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kMuscleBws[] = {0.001f, 0.005f, 0.02f, 0.05f};
    const float kMotorBws[] = {0.01f, 0.1f, 0.3f, 1.0f};
    std::printf("=== Screen (muscleBandwidthGain x motorBandwidthGain grid, poseDecay=0.5 fixed, %d seeds/point, "
                "base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float mb : kMuscleBws) {
        for (float tb : kMotorBws) {
            const Candidate cand = {mb, tb, 0.5f};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("muscleBw=%.3f motorBw=%.2f:\n", mb, tb);
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
