// tests/worm_muscle_calcium_calibration/main.cpp
//
// Calibration for Network::muscle_calcium_tau_ / Params::muscleCalciumTau
// (network.hpp/.cpp, WormSim.h/.cpp) - the first genuinely ARCHITECTURAL
// change on the tempo axis this session, not another rescaling of the
// existing linear network. Every prior lever (cpgGain, muscleBandwidthGain/
// motorBandwidthGain, scale_type_params leak/capacitance, historically
// scale_synapse_sign, bClassOscillatorGain) rescales a coefficient of the
// SAME dV/dt = -leak*(V-rest) + ... linear system per neuron - none of them
// gave the network a genuinely different kind of attractor to have (direct
// quote from tests/worm_bclass_oscillator_calibration's own header). After
// three further searches this session on cpgGain/muscleBandwidthGain itself
// (0/720 high-cpgGain, 0/600 mid-range, both null) and a fourth attempt at
// leak/capacitance (this session's one real but modest win, +47% agar tempo)
// all failed to meaningfully close the ~14-21x tempo gap vs real
// C. elegans, the user explicitly asked for an architecture change.
//
// THE NEW LEVER: Output ("muscle") neurons used to have their state V read
// DIRECTLY as the mechanical drive signal (WormSim.cpp: curvature computed
// straight from net.state(id) for Output neurons) - a single leaky-
// integrator variable serving BOTH as "how fast the network computes" and
// "how fast the muscle mechanically responds", the same two questions every
// prior axis was forced to conflate into one knob. Real C. elegans muscle
// activation is a genuine two-time-scale cascade (fast electrical/synaptic
// depolarization, then a slower intracellular calcium transient that
// actually drives contraction - standard excitation-contraction coupling,
// not a C. elegans-specific claim). Network::muscle_calcium_tau_ adds a
// SECOND state Ca_i per Output neuron, relaxing toward V_i with its own time
// constant: dCa_i/dt = (V_i - Ca_i)/tau. WormSim now reads muscle_output(id)
// (Ca_i if tau>0, else V_i directly - see network.hpp) instead of state(id).
// At tau=0 (still every Params default outside this file) this is BITWISE
// the old behavior - verified via tests/worm_locomotion run 5x post-change,
// identical coiled-ratio range (0.693-0.705) to the pre-change baseline
// (0.617-0.705).
//
// The hypothesis this file tests: with the calcium stage smoothing the
// ACTUAL mechanical output, V itself might be allowed to run much faster
// (higher cpgGain than the 1.901 currently shipped, or even the 5-45 range
// three prior searches this session found always broke the ratio) without
// the muscle/body producing an incoherent, twitchy drive - because the fast
// part now stays upstream of Ca, and only Ca (smoothed) reaches curvature.
// Whether this is actually true is exactly what this file finds out, not
// assumed.
//
// Search scope: {cpgGain, muscleBandwidthGain, motorBandwidthGain,
// muscleCalciumTau} - 4 free params. cpgBaseFreqHz/cpgLoadSensitivity/
// cpgAmpLoadSensitivity/bodyPoseDecayRate held FIXED at their already-
// confirmed shipped values (1.108/0.02/0.0342/1.051) - already searched
// extensively this session, re-opening them here would just re-explore
// ground already covered without adding a dimension that matters for THIS
// hypothesis specifically.
//   cpgGain in [0, 45] linear - DELIBERATELY includes the full range,
//     including the 5-45 region three prior searches this session found
//     ALWAYS inverts the ratio in the old (no-calcium) architecture - this
//     is the whole point of the hypothesis being tested.
//   muscleBandwidthGain in [0.00005, 0.002] log-uniform, motorBandwidthGain
//     in [0.002, 0.03] log-uniform - centered on/around the already-shipped
//     values with headroom, since the calcium stage might shift where the
//     optimum sits.
//   muscleCalciumTau in [0.05, 20] log-uniform - no literature-derived value
//     for this reduced model's own time constant (real muscle Ca2+
//     transients are ~0.1-1s, but this is a lumped 24-position/95-muscle
//     reduction, not a 1:1 anatomical model, so that number is a loose
//     anchor, not a hard bound) - lower bound keeps dt/tau meaningfully <1
//     for a genuine fast/slow split (same derivation as bClassOscillatorTauW
//     elsewhere in this project), upper bound is comparable to this
//     project's other slow time constants (dopamineToneTau, pdf1ReleaseTau).
//
// Health gate: efficiency>=0.40, coiledRatio>=0.30, freqHz>=0.001,
// maxAbsHeadingDelta<=0.5 rad - identical structure to every other axis this
// session, baked in from trial 1. Multi-independent-seed-base distribution
// confirmation required before trusting any single-base hit (this project's
// own repeatedly-learned lesson, most recently re-applied this session in
// tests/worm_leak_capacitance_tempo_calibration).
//
// RESULT: NEGATIVE - the architecture is real (verified: bitwise-identical
// to pre-change behavior at tau=0, tests/worm_locomotion re-run 5x matching
// the pre-change coiled-ratio range exactly) but the hypothesis it was built
// to test did not hold.
//
// Screen A (calciumTau alone at the shipped cpgGain=1.901 point): small tau
// (0.1-0.3) is roughly neutral to mildly positive on ratio (0.30 -> 3.078
// vs shipped 2.910) but does NOT improve frequency - agar frequency actually
// dropped slightly (0.012Hz vs 0.017Hz). tau>=1 actively hurts, water
// frequency specifically (an expected low-pass-filter signature - water
// needs the HIGHEST frequency component, so a slow filter costs water most).
// tau>=20 freezes agar entirely (over-damped, freq->0, unhealthy).
//
// Screen B (the actual hypothesis - does calcium smoothing let a much
// higher, previously-always-broken cpgGain of 5/15/30 work, muscleBw/
// motorBw held at shipped values): NO. At cpgGain>=5 with calciumTau=0, agar
// fails health via LOW EFFICIENCY (0.24-0.35, below the 0.40 floor) - not
// via heading instability, a DIFFERENT failure mode than the one this
// mechanism targets. Adding calciumTau does not fix this - it either leaves
// agar unhealthy, or shifts the "healthy" window down to near-zero frequency
// (cpgGain=5/tau=2: agar freq collapses to 0.003Hz), or over-damps into a
// frozen arc (tau=6: freq=0). No calciumTau value at any tested high cpgGain
// produced a healthy, well-balanced candidate.
//
// Full joint search (1000 trials, cpgGain/muscleBandwidthGain/
// motorBandwidthGain/muscleCalciumTau all free, cpgGain range [0,45] full):
// confirms Screen B was not a fixed-value artifact - 13/1000 cleared the
// health+ratio>=1.15 gate, and EVERY ONE clustered at LOW cpgGain
// (1.6-2.8, the same neighborhood as the already-shipped 1.901), none in the
// 5-45 "realistic tempo" region. Top 4 candidates by frequency-closeness
// score, all confirmed 16/16 healthy via distribution: NONE beat the
// already-shipped point (agar 0.0283Hz/water 0.1232Hz/ratio 2.908) - best
// frequency match (cand1: agar 0.0238Hz/water 0.0975Hz/ratio 2.328) was
// worse on every axis; best ratio (cand4: ratio 3.140) cost more than half
// the shipped point's agar frequency (0.0126Hz).
//
// Diagnosis (why the hypothesis failed): the efficiency collapse at high
// cpgGain happens even BEFORE considering whether the muscle can mechanically
// keep up - the body's motion becomes genuinely incoherent at the network
// level, not merely "twitchy." This points to the actual bottleneck being
// UPSTREAM of the muscle relay: this model's real locomotion-generating
// mechanism is the proprioceptive traveling wave (applyProprioception,
// Boyle/Berri/Cohen 2012), which has its own natural frequency set by the
// network's connectivity/gains - forcing a much faster external rhythm via
// cpgGain onto that loop breaks the WAVE's coherence itself, before the
// signal ever reaches the muscle stage this file's new architecture can
// smooth. A downstream muscle filter cannot fix upstream pattern-generation
// incoherence. Any further attempt on tempo would need to target the
// proprioceptive loop's own dynamics (proprioceptiveGain/Offset, the
// window/delay structure of the wave) directly, not the muscle relay -
// a different, not-yet-attempted architectural direction, out of scope here.
//
// NOT shipped - Params::muscleCalciumTau stays at 0.0 (bitwise old
// behavior). The architecture itself is kept in the codebase (inert at
// default, real infrastructure, not dead code) in case a future attempt at
// the proprioceptive-loop direction above wants to combine with it.
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

// Already-shipped, extensively-confirmed CPG side params - fixed, not searched here.
constexpr float kCpgFreq = 1.108f, kCpgSens = 0.02f, kCpgAmpSens = 0.0342f, kPoseDecay = 1.051f;

// {cpgGain, muscleBandwidthGain, motorBandwidthGain, muscleCalciumTau}
using Candidate = std::array<float, 4>;
enum { kCpgGain = 0, kMuscleBw = 1, kMotorBw = 2, kCalciumTau = 3 };
// Today's shipped point, calcium off (tau=0 -> bitwise old muscle_output behavior).
const Candidate kShippedNoCalcium = {1.901f, 0.000249f, 0.00449f, 0.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.cpgGain = c[kCpgGain];
    sim.params.cpgBaseFreqHz = kCpgFreq;
    sim.params.cpgLoadSensitivity = kCpgSens;
    sim.params.cpgAmpLoadSensitivity = kCpgAmpSens;
    sim.params.cpgWavelengths = 1.0f;
    sim.params.bodyPoseDecayRate = kPoseDecay;
    sim.params.muscleBandwidthGain = c[kMuscleBw];
    sim.params.motorBandwidthGain = c[kMotorBw];
    sim.params.muscleCalciumTau = c[kCalciumTau];
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
constexpr float kMinRatioMargin = 1.15f;  // require a real margin, not knife-edge >1.0 (lesson from today's middle-ground search)

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
    std::printf("[cpgGain=%.3f muscleBw=%.6f motorBw=%.5f calciumTau=%.3f]", c[kCpgGain], c[kMuscleBw], c[kMotorBw],
                c[kCalciumTau]);
}

// Real targets: crawl ~0.4Hz, swim ~1.85Hz - reward closeness to (or past) target.
float freqScore(const AggregateResult& agar, const AggregateResult& water) {
    return std::min(agar.meanFreqHz / 0.4f, 1.5f) * 10.0f + std::min(water.meanFreqHz / 1.85f, 1.5f) * 10.0f;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution cpgGain muscleBw motorBw calciumTau [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kShippedNoCalcium;
        for (int k = 0; k < 4 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 6 ? std::atoi(argv[6]) : 12;
        const int seedsPerBase = argc > 7 ? std::atoi(argv[7]) : 8;
        const int warmupSteps = argc > 8 ? std::atoi(argv[8]) : 300;
        const int measureSteps = argc > 9 ? std::atoi(argv[9]) : 2500;
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

    // ./exe random <trials> <cpgGainLo> <cpgGainHi> <muscleBwLo> <muscleBwHi> <motorBwLo> <motorBwHi> <tauLo> <tauHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float cpgGainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const float cpgGainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 45.0f;
        const float muscleBwLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.00005f;
        const float muscleBwHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 0.002f;
        const float motorBwLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.002f;
        const float motorBwHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 0.03f;
        const float tauLo = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.05f;
        const float tauHi = argc > 10 ? static_cast<float>(std::atof(argv[10])) : 20.0f;
        const int seedsPerTrial = argc > 11 ? std::atoi(argv[11]) : 6;
        const unsigned rngSeed = argc > 12 ? static_cast<unsigned>(std::atoi(argv[12])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, cpgGain linear [%.2f,%.2f], muscleBw log-uniform [%.6f,%.6f], "
                    "motorBw log-uniform [%.5f,%.5f], calciumTau log-uniform [%.3f,%.3f], ratio>=%.2f required, "
                    "%d seeds/trial, rngSeed=%u\n",
                    trials, cpgGainLo, cpgGainHi, muscleBwLo, muscleBwHi, motorBwLo, motorBwHi, tauLo, tauHi,
                    kMinRatioMargin, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> cpgGainDist(cpgGainLo, cpgGainHi);
        std::uniform_real_distribution<float> muscleBwLogDist(std::log(muscleBwLo), std::log(muscleBwHi));
        std::uniform_real_distribution<float> motorBwLogDist(std::log(motorBwLo), std::log(motorBwHi));
        std::uniform_real_distribution<float> tauLogDist(std::log(tauLo), std::log(tauHi));
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
            cand[kCalciumTau] = std::exp(tauLogDist(rng));
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

    // Default: quick screen - calciumTau sweep at the shipped cpgGain/muscleBandwidth point,
    // PLUS a screen of high cpgGain x calciumTau (the actual hypothesis: does calcium smoothing
    // rescue the high-cpgGain region three prior searches this session found always broken?).
    constexpr int kScreenSeeds = 8;
    constexpr int kSeedBase = 42;
    std::printf("=== Screen A: calciumTau sweep at shipped cpgGain=1.901/muscleBw=0.000249/motorBw=0.00449 ===\n");
    const float kTaus[] = {0.0f, 0.1f, 0.3f, 1.0f, 3.0f, 8.0f, 20.0f};
    for (float tau : kTaus) {
        Candidate cand = kShippedNoCalcium;
        cand[kCalciumTau] = tau;
        const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
        const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
        const bool bothOk = isHealthy(agar) && isHealthy(water);
        const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
        std::printf("calciumTau=%.2f:\n", tau);
        printAgg("agar", agar);
        printAgg("water", water);
        std::printf("  ratio water/agar: %s\n", bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)");
        std::fflush(stdout);
    }

    std::printf("\n=== Screen B: high cpgGain x calciumTau, muscleBw/motorBw fixed at shipped values "
                "(the core hypothesis) ===\n");
    const float kHighCpgGains[] = {5.0f, 15.0f, 30.0f};
    const float kTausB[] = {0.0f, 0.5f, 2.0f, 6.0f};
    for (float g : kHighCpgGains) {
        for (float tau : kTausB) {
            Candidate cand = kShippedNoCalcium;
            cand[kCpgGain] = g;
            cand[kCalciumTau] = tau;
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("cpgGain=%.1f calciumTau=%.2f:\n", g, tau);
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
