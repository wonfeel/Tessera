// tests/worm_cpg_muscle_bandwidth_calibration/main.cpp
//
// Joint search of the CPG parameter family (Params::cpgGain/cpgBaseFreqHz/
// cpgLoadSensitivity/cpgAmpLoadSensitivity) x the load-responsive muscle/
// motor bandwidth mechanism (Params::muscleBandwidthGain/motorBandwidthGain,
// tests/worm_muscle_bandwidth_calibration) x bodyPoseDecayRate. Neither
// mechanism alone solves both open problems this project has tracked all
// session (realistic bend TEMPO and water/agar propulsion RATIO):
//   - cpgGain (tests/worm_cpg_calibration): realistic frequency (0.1-1+Hz,
//     best-ever honest agar speed ~0.057 BL/s) but 0/300 healthy ratio>1
//     candidates, twice.
//   - muscleBandwidthGain/motorBandwidthGain (tests/worm_muscle_bandwidth_
//     calibration): BEST ratio in this project's history (1.232, confirmed
//     16/16 bases) but tempo stays low (agar 0.0024Hz, water 0.0120Hz) - and
//     an extended search pushing muscleBandwidthGain/motorBandwidthGain
//     themselves toward realistic bandwidth (up to 0.2/5.0) jointly with
//     bodyPoseDecayRate up to 50 (25x the confirmed-best point, this session,
//     same investigation) found ZERO healthy ratio>1 candidates and ZERO
//     realistic-tempo candidates across 480 trials - so pushing THIS
//     mechanism's own gains higher is a confirmed dead end, not merely
//     unexplored.
//
// Three independent prior attempts to COMBINE any two of this project's
// feedback/gain mechanisms at their own separately-confirmed-good points have
// ALL failed catastrophically: cpgGain+dragAdhesionGain (tests/worm_drag_
// adhesion_cpg_calibration, 0/300, best 0.983), cpgGain+
// dragAdhesionAdditiveGain (tests/worm_drag_adhesion_additive_cpg_
// calibration, 0/300, best 0.424), muscleBandwidthGain+
// dragAdhesionAdditiveGain (scratch test, this session, 0/8 healthy). The
// working hypothesis after three failures was "any two mechanisms touching
// the closed proprioception->curvature->body->motion loop from different
// directions destabilize this connectome's narrow healthy window" - a
// structural property, not a coincidence of any one pairing.
//
// This file tests the ONE combination not yet tried that has a real
// mechanistic reason to differ from the three failures above: CPG
// (Network::add_input - an external forced INPUT into motor neurons) and
// muscle/motor bandwidth (Network::set_muscle_leak/set_motor_leak_scale - the
// GAIN on how much of ANY input, CPG's included, survives the muscle/motor
// relay) are not two competing signals or two competing force-law changes -
// one is a source, the other is a transmission gain on that same source.
// Widening the bandwidth is, if anything, what CPG's OWN header (tests/worm_
// cpg_calibration) already identified as necessary for its signal to reach
// measurable frequency at all ("the CPG can only matter if the muscle layer
// can actually follow it") - that file's own joint search DID include a
// muscle-leak companion (muscleLeakScale, static, range [0.02,3.0]) and still
// got 0/300, but never in the much lower, load-RESPONSIVE regime tests/worm_
// muscle_bandwidth_calibration confirmed actually works alone (effective
// leak orders of magnitude below 0.02). Honest prior, stated up front: after
// three independent failures of the general "combine two mechanisms" pattern,
// this is not expected to break the pattern - it is tested because it has a
// distinct mechanistic argument, not because the outcome is assumed.
//
// muscleLeakScale/motorLeakScale (the STATIC companions) stay at their
// production defaults (0.0/1.0) throughout this file - already searched
// jointly with CPG in tests/worm_cpg_calibration and confirmed not to help
// (0/300). Only the DYNAMIC, load-responsive bandwidth gains are new here.
//
// Search scope: {cpgGain, cpgBaseFreqHz, cpgLoadSensitivity,
// cpgAmpLoadSensitivity, bodyPoseDecayRate, muscleBandwidthGain,
// motorBandwidthGain} - 7 parameters.
//   cpgGain in [0, 50] linear - same range as every other CPG joint search
//     this project has run, includes near-zero so the search can honestly
//     conclude "CPG should stay off" if that's what it finds.
//   cpgBaseFreqHz in [0.5, 4.0]Hz log-uniform, cpgLoadSensitivity in
//     [0.005, 2.0] log-uniform, cpgAmpLoadSensitivity in [0.001, 0.2]
//     log-uniform - all identical, already-validated ranges from tests/
//     worm_cpg_calibration.
//   bodyPoseDecayRate in [0.5, 5.0] log-uniform - NOT extended to the [0.5,50]
//     range this session's muscle-bandwidth-alone extended search already
//     found useless; kept near muscleBandwidthGain's own confirmed-best
//     (2.198) with headroom in case CPG shifts the optimum, not open-ended.
//   muscleBandwidthGain in [0.00005, 0.0005] log-uniform, motorBandwidthGain
//     in [0.002, 0.02] log-uniform - TIGHT brackets around tests/worm_muscle_
//     bandwidth_calibration's own confirmed-best point (0.00012/0.0064) -
//     this session already confirmed wandering outside this narrow pocket
//     fails even WITHOUT CPG in play, so there is no reason to search wider
//     here.
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried. Fitness: water/agar ratio>1.0 with both media healthy,
// same as always - tempo (freqHz) is tracked and reported but is not itself
// the pass/fail criterion (matches tests/worm_muscle_bandwidth_calibration's
// own realistic-freq counter, >0.05Hz on both media).
//
// RESULT: THE FIRST SUCCESSFUL COMBINATION OF TWO MECHANISMS IN THIS
// PROJECT'S HISTORY, AND THE BEST RATIO EVER CONFIRMED (2.692) - inside the
// user's own explicitly requested target of 200-300% (real C. elegans swims
// 2-3x faster than it crawls). This is the FOURTH attempt at combining two
// of this session's mechanisms; the first three (cpgGain+dragAdhesionGain,
// cpgGain+dragAdhesionAdditiveGain, muscleBandwidthGain+
// dragAdhesionAdditiveGain) all collapsed to 0 healthy candidates. This one
// did not - confirming the file's own opening hypothesis that CPG (a signal
// SOURCE) and muscle/motor bandwidth (a GAIN on transmission of any source)
// are a structurally different kind of pairing than two competing signals or
// two competing force-law changes.
//
// Screen (cpgGain sweep, muscleBandwidth off vs at its own confirmed-best):
// with muscleBandwidth OFF, cpgGain>=15 already breaks health (efficiency/
// coiled degrade). With muscleBandwidth ON at its own confirmed-best point,
// cpgGain=5-30 stays HEALTHY and reaches REALISTIC TEMPO on both media for
// the first time this project has seen that (freq>0.05Hz on both agar AND
// water simultaneously) - but the ratio INVERTS hard (0.08-0.18, agar 5-12x
// FASTER than water) as cpgGain grows. Root cause (traced, not guessed):
// muscleBandwidthGain's leak boost scales with normalizedLoad, which is
// ~25x larger on agar than water - so agar's muscle relay bandwidth widens
// far more than water's, enough to outrun cpgGain's own load-based frequency
// SUPPRESSION on agar. Two honest mechanisms, each individually justified,
// pulling in opposite directions on which medium ends up faster.
//
// Broad random search (400 trials, full 7-param ranges from the header):
// 2/400 healthy-ratio>1 hits, both CONFIRMED via 16-base distribution:
// cpgGain=1.049 (small, not the screen's stronger values) + muscleBw=
// 0.000306: ratio=1.386 (beats muscleBandwidthGain-alone's own 1.232 record,
// on every tracked axis - agar freq 0.0103Hz, water freq 0.0632Hz, both
// higher than the alone-mechanism's 0.0024/0.0120Hz). cpgGain=5.7: ratio
// only 1.072 but agar freq 0.078Hz - confirms the tempo/ratio trade-off seen
// in the screen is real and continuous, not a screen artifact.
//
// Tight-bracket refinement (450 trials around the ratio=1.386 point, 3
// parallel shards): dramatically higher hit rate (85/450 vs 2/400 broad) -
// confirms this is a real, correctly-located pocket. Winning candidate
// (chosen by a composite score rewarding BOTH ratio and min(agar,water)
// frequency, not ratio alone - a single freq=0.001Hz/ratio=2.668 hit was
// deliberately NOT chosen despite its higher raw ratio, being right at the
// health gate's frequency floor and thus a likely fragile single-base
// artifact): cpgGain=1.901, cpgBaseFreqHz=1.108, cpgLoadSensitivity=0.02,
// cpgAmpLoadSensitivity=0.0342, bodyPoseDecayRate=1.051,
// muscleBandwidthGain=0.000249, motorBandwidthGain=0.00449.
//
// CONFIRMED via 16 independent bases x 8 seeds: agar healthy=16/16 (mean
// 0.00216 BL/s, 0.0192Hz), water healthy=16/16 (mean 0.00580 BL/s,
// 0.1313Hz), both-healthy=16/16, MEAN RATIO=2.692, water>agar in ALL 16
// bases. This is inside the user's explicit 200-300% target, robust across
// independent seed bases, not a single-base illusion.
//
// Honest remaining gap: tempo is still far from real C. elegans (Fang-Yen
// et al. 2010: crawl ~0.3-0.5Hz, swim ~1.7-2Hz) - 0.0192Hz/0.1313Hz here is
// roughly 15-25x below that, though a real, substantial improvement over
// muscleBandwidthGain-alone (8x agar, 11x water) and light-years past the
// original ~0.005-0.01Hz baseline. Tempo is improved, not solved.
//
// Honest framing of what this ships: unlike muscleBandwidthGain alone
// (purely an honest widening of the existing proprioceptive channel), this
// confirmed-best point uses cpgGain=1.9 - a meaningful, non-trivial
// activation of this project's ONE explicitly user-authorized exception to
// "behavior only from the real network" (see Params::cpgGain). Shipping this
// point means shipping that exception active, not just the honest bandwidth
// mechanism.
//
// SHIPPED (by direct user request, "включи это в проде"): Params defaults
// for cpgGain/cpgBaseFreqHz/cpgLoadSensitivity/cpgAmpLoadSensitivity/
// bodyPoseDecayRate/muscleBandwidthGain/motorBandwidthGain now equal this
// confirmed-best point (WormSim.h). Side effect worth knowing: every test
// file in this suite unity-builds WormSim.cpp fresh and constructs WormSim
// with class defaults unless it explicitly overrides a given field - test
// files written before this change (i.e. every calibration file except this
// one) do NOT override these 7 fields, so re-running any of them today will
// silently run WITH this point active on top of whatever axis that file
// tests, and will NOT reproduce that file's originally-documented numbers
// (which were measured against the old all-zero baseline). Their RESULT text
// remains an accurate historical record of what was true under the
// conditions stated at the time; it is not a live, auto-reproducing
// assertion.
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

// {cpgGain, cpgBaseFreqHz, cpgLoadSensitivity, cpgAmpLoadSensitivity, bodyPoseDecayRate, muscleBandwidthGain, motorBandwidthGain}
using Candidate = std::array<float, 7>;
enum { kCpgGain = 0, kCpgFreq = 1, kCpgSens = 2, kCpgAmpSens = 3, kPoseDecay = 4, kMuscleBw = 5, kMotorBw = 6 };
const Candidate kIdentity = {0.0f, 2.0f, 0.05f, 0.0f, 0.5f, 0.0f, 0.0f};
// tests/worm_muscle_bandwidth_calibration's own confirmed-best point (ratio=1.232, 16/16 bases).
const Candidate kMuscleBwBestOnly = {0.0f, 2.0f, 0.05f, 0.0f, 2.198f, 0.00012f, 0.0064f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.cpgGain = c[kCpgGain];
    sim.params.cpgBaseFreqHz = c[kCpgFreq];
    sim.params.cpgLoadSensitivity = c[kCpgSens];
    sim.params.cpgAmpLoadSensitivity = c[kCpgAmpSens];
    sim.params.cpgWavelengths = kFixedWavelengths;
    sim.params.bodyPoseDecayRate = c[kPoseDecay];
    sim.params.muscleBandwidthGain = c[kMuscleBw];
    sim.params.motorBandwidthGain = c[kMotorBw];
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
    std::printf("[cpgGain=%.3f cpgFreq=%.3f cpgSens=%.3f cpgAmpSens=%.4f poseDecay=%.3f muscleBw=%.6f motorBw=%.5f]",
                c[kCpgGain], c[kCpgFreq], c[kCpgSens], c[kCpgAmpSens], c[kPoseDecay], c[kMuscleBw], c[kMotorBw]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution cpgGain cpgFreq cpgSens cpgAmpSens poseDecay muscleBw motorBw [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kMuscleBwBestOnly;
        for (int k = 0; k < 7 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 9 ? std::atoi(argv[9]) : 12;
        const int seedsPerBase = argc > 10 ? std::atoi(argv[10]) : 8;
        const int warmupSteps = argc > 11 ? std::atoi(argv[11]) : 300;
        const int measureSteps = argc > 12 ? std::atoi(argv[12]) : 2500;
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

    // ./exe random <trials> <cpgGainLo> <cpgGainHi> <freqLo> <freqHi> <sensLo> <sensHi> <ampSensLo> <ampSensHi> <decayLo> <decayHi> <muscleBwLo> <muscleBwHi> <motorBwLo> <motorBwHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float cpgGainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const float cpgGainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 50.0f;
        const float freqLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.5f;
        const float freqHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 4.0f;
        const float sensLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.005f;
        const float sensHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 2.0f;
        const float ampSensLo = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.001f;
        const float ampSensHi = argc > 10 ? static_cast<float>(std::atof(argv[10])) : 0.2f;
        const float decayLo = argc > 11 ? static_cast<float>(std::atof(argv[11])) : 0.5f;
        const float decayHi = argc > 12 ? static_cast<float>(std::atof(argv[12])) : 5.0f;
        const float muscleBwLo = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 0.00005f;
        const float muscleBwHi = argc > 14 ? static_cast<float>(std::atof(argv[14])) : 0.0005f;
        const float motorBwLo = argc > 15 ? static_cast<float>(std::atof(argv[15])) : 0.002f;
        const float motorBwHi = argc > 16 ? static_cast<float>(std::atof(argv[16])) : 0.02f;
        const int seedsPerTrial = argc > 17 ? std::atoi(argv[17]) : 6;
        const unsigned rngSeed = argc > 18 ? static_cast<unsigned>(std::atoi(argv[18])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, cpgGain linear [%.2f,%.2f], freq log-uniform [%.2f,%.2f]Hz, sens "
                    "log-uniform [%.3f,%.3f], ampSens log-uniform [%.3f,%.3f], poseDecay log-uniform [%.2f,%.2f], "
                    "muscleBw log-uniform [%.6f,%.6f], motorBw log-uniform [%.5f,%.5f], %d seeds/trial, rngSeed=%u\n",
                    trials, cpgGainLo, cpgGainHi, freqLo, freqHi, sensLo, sensHi, ampSensLo, ampSensHi, decayLo,
                    decayHi, muscleBwLo, muscleBwHi, motorBwLo, motorBwHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> cpgGainDist(cpgGainLo, cpgGainHi);
        std::uniform_real_distribution<float> freqLogDist(std::log(freqLo), std::log(freqHi));
        std::uniform_real_distribution<float> sensLogDist(std::log(sensLo), std::log(sensHi));
        std::uniform_real_distribution<float> ampSensLogDist(std::log(ampSensLo), std::log(ampSensHi));
        std::uniform_real_distribution<float> decayLogDist(std::log(decayLo), std::log(decayHi));
        std::uniform_real_distribution<float> muscleBwLogDist(std::log(muscleBwLo), std::log(muscleBwHi));
        std::uniform_real_distribution<float> motorBwLogDist(std::log(motorBwLo), std::log(motorBwHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0, foundRealisticFreq = 0;
        float bestRatio = -1.0f;
        Candidate bestCand{};
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kCpgGain] = cpgGainDist(rng);
            cand[kCpgFreq] = std::exp(freqLogDist(rng));
            cand[kCpgSens] = std::exp(sensLogDist(rng));
            cand[kCpgAmpSens] = std::exp(ampSensLogDist(rng));
            cand[kPoseDecay] = std::exp(decayLogDist(rng));
            cand[kMuscleBw] = std::exp(muscleBwLogDist(rng));
            cand[kMotorBw] = std::exp(motorBwLogDist(rng));
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
                std::printf("ratio=%.3f base=%d ", ratio, base);
                printCand(cand);
                std::printf("\n");
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0; %d with ratio>1.5; %d with ratio>2.0; %d with "
                    "BOTH media freq>0.05Hz (realistic-ish tempo). Best ratio: %.3f",
                    foundOver1, trials, foundOver15, foundOver2, foundRealisticFreq, bestRatio < 0.0f ? 0.0f : bestRatio);
        if (bestRatio > 0.0f) {
            std::printf(" at base=%d ", bestBase);
            printCand(bestCand);
        }
        std::printf("\nEach ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // Default: quick screen - cpgGain x muscleBandwidthGain/motorBandwidthGain
    // "on at its own confirmed-best point" vs "off", at increasing cpgGain.
    constexpr int kScreenSeeds = 8;
    constexpr int kSeedBase = 42;
    const float kCpgGains[] = {0.0f, 1.0f, 5.0f, 15.0f, 30.0f};
    std::printf("=== Screen (cpgGain sweep x muscleBandwidth {off, confirmed-best}, freq=2.0/sens=0.05/ampSens=0.0 "
                "fixed, %d seeds/point, base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (int bwOn = 0; bwOn < 2; ++bwOn) {
        for (float g : kCpgGains) {
            Candidate cand = bwOn ? kMuscleBwBestOnly : kIdentity;
            cand[kCpgGain] = g;
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("muscleBw=%s cpgGain=%.1f:\n", bwOn ? "confirmed-best" : "off", g);
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
