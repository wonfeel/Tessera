// tests/worm_proprioceptive_filter_calibration/main.cpp
//
// Calibration for Params::proprioceptiveFilterTau (WormSim.h/.cpp) - a
// follow-up to tests/worm_jerkiness_diagnostic's kDeviationClamp fix. That
// fix clamped the OUTPUT deviation signal (per-position, [-2,2]) right before
// it reaches WormBody - a real, verified partial fix (55-70% peak reduction)
// but not a cure: during the rare burst, 15-17/24 positions saturate the same
// ceiling simultaneously, so the aggregate signal reaching the body is still
// large even with each individual value capped.
//
// By direct user request ("запускай агентов на проверку и поиск информации,
// пускай глянут другие подобные проекты"), four parallel research agents
// looked at how other C. elegans / CPG / neuromechanical projects handle
// this class of problem before this file was written:
//   - OpenWorm/Sibernetic: confirmed our agar/water ratio (3.456) is already
//     realistic (real ratio ~4-5x freq/~2-3x speed) - not directly relevant
//     to jerkiness, but ruled out "the ratio itself is broken" as a
//     motivation for further neural retuning.
//   - c302 / Izquierdo & Beer: no published per-class leak/capacitance
//     benchmark exists to compare against, but the literature on taming rare
//     bursts in leaky-integrator/CTRNN circuits points to a SECOND filtering
//     stage on the network's OUTPUT (Rusakov et al. 2021 "dynamic-leak"
//     model: tau_dyn*dA_dyn/dt = -A_dyn + A_raw) as the standard technique -
//     0.99 cross-correlation with the unfiltered signal in their tests, i.e.
//     it damps rare excursions without flattening the normal oscillation.
//   - CPG/delayed-feedback robotics literature (Stepan/Insperger DDE theory;
//     Ijspeert resonance tuning): identifies exactly this failure mode -
//     proprioceptive feedback delay resonating with a CPG's own period can
//     trigger a Hopf-bifurcation-style sudden large excursion, not a slow
//     drift - matching our own diagnosis in tests/worm_jerkiness_diagnostic
//     almost exactly. The literature's standard mitigation: low-pass filter
//     the FEEDBACK SIGNAL ITSELF (before it re-enters the loop), not the
//     loop's output after the fact - because filtering post-hoc only
//     truncates a symptom after the network's internal state is already
//     corrupted, while filtering the input prevents the destabilizing
//     excitation from reaching the network in the first place.
//   - An independent repo audit (read-only) confirmed the deviation clamp
//     and setMedium() atomic write match what's documented, 6/6 checks
//     passed - ruled out "the existing fix is broken" as an explanation for
//     residual jerk.
//
// THE NEW LEVER: WormSim::step() now runs an EMA low-pass filter over the
// (possibly delay-history-read) body-angle signal BEFORE applyProprioception
// averages it into each motor neuron's window - see Params::
// proprioceptiveFilterTau for the full math/citation. At filterTau=0
// (default, every Params field outside this file) this is bitwise the old
// behavior - no filter state is even allocated.
//
// Complementary, not a replacement: the existing kDeviationClamp in
// WormSim::step stays in place regardless of what this file finds - it is a
// last-resort safety net on the OUTPUT, this is a first-line prevention
// attempt on the INPUT. Both can coexist.
//
// Search plan, same discipline as every other axis this session: cheap
// decisive SWEEP first (does filtering reduce burst severity at all, at the
// already-shipped cpgGain/muscleBandwidthGain/motorBandwidthGain/
// proprioceptiveDelaySeconds point, without breaking health/ratio on the
// SHORT 125s window this session's other calibrations use), THEN a JERK mode
// for detailed long-run (1000s) before/after comparison of the winning tau
// against the current production point, matching tests/worm_jerkiness_
// diagnostic's own methodology exactly so the numbers are comparable. Health
// gate identical to every other axis this session: efficiency>=0.40,
// coiledRatio>=0.30, freqHz>=0.001, maxAbsHeadingDelta<=0.5 rad (short-window
// only - the long-run jerk check is a SEPARATE, additional measurement, not
// part of this gate, exactly like tests/worm_jerkiness_diagnostic explains
// why the short window structurally can't see this failure mode).
//
// RESULT: NEGATIVE. Filtering the proprioceptive INPUT does not reduce the
// burst on this network, contrary to what the general CPG/delayed-feedback
// literature would predict.
//
// Sweep (short-window health/ratio + 400s x6-seed jerk check) across
// filterTau in [0.05, 3.0] (log-spaced, shipped cpg/bandwidth/delay point
// held fixed): filterTau=0.05 is a mathematical no-op at dt=0.05 (alpha =
// clamp(dt/tau,0,1) saturates to 1.0, i.e. filteredAngles is overwritten with
// the raw value every step) - confirmed identical to the tau=0 baseline
// (worst max|delta|=0.3515, over0.2=28, both cases) as a sanity check that
// the implementation is correct. Every OTHER tested value (0.1 through 3.0)
// showed EITHER no improvement or a WORSE worst-case burst than the
// unfiltered baseline (e.g. tau=0.1: 0.4660 vs baseline 0.3515; tau=0.8:
// 0.5064 vs 0.3515) - no value in nearly two decades of tau showed a genuine
// reduction. Health/ratio stayed intact across the whole range (still
// healthy, ratio 3.16-3.51) up to tau=3.0, where ratio started drifting down
// (3.16) and agar's coiled-ratio approached the 0.30 floor (0.605-0.606) -
// so this isn't a case of "the health gate would have hidden a real win";
// there simply wasn't one to hide.
//
// Full 1000s confirmation (jerk mode, tau=0.2, 6 independent seeds, same
// methodology as tests/worm_jerkiness_diagnostic so numbers are directly
// comparable): worst-case max|heading delta| across seeds was 0.4596 rad
// (no filter) vs 0.5138 rad (filterTau=0.2) - an 11.8% WORSE worst case, not
// better; total steps exceeding 0.2 rad was similar (71 vs 77). Short-window
// health/ratio unaffected (still 16/16-equivalent healthy, ratio unchanged
// to 3 significant figures).
//
// Diagnosis: the general literature's recommendation ("filter the feedback
// signal before it re-enters the loop, not the loop's output after the
// fact") assumes the burst is generated BY delay-resonance in the feedback
// loop itself. This project's own diagnostic (tests/worm_jerkiness_
// diagnostic) already hypothesized something different: the burst is an
// excursion intrinsic to the raw 401-neuron network's OWN chaotic recurrent
// dynamics ("states climb toward a large self-sustained equilibrium...
// regardless of external input" - see tests/worm_locomotion's header),
// triggered by high mechanical load (agar-specific, never seen on water),
// not primarily a resonance between proprioceptive delay and CPG period.
// If that diagnosis is right, smoothing the proprioceptive CHANNEL's
// contribution cannot suppress an instability whose energy comes from the
// network's own recurrent weights, independent of what enters through this
// one sensory channel - consistent with what was measured here (no
// improvement, mild worsening from the extra phase lag the filter itself
// introduces on top of an already-substantial 1.596s proprioceptiveDelay).
//
// NOT SHIPPED. proprioceptiveFilterTau stays at its bitwise-identical
// default (0.0) in production. The existing kDeviationClamp (WormSim.cpp,
// output-side) remains the only active jerkiness mitigation - see its own
// RESULT in tests/worm_jerkiness_diagnostic for the honest "reduced, not
// solved" status. Root cause (why the raw network bursts at all under high
// agar load) remains uninvestigated - this file rules out one candidate
// fix, it does not find the root cause.
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

// {cpgGain, muscleBandwidthGain, motorBandwidthGain, proprioceptiveDelaySeconds, proprioceptiveFilterTau}
using Candidate = std::array<float, 5>;
enum { kCpgGain = 0, kMuscleBw = 1, kMotorBw = 2, kDelay = 3, kFilterTau = 4 };

constexpr float kCpgFreq = 1.108f, kCpgSens = 0.02f, kCpgAmpSens = 0.0342f, kPoseDecay = 1.051f;

// Today's shipped point (before this file), filter off.
const Candidate kShipped = {2.621f, 0.000153f, 0.0194f, 1.596f, 0.0f};

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
    sim.params.proprioceptiveFilterTau = c[kFilterTau];
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
    std::printf("[cpgGain=%.3f muscleBw=%.6f motorBw=%.5f delaySec=%.3f filterTau=%.4f]", c[kCpgGain], c[kMuscleBw],
                c[kMotorBw], c[kDelay], c[kFilterTau]);
}

// Long-run (default 1000s) windowed jerkiness measurement - same methodology
// as tests/worm_jerkiness_diagnostic's runOne, parameterized for the 5-param
// Candidate here so results are directly comparable.
struct JerkResult {
    float wholeRunMax = 0.0f;
    long long over02 = 0, over04 = 0;
    long long totalSteps = 0;
    bool anyNaN = false;
};

JerkResult runJerk(const Candidate& cand, float dragNormal, int seed, int totalSteps) {
    JerkResult jr;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    sim.params.dragTangent = 1.0f;
    sim.params.dragNormal = dragNormal;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);

    WormSim::Snapshot snap;
    float prevHeading = 0.0f;
    bool havePrevHeading = false;
    for (int i = 0; i < totalSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float headX = snap.pointsX[0], headY = snap.pointsY[0];
        if (std::isnan(headX) || std::isnan(headY)) { jr.anyNaN = true; break; }
        const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            const float absHd = std::fabs(hd);
            jr.wholeRunMax = std::max(jr.wholeRunMax, absHd);
            if (absHd > 0.2f) ++jr.over02;
            if (absHd > 0.4f) ++jr.over04;
            ++jr.totalSteps;
        }
        prevHeading = heading;
        havePrevHeading = true;
    }
    return jr;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution cpgGain muscleBw motorBw delaySec filterTau [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kShipped;
        for (int k = 0; k < 5 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 7 ? std::atoi(argv[7]) : 12;
        const int seedsPerBase = argc > 8 ? std::atoi(argv[8]) : 8;
        const int warmupSteps = argc > 9 ? std::atoi(argv[9]) : 300;
        const int measureSteps = argc > 10 ? std::atoi(argv[10]) : 2500;
        std::printf("cand="); printCand(cand);
        std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, bothHealthyCount = 0, waterBeatsAgarCount = 0;
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

    // ./exe jerk filterTau [numSeeds] [seedBase] [totalSteps] - agar only (water never shows this bug).
    // Compares filterTau=0 (current shipped) against the given filterTau at
    // the SAME seeds, same methodology as tests/worm_jerkiness_diagnostic.
    if (argc >= 2 && std::string(argv[1]) == "jerk") {
        const float filterTau = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 0.5f;
        const int numSeeds = argc > 3 ? std::atoi(argv[3]) : 4;
        const int seedBase = argc > 4 ? std::atoi(argv[4]) : 42;
        const int totalSteps = argc > 5 ? std::atoi(argv[5]) : 20000;  // 1000s at dt=0.05
        Candidate withFilter = kShipped;
        withFilter[kFilterTau] = filterTau;
        std::printf("=== jerk comparison: %d seeds x %d steps (%.0fs) on agar, filterTau=%.4f vs 0 (shipped) ===\n",
                    numSeeds, totalSteps, totalSteps * 0.05f, filterTau);
        float maxNoFilter = 0.0f, maxWithFilter = 0.0f;
        long long over02NoFilter = 0, over02WithFilter = 0;
        for (int s = 0; s < numSeeds; ++s) {
            const int seed = seedBase + s;
            const JerkResult noFilter = runJerk(kShipped, kDragAgar, seed, totalSteps);
            const JerkResult withF = runJerk(withFilter, kDragAgar, seed, totalSteps);
            std::printf("seed=%d: noFilter max=%.4f (over0.2=%lld) | filterTau=%.4f max=%.4f (over0.2=%lld)%s%s\n",
                        seed, noFilter.wholeRunMax, noFilter.over02, filterTau, withF.wholeRunMax, withF.over02,
                        noFilter.anyNaN ? " [NaN-noFilter]" : "", withF.anyNaN ? " [NaN-withFilter]" : "");
            std::fflush(stdout);
            maxNoFilter = std::max(maxNoFilter, noFilter.wholeRunMax);
            maxWithFilter = std::max(maxWithFilter, withF.wholeRunMax);
            over02NoFilter += noFilter.over02;
            over02WithFilter += withF.over02;
        }
        std::printf("\nAcross %d seeds: worst-case max|delta| noFilter=%.4f vs filterTau=%.4f: %.4f "
                    "(%.1f%% change); total steps>0.2rad noFilter=%lld vs filter=%lld\n",
                    numSeeds, maxNoFilter, filterTau, maxWithFilter,
                    maxNoFilter > 1e-9f ? 100.0 * (maxWithFilter - maxNoFilter) / maxNoFilter : 0.0,
                    over02NoFilter, over02WithFilter);
        // Also print short-window health/ratio so a candidate that kills the burst
        // by killing the whole gait doesn't look like a win.
        const AggregateResult agar = evaluate(withFilter, 8, 42, kDragAgar, 300, 2500);
        const AggregateResult water = evaluate(withFilter, 8, 42, kDragWater, 300, 2500);
        std::printf("Short-window (125s) health check at filterTau=%.4f:\n", filterTau);
        printAgg("agar", agar);
        printAgg("water", water);
        return 0;
    }

    // ./exe sweep [tau1] [tau2] ... - screens several filterTau values at the
    // shipped cpg/bandwidth/delay point: short-window health/ratio (cheap)
    // plus a moderate-length (8000 steps = 400s) multi-seed jerk check
    // (cheaper than the full 1000s jerk mode, still long enough to have a
    // real chance of sampling the ~300-500s-recurrence burst).
    {
        std::vector<float> taus;
        if (argc >= 2 && std::string(argv[1]) == "sweep") {
            for (int k = 2; k < argc; ++k) taus.push_back(static_cast<float>(std::atof(argv[k])));
        }
        if (!taus.empty()) {
            constexpr int kJerkSeeds = 6, kJerkSteps = 8000;  // 400s
            std::printf("=== sweep: %zu filterTau values, shipped cpg/bandwidth/delay held fixed ===\n", taus.size());
            for (float tau : taus) {
                Candidate cand = kShipped;
                cand[kFilterTau] = tau;
                const AggregateResult agar = evaluate(cand, 8, 42, kDragAgar, 300, 2500);
                const AggregateResult water = evaluate(cand, 8, 42, kDragWater, 300, 2500);
                const bool bothOk = isHealthy(agar) && isHealthy(water);
                const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
                float maxDelta = 0.0f;
                long long over02 = 0;
                bool anyNaN = false;
                for (int s = 0; s < kJerkSeeds; ++s) {
                    const JerkResult jr = runJerk(cand, kDragAgar, 100 + s, kJerkSteps);
                    maxDelta = std::max(maxDelta, jr.wholeRunMax);
                    over02 += jr.over02;
                    anyNaN = anyNaN || jr.anyNaN;
                }
                std::printf("filterTau=%.4f:\n", tau);
                printAgg("agar", agar);
                printAgg("water", water);
                std::printf("  ratio water/agar: %s | jerk(400s x%d seeds) worst max|delta|=%.4f over0.2=%lld%s\n",
                            bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)", kJerkSeeds, maxDelta, over02,
                            anyNaN ? " [NaN]" : "");
                std::fflush(stdout);
            }
            return 0;
        }
    }

    // ./exe random <trials> <filterTauLo> <filterTauHi> [seedsPerTrial] [rngSeed]
    // Joint search is NOT run by default here - the shipped cpgGain/
    // muscleBandwidthGain/motorBandwidthGain/proprioceptiveDelaySeconds point
    // is held fixed (already extensively confirmed, see WORM.md section 6);
    // only filterTau is randomized, since this lever targets a specific,
    // separate failure mode (rare bursts) rather than tempo/ratio. Included
    // for the case where 'sweep' alone doesn't localize a clear winner.
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 40;
        const float tauLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.01f;
        const float tauHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 5.0f;
        const int seedsPerTrial = argc > 5 ? std::atoi(argv[5]) : 8;
        const unsigned rngSeed = argc > 6 ? static_cast<unsigned>(std::atoi(argv[6])) : 1u;
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> tauLogDist(std::log(tauLo), std::log(tauHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        std::printf("random filterTau search: %d trials, log-uniform [%.4f,%.4f], ratio>=%.2f required, %d "
                    "seeds/trial, rngSeed=%u\n",
                    trials, tauLo, tauHi, kMinRatioMargin, seedsPerTrial, rngSeed);
        int foundPassingGate = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand = kShipped;
            cand[kFilterTau] = std::exp(tauLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, 300, 2500);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, 300, 2500);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio < kMinRatioMargin) continue;
            ++foundPassingGate;
            std::printf("PASS ratio=%.3f base=%d filterTau=%.4f\n", ratio, base, cand[kFilterTau]);
            printAgg("  agar", agar);
            printAgg("  water", water);
            std::fflush(stdout);
        }
        std::printf("\n%d/%d trials passed health+ratio>=%.2f gate.\n", foundPassingGate, trials, kMinRatioMargin);
        return 0;
    }

    std::printf("Usage: %s [distribution|jerk|sweep|random] ...\n", argv[0]);
    return 0;
}
