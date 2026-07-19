// tests/worm_bclass_oscillator_calibration/main.cpp
//
// Sixth axis on the swim>crawl speed bug (see tests/worm_speed_calibration,
// worm_speed_leak_calibration, worm_synapse_speed_calibration for the full
// history of axes 1-5 - all either insufficient or reverted). Every prior
// axis rescaled coefficients of an otherwise-fixed LINEAR system (diagonal
// leak/capacitance, off-diagonal synapse weights, or additive forcing on
// external_i) - none of them gave the network a genuinely different kind of
// attractor to have. tests/worm_network_eigenmodes independently confirmed
// the structural ceiling: this network's emergent oscillation lives at a
// specific, fragile eigenvalue configuration, which is why 0/480 (leak/
// capacitance) and 0/400 (leak/capacitance + synapse-sign jointly) bounded
// random trials ever found a healthy water>agar candidate, and why the one
// axis that DID move the frequency (synapse-sign alone) had to swing the
// weight matrix hard enough to also break heading stability.
//
// THIS FILE'S LEVER: Network::set_active_current (network.hpp/.cpp) - a
// genuine SELF-REFERENTIAL regenerative current added to the 18 real
// B-class motor neurons (DB1-7, VB1-11 - WormSim.cpp constructor identifies
// them by name into m_classBMotorNeurons), not another rescaling of an
// existing linear term. Biological grounding: Gao, Guan, Fouad et al. 2018,
// eLife 7:e29915 ("Excitatory motor neurons are local oscillators for
// backward locomotion") shows A-class motor neurons are cell-autonomous
// UNC-2(CaV2)-dependent relaxation oscillators via direct electrophysiology,
// ablation and genetics; Fouad et al. 2018, eLife 7:e29913 ("Distributed
// rhythm generators underlie C. elegans forward locomotion") shows the
// analogous local-oscillator property for FORWARD locomotion belongs
// specifically to B-class - the class broken here (Fang-Yen et al. 2010's
// crawl/swim numbers, already cited in WormSim.h, are both forward-
// locomotion measurements). Honesty flag, carried from the design proposal:
// whether UNC-2 SPECIFICALLY underlies B-class oscillation (as opposed to
// A-class, where this is proven) has not been directly shown - Wen, Gao &
// Zhen 2018, Phil. Trans. R. Soc. B 373:20170370 says so explicitly. This
// is a homology-based analogy across sister motor classes, not a directly
// confirmed mechanism for B specifically - the calibration below is what
// tells us whether the analogy is dynamically load-bearing in THIS network.
//
// Equation (network.hpp/cpp): for each targeted neuron i,
//   C_i dV_i/dt = ...existing terms... + activeGain * w_i * sigmoid((V_i-theta)/slope)
//   dw_i/dt = ( [1 - sigmoid((V_i-theta)/slope)] - w_i ) / tau_w
// Two free parameters searched here: gain, tau_w. Isolated from every other
// axis - leak/capacitance, synapse-sign, and localMechanoGain are all left
// at identity/0 so any effect found is attributable to this axis alone.
//
// Search ranges (bounded, not evolutionary - same discipline as
// worm_speed_leak_calibration after Round 1's ungrounded walk):
//   gain   in [0, 20]   - no literature-derived unit for this scalar (it is
//                          this reduced model's own internal current scale,
//                          not a real pA figure) - anchored instead to this
//                          project's existing gain scalars of the same kind
//                          (proprioceptiveGain default 4.0, prior mechanoGain
//                          grids to +/-30) rather than an unbounded search.
//   tau_w  in [0.2, 40]s - lower bound DERIVED: at dt=0.05s default,
//                          alpha_w=dt/tau_w must stay well under 1 for a
//                          genuine fast/slow separation (the mechanism this
//                          whole axis depends on) - 0.2s keeps alpha_w<=0.25.
//                          Upper bound ANCHORED to Gao et al. 2018's own
//                          measured immobilized-A-MN cell-autonomous period
//                          (~50-90s) as a literature-grounded ceiling, kept
//                          conservatively below it.
//
// Health gate: efficiency>=0.40 / coiledRatio>=0.30 / freqHz>kMinFreqHz
// (reject the frozen-static-arc degenerate mode) on EACH preset
// independently, PLUS max |heading delta| in one step <= kMaxHeadingDeltaRad
// - baked in from the START, not bolted on after shipping (the exact mistake
// that shipped the synapse-sign candidate broken). Primary fitness is
// waterBLps > agarBLps with BOTH healthy, not raw speed on either alone.
//
// EXTRA CHECK specific to this axis (per the design proposal's own
// caution): freqHz is tracked and printed for agar and water SEPARATELY,
// not just body-lengths/second. The mechanistic claim this axis is testing
// is that frequency itself should rise in the lower-drag medium (Fang-Yen et
// al. 2010: real undulation frequency falls continuously with rising
// mechanical load) and speed should follow FROM that - a candidate that
// raises waterBLps without freqHz separating the same way is more likely a
// coiling/geometry artifact than a real reproduction of the target
// mechanism, and is flagged (not auto-rejected - a human should look) via
// the "freqSeparated" column.
//
// Multi-independent-seed-base evaluation from the start of the search, not
// bolted on at confirmation - this project's now three-times-learned lesson
// (tests/worm_speed_calibration, worm_synapse_speed_calibration headers).
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

using Candidate = std::array<float, 2>;  // {gain, tau_w}
enum { kGain = 0, kTauW = 1 };
const Candidate kIdentity = {0.0f, 4.0f};  // gain=0 -> tau_w value is irrelevant (see header)

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.bClassOscillatorGain = c[kGain];
    sim.params.bClassOscillatorTauW = c[kTauW];
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

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution gain tau_w [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kTauW] = static_cast<float>(std::atof(argv[3]));
        const int numBases = argc > 4 ? std::atoi(argv[4]) : 12;
        const int seedsPerBase = argc > 5 ? std::atoi(argv[5]) : 8;
        const int warmupSteps = argc > 6 ? std::atoi(argv[6]) : 300;
        const int measureSteps = argc > 7 ? std::atoi(argv[7]) : 2500;
        std::printf("cand=[gain=%.4f tau_w=%.4f] - %d bases x %d seeds\n", cand[kGain], cand[kTauW], numBases,
                    seedsPerBase);
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

    // ./exe random <trials> <gainLo> <gainHi> <tauWLo> <tauWHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float gainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.01f;
        const float gainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 20.0f;
        const float tauWLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.2f;
        const float tauWHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 40.0f;
        const int seedsPerTrial = argc > 7 ? std::atoi(argv[7]) : 6;
        const unsigned rngSeed = argc > 8 ? static_cast<unsigned>(std::atoi(argv[8])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, gain log-uniform [%.3f,%.3f], tau_w log-uniform [%.3f,%.3f]s, %d "
                    "seeds/trial, rngSeed=%u\n",
                    trials, gainLo, gainHi, tauWLo, tauWHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> gainLogDist(std::log(gainLo), std::log(gainHi));
        std::uniform_real_distribution<float> tauWLogDist(std::log(tauWLo), std::log(tauWHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kGain] = std::exp(gainLogDist(rng));
            cand[kTauW] = std::exp(tauWLogDist(rng));
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
                std::printf("[gain=%.4f tau_w=%.4f] base=%d ", cand[kGain], cand[kTauW], base);
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

    std::printf("Usage:\n  %s random <trials> <gainLo> <gainHi> <tauWLo> <tauWHi> [seedsPerTrial] [rngSeed]\n  %s "
                "distribution gain tau_w [numBases] [seedsPerBase] [warmup] [measure]\n",
                argv[0], argv[0]);
    return 0;
}
