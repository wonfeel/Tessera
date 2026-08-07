// tests/worm_proprioception_only_calibration/main.cpp
//
// Cheap, isolated closing-the-loop check on the swim>crawl speed bug (see
// tests/worm_speed_calibration through tests/worm_cpg_calibration for the
// project's full history). Boyle, Berri & Cohen 2012 (Frontiers Comp.
// Neurosci., PMC3296079 - already cited by WormSim.cpp's applyProprioception)
// reproduce the ENTIRE crawl/swim/intermediate gait transition with NO
// pacemaker at all - "no modulatory mechanism, except via the proprioceptive
// response to the physical environment." This project's CPG (tests/worm_cpg_
// calibration) was added on the assumption that proprioception alone is
// insufficient here - but that assumption was never directly, cheaply tested
// on its own: Params::proprioceptiveGain only ever appears as a manual CLI
// override in tests/worm_speed_calibration's "trace" diagnostic (a raw
// waveform dump, not a health-gated search), never as a free dimension in any
// health-gated search. Params::localMechanoGain DID get its own dedicated
// search (tests/worm_local_mechanosensation_calibration) - real but
// insufficient alone (+19% water speed, 0/12 bases water>agar, healthy range
// found to be narrow: roughly [-0.5, 0.05], collapsing to a frozen static arc
// just past it). Neither was ever searched JOINTLY with proprioceptiveGain
// itself - they are two halves of the same real mechanism (B/D-class motor
// neurons acting as their own local stretch receptors, per Wen et al. 2012
// Neuron / Yeon et al. 2018 Cell, cited in worm_local_mechanosensation_
// calibration's header) and might interact even though each alone (mechano-
// gain axes) or was never tried (proprioceptiveGain itself) hit a wall.
//
// Expectations, set honestly low: tests/worm_mechanosensation_calibration
// (DVA global), tests/worm_local_mechanosensation_calibration (local per-
// segment), tests/worm_speed_leak_calibration (7-param leak/capacitance,
// 0/480), and tests/worm_speed_calibration's Round 1 (leak/capacitance,
// reverted) are FOUR independent axes that all tried to retune this
// connectome's OWN emergent oscillation via gain knobs - all four hit the
// same wall, stated plainly in worm_local_mechanosensation_calibration's own
// conclusion: "this connectome's leaky-integrator network, as currently
// loaded, holds a genuinely narrow, rigid oscillation regime (~125s period)
// that resists being retuned by any external perturbation tried so far... a
// structural property of the loaded leak/capacitance/synaptic-weight values
// themselves." proprioceptiveGain is the SAME class of lever (a gain knob on
// the network's own feedback loop, not an external rhythm source like the
// CPG), so there is no strong reason to expect it escapes that wall where its
// four siblings didn't. Separately: tests/worm_cpg_calibration's own screen
// data shows agar and water's DISTANCE-PER-BEND-CYCLE ratio sits at ~0.14
// even when CPG-forced frequencies are matched between media - i.e. the
// per-cycle propulsion efficiency gap (not timing) is the dominant term, and
// nothing in this file touches propulsion efficiency (that is body.cpp
// physics, a separate, not-yet-attempted axis). This file exists to CLOSE
// THE LOOP cheaply before spending effort there, not because it is expected
// to win outright.
//
// Search scope (deliberately narrow - just the B-class stretch-receptor
// mechanism, no CPG, no body pose decay retiming, no muscle/motor leak):
//   proprioceptiveGain in [0.5, 20] log-uniform - no literature unit (this
//     reduced model's own feedback-current scale); default is 4.0, already
//     empirically validated not to reverse locomotion direction across 80
//     independent runs (see WormSim.h comment) - range spans well below and
//     ~5x above that known-safe point.
//   proprioceptiveOffset in [1, 16] linear - stretch-receptor window width in
//     body positions "locally and posteriorly" (Boyle/Berri/Cohen's own
//     phrase); default 4.0; clamped to an int in [1,24] by applyProprioception
//     itself, so the upper bound here (16, two-thirds of the body) is a
//     sanity ceiling, not a hard constraint from the code.
//   localMechanoGain in [-0.6, 0.1] linear (NOT log-uniform - the known-good
//     region straddles zero) - respects the healthy band already established
//     by tests/worm_local_mechanosensation_calibration (roughly [-0.5, 0.05]
//     before the gait collapses to a frozen arc), padded slightly rather than
//     re-discovered from scratch.
//
// cpgGain is never touched (stays at its Params default, 0.0 = off) - this
// file's whole point is testing proprioception WITHOUT the CPG.
//
// RESULT: screen (15-point grid, one base) - water>agar in 0/15, matching
// every other axis's screen behavior in this project. Notable: freqHz==0.0000
// (frozen static arc, unhealthy) for propGain<=2.0 regardless of
// localMechanoGain - the same "rigid regime" collapse tests/worm_local_
// mechanosensation_calibration already documented; health only starts at
// propGain>=4.0, where freq reaches a still-tiny 0.004-0.008Hz (vs real
// C. elegans' 0.5-2Hz). random search (200 trials, propGain log-uniform
// [0.5,20], propOffset linear [1,16], localMechanoGain linear [-0.6,0.1], 5
// seeds/trial, rngSeed=7): 0/200 healthy candidates with water>agar on their
// own fresh base. Conclusion: this is the FIFTH independent axis in this
// project's history (after DVA-global mechanoGain, local per-segment
// mechanoGain, per-class leak/capacitance, and synapse-sign) that tries to
// retune this connectome's own emergent gain/feedback loop and fails to
// produce water>agar - consistent with tests/worm_local_mechanosensation_
// calibration's conclusion that this is a structural property of the loaded
// leak/capacitance/synaptic-weight values, not a missing search axis.
// Boyle/Berri/Cohen 2012's proprioception-only mechanism is confirmed
// present and real in this codebase (it does produce a traveling wave, and
// localMechanoGain does have a genuine small effect - see tests/worm_local_
// mechanosensation_calibration) but does not, on its own, close this gap on
// THIS connectome's raw (uncalibrated) weights - closing the loop this file
// set out to close. Ships at proprioceptiveGain=4.0/proprioceptiveOffset=4.0/
// localMechanoGain=0.0 (unchanged Params defaults - this file only searches,
// never changes production values). Separately: tests/worm_cpg_calibration's
// own screen data shows agar/water's distance-per-bend-cycle ratio sits at
// ~0.14 even at matched CPG-forced frequency - i.e. propulsion efficiency
// per cycle, not gain/timing, is the dominant remaining term, and no
// gain-retuning axis (this one included) can address that; see WORM.md
// section 6 for the current honest status and the two remaining untried
// levers (agar force-law physics; dopamine/serotonin-style adaptive gain
// state through the connectome's own unused ADE/PDE neurons).
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried (efficiency>=0.40, coiledRatio>=0.30, freqHz>=kMinFreqHz,
// maxAbsHeadingDelta<=kMaxHeadingDeltaRad) - baked in from the start, not
// bolted on after a near-miss like tests/worm_synapse_speed_calibration's.
// Primary fitness: waterBLps > agarBLps with both healthy. Multi-independent-
// seed-base evaluation from the first fitness call (this project's repeatedly
// learned lesson - see tests/worm_speed_calibration's Round 1 account of a
// single-base illusion that shipped and had to be reverted).
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

using Candidate = std::array<float, 3>;  // {proprioceptiveGain, proprioceptiveOffset, localMechanoGain}
enum { kGain = 0, kOffset = 1, kLocal = 2 };
const Candidate kIdentity = {4.0f, 4.0f, 0.0f};  // current shipped Params defaults

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.proprioceptiveGain = c[kGain];
    sim.params.proprioceptiveOffset = c[kOffset];
    sim.params.localMechanoGain = c[kLocal];
    sim.params.cpgGain = 0.0f;  // explicit - this file tests proprioception WITHOUT the CPG, see header
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
    // ./exe distribution gain offset local [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kOffset] = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) cand[kLocal] = static_cast<float>(std::atof(argv[4]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 12;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 8;
        const int warmupSteps = argc > 7 ? std::atoi(argv[7]) : 300;
        const int measureSteps = argc > 8 ? std::atoi(argv[8]) : 2500;
        std::printf("cand=[propGain=%.4f propOffset=%.4f localMechano=%.4f] - %d bases x %d seeds\n", cand[kGain],
                    cand[kOffset], cand[kLocal], numBases, seedsPerBase);
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
            std::printf("  water>agar: %s   freq(water)>freq(agar): %s\n", waterFaster ? "YES" : "no",
                        freqSeparated ? "YES" : "no");
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

    // ./exe random <trials> <gainLo> <gainHi> <offsetLo> <offsetHi> <localLo> <localHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float gainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.5f;
        const float gainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 20.0f;
        const float offsetLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 1.0f;
        const float offsetHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 16.0f;
        const float localLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : -0.6f;
        const float localHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 0.1f;
        const int seedsPerTrial = argc > 9 ? std::atoi(argv[9]) : 5;
        const unsigned rngSeed = argc > 10 ? static_cast<unsigned>(std::atoi(argv[10])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, propGain log-uniform [%.2f,%.2f], propOffset linear [%.2f,%.2f], "
                    "localMechano linear [%.2f,%.2f], %d seeds/trial, rngSeed=%u\n",
                    trials, gainLo, gainHi, offsetLo, offsetHi, localLo, localHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> gainLogDist(std::log(gainLo), std::log(gainHi));
        std::uniform_real_distribution<float> offsetDist(offsetLo, offsetHi);
        std::uniform_real_distribution<float> localDist(localLo, localHi);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kGain] = std::exp(gainLogDist(rng));
            cand[kOffset] = offsetDist(rng);
            cand[kLocal] = localDist(rng);
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
                std::printf("[propGain=%.3f propOffset=%.3f localMechano=%.3f] base=%d ", cand[kGain], cand[kOffset],
                            cand[kLocal], base);
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

    // Default: quick screen - vary proprioceptiveGain and localMechanoGain on
    // a small grid, proprioceptiveOffset held at its default (4), one seed
    // base (shortlist only - confirm anything promising with 'distribution'
    // or search jointly with 'random' across many bases before trusting it).
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kGains[] = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f};
    const float kLocals[] = {-0.5f, -0.2f, 0.0f};
    std::printf("=== Screen (proprioceptiveGain x localMechanoGain grid, proprioceptiveOffset=4.0 fixed, %d "
                "seeds/point, base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float g : kGains) {
        for (float l : kLocals) {
            const Candidate cand = {g, 4.0f, l};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            std::printf("propGain=%.1f localMechano=%.2f:\n", g, l);
            printAgg("agar", agar);
            printAgg("water", water);
            const bool waterFaster = isHealthy(agar) && isHealthy(water) && water.meanBLps > agar.meanBLps;
            std::printf("  water>agar: %s\n", waterFaster ? "YES" : "no");
            std::fflush(stdout);
        }
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising point with "
                "'distribution' or search jointly with 'random' across many seed bases before believing it (see "
                "header).\n");
    return 0;
}
