// tests/worm_speed_calibration/main.cpp
//
// Research harness (kept in repo while this investigation is active, same
// convention as tests/worm_chemotaxis_calibration - NOT a one-off).
//
// Goal: the worm crawls/swims dramatically slower than the real animal in
// body-lengths/second (measured live in Demo_worm, see findings below), and
// the direction of the effect is backward for the "Water (swimming)" preset
// (real C. elegans swims FASTER than it crawls; this sim's water preset is
// SLOWER than its agar preset). drag ratio (dragTangent/dragNormal) is
// already at its literature-cited ceiling for agar (WormSim.h) - it is not
// the lever here. This harness investigates whether the emergent BEND
// FREQUENCY of the network+proprioception loop - not drag - is the actual
// speed bottleneck, and whether per-neuron-class leak/capacitance scaling
// (the same lever tests/worm_chemotaxis_calibration explored, and the same
// lever that once cut crawling efficiency 3x when its search ignored
// efficiency - see that file's header) can raise it without breaking gait
// health.
//
// Real C. elegans reference numbers (see WormSim.h dragNormal comment for
// the drag-ratio citations; these are separate, about absolute speed/
// frequency):
//   - Crawling (agar): ~0.1-0.2 body-lengths/s, bend frequency ~0.5 Hz
//     (Fang-Yen et al. 2010, PNAS 107:20323; confinement-crawling speed
//     ~0.1 mm/s corroborates via Backholm et al., PMC3379027, at high
//     confinement - similar geometry to agar-pinned crawling).
//   - Swimming (liquid/M9): ~0.35-0.45 body-lengths/s, bend frequency
//     ~1.7-2 Hz (Fang-Yen et al. 2010 gives 1.7 Hz; Sznitman et al. type
//     M9-buffer measurements report ~0.45 mm/s). Body length ~1mm for both,
//     so mm/s and body-lengths/s are numerically close.
//   The ~3-4x frequency increase (crawl->swim) is what makes swimming NET
//   FASTER despite lower drag anisotropy in liquid - the worm actively
//   compensates. Nothing in this simulation ties bend frequency to drag
//   settings (dragNormal/dragTangent only feed WormBody::solve_propulsion -
//   see body.cpp - never anything upstream in WormSim::step's network side),
//   so the "Water" preset here can only ever get SLOWER than agar, never
//   faster, regardless of how the frequency question below is resolved,
//   unless a frequency-vs-medium coupling is added separately. Out of scope
//   for this harness; noted for whoever picks this up next.
//
// LIVE MEASUREMENT (Demo_worm defaults, no food, centroid path length / 100
// simulated seconds, 16 seeds, see tests/worm_chemotaxis_calibration's
// "displacement" mode which this reuses): agar (dragNormal=40) 0.0033
// body-lengths/s (~30-60x slower than real crawling); water (dragNormal=1.7)
// 0.00065 body-lengths/s (~500-700x slower than real swimming, and slower
// than this sim's OWN agar number - the direction-flip above).
//
// ROUND 1 RESULT - REVERTED, see WormSim.cpp's ctor comment for the full
// story. Short version: the screen-then-confirm search (below, default/
// no-arg mode) found a candidate that looked like a clean 4.85x speedup with
// no efficiency cost on its own final-verification check. It shipped. A
// fresh rerun of the identical search gave a different identity baseline and
// only a 1.44x speedup for the same candidate. Chasing that down (the
// "distribution" mode below) found this network's dynamics are chaotic
// enough that a single seed base's mean - no matter how many seeds drawn
// from it - is not a reliable estimate of a candidate's TRUE (population-
// across-seed-bases) speed. Measured across 20 fresh bases: identity and the
// "winning" candidate have IDENTICAL population-mean speed (0.01079 BL/s
// both), and the candidate has roughly half identity's mean efficiency
// (0.27 vs 0.55) and 4-6x its bend frequency for no distance benefit - worse,
// not better. The underlying diagnosis (emergent cycle ~30-100+s vs real
// ~0.5-2s) stands; this specific fix does not. Any future attempt on this
// axis MUST validate across multiple independent seed BASES (see
// "distribution" mode), not more seeds from one base - that is the mistake
// that shipped this the first time.
//
// UPDATE (later session): re-measured current production state fresh
// (previous numbers above are stale) - "distribution" mode, 20 bases, agar:
// ALL 20 bases now land SLOW (0.00221 BL/s, eff~0.58), zero FAST - the fast
// attractor (~0.0108 BL/s) documented above no longer shows up on any of 20
// fresh bases. Not chased down (would need bisecting the session's other
// changes - e.g. the spatial mean-subtract in WormSim.cpp - against this
// exact harness); noted here so the next person doesn't assume the numbers
// above are still current. Water (dragNormal=1.7): 0.00052 BL/s, same
// eff~0.58 healthy oscillation (freq 0.008Hz) - i.e. still SLOWER than agar,
// backwards from real C. elegans (swimming > crawling). Root cause: curvature
// -> angles_ in WormBody::step is purely kinematic, never resisted by
// drag_normal_, so bend frequency can't respond to medium at all (see body.cpp
// comment at the top of WormBody::step). Tried the direct fix - resist the
// bend-rate itself by drag_normal_ relative to the default Agar preset (40.0),
// same anisotropic-drag philosophy already used for translation - two
// variants: (a) scale only the driven term, (b) scale both driven term and
// the neutral-pose decay together (equivalent to rescaling the mechanical
// step's own time by 1/bendResistance, so drive:decay ratio is preserved).
// BOTH keep agar bit-for-bit identical (bendResistance==1 at the reference
// point, confirmed via this file's own measure/distribution modes and tests/
// worm_locomotion regression - unaffected). BOTH degrade water instead of
// speeding it up: freq collapses to 0.000 Hz (no oscillation at all, body
// settles into one static arc) instead of increasing - variant (a) at
// eff~0.73/speed 0.00045, variant (b) at eff~0.64/speed 0.00047, both worse
// than the unfixed 0.00052 and neither actually undulating. Reverted (see
// body.cpp - kept only as a documented, commented-out-in-spirit "tried and
// reverted" note there, no dead code left behind). Conclusion: same pattern
// as every other calibration attempt in this project's history - this raw,
// uncalibrated network holds its healthy oscillating gait only in a narrow
// operating window, and directly perturbing the mechanical side of the
// network<->body loop (rather than tuning gains already inside that loop)
// knocks it into a non-oscillating absorbing state instead of smoothly
// retuning the frequency. A real fix here is a calibration-SEARCH problem
// (screen many candidate couplings/strengths, confirm across many seed
// bases, gate on health+efficiency - same discipline as the leak/capacitance
// search above), not a single physically-motivated formula - budget for that
// before attempting this again.
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

constexpr int kNumParams = 7;
enum ParamIdx {
    kLeakIP = 0, kLeakP = 1, kLeakPO = 2,
    kCapIP = 3, kCapP = 4, kCapPO = 5, kCapO = 6,
};
using Candidate = std::array<float, kNumParams>;
const Candidate kIdentity = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    connectome::Network& net = sim.network();
    net.scale_type_params(connectome::NeuronType::InputProcessing, c[kLeakIP], c[kCapIP]);
    net.scale_type_params(connectome::NeuronType::Processing, c[kLeakP], c[kCapP]);
    net.scale_type_params(connectome::NeuronType::ProcessingOutput, c[kLeakPO], c[kCapPO]);
    net.scale_type_params(connectome::NeuronType::Output, 1.0f, c[kCapO]);  // Output leak is a dead parameter
}

void printCandidate(const Candidate& c) {
    std::printf("[leakIP=%.3f leakP=%.3f leakPO=%.3f capIP=%.3f capP=%.3f capPO=%.3f capO=%.3f]", c[0], c[1], c[2],
                c[3], c[4], c[5], c[6]);
}

constexpr float kHexSpacing = 36.0f;
// Deliberately much larger than tests/worm_chemotaxis_calibration's 28x20:
// that field is sized for short-radius (180-unit) chemotaxis trials where
// the worm never gets near the edge. This harness searches capacitance
// scales that can make the worm several times faster, over longer (200s)
// windows - large enough that a small field's containBody() wall-bounce
// (reflects heading on contact) would kink the trajectory and contaminate
// the efficiency measurement with wall-bounce artifacts, not genuine gait
// incoherence. 200x150 gives ~7200x5400 world-unit half-extents, far beyond
// anything reachable in these trials.
constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kBodyLength = 576.0f;  // kNumSegments(24) * segment_length(24.0), see WormSim.cpp ctor

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;       // net displacement / path length, 0..1
    float minCoiledRatio = 1e9f;   // bbox diag / arc length, post-transient - health guard
    float freqHz = 0.0f;           // dominant bend frequency at a mid-body position
    bool healthy = true;
};

// Runs one seed: warms up (discarded, lets transients settle), then measures
// centroid path length AND bend frequency (zero-crossing rate of the actual
// post-physics deviation signal fed to the body, at a fixed mid-body
// position) over the same measurement window. No food - this is about raw
// gait speed, not chemotaxis (a different, already-explored axis).
Measurement runTrial(const Candidate& cand, int seed, float dragTangent, float dragNormal, int warmupSteps,
                      int measureSteps, int freqPosition) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    if (dragTangent >= 0.0f) sim.params.dragTangent = dragTangent;
    if (dragNormal >= 0.0f) sim.params.dragNormal = dragNormal;
    std::srand(static_cast<unsigned>(seed));  // after construction - ctor's own srand(time()) would clobber this
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
    // Shape (coiled-ratio) bbox is resampled FRESH every 50 steps - an
    // earlier version of this loop accumulated bx0/bx1/by0/by1 across the
    // WHOLE run without resetting, which measures how far the body's
    // trajectory roamed over 200s (always grows with speed, easily >1.0),
    // not whether the body's own CURRENT shape is tightly coiled at any
    // instant (bbox-diagonal/arc-length, same definition as
    // tests/worm_locomotion and tests/worm_chemotaxis_calibration - always
    // <=1.0 for a fixed-length chain). Track the MINIMUM instantaneous ratio
    // seen, to catch any moment of tight coiling.
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

    const float netDisp = std::sqrt((prevX - startX) * (prevX - startX) + (prevY - startY) * (prevY - startY));
    const float measureSeconds = static_cast<float>(measureSteps) * dt;
    m.efficiency = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
    m.bodyLengthsPerSec = static_cast<float>(pathLen) / measureSeconds / kBodyLength;
    // Frequency from zero-crossing rate: one full oscillation = 2 crossings.
    m.freqHz = static_cast<float>(zeroCrossings) / 2.0f / measureSeconds;
    return m;
}

struct AggregateResult {
    float meanBLps = 0.0f, stderrBLps = 0.0f, meanFreqHz = 0.0f, meanEfficiency = 0.0f, minCoiledRatio = 1e9f;
    bool allHealthy = true;
};

AggregateResult evaluate(const Candidate& cand, int numSeeds, int seedBase, float dragTangent, float dragNormal,
                          int warmupSteps, int measureSteps, int freqPosition = 12) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int s = 0; s < numSeeds; ++s) {
        const Measurement m = runTrial(cand, seedBase + s, dragTangent, dragNormal, warmupSteps, measureSteps,
                                        freqPosition);
        ar.minCoiledRatio = std::min(ar.minCoiledRatio, m.minCoiledRatio);
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

// Health/coherence gates - a candidate that fails these is rejected outright
// regardless of speed (fitness = -1e6), same principle as tests/worm_
// chemotaxis_calibration's coiled-ratio guard: raw distance covered isn't
// useful if the worm gets there by degenerating into something that isn't a
// crawl anymore. minEfficiency=0.40 allows some give below the uncalibrated
// baseline's ~0.52 (net displacement/path length - directional persistence,
// NOT the same "efficiency" as chemotaxis's food-distance metric) but
// firmly excludes the kind of collapse seen manually at capacitance=0.25
// (efficiency 0.03 - covers distance by looping, not crawling).
constexpr float kMinEfficiency = 0.40f;
constexpr float kMinCoiledRatio = 0.30f;

float fitnessOf(const AggregateResult& ar) {
    if (!ar.allHealthy || ar.minCoiledRatio < kMinCoiledRatio || ar.meanEfficiency < kMinEfficiency) return -1e6f;
    return ar.meanBLps;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe measure leakIP leakP leakPO capIP capP capPO capO [dragTangent] [dragNormal] [numSeeds] [warmupSteps] [measureSteps] [seedBase]
    if (argc >= 2 && std::string(argv[1]) == "measure") {
        Candidate cand = kIdentity;
        for (int k = 0; k < kNumParams && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const float dragTangent = argc > 9 ? static_cast<float>(std::atof(argv[9])) : -1.0f;
        const float dragNormal = argc > 10 ? static_cast<float>(std::atof(argv[10])) : -1.0f;
        const int numSeeds = argc > 11 ? std::atoi(argv[11]) : 12;
        const int warmupSteps = argc > 12 ? std::atoi(argv[12]) : 400;
        const int measureSteps = argc > 13 ? std::atoi(argv[13]) : 1600;
        const int seedBase = argc > 14 ? std::atoi(argv[14]) : 271828182;
        const AggregateResult ar = evaluate(cand, numSeeds, seedBase, dragTangent, dragNormal, warmupSteps,
                                             measureSteps);
        std::printf("cand="); printCandidate(cand);
        std::printf(" drag=(%s,%s) seeds=%d warmup=%d measure=%d\n",
                    dragTangent >= 0.0f ? std::to_string(dragTangent).c_str() : "default",
                    dragNormal >= 0.0f ? std::to_string(dragNormal).c_str() : "default", numSeeds, warmupSteps,
                    measureSteps);
        std::printf("  speed=%.5f BL/s  freq=%.3f Hz  efficiency=%.3f  minCoiledRatio=%.3f  allHealthy=%d\n",
                    ar.meanBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio, ar.allHealthy ? 1 : 0);
        return 0;
    }

    // TEMPORARY diagnostic - raw waveform dump to sanity-check the frequency
    // measurement above before trusting it. ./exe trace [dragN] [steps]
    if (argc >= 2 && std::string(argv[1]) == "trace") {
        const float dragNormal = argc > 2 ? static_cast<float>(std::atof(argv[2])) : -1.0f;
        const int steps = argc > 3 ? std::atoi(argv[3]) : 400;
        const float propGain = argc > 4 ? static_cast<float>(std::atof(argv[4])) : -1.0f;
        const float gapGain = argc > 5 ? static_cast<float>(std::atof(argv[5])) : -1.0f;
        WormSim sim("worm_data/celegans_herm.connectome");
        if (dragNormal >= 0.0f) { sim.params.dragTangent = 1.0f; sim.params.dragNormal = dragNormal; }
        if (propGain >= 0.0f) sim.params.proprioceptiveGain = propGain;
        if (gapGain >= 0.0f) sim.params.gapGain = gapGain;
        std::srand(999u);
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        const float dt = sim.params.dt.load();
        for (int i = 0; i < steps; ++i) {
            sim.step();
            const auto& dev = sim.lastCurvatureDeviation();
            if (i % 10 == 0) {
                std::printf("t=%.2f  dev[6]=%.4f dev[12]=%.4f dev[18]=%.4f\n", i * dt,
                            dev.size() > 6 ? dev[6] : 0.0f, dev.size() > 12 ? dev[12] : 0.0f,
                            dev.size() > 18 ? dev[18] : 0.0f);
            }
        }
        return 0;
    }

    // TEMPORARY diagnostic - characterizes whether a candidate's speed is
    // genuinely BIMODAL across different seed bases (a "slow" regime some
    // bases land in, a "fast" regime others do) rather than unimodal-noisy.
    // Round 1's shipped winner looked like a clean 4.85x win against an
    // identity baseline of ~0.0022 BL/s (base=0, base=900000000) - but fresh
    // spot-checks of identity at OTHER bases (111111111, 222222222, and a
    // full search rerun's internal bases) landed consistently around
    // ~0.0107-0.0108, a completely different number for the SAME candidate.
    // ./exe distribution leakIP...capO [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        for (int k = 0; k < kNumParams && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 9 ? std::atoi(argv[9]) : 12;
        const int seedsPerBase = argc > 10 ? std::atoi(argv[10]) : 8;
        const int warmupSteps = argc > 11 ? std::atoi(argv[11]) : 300;
        const int measureSteps = argc > 12 ? std::atoi(argv[12]) : 2500;
        std::printf("cand="); printCandidate(cand);
        std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        double sumAll = 0.0;
        int slowCount = 0, fastCount = 0, ambiguousCount = 0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult ar = evaluate(cand, seedsPerBase, base, 1.0f, 40.0f, warmupSteps, measureSteps);
            const char* regime = ar.meanBLps < 0.005f ? "SLOW" : (ar.meanBLps > 0.008f ? "FAST" : "mid");
            if (ar.meanBLps < 0.005f) ++slowCount; else if (ar.meanBLps > 0.008f) ++fastCount; else ++ambiguousCount;
            std::printf("  base=%d: speed=%.5f+/-%.5f BL/s freq=%.3fHz eff=%.3f [%s]\n", base, ar.meanBLps,
                        ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, regime);
            std::fflush(stdout);
            sumAll += ar.meanBLps;
        }
        std::printf("population mean (mean-of-base-means): %.5f BL/s over %d bases - slow=%d fast=%d mid=%d\n",
                    sumAll / numBases, numBases, slowCount, fastCount, ambiguousCount);
        return 0;
    }

    // Default (no args): the screen-then-confirm search that produced
    // round 1's now-reverted result - KNOWN UNRELIABLE AS A VALIDATION STEP,
    // kept for the same reason tests/worm_chemotaxis_calibration keeps its
    // own history: honest record, and the search STRUCTURE (screen wide on
    // modest seeds, confirm top survivors on more, select by lower-
    // confidence-bound not raw mean) is still reasonable - only the FINAL
    // single-seed-base verification step is the proven-unreliable part. If
    // this axis is revisited, run whatever it finds through "distribution"
    // (multiple independent seed bases) before trusting it, per the header.
    // Bound: 0.02-2.0x log-uniform per class, independently - wider than
    // tests/worm_chemotaxis_calibration's final 0.6-1.6x because a manual
    // uniform sweep (this file's header) showed a few-fold change does
    // essentially nothing to speed here; the effect only appears past a
    // 4-20x reduction. Not a biologically-measured range (unlike the drag-
    // ratio citations elsewhere in this project) - a search bound only.
    std::printf("=== Baseline (identity) ===\n");
    constexpr float kDragTangent = 1.0f, kDragNormal = 40.0f;  // agar, this project's default preset
    constexpr int kWarmup = 300, kMeasure = 2500;  // 15s + 125s
    const AggregateResult idBase = evaluate(kIdentity, 8, 0, kDragTangent, kDragNormal, kWarmup, kMeasure);
    std::printf("identity: speed=%.5f+/-%.5f BL/s freq=%.3fHz efficiency=%.3f coiled=%.3f healthy=%d\n\n",
                idBase.meanBLps, idBase.stderrBLps, idBase.meanFreqHz, idBase.meanEfficiency, idBase.minCoiledRatio,
                idBase.allHealthy ? 1 : 0);

    std::printf("=== Phase 1: screen (0.02-2.0x log-uniform, per class) ===\n");
    constexpr float kBoundLo = 0.02f, kBoundHi = 2.0f;
    constexpr int kScreenCandidates = 120;
    constexpr int kScreenSeeds = 6;
    constexpr int kTopK = 6;

    std::mt19937 rng(4242);
    std::uniform_real_distribution<float> logUniform(std::log(kBoundLo), std::log(kBoundHi));

    struct Scored { Candidate cand; float fitness; AggregateResult ar; };
    std::vector<Scored> top;

    for (int i = 0; i < kScreenCandidates; ++i) {
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = std::exp(logUniform(rng));
        const AggregateResult ar = evaluate(cand, kScreenSeeds, i * 1000, kDragTangent, kDragNormal, kWarmup,
                                             kMeasure);
        const float fit = fitnessOf(ar);
        if (top.size() < static_cast<std::size_t>(kTopK) || fit > top.back().fitness) {
            top.push_back({cand, fit, ar});
            std::sort(top.begin(), top.end(), [](const Scored& a, const Scored& b) { return a.fitness > b.fitness; });
            if (top.size() > static_cast<std::size_t>(kTopK)) top.pop_back();
        }
        if (i % 10 == 0 || i == kScreenCandidates - 1) {
            std::printf("screen %3d/%d: top speed so far = %.5f BL/s\n", i, kScreenCandidates,
                        top.empty() ? -1.0f : top.front().fitness);
            std::fflush(stdout);
        }
    }

    std::printf("\n=== Phase 2: re-confirm top %d with more seeds ===\n", kTopK);
    constexpr int kConfirmSeeds = 16;
    Candidate winner = kIdentity;
    float winnerLcb = -1e9f;
    for (std::size_t i = 0; i < top.size(); ++i) {
        const AggregateResult ar = evaluate(top[i].cand, kConfirmSeeds, 777000000 + static_cast<int>(i) * 1000,
                                             kDragTangent, kDragNormal, kWarmup, kMeasure);
        const float fit = fitnessOf(ar);
        const float lcb = fit > -1e5f ? (ar.meanBLps - ar.stderrBLps) : -1e6f;
        std::printf("candidate %zu: screenSpeed=%.5f confirmedSpeed=%.5f+/-%.5f lcb=%.5f eff=%.3f coiled=%.3f "
                    "healthy=%d ", i, top[i].fitness, ar.meanBLps, ar.stderrBLps, lcb, ar.meanEfficiency,
                    ar.minCoiledRatio, ar.allHealthy ? 1 : 0);
        printCandidate(top[i].cand);
        std::printf("\n");
        std::fflush(stdout);
        if (fit > -1e5f && lcb > winnerLcb) { winnerLcb = lcb; winner = top[i].cand; }
    }

    std::printf("\n=== Final verification (24 seeds, independent seed base) ===\n");
    const AggregateResult finalId = evaluate(kIdentity, 24, 900000000, kDragTangent, kDragNormal, kWarmup, kMeasure);
    const AggregateResult finalWin = evaluate(winner, 24, 900000000, kDragTangent, kDragNormal, kWarmup, kMeasure);
    std::printf("identity : speed=%.5f+/-%.5f BL/s freq=%.3fHz efficiency=%.3f coiled=%.3f healthy=%d\n",
                finalId.meanBLps, finalId.stderrBLps, finalId.meanFreqHz, finalId.meanEfficiency,
                finalId.minCoiledRatio, finalId.allHealthy ? 1 : 0);
    std::printf("winner   : speed=%.5f+/-%.5f BL/s freq=%.3fHz efficiency=%.3f coiled=%.3f healthy=%d  ",
                finalWin.meanBLps, finalWin.stderrBLps, finalWin.meanFreqHz, finalWin.meanEfficiency,
                finalWin.minCoiledRatio, finalWin.allHealthy ? 1 : 0);
    printCandidate(winner);
    std::printf("\n");
    if (finalWin.meanBLps > 1e-6f) {
        std::printf("speedup vs identity: %.2fx\n", finalWin.meanBLps / std::max(1e-6f, finalId.meanBLps));
    }
    return 0;
}
