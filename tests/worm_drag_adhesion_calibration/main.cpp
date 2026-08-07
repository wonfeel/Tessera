// tests/worm_drag_adhesion_calibration/main.cpp
//
// Calibration for Params::dragAdhesionGain (WormSim.h/body.hpp/body.cpp) -
// the first attempt in this project's history to touch the FORCE LAW itself
// (WormBody::solve_propulsion's resistive-force-theory drag), rather than a
// neural gain/tempo layered on top of it. Motivation, in order:
//
// 1. Five independent axes that retune the network's own gain/feedback loop
//    (DVA-global mechanosensation, local per-segment mechanosensation,
//    per-class leak/capacitance, synapse-sign, and proprioceptiveGain itself
//    - see tests/worm_mechanosensation_calibration through tests/worm_
//    proprioception_only_calibration) all failed identically to produce
//    water>agar - see tests/worm_local_mechanosensation_calibration's
//    conclusion that this is a structural property of the loaded connectome
//    weights, not a missing search axis.
// 2. tests/worm_cpg_calibration got the TIMING right (realistic 0.1-1+ Hz,
//    matching Fang-Yen et al. 2010's crawl->swim frequency rise) via an
//    external rhythm source, but STILL 0/300 on water>agar - meaning timing
//    was never the (whole) bottleneck.
// 3. Direct arithmetic on that same file's screen data: agar/water's
//    DISTANCE-PER-BEND-CYCLE ratio sits at ~0.14 (agar ~0.0030-0.0032
//    BL/cycle, water ~0.00042-0.00045 BL/cycle) even when CPG-forced
//    frequencies are matched between media - i.e. propulsion EFFICIENCY per
//    cycle, not tempo, is the dominant remaining term. Even a textbook-
//    perfect crawl->swim frequency ratio (~3.4x, Vidal-Gadea et al. 2011)
//    at today's ~0.14 efficiency ratio only reaches ~0.48 - water still
//    slower. Efficiency-per-cycle needs to roughly DOUBLE, not tempo.
// 4. Real agar friction is NOT well-described by simple linear resistive
//    force theory (RFT) at all: Rabets, Backholm, Dalnoki-Veress & Ryu 2014
//    (Biophysical J. 107:1980) directly measured that substrate
//    viscoelasticity gives NONCONSTANT drag coefficients not captured by
//    linear RFT - the worm settles into and adheres to a shallow groove
//    (capillarity + gel plasticity). Sauvage et al. 2011 (elasto-
//    hydrodynamical friction model) independently propose capillary pinning
//    + lubrication-film hydrodynamics + substrate/body elasticity - a
//    qualitatively different force law, not just a bigger c_n coefficient.
//    This project's body.cpp currently uses ONE linear force law (c_n*v) for
//    BOTH media, differing only by the c_n VALUE (40 vs 1.7) - never the
//    LAW's shape. This is the most likely remaining honest lever, and the
//    one this project has never tried.
//
// Mechanism (WormBody::solve_propulsion): c_n is modulated PER SEGMENT by
// max(0, 1 + dragAdhesionGain * |u_k|), where u_k is the segment center's
// velocity from SHAPE CHANGE ONLY (already fully known before the 3x3
// quasi-static force-balance solve runs - see body.cpp's own comment) - NOT
// the segment's total velocity (which includes the unknown rigid-body
// Vx/Vy/w). This keeps the system exactly LINEAR in (Vx,Vy,w), so the
// existing closed-form Cramer's-rule solve is untouched - no iteration, no
// risk of the numerical-stability issues a true velocity-dependent
// nonlinearity would introduce. dragAdhesionGain=0 reproduces the prior
// pure-linear-RFT behavior bit for bit (verified: tests/worm_locomotion
// still PASSes with numbers matching pre-change runs, see commit history).
//
// PRECEDENT AND WARNING (tests/worm_speed_calibration's WormBody::step
// comment): a DIFFERENT prior attempt at touching body mechanics directly -
// resisting the BEND RATE itself (curvature->angle kinematics) by
// drag_normal_ - completely killed water's oscillation (freq collapsed to
// 0.000 Hz, body froze into one static arc) instead of speeding it up, on
// BOTH tested variants. This file's mechanism is DELIBERATELY DIFFERENT: it
// modulates c_n inside the EXISTING translational force balance (same
// physics the drag ratio already lives in), not the curvature->angle
// kinematic step - but the precedent is reason enough to expect this could
// break oscillation too, not assume it will help. Health gates (below) are
// baked in from the first trial, not bolted on after a near-miss.
//
// Sign: NOT predicted directly by the cited literature for this specific
// parameterization - capillary pinning suggests resistance growing with
// effort (gain>0), a lubrication film suggests the opposite (gain<0). Left
// for the search to find, same discipline as cpgAmpLoadSensitivity and
// every other axis this session without a literature-cited sign.
//
// Search scope: dragAdhesionGain ALONE first (this project's "isolate
// before joint" discipline - see tests/worm_bclass_oscillator_calibration
// being isolated before tests/worm_bclass_body_joint_calibration, and tests/
// worm_cpg_calibration adding bodyPoseDecayRate/muscleLeakScale/
// motorLeakScale only after finding each was a NECESSARY companion, not from
// the start). Everything else stays at production defaults (cpgGain=0,
// proprioceptiveGain=4.0, etc.) - if isolated search shows real promise,
// joint search with bodyPoseDecayRate (which shapes how much shape-velocity
// exists to modulate in the first place) is the natural next file.
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried (efficiency>=0.40, coiledRatio>=0.30, freqHz>=
// kMinFreqHz, maxAbsHeadingDelta<=kMaxHeadingDeltaRad). Primary fitness:
// waterBLps > agarBLps with both healthy. Multi-independent-seed-base
// evaluation from the first fitness call (this project's repeatedly learned
// lesson - see tests/worm_speed_calibration's Round 1 single-base-illusion
// account).
//
// RESULT: this is the STRONGEST effect found in this project's entire
// history on the swim>crawl bug - and still falls short. Isolated 1D screen
// (dragAdhesionGain in [-2,2], localMechanoGain=0, one base): strictly
// monotonic in gain - negative hurts water (ratio falls to 0.34-0.43),
// positive helps water dramatically (agar stays ~flat 0.0021-0.0027 BL/s,
// water climbs from baseline 0.00054 to 0.00210 BL/s by gain=2, ratio
// 0.23->0.82). Extended single-base sweep (gain up to 5000): ratio keeps
// climbing but SATURATES - 0.86 (g=3), 0.95 (g=20), 0.98 (g=100-300),
// 0.979-0.982 (g=1000-5000) - genuinely asymptotic, not slowly-still-rising.
// CONFIRMED robust (not a single-base illusion) via distribution mode: g=20,
// 12 independent bases x 8 seeds - water/agar=0.947-0.951 on EVERY base,
// remarkably tight spread, 12/12 healthy on both media, 0/12 water>agar; g=100,
// 8 bases x 6 seeds - water/agar=0.978 consistently, 8/8 healthy, 0/8
// water>agar. Health stays good (efficiency ~0.55-0.60, coiled ~0.65-0.68)
// across this ENTIRE range at localMechanoGain=0 - unlike every other body-
// mechanics experiment in this project's history (tests/worm_speed_
// calibration's reverted bend-rate-drag attempt collapsed water's oscillation
// to 0.000 Hz; this mechanism never does, at any tested gain).
//
// Joint attempt with localMechanoGain (motivated by combining this ~95-98%
// effect with local mechanosensation's OWN independently-confirmed +19%
// water-speed bump, tests/worm_local_mechanosensation_calibration): screen
// grid (gain in {0,5,10,20,30} x local in {0,-0.2,-0.5}) found the
// interaction is NOT additive - it's DESTRUCTIVE. Any nonzero localMechanoGain
// combined with nonzero dragAdhesionGain collapses efficiency to 0.2-0.3
// (below the 0.40 gate) and pushes maxHeadDelta up sharply (still under the
// 0.5 gate, but visibly worse) - a different, new pathology, not the healthy
// near-parity seen with dragAdhesionGain alone. Not pursued further as a
// combination; localMechanoGain should stay at its production default (0.0)
// if shipping dragAdhesionGain alone.
//
// Conclusion: dragAdhesionGain alone gets to ~95-98% of water=agar parity,
// robustly, without ever breaking gait health - genuine progress and the
// closest this project has come - but has a real mathematical ceiling for
// THIS functional form (as gain->infinity, c_n_k is dominated by
// gain*|u_k|, both media's propulsion efficiency saturates toward the same
// infinite-anisotropy limit, and the small residual gap comes from the
// media's differing |u_k| profiles themselves, which this term does not
// touch). Does not cross to water>agar at ANY tested gain, signed or not.
// Ships at dragAdhesionGain=0.0 (unchanged Params default - this file only
// searches, never changes production values). Next honest steps, not yet
// tried: (a) joint search with bodyPoseDecayRate instead of localMechanoGain
// (untested combination, more physically coherent pairing - both are body
// mechanics, not body+neural); (b) a different functional form that doesn't
// share this ceiling (e.g. modulating c_t in the opposite direction
// alongside c_n, though the ceiling's hypothesized cause - residual |u_k|
// difference, not saturated anisotropy - suggests this may not help either);
// (c) accept ~95-98% as the honest ceiling of linear-RFT-adjacent physics and
// pursue the dopamine/serotonin-style adaptive-gain-state mechanism (see
// WORM.md section 6) as a genuinely different lever instead.
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

using Candidate = std::array<float, 2>;  // {dragAdhesionGain, localMechanoGain}
enum { kGain = 0, kLocal = 1 };
const Candidate kIdentity = {0.0f, 0.0f};

// localMechanoGain joined in AFTER the isolated dragAdhesionGain screen found
// a robust (12/12 bases, tight spread) water/agar ratio of ~0.95 at gain=20 -
// SO close that the already-confirmed-real (tests/worm_local_mechanosensation_
// calibration: +19% water speed at gain=-0.5, 12/12 bases, independent
// mechanism - local per-segment stretch feedback, not the propulsion force
// law) effect is a natural candidate to close the remaining ~5% gap, not
// scope creep - this project's own "isolate first, join when isolated shows
// promise" discipline (see tests/worm_cpg_calibration's header).
void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.dragAdhesionGain = c[kGain];
    sim.params.localMechanoGain = c[kLocal];
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
    // ./exe trace [dragNormal] [gain] [steps] [seed] - dumps per-step |u_k|
    // range (min/mean/max over segments) alongside speed/freq context, to
    // calibrate dragAdhesionGain's search range against the ACTUAL scale of
    // shape-velocity in this sim, rather than guessing by analogy (the exact
    // mistake tests/worm_cpg_calibration's header documents making once
    // already with cpgLoadSensitivity).
    if (argc >= 2 && std::string(argv[1]) == "trace") {
        const float dragNormal = argc > 2 ? static_cast<float>(std::atof(argv[2])) : kDragAgar;
        const float gain = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const int steps = argc > 4 ? std::atoi(argv[4]) : 200;
        const int seed = argc > 5 ? std::atoi(argv[5]) : 42;
        WormSim sim("worm_data/celegans_herm.connectome");
        sim.params.dragAdhesionGain = gain;
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = dragNormal;
        std::srand(static_cast<unsigned>(seed));
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        for (int i = 0; i < 300; ++i) sim.step();  // warmup
        float sumU = 0.0f, maxU = 0.0f;
        int nSamples = 0;
        for (int i = 0; i < steps; ++i) {
            sim.step();
            WormSim::Snapshot snap;
            sim.snapshot(snap);
            // |u_k| isn't directly exposed - approximate via per-step centroid
            // displacement rate as a representative shape-velocity scale (the
            // actual per-segment u_k values live inside WormBody::
            // solve_propulsion and aren't part of the public snapshot; this
            // proxy is the same order of magnitude for calibrating a search
            // RANGE, which is all this mode needs to do).
            static float prevCx = 0.0f, prevCy = 0.0f;
            static bool havePrev = false;
            float cx = 0.0f, cy = 0.0f;
            for (std::size_t k = 0; k < snap.pointsX.size(); ++k) { cx += snap.pointsX[k]; cy += snap.pointsY[k]; }
            cx /= static_cast<float>(snap.pointsX.size());
            cy /= static_cast<float>(snap.pointsY.size());
            if (havePrev) {
                const float dt = sim.params.dt.load();
                const float u = std::sqrt((cx - prevCx) * (cx - prevCx) + (cy - prevCy) * (cy - prevCy)) / dt;
                sumU += u;
                maxU = std::max(maxU, u);
                ++nSamples;
            }
            prevCx = cx; prevCy = cy;
            havePrev = true;
        }
        std::printf("dragNormal=%.2f gain=%.4f: centroid-speed proxy mean=%.5f max=%.5f units/s over %d steps\n",
                    dragNormal, gain, nSamples ? sumU / nSamples : 0.0f, maxU, nSamples);
        return 0;
    }

    // ./exe distribution gain local [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kLocal] = static_cast<float>(std::atof(argv[3]));
        const int numBases = argc > 4 ? std::atoi(argv[4]) : 12;
        const int seedsPerBase = argc > 5 ? std::atoi(argv[5]) : 8;
        const int warmupSteps = argc > 6 ? std::atoi(argv[6]) : 300;
        const int measureSteps = argc > 7 ? std::atoi(argv[7]) : 2500;
        std::printf("cand=[dragAdhesionGain=%.4f localMechanoGain=%.4f] - %d bases x %d seeds\n", cand[kGain],
                    cand[kLocal], numBases, seedsPerBase);
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

    // ./exe random <trials> <gainLo> <gainHi> <localLo> <localHi> [seedsPerTrial] [rngSeed]
    // gainLo/gainHi/localLo/localHi may be negative - SIGNED linear ranges
    // (not log-uniform, since sign isn't predicted a priori and 0 must be
    // reachable). localLo/localHi default to the healthy band already
    // established by tests/worm_local_mechanosensation_calibration (roughly
    // [-0.5,0.05] before the gait collapses), padded slightly.
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float gainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const float gainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 30.0f;
        const float localLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : -0.6f;
        const float localHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 0.1f;
        const int seedsPerTrial = argc > 7 ? std::atoi(argv[7]) : 6;
        const unsigned rngSeed = argc > 8 ? static_cast<unsigned>(std::atoi(argv[8])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, dragAdhesionGain linear [%.3f,%.3f], localMechanoGain linear "
                    "[%.3f,%.3f], %d seeds/trial, rngSeed=%u\n",
                    trials, gainLo, gainHi, localLo, localHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> gainDist(gainLo, gainHi);
        std::uniform_real_distribution<float> localDist(localLo, localHi);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kGain] = gainDist(rng);
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
                std::printf("[dragAdhesionGain=%.4f localMechanoGain=%.4f] base=%d ", cand[kGain], cand[kLocal],
                            base);
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

    // Default: quick screen - dragAdhesionGain x localMechanoGain grid, one
    // seed base (shortlist only). Gain range shifted positive (0..30, not
    // signed) - the isolated 1D screen (see header RESULT) found the effect
    // is monotonic and one-signed (gain>0 helps, gain<0 hurts), saturating
    // around water/agar~=0.95 at gain>=20.
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kGains[] = {0.0f, 5.0f, 10.0f, 20.0f, 30.0f};
    const float kLocals[] = {0.0f, -0.2f, -0.5f};
    std::printf("=== Screen (dragAdhesionGain x localMechanoGain grid, %d seeds/point, base=%d) ===\n", kScreenSeeds,
                kSeedBase);
    for (float g : kGains) {
        for (float l : kLocals) {
            const Candidate cand = {g, l};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            std::printf("dragAdhesionGain=%.1f localMechanoGain=%.2f:\n", g, l);
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
