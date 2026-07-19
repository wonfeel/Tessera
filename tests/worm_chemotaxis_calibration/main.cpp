// tests/worm_chemotaxis_calibration/main.cpp
//
// Research harness (long-running, meant to stay around while this
// investigation is active - unlike the one-off diagnostics elsewhere in
// this history, delete only once a calibration is either adopted into
// production or the approach is abandoned).
//
// Goal: get real, directed chemotaxis (worm net progress toward food that
// is measurably BETTER than its own undirected wandering) out of this
// reduced connectome model. tests/worm_locomotion's header documents an
// exhaustive prior investigation that ruled out global-scalar gain tuning
// (chemGain/gapGain/leakScale/gradientGain/foodMaxConcentration) as a
// strategy, fixed a real saturation bug (foodMaxConcentration) without
// unlocking directed behavior, and explicitly reverted a working-but-
// illegitimate fix (direct heading assignment from neuron state) on
// principle. The two honest options left on the table there:
//   (1) phase-gate sensory drive by an existing locomotion-phase/CPG signal
//   (2) real per-neuron-CLASS (not per-neuron - too many free parameters)
//       leak/capacitance calibration, found via a small evolutionary search
//       against this project's own paired with/without-food metric
// This harness implements (2). All 401 neurons currently get identical
// NeuronParams{leak=1, rest=0, capacitance=1} regardless of type (see
// load_connectome) - real neurons don't share one universal time constant,
// and Network::scale_type_params (added alongside this harness) is the
// first way to vary it, per NeuronType, without exploding the parameter
// count.
//
// Free parameters (7, not 401): Input has no dynamics at all (pure
// passthrough) and Output's own leak is force-zeroed in Network::step
// regardless of NeuronParams (see the comment there) - both dead
// parameters, excluded:
//   leakScale  x {InputProcessing, Processing, ProcessingOutput}
//   capScale   x {InputProcessing, Processing, ProcessingOutput, Output}
//
// Metric (same methodology as the original investigation, same pitfalls
// deliberately avoided):
//   - PAIRED same-seed comparison: for each seed, run once with food and
//     once without, identical RNG stream (std::rand reseeded to the same
//     value right before each run - WormSim's own ctor-time srand(time())
//     is irrelevant noise, overwritten before any rand() is actually
//     consumed) so the only difference is the sensory presence of food.
//   - Food placed at a DIFFERENT ANGLE per seed (golden-angle spacing,
//     offset away from 0) - the original investigation's first version
//     placed food due +x, exactly the worm's initial heading, and that
//     alone produced a fake "effect" identical with/without food.
//   - Food within actual sensing/reachable range (R=180 world units - the
//     original investigation also once placed food 400 units out against a
//     70-unit deposit radius and got a null result for the boring reason
//     that the worm could not physically get there, not a network problem).
//   - effect = distance-to-food(without food) - distance-to-food(with food)
//     averaged over seeds; positive = real attraction beyond whatever the
//     worm's own undirected wandering would have covered anyway.
//   - Health guard: any seed that goes NaN, leaves the field, or coils into
//     a tight knot (same coiled-ratio check as tests/worm_locomotion)
//     rejects the whole candidate outright (fitness = -1e6) - a parameter
//     set is not useful if it "chemotaxes" by breaking locomotion itself.
//
// Search: simple (1+lambda) evolution strategy, log-normal mutation (keeps
// scale factors positive, symmetric in ratio space around 1.0 = today's
// uncalibrated baseline), adaptive step size (grow on improvement, shrink
// on stagnation). Deliberately NOT touching chemGain/gapGain/bodyGain/etc -
// those were the "global-scalar" axis already exhausted; this is a
// genuinely different axis (per-class time constants).
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
// Indices into the candidate array.
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

constexpr float kHexSpacing = 36.0f;
constexpr int kFieldCols = 28, kFieldRows = 20;
// Not const - overridable from argv in standalone mode, to check whether the
// small per-run effect compounds into something bigger/more visible over a
// much longer run than the search itself used.
int g_trialSteps = 5000;
constexpr float kFoodRadius = 180.0f;
constexpr float kGoldenAngleDeg = 137.50776f;

glm::vec2 foodPositionForSeed(glm::vec2 start, int seed) {
    const float angleDeg = 45.0f + static_cast<float>(seed) * kGoldenAngleDeg;  // offset away from initial heading 0
    const float angle = angleDeg * 3.14159265f / 180.0f;
    return start + kFoodRadius * glm::vec2(std::cos(angle), std::sin(angle));
}

// Runs one trial (with or without food) for a given seed+candidate. Returns
// final distance-to-food and whether the run stayed healthy throughout.
struct TrialResult {
    float finalDistToFood = 0.0f;
    bool healthy = true;
};

TrialResult runTrial(const Candidate& cand, int seed, bool withFood, float gradientGainScale = 1.0f,
                      float noiseScale = 1.0f) {
    TrialResult result;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    // Environmental/sensory knobs - default 4.0/3.0 (see WormSim::Params) -
    // scaled on top of whatever per-class calibration `cand` applies. Left
    // at 1.0x (no-op) by every call site except the round-5 env sweep.
    sim.params.gradientGain = sim.params.gradientGain.load() * gradientGainScale;
    sim.params.spontaneousNoise = sim.params.spontaneousNoise.load() * noiseScale;
    // Same seed for both conditions (with/without food) is the entire point
    // of the paired design - WormSim's own ctor-time srand(time()) is
    // irrelevant, overwritten here before any rand() is actually consumed.
    std::srand(static_cast<unsigned>(seed));

    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
    const glm::vec2 boundsMax = HexGrid::worldPos(kFieldCols - 1, kFieldRows - 1, kHexSpacing);
    const glm::vec2 start = boundsMax * 0.5f;  // matches WormSim::setBounds' center (boundsMin is (0,0))
    const glm::vec2 food = foodPositionForSeed(start, seed);
    if (withFood) sim.depositFood(food, 5.0f);

    WormSim::Snapshot snap;
    float minCoiledRatio = 1e9f;
    constexpr float kArcLen = 576.0f;  // kNumSegments(24) * segment_length(24.0), see WormSim.cpp ctor

    for (int i = 0; i < g_trialSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float x = snap.pointsX[0], y = snap.pointsY[0];
        if (std::isnan(x) || std::isnan(y)) {
            result.healthy = false;
            break;
        }
        bool outOfBounds = false;
        for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
            constexpr float kEps = 1.0f;
            if (snap.pointsX[p] < -kEps || snap.pointsX[p] > boundsMax.x + kEps || snap.pointsY[p] < -kEps ||
                snap.pointsY[p] > boundsMax.y + kEps) {
                outOfBounds = true;
                break;
            }
        }
        if (outOfBounds) {
            result.healthy = false;
            break;
        }
        if (i > 200 && i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
                bx0 = std::min(bx0, snap.pointsX[p]);
                bx1 = std::max(bx1, snap.pointsX[p]);
                by0 = std::min(by0, snap.pointsY[p]);
                by1 = std::max(by1, snap.pointsY[p]);
            }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            minCoiledRatio = std::min(minCoiledRatio, diag / kArcLen);
        }
    }
    if (minCoiledRatio < 0.30f) result.healthy = false;

    sim.snapshot(snap);
    const float hx = snap.pointsX[0], hy = snap.pointsY[0];
    result.finalDistToFood = std::sqrt((food.x - hx) * (food.x - hx) + (food.y - hy) * (food.y - hy));
    return result;
}

struct FitnessResult {
    float meanEffect = -1e6f;
    float stderrEffect = 0.0f;
    bool allHealthy = true;
};

FitnessResult evaluate(const Candidate& cand, int numSeeds, int seedBase, float gradientGainScale = 1.0f,
                        float noiseScale = 1.0f) {
    FitnessResult fr;
    std::vector<float> effects;
    effects.reserve(numSeeds);
    for (int s = 0; s < numSeeds; ++s) {
        const int seed = seedBase + s;
        const TrialResult without = runTrial(cand, seed, false, gradientGainScale, noiseScale);
        const TrialResult with = runTrial(cand, seed, true, gradientGainScale, noiseScale);
        if (!without.healthy || !with.healthy) {
            fr.allHealthy = false;
            continue;
        }
        effects.push_back(without.finalDistToFood - with.finalDistToFood);
    }
    if (!fr.allHealthy || effects.empty()) {
        fr.meanEffect = -1e6f;
        return fr;
    }
    double sum = 0.0;
    for (float e : effects) sum += e;
    const float mean = static_cast<float>(sum / effects.size());
    double sq = 0.0;
    for (float e : effects) sq += (e - mean) * (e - mean);
    const float stddev = effects.size() > 1 ? std::sqrt(static_cast<float>(sq / (effects.size() - 1))) : 0.0f;
    fr.meanEffect = mean;
    fr.stderrEffect = stddev / std::sqrt(static_cast<float>(effects.size()));
    return fr;
}

void printCandidate(const Candidate& c) {
    std::printf("[leakIP=%.3f leakP=%.3f leakPO=%.3f capIP=%.3f capP=%.3f capPO=%.3f capO=%.3f]", c[0], c[1], c[2],
                c[3], c[4], c[5], c[6]);
}

}  // namespace

// Round 1 (12 search seeds, plain "any improvement counts", ES hill-climb)
// found bestFitness=5.56 during search that shrank to 0.31 +/- 0.20 (effect/
// stderr ~1.5, not significant) under an independent 24-seed check - the gap
// is the "winner's curse" of picking the max of noisy estimates off too few
// seeds. Round 2 (20 search seeds, incumbent re-measured each generation,
// margin > 0.5x combined stderr to accept) made it WORSE, not better: the
// per-generation stderr routinely came out comparable to or bigger than the
// mean itself (e.g. incumbent -7.20 +/- 7.58), so a 0.5x-stderr bar barely
// filters anything, and the walk wandered to the edge of its own clamp
// bounds (capIP=20.0, capP=0.05, leakP up past 15) - a random walk chasing
// noise into an implausible, probably-chaotic corner of parameter space,
// not converging on a real biological calibration. Killed at gen 54.
//
// Round 3: abandon local hill-climbing outright (it amplifies whichever
// direction looked luckiest on one noisy batch - most vulnerable to exactly
// this failure). Screen-then-confirm instead: sample candidates WIDELY at
// random within a bound (0.3-3.0x) that's actually biologically plausible
// (real neurons vary time constants a few-fold, not 20-fold) and far
// narrower than round 1/2's 0.05-20 - keeps the search near the network's
// already-validated stable operating point rather than roaming into
// unknown, possibly-unstable territory. Rank the WIDE screen on a modest
// seed count, re-confirm only the top few survivors with many more seeds
// (a true effect should hold up on a fresh larger sample; a lucky-noise
// "winner" should regress toward zero) - two independent noise-filters
// instead of one weak per-step threshold.
//
// Round 3 RESULT: worked, and caught itself working correctly. Screen's own
// top candidate (screenFitness=0.48) collapsed to -0.20 +/- 0.11 on the
// 48-seed re-confirm - sign flip, textbook winner's curse, exactly what the
// two-stage design exists to catch. A different, more modest-looking screen
// survivor (screenFitness=0.03) held up: confirmedFitness=0.081 +/- 0.030,
// then 0.0275 +/- 0.0035 on a completely independent 96-seed final check
// (base 900000000, never touched by screen or confirm). Triple-checked
// separately (standalone mode, comparing the shipped calibrated network
// against the ORIGINAL uncalibrated one via inverse scale factors, 120
// seeds, yet another fresh base 555555555): 0.0275 +/- 0.0091 calibrated vs
// -0.0027 +/- 0.0008 original - same point estimate as the 96-seed run,
// ~3.3 combined-stderr apart even on this batch. Applied to production
// (WormSim.cpp ctor): leakScale{IP=1.397, P=1.419, PO=0.520},
// capScale{IP=1.680, P=0.560, PO=1.212, O=0.400}. tests/worm_locomotion
// still PASS (coiled ratio 0.645-0.648, slightly better than the
// pre-calibration 0.622-0.623, run 3x to check for variance).
//
// This IS real, validated, independently-reproduced directed chemotaxis -
// the open question tests/worm_locomotion's header leaves unresolved is
// answered: per-neuron-class calibration, done rigorously, works. The
// effect is small in absolute terms (food 180 units out) - plausibly
// appropriate, since real C. elegans klinokinesis is itself a subtle biased
// random walk, not a steer-to-target behavior, but not yet confirmed
// large enough to be visible by eye over a short play session. Round 4
// (if present below) is a LOCAL refinement around this validated point,
// using the same screen-then-confirm rigor, looking for a stronger nearby
// optimum rather than starting the search over.
//
// Round 4 RESULT: no improvement found, and a good illustration of why the
// two-stage design has to be trusted over any single measurement. Its own
// naive "pick the biggest confirmedFitness" selection chose the WORST-
// conditioned survivor (confirmedFitness=0.1852 +/- 0.2541 - by far the
// largest number, but also by far the noisiest) over more modest, far more
// reliable ones (0.0429 +/- 0.0379, 0.0420 +/- 0.0320) - on independent
// final verification that "winner" scored -0.0150 +/- 0.0768, i.e. slightly
// WORSE than the round-3 baseline it was supposed to refine, not better.
// Fixed by selecting on lower-confidence-bound (mean - stderr) instead of
// raw mean (see the "lcb" logic below) - a smaller, more certain number
// should beat a bigger, less certain one. Re-ran after the fix: still no
// candidate cleared the round-3 baseline. Conclusion: round 3's calibration
// looks like it's at or very near a local optimum for this 7-parameter
// space; local refinement within a biologically-plausible neighborhood
// isn't finding anything better with this sample budget.
//
// Separately (standalone mode, trialSteps override): checked whether the
// effect compounds over a much longer run than the 5000-step (250s) trials
// used throughout search. At 30000 steps (1500s), 64 seeds: 0.0578 +/- 0.0260
// vs the original ~0.0275 +/- 0.003-0.009 - the point estimate roughly
// doubled, but so did the relative noise (stderr scaled up right along with
// it, consistent with this network's known chaotic sensitivity to initial
// conditions over long integration - see tests/worm_locomotion). Not a free
// lunch: longer runs don't cleanly reveal a bigger, more certain signal,
// they reveal a bigger, equally uncertain one.
//
// Round 5 (env sweep, `argv[1]=="envsweep"`): does retuning gradientGain/
// spontaneousNoise ON TOP OF the now-baked per-class calibration unlock
// more? 5x5 grid, 32 seeds/cell, best cell (gradientScale=1.5,
// noiseScale=1.5) looked promising on the grid (lcb=0.0296) but on
// independent 96-seed final verification came in at 0.0167 +/- 0.0029 vs
// baseline's 0.0177 +/- 0.0032 - indistinguishable, if anything slightly
// worse. Another honest negative: the original investigation's own
// gradientGain sweep (pre-calibration) found nothing, and it still finds
// nothing even combined with a working per-class calibration - the two axes
// don't interact usefully.
//
// Where this left things (BEFORE the revert below): per-neuron-class
// calibration is REAL, validated - genuine progress on an open question.
// Its effect is a small, statistically solid directional bias (arguably the
// right ORDER of realism for real C. elegans klinokinesis, which is itself
// subtle), not a dramatic steer-to-target behavior, and pushing on this
// specific axis further (local refinement, longer runs, combining with env
// gains) hit diminishing returns.
//
// REVERTED (later session): the calibration shipped, then a live look at
// Demo_worm (default params, no food) showed the worm settling into a
// static shape and staying there - visibly "moves poorly". A dedicated
// "displacement" mode added to this harness (see below,
// `argv[1]=="displacement"`) measured plain crawling efficiency (net
// displacement / head path length, NO food, NO with/without pairing - this
// is a different axis than everything above) directly: the calibration cut
// it from 0.426 to 0.131 - MORE THAN 3x WORSE - and net displacement from
// 92.6 to 45.2 units over a fixed 800-step window. Every round above (1-5)
// screened for a chemotaxis effect plus basic health (NaN/bounds/coiled-
// ratio, which is a SHAPE health check, not a locomotion-efficiency one) -
// none of them ever measured absolute crawling efficiency, so the search
// was blind to trading away most of the worm's mobility for a directional
// bias worth 0.03-0.06 units against a 180-unit food distance. Reverted in
// WormSim.cpp's constructor (back to the loader's flat leak=1/capacitance=1
// defaults - see that file's own comment for the exact numbers). If this
// axis is revisited, ANY fitness function used here must include a
// crawling-efficiency term, or it will find the same trade again.
//
// The one remaining honest option from tests/worm_locomotion's list not yet
// tried is architecturally different: phase-gate the sensory drive by the
// network's own existing locomotor wave/CPG signal (there IS a real
// travelling wave now, thanks to proprioception) instead of injecting scent
// as a constant additive term - bigger, riskier undertaking, not started
// here.
int main(int argc, char** argv) {
    if (argc >= 2 && std::string(argv[1]) == "displacement") {
        // Live observation (Demo_worm, default params, no food) showed the
        // worm settling into a STATIC shape after ~20s and staying there for
        // another 20s+ - net translation approximately zero despite
        // continuous shape change earlier in the run. This mode measures
        // that directly, headless: net displacement AND total path length of
        // the body CENTROID (average of all 25 body points, NOT points_[0] -
        // see below) over a real run (no with/without-food pairing - this is
        // about absolute crawling quality, not chemotaxis), for a given
        // candidate, no food at all (matches what was observed live).
        //
        // CENTROID, not points_[0]: an earlier version of this mode tracked
        // points_[0] (one end of the segment chain) and used it to justify
        // reverting a chemotaxis calibration for "3x worse crawling". A later
        // investigation (tests/worm_locomotion / WormSim.h's dragNormal
        // comment has the full story) found that's the wrong point to track:
        // under resistive force theory, near-isotropic drag (c_n/c_t -> 1)
        // forces the CENTROID's net velocity toward zero (equal scalar drag
        // on every segment -> zero net force -> no net centroid motion), but
        // a chain endpoint can still swing substantially around a stationary
        // centroid from pure shape change - recoil, not travel. Verified
        // directly: at dragNormal/dragTangent=1 (isotropic), the OLD
        // points_[0] metric read as the best of any tested ratio, while true
        // centroid net displacement was almost nothing. If comparing across
        // different drag settings (unlike the original calibration check,
        // which held drag fixed across both conditions and so was less
        // affected), always use this centroid-based version, not points_[0].
        //   ./exe displacement leakIP leakP leakPO capIP capP capPO capO [numSeeds] [seedBase] [steps] [dragTangent] [dragNormal]
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numSeeds = argc > kNumParams + 2 ? std::atoi(argv[kNumParams + 2]) : 16;
        const int seedBase = argc > kNumParams + 3 ? std::atoi(argv[kNumParams + 3]) : 321321321;
        if (argc > kNumParams + 4) g_trialSteps = std::atoi(argv[kNumParams + 4]);
        const float dragTangent = argc > kNumParams + 5 ? static_cast<float>(std::atof(argv[kNumParams + 5])) : -1.0f;
        const float dragNormal = argc > kNumParams + 6 ? static_cast<float>(std::atof(argv[kNumParams + 6])) : -1.0f;
        std::printf("=== Displacement check: %d seeds, base=%d, steps=%d, drag=(%s,%s) (no food, centroid metric) ===\n",
                    numSeeds, seedBase, g_trialSteps, dragTangent >= 0.0f ? std::to_string(dragTangent).c_str() : "default",
                    dragNormal >= 0.0f ? std::to_string(dragNormal).c_str() : "default");
        auto centroid = [](const WormSim::Snapshot& s, float& cx, float& cy) {
            cx = 0.0f; cy = 0.0f;
            for (std::size_t i = 0; i < s.pointsX.size(); ++i) { cx += s.pointsX[i]; cy += s.pointsY[i]; }
            cx /= static_cast<float>(s.pointsX.size());
            cy /= static_cast<float>(s.pointsY.size());
        };
        double sumNet = 0.0, sumPath = 0.0, sumEff = 0.0;
        int frozenCount = 0;
        for (int s = 0; s < numSeeds; ++s) {
            const int seed = seedBase + s;
            WormSim sim("worm_data/celegans_herm.connectome");
            applyCalibration(sim, cand);
            if (dragTangent >= 0.0f) sim.params.dragTangent = dragTangent;
            if (dragNormal >= 0.0f) sim.params.dragNormal = dragNormal;
            std::srand(static_cast<unsigned>(seed));
            sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
            WormSim::Snapshot snap;
            sim.snapshot(snap);
            float startX, startY;
            centroid(snap, startX, startY);
            float prevX = startX, prevY = startY;
            double pathLen = 0.0;
            // Track whether the shape (not just the centroid) keeps changing -
            // frozen = true if the LAST 200 steps' worth of bbox-diagonal
            // barely differs from 200 steps before that (a static final shape).
            float bboxDiagAt = -1.0f, bboxDiagBefore = -1.0f;
            for (int i = 0; i < g_trialSteps; ++i) {
                sim.step();
                sim.snapshot(snap);
                float x, y;
                centroid(snap, x, y);
                pathLen += std::sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
                prevX = x;
                prevY = y;
                if (i == g_trialSteps - 401) {
                    float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
                    for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
                    for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
                    bboxDiagBefore = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
                }
                if (i == g_trialSteps - 1) {
                    float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
                    for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
                    for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
                    bboxDiagAt = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
                }
            }
            const float netDisp = std::sqrt((prevX - startX) * (prevX - startX) + (prevY - startY) * (prevY - startY));
            const float eff = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
            const bool frozen = std::fabs(bboxDiagAt - bboxDiagBefore) < 2.0f;  // shape barely changed in last 400 steps
            if (frozen) ++frozenCount;
            std::printf("seed %d: netDisp=%.1f pathLen=%.1f efficiency=%.4f shapeFrozenAtEnd=%d\n", seed, netDisp,
                        pathLen, eff, frozen ? 1 : 0);
            sumNet += netDisp;
            sumPath += pathLen;
            sumEff += eff;
        }
        std::printf("MEAN: netDisp=%.1f pathLen=%.1f efficiency=%.4f frozenAtEnd=%d/%d\n", sumNet / numSeeds,
                    sumPath / numSeeds, sumEff / numSeeds, frozenCount, numSeeds);
        return 0;
    }

    if (argc == 2 && std::string(argv[1]) == "envsweep") {
        // Round 5: does retuning gradientGain/spontaneousNoise ON TOP OF the
        // now-baked per-class calibration unlock a bigger effect than either
        // axis alone? (The original investigation swept gradientGain up to
        // 125x default BEFORE any per-class calibration existed and found
        // nothing - worth one honest re-check now that the network itself
        // behaves differently.) Only 2 free dimensions - plain grid search,
        // each cell evaluated directly with enough seeds to trust the
        // number, no separate screen/confirm stage needed at this size.
        constexpr float kGradientScales[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f};
        constexpr float kNoiseScales[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f};
        constexpr int kGridSeeds = 32;
        float bestG = 1.0f, bestN = 1.0f, bestLcb = -1e9f;
        int cell = 0;
        for (float g : kGradientScales) {
            for (float n : kNoiseScales) {
                const FitnessResult fr = evaluate(kIdentity, kGridSeeds, cell * 10000, g, n);
                const float lcb = fr.meanEffect - fr.stderrEffect;
                std::printf("gradientScale=%.1f noiseScale=%.1f: meanEffect=%.4f+/-%.4f lcb=%.4f allHealthy=%d\n", g,
                            n, fr.meanEffect, fr.stderrEffect, lcb, fr.allHealthy ? 1 : 0);
                std::fflush(stdout);
                if (fr.allHealthy && lcb > bestLcb) {
                    bestLcb = lcb;
                    bestG = g;
                    bestN = n;
                }
                ++cell;
            }
        }
        std::printf("\nbest grid cell: gradientScale=%.1f noiseScale=%.1f lcb=%.4f\n", bestG, bestN, bestLcb);
        std::printf("=== Final verification (96 seeds, fresh base) ===\n");
        FitnessResult baseline = evaluate(kIdentity, 96, 444555666, 1.0f, 1.0f);
        FitnessResult best = evaluate(kIdentity, 96, 444555666, bestG, bestN);
        const float denom =
            std::sqrt(baseline.stderrEffect * baseline.stderrEffect + best.stderrEffect * best.stderrEffect);
        const float sigmaAway = denom > 1e-6f ? (best.meanEffect - baseline.meanEffect) / denom : 0.0f;
        std::printf("baseline (gradientScale=1,noiseScale=1): meanEffect=%.4f +/- %.4f allHealthy=%d\n",
                    baseline.meanEffect, baseline.stderrEffect, baseline.allHealthy ? 1 : 0);
        std::printf("best     (gradientScale=%.1f,noiseScale=%.1f): meanEffect=%.4f +/- %.4f allHealthy=%d  (%.2f "
                    "combined-stderr above baseline)\n",
                    bestG, bestN, best.meanEffect, best.stderrEffect, best.allHealthy ? 1 : 0, sigmaAway);
        return 0;
    }

    if (argc >= kNumParams + 1) {
        // Standalone re-verification mode:
        //   ./exe leakIP leakP leakPO capIP capP capPO capO [numSeeds] [seedBase] [trialSteps]
        // - re-checks one specific candidate against a fresh, independently-chosen
        // seed batch without re-running the whole search. trialSteps lets this
        // check whether the effect compounds over a much longer run than the
        // search itself used (5000 steps = 250 simulated seconds).
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 1]));
        const int numSeeds = argc > kNumParams + 1 ? std::atoi(argv[kNumParams + 1]) : 60;
        const int seedBase = argc > kNumParams + 2 ? std::atoi(argv[kNumParams + 2]) : 9000000;
        if (argc > kNumParams + 3) g_trialSteps = std::atoi(argv[kNumParams + 3]);
        std::printf("=== Standalone verification, %d seeds, base=%d, trialSteps=%d ===\n", numSeeds, seedBase,
                    g_trialSteps);
        FitnessResult idFr = evaluate(kIdentity, numSeeds, seedBase);
        FitnessResult candFr = evaluate(cand, numSeeds, seedBase);
        std::printf("identity  : meanEffect=%.4f +/- %.4f  allHealthy=%d\n", idFr.meanEffect, idFr.stderrEffect,
                    idFr.allHealthy ? 1 : 0);
        std::printf("candidate : meanEffect=%.4f +/- %.4f  allHealthy=%d  ", candFr.meanEffect, candFr.stderrEffect,
                    candFr.allHealthy ? 1 : 0);
        printCandidate(cand);
        std::printf("\n");
        return 0;
    }

    // Round 4: WormSim's ctor now bakes in round 3's winning calibration, so
    // kIdentity (scale factors all 1.0, a no-op through applyCalibration) IS
    // that already-validated baseline, not the pre-calibration original -
    // this baseline is expected to show the confirmed ~0.0275 effect, not
    // ~0. Candidates here compound MULTIPLICATIVELY on top of it (round 4
    // is a LOCAL refinement, narrower bounds, not a fresh search).
    std::printf("=== Baseline sanity check (identity = round 3's shipped calibration) ===\n");
    FitnessResult baseline = evaluate(kIdentity, 8, 0);
    std::printf("identity: meanEffect=%.4f +/- %.4f  allHealthy=%d\n\n", baseline.meanEffect, baseline.stderrEffect,
                baseline.allHealthy ? 1 : 0);

    std::printf("=== Phase 1: local refinement screen (0.6x-1.6x around the shipped calibration) ===\n");
    constexpr float kBoundLo = 0.6f, kBoundHi = 1.6f;
    constexpr int kScreenCandidates = 300;
    constexpr int kScreenSeeds = 16;
    constexpr int kTopK = 5;

    std::mt19937 rng(2024);
    std::uniform_real_distribution<float> logUniform(std::log(kBoundLo), std::log(kBoundHi));

    struct Scored {
        Candidate cand;
        float fitness;
    };
    std::vector<Scored> top;  // kept sorted descending by fitness, capped at kTopK

    for (int i = 0; i < kScreenCandidates; ++i) {
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = std::exp(logUniform(rng));
        const FitnessResult fr = evaluate(cand, kScreenSeeds, i * 1000);
        if (top.size() < static_cast<std::size_t>(kTopK) || fr.meanEffect > top.back().fitness) {
            top.push_back({cand, fr.meanEffect});
            std::sort(top.begin(), top.end(), [](const Scored& a, const Scored& b) { return a.fitness > b.fitness; });
            if (top.size() > static_cast<std::size_t>(kTopK)) top.pop_back();
        }
        if (i % 20 == 0 || i == kScreenCandidates - 1) {
            std::printf("screen %3d/%d: top fitness so far = %.4f\n", i, kScreenCandidates,
                        top.empty() ? -1e9f : top.front().fitness);
            std::fflush(stdout);
        }
    }

    std::printf("\n=== Phase 2: re-confirm top %d candidates with more seeds ===\n", kTopK);
    constexpr int kConfirmSeeds = 48;
    Candidate winner = kIdentity;
    // Select by LOWER-confidence-bound (mean - stderr), not raw mean - round
    // 4's own first pass picked candidate 4 here (meanEffect=0.1852, by far
    // the largest number) purely because it had the largest number, ignoring
    // that its stderr (0.2541) was also by far the largest - i.e. it was the
    // LEAST reliable of the five, and duly collapsed to -0.0150 +/- 0.0768
    // (worse than the round-3 baseline) under final independent
    // verification. A noisy overestimate should not outrank a smaller but
    // more trustworthy one.
    float winnerLcb = -1e9f;
    for (std::size_t i = 0; i < top.size(); ++i) {
        const FitnessResult fr = evaluate(top[i].cand, kConfirmSeeds, 777000000 + static_cast<int>(i) * 1000);
        const float lcb = fr.meanEffect - fr.stderrEffect;
        std::printf("candidate %zu: screenFitness=%.4f confirmedFitness=%.4f+/-%.4f lcb=%.4f allHealthy=%d ", i,
                    top[i].fitness, fr.meanEffect, fr.stderrEffect, lcb, fr.allHealthy ? 1 : 0);
        printCandidate(top[i].cand);
        std::printf("\n");
        std::fflush(stdout);
        if (fr.allHealthy && lcb > winnerLcb) {
            winnerLcb = lcb;
            winner = top[i].cand;
        }
    }

    std::printf("\n=== Final verification (96 seeds, independent of all prior seed bases) ===\n");
    FitnessResult finalBaseline = evaluate(kIdentity, 96, 900000000);
    FitnessResult finalBest = evaluate(winner, 96, 900000000);
    const float sigDenom = std::sqrt(finalBaseline.stderrEffect * finalBaseline.stderrEffect +
                                      finalBest.stderrEffect * finalBest.stderrEffect);
    const float sigmaAway = sigDenom > 1e-6f ? (finalBest.meanEffect - finalBaseline.meanEffect) / sigDenom : 0.0f;
    std::printf("identity  : meanEffect=%.4f +/- %.4f  allHealthy=%d\n", finalBaseline.meanEffect,
                finalBaseline.stderrEffect, finalBaseline.allHealthy ? 1 : 0);
    std::printf("calibrated: meanEffect=%.4f +/- %.4f  allHealthy=%d  (%.2f combined-stderr above identity)  ",
                finalBest.meanEffect, finalBest.stderrEffect, finalBest.allHealthy ? 1 : 0, sigmaAway);
    printCandidate(winner);
    std::printf("\n");
    return 0;
}
