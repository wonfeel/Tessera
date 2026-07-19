// tests/worm_synapse_speed_calibration/main.cpp
//
// Third attempt at the gait-SPEED axis (see tests/worm_speed_calibration for
// the full history of the first two - both per-neuron-class leak/capacitance
// calibration, both reverted). Round 1 shipped a false 4.85x speedup that
// turned out to be a single-seed-base measurement artifact (population mean
// across 20 fresh bases: IDENTICAL between candidate and identity, 0.01079
// BL/s both). That failure was a METHODOLOGY bug (trusting one base's mean,
// no matter how many seeds), not proof leak/capacitance specifically can't
// help - but a background research pass this session (see WormSim.cpp
// constructor comment and this project's chat history) independently
// concluded leak/capacitance is "the same axis wearing a different hat" as
// this file's own weight-scaling approach - both are global-ish rescalings
// of the same recurrent dynamics, so a genuine escape needs either (a) a
// different LEVER entirely, or (b) proof this specific lever has real
// headroom the leak/capacitance one didn't.
//
// THIS FILE'S LEVER: scale_synapse_sign (network.hpp) - NOT per-(pre,post)-
// NeuronType weight scaling (which the research pass flagged as "same axis,
// different array", comparable risk to leak/capacitance), but per-SIGN
// scaling of the raw connectome weights: excitatory (cholinergic, positive
// weight) chemical synapses, inhibitory (GABAergic, negative weight)
// chemical synapses, and gap junctions, each get their own multiplier (3
// free parameters, not per-class leak/capacitance's 7, and nowhere near a
// full per-edge search's 4682+1213). This is a genuinely different
// biological unknown: data/README.md confirms connectome weights are raw EM
// synaptic CONTACT COUNTS (Cook et al. 2019), not conductances - and the
// ratio between excitatory and inhibitory POSTSYNAPTIC CURRENT per contact
// is not something contact-count EM reconstruction can measure at all. This
// is a real missing physiological parameter, not a re-run of the same knob.
//
// METHODOLOGY FIX baked in from round 1 (not bolted on after shipping, the
// mistake that got round 1 leak/capacitance shipped wrong): every fitness
// evaluation during the search itself already averages across MULTIPLE
// INDEPENDENT SEED BASES (kScreenBases below), not more seeds from one base.
// A candidate that only looks good on one lucky base cannot win the screen
// in the first place - the chaos this network showed (tests/worm_speed_
// calibration's "distribution" mode) is averaged out from step 1, not
// discovered as a surprise after the fact.
//
// Health gate: same efficiency>=0.40 / coiledRatio>=0.30 guard as tests/
// worm_speed_calibration, for the same reason (raw speed is meaningless if
// achieved by degenerating into a non-crawl).
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

constexpr int kNumParams = 3;
enum ParamIdx { kChemExc = 0, kChemInh = 1, kGap = 2 };
using Candidate = std::array<float, kNumParams>;
const Candidate kIdentity = {1.0f, 1.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    connectome::Network& net = sim.network();
    net.scale_synapse_sign(c[kChemExc], c[kChemInh], c[kGap]);
}

void printCandidate(const Candidate& c) {
    std::printf("[chemExc=%.3f chemInh=%.3f gap=%.3f]", c[0], c[1], c[2]);
}

constexpr float kDragTangent = 1.0f;   // reference tangential drag - fixed across presets project-wide
constexpr float kDragNormalAgar = 40.0f, kDragNormalWater = 1.7f;  // this project's two presets
constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kHexSpacing = 36.0f;
constexpr float kBodyLength = 576.0f;  // kNumSegments(24) * segment_length(24.0), see WormSim.cpp ctor

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;
    float minCoiledRatio = 1e9f;
    float freqHz = 0.0f;
    bool healthy = true;
};

Measurement runTrial(const Candidate& cand, int seed, float dragNormal, int warmupSteps, int measureSteps,
                      int freqPosition = 12) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    sim.params.dragTangent = kDragTangent;
    sim.params.dragNormal = dragNormal;
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
    m.freqHz = static_cast<float>(zeroCrossings) / 2.0f / measureSeconds;
    return m;
}

struct AggregateResult {
    float meanBLps = 0.0f, stderrBLps = 0.0f, meanFreqHz = 0.0f, meanEfficiency = 0.0f, minCoiledRatio = 1e9f;
    bool allHealthy = true;
};

// Averages across MULTIPLE INDEPENDENT SEED BASES, not more seeds from one -
// this IS the fitness function used during the search itself, not a
// validation step bolted on after the fact (see header).
AggregateResult evaluate(const Candidate& cand, const std::vector<int>& seedBases, int seedsPerBase,
                          float dragNormal, int warmupSteps, int measureSteps) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int base : seedBases) {
        for (int s = 0; s < seedsPerBase; ++s) {
            const Measurement m = runTrial(cand, base + s, dragNormal, warmupSteps, measureSteps);
            ar.minCoiledRatio = std::min(ar.minCoiledRatio, m.minCoiledRatio);
            if (!m.healthy) { ar.allHealthy = false; continue; }
            blSamples.push_back(m.bodyLengthsPerSec);
            sumFreq += m.freqHz;
            sumEff += m.efficiency;
        }
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

float fitnessOf(const AggregateResult& ar) {
    if (!ar.allHealthy || ar.minCoiledRatio < kMinCoiledRatio || ar.meanEfficiency < kMinEfficiency) return -1e6f;
    return ar.meanBLps;
}

std::vector<int> makeBases(std::mt19937& rng, int count) {
    std::uniform_int_distribution<int> baseDist(1, 2000000000);
    std::vector<int> bases;
    for (int i = 0; i < count; ++i) bases.push_back(baseDist(rng));
    return bases;
}

// CHEMOTAXIS CHECK - added after adversarial review of the speed winner
// flagged an unaddressed risk: scale_synapse_sign rescales ALL chemical
// synapses network-wide, including chemosensory circuits, but the speed
// search above never measured directed food-seeking, only crawl efficiency/
// coiling - the exact blind spot that let the OTHER reverted calibration
// (leak/capacitance for chemotaxis, see tests/worm_chemotaxis_calibration)
// secretly cut crawling efficiency 3x while shipping. Same paired
// methodology as that file's own validated Round 3 (PMC-grade result there:
// 0.0275 +/- 0.0035 effect, independently reproduced across three separate
// seed batches): field size 28x20, kFoodRadius=180, food-at-per-seed-angle
// debiasing, distance-to-food-with-food-minus-without-food metric.
//
// FIRST ATTEMPT at this check reused those numbers unchanged and got back
// effect=9.42+/-13.25 - stderr BIGGER than the mean, i.e. no information at
// all. Root cause: those numbers were tuned for IDENTITY's speed. A 3-5x
// faster candidate covers proportionally more ground in the same 5000-step
// window, so in the same small 28x20 arena it now spends much of the trial
// bouncing off walls (see WormSim::containBody) instead of genuinely
// wandering/seeking - wall-bounce noise, not a chemotaxis measurement.
// FIX: auto-scale the arena and food radius to the CANDIDATE'S OWN measured
// speed relative to identity's, at the start of every "chemotaxis" run (see
// main()) - not a guessed constant, a real ratio measured from the same
// evaluate() function the speed search itself uses. A 1x-speed candidate
// (identity) gets back the exact original 28x20/180 numbers; a 3x-speed
// candidate gets a 3x bigger arena and food radius, keeping the test's
// original "reachable but not trivial" character intact for whatever
// candidate is actually being checked.
float g_chemHexSpacing = 36.0f;
int g_chemFieldCols = 28, g_chemFieldRows = 20;
float g_foodRadius = 180.0f;
constexpr float kGoldenAngleDeg = 137.50776f;

glm::vec2 foodPositionForSeed(glm::vec2 start, int seed) {
    const float angleDeg = 45.0f + static_cast<float>(seed) * kGoldenAngleDeg;
    const float angle = angleDeg * 3.14159265f / 180.0f;
    return start + g_foodRadius * glm::vec2(std::cos(angle), std::sin(angle));
}

struct ChemTrialResult {
    float finalDistToFood = 0.0f;
    bool healthy = true;
};

ChemTrialResult runChemTrial(const Candidate& cand, int seed, bool withFood, float dragNormal, int steps) {
    ChemTrialResult result;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    sim.params.dragTangent = kDragTangent;
    sim.params.dragNormal = dragNormal;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), g_chemFieldCols, g_chemFieldRows, g_chemHexSpacing);
    const glm::vec2 boundsMax = HexGrid::worldPos(g_chemFieldCols - 1, g_chemFieldRows - 1, g_chemHexSpacing);
    const glm::vec2 start = boundsMax * 0.5f;
    const glm::vec2 food = foodPositionForSeed(start, seed);
    if (withFood) sim.depositFood(food, 5.0f);

    WormSim::Snapshot snap;
    float minCoiledRatio = 1e9f;
    for (int i = 0; i < steps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float x = snap.pointsX[0], y = snap.pointsY[0];
        if (std::isnan(x) || std::isnan(y)) { result.healthy = false; return result; }
        bool outOfBounds = false;
        for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
            constexpr float kEps = 1.0f;
            if (snap.pointsX[p] < -kEps || snap.pointsX[p] > boundsMax.x + kEps || snap.pointsY[p] < -kEps ||
                snap.pointsY[p] > boundsMax.y + kEps) {
                outOfBounds = true;
                break;
            }
        }
        if (outOfBounds) { result.healthy = false; return result; }
        if (i > 200 && i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
                bx0 = std::min(bx0, snap.pointsX[p]);
                bx1 = std::max(bx1, snap.pointsX[p]);
                by0 = std::min(by0, snap.pointsY[p]);
                by1 = std::max(by1, snap.pointsY[p]);
            }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            minCoiledRatio = std::min(minCoiledRatio, diag / kBodyLength);
        }
    }
    if (minCoiledRatio < 0.30f) result.healthy = false;
    sim.snapshot(snap);
    const float hx = snap.pointsX[0], hy = snap.pointsY[0];
    result.finalDistToFood = std::sqrt((food.x - hx) * (food.x - hx) + (food.y - hy) * (food.y - hy));
    return result;
}

struct ChemFitness {
    float meanEffect = 0.0f, stderrEffect = 0.0f;
    bool allHealthy = true;
    int healthyCount = 0, total = 0;
};

ChemFitness evaluateChem(const Candidate& cand, int numSeeds, int seedBase, float dragNormal, int steps) {
    ChemFitness fr;
    std::vector<float> effects;
    for (int s = 0; s < numSeeds; ++s) {
        const int seed = seedBase + s;
        const ChemTrialResult without = runChemTrial(cand, seed, false, dragNormal, steps);
        const ChemTrialResult with = runChemTrial(cand, seed, true, dragNormal, steps);
        fr.total++;
        if (!without.healthy || !with.healthy) { fr.allHealthy = false; continue; }
        fr.healthyCount++;
        effects.push_back(without.finalDistToFood - with.finalDistToFood);
    }
    if (!effects.empty()) {
        double sum = 0.0;
        for (float e : effects) sum += e;
        const float mean = static_cast<float>(sum / effects.size());
        double sq = 0.0;
        for (float e : effects) sq += (e - mean) * (e - mean);
        const float stddev = effects.size() > 1 ? std::sqrt(static_cast<float>(sq / (effects.size() - 1))) : 0.0f;
        fr.meanEffect = mean;
        fr.stderrEffect = stddev / std::sqrt(static_cast<float>(effects.size()));
    }
    return fr;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe chemotaxis chemExc chemInh gap [numSeeds] [seedBase] [steps] [dragNormal]
    // Paired with/without-food check (see comment above evaluateChem) - run
    // BOTH identity and a candidate through this before shipping the
    // candidate, to make sure a real speed gain isn't secretly trading away
    // directed food-seeking (the exact blind spot that shipped the OTHER
    // reverted calibration wrong).
    if (argc >= 2 && std::string(argv[1]) == "chemotaxis") {
        Candidate cand = kIdentity;
        for (int k = 0; k < kNumParams && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numSeeds = argc > 5 ? std::atoi(argv[5]) : 24;
        const int seedBase = argc > 6 ? std::atoi(argv[6]) : 900000000;
        const int steps = argc > 7 ? std::atoi(argv[7]) : 5000;
        const float dragNormal = argc > 8 ? static_cast<float>(std::atof(argv[8])) : kDragNormalAgar;

        // Auto-scale arena/food-radius to this candidate's OWN measured
        // speed relative to identity (see comment above g_chemFieldCols) -
        // a real ratio from evaluate(), not a guessed constant. Small,
        // cheap measurement (2 bases x 3 seeds, short window) - only needs
        // to be in the right ballpark to size the arena correctly.
        std::mt19937 scaleRng(13);
        const std::vector<int> scaleBases = makeBases(scaleRng, 2);
        const AggregateResult idSpeed = evaluate(kIdentity, scaleBases, 3, dragNormal, 300, 800);
        const AggregateResult candSpeed = evaluate(cand, scaleBases, 3, dragNormal, 300, 800);
        const float speedRatio = (idSpeed.meanBLps > 1e-6f && candSpeed.allHealthy)
                                      ? std::max(1.0f, candSpeed.meanBLps / idSpeed.meanBLps)
                                      : 1.0f;
        g_chemFieldCols = std::max(28, static_cast<int>(std::lround(28 * speedRatio)));
        g_chemFieldRows = std::max(20, static_cast<int>(std::lround(20 * speedRatio)));
        g_foodRadius = 180.0f * speedRatio;
        std::printf("speed ratio (candidate/identity) = %.2fx -> arena %dx%d, foodRadius=%.0f\n", speedRatio,
                    g_chemFieldCols, g_chemFieldRows, g_foodRadius);

        const ChemFitness fr = evaluateChem(cand, numSeeds, seedBase, dragNormal, steps);
        std::printf("cand="); printCandidate(cand);
        std::printf(" seeds=%d seedBase=%d steps=%d dragNormal=%.2f\n", numSeeds, seedBase, steps, dragNormal);
        std::printf("  chemotaxis effect=%.4f+/-%.4f  healthy=%d/%d\n", fr.meanEffect, fr.stderrEffect,
                    fr.healthyCount, fr.total);
        return 0;
    }

    // ./exe measure chemExc chemInh gap [numBases] [seedsPerBase] [warmup] [measure] [dragNormal]
    if (argc >= 2 && std::string(argv[1]) == "measure") {
        Candidate cand = kIdentity;
        for (int k = 0; k < kNumParams && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 8;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 4;
        const int warmupSteps = argc > 7 ? std::atoi(argv[7]) : 300;
        const int measureSteps = argc > 8 ? std::atoi(argv[8]) : 2000;
        const float dragNormal = argc > 9 ? static_cast<float>(std::atof(argv[9])) : kDragNormalAgar;
        std::mt19937 rng(20250101);
        const std::vector<int> bases = makeBases(rng, numBases);
        const AggregateResult ar = evaluate(cand, bases, seedsPerBase, dragNormal, warmupSteps, measureSteps);
        std::printf("cand="); printCandidate(cand);
        std::printf(" bases=%d seedsPerBase=%d dragNormal=%.2f\n", numBases, seedsPerBase, dragNormal);
        std::printf("  speed=%.5f+/-%.5f BL/s  freq=%.4fHz  efficiency=%.3f  minCoiledRatio=%.3f  allHealthy=%d\n",
                    ar.meanBLps, ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                    ar.allHealthy ? 1 : 0);
        return 0;
    }

    // ./exe distribution chemExc chemInh gap [numBases] [seedsPerBase] [dragNormal]
    // Final, large-scale confirmation - same idea as tests/worm_speed_
    // calibration's own "distribution" mode, prints per-base numbers so a
    // fake win (identical mean masking huge base-to-base variance) is
    // visible, not just a single averaged number.
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        for (int k = 0; k < kNumParams && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 24;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 8;
        const float dragNormal = argc > 7 ? static_cast<float>(std::atof(argv[7])) : kDragNormalAgar;
        std::printf("cand="); printCandidate(cand);
        std::printf(" - %d bases x %d seeds, dragNormal=%.2f\n", numBases, seedsPerBase, dragNormal);
        std::mt19937 rng(31337);
        const std::vector<int> bases = makeBases(rng, numBases);
        double sumBL = 0.0;
        int healthyBases = 0;
        for (int base : bases) {
            const AggregateResult ar = evaluate(cand, {base}, seedsPerBase, dragNormal, 300, 2000);
            const bool ok = ar.allHealthy && ar.minCoiledRatio >= kMinCoiledRatio && ar.meanEfficiency >= kMinEfficiency;
            std::printf("  base=%d: speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f %s\n", base, ar.meanBLps,
                        ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio, ok ? "ok" : "FAIL");
            std::fflush(stdout);
            if (ok) { sumBL += ar.meanBLps; ++healthyBases; }
        }
        std::printf("\npopulation mean speed: %.5f BL/s over %d/%d healthy bases\n",
                    healthyBases ? sumBL / healthyBases : 0.0, healthyBases, numBases);
        return 0;
    }

    // Default: screen (chemExc, chemInh, gap) log-uniform, multi-base fitness
    // from the start (see header - this is the methodology fix). Confirm top
    // candidates with more bases, then run identity through the SAME final
    // check for a fair comparison.
    std::printf("=== Baseline (identity) ===\n");
    constexpr int kWarmup = 300, kMeasure = 2000;
    constexpr int kScreenBaseCount = 3, kScreenSeedsPerBase = 4;
    std::mt19937 baseRng(4242);
    const AggregateResult idBase = evaluate(kIdentity, makeBases(baseRng, kScreenBaseCount), kScreenSeedsPerBase,
                                             kDragNormalAgar, kWarmup, kMeasure);
    std::printf("identity: speed=%.5f+/-%.5f BL/s freq=%.4fHz efficiency=%.3f coiled=%.3f healthy=%d\n\n",
                idBase.meanBLps, idBase.stderrBLps, idBase.meanFreqHz, idBase.meanEfficiency, idBase.minCoiledRatio,
                idBase.allHealthy ? 1 : 0);

    std::printf("=== Screen (0.1-3.0x log-uniform, per parameter, %d bases x %d seeds/candidate) ===\n",
                kScreenBaseCount, kScreenSeedsPerBase);
    constexpr float kBoundLo = 0.1f, kBoundHi = 3.0f;
    constexpr int kScreenCandidates = 60;
    constexpr int kTopK = 5;

    std::mt19937 candRng(4242);
    std::uniform_real_distribution<float> logUniform(std::log(kBoundLo), std::log(kBoundHi));

    struct Scored { Candidate cand; float fitness; AggregateResult ar; };
    std::vector<Scored> top;

    for (int i = 0; i < kScreenCandidates; ++i) {
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = std::exp(logUniform(candRng));
        std::mt19937 thisBaseRng(1000 + i);
        const AggregateResult ar = evaluate(cand, makeBases(thisBaseRng, kScreenBaseCount), kScreenSeedsPerBase,
                                             kDragNormalAgar, kWarmup, kMeasure);
        const float fit = fitnessOf(ar);
        if (top.size() < static_cast<std::size_t>(kTopK) || fit > top.back().fitness) {
            top.push_back({cand, fit, ar});
            std::sort(top.begin(), top.end(), [](const Scored& a, const Scored& b) { return a.fitness > b.fitness; });
            if (top.size() > static_cast<std::size_t>(kTopK)) top.pop_back();
        }
        if (i % 5 == 0 || i == kScreenCandidates - 1) {
            std::printf("screen %3d/%d: top speed so far = %.5f BL/s\n", i, kScreenCandidates,
                        top.empty() ? -1.0f : top.front().fitness);
            std::fflush(stdout);
        }
    }

    std::printf("\n=== Confirm top %d (8 bases x 6 seeds) ===\n", kTopK);
    constexpr int kConfirmBaseCount = 8, kConfirmSeedsPerBase = 6;
    Candidate winner = kIdentity;
    float winnerLcb = -1e9f;
    for (std::size_t i = 0; i < top.size(); ++i) {
        std::mt19937 confirmBaseRng(777000 + static_cast<int>(i));
        const AggregateResult ar = evaluate(top[i].cand, makeBases(confirmBaseRng, kConfirmBaseCount),
                                             kConfirmSeedsPerBase, kDragNormalAgar, kWarmup, kMeasure);
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

    std::printf("\n=== Final verification (20 bases x 8 seeds, both identity and winner) ===\n");
    std::mt19937 finalBaseRng(900000000);
    const std::vector<int> finalBases = makeBases(finalBaseRng, 20);
    const AggregateResult finalId = evaluate(kIdentity, finalBases, 8, kDragNormalAgar, kWarmup, kMeasure);
    const AggregateResult finalWin = evaluate(winner, finalBases, 8, kDragNormalAgar, kWarmup, kMeasure);
    std::printf("identity : speed=%.5f+/-%.5f BL/s freq=%.4fHz efficiency=%.3f coiled=%.3f healthy=%d\n",
                finalId.meanBLps, finalId.stderrBLps, finalId.meanFreqHz, finalId.meanEfficiency,
                finalId.minCoiledRatio, finalId.allHealthy ? 1 : 0);
    std::printf("winner   : speed=%.5f+/-%.5f BL/s freq=%.4fHz efficiency=%.3f coiled=%.3f healthy=%d  ",
                finalWin.meanBLps, finalWin.stderrBLps, finalWin.meanFreqHz, finalWin.meanEfficiency,
                finalWin.minCoiledRatio, finalWin.allHealthy ? 1 : 0);
    printCandidate(winner);
    std::printf("\n");
    if (finalWin.meanBLps > 1e-6f) {
        std::printf("speedup vs identity: %.2fx\n", finalWin.meanBLps / std::max(1e-6f, finalId.meanBLps));
    }
    return 0;
}
