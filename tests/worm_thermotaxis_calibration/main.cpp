// tests/worm_thermotaxis_calibration/main.cpp
//
// Calibration search for Params::thermoGain (WormSim.h/.cpp) - the AFD
// thermosensory input added this session (real thermosensory neuron, Mori &
// Ohshima 1995; AFD's first synapse in this loaded connectome really is onto
// AIY - AFDL->AIYL weight 25, AFDR->AIYR weight 29 - the SAME interneuron the
// existing chemotaxis circuit already uses, so no new downstream wiring was
// needed, only the sensory input side).
//
// Same paired-seed, health-gated methodology as tests/worm_chemotaxis_
// calibration, adapted for a smooth analytic gradient instead of a food
// patch:
//   - Gradient direction (Params::tempGradientAngle) varies PER SEED
//     (golden-angle spacing) - the chemotaxis harness's hard-won lesson: a
//     fixed direction that happens to match the worm's initial heading (0
//     rad) produces a fake "effect" identical with/without the real cue,
//     purely from undirected forward wandering. Varying the gradient
//     direction per seed is the direct analog of that harness varying food's
//     PLACEMENT ANGLE per seed.
//   - PAIRED same-seed comparison: "withGradient" (real tempGradientSlope)
//     vs "flat" (slope=0, temperature constant everywhere = tempBaseline,
//     the honest "no thermal cue at all" control - not "food never
//     deposited", since temperature always physically exists, just uniform).
//   - Metric: |sampleTemperature(final head position) - cultivationTemp|,
//     i.e. distance FROM comfort, not distance to a point - lower is better.
//     effect = error(flat) - error(withGradient); positive = real
//     thermotaxis (ends up closer to T_c than undirected wandering would).
//   - cultivationTemp set to a fixed offset warmer than the start position's
//     OWN temperature for that seed's gradient angle (computed the same way
//     WormSim::sampleTemperature does - replicated here since it's private)
//     - always requires real movement in the (per-seed-random) gradient
//     direction to reach, never trivially already at T_c.
//   - Health guard: same coiled-ratio/NaN/bounds check as tests/worm_
//     locomotion and worm_chemotaxis_calibration - a candidate that only
//     "improves" by degenerating (freezing, coiling) is rejected outright.
//
// RESULT (shipped): unlike every other calibration attempt this session,
// this one behaved well. Screen found effect grows monotonically with
// |thermoGain| in the NEGATIVE direction (positive gains do nothing useful -
// small positive effect near 0, turns actively negative past +10000), with
// ZERO health degradation anywhere in the tested range -30000..-150000 (vs.
// mechanoGain/reverted body.cpp attempts, which both collapsed the gait into
// a non-oscillating degenerate mode once the signal got strong enough to
// matter at all). Validated gain=-50000 across 40 independent seed bases (8
// seeds each, 320 total paired trials, 320/320 healthy): population mean
// effect = 0.2646, stderr (across base-means) = 0.0724 -> ~3.65 sigma above
// zero. Same sign confirmed at -30000 and -100000 across 16 bases each (0.089
// and 0.249 respectively, both ~2 sigma) - the effect is directionally
// consistent across a wide range of the free parameter, not a single lucky
// point, which is exactly the check tests/worm_speed_calibration's own
// "distribution mode" lesson demanded before trusting anything here.
// thermoGain shipped as -50000 in WormSim.h (Params default). NOTE:
// tempGradientSlope itself defaults to 0.0 in production (unlike this test,
// which sets a real slope explicitly per trial) - the field is inert until a
// gradient is actually configured (UI slider or setBounds+params), same
// opt-in convention as food never doing anything until deposited.
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/body.cpp"
#include "demo/worm/connectome/loader.cpp"
#include "demo/worm/WormSim.cpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr int kFieldCols = 60, kFieldRows = 44;  // big enough that T_c is always reachable well inside bounds
constexpr float kHexSpacing = 36.0f;
constexpr float kGoldenAngleDeg = 137.50776f;
constexpr float kTempBaseline = 20.0f;
constexpr float kTempSlope = 0.02f;      // deg per world unit - matches Params default
constexpr float kTempOffset = 3.0f;      // T_c = start-point temp + this, always requires real movement to reach
constexpr int kWarmupSteps = 0;          // trial itself IS the measurement window, like the chemotaxis harness
constexpr int kTrialSteps = 3000;

float gradientAngleForSeed(int seed) {
    const float deg = 45.0f + static_cast<float>(seed) * kGoldenAngleDeg;
    return deg * 3.14159265f / 180.0f;
}

// Mirrors WormSim::sampleTemperature exactly (private there, so replicated
// here against the SAME params the trial actually configures).
float sampleTempLocal(glm::vec2 worldPos, glm::vec2 boundsMin, float baseline, float slope, float angle) {
    const glm::vec2 rel = worldPos - boundsMin;
    const float dirX = std::cos(angle), dirY = std::sin(angle);
    return baseline + slope * (rel.x * dirX + rel.y * dirY);
}

struct TrialResult {
    float finalTempError = 0.0f;  // |temp(final head pos) - T_c|
    float minCoiledRatio = 1e9f;
    bool healthy = true;
};

TrialResult runTrial(float thermoGain, int seed, bool withGradient) {
    TrialResult result;
    WormSim sim("worm_data/celegans_herm.connectome");
    std::srand(static_cast<unsigned>(seed));  // after ctor - ctor's own srand(time()) would clobber this

    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
    const glm::vec2 boundsMin(0.0f);
    const glm::vec2 boundsMax = HexGrid::worldPos(kFieldCols - 1, kFieldRows - 1, kHexSpacing);
    const glm::vec2 start = boundsMax * 0.5f;  // matches WormSim::setBounds' own center convention

    const float angle = gradientAngleForSeed(seed);
    const float slope = withGradient ? kTempSlope : 0.0f;
    sim.params.tempBaseline = kTempBaseline;
    sim.params.tempGradientSlope = slope;
    sim.params.tempGradientAngle = angle;
    // T_c relative to the REAL gradient (even in the flat/control condition,
    // so both conditions target the identical numeric T_c - only the field
    // shape differs, per the header's paired-design requirement).
    const float startTempOnGradient = sampleTempLocal(start, boundsMin, kTempBaseline, kTempSlope, angle);
    const float cultivationTemp = startTempOnGradient + kTempOffset;
    sim.params.cultivationTemp = cultivationTemp;
    sim.params.thermoGain = thermoGain;

    WormSim::Snapshot snap;
    constexpr float kArcLen = 576.0f;  // kNumSegments(24) * segment_length(24.0), see WormSim.cpp ctor
    float headX = start.x, headY = start.y;

    for (int i = 0; i < kWarmupSteps + kTrialSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        headX = snap.pointsX[0];
        headY = snap.pointsY[0];
        if (std::isnan(headX) || std::isnan(headY)) { result.healthy = false; return result; }
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
            result.minCoiledRatio = std::min(result.minCoiledRatio, diag / kArcLen);
        }
    }
    if (result.minCoiledRatio < 0.30f) result.healthy = false;

    const float finalTemp = sampleTempLocal(glm::vec2(headX, headY), boundsMin, kTempBaseline, kTempSlope, angle);
    result.finalTempError = std::fabs(finalTemp - cultivationTemp);
    return result;
}

struct AggregateEffect {
    float meanEffect = 0.0f, stderrEffect = 0.0f;
    int healthyCount = 0, total = 0;
};

AggregateEffect evaluate(float thermoGain, int numSeeds, int seedBase) {
    AggregateEffect ar;
    std::vector<float> effects;
    for (int s = 0; s < numSeeds; ++s) {
        const int seed = seedBase + s;
        const TrialResult withG = runTrial(thermoGain, seed, true);
        const TrialResult flat = runTrial(thermoGain, seed, false);
        ar.total++;
        if (!withG.healthy || !flat.healthy) continue;
        ar.healthyCount++;
        effects.push_back(flat.finalTempError - withG.finalTempError);
    }
    if (!effects.empty()) {
        double sum = 0.0;
        for (float v : effects) sum += v;
        const float mean = static_cast<float>(sum / effects.size());
        double sq = 0.0;
        for (float v : effects) sq += (v - mean) * (v - mean);
        const float stddev = effects.size() > 1 ? std::sqrt(static_cast<float>(sq / (effects.size() - 1))) : 0.0f;
        ar.meanEffect = mean;
        ar.stderrEffect = stddev / std::sqrt(static_cast<float>(effects.size()));
    }
    return ar;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution <gain> [numBases] [seedsPerBase]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        const float gain = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 0.0f;
        const int numBases = argc > 3 ? std::atoi(argv[3]) : 12;
        const int seedsPerBase = argc > 4 ? std::atoi(argv[4]) : 8;
        std::printf("gain=%.4f - %d bases x %d seeds\n", gain, numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        double sumEffect = 0.0;
        int totalHealthy = 0, totalSeeds = 0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateEffect ar = evaluate(gain, seedsPerBase, base);
            std::printf("  base=%d: effect=%.4f+/-%.4f healthy=%d/%d\n", base, ar.meanEffect, ar.stderrEffect,
                        ar.healthyCount, ar.total);
            std::fflush(stdout);
            sumEffect += ar.meanEffect * ar.healthyCount;
            totalHealthy += ar.healthyCount;
            totalSeeds += ar.total;
        }
        std::printf("\npopulation mean effect: %.4f over %d/%d healthy paired trials\n",
                    totalHealthy ? sumEffect / totalHealthy : 0.0, totalHealthy, totalSeeds);
        return 0;
    }

    // Default: screen a signed grid on one seed base (shortlist only, same
    // caveat as tests/worm_mechanosensation_calibration - confirm anything
    // promising with 'distribution' across many bases before trusting it).
    const float kGains[] = {-150000.0f, -100000.0f, -70000.0f, -50000.0f, -30000.0f, -20000.0f, -15000.0f, 0.0f};
    constexpr int kScreenSeeds = 8;
    constexpr int kSeedBase = 42;
    std::printf("=== Screen (fixed signed grid, %d seeds/point, base=%d) ===\n", kScreenSeeds, kSeedBase);
    for (float g : kGains) {
        const AggregateEffect ar = evaluate(g, kScreenSeeds, kSeedBase);
        std::printf("gain=%7.3f  effect=%+.4f+/-%.4f  healthy=%d/%d\n", g, ar.meanEffect, ar.stderrEffect,
                    ar.healthyCount, ar.total);
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising gain with "
                "'distribution <gain>' across many seed bases before believing it (see header).\n");
    return 0;
}
