// tests/worm_mechanosensation_calibration/main.cpp
//
// Calibration search for Params::mechanoGain (WormSim.h/.cpp) - the DVA
// mechanosensory input added this session (real stretch-receptor neuron, Li,
// Feng & Xu 2006, PMC1500850) that feeds WormBody::mechanical_load() (real
// summed drag-force magnitude from solve_propulsion, see body.cpp) into the
// network. Goal: with mechanoGain==0 (current shipped default, confirmed
// inert - see tests/worm_locomotion/tests/worm_speed_calibration, unaffected)
// the sim has a backwards-from-real-biology bug: water (dragNormal=1.7) is
// SLOWER than agar (dragNormal=40) - real C. elegans swims ~2-3x FASTER than
// it crawls (frequency compensates for weaker anisotropy). A direct physics
// fix (resist the bend-RATE itself by drag_normal_ in WormBody::step, see
// that file's own "TRIED AND REVERTED" comment) collapsed water into a
// non-oscillating static-arc degenerate state instead of speeding it up.
// This harness tries the OTHER path instead: leave body mechanics completely
// untouched (zero risk of that same collapse), and let mechanoGain feed a
// real sensory signal into the EXISTING, already-working recurrent network,
// same architecture as applyFoodDrive/applyProprioception - the network's
// own connectome weights (not a hand-picked formula) decide what a "felt
// more/less load" signal does to the gait.
//
// Health gate learned directly from today's earlier failure: coiled ratio
// and efficiency alone are NOT enough (a frozen static arc scores fine on
// both - efficiency near 1.0, since it's driving dead straight along a
// curve). Added freqHz >= kMinFreqHz to explicitly reject the degenerate
// "not actually oscillating" mode this project has now hit twice.
//
// Same discipline as tests/worm_speed_calibration/worm_chemotaxis_calibration
// after their own single-seed-base mistakes: "screen" mode uses one seed
// base to shortlist, "distribution" mode is the only thing that should ever
// be trusted to confirm a candidate - across many INDEPENDENT seed bases,
// on BOTH presets (agar must stay healthy, water must both stay healthy AND
// beat agar's speed - the actual bug being fixed).
//
// FIRST SCREEN RESULT (single base=42, signed grid -30..+30, 8 seeds/point):
// NO gain keeps both presets healthy AND makes water beat agar. Small
// |gain| (<=0.03) changes nothing measurable on either preset - same "sensory
// signal many orders of magnitude below the network's own intrinsic
// dynamics" finding as the chemotaxis investigation, just for this input.
// Past that, agar collapses FIRST and at much smaller magnitude than water
// (agar unhealthy by gain=+0.1 - freq drops to 0.000, the same static-arc
// degenerate mode as the reverted body.cpp attempt; water stays healthy to
// roughly +/-3 before doing the same). Likely reason, not yet tested: agar's
// mechanical_load() (drag_normal_=40) is ~23x water's (drag_normal_=1.7) in
// raw magnitude for similar motion (same anisotropic-drag physics, see
// body.cpp), so ONE unscaled gain constant hits the two presets at very
// different effective strengths - a fixed gain can never be "just right" for
// both at once. Next lead for whoever continues this: normalize the DVA
// input by a per-preset reference load (e.g. divide by the load measured at
// rest/idle for the CURRENT drag_normal_, refreshed when drag changes) rather
// than feeding raw mechanical_load() - real mechanoreceptors commonly rescale
// to their input's own dynamic range (Weber-Fechner-type relative encoding),
// so this isn't an ad-hoc fudge, but it needs the same distribution-mode
// validation as everything else here before it's trusted. mechanoGain
// shipped at 0.0 (confirmed inert - tests/worm_locomotion and
// worm_speed_calibration both unaffected) pending that follow-up.
//
// FOLLOW-UP: normalization implemented (WormSim.cpp::applyMechanosensation
// now divides mechanical_load() by params.dragNormal before the gain).
// Re-screened the same signed grid: the hypothesis was RIGHT - agar and
// water now go unhealthy at almost the same gain magnitude (both fine
// through +/-1.0, both fail by +/-3.0-10.0) instead of agar breaking ~30x
// earlier than water. But this does NOT fix the actual bug: across the
// entire healthy range, water's speed (0.00052-0.00071 BL/s) never once
// exceeds agar's (0.00219-0.00262 BL/s) - both move together in roughly the
// same direction as |gain| grows, never crossing. Normalizing the SCALE of
// the signal was necessary but not sufficient - the sign/topology of DVA's
// real synaptic path to the motor circuit in this connectome apparently
// doesn't push water's relative speed up faster than agar's within a single
// scalar gain's reach. Not chased further this session (single-neuron,
// single-scalar signal may just not have enough leverage over frequency
// specifically, as opposed to overall gait amplitude/coiling, which DOES
// visibly respond to gain - see efficiency/coiled columns above). Next
// options for whoever continues: (a) wider/finer grid past -30 in case the
// crossing point is further out, (b) a genuine 2D search (gain x something
// else - e.g. combine with the reverted body.cpp bend-resistance idea at a
// much smaller magnitude than tried before, now that DVA gives the network
// forewarning of load rather than the mechanics being blindly overridden),
// (c) accept a single DVA scalar input isn't enough and the real fix needs
// the full B-class stretch-receptor circuit's actual per-cell targets (not
// one lumped neuron) - this project's connectome already has the real DA/DB/
// VA/VB identities, so per-motor-neuron-class mechanosensory input (not just
// one DVA) is a concrete, testable next step.
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/body.cpp"
#include "demo/worm/connectome/loader.cpp"
#include "demo/worm/WormSim.cpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr float kBodyLength = 576.0f;  // kNumSegments(24) * segment_length(24.0), see WormSim.cpp ctor
constexpr int kFieldCols = 200, kFieldRows = 150;  // large - see worm_speed_calibration comment (avoid wall-bounce)
constexpr float kHexSpacing = 36.0f;
constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;  // this project's two presets, dragTangent=1.0 both

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;       // net displacement / path length, 0..1
    float minCoiledRatio = 1e9f;   // bbox diag / arc length, post-transient - health guard
    float freqHz = 0.0f;           // dominant bend frequency at a mid-body position
    bool healthy = true;
};

Measurement runTrial(float mechanoGain, int seed, float dragNormal, int warmupSteps, int measureSteps,
                      int freqPosition) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    sim.params.mechanoGain = mechanoGain;
    sim.params.dragTangent = 1.0f;
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

AggregateResult evaluate(float gain, int numSeeds, int seedBase, float dragNormal, int warmupSteps, int measureSteps,
                          int freqPosition = 12) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int s = 0; s < numSeeds; ++s) {
        const Measurement m = runTrial(gain, seedBase + s, dragNormal, warmupSteps, measureSteps, freqPosition);
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

constexpr float kMinEfficiency = 0.40f;
constexpr float kMinCoiledRatio = 0.30f;
constexpr float kMinFreqHz = 0.001f;  // reject the "static arc, not actually oscillating" degenerate mode

bool isHealthy(const AggregateResult& ar) {
    return ar.allHealthy && ar.minCoiledRatio >= kMinCoiledRatio && ar.meanEfficiency >= kMinEfficiency &&
           ar.meanFreqHz >= kMinFreqHz;
}

void printAgg(const char* label, const AggregateResult& ar) {
    std::printf("  %-6s speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f healthy=%s\n", label, ar.meanBLps,
                ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                isHealthy(ar) ? "yes" : "NO");
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution <gain> [numBases] [seedsPerBase] [warmup] [measure]
    // Validates ONE candidate gain across many independent seed bases, on
    // BOTH presets - the only mode that should be trusted (see header).
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        const float gain = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 0.0f;
        const int numBases = argc > 3 ? std::atoi(argv[3]) : 12;
        const int seedsPerBase = argc > 4 ? std::atoi(argv[4]) : 8;
        const int warmupSteps = argc > 5 ? std::atoi(argv[5]) : 300;
        const int measureSteps = argc > 6 ? std::atoi(argv[6]) : 2500;
        std::printf("gain=%.5f - %d bases x %d seeds\n", gain, numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0;
        double sumAgar = 0.0, sumWater = 0.0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(gain, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(gain, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool waterFaster = waterOk && agarOk && water.meanBLps > agar.meanBLps;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  water>agar: %s\n", waterFaster ? "YES" : "no");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (waterFaster) ++waterBeatsAgarCount;
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s), water healthy=%d/%d (mean "
                    "%.5f BL/s), water>agar in %d/%d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    waterBeatsAgarCount, numBases);
        return 0;
    }

    // Default: screen a fixed grid of candidate gains (signed - we don't know
    // a priori whether DVA's real synaptic weights in this connectome make
    // more sensed load excitatory or inhibitory to the locomotor circuit,
    // that's an empirical fact of the loaded weights, not something to guess)
    // on ONE seed base, both presets, report health + speed for each. Not a
    // final verdict - shortlist only, per header.
    const float kGains[] = {-30.0f, -10.0f, -3.0f, -1.0f, -0.3f, -0.1f, -0.03f, -0.01f, -0.003f, -0.001f,
                             0.001f, 0.003f, 0.01f, 0.03f, 0.1f,  0.3f,  1.0f,   3.0f,   10.0f,   30.0f};
    constexpr int kScreenSeeds = 8;
    constexpr int kWarmup = 300, kMeasure = 2500;
    constexpr int kSeedBase = 42;

    std::printf("=== Baseline (gain=0) ===\n");
    const AggregateResult baseAgar = evaluate(0.0f, kScreenSeeds, kSeedBase, kDragAgar, kWarmup, kMeasure);
    const AggregateResult baseWater = evaluate(0.0f, kScreenSeeds, kSeedBase, kDragWater, kWarmup, kMeasure);
    printAgg("agar", baseAgar);
    printAgg("water", baseWater);

    std::printf("\n=== Screen (fixed signed grid, %d seeds/point) ===\n", kScreenSeeds);
    for (float g : kGains) {
        const AggregateResult agar = evaluate(g, kScreenSeeds, kSeedBase, kDragAgar, kWarmup, kMeasure);
        const AggregateResult water = evaluate(g, kScreenSeeds, kSeedBase, kDragWater, kWarmup, kMeasure);
        const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
        const bool waterFaster = waterOk && agarOk && water.meanBLps > agar.meanBLps;
        std::printf("gain=%8.4f  agar[speed=%.5f freq=%.4f eff=%.3f coiled=%.3f %s]  water[speed=%.5f freq=%.4f "
                    "eff=%.3f coiled=%.3f %s]  %s\n",
                    g, agar.meanBLps, agar.meanFreqHz, agar.meanEfficiency, agar.minCoiledRatio,
                    agarOk ? "ok" : "FAIL", water.meanBLps, water.meanFreqHz, water.meanEfficiency,
                    water.minCoiledRatio, waterOk ? "ok" : "FAIL", waterFaster ? "<-- water>agar" : "");
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising gain with "
                "'distribution <gain>' across many seed bases before believing it (see header).\n");
    return 0;
}
