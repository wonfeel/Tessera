// tests/worm_local_mechanosensation_calibration/main.cpp
//
// Calibration search for Params::localMechanoGain (WormSim.h/.cpp) - a NEW
// axis added this session, distinct from Params::mechanoGain (DVA global
// scalar, see tests/worm_mechanosensation_calibration - EXHAUSTIVELY tried,
// confirmed insufficient: even after correct Weber-Fechner normalization,
// water's speed never once crosses agar's across the entire healthy gain
// range, because a single scalar into one premotor interneuron doesn't have
// enough leverage over LOCAL bend frequency specifically).
//
// Root cause (see tests/worm_speed_calibration header, confirmed against
// Boyle/Berri/Cohen 2012 eLife/Frontiers paper): curvature -> angles_ in
// WormBody::step is purely kinematic (see body.cpp) - drag never resists the
// BEND itself, only the resulting translation. Real C. elegans motor neurons
// (B/D-class) are THEMSELVES local stretch receptors (Wen et al. 2012 Neuron;
// Yeon et al. 2018 Cell) - the swim/crawl frequency difference in Boyle et
// al.'s validated model emerges PASSIVELY from a single set of neural
// parameters once drag changes, with NO separate "sense the environment"
// step, because proprioceptive feedback is local and continuous, not a
// lumped global signal. This project's applyProprioception() already does
// local, windowed, per-motor-neuron feedback of body ANGLE (shape) - this
// harness adds the missing half: the same window, the same motor neurons,
// but fed WormBody::segment_load() (per-segment |friction force|, i.e. what
// that specific stretch receptor's patch of cuticle is actually resisting
// against), not one global sum. This is the closest this architecture can
// get to Boyle et al.'s actual mechanism without abandoning the real
// (401-neuron Cook et al. 2019) connectome for their idealized toy circuit.
//
// Health gate: same as tests/worm_mechanosensation_calibration (efficiency,
// coiled ratio, freqHz - reject the "static frozen arc" degenerate mode)
// PLUS, learned the hard way from this session's synapse-sign-calibration
// near-miss (shipped, then caught by tests/worm_locomotion showing
// consistent ~2.3-3.1 rad single-step heading reversals that NO metric in
// that search/confirm/verify/adversarial-review pipeline had ever measured):
// max |heading delta| in one step, tracked INSIDE this search from the start,
// not bolted on after the fact. Historical healthy baseline is ~0.03 rad;
// tests/worm_locomotion's own hard FAIL line is 3.2 rad. This harness uses a
// much tighter internal gate (kMaxHeadingDeltaRad, see below) so a candidate
// merely "elevated but technically under 3.2" - which is exactly what got
// shipped and reverted last time - is rejected here before it ever reaches
// that regression test.
//
// Same discipline as every other calibration file in this project: "screen"
// (one seed base) shortlists only, "distribution" (many independent seed
// bases) is the only mode that should ever be trusted - see this project's
// history of single-seed-base illusions (tests/worm_speed_calibration,
// tests/worm_synapse_speed_calibration headers).
//
// RESULT (screen, signed grid -8..+8, one base): heading-delta is a non-issue
// here (max ~0.005-0.02 rad across the ENTIRE grid, nowhere near
// kMaxHeadingDeltaRad=0.5 let alone tests/worm_locomotion's 3.2 rad FAIL line)
// - this local, small, physically-bounded-by-drag-normalization signal does
// NOT reproduce the heading-reversal pathology that sank the synapse-sign
// attempt. But frequency is: freq==0.0080Hz (period ~125s, matches the
// pre-existing "too slow" diagnosis in tests/worm_speed_calibration) at
// gain=0 for BOTH presets, and stays essentially PINNED at that same value
// everywhere the gait is healthy (|gain|<=0.5 on the negative side,
// <=0.05 on the positive side) - then, past that, jumps straight to freq=0
// (frozen static arc) rather than continuously rising. There is no gain
// value where frequency moves smoothly through the middle.
//
// CONFIRMED via distribution mode (gain=-0.5, the best screen candidate,
// 12 independent bases x 8 seeds): real and robust (not a single-base
// illusion this time) but WAY short of the bug - agar 12/12 healthy
// (mean 0.00226 BL/s, unchanged from baseline), water 12/12 healthy (mean
// 0.00064 BL/s, up from baseline's 0.00054 - a genuine +19% bump) - but
// water>agar in 0/12 bases (needs to roughly 4x from here). Conclusion:
// this is the fourth independent axis this project has tried for the
// swim>crawl bug (direct bend-rate drag resistance in body.cpp; global DVA
// mechanoGain in tests/worm_mechanosensation_calibration; leak/capacitance
// retuning in tests/worm_speed_calibration's Round 1, reverted for not
// generalizing across seed bases; and this one) - all four hit the SAME
// wall: this connectome's leaky-integrator network, as currently loaded,
// holds a genuinely narrow, rigid oscillation regime (~125s period) that
// resists being retuned by any external perturbation tried so far, healthy
// or not. This is not a sign/topology/scale bug fixable by one more search
// - it looks like a structural property of the loaded leak/capacitance/
// synaptic-weight values themselves. Shipped at localMechanoGain=0.0
// (fully inert - see below), infrastructure (segment_load(),
// applyProprioception's local-load term) kept for whoever takes on the
// leak/capacitance axis again with this signal available as a companion.
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
constexpr float kPi = 3.14159265358979323846f;

// Tight internal gate, well below tests/worm_locomotion's hard 3.2 rad FAIL
// line and well above the ~0.03 rad historical healthy baseline - catches
// "elevated but not yet failing" candidates, which is exactly the failure
// mode that slipped through last time on a different calibration axis.
constexpr float kMaxHeadingDeltaRad = 0.5f;

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;       // net displacement / path length, 0..1
    float minCoiledRatio = 1e9f;   // bbox diag / arc length, post-transient - health guard
    float freqHz = 0.0f;           // dominant bend frequency at a mid-body position
    float maxAbsHeadingDelta = 0.0f;  // rad, single-step - see tests/worm_locomotion
    bool healthy = true;
};

Measurement runTrial(float localGain, int seed, float dragNormal, int warmupSteps, int measureSteps,
                      int freqPosition) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    sim.params.mechanoGain = 0.0f;  // keep the already-exhausted global DVA axis off - isolate this one
    sim.params.localMechanoGain = localGain;
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

AggregateResult evaluate(float localGain, int numSeeds, int seedBase, float dragNormal, int warmupSteps,
                          int measureSteps, int freqPosition = 12) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int s = 0; s < numSeeds; ++s) {
        const Measurement m = runTrial(localGain, seedBase + s, dragNormal, warmupSteps, measureSteps, freqPosition);
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
constexpr float kMinFreqHz = 0.001f;  // reject the "static arc, not actually oscillating" degenerate mode

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
    // ./exe distribution <gain> [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        const float gain = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 0.0f;
        const int numBases = argc > 3 ? std::atoi(argv[3]) : 12;
        const int seedsPerBase = argc > 4 ? std::atoi(argv[4]) : 8;
        const int warmupSteps = argc > 5 ? std::atoi(argv[5]) : 300;
        const int measureSteps = argc > 6 ? std::atoi(argv[6]) : 2500;
        std::printf("localMechanoGain=%.5f - %d bases x %d seeds\n", gain, numBases, seedsPerBase);
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

    // Default: screen a signed grid on ONE seed base, both presets. Range is
    // narrower than worm_mechanosensation_calibration's (-30..30) because
    // this signal is now LOCAL (per-motor-neuron, same magnitude order as
    // the existing proprioceptiveGain default of 4.0) rather than one global
    // scalar - not comparable units, start from a scale that matches the
    // existing, already-working proprioceptive feedback it's added to.
    const float kGains[] = {-8.0f, -4.0f, -2.0f, -1.0f, -0.5f, -0.2f, -0.1f, -0.05f, -0.01f,
                             0.01f, 0.05f, 0.1f,  0.2f,  0.5f,  1.0f,  2.0f,  4.0f,   8.0f};
    constexpr int kScreenSeeds = 8;
    constexpr int kWarmup = 300, kMeasure = 2500;
    constexpr int kSeedBase = 42;

    std::printf("=== Baseline (localMechanoGain=0) ===\n");
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
        std::printf("gain=%8.4f  agar[speed=%.5f freq=%.4f eff=%.3f coiled=%.3f headD=%.3f %s]  "
                    "water[speed=%.5f freq=%.4f eff=%.3f coiled=%.3f headD=%.3f %s]  %s\n",
                    g, agar.meanBLps, agar.meanFreqHz, agar.meanEfficiency, agar.minCoiledRatio,
                    agar.maxHeadingDelta, agarOk ? "ok" : "FAIL", water.meanBLps, water.meanFreqHz,
                    water.meanEfficiency, water.minCoiledRatio, water.maxHeadingDelta, waterOk ? "ok" : "FAIL",
                    waterFaster ? "<-- water>agar" : "");
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising gain with "
                "'distribution <gain>' across many seed bases before believing it (see header).\n");
    return 0;
}
