// tests/worm_roaming_dwelling_calibration/main.cpp
//
// Calibration search for Params::serotoninGain (WormSim.h/.cpp) - the ADF/NSM
// serotonergic input added this session (both are real serotonin-producing
// neurons in C. elegans). Real biology (Flavell et al. 2013, Cell): serotonin
// promotes DWELLING (slower crawling, more turning - typically while on/near
// food); the neuropeptide PDF-1 promotes the opposite ROAMING state (faster,
// straighter). This harness only implements the serotonin half (PDF-1/PDFR
// not wired this session - see WormSim.cpp's applySerotoninDrive comment) -
// m_foodExposureTone is a slow (tens-of-seconds) EMA of real sampled scent,
// feeding ADF/NSM the same way DVA/AFD already feed their own real sensory
// correlate, letting the network's own real synaptic weights (not application
// code) decide what a "sustained food experience" signal does downstream.
//
// Same paired-seed, health-gated methodology as tests/worm_chemotaxis_
// calibration and tests/worm_thermotaxis_calibration:
//   - PAIRED same-seed comparison: "onFood" (worm sits on a continuously
//     replenished food patch - a real bacterial LAWN, not a single crumb that
//     depletes; re-deposited every step at a fixed point so it never runs
//     dry, unlike the one-shot deposits in the chemotaxis/speed harnesses)
//     vs "offFood" (no food anywhere, tone stays ~0 the whole trial).
//   - LONG warmup (default 3000 steps = 150s at dt=0.05, ~5x the default
//     serotoninTau=30s, ~99% EMA convergence) before the measurement window -
//     this is a slow behavioral-state signal, not a fast reflex; measuring
//     before it has actually converged would just be noise.
//   - Metric: bodyLengthsPerSec over the measurement window (same convention
//     as tests/worm_speed_calibration) - effect = speed(offFood) -
//     speed(onFood); POSITIVE = real dwelling-like slowdown on food, matching
//     the real direction Flavell et al. 2013 report.
//   - Health guard: same coiled-ratio/NaN/bounds check as the other harnesses,
//     PLUS a minimum bend frequency gate (freqHz >= kMinFreqHz) - learned
//     directly from tests/worm_mechanosensation_calibration's failure mode
//     (a frozen static arc scores fine on efficiency/coiled-ratio alone). NOTE:
//     this gate must be measured over the FULL warmup+measure window (225s),
//     not just the 75s measure window - the established bend period here is
//     ~125s, so a 75s window has an expected zero-crossing count near 1 and
//     reads 0 (unhealthy) by pure chance disturbingly often, which is exactly
//     what happened on the first version of this file (EVERY gain, including
//     0.0 - provably fine per tests/worm_locomotion - read 0/6 healthy).
//
// RESULT: NOT a clean win like tests/worm_thermotaxis_calibration - closer to
// tests/worm_mechanosensation_calibration's outcome. Two distinct regimes
// found, neither shippable as-is:
//   (a) NEGATIVE gain (suppresses ADF/NSM in proportion to food-tone): stays
//       healthy across the full screened range, and the effect is
//       EXTREMELY reproducible - gain=-1000 validated across 16 independent
//       bases gives effect=-0.00110 BL/s with base-to-base agreement tighter
//       than any other calibration in this project's history (every base
//       within -0.00094..-0.00124). But the SIGN is backwards from the real
//       biology this feature models: Flavell et al. 2013 says MORE serotonin
//       (excitation) promotes dwelling (slower); here, SUPPRESSING the
//       serotonin-associated input makes the worm FASTER on food, the
//       opposite relationship. Health also isn't perfect at this magnitude
//       (81/96 healthy across the 16-base validation, not 100%).
//   (b) Small POSITIVE gain (excites ADF/NSM in proportion to food-tone, the
//       direction that actually matches Flavell et al.'s excitation->dwelling
//       story): gain=2.0 gives a real, very consistently-signed effect
//       (+0.00002 BL/s on 16/16 independent bases) in the CORRECT direction,
//       but the magnitude is two orders of magnitude smaller than (a), health
//       is inconsistent across bases (68/96 = 71%, worse than the single-base
//       screen's 6/6 suggested), and anything past gain~10 collapses health
//       outright (0/6 by gain=300, same asymmetric fragility pattern as
//       mechanoGain/AFD's positive-gain side).
// Neither regime is a validated, healthy, correctly-signed, meaningfully-
// sized effect simultaneously. serotoninGain shipped at 0.0 (confirmed inert
// - tests/worm_locomotion unaffected). Whoever continues this: (a) is the
// stronger, more reproducible signal by far, so worth understanding WHY it's
// sign-inverted relative to the literature before dismissing it outright -
// possibly this connectome's real ADF/NSM->downstream synaptic signs already
// encode something closer to (a)'s relationship than Flavell's, in which case
// (a) may be the more honest calibration for THIS specific network's actual
// wiring, not (b) - needs the same kind of Jacobian/pathway tracing tests/
// worm_network_eigenmodes already does for other circuits before deciding.
//
// ROUND 2 RESULT (discrete pump, same session): the shape hypothesis was
// RIGHT, partially - health is dramatically better-behaved now. Single-base
// screen: FULL health (6/6) across the entire range -10000..+100, vs round
// 1's asymmetric collapse (positive gain unhealthy by +30, fully dead by
// +300). The correct (Flavell-matching, excitatory) direction now survives
// to roughly +600 before collapsing (vs +100-300 before) - a real,
// meaningful stability improvement from feeding a physiologically-shaped
// (bursty, gated-on-food) signal instead of a smooth EMA. BUT: the effect
// size in the safely-healthy zone stayed small - gain=150 validated across
// 16 bases: correct sign on 14/16 bases, population mean +0.00001 BL/s, but
// health only 51/96 (53%) across the full validation (worse than the
// single-base screen suggested, same "single base isn't representative"
// lesson as always). Still not a validated, meaningfully-sized, reliably-
// healthy effect - but the STABILITY finding itself (signal shape, not just
// magnitude, measurably changes how much room this network has before
// collapsing) is real and worth keeping in mind for any future sensory
// pathway on this connectome, independent of whether roaming/dwelling
// specifically ever ships. serotoninGain stays at 0.0 (inert, confirmed via
// tests/worm_locomotion).
//
// ROUND 2 (same session, discrete-pump mechanism): the smooth-EMA input
// above was replaced in WormSim.cpp's applySerotoninDrive with a discrete
// per-pharyngeal-pump impulse (real pump rate 4.3 Hz, Avery & Horvitz 1990/
// Raizen et al. 1995 - not a free parameter) gated on real food presence at
// the head - the hypothesis being that ADF/NSM's real physiological input is
// bursty, not a smooth average, and that this network's demonstrated extreme
// sensitivity to input SHAPE (not just magnitude - see body.cpp's reverted
// bend-resistance attempt, mechanoGain's asymmetric collapse) might behave
// differently against a correctly-shaped signal. This harness's mechanics
// (paired onFood/offFood, same seed, bodyLengthsPerSec effect, health gate)
// are UNCHANGED and apply identically to whatever mechanism WormSim.cpp
// currently implements - re-run screen/distribution fresh against the new
// code; all gain numbers documented above are from the OLD (EMA) mechanism
// and are not comparable to new results (the signal is impulse-shaped now,
// not continuously-scaled, so the useful gain magnitude is a different
// physical quantity entirely).
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

constexpr int kFieldCols = 200, kFieldRows = 150;  // large - avoid wall-bounce contaminating speed/turn metrics
constexpr float kHexSpacing = 36.0f;
constexpr float kBodyLength = 576.0f;  // kNumSegments(24) * segment_length(24.0), see WormSim.cpp ctor
constexpr int kWarmupSteps = 3000;     // 150s at dt=0.05 - ~5x default serotoninTau, EMA converged
constexpr int kMeasureSteps = 1500;    // 75s
constexpr float kMinFreqHz = 0.001f;   // reject the "static arc, not oscillating" degenerate mode
constexpr int kFreqPosition = 12;

struct TrialResult {
    float bodyLengthsPerSec = 0.0f;
    float minCoiledRatio = 1e9f;
    float freqHz = 0.0f;
    bool healthy = true;
};

TrialResult runTrial(float gain, int seed, bool onFood) {
    TrialResult result;
    WormSim sim("worm_data/celegans_herm.connectome");
    sim.params.serotoninGain = gain;
    std::srand(static_cast<unsigned>(seed));  // after ctor - ctor's own srand(time()) would clobber this

    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
    const glm::vec2 boundsMax = HexGrid::worldPos(kFieldCols - 1, kFieldRows - 1, kHexSpacing);
    const glm::vec2 start = boundsMax * 0.5f;
    const float dt = sim.params.dt.load();

    WormSim::Snapshot snap;
    auto centroid = [](const WormSim::Snapshot& s, float& cx, float& cy) {
        cx = 0.0f; cy = 0.0f;
        for (std::size_t i = 0; i < s.pointsX.size(); ++i) { cx += s.pointsX[i]; cy += s.pointsY[i]; }
        cx /= static_cast<float>(s.pointsX.size());
        cy /= static_cast<float>(s.pointsY.size());
    };

    // Zero-crossing tracking spans warmup+measure (225s total, not just the
    // 75s measure window) - the established bend period here is ~125s (see
    // tests/worm_speed_calibration, freq~0.008Hz), so a 75s window has an
    // expected crossing count near 1 - a real, healthy, oscillating gait
    // would still frequently read 0 crossings in that short a window by pure
    // chance (Poisson-thin sampling), which is exactly what made EVERY gain
    // in the first screen (including 0.0, which is provably fine per
    // tests/worm_locomotion) read unhealthy. The full 225s window has an
    // expected count near 3.6 (P(0 crossings) ~ e^-3.6 ~ 2.7%), a much more
    // reliable gate against the real failure mode (a truly frozen arc) while
    // rarely false-failing a genuinely healthy oscillation.
    float prevDeviation = 0.0f;
    bool havePrevDeviation = false;
    int zeroCrossings = 0;
    auto trackFreq = [&]() {
        const auto& dev = sim.lastCurvatureDeviation();
        if (kFreqPosition < static_cast<int>(dev.size())) {
            const float d = dev[static_cast<std::size_t>(kFreqPosition)];
            if (havePrevDeviation && ((d > 0.0f) != (prevDeviation > 0.0f))) ++zeroCrossings;
            prevDeviation = d;
            havePrevDeviation = true;
        }
    };

    for (int i = 0; i < kWarmupSteps; ++i) {
        if (onFood) sim.depositFood(start, dt);  // continuously replenished lawn, never depletes
        sim.step();
        trackFreq();
    }

    sim.snapshot(snap);
    float prevX, prevY;
    centroid(snap, prevX, prevY);
    double pathLen = 0.0;

    for (int i = 0; i < kMeasureSteps; ++i) {
        if (onFood) sim.depositFood(start, dt);
        sim.step();
        trackFreq();
        sim.snapshot(snap);
        float x, y;
        centroid(snap, x, y);
        pathLen += std::sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
        prevX = x; prevY = y;

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
        if (i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
            for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            result.minCoiledRatio = std::min(result.minCoiledRatio, diag / kBodyLength);
        }
    }
    if (result.minCoiledRatio < 0.30f) result.healthy = false;

    const float measureSeconds = static_cast<float>(kMeasureSteps) * dt;
    const float fullTrialSeconds = static_cast<float>(kWarmupSteps + kMeasureSteps) * dt;
    result.bodyLengthsPerSec = static_cast<float>(pathLen) / measureSeconds / kBodyLength;
    result.freqHz = static_cast<float>(zeroCrossings) / 2.0f / fullTrialSeconds;
    if (result.freqHz < kMinFreqHz) result.healthy = false;
    return result;
}

struct AggregateEffect {
    float meanEffect = 0.0f, stderrEffect = 0.0f;
    int healthyCount = 0, total = 0;
};

AggregateEffect evaluate(float gain, int numSeeds, int seedBase) {
    AggregateEffect ar;
    std::vector<float> effects;
    for (int s = 0; s < numSeeds; ++s) {
        const int seed = seedBase + s;
        const TrialResult onFood = runTrial(gain, seed, true);
        const TrialResult offFood = runTrial(gain, seed, false);
        ar.total++;
        if (!onFood.healthy || !offFood.healthy) continue;
        ar.healthyCount++;
        effects.push_back(offFood.bodyLengthsPerSec - onFood.bodyLengthsPerSec);
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
            std::printf("  base=%d: effect=%.5f+/-%.5f healthy=%d/%d\n", base, ar.meanEffect, ar.stderrEffect,
                        ar.healthyCount, ar.total);
            std::fflush(stdout);
            sumEffect += ar.meanEffect * ar.healthyCount;
            totalHealthy += ar.healthyCount;
            totalSeeds += ar.total;
        }
        std::printf("\npopulation mean effect: %.5f BL/s over %d/%d healthy paired trials\n",
                    totalHealthy ? sumEffect / totalHealthy : 0.0, totalHealthy, totalSeeds);
        return 0;
    }

    // Default: screen a signed grid on one seed base (shortlist only - confirm
    // anything promising with 'distribution' across many bases before trusting
    // it, per the header and this project's own repeated lesson on that).
    const float kGains[] = {100.0f, 150.0f, 200.0f, 250.0f, 300.0f, 400.0f, 500.0f, 600.0f, 700.0f, 800.0f};
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    std::printf("=== Screen (fixed signed grid, %d seeds/point, base=%d) ===\n", kScreenSeeds, kSeedBase);
    for (float g : kGains) {
        const AggregateEffect ar = evaluate(g, kScreenSeeds, kSeedBase);
        std::printf("gain=%9.2f  effect=%+.5f+/-%.5f BL/s  healthy=%d/%d\n", g, ar.meanEffect, ar.stderrEffect,
                    ar.healthyCount, ar.total);
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising gain with "
                "'distribution <gain>' across many seed bases before believing it (see header).\n");
    return 0;
}
