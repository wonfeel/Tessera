// tests/worm_pdf1_reversal_diagnostic/main.cpp
//
// DIAGNOSTIC, not a calibration search - answers a specific question raised
// by tests/worm_pdf1_calibration's own RESULT section: gain=-0.10 robustly
// INCREASES turnRateRadPerSec (mean |head-segment heading delta| per
// second), the wrong direction versus Flavell et al. 2013's roaming
// prediction, across the ENTIRE tau_release sweep. But turnRateRadPerSec
// measures how fast the HEAD SEGMENT's absolute orientation rotates
// step-to-step - during entirely normal forward undulatory crawling this is
// ALREADY nonzero (the head sweeps side to side as part of the travelling
// wave itself). An increase in it could mean either (a) genuinely more
// reversal/reorientation events (what Flavell et al. actually measure), or
// (b) simply a wider undulatory sweep AMPLITUDE during otherwise still-
// forward crawling - two different things turnRateRadPerSec cannot tell
// apart. This file checks which one it actually is.
//
// A true reversal is a change in which direction the bending wave
// PROPAGATES along the body (head->tail = forward locomotion, tail->head =
// backward) - not a change in any single position's oscillation amplitude.
// Method: track curvature deviation (lastCurvatureDeviation()) at a
// near-head position (posHead) and a near-tail position (posTail, most of
// the body length away). For each zero-crossing of the near-head signal,
// find the nearest SAME-DIRECTION zero-crossing of the near-tail signal
// within +/-kMaxLagSeconds and record the signed time lag between them.
// lag>0 (tail crossing follows head crossing) = forward propagation for
// that cycle; lag<0 = backward (reversal-like) propagation. This is a
// PHASE-ONLY measure - amplitude cancels out entirely, so unlike
// turnRateRadPerSec it cannot be confounded by hypothesis (b) above.
// Reported alongside: headDevRMS (RMS of the near-head deviation signal
// itself over the window) - if this tracks the turnRate increase 1:1, that
// directly supports (b); if backward-propagation fraction tracks it
// instead, that supports (a) and would mean the ORIGINAL Flavell-style
// prediction was directionally right after all, just mis-measured.
//
// Long window (15000 steps = 750s, ~6x the ~125s established bend period at
// identity, see worm_roaming_dwelling_calibration/worm_pdf1_calibration
// headers) - needed to collect enough zero-crossings per position for the
// lag statistics to mean anything; a single calibration-length (125s) window
// would give at most one crossing per position, not enough to characterize
// propagation direction at all.
//
// RESULT (10 seeds, base=42): NEITHER hypothesis alone - BOTH are real.
//   headDevRMS:   identity=0.3137  pdf1=0.3688  (ratio 1.176, +17.6%)
//   turnRate:     identity=0.0220  pdf1=0.0253  (ratio 1.154, +15.4%)
//   backwardFrac: identity=27/46 (58.7%)  pdf1=32/42 (76.2%)
// Amplitude DOES increase (+17.6%) by almost exactly the same factor as
// turnRateRadPerSec (+15.4%) - hypothesis (b) (amplitude confound) is
// real and explains a good chunk of the original finding. BUT the
// phase-only, amplitude-independent backwardFrac ALSO shifts substantially
// and consistently - identity sits at 50-67% per-seed across all 10 seeds,
// pdf1 sits at 75-80% per-seed across all 10, zero overlap between the two
// distributions. A pure amplitude artifact would leave phase relationships
// (and therefore backwardFrac) unchanged; it didn't.
// CONCLUSION: pdf1Gain=-0.10 causes a genuine, amplitude-independent shift
// toward MORE backward (tail-leads-head, reversal-like) wave propagation,
// not fewer reversals - this CONFIRMS, on a metric immune to the amplitude
// confound, the same wrong-direction conclusion tests/worm_pdf1_calibration
// already reached with turnRateRadPerSec. The original finding was not a
// measurement artifact - pdf1Gain in this implementation genuinely pushes
// this network's dynamics toward more reversal-like behavior, the opposite
// of Flavell et al. 2013's roaming prediction. Ships alongside pdf1Gain=0.0
// as further honest evidence, not a reason to revisit the gain value itself.
//
// Side finding, independent of the above: positions 0-4 (the true anatomical
// head) never cross zero over 750s at ALL (0 crossings, both conditions,
// every seed checked) despite carrying the LARGEST RMS deviation magnitude
// in the whole body (up to 0.84, see 'scan' mode) - this network's emergent
// gait is not a clean sinusoidal travelling wave over the full body length;
// the front ~20% holds a persistent, large, one-signed curl while positions
// ~5-23 do the actual oscillating. Worth keeping in mind for any future
// per-position analysis on this connectome - "position N along the body"
// does not behave uniformly.
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/body.cpp"
#include "demo/worm/connectome/loader.cpp"
#include "demo/worm/WormSim.cpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kHexSpacing = 36.0f;
constexpr int kWarmupSteps = 500;       // 25s - matches worm_pdf1_calibration's validated default
constexpr int kLongMeasureSteps = 15000; // 750s, ~6 periods
// Positions 0-4 (near the true anatomical head) turn out NOT to oscillate at
// all - see 'scan' mode: 0 zero-crossings over 750s, just a large-magnitude
// PERSISTENT one-signed bend (RMS up to 0.84). The travelling-wave region is
// actually positions ~5-23, peaking in crossing-count (most active) around
// 10-11 and tapering toward the tail. kPosHead/kPosTail below are picked
// FROM that oscillating region (not the true anatomical ends) - this
// diagnostic is about propagation direction WITHIN the part of the body
// that actually oscillates, which is not the whole body here.
constexpr int kPosHead = 9, kPosTail = 15;
constexpr float kMaxLagSeconds = 62.0f;     // ~half the established ~125s period

struct Crossing {
    float timeSeconds;
    int direction;  // +1 = rising (neg->pos), -1 = falling (pos->neg)
};

struct DiagResult {
    float headDevRMS = 0.0f;
    float turnRateRadPerSec = 0.0f;
    int forwardCount = 0, backwardCount = 0;
    double sumLag = 0.0;  // signed, over matched pairs
};

std::vector<Crossing> findCrossings(const std::vector<float>& series, float dt) {
    std::vector<Crossing> out;
    for (std::size_t i = 1; i < series.size(); ++i) {
        const float prev = series[i - 1], cur = series[i];
        if ((prev <= 0.0f) == (cur <= 0.0f)) continue;  // no sign change (treats 0 as non-positive side)
        out.push_back(Crossing{static_cast<float>(i) * dt, cur > prev ? 1 : -1});
    }
    return out;
}

DiagResult runDiagnostic(float gain, float tauRelease, int seed, bool verbose) {
    DiagResult r;
    WormSim sim("worm_data/celegans_herm.connectome");
    sim.params.pdf1Gain = gain;
    sim.params.pdf1ReleaseTau = tauRelease;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
    const float dt = sim.params.dt.load();

    for (int i = 0; i < kWarmupSteps; ++i) sim.step();

    std::vector<float> devHeadSeries, devTailSeries;
    devHeadSeries.reserve(kLongMeasureSteps);
    devTailSeries.reserve(kLongMeasureSteps);

    WormSim::Snapshot snap;
    double sumAbsHeadingDelta = 0.0;
    float prevHeading = 0.0f;
    bool havePrevHeading = false;
    constexpr float kPi = 3.14159265358979323846f;

    for (int i = 0; i < kLongMeasureSteps; ++i) {
        sim.step();
        const auto& dev = sim.lastCurvatureDeviation();
        devHeadSeries.push_back(kPosHead < static_cast<int>(dev.size()) ? dev[static_cast<std::size_t>(kPosHead)] : 0.0f);
        devTailSeries.push_back(kPosTail < static_cast<int>(dev.size()) ? dev[static_cast<std::size_t>(kPosTail)] : 0.0f);

        sim.snapshot(snap);
        const float heading = std::atan2(snap.pointsY[1] - snap.pointsY[0], snap.pointsX[1] - snap.pointsX[0]);
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            sumAbsHeadingDelta += std::fabs(hd);
        }
        prevHeading = heading;
        havePrevHeading = true;
    }

    double sumSq = 0.0;
    for (float v : devHeadSeries) sumSq += static_cast<double>(v) * v;
    r.headDevRMS = static_cast<float>(std::sqrt(sumSq / devHeadSeries.size()));
    const float measureSeconds = static_cast<float>(kLongMeasureSteps) * dt;
    r.turnRateRadPerSec = static_cast<float>(sumAbsHeadingDelta) / measureSeconds;

    const auto headCrossings = findCrossings(devHeadSeries, dt);
    const auto tailCrossings = findCrossings(devTailSeries, dt);

    if (verbose) {
        std::printf("  headCrossings (t,dir): ");
        for (const auto& c : headCrossings) std::printf("(%.1f,%+d) ", c.timeSeconds, c.direction);
        std::printf("\n  tailCrossings (t,dir): ");
        for (const auto& c : tailCrossings) std::printf("(%.1f,%+d) ", c.timeSeconds, c.direction);
        std::printf("\n");
    }

    for (const auto& hc : headCrossings) {
        float bestLag = 0.0f;
        bool haveBest = false;
        float bestAbsLag = 1e9f;
        for (const auto& tc : tailCrossings) {
            if (tc.direction != hc.direction) continue;
            const float lag = tc.timeSeconds - hc.timeSeconds;
            if (std::fabs(lag) > kMaxLagSeconds) continue;
            if (std::fabs(lag) < bestAbsLag) { bestAbsLag = std::fabs(lag); bestLag = lag; haveBest = true; }
        }
        if (!haveBest) continue;
        if (bestLag > 0.0f) ++r.forwardCount;
        else if (bestLag < 0.0f) ++r.backwardCount;
        r.sumLag += bestLag;
    }
    return r;
}

void printResult(const char* label, const DiagResult& r) {
    const int total = r.forwardCount + r.backwardCount;
    const float meanLag = total > 0 ? static_cast<float>(r.sumLag / total) : 0.0f;
    std::printf("  %-8s headDevRMS=%.5f turnRate=%.5f rad/s  matchedCycles=%d (forward=%d backward=%d, "
                "backwardFrac=%.2f) meanLag=%+.2fs\n",
                label, r.headDevRMS, r.turnRateRadPerSec, total, r.forwardCount, r.backwardCount,
                total > 0 ? static_cast<float>(r.backwardCount) / total : 0.0f, meanLag);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe scan <seed> - crossing count + RMS at EVERY body position, to find which
    // positions actually oscillate (cross zero) before picking posHead/posTail
    if (argc >= 2 && std::string(argv[1]) == "scan") {
        const int seed = argc > 2 ? std::atoi(argv[2]) : 42;
        WormSim sim("worm_data/celegans_herm.connectome");
        std::srand(static_cast<unsigned>(seed));
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        for (int i = 0; i < kWarmupSteps; ++i) sim.step();
        const auto& firstDev = sim.lastCurvatureDeviation();
        const int numPos = static_cast<int>(firstDev.size());
        std::vector<std::vector<float>> series(static_cast<std::size_t>(numPos));
        for (int i = 0; i < kLongMeasureSteps; ++i) {
            sim.step();
            const auto& dev = sim.lastCurvatureDeviation();
            for (int p = 0; p < numPos; ++p) series[static_cast<std::size_t>(p)].push_back(dev[static_cast<std::size_t>(p)]);
        }
        std::printf("numPositions=%d\n", numPos);
        for (int p = 0; p < numPos; ++p) {
            const auto crossings = findCrossings(series[static_cast<std::size_t>(p)], 0.05f);
            double sumSq = 0.0;
            for (float v : series[static_cast<std::size_t>(p)]) sumSq += static_cast<double>(v) * v;
            const float rms = static_cast<float>(std::sqrt(sumSq / series[static_cast<std::size_t>(p)].size()));
            std::printf("pos=%2d  crossings=%2zu  rms=%.5f\n", p, crossings.size(), rms);
        }
        return 0;
    }

    // ./exe verbose <seed> - dump raw crossing lists for one seed, for sanity-checking the matching logic
    if (argc >= 2 && std::string(argv[1]) == "verbose") {
        const int seed = argc > 2 ? std::atoi(argv[2]) : 42;
        std::printf("=== identity, seed=%d ===\n", seed);
        DiagResult identity = runDiagnostic(0.0f, 20.0f, seed, true);
        printResult("identity", identity);
        std::printf("=== pdf1 gain=-0.10, seed=%d ===\n", seed);
        DiagResult pdf1 = runDiagnostic(-0.10f, 20.0f, seed, true);
        printResult("pdf1", pdf1);
        return 0;
    }

    // ./exe [numSeeds] [seedBase] - aggregate comparison
    const int numSeeds = argc > 1 ? std::atoi(argv[1]) : 6;
    const int seedBase = argc > 2 ? std::atoi(argv[2]) : 42;
    std::printf("identity vs pdf1(gain=-0.10, tau=20.0) - %d seeds from base=%d, %d-step (%0.f s) measurement "
                "window\n",
                numSeeds, seedBase, kLongMeasureSteps, static_cast<float>(kLongMeasureSteps) * 0.05f);

    double sumHeadRMS_id = 0.0, sumHeadRMS_pd = 0.0, sumTurn_id = 0.0, sumTurn_pd = 0.0;
    int sumFwd_id = 0, sumBwd_id = 0, sumFwd_pd = 0, sumBwd_pd = 0;
    for (int s = 0; s < numSeeds; ++s) {
        const int seed = seedBase + s;
        const DiagResult identity = runDiagnostic(0.0f, 20.0f, seed, false);
        const DiagResult pdf1 = runDiagnostic(-0.10f, 20.0f, seed, false);
        std::printf("seed=%d:\n", seed);
        printResult("identity", identity);
        printResult("pdf1", pdf1);
        sumHeadRMS_id += identity.headDevRMS; sumHeadRMS_pd += pdf1.headDevRMS;
        sumTurn_id += identity.turnRateRadPerSec; sumTurn_pd += pdf1.turnRateRadPerSec;
        sumFwd_id += identity.forwardCount; sumBwd_id += identity.backwardCount;
        sumFwd_pd += pdf1.forwardCount; sumBwd_pd += pdf1.backwardCount;
        std::fflush(stdout);
    }
    const int totalId = sumFwd_id + sumBwd_id, totalPd = sumFwd_pd + sumBwd_pd;
    std::printf("\n=== Aggregate over %d seeds ===\n", numSeeds);
    std::printf("headDevRMS:   identity=%.5f  pdf1=%.5f  (ratio pdf1/identity=%.3f)\n", sumHeadRMS_id / numSeeds,
                sumHeadRMS_pd / numSeeds, (sumHeadRMS_id > 1e-9) ? (sumHeadRMS_pd / sumHeadRMS_id) : 0.0);
    std::printf("turnRate:     identity=%.5f  pdf1=%.5f  (ratio pdf1/identity=%.3f)\n", sumTurn_id / numSeeds,
                sumTurn_pd / numSeeds, (sumTurn_id > 1e-9) ? (sumTurn_pd / sumTurn_id) : 0.0);
    std::printf("backwardFrac: identity=%d/%d (%.3f)  pdf1=%d/%d (%.3f)\n", sumBwd_id, totalId,
                totalId > 0 ? static_cast<float>(sumBwd_id) / totalId : 0.0f, sumBwd_pd, totalPd,
                totalPd > 0 ? static_cast<float>(sumBwd_pd) / totalPd : 0.0f);
    return 0;
}
