// tests/worm_pdf1_calibration/main.cpp
//
// Calibration search for Params::pdf1Gain/pdf1ReleaseTau (WormSim.h, network.
// hpp/.cpp) - the PDF-1/PDFR-1 neuropeptide connectivity added this session,
// the "other half" of the roaming/dwelling circuit that tests/worm_roaming_
// dwelling_calibration's own header flagged as missing (serotonin/ADF/NSM
// only). Flavell et al. 2013 (Cell): serotonin promotes DWELLING (slower,
// more turning, near food); the neuropeptide PDF-1 promotes the opposite
// ROAMING state (faster, straighter, fewer reorientations).
//
// THIS AXIS IS STRUCTURALLY DIFFERENT from every prior addition this
// session: the connectivity is REAL DATA, not a name-matched heuristic.
// Ripoll-Sánchez, Watteyne et al. 2023 (Neuron 111:3570-3589, "The
// neuropeptidergic connectome of C. elegans") predicts GPCR-ligand
// co-expression connectivity from CeNGEN single-cell expression data: the
// PDF-1/PDFR-1 mid-range network (of 92 published NPP-GPCR pairs) gives 15
// real source neurons (AIY most prominently) and 156 real target neurons,
// downloaded and converted to this project's own node indices (see
// PEPTIDE_EDGES in demo/worm/data/celegans_herm.connectome). The target set
// includes exactly the reversal/turning circuit this project's own
// NEURONS.md already documents by name: AVA/AIB/AIZ/RIM. No new per-neuron
// name-matching code was needed in WormSim.cpp at all - unlike bClassOscillator
// (DB/VB identified by name in the constructor), the connectivity here loads
// straight from data, exactly like chemical_/gap_ do.
//
// Mechanism (network.hpp/cpp): each of the 15 source neurons has a slow
// "releasable pool" r_j that relaxes toward its own current sigmoid-
// activation with time constant pdf1ReleaseTau (a deliberate simplification
// of burst-dependent dense-core-vesicle release kinetics, not a measured
// kinetic law). Every target neuron i receives
//   C_i dV_i/dt = ...existing terms... + pdf1Gain * sum_j P[i,j] * r_j
// where P is the real (binary) predicted connectivity above. gain=0 (default)
// makes this term identically zero on every step, same convention as every
// other experimental axis this session (bClassOscillatorGain, mechanoGain,
// etc).
//
// Honest caveat carried from the design proposal: this exact circuit
// (roaming/dwelling) already burned one calibration attempt this session -
// serotonin found a robust effect in the WRONG direction and a real-but-weak,
// sub-50%-healthy effect in the right direction (see worm_roaming_dwelling_
// calibration's header). There is a concrete, project-specific reason to
// expect PDF-1 might land in similarly messy territory rather than cleanly
// confirming the literature's direction - a real, reproducible, but not
// simultaneously healthy/correctly-signed/large-enough effect should be
// treated as a normal, non-embarrassing possible outcome here, not a sign the
// idea was wrong.
//
// Metric: unlike every speed-bug axis (which cares about bodyLengthsPerSec
// alone), the real prediction here is about REORIENTATION FREQUENCY more than
// raw speed (Flavell et al. 2013 characterize roaming as reduced turning, not
// just faster crawling) - serotonin's own calibration history already hinted
// that bodyLengthsPerSec alone might not be the most sensitive readout. So
// this harness tracks a SECOND outcome metric alongside bodyLengthsPerSec:
// turnRateRadPerSec, the mean |heading delta| per second over the whole
// measurement window (same per-step heading computation already used as a
// safety gate in worm_bclass_oscillator_calibration/worm_bclass_body_joint_
// calibration, just accumulated instead of maxed). Prediction: positive
// pdf1Gain should DECREASE turnRateRadPerSec (straighter, roaming-like) and
// likely increase bodyLengthsPerSec - but the sign is NOT assumed (PDFR-1's
// net circuit effect on a Gαs pathway is not simple biophysical "excitation",
// see the design proposal's own honesty section) - both directions are
// screened, same discipline as every other axis this session.
//
// Same paired-same-seed methodology as worm_roaming_dwelling_calibration,
// but paired against the gain=0 IDENTITY baseline (not onFood/offFood - this
// mechanism isn't food-gated the way the discrete pharyngeal-pump serotonin
// impulse was; PDF-1's source neurons run on whatever chemosensory drive the
// network's own real synapses already produce) - removes seed-to-seed noise
// from the effect estimate. Health gate: efficiency>=0.40, coiledRatio>=0.30,
// freqHz>kMinFreqHz (reject frozen-arc), maxAbsHeadingDelta<=kMaxHeadingDeltaRad
// - baked in from the START of the search, this project's repeatedly-learned
// lesson, not bolted on after. Multi-independent-seed-base evaluation from
// the first fitness call, not just at final confirmation - same discipline.
//
// RESULT: NOT a working reproduction of the literature's roaming signature -
// pdf1Gain/pdf1ReleaseTau ship at 0.0 (confirmed inert via tests/worm_
// locomotion). Two findings, both solid:
//   (a) POSITIVE gain collapses to the frozen-static-arc degenerate mode
//       (freqHz=0.0000) almost immediately - even the smallest tested value,
//       +0.02, was already frozen, and it stayed frozen out to +20 (single-
//       base screen, kGains grid above). Same asymmetric fragility already
//       seen for mechanoGain/serotoninGain's positive-gain side, and the same
//       qualitative "binary, no graded middle ground" collapse documented for
//       bClassOscillator/bClassOscillator+bodyPoseDecayRate - a fourth
//       distinct mechanism hitting the same wall on this side.
//   (b) NEGATIVE gain, by contrast, has a genuine HEALTHY, GRADED operating
//       window (roughly -0.02 to -0.20 at tau_release=20.0, single-base fine
//       grid) - not binary at all on this side. Confirmed at gain=-0.10
//       across 16 independent seed bases x 8 seeds (128 paired trials vs the
//       gain=0 identity, same base each pair): 16/16 bases fully healthy for
//       both identity and candidate. But the effect itself does NOT match
//       the predicted roaming signature: turnRate INCREASED (not decreased)
//       in 16/16 bases (population mean turnEffect=-0.00061 rad/s, i.e.
//       identity-pdf1 is negative - pdf1 turns MORE, not less), while
//       bodyLengthsPerSec increased only marginally (population mean
//       speedEffect=+0.00003 BL/s, ~1.4% of baseline, correct direction but
//       tiny, 15/16 bases). A small, very reproducible effect in a mixed
//       direction, not the "faster and straighter" signature Flavell et al.
//       2013 predicts for PDF-1/roaming.
// This is now the fourth roaming/dwelling-or-speed-adjacent axis this
// session (serotonin, bClassOscillator alone, bClassOscillator+
// bodyPoseDecayRate, now this) to find real, reproducible, healthy dynamics
// that nonetheless don't reproduce the target behavioral signature - worth
// treating as a pattern (this reduced connectome's dynamics may sit in a
// regime where new modulatory inputs reliably perturb SOMETHING measurable
// without reliably perturbing it the intended way) rather than four
// unrelated near-misses. Whoever continues this: the healthy negative-gain
// window itself (a rare non-binary result this session) may be worth
// understanding mechanistically before trying yet another gain value - why
// is it graded here when every other self-referential/broadcast axis this
// session was binary?
//
// TAU_RELEASE SWEEP (follow-up, same session): swept pdf1ReleaseTau across
// the ENTIRE literature-defensible range {2, 5, 10, 20, 40, 80, 150}s at the
// known-healthy gain=-0.10 (4 bases x 8 seeds each, tau=20 already at 16
// bases from above). Result: turnEffect stayed NEGATIVE (wrong direction)
// at EVERY tau tested - 2:-0.00073, 5:-0.00076, 10:-0.00075, 20:-0.00061,
// 40:-0.00072, 80:-0.00058, 150:-0.00039 rad/s - 100% healthy throughout (no
// health/tau tradeoff at all). speedEffect stayed positive (correct
// direction) throughout too, small and roughly tau-dependent - peaks around
// tau=40-80 (+0.00005..+0.00006 BL/s) vs tau=2's +0.00001.
// This is NOT a tau-tuning problem and re-sweeping tau further cannot fix
// it: tau_release only smooths/delays the source neurons' own activity
// before it reaches the peptide_ matrix - it cannot change the SIGN of the
// pathway's net effect on its targets, which is set entirely by pdf1Gain's
// sign and the (fixed, real, binary) connectivity itself. The mild trend
// (wrongness shrinking toward tau=150) hints a sign flip might exist further
// out, but chasing that would mean tau values with no remaining connection
// to Flavell et al. 2013's "minutes"-scale justification - tuning a
// free parameter until a metric flips sign, exactly what this project's
// discipline (bounded, literature-grounded search) exists to avoid. If this
// axis is revisited, the more promising levers are pdf1Gain's SIGN/target
// set (not yet re-examined) or - a real, substantive alternative hypothesis
// - whether turnRateRadPerSec (mean |head-segment heading delta|) is even
// measuring "reorientation frequency" in Flavell et al.'s sense, as opposed
// to the amplitude of ordinary undulatory head sweep during otherwise-normal
// forward crawling; those are two different things this metric cannot
// currently distinguish, and negatively gaining the reversal circuit
// (AVA/AIB/AIZ/RIM) could plausibly widen the latter without producing more
// of the former.
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
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMaxHeadingDeltaRad = 0.5f;
// 2000 steps (5x pdf1ReleaseTau=20s, "full EMA convergence") was tried FIRST
// and empirically REJECTED: it made even the gain=0 identity baseline
// unhealthy (efficiency ~0.17-0.19, consistently across independent bases -
// not noise) via some warmup-length-dependent phase/efficiency interaction
// unrelated to pdf1 itself (bodyLengthsPerSec/turnRate were unaffected, only
// the net-displacement/path-length ratio). 500 steps (matching this
// project's other axis tests, e.g. worm_bclass_oscillator_calibration's own
// 300-step default) empirically gives a healthy, stable identity baseline
// (eff~0.57-0.58) reproducibly across independent bases - used here despite
// not reaching strict 5x-tau convergence, the same practical tradeoff every
// other axis in this project already makes.
constexpr int kDefaultWarmupSteps = 500;    // 25s @ dt=0.05
constexpr int kDefaultMeasureSteps = 2500;  // 125s

using Candidate = std::array<float, 2>;  // {gain, tau_release}
enum { kGain = 0, kTauRelease = 1 };
const Candidate kIdentity = {0.0f, 20.0f};  // gain=0 -> tau_release value is irrelevant (see header)

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.pdf1Gain = c[kGain];
    sim.params.pdf1ReleaseTau = c[kTauRelease];
}

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float turnRateRadPerSec = 0.0f;
    float efficiency = 0.0f;
    float minCoiledRatio = 1e9f;
    float freqHz = 0.0f;
    float maxAbsHeadingDelta = 0.0f;
    bool healthy = true;
};

Measurement runTrial(const Candidate& cand, int seed, int warmupSteps, int measureSteps, int freqPosition = 12) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    std::srand(static_cast<unsigned>(seed));  // after ctor - ctor's own srand(time()) would clobber this
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
    double sumAbsHeadingDelta = 0.0;

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
            sumAbsHeadingDelta += std::fabs(hd);
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
    m.turnRateRadPerSec = static_cast<float>(sumAbsHeadingDelta) / measureSeconds;
    return m;
}

struct AggregateResult {
    float meanBLps = 0.0f, stderrBLps = 0.0f;
    float meanTurnRate = 0.0f, stderrTurnRate = 0.0f;
    float meanFreqHz = 0.0f, meanEfficiency = 0.0f, minCoiledRatio = 1e9f;
    float maxHeadingDelta = 0.0f;
    bool allHealthy = true;
    int healthyCount = 0, total = 0;
};

AggregateResult evaluate(const Candidate& cand, int numSeeds, int seedBase, int warmupSteps, int measureSteps) {
    AggregateResult ar;
    std::vector<float> blSamples, turnSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int s = 0; s < numSeeds; ++s) {
        const Measurement m = runTrial(cand, seedBase + s, warmupSteps, measureSteps);
        ar.total++;
        ar.minCoiledRatio = std::min(ar.minCoiledRatio, m.minCoiledRatio);
        ar.maxHeadingDelta = std::max(ar.maxHeadingDelta, m.maxAbsHeadingDelta);
        if (!m.healthy) { ar.allHealthy = false; continue; }
        ar.healthyCount++;
        blSamples.push_back(m.bodyLengthsPerSec);
        turnSamples.push_back(m.turnRateRadPerSec);
        sumFreq += m.freqHz;
        sumEff += m.efficiency;
    }
    auto meanStderr = [](const std::vector<float>& v, float& mean, float& stderrOut) {
        if (v.empty()) { mean = stderrOut = 0.0f; return; }
        double sum = 0.0;
        for (float x : v) sum += x;
        mean = static_cast<float>(sum / v.size());
        double sq = 0.0;
        for (float x : v) sq += (x - mean) * (x - mean);
        const float stddev = v.size() > 1 ? std::sqrt(static_cast<float>(sq / (v.size() - 1))) : 0.0f;
        stderrOut = stddev / std::sqrt(static_cast<float>(v.size()));
    };
    meanStderr(blSamples, ar.meanBLps, ar.stderrBLps);
    meanStderr(turnSamples, ar.meanTurnRate, ar.stderrTurnRate);
    if (!blSamples.empty()) {
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
    std::printf("  %-8s speed=%.5f+/-%.5f BL/s turnRate=%.5f+/-%.5f rad/s freq=%.4fHz eff=%.3f coiled=%.3f "
                "maxHeadDelta=%.4f healthy=%s(%d/%d)\n",
                label, ar.meanBLps, ar.stderrBLps, ar.meanTurnRate, ar.stderrTurnRate, ar.meanFreqHz,
                ar.meanEfficiency, ar.minCoiledRatio, ar.maxHeadingDelta, isHealthy(ar) ? "yes" : "NO",
                ar.healthyCount, ar.total);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution gain tau_release [numBases] [seedsPerBase] [warmup] [measure]
    // Paired against the gain=0 identity baseline on the SAME seed base.
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kTauRelease] = static_cast<float>(std::atof(argv[3]));
        const int numBases = argc > 4 ? std::atoi(argv[4]) : 12;
        const int seedsPerBase = argc > 5 ? std::atoi(argv[5]) : 8;
        const int warmupSteps = argc > 6 ? std::atoi(argv[6]) : kDefaultWarmupSteps;
        const int measureSteps = argc > 7 ? std::atoi(argv[7]) : kDefaultMeasureSteps;
        std::printf("cand=[gain=%.4f tau_release=%.4f] vs identity - %d bases x %d seeds\n", cand[kGain],
                    cand[kTauRelease], numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int baseHealthyCount = 0, candHealthyCount = 0, turnReducedCount = 0, speedIncreasedCount = 0;
        double sumTurnEffect = 0.0, sumSpeedEffect = 0.0;
        int pairedCount = 0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult identity = evaluate(kIdentity, seedsPerBase, base, warmupSteps, measureSteps);
            const AggregateResult candidate = evaluate(cand, seedsPerBase, base, warmupSteps, measureSteps);
            const bool baseOk = isHealthy(identity), candOk = isHealthy(candidate);
            std::printf("base=%d:\n", base);
            printAgg("identity", identity);
            printAgg("pdf1", candidate);
            if (baseOk && candOk) {
                const float turnEffect = identity.meanTurnRate - candidate.meanTurnRate;  // + = pdf1 reduces turning
                const float speedEffect = candidate.meanBLps - identity.meanBLps;          // + = pdf1 speeds up
                sumTurnEffect += turnEffect;
                sumSpeedEffect += speedEffect;
                ++pairedCount;
                if (turnEffect > 0.0f) ++turnReducedCount;
                if (speedEffect > 0.0f) ++speedIncreasedCount;
                std::printf("  turnEffect(identity-pdf1)=%+.5f rad/s  speedEffect(pdf1-identity)=%+.5f BL/s\n",
                            turnEffect, speedEffect);
            } else {
                std::printf("  (skipped - not both healthy)\n");
            }
            std::fflush(stdout);
            if (baseOk) ++baseHealthyCount;
            if (candOk) ++candHealthyCount;
        }
        std::printf("\nSummary over %d bases: identity healthy=%d/%d, pdf1 healthy=%d/%d, both-healthy pairs=%d/%d\n",
                    numBases, baseHealthyCount, numBases, candHealthyCount, numBases, pairedCount, numBases);
        if (pairedCount > 0) {
            std::printf("population mean turnEffect=%.5f rad/s (reduced turning in %d/%d pairs), mean "
                        "speedEffect=%.5f BL/s (increased speed in %d/%d pairs)\n",
                        sumTurnEffect / pairedCount, turnReducedCount, pairedCount, sumSpeedEffect / pairedCount,
                        speedIncreasedCount, pairedCount);
        }
        return 0;
    }

    // Default: screen a signed grid on one seed base (shortlist only - confirm
    // anything promising with 'distribution <gain> <tau>' across many bases
    // before trusting it, per the header and this project's own repeated
    // lesson on that). tau_release held at the literature-anchored default.
    const float kGains[] = {-0.30f, -0.20f, -0.15f, -0.12f, -0.10f, -0.08f, -0.05f, -0.02f,
                             0.02f,  0.05f,  0.08f,  0.10f,  0.12f,  0.15f,  0.20f,  0.30f};
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    std::printf("=== Screen (fixed signed grid, tau_release=20.0, %d seeds/point, base=%d) ===\n", kScreenSeeds,
                kSeedBase);
    const AggregateResult identity = evaluate(kIdentity, kScreenSeeds, kSeedBase, kDefaultWarmupSteps,
                                               kDefaultMeasureSteps);
    printAgg("identity", identity);
    for (float g : kGains) {
        const Candidate cand = {g, 20.0f};
        const AggregateResult ar = evaluate(cand, kScreenSeeds, kSeedBase, kDefaultWarmupSteps, kDefaultMeasureSteps);
        char label[16];
        std::snprintf(label, sizeof(label), "g=%+.2f", g);
        printAgg(label, ar);
        std::fflush(stdout);
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising gain with "
                "'distribution <gain> <tau_release>' across many seed bases before believing it (see header).\n");
    return 0;
}
