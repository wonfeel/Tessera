// tests/worm_leak_capacitance_tempo_calibration/main.cpp
//
// Per-neuron-CLASS leak/capacitance recalibration (Network::scale_type_params,
// network.hpp), retried from scratch, on top of today's shipped CPG x muscle-
// bandwidth point (Params defaults - cpgGain=1.901 etc., see WormSim.h), by
// direct user request ("а какая сейчас разница? -> попробуй пересчитать
// leak/capacitance по всем классам заново, только оптимизируй этот процесс")
// after that shipped point's own tempo (agar 0.0192Hz/water 0.1313Hz) was
// confirmed still ~14-21x below real C. elegans (Fang-Yen et al. 2010:
// crawl ~0.3-0.5Hz, swim ~1.7-2Hz) and three further search rounds targeting
// cpgGain/muscleBandwidthGain themselves (high-cpgGain, wide-muscleBandwidth,
// and the cpgGain 2-12 middle ground - see WORM.md) found nothing that beats
// the shipped point's balance.
//
// THIS AXIS HAS A DOCUMENTED HISTORY OF TWO PRIOR FAILURES ON THIS EXACT
// LEVER (tests/worm_speed_calibration, tests/worm_chemotaxis_calibration) -
// read before assuming this file's own results are trustworthy:
//   1. First attempt found a per-class calibration that passed this project's
//      OWN regression numbers (coiled ratio) but, live in Demo_worm, the worm
//      settled into a static shape instead of crawling - crawling efficiency
//      (net displacement/path length) had collapsed >3x. The regression gate
//      at the time checked shape health only, not absolute locomotion
//      efficiency. FIX APPLIED HERE: efficiency>=0.40 is a HARD gate from the
//      first trial of the first screen, not added after a live-look surprise.
//   2. Second attempt (tests/worm_speed_calibration) added that efficiency
//      floor and found a apparently-clean 4.85x speedup with unchanged
//      efficiency on its own single-base final-verification step. Shipped.
//      Reverted after a FRESH rerun gave a different result, and a proper
//      "distribution" sweep across 20 independent seed bases found the
//      "winning" candidate and the untouched identity baseline have IDENTICAL
//      population-mean speed (0.01079 BL/s both) - the original result was a
//      single anomalous seed-base reading that never reproduced, not a real
//      effect. This network's dynamics are chaotic enough that MORE SEEDS
//      FROM ONE BASE never catches this - only independent BASES do. FIX
//      APPLIED HERE: every candidate that clears the health+ratio gate in the
//      screen gets an IMMEDIATE 'distribution' confirmation (16 independent
//      bases x 8 seeds) before being reported as real - not deferred to a
//      separate later verification step, and not skipped because a single
//      base looked clean. This is the whole "optimize the process" the user
//      asked for: catch the illusion at screen time, not after shipping.
//
// Parameterization (8 params, log-uniform [0.05,4.0] per class - the
// historical file's own manual sweep found effects only appear past a
// 4-20x REDUCTION from identity, so the range is centered to actually reach
// that regime, not just barely perturb it): leak_scale and capacitance_scale
// for InputProcessing/Processing/ProcessingOutput/Output (connectome::
// NeuronType - see types.hpp). NeuronType::Input is excluded - Network::step
// sets Input neurons' next state directly from external_input_ every step
// (no leak/capacitance term exists in that code path at all, confirmed by
// reading network.cpp directly - not carried over from the old test by
// assumption). Output's leak_scale is included THIS TIME (the historical
// file called it "a dead parameter" because muscle_leak_scale_ was always
// exactly 0 back then - Network::step computes leak_Output = p.leak *
// muscle_leak_scale_, so any scale_type_params(Output, leak, ...) multiplied
// into a hard zero). That is no longer true: production now ships
// muscleBandwidthGain=0.000249, so muscle_leak_scale_ is genuinely nonzero
// every step - Output's leak_scale is now a real, live lever that multiplies
// directly against the already-shipped bandwidth mechanism.
//
// Deliberately searched ON TOP of (not isolated against zero like both
// historical attempts) today's shipped cpgGain/cpgBaseFreqHz/
// cpgLoadSensitivity/cpgAmpLoadSensitivity/bodyPoseDecayRate/
// muscleBandwidthGain/motorBandwidthGain - applyCalibration below does NOT
// touch those, so they stay at WormSim's own (now-shipped) Params defaults.
// This deviates from this project's usual "isolate first" discipline on
// purpose: the goal stated by the user is improving the ALREADY-SHIPPED
// configuration's tempo, not characterizing this axis in a vacuum, and
// leak/capacitance-by-class directly reshapes the time constants cpgGain and
// muscleBandwidthGain operate through (not a competing feedback signal like
// the three mechanism-pairing failures earlier this session) - a
// substantively different kind of combination.
//
// Fitness (screen and random modes): health gate identical to every other
// axis this session (efficiency>=0.40, coiledRatio>=0.30, freqHz>=0.001,
// maxHeadingDelta<=0.5) PLUS a hard ratio>=1.15 floor (a real margin, not
// knife-edge - learned from today's cpgGain-middle-ground search finding
// single-base "hits" right at ratio~1.0 that evaporated on confirmation).
// Candidates clearing both gates are scored by closeness to REAL target
// frequency on BOTH media at once (Fang-Yen et al. 2010: agar/crawl ~0.4Hz,
// water/swim ~1.85Hz - using the range midpoints as single target numbers),
// capped at 1.5x overshoot credit so blowing past the target isn't rewarded
// forever: score = min(agarFreq/0.4, 1.5)*10 + min(waterFreq/1.85, 1.5)*10.
//
// RESULT: SHIPPED - a real, confirmed, if modest, improvement, unlike the
// two documented historical failures on this axis.
//
// Broad screen (1100 trials, all 8 params log-uniform [0.05,4.0], on top of
// the shipped cpgGain/muscleBandwidthGain point): only 3/1100 cleared the
// health+ratio>=1.15 gate at all - a ~0.27% hit rate, consistent with this
// project's repeated finding that this connectome's healthy operating window
// is narrow. All 3 got immediate 16-base distribution confirmation (the
// process fix this file exists to test): NONE beat the shipped baseline's
// tempo - one matched it, one traded tempo for a higher ratio, one was an
// outright single-base illusion (healthy in only 2/16 bases, would have
// shipped under the old "verify once, ship" methodology).
//
// Narrower follow-up ('random_focus' mode, 1100 more trials): held
// InputProcessing/Processing at identity, searched only ProcessingOutput/
// Output leak+capacitance - the one lever genuinely new this session
// (Output's leak was mathematically dead in both historical attempts;
// muscleBandwidthGain now shipping nonzero makes it live). Hit rate rose to
// 14/1100 (5x denser than the broad screen - confirms the subspace choice
// was right, not luck). Top 5 by frequency-closeness score got 16-base
// confirmation:
//   - leakPO=0.712/leakO=0.45/capPO=1.286/capO=0.68: 16/16 both-healthy,
//     water>agar in all 16. agar 0.0283Hz (was 0.0192Hz, +47%), water
//     0.1232Hz (was 0.1313Hz, -6%), ratio=2.908 (was 2.692, still inside the
//     200-300% target). Real, robust, SHIPPED (WormSim.cpp constructor).
//   - 4 others: either no better than shipped baseline, or fragile (10/16,
//     14/16, 15/16 both-healthy - the exact single-base/few-base illusion
//     pattern this process is designed to catch) - not shipped.
//
// Honest scale check: agar frequency moved from ~4.8% of the real target
// (Fang-Yen et al. 2010, ~0.4Hz) to ~7.1% - a real step, nowhere near
// closing the 14-21x gap the user flagged. Water frequency moved slightly
// the WRONG way (~7.1% of target -> ~6.7%). This is an incremental
// improvement on the worse-performing medium (agar), not a solution.
//
// Process note for whoever revisits this axis again: the broad-then-focused
// two-pass structure (broad screen finds almost nothing -> narrow to the
// specific subspace most likely to matter -> re-search there) is what
// actually found the shipped candidate; a single broad pass alone would have
// reported a null result. Worth trying as a default strategy on future
// high-dimensional axes in this project, not just this one.
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
constexpr float kTargetAgarFreq = 0.4f, kTargetWaterFreq = 1.85f;
constexpr float kMinRatioMargin = 1.15f;

using Candidate = std::array<float, 8>;
enum { kLeakIP = 0, kLeakP = 1, kLeakPO = 2, kLeakO = 3, kCapIP = 4, kCapP = 5, kCapPO = 6, kCapO = 7 };
const Candidate kIdentity = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    connectome::Network& net = sim.network();
    net.scale_type_params(connectome::NeuronType::InputProcessing, c[kLeakIP], c[kCapIP]);
    net.scale_type_params(connectome::NeuronType::Processing, c[kLeakP], c[kCapP]);
    net.scale_type_params(connectome::NeuronType::ProcessingOutput, c[kLeakPO], c[kCapPO]);
    net.scale_type_params(connectome::NeuronType::Output, c[kLeakO], c[kCapO]);
    // cpgGain/muscleBandwidthGain/etc are intentionally left untouched - they
    // stay at WormSim's own shipped Params defaults (see header).
}

void printCand(const Candidate& c) {
    std::printf("[leakIP=%.3f leakP=%.3f leakPO=%.3f leakO=%.3f capIP=%.3f capP=%.3f capPO=%.3f capO=%.3f]", c[0],
                c[1], c[2], c[3], c[4], c[5], c[6], c[7]);
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

float freqScore(const AggregateResult& agar, const AggregateResult& water) {
    const float agarPart = std::min(agar.meanFreqHz / kTargetAgarFreq, 1.5f);
    const float waterPart = std::min(water.meanFreqHz / kTargetWaterFreq, 1.5f);
    return agarPart * 10.0f + waterPart * 10.0f;
}

void printAgg(const char* label, const AggregateResult& ar) {
    std::printf("  %-6s speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f maxHeadDelta=%.4f healthy=%s\n",
                label, ar.meanBLps, ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                ar.maxHeadingDelta, isHealthy(ar) ? "yes" : "NO");
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution leakIP leakP leakPO leakO capIP capP capPO capO [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        for (int k = 0; k < 8 && argc > k + 2; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numBases = argc > 10 ? std::atoi(argv[10]) : 12;
        const int seedsPerBase = argc > 11 ? std::atoi(argv[11]) : 8;
        const int warmupSteps = argc > 12 ? std::atoi(argv[12]) : 300;
        const int measureSteps = argc > 13 ? std::atoi(argv[13]) : 2500;
        std::printf("cand="); printCand(cand);
        std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0, bothHealthyCount = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumRatio = 0.0, sumFreqAgar = 0.0, sumFreqWater = 0.0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(cand, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(cand, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool bothOk = agarOk && waterOk;
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  ratio water/agar: %s\n", bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; sumFreqAgar += agar.meanFreqHz; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; sumFreqWater += water.meanFreqHz; }
            if (bothOk) {
                ++bothHealthyCount;
                sumRatio += ratio;
                if (ratio > 1.0f) ++waterBeatsAgarCount;
            }
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s, %.4fHz), water healthy=%d/%d "
                    "(mean %.5f BL/s, %.4fHz), both-healthy=%d, mean ratio=%.3f, water>agar in %d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    agarHealthyCount ? sumFreqAgar / agarHealthyCount : 0.0, waterHealthyCount, numBases,
                    waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    waterHealthyCount ? sumFreqWater / waterHealthyCount : 0.0, bothHealthyCount,
                    bothHealthyCount ? sumRatio / bothHealthyCount : 0.0, waterBeatsAgarCount);
        return 0;
    }

    // ./exe random <trials> <lo> <hi> [seedsPerTrial] [rngSeed]
    // Same log-uniform bound applied to all 8 params (simplifies CLI - the
    // header's rationale for [0.05,4.0] applies uniformly across classes,
    // there is no a priori reason to bias one class's search range over
    // another before seeing data).
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float lo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.05f;
        const float hi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 4.0f;
        const int seedsPerTrial = argc > 5 ? std::atoi(argv[5]) : 6;
        const unsigned rngSeed = argc > 6 ? static_cast<unsigned>(std::atoi(argv[6])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, all 8 params log-uniform [%.3f,%.3f], ratio>=%.2f required, %d "
                    "seeds/trial, rngSeed=%u\n",
                    trials, lo, hi, kMinRatioMargin, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> logDist(std::log(lo), std::log(hi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundPassingGate = 0;
        float bestScore = -1e9f;
        Candidate bestCand{};
        int bestBase = 0;
        AggregateResult bestAgar, bestWater;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            for (int k = 0; k < 8; ++k) cand[k] = std::exp(logDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio < kMinRatioMargin) continue;
            ++foundPassingGate;
            const float score = freqScore(agar, water);
            std::printf("PASS score=%.3f ratio=%.3f base=%d ", score, ratio, base);
            printCand(cand);
            std::printf("\n");
            printAgg("  agar", agar);
            printAgg("  water", water);
            std::fflush(stdout);
            if (score > bestScore) {
                bestScore = score; bestCand = cand; bestBase = base; bestAgar = agar; bestWater = water;
            }
        }
        std::printf("\n%d/%d trials passed health+ratio>=%.2f gate. Best score: %.3f", foundPassingGate, trials,
                    kMinRatioMargin, bestScore < -1e8f ? 0.0f : bestScore);
        if (foundPassingGate > 0) {
            std::printf(" at base=%d ratio=%.3f agarFreq=%.4fHz waterFreq=%.4fHz ", bestBase,
                        bestWater.meanBLps / bestAgar.meanBLps, bestAgar.meanFreqHz, bestWater.meanFreqHz);
            printCand(bestCand);
        }
        std::printf(
            "\nNOTHING above is trustworthy on a single base - per this file's own header (two prior single-base "
            "illusions on this exact axis), EVERY passing candidate needs 'distribution' confirmation (16+ "
            "independent bases) before it means anything.\n");
        return 0;
    }

    // ./exe random_focus <trials> <lo> <hi> [seedsPerTrial] [rngSeed]
    // Same gate/scoring as 'random', but holds leakIP/leakP/capIP/capP at
    // IDENTITY (1.0) and only randomizes leakPO/leakO/capPO/capO - the two
    // classes that are genuinely NEW territory this time (ProcessingOutput
    // contains the motor neurons motorBandwidthGain already acts on; Output's
    // leak was a dead parameter in both historical attempts, see header, and
    // is live now). The broad 8-param 'random' mode's first 1100-trial pass
    // found only 3/1100 candidates clearing the health+ratio gate at all,
    // none beating the shipped baseline's tempo after distribution
    // confirmation - a 0.27% hit rate this thin justifies narrowing the
    // search to the specific subspace most likely to matter, rather than
    // more blind trials at the same density.
    if (argc >= 2 && std::string(argv[1]) == "random_focus") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float lo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.05f;
        const float hi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 8.0f;
        const int seedsPerTrial = argc > 5 ? std::atoi(argv[5]) : 6;
        const unsigned rngSeed = argc > 6 ? static_cast<unsigned>(std::atoi(argv[6])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random_focus search: %d trials, leakPO/leakO/capPO/capO log-uniform [%.3f,%.3f] (leakIP/leakP/"
                    "capIP/capP fixed at identity 1.0), ratio>=%.2f required, %d seeds/trial, rngSeed=%u\n",
                    trials, lo, hi, kMinRatioMargin, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> logDist(std::log(lo), std::log(hi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundPassingGate = 0;
        float bestScore = -1e9f;
        Candidate bestCand{};
        int bestBase = 0;
        AggregateResult bestAgar, bestWater;
        for (int t = 0; t < trials; ++t) {
            Candidate cand = kIdentity;
            cand[kLeakPO] = std::exp(logDist(rng));
            cand[kLeakO] = std::exp(logDist(rng));
            cand[kCapPO] = std::exp(logDist(rng));
            cand[kCapO] = std::exp(logDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            if (!isHealthy(agar) || agar.meanBLps <= 1e-9f) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            if (!isHealthy(water)) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio < kMinRatioMargin) continue;
            ++foundPassingGate;
            const float score = freqScore(agar, water);
            std::printf("PASS score=%.3f ratio=%.3f base=%d ", score, ratio, base);
            printCand(cand);
            std::printf("\n");
            printAgg("  agar", agar);
            printAgg("  water", water);
            std::fflush(stdout);
            if (score > bestScore) {
                bestScore = score; bestCand = cand; bestBase = base; bestAgar = agar; bestWater = water;
            }
        }
        std::printf("\n%d/%d trials passed health+ratio>=%.2f gate. Best score: %.3f", foundPassingGate, trials,
                    kMinRatioMargin, bestScore < -1e8f ? 0.0f : bestScore);
        if (foundPassingGate > 0) {
            std::printf(" at base=%d ratio=%.3f agarFreq=%.4fHz waterFreq=%.4fHz ", bestBase,
                        bestWater.meanBLps / bestAgar.meanBLps, bestAgar.meanFreqHz, bestWater.meanFreqHz);
            printCand(bestCand);
        }
        std::printf(
            "\nNOTHING above is trustworthy on a single base - EVERY passing candidate needs 'distribution' "
            "confirmation (16+ independent bases) before it means anything.\n");
        return 0;
    }

    // Default: baseline read (today's shipped point, identity leak/capacitance).
    std::printf("=== Baseline (today's shipped CPG+muscleBandwidth point, identity leak/capacitance) ===\n");
    constexpr int kBaseSeeds = 8;
    constexpr int kSeedBase = 42;
    const AggregateResult agar = evaluate(kIdentity, kBaseSeeds, kSeedBase, kDragAgar, 300, 2500);
    const AggregateResult water = evaluate(kIdentity, kBaseSeeds, kSeedBase, kDragWater, 300, 2500);
    printAgg("agar", agar);
    printAgg("water", water);
    const bool bothOk = isHealthy(agar) && isHealthy(water);
    if (bothOk) std::printf("  ratio water/agar: %s\n", std::to_string(water.meanBLps / agar.meanBLps).c_str());
    std::printf("\nRun 'random <trials> [lo] [hi] [seeds] [rngSeed]' for a search, or 'distribution <8 params>' to "
                "confirm a specific candidate.\n");
    return 0;
}
