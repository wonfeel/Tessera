// tests/worm_v2_mechanosensation_calibration/main.cpp
//
// Calibration search for Params::mechanoGain / Params::localMechanoGain
// JOINTLY with Params::dragSettleGain, on top of the "worm-v2-architecture"
// rewrite (see demo/worm/WORM_V2_DESIGN.md, WORM_V2_RESULTS.md). Modeled
// directly on tests/worm_v2_measurement/main.cpp's methodology (same medium
// setup, same centroid-based efficiency/frequency/coiled-ratio measurement,
// same health-gate constants, same independent seed BASES not just more
// seeds from one) - reused deliberately, per this project's established
// convention.
//
// WHY THIS AXIS, WHY NOW (see WORM_V2_RESULTS.md section 5): the v2 rewrite
// removed the external CPG oscillator, which was the ONLY mechanism that
// ever made bend frequency depend on medium (cpgLoadSensitivity explicitly
// divided frequency by (1+gain*normalizedLoad)). Post-removal,
// WormBody::step()'s curvature-deviation update depends ONLY on network
// curvature output - never on dragTangent/dragNormal - so agar and water now
// measure BITWISE-IDENTICAL frequency (0.0984Hz both, freq ratio == 1.000
// exactly) no matter what. mechanoGain/localMechanoGain (both 0.0, never
// calibrated in EITHER v1 or v2) are the only remaining honest, already-
// existing channel that could restore real medium-dependent frequency:
// mechanoGain feeds DVA (Li, Feng & Xu 2006, PMC1500850, real stretch-
// receptor neuron) with net.set_input(m_dva, gain *
// (mechanical_load()/dragNormal) + noise) - a drag-normalized mechanical
// load signal that legitimately differs between agar (dragNormal=40) and
// water (dragNormal=1.7) through mechanical_load()'s absolute scale, then
// feeds back into the real network via DVA's actual synaptic connectivity.
// localMechanoGain adds a per-position local load term directly into the
// proprioceptive feedback sum (applyProprioception).
//
// HISTORICAL WARNING (see demo/worm/WORM.md section 6, OLD v1 architecture,
// tests/worm_mechanosensation_calibration and
// tests/worm_local_mechanosensation_calibration): this exact axis was
// already tried twice before, independently, and both failed - part of
// "five independent axes that failed the same way" (DVA mechanosensation,
// local mechanosensation, leak/capacitance-by-class, synapse-sign,
// proprioception-only). At low gain, frequency froze at 0.0000Hz (a "hard"
// locked-arc failure mode) regardless of localMechanoGain. ALSO: when
// localMechanoGain was combined with the (now fully removed) drag-adhesion
// mechanisms (dragAdhesionGain), ANY nonzero localMechanoGain together with
// ANY nonzero adhesion gain destructively collapsed crawling efficiency to
// 0.2-0.3 (below the 0.40 health floor), NOT additively - "any nonzero X
// together with any nonzero Y" was the actual finding, not "large X/Y".
// Given that history, and that dragSettleGain (v2's new, not-yet-shipped
// friction-memory mechanism, structurally different from the removed
// adhesion formulas but occupying the same conceptual slot - see
// WORM_V2_DESIGN.md section 4) is a plausible new destructive-interaction
// partner, this harness searches mechanoGain/localMechanoGain/dragSettleGain
// JOINTLY from the very first screen - never sequentially. All other v2
// params (muscleLeakScale, muscleCalciumTau, bodyFrameRateLimitHz, bodyGain,
// bodyPoseDecayRate, proprioceptiveDelaySeconds, dragSettleTau) are fixed at
// the current v2 shipped defaults (see V2Point below) - this harness does
// NOT re-search those, only the three free axes named in the task.
//
// HONEST BASELINE TO BEAT (v2, all three free params at 0 - i.e. today's
// actual shipped default, see WORM_V2_RESULTS.md section 4/7):
//   agar:  speed=0.00538 BL/s, freq=0.0984Hz
//   water: speed=0.00028 BL/s, freq=0.0984Hz
//   freq ratio (water/agar) = 1.000 exactly - the problem this file exists
//   to try to fix.
// Also relevant (found in the v2 session, NOT shipped as default):
// dragSettleGain=16 alone (mechanoGain=localMechanoGain=0) gives agar
// speed=0.00973, water speed=0.01087, speed ratio=1.117, freq ratio still
// exactly 1.000 (friction-memory affects speed, not frequency, by itself).
//
// REAL TARGET (Fang-Yen et al. 2010 / Vidal-Gadea et al. 2011): real
// C. elegans frequency ratio water/agar ~2-4x (some sources cite up to
// 3.4-6.7x for stride frequency specifically). A ratio far from these real
// numbers by a lot in EITHER direction is not "more realistic" even if it's
// "not 1.000" - chase a plausible ratio, not an arbitrarily large one.
//
// Health gate (same as tests/worm_v2_measurement / tests/worm_speed_
// combined_calibration): efficiency>=0.40, coiledRatio>=0.30,
// maxHeadingDelta<=0.5 rad, freqHz>=0.001, measured on BOTH agar and water
// independently - a candidate must pass on BOTH media to count as healthy.
//
// METHODOLOGY: screen cheap (few seed bases) -> confirm expensive (16+
// independent fresh seed bases, DIFFERENT seeds than the screen) -> select
// on the CONFIRMED number, not the screen number (winner's-curse
// protection). Never report a single lucky run as a finding. If nothing
// survives confirmation, that is a valid, complete, reportable result.
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

constexpr float kBodyLength = 576.0f;
constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kHexSpacing = 36.0f;
constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMaxHeadingDeltaRad = 0.5f;

// Honest comparison points - see file header. NOT targets, reference only.
constexpr float kV2BaselineAgarBL = 0.00538f, kV2BaselineWaterBL = 0.00028f, kV2BaselineFreq = 0.0984f;
constexpr float kDragSettle16AgarBL = 0.00973f, kDragSettle16WaterBL = 0.01087f;

// V2 defaults, fixed for this whole search - see WormSim.h for each field's
// full citation/derivation. Only mechanoGain/localMechanoGain/dragSettleGain
// vary in this harness.
struct V2Point {
    float muscleLeakScale = 50.0f;
    float muscleCalciumTau = 0.3f;
    float bodyPoseDecayRate = 0.5f;
    float bodyFrameRateLimitHz = 2.0f;
    float dragSettleTau = 2.0f;
    float dragSettleGain = 0.0f;         // FREE for this search
    float proprioceptiveDelaySeconds = 0.0f;
    float mechanoGain = 0.0f;            // FREE for this search
    float localMechanoGain = 0.0f;       // FREE for this search
    float bodyGain = 200.0f;             // v2 shipped default, not touched here
};

void applyPoint(WormSim& sim, const V2Point& p) {
    sim.params.muscleLeakScale = p.muscleLeakScale;
    sim.params.muscleCalciumTau = p.muscleCalciumTau;
    sim.params.bodyPoseDecayRate = p.bodyPoseDecayRate;
    sim.params.bodyFrameRateLimitHz = p.bodyFrameRateLimitHz;
    sim.params.dragSettleTau = p.dragSettleTau;
    sim.params.dragSettleGain = p.dragSettleGain;
    sim.params.proprioceptiveDelaySeconds = p.proprioceptiveDelaySeconds;
    sim.params.mechanoGain = p.mechanoGain;
    sim.params.localMechanoGain = p.localMechanoGain;
    sim.params.bodyGain = p.bodyGain;
}

void printPoint(const V2Point& p) {
    std::printf("[mechanoGain=%.4f localMechanoGain=%.4f dragSettleGain=%.3f | fixed: muscleLeak=%.1f "
                "muscleCaTau=%.2f poseDecay=%.2f frameRateHz=%.2f dragSettleTau=%.2f propDelay=%.3f "
                "bodyGain=%.1f]",
                p.mechanoGain, p.localMechanoGain, p.dragSettleGain, p.muscleLeakScale, p.muscleCalciumTau,
                p.bodyPoseDecayRate, p.bodyFrameRateLimitHz, p.dragSettleTau, p.proprioceptiveDelaySeconds,
                p.bodyGain);
}

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;
    float minCoiledRatio = 1e9f;
    float freqHz = 0.0f;
    float maxAbsHeadingDelta = 0.0f;
    bool healthy = true;
};

Measurement runTrial(const V2Point& pt, int seed, float dragNormal, int warmupSteps, int measureSteps,
                      int freqPosition) {
    Measurement m;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyPoint(sim, pt);
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
    int healthyCount = 0, totalCount = 0;
};

AggregateResult evaluate(const V2Point& pt, int numSeeds, int seedBase, float dragNormal, int warmupSteps,
                          int measureSteps, int freqPosition = 12) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0;
    for (int s = 0; s < numSeeds; ++s) {
        const Measurement m = runTrial(pt, seedBase + s, dragNormal, warmupSteps, measureSteps, freqPosition);
        ar.totalCount++;
        ar.minCoiledRatio = std::min(ar.minCoiledRatio, m.minCoiledRatio);
        ar.maxHeadingDelta = std::max(ar.maxHeadingDelta, m.maxAbsHeadingDelta);
        if (!m.healthy) { ar.allHealthy = false; continue; }
        ar.healthyCount++;
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
    std::printf("  %-6s speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f maxHeadDelta=%.4f "
                "seeds=%d/%d healthy=%s\n",
                label, ar.meanBLps, ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                ar.maxHeadingDelta, ar.healthyCount, ar.totalCount, isHealthy(ar) ? "yes" : "NO");
}

// One evaluated base (agar+water) for a fixed point.
struct BaseResult {
    int base = 0;
    AggregateResult agar, water;
    bool healthyBoth = false;
};

BaseResult evalBase(const V2Point& pt, int base, int seedsPerBase, int warmupSteps, int measureSteps) {
    BaseResult r;
    r.base = base;
    r.agar = evaluate(pt, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
    r.water = evaluate(pt, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
    r.healthyBoth = isHealthy(r.agar) && isHealthy(r.water);
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe point <mechanoGain> <localMechanoGain> <dragSettleGain>
    //             [numBases] [seedsPerBase] [warmupSteps] [measureSteps]
    // All other v2 params fixed at shipped defaults (see V2Point). Default
    // 4x3 for cheap screening; pass 16x3 (different seeds than any prior
    // screen - baseRng is fixed-seeded here, so re-running with more bases
    // extends the SAME sequence rather than reusing it, but for a true
    // independent confirmation pass a different --seed via rngSeed below,
    // or just trust that 16 bases drawn from this stream already are 16
    // independent bases not used by the 4-base screen unless the exact same
    // numBases was used before).
    if (argc >= 2 && std::string(argv[1]) == "point") {
        V2Point pt;
        if (argc > 2) pt.mechanoGain = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) pt.localMechanoGain = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) pt.dragSettleGain = static_cast<float>(std::atof(argv[4]));
        const int numBases = argc > 5 ? std::atoi(argv[5]) : 4;
        const int seedsPerBase = argc > 6 ? std::atoi(argv[6]) : 3;
        const int warmupSteps = argc > 7 ? std::atoi(argv[7]) : 300;
        const int measureSteps = argc > 8 ? std::atoi(argv[8]) : 2500;
        std::printf("point="); printPoint(pt);
        std::printf(" - %d bases x %d seeds, warmup=%d measure=%d\n", numBases, seedsPerBase, warmupSteps, measureSteps);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, bothHealthyCount = 0, waterBeatsAgarFreqCount = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumAgarFreq = 0.0, sumWaterFreq = 0.0;
        int agarFreqCount = 0, waterFreqCount = 0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const BaseResult r = evalBase(pt, base, seedsPerBase, warmupSteps, measureSteps);
            std::printf("base=%d:\n", base);
            printAgg("agar", r.agar);
            printAgg("water", r.water);
            const bool waterFasterFreq = r.healthyBoth && r.water.meanFreqHz > r.agar.meanFreqHz;
            std::printf("  water>agar (freq): %s   healthy(both): %s\n", waterFasterFreq ? "YES" : "no",
                        r.healthyBoth ? "YES" : "no");
            std::fflush(stdout);
            if (isHealthy(r.agar)) { ++agarHealthyCount; sumAgar += r.agar.meanBLps; }
            if (r.agar.healthyCount > 0) { sumAgarFreq += r.agar.meanFreqHz; ++agarFreqCount; }
            if (isHealthy(r.water)) { ++waterHealthyCount; sumWater += r.water.meanBLps; }
            if (r.water.healthyCount > 0) { sumWaterFreq += r.water.meanFreqHz; ++waterFreqCount; }
            if (r.healthyBoth) ++bothHealthyCount;
            if (waterFasterFreq) ++waterBeatsAgarFreqCount;
        }
        const double meanAgarFreq = agarFreqCount ? sumAgarFreq / agarFreqCount : 0.0;
        const double meanWaterFreq = waterFreqCount ? sumWaterFreq / waterFreqCount : 0.0;
        const double meanAgarBL = agarHealthyCount ? sumAgar / agarHealthyCount : 0.0;
        const double meanWaterBL = waterHealthyCount ? sumWater / waterHealthyCount : 0.0;
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s, %.4fHz), water healthy=%d/%d "
                    "(mean %.5f BL/s, %.4fHz), BOTH healthy=%d/%d, water>agar(freq) in %d/%d bases, "
                    "freqRatio(water/agar)=%.3f speedRatio(water/agar)=%.3f\n",
                    numBases, agarHealthyCount, numBases, meanAgarBL, meanAgarFreq, waterHealthyCount, numBases,
                    meanWaterBL, meanWaterFreq, bothHealthyCount, numBases, waterBeatsAgarFreqCount, numBases,
                    meanAgarFreq > 1e-9 ? meanWaterFreq / meanAgarFreq : 0.0,
                    meanAgarBL > 1e-9 ? meanWaterBL / meanAgarBL : 0.0);
        std::printf("Reference: v2 baseline (all 3 free params=0): agar=%.5fBL/s water=%.5fBL/s freq=%.4fHz both "
                    "media, freqRatio=1.000\n", kV2BaselineAgarBL, kV2BaselineWaterBL, kV2BaselineFreq);
        std::printf("Reference: dragSettleGain=16 alone (16-base confirm, not this run's bases): "
                    "agar=%.5fBL/s water=%.5fBL/s speedRatio=1.117, freqRatio=1.000\n", kDragSettle16AgarBL,
                    kDragSettle16WaterBL);
        return 0;
    }

    // ./exe random <n> <mechanoGainMax> <localMechanoGainMax> <dragSettleGainMax>
    //              <numBases> <seedsPerBase> <rngSeed>
    //
    // mechanoGain drawn from [-mechanoGainMax, +mechanoGainMax] (sign
    // INCLUDED): DVA/mechanosensation feedback is biologically more likely
    // positive-gain (a real stretch receptor exciting premotor
    // interneurons that amplify locomotor amplitude under load), but this
    // project has repeatedly found that citation does not reliably predict
    // sign for these load-based feedback axes once they interact with the
    // rest of the network (e.g. dragAdhesionGain's sign was NOT predicted a
    // priori either, see WormSim.h's own comment on it) - cheap to include
    // both signs in a screen, so we do.
    // localMechanoGain drawn from [0, localMechanoGainMax] ONLY (no
    // negative): it is added directly, unsigned, into the same windowed
    // proprioceptive feedback sum that already carries the signed
    // dorsal/ventral traveling-wave term (applyProprioception) - a negative
    // local load term would subtract excitation asymmetrically depending on
    // which side's window currently has more load, which has no clean
    // biological reading (real B/D-class stretch receptors are understood
    // as excitatory to their own local circuit under stretch, not
    // inhibitory) and duplicates the already-signed proprioceptive term's
    // job if allowed to flip sign. Positive-only, per WORM_V2_DESIGN.md's
    // own framing of this channel as a magnitude-only addition to c_n-style
    // physical resistance sensing.
    // dragSettleGain drawn from [0, dragSettleGainMax] ONLY: WORM_V2_
    // DESIGN.md section 4.3 defines it as strictly additive resistance
    // (cnk = cn + gain*penetration_depth, penetration_depth>=0) - negative
    // gain would mean the settled substrate REDUCES drag below the bare
    // linear RFT value, which has no citation behind it in this project
    // (Rabets 2014 describes settling/sticking, never a lubricating,
    // drag-reducing regime) and was never tried in the v2 dragSettleGain
    // sweep (WORM_V2_RESULTS.md section 4 only screened >=0).
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int n = argc > 2 ? std::atoi(argv[2]) : 40;
        const float mechanoGainMax = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 1.0f;
        const float localMechanoGainMax = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 1.0f;
        const float dragSettleGainMax = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 30.0f;
        const int numBases = argc > 6 ? std::atoi(argv[6]) : 4;
        const int seedsPerBase = argc > 7 ? std::atoi(argv[7]) : 3;
        const unsigned rngSeed = argc > 8 ? static_cast<unsigned>(std::atoi(argv[8])) : 777u;
        const int warmupSteps = 300, measureSteps = 2500;

        std::printf("random: n=%d mechanoGainMax=+/-%.4f localMechanoGainMax=[0,%.4f] "
                    "dragSettleGainMax=[0,%.3f] bases=%d seeds/base=%d rngSeed=%u\n",
                    n, mechanoGainMax, localMechanoGainMax, dragSettleGainMax, numBases, seedsPerBase, rngSeed);
        std::printf("(mechanoGain signed, localMechanoGain/dragSettleGain positive-only - see source comment "
                    "for why)\n\n");

        std::mt19937 paramRng(rngSeed);
        std::mt19937 baseRng(rngSeed + 999983u); // independent stream from paramRng, deterministic per rngSeed
        std::uniform_real_distribution<float> mechanoDist(-mechanoGainMax, mechanoGainMax);
        std::uniform_real_distribution<float> localDist(0.0f, localMechanoGainMax);
        std::uniform_real_distribution<float> dragDist(0.0f, dragSettleGainMax);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);

        int healthyBothTrialCount = 0, unhealthyTrialCount = 0;
        int waterBeatsAgarFreqTrialCount = 0;
        for (int t = 0; t < n; ++t) {
            V2Point pt;
            pt.mechanoGain = mechanoDist(paramRng);
            pt.localMechanoGain = localDist(paramRng);
            pt.dragSettleGain = dragDist(paramRng);

            int agarHealthyBases = 0, waterHealthyBases = 0;
            double sumAgarBL = 0.0, sumWaterBL = 0.0, sumAgarFreq = 0.0, sumWaterFreq = 0.0;
            int agarFreqCount = 0, waterFreqCount = 0;
            for (int b = 0; b < numBases; ++b) {
                const int base = baseDist(baseRng);
                const BaseResult r = evalBase(pt, base, seedsPerBase, warmupSteps, measureSteps);
                if (isHealthy(r.agar)) { ++agarHealthyBases; sumAgarBL += r.agar.meanBLps; }
                if (r.agar.healthyCount > 0) { sumAgarFreq += r.agar.meanFreqHz; ++agarFreqCount; }
                if (isHealthy(r.water)) { ++waterHealthyBases; sumWaterBL += r.water.meanBLps; }
                if (r.water.healthyCount > 0) { sumWaterFreq += r.water.meanFreqHz; ++waterFreqCount; }
            }
            const bool healthyBoth = (agarHealthyBases == numBases) && (waterHealthyBases == numBases);
            if (!healthyBoth) { ++unhealthyTrialCount; continue; }
            ++healthyBothTrialCount;

            const double meanAgarBL = sumAgarBL / agarHealthyBases;
            const double meanWaterBL = sumWaterBL / waterHealthyBases;
            const double meanAgarFreq = agarFreqCount ? sumAgarFreq / agarFreqCount : 0.0;
            const double meanWaterFreq = waterFreqCount ? sumWaterFreq / waterFreqCount : 0.0;
            const double freqRatio = meanAgarFreq > 1e-9 ? meanWaterFreq / meanAgarFreq : 0.0;
            const double speedRatio = meanAgarBL > 1e-9 ? meanWaterBL / meanAgarBL : 0.0;
            const bool waterBeatsAgarFreq = meanWaterFreq > meanAgarFreq;
            if (waterBeatsAgarFreq) ++waterBeatsAgarFreqTrialCount;

            std::printf("trial %3d HEALTHY(both, %d/%d bases): mechanoGain=%+.4f localMechanoGain=%.4f "
                        "dragSettleGain=%.3f | agar=%.5fBL/s(%.4fHz) water=%.5fBL/s(%.4fHz) "
                        "freqRatio=%.3f speedRatio=%.3f water>agar(freq)=%s\n",
                        t, numBases, numBases, pt.mechanoGain, pt.localMechanoGain, pt.dragSettleGain,
                        meanAgarBL, meanAgarFreq, meanWaterBL, meanWaterFreq, freqRatio, speedRatio,
                        waterBeatsAgarFreq ? "YES" : "no");
            std::fflush(stdout);
        }
        std::printf("\nrandom summary: %d trials, %d healthy on BOTH media (all %d bases each), %d unhealthy "
                    "(skipped above), %d/%d healthy trials had water>agar frequency\n",
                    n, healthyBothTrialCount, numBases, unhealthyTrialCount, waterBeatsAgarFreqTrialCount,
                    healthyBothTrialCount);
        std::printf("Reference: v2 baseline (all 3 free params=0): agar=%.5fBL/s water=%.5fBL/s freq=%.4fHz both "
                    "media, freqRatio=1.000\n", kV2BaselineAgarBL, kV2BaselineWaterBL, kV2BaselineFreq);
        return 0;
    }

    std::printf("Usage:\n"
                "  %s point <mechanoGain> <localMechanoGain> <dragSettleGain> [numBases=4] [seedsPerBase=3] "
                "[warmupSteps=300] [measureSteps=2500]\n"
                "  %s random <n> <mechanoGainMax> <localMechanoGainMax> <dragSettleGainMax> [numBases=4] "
                "[seedsPerBase=3] [rngSeed=777]\n",
                argv[0], argv[0]);
    return 0;
}
