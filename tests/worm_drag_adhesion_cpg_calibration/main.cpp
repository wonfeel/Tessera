// tests/worm_drag_adhesion_cpg_calibration/main.cpp
//
// Joint search of Params::dragAdhesionGain (body.cpp force-law fix) x the
// full CPG parameter set (WormSim.h/.cpp rhythm generator). Motivated by a
// specific mathematical argument, not a blind "combine the two best axes"
// guess:
//
// tests/worm_drag_adhesion_calibration found dragAdhesionGain ALONE gets
// water/agar speed ratio to ~0.95-0.98 and no higher, however high gain goes
// (tested to 5000) - a genuine, robust, health-preserving effect that still
// falls short. WHY it has a ceiling at exactly 1.0 (not just "saturates
// somewhere below 1"): the mechanism is c_n_k = c_n * max(0, 1+gain*|u_k|),
// where |u_k| is the segment's shape-change-only velocity - a PURELY
// KINEMATIC quantity (curvature->angle in WormBody::step never reads drag at
// all, see body.cpp's own top-of-file comment) that is IDENTICAL between
// agar/water runs whenever nothing feeds mechanical load back into the
// network's drive (true of the isolated dragAdhesionGain search: mechanoGain=
// localMechanoGain=cpgGain=0 there). As gain->infinity, c_n_k is dominated by
// gain*|u_k| for BOTH media - RFT's own well-known infinite-anisotropy limit
// makes the force-balance SOLUTION a function of the shape pattern alone,
// independent of the actual c_n magnitude. Since |u_k| is identical between
// media in that configuration, BOTH media's speed converges to the EXACT SAME
// asymptotic value - i.e. this functional form can only reach water==agar in
// the limit, never water>agar, no matter how high gain goes. This is a
// structural property of the mechanism, confirmed by the empirical data
// (ratio rising ever more slowly toward 1.0, never crossing, at gain up to
// 5000 - see tests/worm_drag_adhesion_calibration's RESULT).
//
// The fix this file tests: break the "|u_k| is identical between media"
// assumption HONESTLY, not by hardcoding an asymmetry - the CPG (tests/worm_
// cpg_calibration) already makes frequency AND amplitude real functions of
// m_body.mechanical_load(), which genuinely differs by ~23x between agar and
// water (see WormSim.h's dragNormal comment). Once CPG is active, |u_k|
// itself should differ meaningfully between media (higher frequency + load-
// dependent amplitude in water vs agar), which could let dragAdhesionGain's
// saturating term push water PAST the shared-kinematics ceiling instead of
// only ever approaching it. This is a genuinely new hypothesis (not "try
// bigger gain" on either axis alone) - CPG alone was 0/300 (tests/worm_cpg_
// calibration, timing right but propulsion-efficiency-per-cycle still ~0.14
// water/agar at matched frequency); dragAdhesionGain alone is 0.95-0.98
// capped by the argument above. Neither one touches what the other is
// missing.
//
// Search scope: dragAdhesionGain jointly with the SAME 7 CPG-family
// parameters tests/worm_cpg_calibration searched (cpgGain, cpgBaseFreqHz,
// cpgLoadSensitivity, cpgAmpLoadSensitivity, bodyPoseDecayRate,
// muscleLeakScale, motorLeakScale) - 8 total. Ranges are NOT re-derived from
// scratch - they are inherited directly from the two predecessor files'
// already-validated bounds (dragAdhesionGain from tests/worm_drag_adhesion_
// calibration's confirmed-healthy region; the other 7 from tests/worm_cpg_
// calibration's header, itself corrected mid-session for cpgLoadSensitivity's
// real empirical scale - see that file for the full derivation). cpgGain
// uses LINEAR (not log) [0,50] here specifically so 0 (== CPG fully off, pure
// dragAdhesionGain) remains a reachable corner of the search, not excluded by
// construction - unlike tests/worm_cpg_calibration where cpgGain=0 was
// handled by a separate identity candidate, not the search range itself.
//
// Fitness/reporting: the user's explicit target is NOT "water>agar" as a
// boolean - real C. elegans swims 2-3x faster than it crawls (ratio ~2.0-3.0,
// see WormSim.h/tests/worm_speed_calibration's Fang-Yen et al. 2010
// citations), and settling for "just past 1.0" would not be honest progress
// toward that number. This file tracks and reports the ACTUAL water/agar
// ratio for every healthy candidate, not just a >1 flag, and separately
// counts candidates clearing 1.0, 1.5, and 2.0 so a partial win is visible
// as partial, not rounded up to "solved".
//
// Health gate: identical structure/thresholds to every other axis this
// project has tried (efficiency>=0.40, coiledRatio>=0.30, freqHz>=
// kMinFreqHz, maxAbsHeadingDelta<=kMaxHeadingDeltaRad) - baked in from the
// first trial. Multi-independent-seed-base evaluation from the first fitness
// call (this project's repeatedly learned lesson).
//
// RESULT: screen (adhesion x cpgGain grid, muscleLeak=0.2/motorLeak=20.0
// fixed companions) showed the two mechanisms do NOT combine gracefully at
// these fixed points: any nonzero adhesion combined with any nonzero cpgGain
// (5/15/30) collapsed the gait entirely (freq=0.0000Hz, maxHeadDelta
// 1.5-3.1 rad - a frozen-arc/wild-heading pathology, not a borderline
// unhealthy result). 300-trial random search across the full 8-parameter
// space (dragAdhesionGain linear [0,60], cpgGain linear [0,50], the other 6
// CPG-family params on tests/worm_cpg_calibration's own ranges): 0/300
// healthy-on-both with ratio>1.0. Best ratio found: 0.983 (cpgGain=12.08,
// i.e. CPG genuinely active, not just a gain=0 corner) - essentially
// UNCHANGED from tests/worm_drag_adhesion_calibration's own solo ceiling
// (0.95-0.98) - CPG's genuine load-responsive |u_k| asymmetry did not
// meaningfully move this mechanism's ceiling. Conclusion: dragAdhesionGain
// (multiplicative, c_n*(1+gain*|u_k|)) and cpgGain do not combine
// productively - see tests/worm_drag_adhesion_additive_calibration instead,
// where a differently-shaped (additive, c_n+gain*|u_k|) version of the same
// physical idea DOES break past ratio=1.0 on its own (first such result in
// this project's history) - though combining THAT mechanism with cpgGain
// (tests/worm_drag_adhesion_additive_cpg_calibration) also failed (0/300,
// best 0.424, worse than the additive mechanism alone). Ships at
// dragAdhesionGain=0.0 (unchanged Params default).
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
constexpr float kFixedWavelengths = 1.0f;

// {dragAdhesionGain, cpgGain, cpgBaseFreqHz, cpgLoadSensitivity, cpgAmpLoadSensitivity, bodyPoseDecayRate, muscleLeakScale, motorLeakScale}
using Candidate = std::array<float, 8>;
enum { kAdhesion = 0, kCpgGain = 1, kCpgFreq = 2, kCpgSens = 3, kCpgAmpSens = 4, kPoseDecay = 5, kMuscleLeak = 6, kMotorLeak = 7 };
const Candidate kIdentity = {0.0f, 0.0f, 2.0f, 0.05f, 0.0f, 0.5f, 0.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.dragAdhesionGain = c[kAdhesion];
    sim.params.cpgGain = c[kCpgGain];
    sim.params.cpgBaseFreqHz = c[kCpgFreq];
    sim.params.cpgLoadSensitivity = c[kCpgSens];
    sim.params.cpgAmpLoadSensitivity = c[kCpgAmpSens];
    sim.params.cpgWavelengths = kFixedWavelengths;
    sim.params.bodyPoseDecayRate = c[kPoseDecay];
    sim.params.muscleLeakScale = c[kMuscleLeak];
    sim.params.motorLeakScale = c[kMotorLeak];
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

void printAgg(const char* label, const AggregateResult& ar) {
    std::printf("  %-6s speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f maxHeadDelta=%.4f healthy=%s\n",
                label, ar.meanBLps, ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                ar.maxHeadingDelta, isHealthy(ar) ? "yes" : "NO");
}

void printCand(const Candidate& c) {
    std::printf("[adhesion=%.3f cpgGain=%.3f cpgFreq=%.3f cpgSens=%.3f cpgAmpSens=%.4f poseDecay=%.3f "
                "muscleLeak=%.3f motorLeak=%.3f]",
                c[kAdhesion], c[kCpgGain], c[kCpgFreq], c[kCpgSens], c[kCpgAmpSens], c[kPoseDecay], c[kMuscleLeak],
                c[kMotorLeak]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution adhesion cpgGain cpgFreq cpgSens cpgAmpSens poseDecay muscleLeak motorLeak [numBases] [seedsPerBase] [warmup] [measure]
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
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0;
        int ratioAbove15 = 0, ratioAbove20 = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumRatio = 0.0;
        float minRatio = 1e9f, maxRatio = -1e9f;
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
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (bothOk) {
                sumRatio += ratio;
                minRatio = std::min(minRatio, ratio);
                maxRatio = std::max(maxRatio, ratio);
                if (ratio > 1.0f) ++waterBeatsAgarCount;
                if (ratio > 1.5f) ++ratioAbove15;
                if (ratio > 2.0f) ++ratioAbove20;
            }
        }
        const int bothHealthyCount = std::min(agarHealthyCount, waterHealthyCount);
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s), water healthy=%d/%d (mean "
                    "%.5f BL/s)\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0);
        std::printf("ratio water/agar (both-healthy bases only): mean=%.3f min=%.3f max=%.3f, water>agar in %d "
                    "bases, ratio>1.5 in %d, ratio>2.0 in %d (out of bothHealthy~=%d)\n",
                    bothHealthyCount ? sumRatio / std::max(1, waterBeatsAgarCount + (bothHealthyCount - waterBeatsAgarCount)) : 0.0,
                    minRatio > 1e8f ? 0.0f : minRatio, maxRatio < -1e8f ? 0.0f : maxRatio, waterBeatsAgarCount,
                    ratioAbove15, ratioAbove20, bothHealthyCount);
        return 0;
    }

    // ./exe random <trials> <adhLo> <adhHi> <cpgGainLo> <cpgGainHi> <freqLo> <freqHi> <sensLo> <sensHi> <ampSensLo> <ampSensHi> <decayLo> <decayHi> <leakLo> <leakHi> <motorLo> <motorHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float adhLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.0f;
        const float adhHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 60.0f;
        const float cpgGainLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.0f;
        const float cpgGainHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 50.0f;
        const float freqLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.5f;
        const float freqHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 4.0f;
        const float sensLo = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.005f;
        const float sensHi = argc > 10 ? static_cast<float>(std::atof(argv[10])) : 2.0f;
        const float ampSensLo = argc > 11 ? static_cast<float>(std::atof(argv[11])) : 0.001f;
        const float ampSensHi = argc > 12 ? static_cast<float>(std::atof(argv[12])) : 0.2f;
        const float decayLo = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 0.1f;
        const float decayHi = argc > 14 ? static_cast<float>(std::atof(argv[14])) : 2.0f;
        const float leakLo = argc > 15 ? static_cast<float>(std::atof(argv[15])) : 0.02f;
        const float leakHi = argc > 16 ? static_cast<float>(std::atof(argv[16])) : 3.0f;
        const float motorLo = argc > 17 ? static_cast<float>(std::atof(argv[17])) : 1.0f;
        const float motorHi = argc > 18 ? static_cast<float>(std::atof(argv[18])) : 100.0f;
        const int seedsPerTrial = argc > 19 ? std::atoi(argv[19]) : 6;
        const unsigned rngSeed = argc > 20 ? static_cast<unsigned>(std::atoi(argv[20])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, adhesion linear [%.2f,%.2f], cpgGain linear [%.2f,%.2f], freq "
                    "log-uniform [%.2f,%.2f]Hz, sens log-uniform [%.3f,%.3f], ampSens log-uniform [%.3f,%.3f], "
                    "poseDecay log-uniform [%.2f,%.2f], muscleLeak log-uniform [%.2f,%.2f], motorLeak log-uniform "
                    "[%.2f,%.2f], %d seeds/trial, rngSeed=%u\n",
                    trials, adhLo, adhHi, cpgGainLo, cpgGainHi, freqLo, freqHi, sensLo, sensHi, ampSensLo, ampSensHi,
                    decayLo, decayHi, leakLo, leakHi, motorLo, motorHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> adhDist(adhLo, adhHi);
        std::uniform_real_distribution<float> cpgGainDist(cpgGainLo, cpgGainHi);
        std::uniform_real_distribution<float> freqLogDist(std::log(freqLo), std::log(freqHi));
        std::uniform_real_distribution<float> sensLogDist(std::log(sensLo), std::log(sensHi));
        std::uniform_real_distribution<float> ampSensLogDist(std::log(ampSensLo), std::log(ampSensHi));
        std::uniform_real_distribution<float> decayLogDist(std::log(decayLo), std::log(decayHi));
        std::uniform_real_distribution<float> leakLogDist(std::log(leakLo), std::log(leakHi));
        std::uniform_real_distribution<float> motorLogDist(std::log(motorLo), std::log(motorHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int foundOver1 = 0, foundOver15 = 0, foundOver2 = 0;
        float bestRatio = -1.0f;
        Candidate bestCand{};
        int bestBase = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kAdhesion] = adhDist(rng);
            cand[kCpgGain] = cpgGainDist(rng);
            cand[kCpgFreq] = std::exp(freqLogDist(rng));
            cand[kCpgSens] = std::exp(sensLogDist(rng));
            cand[kCpgAmpSens] = std::exp(ampSensLogDist(rng));
            cand[kPoseDecay] = std::exp(decayLogDist(rng));
            cand[kMuscleLeak] = std::exp(leakLogDist(rng));
            cand[kMotorLeak] = std::exp(motorLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            const bool agarOk = isHealthy(agar);
            if (!agarOk) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            const bool waterOk = isHealthy(water);
            if (!waterOk || agar.meanBLps <= 1e-9f) continue;
            const float ratio = water.meanBLps / agar.meanBLps;
            if (ratio > bestRatio) { bestRatio = ratio; bestCand = cand; bestBase = base; }
            if (ratio > 1.0f) {
                ++foundOver1;
                if (ratio > 1.5f) ++foundOver15;
                if (ratio > 2.0f) ++foundOver2;
                std::printf("ratio=%.3f base=%d ", ratio, base);
                printCand(cand);
                std::printf("\n");
                printAgg("  agar", agar);
                printAgg("  water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials healthy-on-both with ratio>1.0 (water>agar); %d with ratio>1.5; %d with "
                    "ratio>2.0. Best ratio found: %.3f",
                    foundOver1, trials, foundOver15, foundOver2, bestRatio < 0.0f ? 0.0f : bestRatio);
        if (bestRatio > 0.0f) {
            std::printf(" at base=%d ", bestBase);
            printCand(bestCand);
        }
        std::printf("\nEach ratio>1.0 hit above still needs 'distribution' confirmation across many MORE "
                    "independent bases before it means anything (see header).\n");
        return 0;
    }

    // Default: quick screen - dragAdhesionGain x cpgGain grid, other CPG
    // params fixed at reasonable values (freq=2.0, sens=0.05, ampSens=0.02 -
    // matching tests/worm_cpg_calibration's own screen defaults; muscleLeak/
    // motorLeak raised from 0/1 defaults since CPG needs them to have ANY
    // measurable effect, see that file's header), one seed base (shortlist
    // only - confirm anything promising with 'distribution' or 'random').
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    const float kAdhesions[] = {0.0f, 10.0f, 20.0f, 40.0f};
    const float kCpgGains[] = {0.0f, 5.0f, 15.0f, 30.0f};
    std::printf("=== Screen (dragAdhesionGain x cpgGain grid, freq=2.0/sens=0.05/ampSens=0.02/poseDecay=0.5/"
                "muscleLeak=0.2/motorLeak=20.0 fixed, %d seeds/point, base=%d) ===\n",
                kScreenSeeds, kSeedBase);
    for (float a : kAdhesions) {
        for (float g : kCpgGains) {
            const Candidate cand = {a, g, 2.0f, 0.05f, 0.02f, 0.5f, 0.2f, 20.0f};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            const bool bothOk = isHealthy(agar) && isHealthy(water);
            const float ratio = (bothOk && agar.meanBLps > 1e-9f) ? water.meanBLps / agar.meanBLps : -1.0f;
            std::printf("adhesion=%.1f cpgGain=%.1f:\n", a, g);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  ratio water/agar: %s\n", bothOk ? std::to_string(ratio).c_str() : "N/A (unhealthy)");
            std::fflush(stdout);
        }
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising point with "
                "'distribution' or search jointly with 'random' across many seed bases before believing it (see "
                "header).\n");
    return 0;
}
