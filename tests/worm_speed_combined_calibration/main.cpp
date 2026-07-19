// tests/worm_speed_combined_calibration/main.cpp
//
// Fifth axis on the swim>crawl speed bug (see tests/worm_speed_calibration
// and tests/worm_speed_leak_calibration for the full history of axes 1-4,
// all either insufficient or reverted). This file combines the TWO axes
// that, of everything tried this project's history, are the only ones that
// touch the network's actual VECTOR FIELD rather than just adding forcing
// on top of it:
//   - Network::scale_type_params (7 params: leak/capacitance per
//     architectural class - tests/worm_speed_leak_calibration, RESULT:
//     0/480 healthy water>agar candidates found via literature-grounded
//     bounded random search [0.2x, 300x], own base each trial)
//   - Network::scale_synapse_sign (3 params: excitatory/inhibitory/gap
//     multipliers on the raw Cook et al. 2019 contact-count weights -
//     tests/worm_synapse_speed_calibration, RESULT: found a real, adversarially-
//     confirmed ~3.1x net-progress speedup on agar at
//     [chemExc=2.160 chemInh=0.142 gap=0.103], briefly SHIPPED, then REVERTED
//     - see WormSim.cpp ctor comment - because it caused consistent
//     near-180-degree single-step heading reversals, a failure mode NONE of
//     that search's health gates ever measured)
//
// Rationale for combining rather than re-running either alone: synapse-sign
// is the ONLY axis with a demonstrated real effect on speed - the problem
// was never "no effect", it was "effect comes with a side effect no gate
// caught". leak/capacitance alone has no demonstrated effect at all. The
// hypothesis this file tests: does retuning per-class time constants
// ALONGSIDE a synapse-sign rescaling (in the same joint 10-parameter search)
// find an operating point where the speedup survives but the heading
// pathology doesn't - i.e. does leak/capacitance have EXACTLY the kind of
// leverage needed to counteract synapse-sign's side effect, even though it
// has no leverage on speed by itself? This is a genuinely different
// question from either file alone answered.
//
// Health gate: same efficiency/coiledRatio/freqHz gate as every file in this
// project, PLUS max |heading delta| in one step - baked in from the START
// this time (kMaxHeadingDeltaRad below), the exact metric whose ABSENCE from
// the search let the synapse-sign candidate ship broken the first time.
//
// Search: bounded random, log-uniform per parameter, EACH trial on its own
// fresh independent seed base (not confirmed on the same base twice) - same
// discipline as tests/worm_speed_leak_calibration. Ranges: leak/capacitance
// params default to the literature-grounded [0.2, 300] range agreed by three
// independent research agents (see tests/worm_speed_leak_calibration
// header); synapse-sign params default centered on the KNOWN real-effect
// region ([2.16, 0.142, 0.103]) with a configurable spread, since that is
// the one point in this project's history with a confirmed non-null effect
// worth searching NEAR, not from scratch.
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
constexpr float kMaxHeadingDeltaRad = 0.5f;  // see tests/worm_speed_leak_calibration for rationale

// leak/cap (7) then synapse-sign (3) - 10 total.
constexpr int kNumParams = 10;
enum ParamIdx {
    kLeakIP = 0, kLeakP = 1, kLeakPO = 2, kCapIP = 3, kCapP = 4, kCapPO = 5, kCapO = 6,
    kChemExc = 7, kChemInh = 8, kGap = 9,
};
using Candidate = std::array<float, kNumParams>;
const Candidate kIdentity = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
// The one point in this project's history with a confirmed non-null speed
// effect (see tests/worm_synapse_speed_calibration) - a natural search
// anchor for the synapse-sign half of this joint search.
const Candidate kSynapseSignWinner = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 2.160f, 0.142f, 0.103f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    connectome::Network& net = sim.network();
    net.scale_type_params(connectome::NeuronType::InputProcessing, c[kLeakIP], c[kCapIP]);
    net.scale_type_params(connectome::NeuronType::Processing, c[kLeakP], c[kCapP]);
    net.scale_type_params(connectome::NeuronType::ProcessingOutput, c[kLeakPO], c[kCapPO]);
    net.scale_type_params(connectome::NeuronType::Output, 1.0f, c[kCapO]);
    net.scale_synapse_sign(c[kChemExc], c[kChemInh], c[kGap]);
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
                      int freqPosition) {
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
    std::printf("[leakIP=%.3f leakP=%.3f leakPO=%.3f capIP=%.3f capP=%.3f capPO=%.3f capO=%.3f chemExc=%.3f "
                "chemInh=%.3f gap=%.3f]",
                c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe distribution leakIP leakP leakPO capIP capP capPO capO chemExc chemInh gap [numBases] [seedsPerBase]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        for (int i = 0; i < kNumParams && argc > 2 + i; ++i) cand[i] = static_cast<float>(std::atof(argv[2 + i]));
        const int numBases = argc > 12 ? std::atoi(argv[12]) : 12;
        const int seedsPerBase = argc > 13 ? std::atoi(argv[13]) : 8;
        const int warmupSteps = argc > 14 ? std::atoi(argv[14]) : 300;
        const int measureSteps = argc > 15 ? std::atoi(argv[15]) : 2500;
        std::printf("cand="); printCand(cand); std::printf(" - %d bases x %d seeds\n", numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0;
        double sumAgar = 0.0, sumWater = 0.0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(cand, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(cand, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
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

    // ./exe random <trials> <leakCapLo> <leakCapHi> <synSpread> [seedsPerTrial] [rngSeed]
    // leak/cap 7 params: log-uniform in [leakCapLo, leakCapHi] (literature-
    // grounded default [0.2, 300], see tests/worm_speed_leak_calibration).
    // synapse-sign 3 params: log-uniform CENTERED ON kSynapseSignWinner with
    // a multiplicative spread of synSpread each way (e.g. synSpread=2.0
    // searches [winner/2, winner*2] per param) - anchored on the one
    // confirmed-real point in this project's history, not searched from
    // scratch like leak/capacitance (no prior evidence it has ANY effect
    // alone, so no informative center to anchor around).
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 200;
        const float leakCapLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.2f;
        const float leakCapHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 300.0f;
        const float synSpread = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 2.0f;
        const int seedsPerTrial = argc > 6 ? std::atoi(argv[6]) : 6;
        const unsigned rngSeed = argc > 7 ? static_cast<unsigned>(std::atoi(argv[7])) : 1u;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, leak/cap log-uniform [%.3f,%.3f], synapse-sign spread %.2fx around "
                    "known winner, %d seeds/trial, rngSeed=%u\n",
                    trials, leakCapLo, leakCapHi, synSpread, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> leakCapLogDist(std::log(leakCapLo), std::log(leakCapHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            for (int i = 0; i < 7; ++i) cand[i] = std::exp(leakCapLogDist(rng));
            for (int i = 7; i < kNumParams; ++i) {
                const float center = kSynapseSignWinner[i];
                std::uniform_real_distribution<float> synLogDist(std::log(center / synSpread), std::log(center * synSpread));
                cand[i] = std::exp(synLogDist(rng));
            }
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            const bool agarOk = isHealthy(agar);
            if (!agarOk) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            const bool waterOk = isHealthy(water);
            const bool waterFaster = waterOk && water.meanBLps > agar.meanBLps;
            if (waterFaster) {
                ++found;
                printCand(cand);
                std::printf(" base=%d ", base);
                printAgg(" agar", agar);
                printAgg(" water", water);
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials had a healthy candidate with water>agar on its OWN (single, fresh) base - each "
                    "one still needs 'distribution' confirmation across many MORE independent bases before it "
                    "means anything (see header).\n",
                    found, trials);
        return 0;
    }

    std::printf("Usage:\n  %s random <trials> <leakCapLo> <leakCapHi> <synSpread> [seedsPerTrial] [rngSeed]\n  %s "
                "distribution leakIP leakP leakPO capIP capP capPO capO chemExc chemInh gap [numBases] "
                "[seedsPerBase]\n",
                argv[0], argv[0]);
    return 0;
}
