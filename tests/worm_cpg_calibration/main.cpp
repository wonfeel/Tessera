// tests/worm_cpg_calibration/main.cpp
//
// Calibration for Params::cpgGain/cpgBaseFreqHz/cpgLoadSensitivity
// (WormSim.h/.cpp) - the walking-rhythm generator (CPG), this project's one
// deliberate, explicitly user-authorized exception to "behavior only from
// the real connectome network" (see WormSim.h's Params::cpgGain comment and
// WORM.md section 6 for the full justification). Added only after 7+
// independent, honest axes (leak/capacitance, synapse-sign, B-class active
// current alone and jointly with body pose decay, local mechanosensation,
// muscle leak alone and jointly with body pose decay - see tests/worm_speed_
// calibration through tests/worm_muscle_body_joint_calibration) all failed
// to make the real 401-neuron connectome hold a realistic bend frequency
// without breaking propulsion. Precedent: OpenWorm's 2024 integrative model
// (fully hand-fed sinusoid) and Ji et al. 2021, eLife (phenomenological
// switching oscillator over real RFT body physics) - both introduce a
// non-biological rhythm source for the same reason.
//
// WHAT STAYS HONEST HERE (the whole point of testing this at all): the
// CPG's FREQUENCY is not a hardcoded per-environment number - it's a real
// function of m_body.mechanical_load(), the same physically-computed load
// signal already driving DVA/mechanoGain, read AFTER the body physics solve
// each step. Lower load (water) -> higher frequency EMERGES from the
// formula, exactly mirroring Fang-Yen et al. 2010's finding that real
// C. elegans bend frequency falls continuously with rising mechanical load.
// The real network still decides everything else (heading, turning,
// reactions to food/temperature) - this mechanism only forces a TEMPO
// through the real motor neurons (Network::add_input, additive on top of
// applyProprioception, not a replacement).
//
// Equation (WormSim.cpp::applyRhythmGenerator):
//   instFreqHz = cpgBaseFreqHz / (1 + cpgLoadSensitivity * normalizedLoad)
//   phase += 2*pi*instFreqHz*dt
//   drive_i = cpgGain * (dorsal ? +1 : -1) * sin(phase - pos_i * 2*pi*cpgWavelengths/24)
// cpgWavelengths is held FIXED at its literature-anchored default (1.0 -
// roughly one full bend wavelength along the body, the usual estimate for
// crawling) for this search - not because it's unimportant, but to keep the
// FIRST search focused on the 3 parameters most directly tied to the actual
// crawl/swim speed question, matching this project's own "don't search
// everything at once" discipline (see e.g. worm_bclass_oscillator_
// calibration being isolated before worm_bclass_body_joint_calibration).
// bodyPoseDecayRate (WormBody's own angle-decay time constant) IS included
// as a 4th free parameter - the historical lesson from this exact project
// (network.cpp's "TRIED CHANGING, REVERTED" account, and tests/worm_muscle_
// body_joint_calibration's confirmation) is that a faster commanded rhythm
// and the body's own ability to turn it into propulsion are coupled, not
// independent - searching bodyPoseDecayRate jointly from the start avoids
// repeating that exact mistake a third time.
//
// muscleLeakScale (Output/muscle leak, see tests/worm_muscle_body_joint_
// calibration) is included as a 5th free parameter, added mid-session after
// this file's OWN first screen (muscleLeakScale implicitly 0, the Params
// default) found the injected CPG signal never showed up as measured
// frequency at all - stayed ~0.01-0.02Hz regardless of cpgBaseFreqHz. Cause:
// the muscle (Output) layer's own architectural near-zero leak integrates/
// averages away any fast upstream input before curvature is ever read from
// it - the CPG can drive the real motor neurons all it wants, but a
// downstream integrator with a ~755s time constant (tests/worm_network_
// eigenmodes) cannot track a 1-4Hz signal. This is not scope creep - it is
// the direct, necessary companion finding: the CPG can only matter if the
// muscle layer can actually follow it.
//
// Search ranges (bounded, not an unconstrained walk):
//   cpgGain            in [0.5, 50]     - no literature unit (this reduced
//                        model's own injected-current scale); lower bound
//                        anchored to be comparable to the existing
//                        proprioceptiveGain default (4.0) so the CPG can
//                        meaningfully compete with/dominate the real
//                        feedback signal it's added on top of, not be lost
//                        in its noise floor.
//   cpgBaseFreqHz      in [0.5, 4.0]Hz  - literature-anchored: real crawl
//                        ~0.3-0.5Hz, real swim ~1.7-2Hz (Fang-Yen et al.
//                        2010) - this is the zero-load LIMIT, so the upper
//                        bound is set somewhat above the real swim ceiling
//                        (real water is never quite zero load) rather than
//                        exactly at it.
//   cpgLoadSensitivity in [0.005, 2.0]  - no literature unit (this reduced
//                        model's own load-to-frequency coupling constant).
//                        CORRECTED mid-session via direct trace diagnostic
//                        (WormSim::debugCpgPhase): normalizedLoad = m_body.
//                        mechanical_load()/dragNormal turns out to be ~70-80
//                        on agar, NOT O(1) as assumed by analogy with
//                        mechanoGain's own documentation - the original
//                        [0.1,50] range made even its SMALLEST value
//                        (0.1) suppress frequency by a factor of ~8, and its
//                        typical tested value (5.0) by a factor of ~400,
//                        which is why early screens showed NO cpgBaseFreqHz
//                        effect at all (not attenuation - the phase was
//                        genuinely advancing ~350x slower than intended,
//                        confirmed by directly tracing cpgPhase step-by-step,
//                        not inferred). This range is picked to span
//                        "barely suppressed" (~2Hz) to "heavily suppressed"
//                        (~0.03Hz) at agar's actual load scale.
//   bodyPoseDecayRate  in [0.1, 2.0]    - identical range already used and
//                        justified in tests/worm_bclass_body_joint_
//                        calibration and tests/worm_muscle_body_joint_
//                        calibration for the same physical parameter.
//   muscleLeakScale    in [0.02, 3.0]   - identical range already used and
//                        justified in tests/worm_muscle_body_joint_
//                        calibration for the same physical parameter.
//   cpgAmpLoadSensitivity in [0.001, 0.2] - added mid-session: only frequency
//                        responded to load until now, but real C. elegans
//                        also changes bend AMPLITUDE/waveform between crawl
//                        and swim (Fang-Yen et al. 2010) - instGain =
//                        cpgGain*(1+cpgAmpLoadSensitivity*normalizedLoad),
//                        amplitude GROWS with load (opposite direction from
//                        frequency) - large slow bends on agar, small fast
//                        ones in water. Honest caveat: unlike the frequency-
//                        load relationship (directly measured by Fang-Yen et
//                        al.), this specific amplitude-load coupling is an
//                        engineering hypothesis, not a direct citation.
//                        Range pre-checked against agar's actual
//                        normalizedLoad scale (~70-80, see cpgLoadSensitivity
//                        above) so it isn't miscalibrated the same way
//                        cpgLoadSensitivity's first range was.
//
// Health gate: efficiency>=0.40, coiledRatio>=0.30, freqHz>kMinFreqHz (reject
// frozen-static-arc), maxAbsHeadingDelta<=kMaxHeadingDeltaRad - identical
// structure to every other axis this session, baked in from the START.
// Primary fitness: waterBLps > agarBLps with BOTH healthy - this IS the
// original crawl/swim target, not a side question. freqHz tracked
// separately for agar/water (freqSeparated flag, same convention as
// worm_bclass_oscillator_calibration) - the mechanistic claim is frequency
// itself should rise in the lower-drag medium, so speed-up without
// frequency separating the same way is flagged as suspicious.
//
// Multi-independent-seed-base evaluation from the first fitness call, this
// project's repeatedly-learned lesson, not bolted on at confirmation.
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

using Candidate = std::array<float, 7>;  // {cpgGain, cpgBaseFreqHz, cpgLoadSensitivity, bodyPoseDecayRate, muscleLeakScale, motorLeakScale, cpgAmpLoadSensitivity}
enum { kGain = 0, kFreq = 1, kSensitivity = 2, kPoseDecay = 3, kMuscleLeak = 4, kMotorLeak = 5, kAmpSensitivity = 6 };
const Candidate kIdentity = {0.0f, 2.0f, 0.05f, 0.5f, 0.0f, 1.0f, 0.0f};  // cpgGain=0 -> the rest are irrelevant (see header); motorLeakScale=1.0 is ITS identity (multiplier, not gain)

void applyCalibration(WormSim& sim, const Candidate& c) {
    sim.params.cpgGain = c[kGain];
    sim.params.cpgBaseFreqHz = c[kFreq];
    sim.params.cpgLoadSensitivity = c[kSensitivity];
    sim.params.cpgAmpLoadSensitivity = c[kAmpSensitivity];
    sim.params.cpgWavelengths = kFixedWavelengths;
    sim.params.bodyPoseDecayRate = c[kPoseDecay];
    // muscleLeakScale (см. tests/worm_muscle_body_joint_calibration за этой
    // же осью отдельно) - добавлено ПОСЛЕ первого скрина этого файла нашёл,
    // что без него впрыснутый CPG-сигнал тонет в собственной околонулевой
    // утечке Output-нейронов (мышечный слой попросту усредняет любой быстрый
    // вход в почти постоянную составляющую) - измеренная частота оставалась
    // ~0.01-0.02Гц независимо от cpgBaseFreqHz, пока это не нашлось.
    sim.params.muscleLeakScale = c[kMuscleLeak];
    // motorLeakScale - добавлено ПОСЛЕ того, как muscleLeakScale САМ ПО СЕБЕ
    // тоже не помог: мотонейроны (не мышцы) используют общий leakScale
    // (дефолт 1.0, ~0.16Гц полоса) и сглаживали сигнал ещё до мышц. МНОЖИТЕЛЬ,
    // не гейн - identity=1.0, не 0.0 (см. Params::motorLeakScale).
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

}  // namespace

int main(int argc, char** argv) {
    // ./exe trace gain freq sensitivity poseDecay muscleLeak motorLeak [steps] [seed]
    // Dumps raw dev[9]/dev[12]/dev[15] every step - direct visual check of
    // whether ANY fast oscillation reaches the final curvature signal at all,
    // instead of inferring it indirectly from zero-crossing-based freqHz.
    if (argc >= 2 && std::string(argv[1]) == "trace") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kFreq] = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) cand[kSensitivity] = static_cast<float>(std::atof(argv[4]));
        if (argc > 5) cand[kPoseDecay] = static_cast<float>(std::atof(argv[5]));
        if (argc > 6) cand[kMuscleLeak] = static_cast<float>(std::atof(argv[6]));
        if (argc > 7) cand[kMotorLeak] = static_cast<float>(std::atof(argv[7]));
        const int steps = argc > 8 ? std::atoi(argv[8]) : 100;
        const int seed = argc > 9 ? std::atoi(argv[9]) : 42;
        WormSim sim("worm_data/celegans_herm.connectome");
        applyCalibration(sim, cand);
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = kDragAgar;
        std::srand(static_cast<unsigned>(seed));
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        for (int i = 0; i < 300; ++i) sim.step();  // warmup

        // Find a couple of real motor neuron IDs by name to inspect their raw state directly.
        const auto& names = sim.neuronNames();
        int db1Id = -1, vb1Id = -1;
        for (std::size_t i = 0; i < names.size(); ++i) {
            if (names[i] == "DB1") db1Id = static_cast<int>(i);
            if (names[i] == "VB1") vb1Id = static_cast<int>(i);
        }
        std::printf("DB1 id=%d  VB1 id=%d\n", db1Id, vb1Id);
        std::printf("step  cpgPhase  DB1state  VB1state  dev[9]    dev[12]   dev[15]\n");
        for (int i = 0; i < steps; ++i) {
            sim.step();
            const auto& dev = sim.lastCurvatureDeviation();
            const float db1State = db1Id >= 0 ? sim.network().state(static_cast<connectome::NeuronId>(db1Id)) : 0.0f;
            const float vb1State = vb1Id >= 0 ? sim.network().state(static_cast<connectome::NeuronId>(vb1Id)) : 0.0f;
            std::printf("%4d  %+.4f   %+.5f  %+.5f  %+.5f  %+.5f  %+.5f\n", i, sim.debugCpgPhase(), db1State,
                        vb1State, 9 < static_cast<int>(dev.size()) ? dev[9] : 0.0f,
                        12 < static_cast<int>(dev.size()) ? dev[12] : 0.0f,
                        15 < static_cast<int>(dev.size()) ? dev[15] : 0.0f);
        }
        return 0;
    }

    // ./exe distribution gain freq sensitivity poseDecay muscleLeak motorLeak ampSensitivity [numBases] [seedsPerBase] [warmup] [measure]
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand = kIdentity;
        if (argc > 2) cand[kGain] = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) cand[kFreq] = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) cand[kSensitivity] = static_cast<float>(std::atof(argv[4]));
        if (argc > 5) cand[kPoseDecay] = static_cast<float>(std::atof(argv[5]));
        if (argc > 6) cand[kMuscleLeak] = static_cast<float>(std::atof(argv[6]));
        if (argc > 7) cand[kMotorLeak] = static_cast<float>(std::atof(argv[7]));
        if (argc > 8) cand[kAmpSensitivity] = static_cast<float>(std::atof(argv[8]));
        const int numBases = argc > 9 ? std::atoi(argv[9]) : 12;
        const int seedsPerBase = argc > 10 ? std::atoi(argv[10]) : 8;
        const int warmupSteps = argc > 11 ? std::atoi(argv[11]) : 300;
        const int measureSteps = argc > 12 ? std::atoi(argv[12]) : 2500;
        std::printf("cand=[gain=%.4f freq=%.4f sens=%.4f poseDecay=%.4f muscleLeak=%.4f motorLeak=%.4f "
                    "ampSens=%.4f] - %d bases x %d seeds\n",
                    cand[kGain], cand[kFreq], cand[kSensitivity], cand[kPoseDecay], cand[kMuscleLeak],
                    cand[kMotorLeak], cand[kAmpSensitivity], numBases, seedsPerBase);
        std::mt19937 baseRng(31337);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0, freqSeparatedCount = 0;
        double sumAgar = 0.0, sumWater = 0.0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(cand, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(cand, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool waterFaster = waterOk && agarOk && water.meanBLps > agar.meanBLps;
            const bool freqSeparated = waterOk && agarOk && water.meanFreqHz > agar.meanFreqHz;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  water>agar: %s   freq(water)>freq(agar): %s%s\n", waterFaster ? "YES" : "no",
                        freqSeparated ? "YES" : "no",
                        (waterFaster && !freqSeparated) ? "  <-- FLAG: speed up without freq separating" : "");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (waterFaster) ++waterBeatsAgarCount;
            if (freqSeparated) ++freqSeparatedCount;
        }
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s), water healthy=%d/%d (mean "
                    "%.5f BL/s), water>agar in %d/%d bases, freq(water)>freq(agar) in %d/%d bases\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    waterBeatsAgarCount, numBases, freqSeparatedCount, numBases);
        return 0;
    }

    // ./exe random <trials> <gainLo> <gainHi> <freqLo> <freqHi> <sensLo> <sensHi> <decayLo> <decayHi> <leakLo> <leakHi> <motorLo> <motorHi> [seedsPerTrial] [rngSeed]
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 400;
        const float gainLo = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.5f;
        const float gainHi = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 50.0f;
        const float freqLo = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.5f;
        const float freqHi = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 4.0f;
        const float sensLo = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.005f;
        const float sensHi = argc > 8 ? static_cast<float>(std::atof(argv[8])) : 2.0f;
        const float decayLo = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.1f;
        const float decayHi = argc > 10 ? static_cast<float>(std::atof(argv[10])) : 2.0f;
        const float leakLo = argc > 11 ? static_cast<float>(std::atof(argv[11])) : 0.02f;
        const float leakHi = argc > 12 ? static_cast<float>(std::atof(argv[12])) : 3.0f;
        const float motorLo = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 1.0f;
        const float motorHi = argc > 14 ? static_cast<float>(std::atof(argv[14])) : 100.0f;
        const int seedsPerTrial = argc > 15 ? std::atoi(argv[15]) : 6;
        const unsigned rngSeed = argc > 16 ? static_cast<unsigned>(std::atoi(argv[16])) : 1u;
        // ampSensitivity range fixed (not CLI-overridable, keeps the CLI from
        // growing further) - see the header's cpgAmpLoadSensitivity range note.
        constexpr float kAmpSensLo = 0.001f, kAmpSensHi = 0.2f;
        constexpr int kWarmup = 300, kMeasure = 2500;
        std::printf("random search: %d trials, gain log-uniform [%.2f,%.2f], freq log-uniform [%.2f,%.2f]Hz, "
                    "sensitivity log-uniform [%.2f,%.2f], poseDecay log-uniform [%.2f,%.2f], muscleLeak "
                    "log-uniform [%.2f,%.2f], motorLeak log-uniform [%.2f,%.2f], ampSensitivity log-uniform "
                    "[%.3f,%.3f], %d seeds/trial, rngSeed=%u\n",
                    trials, gainLo, gainHi, freqLo, freqHi, sensLo, sensHi, decayLo, decayHi, leakLo, leakHi,
                    motorLo, motorHi, kAmpSensLo, kAmpSensHi, seedsPerTrial, rngSeed);
        std::mt19937 rng(rngSeed);
        std::uniform_real_distribution<float> gainLogDist(std::log(gainLo), std::log(gainHi));
        std::uniform_real_distribution<float> freqLogDist(std::log(freqLo), std::log(freqHi));
        std::uniform_real_distribution<float> sensLogDist(std::log(sensLo), std::log(sensHi));
        std::uniform_real_distribution<float> decayLogDist(std::log(decayLo), std::log(decayHi));
        std::uniform_real_distribution<float> leakLogDist(std::log(leakLo), std::log(leakHi));
        std::uniform_real_distribution<float> motorLogDist(std::log(motorLo), std::log(motorHi));
        std::uniform_real_distribution<float> ampSensLogDist(std::log(kAmpSensLo), std::log(kAmpSensHi));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int found = 0;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            cand[kGain] = std::exp(gainLogDist(rng));
            cand[kFreq] = std::exp(freqLogDist(rng));
            cand[kSensitivity] = std::exp(sensLogDist(rng));
            cand[kPoseDecay] = std::exp(decayLogDist(rng));
            cand[kMuscleLeak] = std::exp(leakLogDist(rng));
            cand[kMotorLeak] = std::exp(motorLogDist(rng));
            cand[kAmpSensitivity] = std::exp(ampSensLogDist(rng));
            const int base = baseDist(rng);
            const AggregateResult agar = evaluate(cand, seedsPerTrial, base, kDragAgar, kWarmup, kMeasure);
            const bool agarOk = isHealthy(agar);
            if (!agarOk) continue;
            const AggregateResult water = evaluate(cand, seedsPerTrial, base, kDragWater, kWarmup, kMeasure);
            const bool waterOk = isHealthy(water);
            const bool waterFaster = waterOk && water.meanBLps > agar.meanBLps;
            if (waterFaster) {
                ++found;
                const bool freqSeparated = water.meanFreqHz > agar.meanFreqHz;
                std::printf("[gain=%.3f freq=%.3f sens=%.3f poseDecay=%.3f muscleLeak=%.3f motorLeak=%.3f "
                            "ampSens=%.4f] base=%d ",
                            cand[kGain], cand[kFreq], cand[kSensitivity], cand[kPoseDecay], cand[kMuscleLeak],
                            cand[kMotorLeak], cand[kAmpSensitivity], base);
                printAgg(" agar", agar);
                printAgg(" water", water);
                std::printf("  freqSeparated=%s\n", freqSeparated ? "yes" : "NO (flag)");
                std::fflush(stdout);
            }
        }
        std::printf("\n%d/%d trials had a healthy candidate with water>agar on its OWN (single, fresh) base - each "
                    "one still needs 'distribution' confirmation across many MORE independent bases before it "
                    "means anything (see header).\n",
                    found, trials);
        return 0;
    }

    // Default: quick screen - vary cpgGain and cpgBaseFreqHz on a small grid,
    // sensitivity/poseDecay/muscleLeak/motorLeak held fixed, one seed base
    // (shortlist only - confirm anything promising with 'distribution' or
    // search jointly with 'random' across many bases before trusting it).
    // muscleLeak=0.2, motorLeak=20.0 - NOT the Params defaults (0.0/1.0):
    // this screen's own history found the injected CPG signal never showed
    // up as measured frequency at all with either left at its default - the
    // signal passes through TWO slow-integrating relay layers before
    // reaching curvature (motor neurons, then muscles), each needing its own
    // leak raised well past its Params default to have enough bandwidth to
    // track a 1-4Hz drive at all. motorLeak=20 targets a ~3.2Hz bandwidth
    // ceiling (k/(2*pi)) for motor neurons specifically.
    // sensitivity=0.05, NOT 5.0 - see cpgLoadSensitivity's corrected range
    // above: normalizedLoad on agar is ~70-80, not O(1), so 5.0 suppressed
    // frequency by ~400x (confirmed via direct cpgPhase tracing) - this is
    // why earlier screens showed no cpgBaseFreqHz effect at all.
    constexpr int kScreenSeeds = 6;
    constexpr int kSeedBase = 42;
    constexpr float kScreenMuscleLeak = 0.2f;
    constexpr float kScreenMotorLeak = 20.0f;
    constexpr float kScreenSensitivity = 0.05f;
    constexpr float kScreenAmpSensitivity = 0.02f;  // see Params::cpgAmpLoadSensitivity - amplitude grows with load
    const float kGains[] = {1.0f, 3.0f, 8.0f, 15.0f, 30.0f};
    const float kFreqs[] = {1.0f, 2.0f, 3.0f};
    std::printf("=== Screen (cpgGain x cpgBaseFreqHz grid, sensitivity=%.2f/ampSens=%.2f/poseDecay=0.5/"
                "muscleLeak=%.1f/motorLeak=%.1f fixed, %d seeds/point, base=%d) ===\n",
                kScreenSensitivity, kScreenAmpSensitivity, kScreenMuscleLeak, kScreenMotorLeak, kScreenSeeds,
                kSeedBase);
    for (float g : kGains) {
        for (float f : kFreqs) {
            const Candidate cand = {g, f, kScreenSensitivity, 0.5f, kScreenMuscleLeak, kScreenMotorLeak,
                                     kScreenAmpSensitivity};
            const AggregateResult agar = evaluate(cand, kScreenSeeds, kSeedBase, kDragAgar, 300, 2500);
            const AggregateResult water = evaluate(cand, kScreenSeeds, kSeedBase, kDragWater, 300, 2500);
            std::printf("gain=%.1f freq=%.1f:\n", g, f);
            printAgg("agar", agar);
            printAgg("water", water);
            const bool waterFaster = isHealthy(agar) && isHealthy(water) && water.meanBLps > agar.meanBLps;
            std::printf("  water>agar: %s\n", waterFaster ? "YES" : "no");
            std::fflush(stdout);
        }
    }
    std::printf("\nNo single-base result above is trustworthy on its own - confirm any promising point with "
                "'distribution' or search jointly with 'random' across many seed bases before believing it (see "
                "header).\n");
    return 0;
}
