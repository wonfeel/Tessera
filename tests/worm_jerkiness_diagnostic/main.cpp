// tests/worm_jerkiness_diagnostic/main.cpp
//
// One-off investigation, triggered by a live report ("moves abruptly now")
// after today's session shipped proprioceptiveDelaySeconds=1.596 alongside
// a higher cpgGain/motorBandwidthGain. Time-delayed feedback is a classic
// source of reduced stability margin / growing oscillation in control
// theory - worth checking directly, since every calibration health gate
// this session only ever measured a 125-simulated-second window (2500 steps
// at dt=0.05), never anything long enough to see a SLOW-building instability.
//
// This harness runs MUCH longer (20000 steps = 1000s) at both the OLD
// (pre-today) and NEW (today's) production points, on both media, and
// reports per-window statistics (not just a single whole-run max) so a
// growing-over-time problem is visible even if the whole-run max alone
// looks acceptable.
//
// RESULT: a real, PRE-EXISTING instability - confirmed present in BOTH the
// OLD (pre-today) and NEW (today's) production points, on agar, at roughly
// the same rate (~2 events per 1000s in both). NOT introduced by today's
// changes (proprioceptiveDelaySeconds, higher cpgGain/motorBandwidthGain).
//
// Windowed run (1000s, 100s windows, seed=42): whole-run max |heading delta|
// on agar was 1.36 rad (OLD) and 1.21 rad (NEW) - both far past the "normal"
// baseline (~0.001-0.01 rad/step) - with sporadic, non-monotonic spikes in
// specific 100s windows, not a slow drift. Water showed NO such spikes in
// either point (max 0.06-0.11 rad throughout) - this is agar/high-load
// specific.
//
// Traced (trace mode) one such event on each point - IDENTICAL signature:
// curvature deviation (sim.lastCurvatureDeviation()) grows from its normal
// range (sumAbsDev ~5-8 across 24 positions) to ~30-50x that within 2-3
// steps (sumAbsDev 91-250+), heading delta spikes to 0.4-1.0 rad in the
// same window, then the system self-stabilizes at an elevated-but-bounded
// plateau within ~5-10 steps (does NOT diverge to NaN in either trace, nor
// in the full 1000s windowed runs on any of the 4 point/medium
// combinations). Both traces show the SAME lead-up pattern: deviation range
// and same-sign-position count both start shifting several steps before the
// visible heading spike, then escalate rapidly.
//
// Not root-caused to a specific line of code - the leading hypothesis,
// consistent with this project's own repeatedly-documented finding that the
// raw, uncalibrated 401-neuron network is genuinely chaotic (not merely
// noisy - see tests/worm_locomotion's header, "states climb toward a large
// self-sustained equilibrium... regardless of external input"), is that
// this is an occasional large excursion intrinsic to that chaotic dynamics,
// specifically excitable under agar's high mechanical load - not a specific
// off-by-one or sign error introduced by any one commit. Never caught by
// any calibration test this session OR historically because every one of
// them (including tests/worm_locomotion's own 150s regression window) uses
// a measurement window far shorter than this event's ~300-500s recurrence
// interval - a 125s-or-shorter window has a real chance of never sampling
// one at all, which is exactly what happened across all of today's 16-base
// distribution confirmations.
//
// PARTIALLY FIXED (later, by direct user request "почини"). WormSim.cpp now
// clamps each position's deviation to [-2,2] before it reaches WormBody -
// same "cheap insurance" philosophy as applyProprioception's existing
// feedback clamp. Verified effect (same seed=42, before/after): whole-run
// max |heading delta| on agar dropped from 1.3586->0.5287 rad (OLD point)
// and 1.2055->0.3668 rad (NEW point) - roughly a 55-70% reduction in worst-
// case severity. Shipped production point reconfirmed unchanged (16/16
// healthy, identical agar/water speed+freq+ratio to before the clamp) -
// the clamp is inert under normal operation, only engages during the rare
// excursion.
//
// Does NOT fully eliminate the jerk - trace data explains why: during the
// excursion, 15-17 of 24 body positions simultaneously saturate the SAME
// clamp ceiling at once (not one or two outliers), so the aggregate signal
// reaching the body is still large even with each individual value capped.
//
// A STRONGER, two-stage version was tried (per-element clamp PLUS an
// aggregate cap on sum(|deviation|) across all 24 positions, proportionally
// scaling the whole vector down when the sum exceeded 15.0) and REVERTED -
// it froze agar locomotion completely (freq=0.0000Hz, coiled~0.71-0.72,
// unhealthy) on all 16/16 confirmation bases. Root cause: normal agar
// operation at today's shipped point (wider motorBandwidthGain than the
// mechanism this clamp was designed against) already rides a sum-of-
// deviation baseline close to ~9 during ordinary wave amplitude modulation -
// a ceiling of 15 was close enough to that normal peak that it chronically
// suppressed the oscillation itself, not just the rare pathological burst.
// Lesson: this specific network's normal and pathological operating ranges
// are close enough on the AGGREGATE signal that a global sum-based clamp is
// unsafe without much more careful per-point tuning (or a smarter trigger
// that distinguishes "many positions moving together as a coherent healthy
// wave" from "many positions all saturating the same ceiling at once") -
// not attempted further here given the real risk of silently breaking
// production again.
//
// Honest summary: the jerk is reduced, not solved. A ~0.37-0.53 rad (21-30
// degree) single-step heading change can still occur roughly once per
// 300-500 simulated seconds on agar. Root cause (why the raw network
// occasionally bursts) remains uninvestigated.
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
#include <string>
#include <vector>

namespace {

constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kHexSpacing = 36.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kBodyLength = 576.0f;

struct Point {
    float cpgGain, muscleBw, motorBw, delaySec;
    const char* label;
};

void runOne(const Point& pt, float dragNormal, const char* mediumLabel, int seed, int totalSteps, int windowSteps) {
    WormSim sim("worm_data/celegans_herm.connectome");
    sim.params.cpgGain = pt.cpgGain;
    sim.params.cpgBaseFreqHz = 1.108f;
    sim.params.cpgLoadSensitivity = 0.02f;
    sim.params.cpgAmpLoadSensitivity = 0.0342f;
    sim.params.cpgWavelengths = 1.0f;
    sim.params.bodyPoseDecayRate = 1.051f;
    sim.params.muscleBandwidthGain = pt.muscleBw;
    sim.params.motorBandwidthGain = pt.motorBw;
    sim.params.proprioceptiveDelaySeconds = pt.delaySec;
    sim.params.dragTangent = 1.0f;
    sim.params.dragNormal = dragNormal;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
    const float dt = sim.params.dt.load();

    WormSim::Snapshot snap;
    float prevHeading = 0.0f;
    bool havePrevHeading = false;

    std::vector<float> windowMaxDelta;
    std::vector<float> windowMeanAbsDelta;
    std::vector<float> windowCoiledMin;
    float curWindowMax = 0.0f;
    double curWindowSum = 0.0;
    int curWindowCount = 0;
    float curWindowCoiledMin = 1e9f;

    long long over01 = 0, over02 = 0, over03 = 0, over04 = 0;
    float wholeRunMax = 0.0f;
    bool anyNaN = false;

    for (int i = 0; i < totalSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float headX = snap.pointsX[0], headY = snap.pointsY[0];
        if (std::isnan(headX) || std::isnan(headY)) { anyNaN = true; break; }
        const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            const float absHd = std::fabs(hd);
            curWindowMax = std::max(curWindowMax, absHd);
            curWindowSum += absHd;
            ++curWindowCount;
            wholeRunMax = std::max(wholeRunMax, absHd);
            if (absHd > 0.1f) ++over01;
            if (absHd > 0.2f) ++over02;
            if (absHd > 0.3f) ++over03;
            if (absHd > 0.4f) ++over04;
        }
        prevHeading = heading;
        havePrevHeading = true;

        if (i % 20 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
            for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            curWindowCoiledMin = std::min(curWindowCoiledMin, diag / kBodyLength);
        }

        if ((i + 1) % windowSteps == 0) {
            windowMaxDelta.push_back(curWindowMax);
            windowMeanAbsDelta.push_back(curWindowCount > 0 ? static_cast<float>(curWindowSum / curWindowCount) : 0.0f);
            windowCoiledMin.push_back(curWindowCoiledMin);
            curWindowMax = 0.0f; curWindowSum = 0.0; curWindowCount = 0; curWindowCoiledMin = 1e9f;
        }
    }

    std::printf("--- %s / %s (cpgGain=%.3f muscleBw=%.6f motorBw=%.5f delaySec=%.3f) seed=%d ---\n", pt.label,
                mediumLabel, pt.cpgGain, pt.muscleBw, pt.motorBw, pt.delaySec, seed);
    if (anyNaN) { std::printf("  FAIL: went NaN before completing %d steps\n", totalSteps); return; }
    std::printf("  whole-run max |heading delta|: %.4f rad\n", wholeRunMax);
    std::printf("  fraction of steps with |delta|>0.1/0.2/0.3/0.4 rad: %.4f%% / %.4f%% / %.4f%% / %.4f%%\n",
                100.0 * static_cast<double>(over01) / (totalSteps - 1), 100.0 * static_cast<double>(over02) / (totalSteps - 1),
                100.0 * static_cast<double>(over03) / (totalSteps - 1), 100.0 * static_cast<double>(over04) / (totalSteps - 1));
    std::printf("  per-window (%.0fs each) max|delta| trend: ", windowSteps * dt);
    for (float v : windowMaxDelta) std::printf("%.3f ", v);
    std::printf("\n  per-window mean|delta| trend:              ");
    for (float v : windowMeanAbsDelta) std::printf("%.4f ", v);
    std::printf("\n  per-window min coiled-ratio trend:          ");
    for (float v : windowCoiledMin) std::printf("%.3f ", v);
    std::printf("\n");
    std::fflush(stdout);
}

}  // namespace

void traceSpike(const Point& pt, float dragNormal, const char* mediumLabel, int seed, int totalSteps,
                 float spikeThreshold, float dtOverride = 0.0f, float velocityClamp = 0.0f) {
    WormSim sim("worm_data/celegans_herm.connectome");
    sim.params.cpgGain = pt.cpgGain;
    sim.params.cpgBaseFreqHz = 1.108f;
    sim.params.cpgLoadSensitivity = 0.02f;
    sim.params.cpgAmpLoadSensitivity = 0.0342f;
    sim.params.cpgWavelengths = 1.0f;
    sim.params.bodyPoseDecayRate = 1.051f;
    sim.params.muscleBandwidthGain = pt.muscleBw;
    sim.params.motorBandwidthGain = pt.motorBw;
    sim.params.proprioceptiveDelaySeconds = pt.delaySec;
    sim.params.dragTangent = 1.0f;
    sim.params.dragNormal = dragNormal;
    sim.params.propulsionVelocityClamp = velocityClamp;
    // dtOverride>0 - для проверки гипотезы "это численный артефакт
    // интегратора": если выброс - следствие слишком грубого dt относительно
    // скорости нарастания рекуррентной связи (химсинапсы+gap junctions
    // считаются явно, с лагом в один шаг - см. Network::step), уменьшение dt
    // (при том же общем симулированном времени, totalSteps масштабируется
    // вызывающим кодом) должно снизить частоту/силу всплесков. Если это
    // настоящая динамика (не артефакт дискретизации) - не должно.
    if (dtOverride > 1e-6f) sim.params.dt = dtOverride;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);

    WormSim::Snapshot snap;
    float prevHeading = 0.0f;
    bool havePrevHeading = false;
    std::vector<std::string> ring;
    constexpr int kRingSize = 15;

    std::printf("--- TRACE %s / %s (looking for first |delta|>%.2f rad) ---\n", pt.label, mediumLabel, spikeThreshold);
    for (int i = 0; i < totalSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float headX = snap.pointsX[0], headY = snap.pointsY[0];
        if (std::isnan(headX) || std::isnan(headY)) { std::printf("  NaN at step %d\n", i); return; }
        const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
        float absHd = 0.0f;
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            absHd = std::fabs(hd);
        }
        prevHeading = heading;
        havePrevHeading = true;

        const auto& dev = sim.lastCurvatureDeviation();
        float minDev = 1e9f, maxDev = -1e9f, sumAbsDev = 0.0f;
        int sameSignCount = 0, posCount = 0, negCount = 0;
        for (float d : dev) {
            minDev = std::min(minDev, d); maxDev = std::max(maxDev, d); sumAbsDev += std::fabs(d);
            if (d > 0.001f) ++posCount; else if (d < -0.001f) ++negCount;
        }
        sameSignCount = std::max(posCount, negCount);
        float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
        for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
        for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
        const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
        const float coiled = diag / kBodyLength;
        const float load = sim.debugMechanicalLoad();
        const float det = sim.debugPropulsionDeterminant();
        const float rhs = sim.debugPropulsionRhsMagnitude();

        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "  step=%6d |hd|=%.4f coiled=%.3f devRange=[%.3f,%.3f] sumAbsDev=%.2f sameSignPositions=%d/%d "
                      "load=%.3f det=%.6g rhs=%.3f",
                      i, absHd, coiled, minDev, maxDev, sumAbsDev, sameSignCount, static_cast<int>(dev.size()), load,
                      det, rhs);
        ring.emplace_back(buf);
        if (static_cast<int>(ring.size()) > kRingSize) ring.erase(ring.begin());

        if (absHd > spikeThreshold) {
            std::printf("  SPIKE at step %d (|hd|=%.4f rad) - last %d steps leading up to it:\n", i, absHd,
                        static_cast<int>(ring.size()));
            for (const auto& line : ring) std::printf("%s\n", line.c_str());
            // Print a few steps AFTER too.
            for (int k = 0; k < 5 && i + 1 < totalSteps; ++k) {
                sim.step();
                sim.snapshot(snap);
                const auto& dev2 = sim.lastCurvatureDeviation();
                float minDev2 = 1e9f, maxDev2 = -1e9f, sumAbsDev2 = 0.0f;
                for (float d : dev2) { minDev2 = std::min(minDev2, d); maxDev2 = std::max(maxDev2, d); sumAbsDev2 += std::fabs(d); }
                float bx0b = 1e9f, bx1b = -1e9f, by0b = 1e9f, by1b = -1e9f;
                for (float px : snap.pointsX) { bx0b = std::min(bx0b, px); bx1b = std::max(bx1b, px); }
                for (float py : snap.pointsY) { by0b = std::min(by0b, py); by1b = std::max(by1b, py); }
                const float diag2 = std::sqrt((bx1b - bx0b) * (bx1b - bx0b) + (by1b - by0b) * (by1b - by0b));
                std::printf("  step=%6d (+%d after spike) coiled=%.3f devRange=[%.3f,%.3f] sumAbsDev=%.2f load=%.3f "
                            "det=%.6g rhs=%.3f\n",
                            i + 1 + k, k + 1, diag2 / kBodyLength, minDev2, maxDev2, sumAbsDev2,
                            sim.debugMechanicalLoad(), sim.debugPropulsionDeterminant(),
                            sim.debugPropulsionRhsMagnitude());
                ++i;
            }
            std::fflush(stdout);
            return;
        }
    }
    std::printf("  no spike >%.2f rad found in %d steps\n", spikeThreshold, totalSteps);
}

int main(int argc, char** argv) {
    if (argc >= 2 && std::string(argv[1]) == "trace") {
        const int totalSteps = argc > 2 ? std::atoi(argv[2]) : 20000;
        const int seed = argc > 3 ? std::atoi(argv[3]) : 42;
        const float threshold = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 0.4f;
        const bool useOld = argc > 5 && std::string(argv[5]) == "old";
        const float dtOverride = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 0.0f;
        const float velocityClamp = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.0f;
        const Point kOld = {1.901f, 0.000249f, 0.00449f, 0.0f, "OLD(pre-today)"};
        const Point kNew = {2.621f, 0.000153f, 0.0194f, 1.596f, "NEW(shipped-today)"};
        traceSpike(useOld ? kOld : kNew, 40.0f, "agar", seed, totalSteps, threshold, dtOverride, velocityClamp);
        return 0;
    }

    // ./exe dtsweep [simSeconds] [threshold] [seed1] [seed2] ... - numerical-
    // artifact-vs-real-dynamics test (see Params::proprioceptiveFilterTau's
    // neighbor discussion / user request "думай"): runs the SAME simulated
    // duration at several dt values and counts how many spikes >threshold
    // occur. If the burst is a discretization artifact of the explicit,
    // one-step-lagged chemical/gap coupling (Network::step), a finer dt
    // should show FEWER/SMALLER spikes per simulated second. If it is a real
    // property of the continuous dynamics, spike rate should stay roughly
    // constant across dt.
    if (argc >= 2 && std::string(argv[1]) == "dtsweep") {
        const float simSeconds = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 500.0f;
        const float threshold = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.2f;
        std::vector<int> seeds;
        for (int k = 4; k < argc; ++k) seeds.push_back(std::atoi(argv[k]));
        if (seeds.empty()) seeds = {42, 43, 44};
        const Point kNew = {2.621f, 0.000153f, 0.0194f, 1.596f, "NEW(shipped-today)"};
        const float dts[] = {0.05f, 0.025f, 0.0125f, 0.00625f};
        std::printf("=== dt-sensitivity sweep: %.0fs simulated, threshold=%.2f rad, %zu seeds ===\n", simSeconds,
                    threshold, seeds.size());
        for (float dt : dts) {
            const int totalSteps = static_cast<int>(simSeconds / dt);
            long long totalOverThresh = 0;
            float worstMax = 0.0f;
            bool anyNaN = false;
            for (int seed : seeds) {
                WormSim sim("worm_data/celegans_herm.connectome");
                sim.params.cpgGain = kNew.cpgGain;
                sim.params.cpgBaseFreqHz = 1.108f;
                sim.params.cpgLoadSensitivity = 0.02f;
                sim.params.cpgAmpLoadSensitivity = 0.0342f;
                sim.params.cpgWavelengths = 1.0f;
                sim.params.bodyPoseDecayRate = 1.051f;
                sim.params.muscleBandwidthGain = kNew.muscleBw;
                sim.params.motorBandwidthGain = kNew.motorBw;
                sim.params.proprioceptiveDelaySeconds = kNew.delaySec;
                sim.params.dragTangent = 1.0f;
                sim.params.dragNormal = 40.0f;
                sim.params.dt = dt;
                std::srand(static_cast<unsigned>(seed));
                sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
                WormSim::Snapshot snap;
                float prevHeading = 0.0f;
                bool havePrevHeading = false;
                for (int i = 0; i < totalSteps; ++i) {
                    sim.step();
                    sim.snapshot(snap);
                    const float headX = snap.pointsX[0], headY = snap.pointsY[0];
                    if (std::isnan(headX) || std::isnan(headY)) { anyNaN = true; break; }
                    const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
                    if (havePrevHeading) {
                        float hd = heading - prevHeading;
                        while (hd > kPi) hd -= 2.0f * kPi;
                        while (hd < -kPi) hd += 2.0f * kPi;
                        const float absHd = std::fabs(hd);
                        worstMax = std::max(worstMax, absHd);
                        if (absHd > threshold) ++totalOverThresh;
                    }
                    prevHeading = heading;
                    havePrevHeading = true;
                }
            }
            std::printf("dt=%.5f (%d steps/seed): worst max|delta|=%.4f, total steps>%.2frad across %zu seeds=%lld%s\n",
                        dt, totalSteps, worstMax, threshold, seeds.size(), totalOverThresh, anyNaN ? " [NaN]" : "");
            std::fflush(stdout);
        }
        return 0;
    }

    // ./exe vclampsweep [simSeconds] [threshold] [clamp1] [clamp2] ... - sweep
    // Params::propulsionVelocityClamp (WormBody::set_max_shape_velocity).
    // For each candidate: (a) long-run worst max|heading delta| across
    // several seeds on agar (the burst-prone medium), (b) short-window
    // health/efficiency/frequency on BOTH agar and water, to catch a value
    // that kills the burst by also killing normal locomotion (same failure
    // mode as the reverted aggregate-deviation-clamp attempt).
    if (argc >= 2 && std::string(argv[1]) == "vclampsweep") {
        const float simSeconds = argc > 2 ? static_cast<float>(std::atof(argv[2])) : 500.0f;
        const float threshold = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.15f;
        std::vector<float> clamps;
        for (int k = 4; k < argc; ++k) clamps.push_back(static_cast<float>(std::atof(argv[k])));
        if (clamps.empty()) clamps = {0.0f, 500.0f, 200.0f, 100.0f, 50.0f, 20.0f, 10.0f};
        const Point kNew = {2.621f, 0.000153f, 0.0194f, 1.596f, "NEW(shipped-today)"};
        const int jerkSeeds[] = {42, 43, 44, 45, 100};
        constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;
        std::printf("=== velocity-clamp sweep: %.0fs simulated jerk check, threshold=%.2f rad ===\n", simSeconds,
                    threshold);
        for (float clamp : clamps) {
            const int totalSteps = static_cast<int>(simSeconds / 0.05f);
            long long totalOverThresh = 0;
            float worstMax = 0.0f;
            bool anyNaN = false;
            for (int seed : jerkSeeds) {
                WormSim sim("worm_data/celegans_herm.connectome");
                sim.params.cpgGain = kNew.cpgGain;
                sim.params.cpgBaseFreqHz = 1.108f;
                sim.params.cpgLoadSensitivity = 0.02f;
                sim.params.cpgAmpLoadSensitivity = 0.0342f;
                sim.params.cpgWavelengths = 1.0f;
                sim.params.bodyPoseDecayRate = 1.051f;
                sim.params.muscleBandwidthGain = kNew.muscleBw;
                sim.params.motorBandwidthGain = kNew.motorBw;
                sim.params.proprioceptiveDelaySeconds = kNew.delaySec;
                sim.params.dragTangent = 1.0f;
                sim.params.dragNormal = kDragAgar;
                sim.params.propulsionVelocityClamp = clamp;
                std::srand(static_cast<unsigned>(seed));
                sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
                WormSim::Snapshot snap;
                float prevHeading = 0.0f;
                bool havePrevHeading = false;
                for (int i = 0; i < totalSteps; ++i) {
                    sim.step();
                    sim.snapshot(snap);
                    const float headX = snap.pointsX[0], headY = snap.pointsY[0];
                    if (std::isnan(headX) || std::isnan(headY)) { anyNaN = true; break; }
                    const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
                    if (havePrevHeading) {
                        float hd = heading - prevHeading;
                        while (hd > kPi) hd -= 2.0f * kPi;
                        while (hd < -kPi) hd += 2.0f * kPi;
                        const float absHd = std::fabs(hd);
                        worstMax = std::max(worstMax, absHd);
                        if (absHd > threshold) ++totalOverThresh;
                    }
                    prevHeading = heading;
                    havePrevHeading = true;
                }
            }

            // Short-window health/ratio on both media (300 warmup + 2500 measure, 8 seeds, same as other calibration files).
            auto quickHealth = [&](float dragNormal, float& outEff, float& outFreq, float& outCoiled,
                                   bool& outHealthy) {
                WormSim sim("worm_data/celegans_herm.connectome");
                sim.params.cpgGain = kNew.cpgGain;
                sim.params.cpgBaseFreqHz = 1.108f;
                sim.params.cpgLoadSensitivity = 0.02f;
                sim.params.cpgAmpLoadSensitivity = 0.0342f;
                sim.params.cpgWavelengths = 1.0f;
                sim.params.bodyPoseDecayRate = 1.051f;
                sim.params.muscleBandwidthGain = kNew.muscleBw;
                sim.params.motorBandwidthGain = kNew.motorBw;
                sim.params.proprioceptiveDelaySeconds = kNew.delaySec;
                sim.params.dragTangent = 1.0f;
                sim.params.dragNormal = dragNormal;
                sim.params.propulsionVelocityClamp = clamp;
                std::srand(42);
                sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
                for (int i = 0; i < 300; ++i) sim.step();
                WormSim::Snapshot snap;
                sim.snapshot(snap);
                float cx = 0, cy = 0;
                for (std::size_t i = 0; i < snap.pointsX.size(); ++i) { cx += snap.pointsX[i]; cy += snap.pointsY[i]; }
                cx /= snap.pointsX.size(); cy /= snap.pointsX.size();
                const float startX = cx, startY = cy;
                double pathLen = 0.0;
                float minCoiled = 1e9f;
                int zeroCrossings = 0;
                float prevDev = 0.0f;
                bool haveDev = false;
                for (int i = 0; i < 2500; ++i) {
                    sim.step();
                    sim.snapshot(snap);
                    float x = 0, y = 0;
                    for (std::size_t j = 0; j < snap.pointsX.size(); ++j) { x += snap.pointsX[j]; y += snap.pointsY[j]; }
                    x /= snap.pointsX.size(); y /= snap.pointsX.size();
                    pathLen += std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
                    cx = x; cy = y;
                    const auto& dev = sim.lastCurvatureDeviation();
                    if (dev.size() > 12) {
                        const float d = dev[12];
                        if (haveDev && ((d > 0.0f) != (prevDev > 0.0f))) ++zeroCrossings;
                        prevDev = d; haveDev = true;
                    }
                    if (i % 50 == 0) {
                        float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
                        for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
                        for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
                        minCoiled = std::min(minCoiled, std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0)) / kBodyLength);
                    }
                }
                const float netDisp = std::sqrt((cx - startX) * (cx - startX) + (cy - startY) * (cy - startY));
                outEff = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
                outFreq = static_cast<float>(zeroCrossings) / 2.0f / (2500.0f * 0.05f);
                outCoiled = minCoiled;
                outHealthy = outEff >= 0.40f && outCoiled >= 0.30f && outFreq >= 0.001f;
            };
            float agarEff, agarFreq, agarCoiled, waterEff, waterFreq, waterCoiled;
            bool agarHealthy, waterHealthy;
            quickHealth(kDragAgar, agarEff, agarFreq, agarCoiled, agarHealthy);
            quickHealth(kDragWater, waterEff, waterFreq, waterCoiled, waterHealthy);

            std::printf("clamp=%.1f: jerk(%.0fsx%zu seeds) worst max|delta|=%.4f over%.2f=%lld%s | "
                        "agar eff=%.3f freq=%.4f coiled=%.3f healthy=%s | water eff=%.3f freq=%.4f coiled=%.3f healthy=%s\n",
                        clamp, simSeconds, sizeof(jerkSeeds) / sizeof(int), worstMax, threshold, totalOverThresh,
                        anyNaN ? " [NaN]" : "", agarEff, agarFreq, agarCoiled, agarHealthy ? "yes" : "NO", waterEff,
                        waterFreq, waterCoiled, waterHealthy ? "yes" : "NO");
            std::fflush(stdout);
        }
        return 0;
    }

    const int totalSteps = argc > 1 ? std::atoi(argv[1]) : 20000;   // 1000s at dt=0.05
    const int windowSteps = argc > 2 ? std::atoi(argv[2]) : 2000;   // 100s windows
    const int seed = argc > 3 ? std::atoi(argv[3]) : 42;
    constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;

    const Point kOld = {1.901f, 0.000249f, 0.00449f, 0.0f, "OLD(pre-today)"};
    const Point kNew = {2.621f, 0.000153f, 0.0194f, 1.596f, "NEW(shipped-today)"};

    std::printf("=== Long-run jerkiness diagnostic: %d steps (%.0fs), %d-step (%.0fs) windows, seed=%d ===\n",
                totalSteps, totalSteps * 0.05f, windowSteps, windowSteps * 0.05f, seed);

    runOne(kOld, kDragAgar, "agar", seed, totalSteps, windowSteps);
    runOne(kOld, kDragWater, "water", seed, totalSteps, windowSteps);
    runOne(kNew, kDragAgar, "agar", seed, totalSteps, windowSteps);
    runOne(kNew, kDragWater, "water", seed, totalSteps, windowSteps);
    return 0;
}
