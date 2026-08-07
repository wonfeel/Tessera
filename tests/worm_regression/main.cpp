// tests/worm_regression/main.cpp
//
// Normal, CTest-registered regression test for the worm simulation - no CLI
// arguments, no manual "run the exe, grep Summary, eyeball the numbers"
// ritual. Validates the CURRENT SHIPPED WormSim::Params defaults (default-
// constructed, zero overrides) on both media across multiple independent
// seed bases, and returns a normal process exit code (0 = PASS, 1 = FAIL)
// so `ctest` (or any CI) reports it like any other test - see
// CMakeLists.txt's add_test() for this target.
//
// This exists because every honest measurement this session
// (WORM_V2_RESULTS.md) was done by hand: build, invoke tests/worm_v2_
// measurement with 8-16 explicit CLI floats, grep "Summary", read numbers
// off the screen. Fine for exploratory calibration, not a real regression
// test - nothing caught a future change quietly breaking the shipped point.
// This file always measures the ACTUAL live Params{} defaults - it can't
// silently drift out of sync with what's shipped, unlike a hand-typed CLI
// invocation someone has to remember to keep updated.
//
// IMPORTANT DESIGN NOTE (see WORM_V2_RESULTS.md section 17 for the full
// story): an earlier version of this file bundled path-efficiency into the
// same "healthy" bool as coiling/freezing/NaN and required only 70% of
// trials to pass - a real design mistake, caught by direct question ("the
// worm should survive 100% of the time"). Diagnosing WHY trials failed
// (per-trial failure-reason logging, not just a pass count) across 180
// trials found EVERY SINGLE failure was low path-efficiency - zero coiling,
// zero freezing, zero implausible heading whips, zero silent-network
// (non-oscillating) failures, across all 180. So SURVIVAL (the body doesn't
// break, freeze, spin implausibly, or go silent) genuinely IS 100% reliable
// on this stack, and is tested here as a hard, zero-tolerance gate, exactly
// as it should be.
//
// Path efficiency (net centroid displacement / path length) is a DIFFERENT
// kind of quantity and is deliberately NOT part of the survival gate. This
// project has confirmed, repeatedly (WORM_V2_RESULTS.md section 15,
// WORM.md section 6), that the worm has NO directional drive of any kind -
// chemotaxis is measurably absent. With zero directional bias, the worm's
// path across the plane is an undirected, noise-driven walk; how much any
// given 125-simulated-second window happens to loop or double back versus
// travel in a fairly straight line is a property of where the random walk
// happened to wander, not of whether the simulation is "broken." Demanding
// 100% of random seeds produce a straight-ish path would be demanding
// something structurally impossible for an undirected walker, not
// verifying correctness. So this file reports path-efficiency as a
// MEASURED STATISTIC (mean, distribution) with its own separate, honestly-
// calibrated regression floor (comparable to what's actually been observed
// across this whole session, not an arbitrary "must always pass" bar) -
// never silently merged into "is the worm alive."
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/body.cpp"
#include "demo/worm/connectome/loader.cpp"
#include "demo/worm/WormSim.cpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

constexpr float kBodyLength = 576.0f;
constexpr int kFieldCols = 200, kFieldRows = 150;
constexpr float kHexSpacing = 36.0f;
constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;
constexpr float kPi = 3.14159265358979323846f;

// SURVIVAL gates - confirmed 100%-reliable across 180 trials (see header).
// Any failure here on a future run is a genuine regression, not noise.
constexpr float kMinCoiledRatio = 0.30f;   // coiled into a tight knot
constexpr float kMinShapeRange = 0.02f;    // frozen straight, no bending wave at all
constexpr float kMaxHeadingDeltaRad = 0.5f;  // implausible single-step whip
constexpr float kMinFreqHz = 0.001f;       // not oscillating at all (silent network)

// LOCOMOTION QUALITY - measured and reported, NOT part of the survival
// gate (see header). Floors below were originally calibrated against the
// pre-WORM_V5 shipped point (dragSettleGain=25, muscleLeakScale=50,
// muscleCalciumTau=0.1, bodyPoseDecayRate=0.6): agar mean efficiency ~0.55,
// water ~0.52.
//
// REVISED (see WORM_V5_REAL_AMPLITUDE_CALIBRATION.md/WORM_V5_RESULTS.md):
// muscleLeakScale/muscleCalciumTau/bodyPoseDecayRate were deliberately
// reshipped (50/0.1/0.6 -> 25/0.05/0.5) to close part of a large, real,
// user-observed bend-amplitude gap against real C. elegans (Vidal-Gadea
// et al. 2011/Pierce-Shimomura et al. 2008, cross-checked against Karbowski
// et al. 2008) - an explicit, disclosed, KNOWN trade of path-efficiency for
// bend amplitude, not a regression nobody noticed. Efficiency at the new
// point genuinely sits lower (agar/water mean ~0.21, ~2-8% of trials clear
// the old 0.40 bar) - the floors below are updated to match this new,
// deliberately-chosen normal, exactly the same honest-recalibration
// discipline already used once for kMinShapeRange/bendAmplitude thresholds
// in this same file. A collapse meaningfully BELOW this new floor (e.g.
// mean efficiency near zero) would still be a real signal worth
// investigating - this is not a blank check, just a re-anchored one.
//
// kQualityBarEfficiency=0.40 is left UNCHANGED (still the pre-WORM_V5 bar)
// deliberately - at the new mean (~0.21), "fraction of trials clearing a
// bar nearly 2x the mean" is inherently a small, noisy statistic (observed
// 2%/8% agar/water on one seed base), not a stable health signal anymore.
// kMinFractionAboveQualityBar is set to 0.0 (report-only, non-gating in
// practice) rather than a tight number that would flip on ordinary seed
// variance - kMinMeanEfficiency (a much less noisy 48-trial aggregate) is
// the real gate here now.
//
// ВОЗВРАЩЕНО К 0.35/0.50 (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md). Всё
// послабление выше делалось под точку, которая существовала только для
// компенсации ошибки в Params::motorPositionSource. После её исправления и
// отката компенсаций (muscleLeakScale 25->50, muscleCalciumTau 0.05->0.1,
// jointStiffeningGain 4.0->0.0) качество вернулось ВЫШЕ исходного уровня:
// agar mean efficiency 0.558 / 81% выше 0.40-бара, water 0.566 / 92%,
// survival 48/48 на обеих средах. Держать заниженные пороги больше не за
// чем - они бы просто скрывали будущие регрессии.
constexpr float kMinMeanEfficiency = 0.35f;
constexpr double kMinFractionAboveQualityBar = 0.50;
constexpr float kQualityBarEfficiency = 0.40f;  // same bar used throughout this project's calibration history

// BEND AMPLITUDE - measured and reported, NOT part of the survival gate, NOT
// (yet) a quality gate either - see header for the full story. Added because
// the project owner directly observed, watching Demo_worm.exe live, that
// "the worm moves almost without bending its body" - a defect the coarse
// SURVIVAL gates above cannot see (kMinShapeRange checks the RANGE of a
// whole-body bbox-diagonal scalar over the whole window, not how strongly
// any individual joint is bending at a given moment; a worm that barely
// bends but never freezes bit-for-bit still drifts the bbox diagonal by
// more than 0.02 over 2500 steps and sails through that gate untouched).
//
// A confirmation pass (8 fresh seeds - 7,13,10007,2023,5555,8642,31415,99999
// - both media, same warmupSteps=300/measureSteps=2500 split used here)
// quantified it: per-step max|angle_i| (over all 24 joints) starts pinned at
// 0.2497-0.2500 rad - i.e. clamp-saturated against WormBody's hard per-joint
// +/-0.25 rad limit, NOT a meaningful "organic peak" - then collapses within
// roughly steps 500-1000 to a noisy plateau that holds for the rest of the
// window with no further decay and no recovery. Windowed-mean max|angle_i|
// in that settled plateau (global steps [1500,2800) of an 8-seed x 2-medium
// sweep): min 0.0287, mean 0.0378, max 0.0416 rad - a 6.0x-8.7x drop (mean
// 6.7x) from the clamp-saturated early value, in every single seed tried,
// on both media (the two media are bit-for-bit identical here by
// construction: WormBody::step() drives angles_ purely kinematically from
// curvature/bend-stiffness/pose-decay, and dragNormal never enters that
// loop at localMechanoGain=0 - see body.cpp - so this metric cannot yet
// distinguish agar from water; that's a real architectural fact, not a bug
// in this test).
//
// Why a reported statistic and not a gate: we do not yet know whether this
// amplitude collapse is a fixable regression (e.g. muscleLeak/muscleCaTau/
// poseDecay interacting badly at these particular defaults) or some more
// fundamental property of the current curvature -> angle kinematics that
// needs its own separate investigation before anyone can say what a
// "healthy" floor even is. Wiring an arbitrary number into the survived
// bool before that's understood would repeat, on a new metric, the exact
// mistake this file's own history already made once with path-efficiency
// (see header) - so this stays parallel to LOCOMOTION QUALITY: measured,
// printed honestly, not gating.
//
// If a future change DOES want to gate on this once the above is
// understood, calibrate the floor against what THIS file actually reports
// below (mean, across survived trials, of each trial's own
// minSustainedBendAmplitude - currently ~0.0178 rad agar / ~0.0180 rad water
// on the shipped default) - NOT against the confirmation pass's windowed-
// mean-of-means figure above (0.0287-0.0416 rad), which is a different
// statistic (min-of-a-noisy-series vs. mean-of-that-series) and is
// necessarily higher. An independent verification pass caught this exact
// trap: naively importing "~0.02 rad" as a floor would immediately fail the
// current, presumably-healthy shipped default (0.0178/0.0180 < 0.02) before
// anyone decided that default was actually unhealthy. Also first sweep this
// metric across several already-known-survival-healthy alternate points
// (e.g. the bodyBendStiffness values WORM_V3_RESULTS.md documents as still
// healthy) to learn its normal range - only one configuration (the shipped
// default) has been characterized with this metric so far, which is not
// enough to know what "normal" spread looks like across genuinely good
// points, let alone set a floor.
//
// Split point: the settled/plateau window here is measured over the BACK
// HALF of measureSteps (i.e. local step >= measureSteps/2 within runTrial's
// measurement loop, which starts at global step warmupSteps). For the
// warmupSteps=300/measureSteps=2500 actually used below, that's local step
// 1250 = global step 1550 - just past the confirmation pass's own
// [1500,2800) late window, and comfortably after the collapse, which their
// full-resolution trace showed essentially finished by global step ~1000.
// So half-of-measureSteps is not an arbitrary round number here; it lines
// up with what was actually measured. Sampled at the same every-50-steps
// cadence as the existing coiled-ratio check (see runTrial) - no need for a
// finer stride, see the confirmation pass's own note that a windowed MEAN
// is what's stable, not any single raw step.

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;
    float minCoiledRatio = 1e9f;
    float maxCoiledRatioSeen = -1e9f;
    float freqHz = 0.0f;
    float maxAbsHeadingDelta = 0.0f;
    // Minimum, over the back half of the measurement window, of the
    // per-step max|angle_i| across all 24 joints - "how weak does the
    // sustained bending get, once any initial transient has settled." See
    // the BEND AMPLITUDE comment above. NOT part of the survival gate.
    float minSustainedBendAmplitude = 1e9f;
    bool survived = true;
    const char* failReason = "";  // which specific SURVIVAL gate failed, if any
};

Measurement runTrial(int seed, float dragNormal, int warmupSteps, int measureSteps, int freqPosition) {
    Measurement m;
    // Deliberately NOT touching a single Params field beyond medium - this
    // is what makes it a regression test of what's actually shipped, not of
    // some hand-picked point that has to be kept in sync by hand.
    WormSim sim("worm_data/celegans_herm.connectome");
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
    float minCoiled = 1e9f, maxCoiled = -1e9f;
    // Те же две величины, но набранные ТОЛЬКО во время омега-поворота - см.
    // разведение гейтов по фазам ниже. 1e9/0 = омега за прогон не случилась,
    // тогда соответствующий гейт тождественно проходит.
    float minCoiledOmega = 1e9f, maxHeadingDeltaOmega = 0.0f;
    // ФОРМА ОТСТАЁТ ОТ ФАЗЫ: после конца омеги суставы распрямляются с обычным
    // локомоторным пределом скорости, путь от потолка омеги до локомоторного
    // занимает (1.2-0.55)/0.5 = 1.3с. Выборка, попавшая в это окно, физически
    // ещё меряет омегу. Величина производная от отгруженных параметров.
    // Гейт свёрнутости - по ДОЛЕ времени, а не по мгновенному минимуму: тот же
    // критерий и по тем же причинам, что в Test_worm_locomotion, стенде
    // хемотаксиса и worm_v2_measurement (см. DIAGNOSIS раздел 30.8). Реверс
    // честно сгибает тело глубже обычного хода - это второй режим локомоции, а
    // не узел; ловить надо "свернулся и ОСТАЛСЯ".
    long coiledBelow = 0, coiledSamples = 0;
    int lastOmegaStep = -1000000;
    const int relaxSteps = static_cast<int>(1.3f / std::max(1e-6f, dt));
    // See BEND AMPLITUDE comment above (near kMinFractionAboveQualityBar)
    // for why this split point.
    const int sustainedWindowStart = measureSteps / 2;
    float minSustainedBendAmplitude = 1e9f;

    for (int i = 0; i < measureSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        float x, y;
        centroid(snap, x, y);
        if (std::isnan(x) || std::isnan(y)) { m.survived = false; m.failReason = "NaN"; return m; }
        pathLen += std::sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
        prevX = x; prevY = y;

        const auto& dev = sim.lastCurvatureDeviation();
        if (freqPosition >= 0 && freqPosition < static_cast<int>(dev.size())) {
            const float d = dev[static_cast<std::size_t>(freqPosition)];
            if (havePrevDeviation && ((d > 0.0f) != (prevDeviation > 0.0f))) ++zeroCrossings;
            prevDeviation = d;
            havePrevDeviation = true;
        }

        // ФАЗА ЛОКОМОЦИИ (WormSim::updateLocomotionState). Гейты ниже заданы
        // для ОБЫЧНОГО хода; во время омега-поворота предел скорости сустава
        // намеренно втрое выше, а тело намеренно сворачивается глубже, поэтому
        // те же пороги там меряли бы не то. Разводятся по фазам, а не
        // ослабляются: обычный ход проверяется ровно как раньше.
        if (sim.debugLocomotionPhase() == 2) lastOmegaStep = i;
        const bool inOmega = (i - lastOmegaStep) <= relaxSteps;
        const float headX = snap.pointsX[0], headY = snap.pointsY[0];
        const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            if (inOmega) maxHeadingDeltaOmega = std::max(maxHeadingDeltaOmega, std::fabs(hd));
            else m.maxAbsHeadingDelta = std::max(m.maxAbsHeadingDelta, std::fabs(hd));
        }
        prevHeading = heading;
        havePrevHeading = true;

        if (i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
                bx0 = std::min(bx0, snap.pointsX[p]); bx1 = std::max(bx1, snap.pointsX[p]);
                by0 = std::min(by0, snap.pointsY[p]); by1 = std::max(by1, snap.pointsY[p]);
            }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            const float coiled = diag / kBodyLength;
            if (inOmega) {
                minCoiledOmega = std::min(minCoiledOmega, coiled);
            } else {
                minCoiled = std::min(minCoiled, coiled);
                coiledBelow += (coiled < kMinCoiledRatio) ? 1 : 0;
                coiledSamples++;
            }
            maxCoiled = std::max(maxCoiled, coiled);

            // BEND AMPLITUDE (see comment above, near kMinFractionAboveQualityBar) -
            // same accessor already used by tests/worm_v2_measurement's trace
            // mode (sim.debugBodyAngles(), which is WormBody::angles() - the
            // real, post-clamp/decay/bend-stiffness joint angle, not the raw
            // network curvature signal). Same sampling cadence as the
            // coiled-ratio check just above, kept minimal and consistent.
            // ШАГИ ОМЕГА-ПОВОРОТА ИСКЛЮЧЕНЫ. Это метрика амплитуды ЛОКОМОТОРНОЙ
            // волны, и сравнивается она с биологическим диапазоном пика изгиба
            // ПРИ ПОЛЗАНИИ (0.494-0.589 рад). Во время омеги потолок сустава
            // намеренно поднят до 1.2 рад, поэтому один такой шаг задирал
            // максимум до 0.76-0.84 - то есть метрика начинала мерить омегу, а
            // не волну. Тот же класс ошибки, что и с гейтами (раздел 30.8).
            if (i >= sustainedWindowStart && !inOmega) {
                const auto& angles = sim.debugBodyAngles();
                float maxAbsAngle = 0.0f;
                for (float a : angles) maxAbsAngle = std::max(maxAbsAngle, std::fabs(a));
                minSustainedBendAmplitude = std::min(minSustainedBendAmplitude, maxAbsAngle);
            }
        }
    }

    m.minCoiledRatio = minCoiled;
    m.maxCoiledRatioSeen = maxCoiled;
    m.minSustainedBendAmplitude = minSustainedBendAmplitude;
    const float measureSeconds = static_cast<float>(measureSteps) * dt;
    const float netDisp = std::sqrt((prevX - startX) * (prevX - startX) + (prevY - startY) * (prevY - startY));
    m.efficiency = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
    m.bodyLengthsPerSec = static_cast<float>(pathLen) / measureSeconds / kBodyLength;
    m.freqHz = static_cast<float>(zeroCrossings) / 2.0f / measureSeconds;

    // SURVIVAL gates only - path efficiency does NOT appear here, see header.
    // Пороги для ОМЕГА-ПОВОРОТА - производные от локомоторных, а не отдельно
    // подобранные: во время омеги предел скорости сустава поднят в
    // omegaRateLimitScale=3 раза, поэтому и порог поворота головы втрое выше;
    // потолок угла поднят до аварийного 1.2 рад, при котором 24 сустава дают
    // замкнутую петлю с bbox-диагональю около 0.30 длины дуги, поэтому пол по
    // свёрнутости опускается вдвое ниже этого - до 0.15.
    constexpr float kMaxHeadingDeltaOmegaRad = kMaxHeadingDeltaRad * 3.0f;
    constexpr float kMinCoiledRatioOmega = 0.15f;
    const float coiledFraction =
        coiledSamples > 0 ? static_cast<float>(coiledBelow) / static_cast<float>(coiledSamples) : 0.0f;
    if (coiledFraction > 0.10f) { m.survived = false; m.failReason = "stays-coiled"; }
    else if (minCoiled < kMinCoiledRatioOmega) { m.survived = false; m.failReason = "coiled-into-knot"; }
    else if (minCoiledOmega < kMinCoiledRatioOmega) { m.survived = false; m.failReason = "omega-knot"; }
    else if (maxCoiled - minCoiled < kMinShapeRange) { m.survived = false; m.failReason = "frozen-straight"; }
    else if (m.maxAbsHeadingDelta > kMaxHeadingDeltaRad) { m.survived = false; m.failReason = "implausible-heading-whip"; }
    else if (maxHeadingDeltaOmega > kMaxHeadingDeltaOmegaRad) { m.survived = false; m.failReason = "omega-whip"; }
    else if (m.freqHz < kMinFreqHz) { m.survived = false; m.failReason = "not-oscillating"; }
    return m;
}

struct MediumResult {
    int survivedCount = 0, totalCount = 0;
    int aboveQualityBarCount = 0;
    double sumSpeed = 0.0, sumFreq = 0.0, sumEff = 0.0;  // over SURVIVED trials only
    // BEND AMPLITUDE (see comment above runTrial, near kMinFractionAboveQualityBar)
    // - mean AND the actual observed range across survived trials. Reported
    // honestly, not gated - see header/comment.
    double sumMinSustainedBendAmplitude = 0.0;
    float minBendAmplitudeObserved = 1e9f;
    float maxBendAmplitudeObserved = -1e9f;
};

MediumResult evaluateMedium(const char* label, float dragNormal, int numBases, int seedsPerBase, unsigned baseSeed) {
    MediumResult r;
    std::mt19937 baseRng(baseSeed);
    std::uniform_int_distribution<int> baseDist(1, 2000000000);
    std::printf("--- %s (dragNormal=%.2f) ---\n", label, dragNormal);
    for (int b = 0; b < numBases; ++b) {
        const int base = baseDist(baseRng);
        for (int s = 0; s < seedsPerBase; ++s) {
            const Measurement m = runTrial(base + s, dragNormal, 300, 2500, 12);
            r.totalCount++;
            if (!m.survived) {
                // A SURVIVAL failure is a real regression signal - print
                // loudly and unconditionally, not folded into a percentage.
                std::printf("  seed=%d DID NOT SURVIVE: %s (coiled=[%.3f,%.3f] freq=%.4fHz maxHeadDelta=%.4f)\n",
                            base + s, m.failReason, m.minCoiledRatio, m.maxCoiledRatioSeen, m.freqHz,
                            m.maxAbsHeadingDelta);
                continue;
            }
            r.survivedCount++;
            r.sumSpeed += m.bodyLengthsPerSec;
            r.sumFreq += m.freqHz;
            r.sumEff += m.efficiency;
            if (m.efficiency >= kQualityBarEfficiency) ++r.aboveQualityBarCount;
            r.sumMinSustainedBendAmplitude += m.minSustainedBendAmplitude;
            r.minBendAmplitudeObserved = std::min(r.minBendAmplitudeObserved, m.minSustainedBendAmplitude);
            r.maxBendAmplitudeObserved = std::max(r.maxBendAmplitudeObserved, m.minSustainedBendAmplitude);
        }
    }
    return r;
}

}  // namespace

int main() {
    std::printf("=== Test_worm_regression: validating LIVE WormSim::Params defaults, no overrides ===\n\n");

    const int kNumBases = 16, kSeedsPerBase = 3;
    const unsigned kBaseSeed = 424242u;

    MediumResult agar = evaluateMedium("AGAR", kDragAgar, kNumBases, kSeedsPerBase, kBaseSeed);
    MediumResult water = evaluateMedium("WATER", kDragWater, kNumBases, kSeedsPerBase, kBaseSeed + 1000000);

    bool ok = true;

    std::printf("\n== SURVIVAL (hard gate - coiling/freezing/implausible-whip/silent-network) ==\n");
    for (const auto& [label, r] : {std::pair{"AGAR", agar}, std::pair{"WATER", water}}) {
        const bool allSurvived = r.survivedCount == r.totalCount;
        std::printf("%-6s %d/%d survived %s\n", label, r.survivedCount, r.totalCount,
                    allSurvived ? "(100%)" : "(!!! REGRESSION - was 100% throughout this project's history)");
        if (!allSurvived) ok = false;
    }

    std::printf("\n== LOCOMOTION QUALITY (measured statistic, undirected-walk variance expected - NOT a survival gate) ==\n");
    for (const auto& [label, r] : {std::pair{"AGAR", agar}, std::pair{"WATER", water}}) {
        if (r.survivedCount == 0) { std::printf("%-6s no surviving trials - cannot measure quality\n", label); continue; }
        const double meanEff = r.sumEff / r.survivedCount;
        const double fracAboveBar = static_cast<double>(r.aboveQualityBarCount) / r.survivedCount;
        std::printf("%-6s mean speed=%.5f BL/s mean freq=%.4fHz mean efficiency=%.3f  "
                    "(%.0f%% of survived trials clear the %.2f quality bar)\n",
                    label, r.sumSpeed / r.survivedCount, r.sumFreq / r.survivedCount, meanEff,
                    fracAboveBar * 100.0, kQualityBarEfficiency);
        if (meanEff < kMinMeanEfficiency) {
            std::printf("  FAIL: mean efficiency %.3f below floor %.2f - real quality regression, not just one wandering seed\n",
                        meanEff, kMinMeanEfficiency);
            ok = false;
        }
        if (fracAboveBar < kMinFractionAboveQualityBar) {
            std::printf("  FAIL: only %.0f%% of trials clear the quality bar, below %.0f%% floor\n",
                        fracAboveBar * 100.0, kMinFractionAboveQualityBar * 100.0);
            ok = false;
        }
    }

    std::printf("\n== BEND AMPLITUDE (measured statistic, newly added - NOT a gate yet, see header/comment) ==\n");
    for (const auto& [label, r] : {std::pair{"AGAR", agar}, std::pair{"WATER", water}}) {
        if (r.survivedCount == 0) { std::printf("%-6s no surviving trials - cannot measure bend amplitude\n", label); continue; }
        const double meanMinSustained = r.sumMinSustainedBendAmplitude / r.survivedCount;
        std::printf("%-6s mean minSustainedBendAmplitude=%.4f rad  (observed range across trials: [%.4f, %.4f] rad)\n",
                    label, meanMinSustained, r.minBendAmplitudeObserved, r.maxBendAmplitudeObserved);
    }

    std::printf("\n%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
