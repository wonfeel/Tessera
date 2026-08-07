// tests/worm_locomotion/main.cpp
//
// Headless-регресс на движение WormSim. Полная история находок - в
// WormSim.h/.cpp и git-логе (numerical stability, muscle connectivity,
// curvature clamping, resistive-force physics, gap-junction over-coupling,
// whole-body containment, dt/speed decoupling, hex-grid alignment,
// body-clamp coiling, mechanical proprioception per Boyle/Berri/Cohen 2012).
//
// KNOWN OPEN ISSUE: food demonstrably excites AWA/ASE (neuron state differs
// sharply, e.g. up to 30x, with vs without a nearby food patch) and the
// synaptic path to the locomotor circuit is short and well-connected
// (graph BFS: all real hub interneurons AIY/AIZ/AIB/RIA at 1 hop,
// AVA/AVB/PVC/RIM at 2 hops; all 69 motor neurons and 95 muscles reached
// within 2 hops - not a topological bottleneck). Despite this, a seeded
// with-food/without-food comparison (same RNG seed, only food presence
// differs) shows no practically meaningful difference in final distance to
// food after 4000 steps, across an extensive sweep: chemGain up to 10x
// default, gradientGain up to 125x default, activationTheta 0-10,
// baseline EMA tau up to 5x, and noise down to zero. The sensory
// perturbation is real but appears to be many orders of magnitude smaller
// than the network's own intrinsic recurrent dynamics (visible even with
// zero external input/noise - the network never truly rests, states climb
// toward a large self-sustained equilibrium over tens of seconds
// regardless of sensory input).
//
// Follow-up (same session): tested the two most likely mechanisms behind
// that "recurrent dynamics drowns out sensing" gap directly, rather than
// just tuning gain knobs further. Both are properties of a real leaky-
// integrator network built on uniform, uncalibrated per-synapse weights:
// (1) sigmoid(0)=0.5, so an at-rest neuron transmits half-strength forever
// ("idle hum") instead of ~0; (2) a neuron's accumulated chemical current
// scales with its raw fan-in (sum of |weights| in), not with anything
// biologically meaningful, so hub neurons carry proportionally more idle
// current than sparse ones. Implemented both as opt-in Network toggles
// (subtract resting-state activation before the synapse matvec; divide
// accumulated chemical current by summed |incoming weight|) and measured
// them the right way this time: a *directional* metric (paired progress-
// toward-food-with-food minus progress-toward-food-without-food, same
// seed, food placed at a different angle per seed so no single fixed
// heading bias can masquerade as chemotaxis - the first version of this
// experiment placed food due +x from start, exactly where the worm's
// initial heading already points, and that alone produced a fake "effect"
// identical with and without food). Across 16 seeds: baseline effect
// -0.01 +/- 0.01 (stderr), zero-at-rest 0.00 +/- 0.01, indegree-normalized
// 0.00 +/- 0.00 - all indistinguishable from zero, not just underpowered.
// Indegree normalization additionally collapsed locomotion itself (roamed
// bounding-box diagonal dropped from ~330 to ~36 units, ~4 units when
// combined with zero-at-rest) because chemGain=0.02 was calibrated against
// raw unnormalized currents and is ~2 orders of magnitude too small once
// currents are rescaled to per-unit-fan-in - a real regression, not a fix.
// Both hypotheses reverted (production Network/WormSim unchanged) rather
// than left in as unused toggles. Conclusion: this isn't a synapse-
// transmission-shape or fan-in-dilution problem either. Still not
// resolved; the remaining honest options are per-neuron-type parameter
// calibration against real physiology (leak/capacitance/synaptic weight,
// not just 3 global scalars) or accepting that a reduced model on raw
// Cook2019 topology without calibrated weights may not support directed
// klinokinesis at all, and that emergent food-seeking may need to come
// from elsewhere (e.g. behavioral-state switching) rather than gradient-
// following through this circuit.
//
// Follow-up 2 (later session): before touching gains again, actually
// measured whether hub neurons are saturated rather than assuming a
// signal-magnitude problem. Two real diagnostics, each cross-checked by
// two independently-written analyses reading the same real exported data:
// (a) a saturation probe sampling real neuron V/sigmoid'(V) with vs without
// food, plus an open-loop perturbation sweep on real hub neurons; (b) a
// from-scratch 401x401 Jacobian of the network's continuous-time dynamics
// (numpy eig, cross-checked via trace=sum(eigenvalues) and two independent
// re-derivations). Findings: the "network is linearly unstable at rest"
// hypothesis is FALSE - 0/401 eigenvalues have Re>0, the resting state is
// genuinely stable (chemGain would need to rise ~2.78x, to ~0.056, before
// crossing into instability). But AWA/AWC (which read raw scent directly,
// with NO gain multiplier anywhere in applyFoodDrive - gradientGain only
// scales ASEL/ASER's derivative term, a distinct pathway) were hitting
// V~70-79 under a saturated food patch against foodMaxConcentration=100 -
// sigmoid'(V) exactly 0.0 in float, complete saturation, physically zero
// transmissible signal regardless of any other gain. ASE/AIZ saturate
// nearly as badly as a secondary cascade. Fix: foodMaxConcentration
// 100->6 (WormSim.h) - the only clamp ceiling feeding both the AWA/AWC
// absolute-level pathway and the ASE delta-concentration pathway, so a
// single change restores real headroom to both without touching gains at
// all. Confirmed via the saturation probe that sigmoid' is no longer
// clipped to exactly 0. Separately, RIA sits at only 5.7-8.4% of its max
// sigmoid slope even with literally zero external input ever supplied
// (an autonomous-equilibrium property, not food-driven) - real C. elegans
// RIA is specifically implicated in steering/klinotaxis, so tested pulling
// its baseline down via leakScale 1.0->1.3->1.6; measured no effect
// (meanEffect pinned at -0.02 +/- 0.002 regardless of leakScale) - reverted,
// not worth the added global-parameter risk for zero payoff.
//
// Despite fixing the one clearly, rigorously confirmed saturation bug
// (foodMaxConcentration, kept in production), the paired with/without-food
// chemotaxis effect is STILL indistinguishable from zero (-0.02 +/- 0.002,
// 16 seeds, food placed within actual sensing range this time - the first
// rebuild of this specific test accidentally placed food 400 units from
// start against a 70-unit depositFood radius, producing an identical-
// looking null result for a completely different, boring reason: the worm
// could not physically reach the patch, not a network problem). Also tried
// running 20x longer (50000 steps, well past the Jacobian's slowest
// ~755s-time-constant mode) in case the effect needs more time to build up
// - instead of a clean signal, one seed out of sixteen produced an effect
// ~30x larger than all the others combined, wrong-signed, while the rest
// stayed tiny and mixed-sign: the signature of chaotic sensitivity to
// initial conditions amplifying noise over long integration, not real
// directional bias, and unreliable/slow enough (>300s per run) to not be
// worth using as a test methodology.
//
// Conclusion: saturation was real and is now fixed, but was not the
// (or not the only) bottleneck - unsaturating the sensors did not unlock
// directed behavior. This matches independent literature findings (Wicks
// 1996: parameter magnitude barely matters, only sign/topology does, at
// this kind of network; Chen et al. 2022: the one paper that got real
// klinotaxis out of a full C. elegans connectome needed a genetic-
// algorithm search over 463 per-neuron/per-synapse parameters, 36% of GA
// runs succeeding even then, plus a phase/CPG-gating mechanism rather than
// static bias). Global-scalar gain tuning (chemGain/gapGain/leakScale/
// gradientGain/foodMaxConcentration) has now been exhausted as a strategy.
// Next honest options, in rough order of effort: (1) phase-gate the
// sensory drive by an existing locomotion-phase/CPG signal instead of
// injecting it as a constant additive term; (2) real per-neuron-class
// (not per-neuron - too many free parameters) leak/capacitance/tau values,
// derived from morphology or found via a small evolutionary search against
// this project's own paired with/without-food metric; (3) accept that
// directed klinokinesis may not be achievable from this reduced model
// without one of the above, and let food-seeking emerge from a different
// mechanism entirely (e.g. discrete behavioral-state switching, closer to
// how real C. elegans pirouettes rather than continuously steers).
//
// TRIED AND REVERTED (later session): implemented option (3) above -
// WormSim::applyBehavioralStateSwitch read real, already-emergent AVA
// (reversal)/AVB (forward) command-neuron activity, threshold-crossed it
// into a discrete turn, threshold itself biased by real d(scent)/dt. It
// DID produce this whole investigation's first real, positive, clearly-
// above-noise directed effect (best headless sweep: 52.85 +/- 12.25 over
// 16 seeds, vs. -0.02 +/- 0.002 for every prior continuous-steering
// attempt above) and survived three real bug-hunts along the way (a
// pre-existing body-freeze-into-a-circle issue under sustained food
// contact, unrelated to this mechanism - see the surviving fix below;
// a level-vs-edge-triggering bug that caused visible repeated spinning;
// an instant-vs-smoothed turn-application glitch). None of that matters:
// the mechanism itself was WRONG and got fully reverted, not kept. Its
// actual effector was `m_heading += dir * turnAngle` - application code
// writing directly to the worm's heading. That is exactly the thing this
// whole file's own header comment forbids ("не отдельная эвристика 'знак
// кривизны -> поворот'", "перемещение тела - решается каждый шаг из
// баланса сил трения на форме, которую породила сеть"). Reading real
// network state to decide WHEN to fire does not make the actuator
// legitimate - the turn never passed through WormBody's physics, so it
// was a hardcoded controller wearing a neural justification, not emergent
// behavior. Any future attempt at discrete reorientation must produce it
// by making the NETWORK (via calibrated parameters/weights) drive a real,
// physical whole-body bend through the existing, unmodified body-physics
// pipeline - never by having application code assign to m_heading/
// m_position directly from a "decision" computed off neuron state.
//
// The ONE genuinely separate finding from that investigation that IS kept
// in production: live play surfaced a pre-existing bug (nothing to do
// with the reverted mechanism) where sustained food contact (not merely
// "runs a long time" - a food-free 400000-step/20000s headless run never
// showed it) drove the body to freeze solid, position literally unchanged
// for 500-1000+ simulated seconds at a stretch. Root cause, confirmed by
// directly logging WormSim::lastCurvatureDeviation() (signed, per body
// position) during a frozen episode: under sustained food drive, curvature
// deviation becomes the SAME SIGN across all (or nearly all) 24 body
// positions AT ONCE - not an alternating travelling wave, a whole-body
// common-mode lean. The existing per-position TEMPORAL baseline (4s EMA)
// only catches drift in each position separately over time and can't react
// fast enough to a bias appearing simultaneously everywhere; the body's
// per-segment clamp (+/-0.25 rad) gets hit on all 24 segments within a
// handful of steps, and rebuild_points() sums segment angles into a
// cumulative heading - 24*0.25=6.0 rad is almost exactly 2*pi, i.e. a
// closed loop is close to geometrically inevitable once this happens.
// Once locked, shape stops changing, so solve_propulsion's shape-change-
// driven velocity solves to ~0 - the observed frozen position, exactly.
// Fix (kept, legitimate - it filters the network's own emergent curvature
// signal, same category as the pre-existing temporal baseline, no direct
// actuator): added a SPATIAL high-pass alongside the existing temporal
// one - subtract the across-body mean of curvature[] every step, before
// the per-position baseline/deviation split (see the comment at that call
// site in WormSim.cpp). This directly prevents the whole-body-same-sign
// case from ever accumulating, while leaving genuine alternating waves
// (which already sum near zero) untouched. Verified: the same continuous-
// food 200000-step/10000s scenario that previously froze repeatedly now
// shows zero freezes and a healthy, always-mixed curvature sign
// distribution throughout. Locomotion regression coiled ratio improved
// (0.47-0.52 -> 0.62-0.65).
//
// So: directed chemotaxis is, once again, honestly unresolved - the one
// approach that measurably worked was disqualified on principle, not on
// results. The real options remain exactly what they were before this
// detour: (1) phase-gate the sensory drive by an existing locomotion-
// phase/CPG signal; (2) per-neuron-class parameter calibration (leak/
// capacitance/tau), derived from morphology or a small evolutionary
// search against this project's own paired with/without-food metric -
// with the network's own output still required to reach the worm only
// through the existing curvature -> WormBody -> resistive-force pipeline,
// never through a shortcut like the one just reverted.
//
// UPDATE (later session): option (2) above worked, then got reverted. See
// tests/worm_chemotaxis_calibration for the full search history (two failed
// rounds chasing noise, then a screen-many/confirm-few design that caught
// its own top false positive and validated a real one three independent
// ways) - the found per-class leak/capacitance calibration was applied in
// WormSim.cpp's constructor and passed this file's regression numbers
// (coiled ratio unaffected/slightly improved). It was THEN REVERTED after a
// live look at the actual demo showed the worm settling into a static shape
// instead of crawling - a "displacement" mode added to the calibration
// harness measured it directly: the calibration cut plain crawling
// efficiency (net displacement / path length, no food) by more than 3x.
// This regression test's own health checks (coiled ratio, NaN, bounds,
// "roamed at least 1 unit") never caught it because none of them measure
// absolute locomotion efficiency, only shape health and a near-zero
// movement floor - worth keeping in mind before trusting a PASS here as
// proof of good crawling. Production is back to the loader's flat
// leak=1/capacitance=1 defaults; see WormSim.cpp's constructor comment.
//
// UPDATE (later session, same leak/capacitance axis, different objective -
// gait SPEED this time): live measurement found the worm's emergent bend
// cycle ~30-100+ simulated seconds against a real worm's 0.5-2 Hz - tests/
// worm_speed_calibration searched this axis again, this time WITH an
// efficiency floor from the start (learned from the paragraph above). Its
// own final-verification step looked clean (4.85x speedup, efficiency
// unchanged) and shipped. It was THEN REVERTED again, for a NEW reason:
// this network's dynamics turned out chaotic enough that a single seed
// base's mean (this file's own PASS numbers included - see "min coiled
// ratio" above, always from one fixed seed) is not a reliable estimate for
// this kind of global comparison. Measuring the "winning" candidate and the
// uncalibrated default across 20 independent fresh seed bases found
// IDENTICAL population-mean speed (0.01079 body-lengths/s both) - the
// original 4.85x came from one anomalous baseline reading that never
// reproduced again. The candidate was actually worse on the metric that
// matters (roughly half the mean efficiency, several times the bend
// frequency, for zero net-distance gain). This file's own single-seed PASS
// is silent on exactly this kind of multi-base instability - a green run
// here says the shape stayed healthy on THIS seed, not that a parameter
// change is a genuine improvement across seeds in general.
//
// fe_add_app собирает исходники только из папки самого теста - отдельной
// цели под WormSim/connectome тут нет, поэтому .cpp подключены напрямую
// (unity-стиль), только в этом одноразовом verification-таргете.
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/body.cpp"
#include "demo/worm/connectome/loader.cpp"
#include "demo/worm/WormSim.cpp"

#include <cmath>
#include <cstdio>

int main() {
    WormSim sim("worm_data/celegans_herm.connectome");
    const glm::vec2 boundsMin(0.0f);
    constexpr int kFieldCols = 28, kFieldRows = 20;
    constexpr float kHexSpacing = 36.0f;
    sim.setBounds(boundsMin, kFieldCols, kFieldRows, kHexSpacing);
    const glm::vec2 boundsMax = boundsMin + HexGrid::worldPos(kFieldCols - 1, kFieldRows - 1, kHexSpacing);

    const glm::vec2 food(900.0f, 600.0f);
    sim.depositFood(food, 5.0f);
    const float initialFood = sim.totalFood();

    float prevHeading = 0.0f;
    float maxAbsStepDelta = 0.0f;
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    bool anyNaN = false;
    bool outOfBounds = false;
    float minCoiledRatio = 1e9f;
    long coiledSteps = 0, coiledSamples = 0;  // см. гейт "свернулся в узел" ниже
    float maxCoiledRatio = -1e9f;
    WormSim::Snapshot snap;

    constexpr int kSteps = 3000;
    for (int i = 0; i < kSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float x = snap.pointsX[0], y = snap.pointsY[0];
        if (std::isnan(x) || std::isnan(y)) {
            anyNaN = true;
            break;
        }
        for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
            constexpr float kEps = 1.0f;
            if (snap.pointsX[p] < boundsMin.x - kEps || snap.pointsX[p] > boundsMax.x + kEps ||
                snap.pointsY[p] < boundsMin.y - kEps || snap.pointsY[p] > boundsMax.y + kEps) {
                outOfBounds = true;
            }
        }
        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);

        if (i > 200) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
                bx0 = std::min(bx0, snap.pointsX[p]); bx1 = std::max(bx1, snap.pointsX[p]);
                by0 = std::min(by0, snap.pointsY[p]); by1 = std::max(by1, snap.pointsY[p]);
            }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            constexpr float kArcLen = 576.0f;
            const float coiledRatio = diag / kArcLen;
            minCoiledRatio = std::min(minCoiledRatio, coiledRatio);
            maxCoiledRatio = std::max(maxCoiledRatio, coiledRatio);
            coiledSteps += (coiledRatio < 0.35f) ? 1 : 0;
            coiledSamples++;
        }

        const float heading = std::atan2(snap.pointsY[1] - y, snap.pointsX[1] - x);
        if (i > 0) {
            float d = heading - prevHeading;
            while (d > 3.14159f) d -= 2.0f * 3.14159f;
            while (d < -3.14159f) d += 2.0f * 3.14159f;
            maxAbsStepDelta = std::max(maxAbsStepDelta, std::fabs(d));
        }
        prevHeading = heading;

        if (i % 500 == 0 || i == kSteps - 1) {
            float dist = std::sqrt((food.x - x) * (food.x - x) + (food.y - y) * (food.y - y));
            std::printf("step %4d: pos=(%.1f,%.1f) distToFood=%.1f foodTotal=%.1f\n", i, x, y, dist, sim.totalFood());
        }
    }

    const float roamedX = maxX - minX, roamedY = maxY - minY;
    std::printf("position range: x=[%.1f,%.1f] y=[%.1f,%.1f]\n", minX, maxX, minY, maxY);
    std::printf("max |heading delta| in one step: %.4f rad\n", maxAbsStepDelta);
    std::printf("min coiled ratio (bbox diag / arc length, post-transient): %.3f\n", minCoiledRatio);
    std::printf("max coiled ratio (post-transient): %.3f\n", maxCoiledRatio);
    const float coiledFraction =
        coiledSamples > 0 ? static_cast<float>(coiledSteps) / static_cast<float>(coiledSamples) : 0.0f;
    std::printf("coiled fraction (steps below 0.35): %.1f%%\n", 100.0f * coiledFraction);
    std::printf("food total: initial=%.1f final=%.1f\n", initialFood, sim.totalFood());

    bool ok = true;
    if (anyNaN) { std::printf("FAIL: position went NaN\n"); ok = false; }
    if (outOfBounds) { std::printf("FAIL: some point of the body left the bounds\n"); ok = false; }
    if (maxAbsStepDelta > 3.2f) { std::printf("FAIL: single-step heading change is physically implausible\n"); ok = false; }
    if (roamedX < 1.0f && roamedY < 1.0f) { std::printf("FAIL: worm never moved at all\n"); ok = false; }
    // ГЕЙТ "СВЕРНУЛСЯ В УЗЕЛ" - ПО ДЛИТЕЛЬНОСТИ, А НЕ ПО МГНОВЕННОМУ МИНИМУМУ.
    //
    // Failure mode, который он ловит, всегда был один: тело сматывается и
    // ОСТАЁТСЯ смотанным (ratio уходит к нулю и не возвращается). Мгновенный
    // минимум был для этого годной мерой ровно до тех пор, пока в модели не
    // существовало ни одного законного глубокого изгиба.
    //
    // Теперь существует: омега-поворот (WormSim::updateLocomotionState,
    // Gray, Hill & Bargmann 2005) - это буквально сворачивание тела, голова
    // достаёт до хвоста, и он ОБЯЗАН давать низкое мгновенное значение. Он
    // занимает ~6% времени (измерено режимом pirouette), после чего тело
    // распрямляется само.
    //
    // Поэтому проверяются две разные вещи:
    //   - доля времени в свёрнутом состоянии: 20% - заведомо выше собственной
    //     скважности механизма (~6%) и заведомо ниже "смотался и остался"
    //     (там доля стремится к 100%);
    //   - жёсткий пол на настоящий узел, который не проходит ни один
    //     законный изгиб: 24 сустава по 1.2 рад (аварийный предел в body.cpp)
    //     дают замкнутую петлю, а её bbox-диагональ - около 0.30 длины дуги,
    //     так что 0.15 остаётся вдвое ниже самого крутого физически
    //     достижимого изгиба.
    if (coiledFraction > 0.20f) {
        std::printf("FAIL: body stays coiled (%.1f%% of steps below 0.35)\n", 100.0f * coiledFraction);
        ok = false;
    }
    if (minCoiledRatio < 0.15f) { std::printf("FAIL: body coiled into a tight knot\n"); ok = false; }
    // ДОБАВЛЕНО (см. WORM_V2_RESULTS.md раздел 2 и 11 пункт 10): этот гейт
    // ловил "смотано в узел" (ratio->0), но пропускал противоположный
    // failure mode - тело идеально ПРЯМОЕ и НИКОГДА не гнётся (ratio
    // застревает ровно на 1.000 всю дорогу, bbox-диагональ == длине дуги).
    // Живой случай: bodyGain=2.0 на новой (leak=50) мышечной динамике - roam
    // технически проходил порог roamedX/Y<1.0 (одна ось едва вышла за 1
    // unit), min coiled ratio=1.000 не меньше 0.35, ни один старый гейт не
    // ловил - тело было буквально заморожено. Настоящий сигнал - не
    // абсолютное значение ratio, а то, что оно вообще НЕ МЕНЯЕТСЯ (никакой
    // волны/дыхания формы) за весь прогон.
    if (maxCoiledRatio - minCoiledRatio < 0.02f) {
        std::printf("FAIL: body shape never changes (frozen straight, no bending wave)\n");
        ok = false;
    }
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
