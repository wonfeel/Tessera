// tests/worm_v2_measurement/main.cpp
//
// Honest measurement harness for the "worm-v2-architecture" rewrite (see
// H:\workspace\projects\Tessera\demo\worm\WORM_V2_DESIGN.md and
// WORM_V2_RESULTS.md). Modeled directly on tests/worm_speed_combined_
// calibration's methodology (centroid-based net displacement/path length,
// zero-crossing frequency at a fixed body position, coiled-ratio and
// max-heading-delta health gates, independent seed BASES not just more seeds
// from one) - reused deliberately, not reinvented, per this project's own
// established convention (see that file's header for why single-base
// verification is unreliable on this network).
//
// Unlike worm_speed_combined_calibration this is NOT a search - it takes an
// explicit V2 parameter point on the command line and measures it honestly
// across many independent bases, for the staged verification order in
// WORM_V2_DESIGN.md section 8.1 (muscle-only, kinematics-only, combined,
// +drag-settle, +proprioceptive-delay sensitivity).
//
// "trace" mode: single run, dumps muscle_output()/curvature-deviation range
// and frame_heading vs target for one segment over time - a saturation/
// dead-zone diagnostic in the spirit of tests/worm_saturation_probe (see
// WORM_V2_DESIGN.md section 2.4: "do not carry bodyGain/kDeviationClamp over
// blindly - diagnose the same way foodMaxConcentration was diagnosed").
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
// Множитель размера арены для пакетных прогонов. 1 - историческая арена, на
// которой снята вся таблица результатов. Взаимодействие со стеной оказалось
// вкладом в метрики скорости/эффективности/выбросов (см. режим burst: разворот
// у стены двигал центроид вбок на 1.26 длины тела в секунду), поэтому нужен
// способ померить ЧИСТУЮ локомоцию, без стен, тем же кодом.
int g_arenaScale = 1;
constexpr float kDragAgar = 40.0f, kDragWater = 1.7f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMaxHeadingDeltaRad = 0.5f;

// The point this whole rewrite is honestly compared against - see
// WORM_V2_DESIGN.md section 8 ("Честная точка сравнения").
constexpr float kOldShippedAgarHz = 0.0637f, kOldShippedWaterHz = 0.1120f, kOldShippedRatio = 3.523f;

struct V2Point {
    float muscleLeakScale = 50.0f;
    float muscleCalciumTau = 0.3f;
    float bodyPoseDecayRate = 0.5f;
    float bodyFrameRateLimitHz = 2.0f;
    float dragSettleTau = 2.0f;
    float dragSettleGain = 0.0f;
    float proprioceptiveDelaySeconds = 0.0f;
    // bodyGain<0 => leave WormSim's own default (2.0) untouched. See
    // WORM_V2_RESULTS.md for why this needed re-diagnosis: it is NOT one of
    // the four replaced mechanisms, but WORM_V2_DESIGN.md section 2.4
    // explicitly warns its v1 value (2.0, itself already search-found) was
    // tuned against the OLD near-zero-leak muscle dynamic range and must be
    // re-diagnosed (not blindly carried over) now that Output has a real
    // leak - same category of fix as the foodMaxConcentration saturation
    // diagnosis this project already did once.
    float bodyGain = -1.0f;
    // DVA mechanosensation (net.set_input(m_dva, gain*normalizedLoad+noise))
    // and local mechanosensation (direct additive term inside
    // applyProprioception's feedback sum) - both 0.0 in shipped defaults,
    // both untouched by the v2 rewrite (out of its mandate). Added here to
    // directly test whether either channel can reintroduce genuine
    // medium-dependent curvature amplitude/wavelength - see WORM_V2_RESULTS.md
    // section 11 item 2 ("частота воды не реалистична... structurally
    // identical dev/muscleOut/V between agar and water, confirmed by direct
    // trace").
    float mechanoGain = 0.0f;
    float localMechanoGain = 0.0f;
    // activationSlope<=0 => leave WormSim's own default (1.0) untouched.
    // Global sigmoid width (sigmoid((V-theta)/slope)) - affects EVERY
    // synapse in the network, not just muscle/ProcessingOutput. Added to
    // directly test the eigenmode-analysis prediction (pure linear algebra,
    // see WORM_V2_RESULTS.md section 13) that widening the sigmoid (higher
    // slope) should raise the network's own linear frequency ceiling AND
    // give the observed V~4 saturated operating point (section 10.3) more
    // real headroom, since chemGain=0.02 is kept fixed either way.
    float activationSlope = -1.0f;
    // WormBody::bend_stiffness_ (see WORM_V3_DESIGN.md section 2) - spatial
    // (not temporal) discrete-Laplacian coupling between neighboring joints,
    // 0.0 = off, bitwise prior behavior. Added as argv[18] (next free slot
    // after activationSlope's argv[17]).
    float bodyBendStiffness = 0.0f;
    // proprioceptiveOffset/proprioceptiveGain (WORM_V4_DESIGN.md section 2) -
    // <=0 => leave WormSim's own shipped default untouched (4.0/4.0). Added
    // as argv[19]/argv[20] (next free slots after bodyBendStiffness's
    // argv[18]) to screen the posterior stretch-receptor window width
    // (Boyle/Berri/Cohen 2012's N_SR) against a target of ~half body length.
    float proprioceptiveOffset = -1.0f;
    float proprioceptiveGain = -1.0f;
    // WormBody::joint_angle_clamp_ (WORM_V5_JOINT_CLAMP_RESULTS.md) - <=0
    // leaves WormSim's own shipped default (0.25 rad) untouched, same
    // sentinel convention as bodyGain/activationSlope/proprioceptiveOffset/
    // proprioceptiveGain above. Added as argv[21] (next free slot after
    // proprioceptiveGain's argv[20]).
    float jointAngleClamp = -1.0f;
    // WormBody::joint_soft_saturation_/joint_stiffening_gain_ (WORM_V5_SOFT_
    // JOINT_RESISTANCE_RESULTS.md) - both 0.0 by default, same as
    // bodyBendStiffness/mechanoGain above (0.0 IS the off state, so no <=0
    // sentinel gating is needed - always applied directly in applyPoint).
    // Added as argv[22]/argv[23] (next free slots after jointAngleClamp's
    // argv[21]).
    float jointSoftSaturation = 0.0f;
    float jointStiffeningGain = 0.0f;
    // Params::bClassOscillatorGain/bClassOscillatorTauW - распределённые
    // собственные осцилляторы B-класса (DB1-7/VB1-11), Gao/Fouad et al. 2018.
    // Оба 0/дефолт = выключено тождественно (та же схема, что у
    // jointSoftSaturation выше: 0.0 И ЕСТЬ состояние "выкл", сентинел не
    // нужен). Добавлены как argv[24]/argv[25] (следующие свободные слоты
    // после jointStiffeningGain's argv[23]) для WORM_V5_SPATIAL_ENVELOPE_
    // DIAGNOSIS.md: диагноз показал, что моторный выход не доходит до головы
    // и хвоста, а распределённый ритмогенератор - ровно тот биологический
    // механизм, который у настоящего червя гонит волну по всему телу. tauW
    // <=0 оставляет дефолт WormSim (4.0) нетронутым.
    float bClassOscillatorGain = 0.0f;
    float bClassOscillatorTauW = -1.0f;
    // Params::motorPositionSource - 0 (дефолт) синтетическая позиция
    // мотонейрона, 1 реальный центр иннервации. argv[26]. См.
    // WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md.
    int motorPositionSource = 0;
    // Params::proprioceptiveAnterior - 0 (дефолт) окно к хвосту, 1 к голове.
    // argv[27].
    int proprioceptiveAnterior = 0;
    // Params::driveEqualizationGain/Tau - пер-позиционная нормировка привода.
    // argv[28]/argv[29]. 0 = выключено (дефолт), прежнее поведение.
    float driveEqualizationGain = 0.0f;
    float driveEqualizationTau = -1.0f;
    // Params::mediumAmplitudeWaterRatio - зависимость амплитуды от среды.
    // argv[30]. 1.0 = выключено (дефолт).
    float mediumAmplitudeWaterRatio = 1.0f;
    // СЕТЕВОЕ масштабирование времени: leakScale/chemGain/gapGain.
    // argv[31]/[32]/[33]. <=0 оставляет дефолт WormSim.
    //
    // Из Network::step: dV/dt = -k*V + f, где k содержит leak_scale_*p.leak и
    // rowsum от gap_gain_, а f содержит chem_gain_*chem_input и
    // gap_gain_*gap_neighbors. Умножение ВСЕХ ТРЁХ на один множитель m даёт
    // k -> m*k и f -> m*f, то есть установившееся V=f/k не меняется, а вся
    // динамика идёт в m раз быстрее. Это чистое масштабирование времени сети,
    // а не смена рабочей точки - тот же приём, которым в разделе 14 отчёта
    // поднимался темп тела (bodyGain вместе с bodyPoseDecayRate).
    float leakScale = -1.0f;
    float chemGain = -1.0f;
    float gapGain = -1.0f;
    // Params::jointAngleRateLimit - предел скорости изменения угла сустава.
    // argv[34]. 0 = выключено (дефолт).
    float jointAngleRateLimit = 0.0f;
    // Params::mediumAmplitudeViaDecay. argv[35]. 0 = через потолок (дефолт).
    int mediumAmplitudeViaDecay = 0;
    // Params::muscleTargetTau. argv[36]. 0 = прежняя кинематика (дефолт).
    float muscleTargetTau = 0.0f;
    // Params::mediumBendInternalFraction (Fang-Yen et al. 2010) - 1.0 выключено.
    float mediumBendInternalFraction = 1.0f;
    // Params::dragSettleGain - стенд по историческим причинам гонял 0, тогда как
    // отгружено 25. Вынесен явно, чтобы можно было мерить ОТГРУЖЕННУЮ физику.
    float dragSettleGainOverride = -1.0f;
};

void applyPoint(WormSim& sim, const V2Point& p) {
    sim.params.muscleLeakScale = p.muscleLeakScale;
    sim.params.muscleCalciumTau = p.muscleCalciumTau;
    sim.params.bodyPoseDecayRate = p.bodyPoseDecayRate;
    sim.params.bodyFrameRateLimitHz = p.bodyFrameRateLimitHz;
    sim.params.dragSettleTau = p.dragSettleTau;
    sim.params.dragSettleGain = p.dragSettleGain;
    sim.params.proprioceptiveDelaySeconds = p.proprioceptiveDelaySeconds;
    if (p.bodyGain > 0.0f) sim.params.bodyGain = p.bodyGain;
    sim.params.mechanoGain = p.mechanoGain;
    sim.params.localMechanoGain = p.localMechanoGain;
    if (p.activationSlope > 0.0f) sim.params.activationSlope = p.activationSlope;
    sim.params.bodyBendStiffness = p.bodyBendStiffness;
    if (p.proprioceptiveOffset > 0.0f) sim.params.proprioceptiveOffset = p.proprioceptiveOffset;
    if (p.proprioceptiveGain > 0.0f) sim.params.proprioceptiveGain = p.proprioceptiveGain;
    if (p.jointAngleClamp > 0.0f) sim.params.jointAngleClamp = p.jointAngleClamp;
    sim.params.jointSoftSaturation = p.jointSoftSaturation;
    sim.params.jointStiffeningGain = p.jointStiffeningGain;
    sim.params.bClassOscillatorGain = p.bClassOscillatorGain;
    if (p.bClassOscillatorTauW > 0.0f) sim.params.bClassOscillatorTauW = p.bClassOscillatorTauW;
    sim.params.motorPositionSource = p.motorPositionSource;
    sim.params.proprioceptiveAnterior = p.proprioceptiveAnterior;
    sim.params.driveEqualizationGain = p.driveEqualizationGain;
    if (p.driveEqualizationTau > 0.0f) sim.params.driveEqualizationTau = p.driveEqualizationTau;
    sim.params.mediumAmplitudeWaterRatio = p.mediumAmplitudeWaterRatio;
    if (p.leakScale > 0.0f) sim.params.leakScale = p.leakScale;
    if (p.chemGain > 0.0f) sim.params.chemGain = p.chemGain;
    if (p.gapGain > 0.0f) sim.params.gapGain = p.gapGain;
    sim.params.jointAngleRateLimit = p.jointAngleRateLimit;
    sim.params.mediumAmplitudeViaDecay = p.mediumAmplitudeViaDecay;
    sim.params.muscleTargetTau = p.muscleTargetTau;
    sim.params.mediumBendInternalFraction = p.mediumBendInternalFraction;
}

void printPoint(const V2Point& p) {
    std::printf("[muscleLeak=%.3f muscleCaTau=%.3f poseDecay=%.3f frameRateHz=%.3f dragSettleTau=%.3f "
                "dragSettleGain=%.3f propDelay=%.4f bodyGain=%.3f bendStiffness=%.3f propOffset=%.3f propGain=%.3f "
                "jointClamp=%.3f jointSoftSat=%.3f jointStiffGain=%.4f bClassGain=%.4f bClassTauW=%.3f motorPosSrc=%d propAnterior=%d]",
                p.muscleLeakScale, p.muscleCalciumTau, p.bodyPoseDecayRate, p.bodyFrameRateLimitHz,
                p.dragSettleTau, p.dragSettleGain, p.proprioceptiveDelaySeconds, p.bodyGain, p.bodyBendStiffness,
                p.proprioceptiveOffset, p.proprioceptiveGain, p.jointAngleClamp > 0.0f ? p.jointAngleClamp : 0.25f,
                p.jointSoftSaturation, p.jointStiffeningGain, p.bClassOscillatorGain,
                p.bClassOscillatorTauW > 0.0f ? p.bClassOscillatorTauW : 4.0f, p.motorPositionSource, p.proprioceptiveAnterior);
}

// Окно распрямления после омеги: суставы возвращаются с обычным локомоторным
// пределом скорости, путь от потолка омеги до локомоторного занимает 1.3с.
// Выборка, попавшая в это окно, физически ещё меряет омегу.
constexpr int kOmegaRelaxSteps = 26;

struct Measurement {
    float bodyLengthsPerSec = 0.0f;
    float efficiency = 0.0f;
    float minCoiledRatio = 1e9f;
    long coiledBelow = 0, coiledSamples = 0;  // см. гейт свёрнутости по доле времени
    float freqHz = 0.0f;
    float maxAbsHeadingDelta = 0.0f;
    float maxHeadingDeltaOmega = 0.0f;  // см. разведение порога по фазам
    // Minimum, over the back half of the measurement window, of the per-step
    // max|angle_i| across all 24 joints - same definition/sampling cadence as
    // tests/worm_regression's minSustainedBendAmplitude (sim.debugBodyAngles(),
    // every 50 steps, local step >= measureSteps/2). Added purely additively
    // so this "point" mode can report bend amplitude alongside the existing
    // speed/freq/efficiency stats for a candidate (non-shipped-default) params
    // point - see WORM_V5_MUSCLE_LEAK_RESULTS.md for the motivating sweep.
    float minSustainedBendAmplitude = 1e9f;
    // Maximum, over the SAME back-half window/cadence as
    // minSustainedBendAmplitude above, of the per-step max|angle_i| across
    // all 24 joints - the PEAK (not floor) per-joint bend, added purely
    // additively (same pattern as minSustainedBendAmplitude's own addition,
    // see WORM_V5_MUSCLE_LEAK_RESULTS.md section 2.1) to compare directly
    // against the real-biology-grounded per-joint target derived in
    // WORM_V5_REAL_AMPLITUDE_TARGET.md section 5.2 (peak-to-peak/24-joint-
    // discretization-corrected crawl target ~0.49-0.59 rad) - that report's
    // own measurement used a separate offline trace+perl script; this makes
    // the same statistic a first-class "point" mode output for the
    // WORM_V5_REAL_AMPLITUDE_CALIBRATION.md joint-recalibration phase.
    float maxSustainedBendAmplitude = 0.0f;
    // ОГИБАЮЩАЯ изгиба вдоль тела - см. WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md.
    // Обе метрики выше (min/maxSustainedBendAmplitude) считают max|angle_i| ПО
    // ВСЕМУ телу за шаг и потому одинаковы и когда гнётся один сустав, и когда
    // гнутся все 24 - именно эта слепота к пространству и пропустила главный
    // дефект (изгиб локализован в суставах 9-19, голова и хвост прямые), пока
    // вся калибровка V5 гналась за пиковой амплитудой.
    //
    // bendCoverage - доля из 24 суставов, чей размах (p2p) за окно составляет
    // не менее kCoverageFraction от размаха самого активного сустава. У
    // настоящей C. elegans волна идёт по всему телу с примерно равномерной
    // амплитудой => coverage близка к 1.0; одиночный горб в середине даёт
    // ~0.3-0.4.
    //
    // endToPeakRatio - средний размах на концах тела (суставы 0-4 и 19-23)
    // к размаху самого активного сустава. Прямо ловит "мёртвые голова и
    // хвост"; у реального червя ~0.7-1.0 (амплитуда к хвосту скорее растёт).
    float bendCoverage = 0.0f;
    float endToPeakRatio = 0.0f;
    // НАПРАВЛЕНИЕ хода: средняя по шагам проекция мгновенного смещения
    // центроида на единичную ось "хвост -> голова" (голова - points[0], хвост
    // - последняя точка), нормированная на длину тела и делённая на dt, то
    // есть BL/s СО ЗНАКОМ. >0 - головой вперёд, <0 - задом наперёд.
    //
    // Все прежние метрики скорости в этом проекте - модуль (pathLen и
    // |netDisp|), направление хода относительно тела не измерялось НИКОГДА,
    // хотя вопрос "не идёт ли червь задом наперёд" в комментариях к
    // Params::proprioceptiveGain разбирается как решённый. Проекция берётся
    // пошагово, а не по итоговому смещению: за длинный прогон червь
    // поворачивает, и итоговый вектор смещения перестаёт быть связан с
    // ориентацией тела.
    float signedForwardBLps = 0.0f;
    // ВЫБРОСЫ СКОРОСТИ. Владелец проекта в живой демке: "он в моменте умеет
    // ускоряться в десятки раз". Все прежние метрики скорости - средние по
    // прогону (pathLen/время), и такое событие в них не видно вообще: короткий
    // выброс на фоне 2500 шагов почти не сдвигает среднее.
    //
    // burstRatio - отношение максимального мгновенного шага центроида к
    // МЕДИАННОМУ. Медиана, а не среднее: среднее само тянется выбросом, и
    // отношение занижалось бы тем сильнее, чем сильнее событие. У ровного хода
    // отношение порядка единиц; "в десятки раз" даст десятки.
    float burstRatio = 0.0f;
    // Доля шагов, где мгновенный шаг превышает медианный больше чем в 5 раз -
    // отвечает на "это единичный сбой или регулярный режим". Порог 5
    // выбран как явно выходящий за разброс ровного хода, не выведен.
    float burstFraction = 0.0f;
    bool healthy = true;
};

// Порог для Measurement::bendCoverage. 0.5 - половина размаха самого активного
// сустава; выбран как явно читаемая граница "сустав участвует в волне против
// почти не двигается", НЕ выведен из данных. Смена порога двигает абсолютное
// значение coverage, но не порядок точек между собой.
constexpr float kCoverageFraction = 0.5f;

Measurement runTrial(const V2Point& pt, int seed, float dragNormal, int warmupSteps, int measureSteps,
                      int freqPosition) {
    Measurement m;
    int lastOmegaStep = -1000000;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyPoint(sim, pt);
    sim.params.dragTangent = 1.0f;
    sim.params.dragNormal = dragNormal;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), kFieldCols * g_arenaScale, kFieldRows * g_arenaScale, kHexSpacing);
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
    double signedForwardSum = 0.0; // см. Measurement::signedForwardBLps
    std::vector<float> stepLens;   // см. Measurement::burstRatio
    stepLens.reserve(static_cast<std::size_t>(std::max(0, measureSteps)));

    float prevDeviation = 0.0f;
    bool havePrevDeviation = false;
    int zeroCrossings = 0;
    float prevHeading = 0.0f;
    bool havePrevHeading = false;
    // See BEND AMPLITUDE comment on Measurement::minSustainedBendAmplitude
    // above - same split point as tests/worm_regression.
    const int sustainedWindowStart = measureSteps / 2;
    float minSustainedBendAmplitude = 1e9f;
    // See Measurement::maxSustainedBendAmplitude above - same window/cadence,
    // tracked in the same loop/sample as the min, just accumulating max
    // instead of min so both the floor and the peak of the sustained window
    // are available from a single pass (no second simulation run needed).
    float maxSustainedBendAmplitude = 0.0f;
    // Пер-суставные min/max за то же устоявшееся окно - вход для
    // bendCoverage/endToPeakRatio (см. Measurement). Собираются КАЖДЫЙ шаг
    // окна (а не раз в 50, как min/max выше): размах отдельного сустава на
    // редкой выборке систематически занижается, и тем сильнее, чем быстрее
    // сустав колеблется - это сместило бы сравнение точек с разной частотой.
    std::vector<float> jointMin, jointMax;
    for (int i = 0; i < measureSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        float x, y;
        centroid(snap, x, y);
        const float stepLen = std::sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
        pathLen += stepLen;
        stepLens.push_back(stepLen);
        // Знаковая проекция шага на ось тела - см. Measurement::
        // signedForwardBLps. Ось строится по КРАЙНИМ точкам (голова минус
        // хвост), а не по первому сегменту: при сильном изгибе передний
        // сегмент смотрит вбок и знак начал бы шуметь.
        {
            const std::size_t last = snap.pointsX.size() - 1;
            const float axX = snap.pointsX[0] - snap.pointsX[last];
            const float axY = snap.pointsY[0] - snap.pointsY[last];
            const float axLen = std::sqrt(axX * axX + axY * axY);
            if (axLen > 1e-6f)
                signedForwardSum += ((x - prevX) * axX + (y - prevY) * axY) / axLen;
        }
        prevX = x; prevY = y;

        const auto& dev = sim.lastCurvatureDeviation();
        if (freqPosition >= 0 && freqPosition < static_cast<int>(dev.size())) {
            const float d = dev[static_cast<std::size_t>(freqPosition)];
            if (havePrevDeviation && ((d > 0.0f) != (prevDeviation > 0.0f))) ++zeroCrossings;
            prevDeviation = d;
            havePrevDeviation = true;
        }

        // ФОРМА ОТСТАЁТ ОТ ФАЗЫ: после конца омеги суставы распрямляются с
        // обычным локомоторным пределом скорости, и это занимает 1.3с. Гейты
        // ниже разведены по фазам, и окно распрямления считается омегой.
        if (sim.debugLocomotionPhase() == 2) lastOmegaStep = i;
        const float headX = snap.pointsX[0], headY = snap.pointsY[0];
        const float heading = std::atan2(snap.pointsY[1] - headY, snap.pointsX[1] - headX);
        if (havePrevHeading) {
            float hd = heading - prevHeading;
            while (hd > kPi) hd -= 2.0f * kPi;
            while (hd < -kPi) hd += 2.0f * kPi;
            // Разведение по фазам - см. гейт ниже. Окно распрямления после
            // омеги (1.3с) считается омегой: форма отстаёт от фазы.
            if (i - lastOmegaStep <= kOmegaRelaxSteps)
                m.maxHeadingDeltaOmega = std::max(m.maxHeadingDeltaOmega, std::fabs(hd));
            else m.maxAbsHeadingDelta = std::max(m.maxAbsHeadingDelta, std::fabs(hd));
        }
        prevHeading = heading;
        havePrevHeading = true;

        if (std::isnan(x) || std::isnan(y)) { m.healthy = false; return m; }
        if (i >= sustainedWindowStart) {
            const auto& angles = sim.debugBodyAngles();
            if (jointMin.size() != angles.size()) {
                jointMin.assign(angles.size(), 1e9f);
                jointMax.assign(angles.size(), -1e9f);
            }
            for (std::size_t j = 0; j < angles.size(); ++j) {
                jointMin[j] = std::min(jointMin[j], angles[j]);
                jointMax[j] = std::max(jointMax[j], angles[j]);
            }
        }
        if (i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
            for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
            const float diagNow = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            const float coiledNow = diagNow / kBodyLength;
            m.minCoiledRatio = std::min(m.minCoiledRatio, coiledNow);
            m.coiledBelow += (coiledNow < 0.30f) ? 1 : 0;
            m.coiledSamples++;

            // BEND AMPLITUDE - same accessor/cadence/split-point as
            // tests/worm_regression's minSustainedBendAmplitude.
            if (i >= sustainedWindowStart) {
                const auto& angles = sim.debugBodyAngles();
                float maxAbsAngle = 0.0f;
                for (float a : angles) maxAbsAngle = std::max(maxAbsAngle, std::fabs(a));
                minSustainedBendAmplitude = std::min(minSustainedBendAmplitude, maxAbsAngle);
                maxSustainedBendAmplitude = std::max(maxSustainedBendAmplitude, maxAbsAngle);
            }
        }
    }

    m.minSustainedBendAmplitude = minSustainedBendAmplitude;
    m.maxSustainedBendAmplitude = maxSustainedBendAmplitude;
    // bendCoverage/endToPeakRatio из собранных пер-суставных размахов - см.
    // Measurement. Концы тела - по 5 суставов с каждой стороны (0-4 и 19-23):
    // граница выбрана так, чтобы "концы" не пересекались с наблюдаемым
    // активным участком 9-19, а не выведена из анатомии.
    if (!jointMin.empty()) {
        const std::size_t n = jointMin.size();
        std::vector<float> p2p(n);
        float peak = 0.0f;
        for (std::size_t j = 0; j < n; ++j) {
            p2p[j] = jointMax[j] - jointMin[j];
            peak = std::max(peak, p2p[j]);
        }
        if (peak > 1e-6f) {
            int covered = 0;
            for (std::size_t j = 0; j < n; ++j)
                if (p2p[j] >= kCoverageFraction * peak) ++covered;
            m.bendCoverage = static_cast<float>(covered) / static_cast<float>(n);

            const std::size_t endSpan = std::min<std::size_t>(5, n / 2);
            float endSum = 0.0f;
            int endCount = 0;
            for (std::size_t j = 0; j < endSpan; ++j) { endSum += p2p[j]; ++endCount; }
            for (std::size_t j = n - endSpan; j < n; ++j) { endSum += p2p[j]; ++endCount; }
            m.endToPeakRatio = (endSum / static_cast<float>(endCount)) / peak;
        }
    }
    // ГЕЙТ СВЁРНУТОСТИ - ПО ДОЛЕ ВРЕМЕНИ. Порог 0.30 калиброван на черве,
    // который умел только ползти вперёд; омега-поворот - законное
    // кратковременное сворачивание тела. Тот же критерий и по тем же причинам
    // применён в Test_worm_locomotion, Test_worm_regression и стенде
    // хемотаксиса - см. WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 30.8.
    if (m.coiledSamples > 0 && static_cast<float>(m.coiledBelow) / m.coiledSamples > 0.10f) m.healthy = false;
    if (m.minCoiledRatio < 0.15f) m.healthy = false;
    // Порог поворота головы РАЗВЕДЁН ПО ФАЗАМ, а не ослаблен для всех: во время
    // омеги предел скорости сустава намеренно поднят в omegaRateLimitScale раз,
    // вне её порог обязан остаться прежним. Единый ослабленный порог - ошибка,
    // которая уже один раз обошлась: стенд показывал 16/16 там, где
    // Test_worm_regression честно видел 21/48, и разница была ровно в этом.
    if (m.maxAbsHeadingDelta > kMaxHeadingDeltaRad) m.healthy = false;
    if (m.maxHeadingDeltaOmega > kMaxHeadingDeltaRad * 3.0f) m.healthy = false;

    const float netDisp = std::sqrt((prevX - startX) * (prevX - startX) + (prevY - startY) * (prevY - startY));
    const float measureSeconds = static_cast<float>(measureSteps) * dt;
    m.efficiency = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
    m.bodyLengthsPerSec = static_cast<float>(pathLen) / measureSeconds / kBodyLength;
    m.signedForwardBLps = static_cast<float>(signedForwardSum) / measureSeconds / kBodyLength;
    // burstRatio/burstFraction - см. Measurement. Медиана берётся по копии,
    // чтобы не портить порядок stepLens (он больше не нужен, но nth_element
    // на самом векторе сделал бы последующее чтение неочевидным).
    if (!stepLens.empty()) {
        std::vector<float> sorted = stepLens;
        std::sort(sorted.begin(), sorted.end());
        const float median = sorted[sorted.size() / 2];
        const float maxStep = sorted.back();
        if (median > 1e-9f) {
            m.burstRatio = maxStep / median;
            int over = 0;
            for (float v : stepLens)
                if (v > 5.0f * median) ++over;
            m.burstFraction = static_cast<float>(over) / static_cast<float>(stepLens.size());
        }
    }
    m.freqHz = static_cast<float>(zeroCrossings) / 2.0f / measureSeconds;
    return m;
}

struct AggregateResult {
    float meanBLps = 0.0f, stderrBLps = 0.0f, meanFreqHz = 0.0f, meanEfficiency = 0.0f, minCoiledRatio = 1e9f;
    float maxHeadingDelta = 0.0f;
    // Mean, over healthy trials, of each trial's own minSustainedBendAmplitude
    // - same statistic tests/worm_regression reports, added here so "point"
    // mode can measure bend amplitude at candidate (non-shipped) params
    // points too. See Measurement::minSustainedBendAmplitude above.
    float meanBendAmplitude = 0.0f;
    // Mean, over healthy trials, of each trial's own maxSustainedBendAmplitude
    // - the PEAK-per-joint counterpart to meanBendAmplitude above (which
    // averages the floor/min statistic). See Measurement::
    // maxSustainedBendAmplitude for why this is the correct statistic to
    // compare against the real-biology target.
    float meanPeakBendAmplitude = 0.0f;
    // Средние по здоровым прогонам bendCoverage/endToPeakRatio - см.
    // Measurement за определением обеих и WORM_V5_SPATIAL_ENVELOPE_
    // DIAGNOSIS.md за тем, почему без них вся калибровка V5 была слепа к
    // главному дефекту формы.
    float meanBendCoverage = 0.0f;
    float meanEndToPeakRatio = 0.0f;
    // Среднее по здоровым прогонам Measurement::signedForwardBLps - знаковая
    // скорость вдоль оси тела. >0 головой вперёд, <0 задом наперёд.
    float meanSignedForwardBLps = 0.0f;
    // МАКСИМУМ (не среднее) burstRatio по здоровым прогонам - выброс это
    // событие, и усреднение по прогонам его размывает так же, как усреднение
    // по шагам. Плюс средняя доля шагов-выбросов. См. Measurement::burstRatio.
    float maxBurstRatio = 0.0f;
    float meanBurstFraction = 0.0f;
    bool allHealthy = true;
    int healthyCount = 0, totalCount = 0;
};

AggregateResult evaluate(const V2Point& pt, int numSeeds, int seedBase, float dragNormal, int warmupSteps,
                          int measureSteps, int freqPosition = 12) {
    AggregateResult ar;
    std::vector<float> blSamples;
    double sumFreq = 0.0, sumEff = 0.0, sumBendAmplitude = 0.0, sumPeakBendAmplitude = 0.0;
    double sumCoverage = 0.0, sumEndRatio = 0.0, sumSignedForward = 0.0, sumBurstFraction = 0.0;
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
        sumBendAmplitude += m.minSustainedBendAmplitude;
        sumPeakBendAmplitude += m.maxSustainedBendAmplitude;
        sumCoverage += m.bendCoverage;
        sumEndRatio += m.endToPeakRatio;
        sumSignedForward += m.signedForwardBLps;
        ar.maxBurstRatio = std::max(ar.maxBurstRatio, m.burstRatio);
        sumBurstFraction += m.burstFraction;
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
        ar.meanBendAmplitude = static_cast<float>(sumBendAmplitude / blSamples.size());
        ar.meanPeakBendAmplitude = static_cast<float>(sumPeakBendAmplitude / blSamples.size());
        ar.meanBendCoverage = static_cast<float>(sumCoverage / blSamples.size());
        ar.meanEndToPeakRatio = static_cast<float>(sumEndRatio / blSamples.size());
        ar.meanSignedForwardBLps = static_cast<float>(sumSignedForward / blSamples.size());
        ar.meanBurstFraction = static_cast<float>(sumBurstFraction / blSamples.size());
    }
    return ar;
}

// Порог эффективности пути снижен: с появлением пируэтов прямолинейность
// перестала быть признаком здоровья - настоящий червь переориентируется, и
// измеренная эффективность на отгруженной точке 0.55-0.58 против прежних ~0.9.
// 0.30 остаётся заведомо выше "не движется никуда".
constexpr float kMinEfficiency = 0.30f;
constexpr float kMinCoiledRatio = 0.15f;
constexpr float kMinFreqHz = 0.001f;

bool isHealthy(const AggregateResult& ar) {
    return ar.allHealthy && ar.minCoiledRatio >= kMinCoiledRatio && ar.meanEfficiency >= kMinEfficiency &&
           ar.meanFreqHz >= kMinFreqHz && ar.maxHeadingDelta <= kMaxHeadingDeltaRad;
}

void printAgg(const char* label, const AggregateResult& ar) {
    // bendAmp/peakBendAmp appended at the END of the line - purely additive,
    // every field before peakBendAmp is unchanged from before that metric was
    // added (see WORM_V5_MUSCLE_LEAK_RESULTS.md / WORM_V5_REAL_AMPLITUDE_
    // CALIBRATION.md).
    std::printf("  %-6s speed=%.5f+/-%.5f BL/s freq=%.4fHz eff=%.3f coiled=%.3f maxHeadDelta=%.4f "
                "seeds=%d/%d healthy=%s bendAmp=%.4f peakBendAmp=%.4f coverage=%.3f endRatio=%.3f fwd=%+.5f burstRatio=%.1f burstFrac=%.4f\n",
                label, ar.meanBLps, ar.stderrBLps, ar.meanFreqHz, ar.meanEfficiency, ar.minCoiledRatio,
                ar.maxHeadingDelta, ar.healthyCount, ar.totalCount, isHealthy(ar) ? "yes" : "NO",
                ar.meanBendAmplitude, ar.meanPeakBendAmplitude, ar.meanBendCoverage, ar.meanEndToPeakRatio, ar.meanSignedForwardBLps, ar.maxBurstRatio,
                ar.meanBurstFraction);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe trace [seed] [steps] [dragNormal] [bodyGain] [mechanoGain]
    //             [localMechanoGain] [dumpPhase=0/1] [bodyBendStiffness]
    //             [proprioceptiveOffset] [proprioceptiveGain] [kickStep]
    //             [kickMagnitude] [muscleLeakScale] [muscleCalciumTau]
    //             [bodyPoseDecayRate] [jointAngleClamp] [jointSoftSaturation]
    //             [jointStiffeningGain]
    // Dumps muscle_output()/curvature-deviation range and frame_heading vs
    // target for a handful of segments over time - saturation/dead-zone
    // diagnostic (see WORM_V2_DESIGN.md section 2.4). dumpPhase=1 additionally
    // dumps the full per-position dev[]/angles() vectors every step (see
    // WORM_V3_DESIGN.md section 5.4's geometric-phase diagnostic). kickStep/
    // kickMagnitude (>0) inject a one-time random-sign voltage perturbation
    // into every motor neuron right after that step number - limit-cycle
    // attractor diagnostic, see WORM_V5_KICKTEST_RESULTS.md.
    // ./exe burst <seed> <steps> <dragNormal> <thresholdMult> [bodyGain] [decay] [leak] [chem] [gap] [waterRatio]
    // ДИАГНОСТИКА ВЫБРОСА СКОРОСТИ. Выбросы оказались общим ограничителем для
    // всех трёх осей, которые эта сессия пыталась улучшить (амплитуда, отношение
    // амплитуд вода/агар, частота+скорость) - см. WORM_V5_SPATIAL_ENVELOPE_
    // DIAGNOSIS.md раздел 25.5. До сих пор они только СЧИТАЛИСЬ (burstRatio), а
    // механизм не смотрели.
    //
    // Режим ловит шаги, где мгновенная скорость центроида превышает медианную
    // более чем в thresholdMult раз, и печатает внутренности решения баланса
    // сил в ЭТОТ момент: определитель матрицы 3x3, норму правой части,
    // механическую нагрузку, максимальный |угол| и |deviation|. Плюс два шага
    // до и два после, чтобы видеть нарастание, а не только пик.
    //
    // Главный подозреваемый - определитель: скорости решаются по Крамеру как
    // det(M_col)/det(A), и страховка в solve_propulsion срабатывает только при
    // |det|<1e-9. Почти вырожденная матрица (скажем 1e-6) даёт огромные
    // скорости, НЕ поднимая никакого флага.
    // ./exe pirouette <seed> <steps> <dragNormal> <omegaBendBias> [reversalBaseRate]
    //                 [reversalGradientGain] [omegaDuration] [omegaBendSpan] [rateLimit]
    // ДИАГНОСТИКА ПИРУЭТА (Pierce-Shimomura et al. 1999) - см.
    // WormSim::updateLocomotionState. Меряет ровно три вещи, которые механизм
    // обязан давать и которые ничем косвенным не проверяются:
    //   1. идёт ли червь в фазе Reverse ДЕЙСТВИТЕЛЬНО назад (знаковая
    //      проекция смещения центроида на ось хвост->голова должна сменить
    //      знак, иначе "реверс" - это просто пауза);
    //   2. на сколько градусов меняется курс за один омега-поворот (у
    //      настоящего червя это большая величина, порядка 150 градусов, не
    //      подруливание);
    //   3. сколько времени тратится на каждую фазу.
    if (argc >= 2 && std::string(argv[1]) == "pirouette") {
        const unsigned seed = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 12345u;
        const int steps = argc > 3 ? std::atoi(argv[3]) : 20000;
        const float dragNormal = argc > 4 ? static_cast<float>(std::atof(argv[4])) : kDragAgar;
        V2Point pt;
        pt.muscleLeakScale = 50.0f;
        pt.muscleCalciumTau = 0.05f;
        pt.bodyPoseDecayRate = 9.0f;
        pt.motorPositionSource = 1;
        pt.proprioceptiveAnterior = 1;
        pt.driveEqualizationGain = 1.0f;
        pt.jointAngleClamp = 0.55f;
        pt.mediumAmplitudeWaterRatio = 0.45f;
        pt.jointAngleRateLimit = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.5f;
        pt.muscleTargetTau = 0.1f;
        pt.bodyGain = 300.0f;
        pt.leakScale = 3.0f;
        pt.chemGain = 0.06f;
        pt.gapGain = 0.06f;

        WormSim sim("worm_data/celegans_herm.connectome");
        applyPoint(sim, pt);
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = dragNormal;
        if (argc > 5) sim.params.omegaBendBias = static_cast<float>(std::atof(argv[5]));
        if (argc > 6) sim.params.reversalBaseRate = static_cast<float>(std::atof(argv[6]));
        if (argc > 7) sim.params.reversalGradientGain = static_cast<float>(std::atof(argv[7]));
        if (argc > 8) sim.params.omegaDuration = static_cast<float>(std::atof(argv[8]));
        if (argc > 10) sim.params.omegaClampScale = static_cast<float>(std::atof(argv[10]));
        if (argc > 11) sim.params.omegaRateLimitScale = static_cast<float>(std::atof(argv[11]));
        if (argc > 12) sim.params.omegaBendSpan = std::atoi(argv[12]);
        std::srand(seed);
        // Арена без достижимых стен: разворот у границы - отдельный механизм, и
        // мешать его со сменой курса от омега-поворота нельзя.
        sim.setBounds(glm::vec2(0.0f), kFieldCols * 8, kFieldRows * 8, kHexSpacing);
        const float dt = sim.params.dt.load();
        std::printf("pirouette diag: seed=%u steps=%d drag=%.2f omegaBias=%.3f baseRate=%.4f gradGain=%.1f "
                    "omegaDur=%.2f rateLimit=%.2f\n",
                    seed, steps, dragNormal, sim.params.omegaBendBias.load(), sim.params.reversalBaseRate.load(),
                    sim.params.reversalGradientGain.load(), sim.params.omegaDuration.load(),
                    sim.params.jointAngleRateLimit.load());

        WormSim::Snapshot snap;
        int phaseSteps[3] = {0, 0, 0};
        double phaseFwd[3] = {0.0, 0.0, 0.0};
        int prevPhase = 0;
        float axAtPhaseStart = 0.0f;
        std::vector<float> omegaTurns, reverseRuns, forwardRuns;
        struct P2 { float x, y; };
        std::vector<P2> centroidTrack;
        centroidTrack.reserve(static_cast<std::size_t>(steps) + 1);
        constexpr int kRunWindow = 60;  // 3с при dt=0.05 - окно оценки направления пробега
        int episodeStart = -1, pendingEnd = -1;
        int phaseStartStep = 0;
        float prevCx = 0.0f, prevCy = 0.0f;
        bool havePrev = false;
        for (int i = 0; i < steps; ++i) {
            sim.step();
            sim.snapshot(snap);
            float cx = 0.0f, cy = 0.0f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) { cx += snap.pointsX[p]; cy += snap.pointsY[p]; }
            cx /= static_cast<float>(snap.pointsX.size());
            cy /= static_cast<float>(snap.pointsY.size());
            const float axX = snap.pointsX.front() - snap.pointsX.back();
            const float axY = snap.pointsY.front() - snap.pointsY.back();
            const float ang = std::atan2(axY, axX);
            const int phase = sim.debugLocomotionPhase();
            if (havePrev && i > 300) {
                const float alen = std::sqrt(axX * axX + axY * axY);
                const float ux = alen > 1e-6f ? axX / alen : 1.0f;
                const float uy = alen > 1e-6f ? axY / alen : 0.0f;
                phaseFwd[phase] += ((cx - prevCx) * ux + (cy - prevCy) * uy) / dt / kBodyLength;
                phaseSteps[phase]++;
            }
            // УГОЛ ПИРУЭТА меряется как угол между НАПРАВЛЕНИЕМ ДВИЖЕНИЯ до и
            // после эпизода, а не по оси тела в момент поворота: во время
            // глубокого изгиба ось голова-хвост физически бессмысленна (тело
            // сложено), а для навигации важно ровно одно - куда червь поехал
            // после. Это же величина, которую меряют Pierce-Shimomura et al.
            // (угол между соседними пробегами).
            centroidTrack.push_back({cx, cy});
            if (phase != prevPhase) {
                const float durSec = static_cast<float>(i - phaseStartStep) * dt;
                if (i > 300) {
                    if (prevPhase == 1) reverseRuns.push_back(durSec);
                    else if (prevPhase == 0) forwardRuns.push_back(durSec);
                }
                if (prevPhase == 0 && phase == 1) episodeStart = static_cast<int>(centroidTrack.size()) - 1;
                if (phase == 0 && prevPhase != 0 && episodeStart > 0) pendingEnd = static_cast<int>(centroidTrack.size()) - 1;
                phaseStartStep = i;
                axAtPhaseStart = ang;
                prevPhase = phase;
            }
            // Направление после эпизода берётся, когда после него набралось
            // kRunWindow шагов чистого хода вперёд.
            if (pendingEnd > 0 && static_cast<int>(centroidTrack.size()) - pendingEnd > kRunWindow
                && phase == 0) {
                const auto dirOf = [&](int a, int b) {
                    return std::atan2(centroidTrack[b].y - centroidTrack[a].y,
                                      centroidTrack[b].x - centroidTrack[a].x);
                };
                if (episodeStart - kRunWindow >= 0) {
                    const float before = dirOf(episodeStart - kRunWindow, episodeStart);
                    const float after = dirOf(pendingEnd, pendingEnd + kRunWindow);
                    float d = after - before;
                    while (d > 3.14159265f) d -= 6.2831853f;
                    while (d < -3.14159265f) d += 6.2831853f;
                    omegaTurns.push_back(std::fabs(d) * 57.29578f);
                }
                pendingEnd = -1;
                episodeStart = -1;
            }
            prevCx = cx; prevCy = cy; havePrev = true;
        }
        auto stat = [](std::vector<float>& v, const char* name) {
            if (v.empty()) { std::printf("  %-16s n=0\n", name); return; }
            std::sort(v.begin(), v.end());
            double s = 0.0;
            for (float x : v) s += x;
            std::printf("  %-16s n=%3zu mean=%8.2f median=%8.2f min=%7.2f max=%7.2f\n", name, v.size(),
                        s / v.size(), v[v.size() / 2], v.front(), v.back());
        };
        const char* names[3] = {"forward", "reverse", "omega"};
        const double total = static_cast<double>(phaseSteps[0] + phaseSteps[1] + phaseSteps[2]);
        for (int p = 0; p < 3; ++p)
            std::printf("  phase %-8s time=%5.1f%%  meanSignedFwd=%+.5f BL/s\n", names[p],
                        total > 0 ? 100.0 * phaseSteps[p] / total : 0.0,
                        phaseSteps[p] > 0 ? phaseFwd[p] / phaseSteps[p] : 0.0);
        stat(omegaTurns, "pirouette deg");
        stat(reverseRuns, "reverse sec");
        stat(forwardRuns, "forward run sec");
        return 0;
    }

    // ./exe imprint <seed> <steps> <gradientSlope> <startFrac> <imprintTau>
    // ТЕРМАЛЬНЫЙ ИМПРИНТИНГ (Hedgecock & Russell 1975) - единственная в модели
    // память, переживающая опыт. Проверяет ровно то, что делает память памятью:
    // запомненная комфортная температура T_c должна СЛЕДОВАТЬ за пережитой, а
    // не оставаться константой, и после переноса животного в другое место
    // должна туда переползти.
    //
    // Стенд: линейный градиент температуры по арене, червь стартует в заданной
    // её доле. Печатается T_c и текущая температура по времени.
    if (argc >= 2 && std::string(argv[1]) == "imprint") {
        const unsigned seed = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 12345u;
        const int steps = argc > 3 ? std::atoi(argv[3]) : 12000;
        const float slope = argc > 4 ? static_cast<float>(std::atof(argv[4])) : 0.002f;
        const float startFrac = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 0.25f;
        std::srand(seed);
        WormSim sim("worm_data/celegans_herm.connectome");
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = kDragAgar;
        sim.params.tempGradientSlope = slope;
        sim.params.tempGradientAngle = 0.0f;
        if (argc > 6) sim.params.thermalImprintTau = static_cast<float>(std::atof(argv[6]));
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        // Червь начинает в холодной или тёплой части арены - от этого и зависит,
        // какую температуру он "проживёт" и, значит, запомнит.
        sim.teleport(glm::vec2(kFieldCols * kHexSpacing * startFrac, kFieldRows * kHexSpacing * 0.5f));
        const float dt = sim.params.dt.load();
        std::printf("imprint: seed=%u steps=%d slope=%.4f startFrac=%.2f tau=%.1f T_c(0)=%.3f\n", seed, steps,
                    slope, startFrac, sim.params.thermalImprintTau.load(), sim.params.cultivationTemp.load());
        WormSim::Snapshot snap;
        for (int i = 0; i < steps; ++i) {
            sim.step();
            if (i % 1000 == 0 || i == steps - 1) {
                sim.snapshot(snap);
                std::printf("  t=%6.1fs  T_c=%.4f  T_here=%.4f  x=%.0f\n", i * dt,
                            sim.params.cultivationTemp.load(), sim.temperatureAt(snap.pointsX[0], snap.pointsY[0]),
                            snap.pointsX[0]);
            }
        }
        return 0;
    }

    if (argc >= 2 && std::string(argv[1]) == "burst") {
        const unsigned seed = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 12345u;
        const int steps = argc > 3 ? std::atoi(argv[3]) : 3000;
        const float dragNormal = argc > 4 ? static_cast<float>(std::atof(argv[4])) : kDragAgar;
        const float threshMult = argc > 5 ? static_cast<float>(std::atof(argv[5])) : 5.0f;
        V2Point pt;
        pt.muscleLeakScale = 50.0f;
        pt.muscleCalciumTau = 0.05f;
        pt.bodyPoseDecayRate = 3.0f;
        pt.motorPositionSource = 1;
        pt.proprioceptiveAnterior = 1;
        pt.driveEqualizationGain = 1.0f;
        pt.jointAngleClamp = 0.55f;
        pt.mediumAmplitudeWaterRatio = 0.45f;
        if (argc > 6) pt.bodyGain = static_cast<float>(std::atof(argv[6]));
        if (argc > 7) pt.bodyPoseDecayRate = static_cast<float>(std::atof(argv[7]));
        if (argc > 8) pt.leakScale = static_cast<float>(std::atof(argv[8]));
        if (argc > 9) pt.chemGain = static_cast<float>(std::atof(argv[9]));
        if (argc > 10) pt.gapGain = static_cast<float>(std::atof(argv[10]));
        if (argc > 11) pt.mediumAmplitudeWaterRatio = static_cast<float>(std::atof(argv[11]));
        // argv[12]/argv[13]: jointAngleRateLimit/muscleTargetTau - режим burst
        // появился ДО обоих механизмов и поэтому смотрел только кинематическую
        // формулировку. Выброс в отгруженном режиме нужно ловить именно при
        // поднятом пределе скорости сустава: измерено, что f*A == rateLimit/4 с
        // точностью 5% на пяти независимых точках, то есть предел скорости и
        // есть весь потолок "амплитуда x частота", а выбросы - единственное,
        // что мешает его поднять.
        pt.jointAngleRateLimit = 0.25f;
        pt.muscleTargetTau = 0.1f;
        if (argc > 12) pt.jointAngleRateLimit = static_cast<float>(std::atof(argv[12]));
        if (argc > 13) pt.muscleTargetTau = static_cast<float>(std::atof(argv[13]));
        // argv[14]: множитель размера арены. Проверка гипотезы "выброс - это
        // разворот у стены, а не локомоция": containBody поворачивает курс со
        // скоростью 3 рад/с ВОКРУГ ГОЛОВЫ, значит центроид (~0.4 длины тела
        // позади головы) при этом движется вбок со скоростью ~0.4*3 = 1.2
        // длины тела в секунду. Если выбросы исчезают на арене, до стен которой
        // червь за прогон не доходит, - метрика выбросов меряла столкновения со
        // стеной, а не качество хода.
        const int arenaScale = argc > 14 ? std::max(1, std::atoi(argv[14])) : 1;
        // argv[15]: dragSettleGain. Режим burst (как и V2Point) по умолчанию
        // ставил 0, тогда как демо отгружает 25 - то есть вся диагностика шла не
        // в том режиме трения, в котором червя видит владелец проекта.
        if (argc > 15) pt.dragSettleGain = static_cast<float>(std::atof(argv[15]));

        WormSim sim("worm_data/celegans_herm.connectome");
        applyPoint(sim, pt);
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = dragNormal;
        std::srand(seed);
        sim.setBounds(glm::vec2(0.0f), kFieldCols * arenaScale, kFieldRows * arenaScale, kHexSpacing);
        const float dt = sim.params.dt.load();
        std::printf("burst diag: seed=%u steps=%d dragNormal=%.2f thresh=%.1fx bodyGain=%.0f decay=%.2f "
                    "leak=%.2f chem=%.4f gap=%.4f waterRatio=%.3f rateLimit=%.3f targetTau=%.3f arenaScale=%d\n",
                    seed, steps, dragNormal, threshMult, sim.params.bodyGain.load(),
                    sim.params.bodyPoseDecayRate.load(), sim.params.leakScale.load(), sim.params.chemGain.load(),
                    sim.params.gapGain.load(), sim.params.mediumAmplitudeWaterRatio.load(),
                    sim.params.jointAngleRateLimit.load(), sim.params.muscleTargetTau.load(), arenaScale);

        WormSim::Snapshot snap;
        // fwd/lat - разложение шага центроида на ось "хвост -> голова" и
        // перпендикуляр. Выброс тяги и выброс бокового рывка - разные поломки, а
        // модуль скорости их не различает.
        // maxRate/satFrac - максимальная и доля насыщенных |dtheta/dt| по 24
        // суставам за этот шаг: проверяет, действительно ли выброс совпадает с
        // быстрым изменением формы, или форма меняется штатно, а скачет решение
        // баланса сил.
        struct Rec { int step; float speed, fwd, lat, dHead, headSeg, det, rhs, load, maxAngle, maxDev, coiled, maxRate, satFrac; int phase; };
        std::vector<Rec> hist;
        hist.reserve(static_cast<std::size_t>(steps) + 1);
        float prevCx = 0.0f, prevCy = 0.0f, prevAxX = 1.0f, prevAxY = 0.0f;
        float prevHeadSeg = 0.0f;
        bool havePrevHeadSeg = false;
        bool havePrev = false;
        std::vector<float> prevAngles;
        const float rateLimit = sim.params.jointAngleRateLimit.load();
        for (int i = 0; i < steps; ++i) {
            sim.step();
            sim.snapshot(snap);
            float cx = 0.0f, cy = 0.0f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) { cx += snap.pointsX[p]; cy += snap.pointsY[p]; }
            cx /= static_cast<float>(snap.pointsX.size());
            cy /= static_cast<float>(snap.pointsY.size());
            // Ось тела: хвост -> голова. points_[0] - голова (см. rebuild_points).
            float axX = snap.pointsX.front() - snap.pointsX.back();
            float axY = snap.pointsY.front() - snap.pointsY.back();
            const float axLen = std::sqrt(axX * axX + axY * axY);
            if (axLen > 1e-6f) { axX /= axLen; axY /= axLen; } else { axX = 1.0f; axY = 0.0f; }
            float sp = 0.0f, fwd = 0.0f, lat = 0.0f, dHead = 0.0f;
            if (havePrev) {
                const float dx = cx - prevCx, dy = cy - prevCy;
                sp = std::sqrt(dx * dx + dy * dy) / dt / kBodyLength;
                fwd = (dx * axX + dy * axY) / dt / kBodyLength;
                lat = (dx * -axY + dy * axX) / dt / kBodyLength;
                // Скорость поворота оси тела в МИРОВЫХ координатах. Проверяет
                // гипотезу "рывок = разворот у стены": ограничитель в
                // containBody стоит на 3 рад/с.
                dHead = std::atan2(prevAxX * axY - prevAxY * axX, prevAxX * axX + prevAxY * axY) / dt;
            }
            prevCx = cx; prevCy = cy; prevAxX = axX; prevAxY = axY; havePrev = true;
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
            for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            const std::vector<float>& ang = sim.debugBodyAngles();
            float maxAngle = 0.0f;
            for (float a : ang) maxAngle = std::max(maxAngle, std::fabs(a));
            float maxRate = 0.0f;
            int satCount = 0;
            if (prevAngles.size() == ang.size() && dt > 0.0f) {
                for (std::size_t j = 0; j < ang.size(); ++j) {
                    const float r = std::fabs(ang[j] - prevAngles[j]) / dt;
                    maxRate = std::max(maxRate, r);
                    if (rateLimit > 0.0f && r > 0.98f * rateLimit) satCount++;
                }
            }
            const float satFrac = ang.empty() ? 0.0f : static_cast<float>(satCount) / static_cast<float>(ang.size());
            prevAngles = ang;
            float maxDev = 0.0f;
            for (float d : sim.lastCurvatureDeviation()) maxDev = std::max(maxDev, std::fabs(d));
            // headSeg - скорость поворота ПЕРВОГО СЕГМЕНТА в мировых координатах.
            // Именно её меряет гейт implausible-heading-whip, и именно она
            // бракует воду. Она равна сумме жёсткого вращения тела и скорости
            // первого сустава, поэтому рядом печатается maxRate: если headSeg
            // намного больше него, виновато вращение, а не сустав.
            const float headSegNow =
                std::atan2(snap.pointsY[1] - snap.pointsY[0], snap.pointsX[1] - snap.pointsX[0]);
            float headSeg = 0.0f;
            if (havePrevHeadSeg) {
                float d = headSegNow - prevHeadSeg;
                while (d > 3.14159265f) d -= 6.2831853f;
                while (d < -3.14159265f) d += 6.2831853f;
                headSeg = d / dt;
            }
            prevHeadSeg = headSegNow; havePrevHeadSeg = true;
            hist.push_back({i, sp, fwd, lat, dHead, headSeg, sim.debugPropulsionDeterminant(),
                            sim.debugPropulsionRhsMagnitude(), sim.debugMechanicalLoad(), maxAngle, maxDev,
                            diag / kBodyLength, maxRate, satFrac, sim.debugLocomotionPhase()});
        }
        // Медиана по устоявшейся части.
        std::vector<float> sp;
        for (std::size_t i = 500; i < hist.size(); ++i) sp.push_back(hist[i].speed);
        if (sp.empty()) { std::printf("too few steps\n"); return 0; }
        std::sort(sp.begin(), sp.end());
        const float median = sp[sp.size() / 2];
        std::printf("median centroid speed (post-warmup) = %.5f BL/s ; threshold = %.5f\n", median,
                    median * threshMult);

        // СВОДКА ПО ВСЕМУ УСТОЯВШЕМУСЯ УЧАСТКУ, а не только вокруг выбросов.
        // Дамп вокруг пика показывает, ЧТО происходит в момент выброса, но не
        // отвечает, отличается ли этот момент от обычного шага. Корреляция
        // speed<->maxRate и доля пути, набранная на 1% самых быстрых шагов,
        // отвечают. Плюс средние |fwd|/|lat|: тяга вдоль тела и боковой рывок -
        // разные поломки.
        {
            double sSp = 0, sRt = 0, sSp2 = 0, sRt2 = 0, sSpRt = 0, sFwd = 0, sLat = 0, sSat = 0;
            std::size_t n = 0, turning = 0;
            for (std::size_t i = 500; i < hist.size(); ++i) {
                const Rec& r = hist[i];
                sSp += r.speed; sRt += r.maxRate; sSp2 += double(r.speed) * r.speed;
                sRt2 += double(r.maxRate) * r.maxRate; sSpRt += double(r.speed) * r.maxRate;
                sFwd += r.fwd; sLat += std::fabs(r.lat); sSat += r.satFrac;
                if (std::fabs(r.dHead) > 1.0f) turning++;  // kWallTurnRate=3.0, треть от него
                n++;
            }
            const double mSp = sSp / n, mRt = sRt / n;
            const double cov = sSpRt / n - mSp * mRt;
            const double sd1 = std::sqrt(std::max(1e-30, sSp2 / n - mSp * mSp));
            const double sd2 = std::sqrt(std::max(1e-30, sRt2 / n - mRt * mRt));
            std::vector<float> sorted = sp;  // уже отсортирован
            const std::size_t top = std::max<std::size_t>(1, sorted.size() / 100);
            double topSum = 0, allSum = 0;
            for (std::size_t i = 0; i < sorted.size(); ++i) {
                allSum += sorted[i];
                if (i >= sorted.size() - top) topSum += sorted[i];
            }
            // sd2 обнуляется, когда предел скорости сустава связывает КАЖДЫЙ шаг
            // (maxRate тождественно равен пределу) - тогда корреляция не
            // определена, и печатать вместо неё нужно это, а не мусор от 0/0.
            char corrBuf[32];
            if (sd2 < 1e-6) std::snprintf(corrBuf, sizeof(corrBuf), "n/a(pinned)");
            else std::snprintf(corrBuf, sizeof(corrBuf), "%+.3f", cov / (sd1 * sd2));
            std::printf("summary: meanSpeed=%.5f meanFwd=%+.5f mean|lat|=%.5f meanMaxRate=%.4f sd(maxRate)=%.5f "
                        "meanSatFrac=%.3f corr(speed,maxRate)=%s turnFrac=%.4f p99share=%.3f p99/median=%.1fx\n",
                        mSp, sFwd / n, sLat / n, mRt, sd2, sSat / n, corrBuf,
                        static_cast<double>(turning) / static_cast<double>(n),
                        allSum > 0 ? topSum / allSum : 0.0,
                        median > 0 ? sorted[sorted.size() - top] / median : 0.0f);
        }

        // Отдельная сводка по гейту, который реально бракует воду
        // (implausible-heading-whip). headSeg - скорость поворота ПЕРВОГО
        // СЕГМЕНТА, то есть сумма жёсткого вращения тела и скорости первого
        // сустава. Если она сильно больше maxRate, виновато вращение, а не
        // сустав, и чинить надо не предел скорости.
        {
            float worst = 0.0f;
            int worstStep = -1, worstPhase = 0;
            float worstRate = 0.0f;
            for (std::size_t i = 500; i < hist.size(); ++i)
                if (std::fabs(hist[i].headSeg) > worst) {
                    worst = std::fabs(hist[i].headSeg);
                    worstStep = hist[i].step;
                    worstPhase = hist[i].phase;
                    worstRate = hist[i].maxRate;
                }
            const char* pn[3] = {"forward", "reverse", "omega"};
            std::printf("worst head-segment rotation: %.3f rad/s (%.4f rad/step) at step %d, phase=%s, "
                        "jointMaxRate=%.3f rad/s\n",
                        worst, worst * dt, worstStep, pn[std::clamp(worstPhase, 0, 2)], worstRate);
        }
        std::printf("%6s %10s %10s %10s %9s %9s %8s %9s %8s %7s\n", "step", "speed", "fwd", "lat", "dHead",
                    "headSeg", "coiled", "maxRate", "satFrac", "phase");
        int shown = 0;
        for (std::size_t i = 502; i + 2 < hist.size() && shown < 40; ++i) {
            if (hist[i].speed <= median * threshMult) continue;
            for (std::size_t j = i - 2; j <= i + 2; ++j) {
                const Rec& r = hist[j];
                const char* pn2[3] = {"fwd", "rev", "omg"};
                std::printf("%6d %10.5f %+10.5f %+10.5f %+9.4f %+9.4f %8.3f %9.4f %8.3f %7s%s\n", r.step,
                            r.speed, r.fwd, r.lat, r.dHead, r.headSeg, r.coiled, r.maxRate, r.satFrac,
                            pn2[std::clamp(r.phase, 0, 2)], (j == i ? "  <== BURST" : ""));
            }
            std::printf("\n");
            shown++;
            i += 2;
        }
        if (shown == 0) std::printf("no burst above %.1fx median in this run\n", threshMult);
        return 0;
    }

    if (argc >= 2 && std::string(argv[1]) == "trace") {
        const unsigned seed = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 12345u;
        const int steps = argc > 3 ? std::atoi(argv[3]) : 600;
        const float dragNormal = argc > 4 ? static_cast<float>(std::atof(argv[4])) : kDragAgar;
        const float bodyGain = argc > 5 ? static_cast<float>(std::atof(argv[5])) : -1.0f;
        // argv[6]/argv[7]: mechanoGain/localMechanoGain - added to trace the
        // per-segment SPATIAL PHASE of the curvature wave (not just its
        // amplitude range) at a point where localMechanoGain leaves water
        // healthy on coiled/heading but consistently failing on path
        // efficiency (see WORM_V2_MECHANOSENSATION_JOINT_RESULTS.md section
        // 6, hypothesis: traveling-wave phase coherence collapses in water
        // at low drag, not raw instability).
        const float mechanoGain = argc > 6 ? static_cast<float>(std::atof(argv[6])) : 0.0f;
        const float localMechanoGain = argc > 7 ? static_cast<float>(std::atof(argv[7])) : 0.0f;
        // argv[8]: if "1", dump the FULL per-position dev[] vector every
        // step (not just min/max/meanAbs) - lets you see whether the
        // curvature pattern's peak PROPAGATES along the 24 body positions
        // over time (a coherent traveling wave) or stays fixed/reverses
        // (a standing-wave / phase-incoherent pattern, which would explain
        // good local oscillation + good shape health + poor net forward
        // translation all at once).
        const bool dumpPhase = argc > 8 && std::string(argv[8]) == "1";
        // argv[9]: bodyBendStiffness (WORM_V3_DESIGN.md section 2) - added for
        // the Rieser et al. 2024 geometric-phase diagnostic (section 5.4):
        // compares enclosed-area (PCA + shoelace formula) BEFORE (0) and
        // AFTER (candidate value) bend stiffness on the SAME angles() dump.
        const float bodyBendStiffness = argc > 9 ? static_cast<float>(std::atof(argv[9])) : 0.0f;
        // argv[10]/argv[11]: proprioceptiveOffset/proprioceptiveGain
        // (WORM_V4_DESIGN.md section 8.4) - re-run the same geometric-phase
        // diagnostic at the candidate posterior stretch-receptor window
        // width, on the localMechanoGain=0.2 motivating point. <=0 leaves
        // WormSim's own shipped default (4.0/4.0) untouched.
        const float proprioceptiveOffset = argc > 10 ? static_cast<float>(std::atof(argv[10])) : -1.0f;
        const float proprioceptiveGain = argc > 11 ? static_cast<float>(std::atof(argv[11])) : -1.0f;
        // argv[12]/argv[13]: kickStep/kickMagnitude (WORM_V5_KICKTEST_
        // RESULTS.md, Phase 4 - limit-cycle perturbation diagnostic). At
        // step==kickStep (1-indexed against the loop counter i below, so
        // kickStep=2000 fires right after the step-2000 iteration, deep in
        // the settled plateau per WORM_V5_RESULTS.md), inject a one-time
        // random-sign voltage kick of kickMagnitude into every dorsal AND
        // ventral motor neuron (WormSim::debugKickMotorNeurons - direct
        // state_ perturbation, NOT set_input/add_input, so it decays under
        // the network's own unperturbed dynamics afterward, not a persistent
        // forcing term). kickStep<=0 (default) => never kicks, bitwise prior
        // behavior. The kick's own RNG is seeded from the run's seed (offset
        // by a constant) - independent of, and does not disturb, the global
        // std::rand() stream std::srand(seed) below seeds for the rest of
        // the simulation's intrinsic noise.
        const int kickStep = argc > 12 ? std::atoi(argv[12]) : 0;
        const float kickMagnitude = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 0.0f;
        // argv[14]/argv[15]/argv[16]: muscleLeakScale/muscleCalciumTau/
        // bodyPoseDecayRate overrides - next free slots after kickMagnitude's
        // argv[13]. <=0 leaves WormSim's own shipped default untouched (same
        // sentinel convention as bodyGain/activationSlope/proprioceptiveOffset/
        // proprioceptiveGain above), preserving bitwise-identical behavior for
        // every prior invocation that didn't pass these. Added for the
        // WORM_V5_REAL_AMPLITUDE_CALIBRATION.md joint-recalibration phase -
        // "point" mode already lets these three vary, but the dumpPhase=1
        // traveling-wave-vs-stuck-joint diagnostic (WORM_V3_DESIGN.md section
        // 5.4, already used by WORM_V5_MUSCLE_LEAK_RESULTS.md section 8's
        // still-open question) only exists in "trace" mode, which previously
        // could only ever inspect the three shipped defaults.
        const float muscleLeakScale = argc > 14 ? static_cast<float>(std::atof(argv[14])) : -1.0f;
        const float muscleCalciumTau = argc > 15 ? static_cast<float>(std::atof(argv[15])) : -1.0f;
        const float bodyPoseDecayRateArg = argc > 16 ? static_cast<float>(std::atof(argv[16])) : -1.0f;
        // argv[17]: jointAngleClamp override (WORM_V5_JOINT_CLAMP_RESULTS.md)
        // - next free slot after bodyPoseDecayRateArg's argv[16]. <=0 leaves
        // WormSim's own shipped default (0.25 rad) untouched (same sentinel
        // convention as the three overrides just above), preserving
        // bitwise-identical behavior for every prior invocation that didn't
        // pass this. Lets the dumpPhase=1 clamp-saturation diagnostic
        // (WORM_V3_DESIGN.md section 5.4) inspect any joint-clamp value, not
        // just the shipped 0.25.
        const float jointAngleClamp = argc > 17 ? static_cast<float>(std::atof(argv[17])) : -1.0f;
        // argv[18]/argv[19]: jointSoftSaturation/jointStiffeningGain
        // (WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md) - next free slots after
        // jointAngleClamp's argv[17]. Both 0.0 by default (already the off
        // state - bitwise-identical hard clamp at jointAngleClamp), same
        // no-sentinel-needed pattern as bodyBendStiffness above.
        const float jointSoftSaturation = argc > 18 ? static_cast<float>(std::atof(argv[18])) : 0.0f;
        const float jointStiffeningGain = argc > 19 ? static_cast<float>(std::atof(argv[19])) : 0.0f;
        std::srand(seed);
        WormSim sim("worm_data/celegans_herm.connectome");
        sim.params.dragTangent = 1.0f;
        sim.params.dragNormal = dragNormal;
        if (bodyGain > 0.0f) sim.params.bodyGain = bodyGain;
        sim.params.mechanoGain = mechanoGain;
        sim.params.localMechanoGain = localMechanoGain;
        sim.params.bodyBendStiffness = bodyBendStiffness;
        if (proprioceptiveOffset > 0.0f) sim.params.proprioceptiveOffset = proprioceptiveOffset;
        if (proprioceptiveGain > 0.0f) sim.params.proprioceptiveGain = proprioceptiveGain;
        if (muscleLeakScale > 0.0f) sim.params.muscleLeakScale = muscleLeakScale;
        if (muscleCalciumTau > 0.0f) sim.params.muscleCalciumTau = muscleCalciumTau;
        if (bodyPoseDecayRateArg > 0.0f) sim.params.bodyPoseDecayRate = bodyPoseDecayRateArg;
        if (jointAngleClamp > 0.0f) sim.params.jointAngleClamp = jointAngleClamp;
        sim.params.jointSoftSaturation = jointSoftSaturation;
        sim.params.jointStiffeningGain = jointStiffeningGain;
        sim.setBounds(glm::vec2(0.0f), kFieldCols, kFieldRows, kHexSpacing);
        std::printf("trace: seed=%u steps=%d dragNormal=%.2f muscleLeak=%.2f muscleCaTau=%.2f "
                    "poseDecay=%.2f frameRateHz=%.2f bendStiffness=%.2f propOffset=%.2f propGain=%.2f "
                    "kickStep=%d kickMagnitude=%.3f jointClamp=%.3f jointSoftSat=%.3f jointStiffGain=%.4f\n",
                    seed, steps, dragNormal, sim.params.muscleLeakScale.load(), sim.params.muscleCalciumTau.load(),
                    sim.params.bodyPoseDecayRate.load(), sim.params.bodyFrameRateLimitHz.load(),
                    sim.params.bodyBendStiffness.load(), sim.params.proprioceptiveOffset.load(),
                    sim.params.proprioceptiveGain.load(), kickStep, kickMagnitude,
                    sim.params.jointAngleClamp.load(), sim.params.jointSoftSaturation.load(),
                    sim.params.jointStiffeningGain.load());
        connectome::Network& net = sim.network();
        for (int i = 0; i < steps; ++i) {
            sim.step();
            // Phase 4 limit-cycle perturbation (WORM_V5_KICKTEST_RESULTS.md):
            // fires exactly once, right after the iteration whose 1-indexed
            // step number equals kickStep (kickStep<=0 => never, bitwise
            // prior behavior for every existing invocation that doesn't pass
            // argv[12]/[13]). RNG seed is derived from the run seed, offset
            // by a large odd constant, so it's independent of (and doesn't
            // perturb) the global std::rand() stream already seeded above.
            if (kickStep > 0 && (i + 1) == kickStep) {
                sim.debugKickMotorNeurons(kickMagnitude, seed + 2654435761u);
                std::printf("KICK applied at step %d: magnitude=%.3f\n", i + 1, kickMagnitude);
            }
            if (dumpPhase) {
                // dev[] (network curvature output, pre-body) AND angles()
                // (real, post-kinematics joint angle - what the Rieser et al.
                // 2024 geometric-phase diagnostic in WORM_V3_DESIGN.md section
                // 5.4 actually needs, since bend stiffness acts on angles_,
                // not on curvature/dev directly) - two lines per step, tagged
                // so an offline script can tell them apart.
                const auto& dev = sim.lastCurvatureDeviation();
                std::printf("phase %4d:", i);
                for (float d : dev) std::printf(" %+.3f", d);
                std::printf("\n");
                const auto& ang = sim.debugBodyAngles();
                std::printf("angles %4d:", i);
                for (float a : ang) std::printf(" %+.4f", a);
                std::printf("\n");
                // РЕАЛИЗОВАННЫЙ угол сустава - разность соседних frame_heading_
                // (см. WormSim::debugFrameHeading). Совпадает с "angles" выше
                // ровно до тех пор, пока ограничитель скорости ориентации
                // (Params::bodyFrameRateLimitHz) не начинает связывать; всё
                // расхождение между двумя строками - это команда сети, которая
                // до реальной формы тела не доехала.
                const auto& fh = sim.debugFrameHeading();
                std::printf("realized %4d:", i);
                for (std::size_t j = 0; j < fh.size(); ++j)
                    std::printf(" %+.4f", j == 0 ? fh[0] : (fh[j] - fh[j - 1]));
                std::printf("\n");
                // Raw per-position curvature BEFORE the m_curvatureBaseline
                // temporal high-pass (post-bodyGain, post-spatial-mean-removal)
                // - added to distinguish a genuine network/muscle-side decay of
                // the bend signal from a baseline-EMA cold-start transient (the
                // baseline starts at zero, so early "phase"/dev[] amplitude may
                // be inflated regardless of curvature[]'s own steady level -
                // see session notes on the ~0.25 rad -> ~0.02-0.04 rad collapse).
                const auto& rawCurv = sim.debugRawCurvature();
                std::printf("curvature %4d:", i);
                for (float c : rawCurv) std::printf(" %+.4f", c);
                std::printf("\n");
                // Raw per-position (d-v)*bodyGain from net.state() (motor
                // neuron V, BEFORE muscle_calcium_) - added to localize WHERE
                // a genuine (non-baseline-filter) amplitude collapse
                // originates: in the raw network/motor voltage itself, or
                // specifically introduced by the muscle_calcium_ low-pass
                // stage (see WORM_V5_BASELINE_TAU_RESULTS.md).
                const auto& rawVoltage = sim.debugRawMotorVoltage();
                std::printf("voltage %4d:", i);
                for (float vv : rawVoltage) std::printf(" %+.4f", vv);
                std::printf("\n");
                // Absolute (not d-v) dorsal/ventral avgState() per position -
                // added to test the "sigmoid saturation via DC/common-mode
                // drift" hypothesis directly: does the mean state level of
                // each group drift away from activationTheta over time in a
                // way that would flatten sigmoid'(V) and reduce the closed
                // loop's effective AC gain (see WORM_V5_SATURATION_RESULTS.md)?
                std::vector<float> dorsalState, ventralState;
                sim.debugRawDorsalVentralState(dorsalState, ventralState);
                std::printf("dorsalV %4d:", i);
                for (float dv : dorsalState) std::printf(" %+.4f", dv);
                std::printf("\n");
                std::printf("ventralV %4d:", i);
                for (float vv : ventralState) std::printf(" %+.4f", vv);
                std::printf("\n");
            }
            if (i % 20 == 0 || i == steps - 1) {
                const auto& dev = sim.lastCurvatureDeviation();
                float devMin = 1e9f, devMax = -1e9f, devAbsSum = 0.0f;
                for (float d : dev) { devMin = std::min(devMin, d); devMax = std::max(devMax, d); devAbsSum += std::fabs(d); }
                float muMin = 1e9f, muMax = -1e9f;
                for (connectome::NeuronId id = 0; id < net.size(); ++id) {
                    if (net.type(id) != connectome::NeuronType::Output) continue;
                    const float mo = net.muscle_output(id);
                    muMin = std::min(muMin, mo);
                    muMax = std::max(muMax, mo);
                }
                // Raw membrane-potential V range per neuron TYPE (not muscle_
                // output, which is already Ca-filtered for Output) - checks
                // whether the sigmoid activation sigmoid((V-theta)/slope),
                // theta=0/slope=1 default, is operating in its near-linear
                // regime (|V| small) or saturating (|V| large, derivative
                // ~0) for each layer, per WORM_V2_RESULTS.md's open question
                // about whether nonlinear saturation (not just muscleCalciumTau)
                // contributes to the frequency gap.
                float vMin[5] = {1e9f, 1e9f, 1e9f, 1e9f, 1e9f};
                float vMax[5] = {-1e9f, -1e9f, -1e9f, -1e9f, -1e9f};
                for (connectome::NeuronId id = 0; id < net.size(); ++id) {
                    const int t = static_cast<int>(net.type(id));
                    const float v = net.state(id);
                    vMin[t] = std::min(vMin[t], v);
                    vMax[t] = std::max(vMax[t], v);
                }
                std::printf("step %4d: dev[min=%.4f max=%.4f meanAbs=%.4f] muscleOut[min=%.4f max=%.4f] "
                            "load=%.3f\n",
                            i, devMin, devMax, devAbsSum / static_cast<float>(dev.size()), muMin, muMax,
                            sim.debugMechanicalLoad());
                std::printf("          V-range: In[%.3f,%.3f] InProc[%.3f,%.3f] Proc[%.3f,%.3f] "
                            "ProcOut[%.3f,%.3f] Out[%.3f,%.3f]\n",
                            vMin[0], vMax[0], vMin[1], vMax[1], vMin[2], vMax[2], vMin[3], vMax[3], vMin[4],
                            vMax[4]);
            }
        }
        return 0;
    }

    // ./exe point <muscleLeakScale> <muscleCalciumTau> <bodyPoseDecayRate> <bodyFrameRateLimitHz>
    //             <dragSettleTau> <dragSettleGain> <proprioceptiveDelaySeconds>
    //             [numBases] [seedsPerBase] [warmupSteps] [measureSteps]
    if (argc >= 2 && std::string(argv[1]) == "point") {
        V2Point pt;
        if (argc > 2) pt.muscleLeakScale = static_cast<float>(std::atof(argv[2]));
        if (argc > 3) pt.muscleCalciumTau = static_cast<float>(std::atof(argv[3]));
        if (argc > 4) pt.bodyPoseDecayRate = static_cast<float>(std::atof(argv[4]));
        if (argc > 5) pt.bodyFrameRateLimitHz = static_cast<float>(std::atof(argv[5]));
        if (argc > 6) pt.dragSettleTau = static_cast<float>(std::atof(argv[6]));
        if (argc > 7) pt.dragSettleGain = static_cast<float>(std::atof(argv[7]));
        if (argc > 8) pt.proprioceptiveDelaySeconds = static_cast<float>(std::atof(argv[8]));
        if (argc > 9) pt.bodyGain = static_cast<float>(std::atof(argv[9]));
        const int numBases = argc > 10 ? std::atoi(argv[10]) : 16;
        const int seedsPerBase = argc > 11 ? std::atoi(argv[11]) : 4;
        const int warmupSteps = argc > 12 ? std::atoi(argv[12]) : 300;
        const int measureSteps = argc > 13 ? std::atoi(argv[13]) : 2500;
        // Trailing optional args (kept AFTER the original 13 for backward
        // compatibility with every invocation already documented in
        // WORM_V2_RESULTS.md): mechanoGain, localMechanoGain, and a base-RNG
        // seed override. The seed was hardcoded to 31337 - meaning any two
        // calls with the same numBases always drew the identical base
        // sequence, so a "16-base confirmation" run twice was never a
        // genuinely independent re-check (found by the adversarial
        // verification agent, WORM_V2_RESULTS.md section 10.6). Defaulting
        // to 31337 preserves bitwise-identical behavior for every prior
        // invocation that didn't pass argv[16].
        if (argc > 14) pt.mechanoGain = static_cast<float>(std::atof(argv[14]));
        if (argc > 15) pt.localMechanoGain = static_cast<float>(std::atof(argv[15]));
        const unsigned baseRngSeed = argc > 16 ? static_cast<unsigned>(std::atoll(argv[16])) : 31337u;
        if (argc > 17) pt.activationSlope = static_cast<float>(std::atof(argv[17]));
        // argv[18]: bodyBendStiffness (WORM_V3_DESIGN.md section 2) - next
        // free slot after activationSlope's argv[17]. Default 0.0 preserves
        // bitwise-identical behavior for every prior invocation.
        if (argc > 18) pt.bodyBendStiffness = static_cast<float>(std::atof(argv[18]));
        // argv[19]/argv[20]: proprioceptiveOffset/proprioceptiveGain
        // (WORM_V4_DESIGN.md section 2) - next free slots after
        // bodyBendStiffness's argv[18]. <=0 leaves WormSim's own shipped
        // default (4.0/4.0) untouched, preserving bitwise-identical behavior
        // for every prior invocation that didn't pass these.
        if (argc > 19) pt.proprioceptiveOffset = static_cast<float>(std::atof(argv[19]));
        if (argc > 20) pt.proprioceptiveGain = static_cast<float>(std::atof(argv[20]));
        // argv[21]: jointAngleClamp (WORM_V5_JOINT_CLAMP_RESULTS.md) - next
        // free slot after proprioceptiveGain's argv[20]. <=0 leaves
        // WormSim's own shipped default (0.25 rad) untouched, preserving
        // bitwise-identical behavior for every prior invocation that didn't
        // pass this.
        if (argc > 21) pt.jointAngleClamp = static_cast<float>(std::atof(argv[21]));
        // argv[22]/argv[23]: jointSoftSaturation/jointStiffeningGain
        // (WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md) - next free slots after
        // jointAngleClamp's argv[21]. Both default to 0.0, which is already
        // the off state (bitwise-identical hard clamp), so no <=0 sentinel
        // gating is needed here (same pattern as bodyBendStiffness above).
        if (argc > 22) pt.jointSoftSaturation = static_cast<float>(std::atof(argv[22]));
        if (argc > 23) pt.jointStiffeningGain = static_cast<float>(std::atof(argv[23]));
        // argv[24]/argv[25]: bClassOscillatorGain/bClassOscillatorTauW - см.
        // V2Point за обоснованием. gain=0.0 (дефолт) - выключено тождественно,
        // побитово прежнее поведение для любого вызова, который их не передаёт;
        // tauW<=0 оставляет дефолт WormSim.
        if (argc > 24) pt.bClassOscillatorGain = static_cast<float>(std::atof(argv[24]));
        if (argc > 25) pt.bClassOscillatorTauW = static_cast<float>(std::atof(argv[25]));
        // argv[26]: motorPositionSource - 0 (дефолт) прежнее поведение.
        if (argc > 26) pt.motorPositionSource = std::atoi(argv[26]);
        // argv[27]: proprioceptiveAnterior - 0 (дефолт) прежнее поведение.
        if (argc > 27) pt.proprioceptiveAnterior = std::atoi(argv[27]);
        // argv[28]/argv[29]: driveEqualizationGain/Tau - см. V2Point.
        if (argc > 28) pt.driveEqualizationGain = static_cast<float>(std::atof(argv[28]));
        if (argc > 29) pt.driveEqualizationTau = static_cast<float>(std::atof(argv[29]));
        // argv[30]: mediumAmplitudeWaterRatio - см. V2Point.
        if (argc > 30) pt.mediumAmplitudeWaterRatio = static_cast<float>(std::atof(argv[30]));
        // argv[31]/[32]/[33]: leakScale/chemGain/gapGain - см. V2Point.
        if (argc > 31) pt.leakScale = static_cast<float>(std::atof(argv[31]));
        if (argc > 32) pt.chemGain = static_cast<float>(std::atof(argv[32]));
        if (argc > 33) pt.gapGain = static_cast<float>(std::atof(argv[33]));
        // argv[34]: jointAngleRateLimit - см. V2Point.
        if (argc > 34) pt.jointAngleRateLimit = static_cast<float>(std::atof(argv[34]));
        // argv[35]: mediumAmplitudeViaDecay - см. V2Point.
        if (argc > 35) pt.mediumAmplitudeViaDecay = std::atoi(argv[35]);
        // argv[36]: muscleTargetTau - см. V2Point.
        if (argc > 36) pt.muscleTargetTau = static_cast<float>(std::atof(argv[36]));
        // argv[37]: arenaScale - см. g_arenaScale. 1 (дефолт) - историческая
        // арена, побитово прежнее поведение для любого прежнего вызова.
        if (argc > 37) g_arenaScale = std::max(1, std::atoi(argv[37]));
        // argv[38]: mediumBendInternalFraction - см. V2Point.
        if (argc > 38) pt.mediumBendInternalFraction = static_cast<float>(std::atof(argv[38]));
        std::printf("point="); printPoint(pt);
        std::printf(" - %d bases x %d seeds, warmup=%d measure=%d, mechanoGain=%.3f localMechanoGain=%.3f baseSeed=%u\n",
                    numBases, seedsPerBase, warmupSteps, measureSteps, pt.mechanoGain, pt.localMechanoGain, baseRngSeed);
        std::mt19937 baseRng(baseRngSeed);
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int agarHealthyCount = 0, waterHealthyCount = 0, waterBeatsAgarCount = 0;
        double sumAgar = 0.0, sumWater = 0.0, sumAgarFreq = 0.0, sumWaterFreq = 0.0;
        int agarFreqCount = 0, waterFreqCount = 0;
        // Bend amplitude summary - see Measurement::minSustainedBendAmplitude/
        // AggregateResult::meanBendAmplitude above. Averaged across bases that
        // had at least one healthy trial (same gating as the freq averages
        // just above), purely additive to this summary line.
        double sumAgarBendAmp = 0.0, sumWaterBendAmp = 0.0;
        int agarBendAmpCount = 0, waterBendAmpCount = 0;
        // Peak (not floor) bend amplitude summary - see AggregateResult::
        // meanPeakBendAmplitude above. Same gating (>=1 healthy trial in the
        // base) as the existing min-based sums just above, purely additive.
        double sumAgarPeakBendAmp = 0.0, sumWaterPeakBendAmp = 0.0;
        int agarPeakBendAmpCount = 0, waterPeakBendAmpCount = 0;
        for (int b = 0; b < numBases; ++b) {
            const int base = baseDist(baseRng);
            const AggregateResult agar = evaluate(pt, seedsPerBase, base, kDragAgar, warmupSteps, measureSteps);
            const AggregateResult water = evaluate(pt, seedsPerBase, base, kDragWater, warmupSteps, measureSteps);
            const bool agarOk = isHealthy(agar), waterOk = isHealthy(water);
            const bool waterFaster = waterOk && agarOk && water.meanFreqHz > agar.meanFreqHz;
            std::printf("base=%d:\n", base);
            printAgg("agar", agar);
            printAgg("water", water);
            std::printf("  water>agar (freq): %s\n", waterFaster ? "YES" : "no");
            std::fflush(stdout);
            if (agarOk) { ++agarHealthyCount; sumAgar += agar.meanBLps; }
            if (agar.healthyCount > 0) {
                sumAgarFreq += agar.meanFreqHz; ++agarFreqCount;
                sumAgarBendAmp += agar.meanBendAmplitude; ++agarBendAmpCount;
                sumAgarPeakBendAmp += agar.meanPeakBendAmplitude; ++agarPeakBendAmpCount;
            }
            if (waterOk) { ++waterHealthyCount; sumWater += water.meanBLps; }
            if (water.healthyCount > 0) {
                sumWaterFreq += water.meanFreqHz; ++waterFreqCount;
                sumWaterBendAmp += water.meanBendAmplitude; ++waterBendAmpCount;
                sumWaterPeakBendAmp += water.meanPeakBendAmplitude; ++waterPeakBendAmpCount;
            }
            if (waterFaster) ++waterBeatsAgarCount;
        }
        const double meanAgarFreq = agarFreqCount ? sumAgarFreq / agarFreqCount : 0.0;
        const double meanWaterFreq = waterFreqCount ? sumWaterFreq / waterFreqCount : 0.0;
        const double meanAgarBendAmp = agarBendAmpCount ? sumAgarBendAmp / agarBendAmpCount : 0.0;
        const double meanWaterBendAmp = waterBendAmpCount ? sumWaterBendAmp / waterBendAmpCount : 0.0;
        const double meanAgarPeakBendAmp = agarPeakBendAmpCount ? sumAgarPeakBendAmp / agarPeakBendAmpCount : 0.0;
        const double meanWaterPeakBendAmp = waterPeakBendAmpCount ? sumWaterPeakBendAmp / waterPeakBendAmpCount : 0.0;
        std::printf("\nSummary over %d bases: agar healthy=%d/%d (mean %.5f BL/s, %.4fHz), water healthy=%d/%d "
                    "(mean %.5f BL/s, %.4fHz), water>agar(freq) in %d/%d bases, ratio(freq)=%.3f\n",
                    numBases, agarHealthyCount, numBases, agarHealthyCount ? sumAgar / agarHealthyCount : 0.0,
                    meanAgarFreq, waterHealthyCount, numBases, waterHealthyCount ? sumWater / waterHealthyCount : 0.0,
                    meanWaterFreq, waterBeatsAgarCount, numBases, meanAgarFreq > 1e-9 ? meanWaterFreq / meanAgarFreq : 0.0);
        // Additive new line - bend amplitude summary (see
        // WORM_V5_MUSCLE_LEAK_RESULTS.md). Does not alter anything printed
        // above.
        std::printf("Bend amplitude (mean minSustainedBendAmplitude over bases with >=1 healthy trial): "
                    "agar=%.4f rad (n=%d/%d) water=%.4f rad (n=%d/%d)\n",
                    meanAgarBendAmp, agarBendAmpCount, numBases, meanWaterBendAmp, waterBendAmpCount, numBases);
        // Peak bend amplitude - see AggregateResult::meanPeakBendAmplitude
        // above (WORM_V5_REAL_AMPLITUDE_CALIBRATION.md). Purely additive new
        // lines, printed after the existing min-based summary line so every
        // prior invocation's output is unchanged up to this point.
        std::printf("Peak bend amplitude (mean maxSustainedBendAmplitude over bases with >=1 healthy trial): "
                    "agar=%.4f rad (n=%d/%d) water=%.4f rad (n=%d/%d)\n",
                    meanAgarPeakBendAmp, agarPeakBendAmpCount, numBases, meanWaterPeakBendAmp, waterPeakBendAmpCount,
                    numBases);
        // Real-biology crawl target - see WORM_V5_REAL_AMPLITUDE_TARGET.md
        // section 5.2: two independently-derived peak per-joint estimates,
        // 0.494 rad (Karbowski et al. 2008, cross-check) and 0.589 rad
        // (Scenario III, Vidal-Gadea/Pierce-Shimomura peak-to-peak + 24-joint
        // discretization correction) bracket the most defensible crawl
        // target range. Printed as a plain ratio, no health/pass-fail
        // judgment baked in here - see WORM_V5_REAL_AMPLITUDE_CALIBRATION.md
        // for the honest interpretation.
        constexpr double kCrawlTargetLowRad = 0.494, kCrawlTargetHighRad = 0.589;
        std::printf("Real-biology crawl target (WORM_V5_REAL_AMPLITUDE_TARGET.md Sec.5.2): %.3f-%.3f rad peak "
                    "per-joint - agar peakBendAmp reaches %.1f%%-%.1f%% of that range\n",
                    kCrawlTargetLowRad, kCrawlTargetHighRad, 100.0 * meanAgarPeakBendAmp / kCrawlTargetHighRad,
                    100.0 * meanAgarPeakBendAmp / kCrawlTargetLowRad);
        std::printf("Old shipped point (v1, all 4 patches): agar=%.4fHz water=%.4fHz ratio=%.3f\n",
                    kOldShippedAgarHz, kOldShippedWaterHz, kOldShippedRatio);
        return 0;
    }

    std::printf("Usage:\n  %s trace [seed] [steps] [dragNormal] [bodyGain] [mechanoGain] [localMechanoGain] "
                "[dumpPhase=0/1] [bodyBendStiffness] [proprioceptiveOffset] [proprioceptiveGain] [kickStep] "
                "[kickMagnitude] [muscleLeakScale] [muscleCalciumTau] [bodyPoseDecayRate] [jointAngleClamp] "
                "[jointSoftSaturation] [jointStiffeningGain]\n"
                "  %s point <muscleLeakScale> <muscleCalciumTau> "
                "<bodyPoseDecayRate> <bodyFrameRateLimitHz> <dragSettleTau> <dragSettleGain> "
                "<proprioceptiveDelaySeconds> [bodyGain] [numBases] [seedsPerBase] [warmupSteps] [measureSteps] "
                "[mechanoGain] [localMechanoGain] [baseRngSeed] [activationSlope] [bodyBendStiffness] "
                "[proprioceptiveOffset] [proprioceptiveGain] [jointAngleClamp] [jointSoftSaturation] "
                "[jointStiffeningGain]\n",
                argv[0], argv[0]);
    return 0;
}
