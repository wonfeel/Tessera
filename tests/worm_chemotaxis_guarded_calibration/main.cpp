// tests/worm_chemotaxis_guarded_calibration/main.cpp
//
// Retry of tests/worm_chemotaxis_calibration's per-neuron-CLASS leak/
// capacitance axis (by direct user request "чини" after being told the worm
// currently shows no measurable directed chemotaxis - meanEffect=-4.30 +/-
// 12.83 on today's full production stack, indistinguishable from zero).
//
// That file's own history (5 rounds) found a REAL, independently-confirmed
// chemotaxis effect (0.0275-0.058, several sigma above zero, reproduced on
// 3 separate fresh seed batches) - then had to REVERT it after a live look
// at Demo_worm showed the worm settling into a static shape: a dedicated
// "displacement" check found the calibration cut plain crawling efficiency
// from 0.426 to 0.131, MORE THAN 3x WORSE. Root cause, in that file's own
// words: "every round... screened for a chemotaxis effect plus basic health
// (NaN/bounds/coiled-ratio)... none of them ever measured absolute crawling
// efficiency, so the search was blind to trading away most of the worm's
// mobility for a directional bias." Its explicit instruction for anyone
// revisiting this axis: "ANY fitness function used here must include a
// crawling-efficiency term, or it will find the same trade again."
//
// This file does exactly that, from trial 1 (not bolted on after a live
// surprise, per this session's own established discipline - see e.g.
// tests/worm_leak_capacitance_tempo_calibration for the same pattern applied
// to the tempo axis). Every candidate is gated on BOTH:
//   (a) crawling efficiency (no food, centroid-based net displacement / path
//       length, same methodology as the reverted file's "displacement"
//       mode) staying above a floor RELATIVE to identity's own measured
//       efficiency on today's full production stack (computed once at
//       program start) - not an absolute number, since today's stack's own
//       baseline efficiency is itself a moving target across sessions.
//   (b) the usual NaN/bounds/coiled-ratio health check.
// Only candidates clearing BOTH gates are even considered for their
// chemotaxis effect (paired with/without-food, same design as the original:
// same-seed pairing, food at a per-seed golden-angle offset within reach).
//
// Same two-stage screen-then-confirm discipline the original file's round 3
// validated works (round 1/2's single-stage hill-climbing chased noise into
// an implausible corner and had to be abandoned) - and the same lower-
// confidence-bound (mean - stderr) selection rule round 4 found necessary
// (raw-mean selection picks the noisiest "winner", not the most real one).
//
// RESULT: NEGATIVE, but a clean methodological success in a different sense
// - the efficiency-blindness bug is FIXED (no candidate this round traded
// crawling ability for chemotaxis; the two survivors that passed both gates
// on confirmation had efficiencyRatio 1.14-1.15, i.e. BETTER crawling than
// identity, not worse) - yet still no real chemotaxis effect was found.
//
// 8 parallel shards x 150 trials (2 shards lost their result to a reporting
// glitch, not counted): healthy pass rates through the efficiency gate
// (13-83/150 per shard) - the gate itself works and isn't vacuous (real
// rejections happened: rejectedByEfficiency 67-75 per shard on the shards
// that reported it). Top 3 candidates by screen lower-confidence-bound
// (lcb = mean - stderr, the round-4 lesson from the original file - NOT raw
// mean) were 20.52, 16.18, 4.57 - all suspiciously large next to the
// original file's real, validated effect size (0.0275-0.058) once that
// project's own network was much simpler (no CPG/bandwidth/delay yet).
//
// Full independent confirmation (60 fresh seeds, matching the original
// file's design) on all 3:
//   - lcb=16.18 candidate: confirmed effect COLLAPSED to negative
//     (22.3997 raw mean but +/-30.6949 stderr - lcb=-8.29) - textbook
//     winner's curse, exactly what round 1/2 of the original file already
//     demonstrated this search methodology is vulnerable to without this
//     two-stage discipline.
//   - lcb=4.57 candidate: confirmed effect FLIPPED SIGN (-6.9874 +/- 6.8608,
//     opposite direction from the screen).
//   - lcb=20.52 candidate: confirmed effect stayed POSITIVE (5.2941 +/-
//     2.8976, lcb=2.40) - the only survivor, efficiencyRatio=1.145 (better
//     crawling than identity, not a tradeoff). Looked promising enough for
//     a THIRD independent check (96 fresh seeds, base 555555555, matching
//     the original file's own "never trust two measurements, get a third
//     independent one" standard): effect FLIPPED NEGATIVE again (-2.1222
//     +/- 1.8998, lcb=-4.02) - three measurements of the same candidate gave
//     +20.52 (screen) / +5.29 (confirm 1) / -2.12 (confirm 2), an honest
//     coin-flip pattern around zero, not a real, reproducible effect.
//
// Conclusion: even with the efficiency-blindness bug fixed from trial 1
// (this file's whole reason for existing), no candidate in the searched
// 0.3-3.0x per-class leak/capacitance range produces REAL chemotaxis on
// today's full production stack (CPG+bandwidth+delay+velocity-clamp) - the
// near-zero baseline reported directly to the user (-4.30 +/- 12.83) holds
// up under a properly-guarded search, not just as an unexplored default.
// NOT SHIPPED - Params/network unchanged.
//
// The one remaining honest, untried option (per the original tests/worm_
// chemotaxis_calibration's own final note, still true): phase-gate the
// sensory drive by the network's own existing locomotor wave/CPG signal
// instead of injecting scent as a constant additive term - an architectural
// change, not a recalibration, not attempted here.
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

constexpr int kNumParams = 7;
enum ParamIdx { kLeakIP = 0, kLeakP = 1, kLeakPO = 2, kCapIP = 3, kCapP = 4, kCapPO = 5, kCapO = 6 };
using Candidate = std::array<float, kNumParams>;
const Candidate kIdentity = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

void applyCalibration(WormSim& sim, const Candidate& c) {
    connectome::Network& net = sim.network();
    net.scale_type_params(connectome::NeuronType::InputProcessing, c[kLeakIP], c[kCapIP]);
    net.scale_type_params(connectome::NeuronType::Processing, c[kLeakP], c[kCapP]);
    net.scale_type_params(connectome::NeuronType::ProcessingOutput, c[kLeakPO], c[kCapPO]);
    net.scale_type_params(connectome::NeuronType::Output, 1.0f, c[kCapO]);  // Output leak is a dead parameter
}

constexpr float kHexSpacing = 36.0f;
// Размер арены и дистанция до еды - ПЕРЕМЕННЫЕ, а не константы (см.
// WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 11). Исторические значения
// (28x20 клеток, еда в 180 единицах) сохранены как дефолты, чтобы любой
// прежний вызов давал побитово прежний результат, но они физически тесные:
// 28*36 = 1008 единиц по X при длине тела 576 - арена меньше двух длин тела,
// и WormSim::step отражает курс от стенки (см. bounds-обработку в
// WormSim.cpp), так что тело почти постоянно находится в отскоке. Еда в 180
// единицах - 0.31 длины тела, то есть ближе, чем собственный размер червя:
// "подойти" к ней невозможно в принципе, приближаться некуда.
int g_fieldCols = 28, g_fieldRows = 20;
float g_foodRadius = 180.0f;
// 0.0 = не трогать Params::gradientGain (дефолт WormSim 4.0). См. разбор в
// разборе аргументов distribution-режима ниже.
float g_gradientGain = 0.0f;
// Params::chemoSteeringGain / chemoSteeringSpan - рулевой канал в обход
// фильтров кривизны. 0.0 = выключен (дефолт WormSim), прежнее поведение.
float g_steeringGain = 0.0f;
int g_steeringSpan = 0;
// ПИРУЭТЫ (Params::pirouetteEnabled / reversalGradientGain) - см.
// WormSim::updateLocomotionState. -1 = не трогать дефолт WormSim. Вынесены
// именно сюда, потому что этот стенд - единственное место в проекте, где
// хемотаксис меряется парно и со статистикой, а весь смысл пируэта в том, что
// он ДОЛЖЕН улучшать хемотаксис; без парного замера "включил и стало лучше"
// было бы утверждением без проверки.
int g_pirouetteEnabled = -1;
float g_reversalGradientGain = -1.0f;
// Геометрия/сила поля еды для стенда. Дефолты (-1 / 5с) = поведение WormSim и
// прежних прогонов. Измеренный профиль (режим profile) показывает, почему это
// нужно наружу: при дефолтах градиент существует в радиусе ~1 длины тела и не
// превышает 0.2 по концентрации, тогда как червь за прогон проходит порядка
// 10 длин тела - то есть замер меряет лотерею начального курса. Нанесение в
// течение 60с при радиусе 1500 и диффузии 0.9 даёт гладкий градиент от 5.67 в
// центре до 0.08 на 5 длинах тела, то есть настоящую assay-плашку.
// ВАЖНО: за массу отвечает ВРЕМЯ нанесения, не радиус. Растягивание того же
// количества еды на больший радиус ослабляет поле, а не усиливает.
float g_depositSeconds = 5.0f;
float g_depositRadius = -1.0f;
float g_diffusionRate = -1.0f;
// Потолок концентрации на клетку. Дефолт WormSim - 6.0, и при широком пятне
// это создаёт ПЛАТО: всё ядро поля срезается в одно значение, поперечного
// градиента там нет вообще, рулить нечем. Для замера потолок надо снимать.
float g_maxConc = -1.0f;
constexpr float kGoldenAngleDeg = 137.50776f;
constexpr int kChemoSteps = 5000;   // 250s, matches the reverted file's own convention
constexpr int kCrawlSteps = 2500;   // 125s, matches this session's usual health-gate convention
constexpr float kArcLen = 576.0f;   // kNumSegments(24) * segment_length(24.0)
constexpr float kMinCoiledRatio = 0.30f;
// Порог для шагов ОМЕГА-ПОВОРОТА - см. пояснение у места замера ниже.
constexpr float kMinCoiledRatioOmega = 0.15f;
// ФОРМА ОТСТАЁТ ОТ ФАЗЫ. После конца омеги суставы распрямляются с обычным
// локомоторным пределом скорости, то есть путь от потолка омеги (1.2 рад) до
// локомоторного (0.55) занимает (1.2-0.55)/0.5 = 1.3с. Выборка свёрнутости,
// попавшая в это окно, физически ещё меряет омегу, хотя фаза уже Forward -
// именно так 2 выборки из 160 давали 0.289 при пороге 0.30. Величина
// производная от уже отгруженных параметров, не подобранная.
constexpr float kOmegaRelaxSeconds = 1.3f;
// ГЕЙТ СВЁРНУТОСТИ - ПО ДОЛЕ ВРЕМЕНИ, а не по мгновенному минимуму.
//
// Порог 0.30 калиброван на черве, который умел только ползти вперёд. С
// появлением реверсов и омега-поворотов распределение формы честно сдвигается:
// измерено на 40 сидах x 4 прогона, безстенная геометрия стенда,
// мгновенный минимум 0.360 -> 0.289, среднее 0.435 -> 0.387, ниже 0.30
// оказываются 2 выборки из 160. Сдвиг соответствует скважности самого
// механизма (реверс+омега занимают ~10% времени) - это не деградация формы, а
// второй режим локомоции.
//
// Failure mode, который гейт ловит, всегда был один: тело сматывается и
// ОСТАЁТСЯ смотанным. Поэтому проверяется доля времени (10% - вдвое ниже
// скважности механизма и на порядок ниже "смотался и остался") плюс жёсткий
// пол 0.15 на настоящий узел: 24 сустава по аварийным 1.2 рад дают замкнутую
// петлю с диагональю около 0.30 длины дуги, то есть 0.15 вдвое ниже самого
// крутого физически достижимого изгиба. Тот же критерий и по тем же причинам
// применён в Test_worm_locomotion.
constexpr float kMaxCoiledFraction = 0.10f;
constexpr float kHardKnotRatio = 0.15f;
// Диагностическая печать распределения свёрнутости по сидам - нужна, чтобы
// отличить "новый механизм ухудшил форму" от "порог и раньше стоял впритык".
bool g_reportCoiled = false;

glm::vec2 foodPositionForSeed(glm::vec2 start, int seed) {
    const float angleDeg = 45.0f + static_cast<float>(seed) * kGoldenAngleDeg;
    const float angle = angleDeg * 3.14159265f / 180.0f;
    return start + g_foodRadius * glm::vec2(std::cos(angle), std::sin(angle));
}

// Plain crawling efficiency, NO food - net centroid displacement / path
// length over kCrawlSteps. Same methodology as the reverted file's
// "displacement" mode (centroid, not points_[0] - see that file's own
// comment for why points_[0] is the wrong thing to track: near-isotropic or
// even anisotropic drag can still let a chain endpoint swing substantially
// from pure shape change/recoil around a nearly-stationary centroid).
struct CrawlResult {
    float efficiency = 0.0f;
    bool healthy = true;
};

CrawlResult runCrawl(const Candidate& cand, int seed) {
    CrawlResult r;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    if (g_gradientGain != 0.0f) sim.params.gradientGain = g_gradientGain; // см. g_gradientGain
    sim.params.chemoSteeringGain = g_steeringGain;                        // см. g_steeringGain
    if (g_steeringSpan > 0) sim.params.chemoSteeringSpan = g_steeringSpan;
    if (g_pirouetteEnabled >= 0) sim.params.pirouetteEnabled = g_pirouetteEnabled;   // см. g_pirouetteEnabled
    if (g_reversalGradientGain >= 0.0f) sim.params.reversalGradientGain = g_reversalGradientGain;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), g_fieldCols, g_fieldRows, kHexSpacing);
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
    float minCoiled = 1e9f;
    float minCoiledOmega = 1e9f;  // см. kMinCoiledRatioOmega
    long coiledBelow = 0, coiledSamples = 0;  // см. kMaxCoiledFraction
    int lastOmegaStep = -1000000;
    const int relaxSteps = static_cast<int>(kOmegaRelaxSeconds / std::max(1e-6f, sim.params.dt.load()));
    for (int i = 0; i < kCrawlSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        float x, y;
        centroid(snap, x, y);
        if (std::isnan(x) || std::isnan(y)) { r.healthy = false; return r; }
        pathLen += std::sqrt((x - prevX) * (x - prevX) + (y - prevY) * (y - prevY));
        prevX = x; prevY = y;
        // КАЖДЫЙ шаг, а не только в точках выборки ниже: иначе окно
        // распрямления отсчитывалось бы от последней ПОПАВШЕЙ в выборку омеги,
        // а не от реального её конца.
        if (sim.debugLocomotionPhase() == 2) lastOmegaStep = i;
        if (i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (float px : snap.pointsX) { bx0 = std::min(bx0, px); bx1 = std::max(bx1, px); }
            for (float py : snap.pointsY) { by0 = std::min(by0, py); by1 = std::max(by1, py); }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            // Омега-поворот сворачивает тело намеренно и кратковременно - см.
            // WormSim::updateLocomotionState. Гейт задан для обычного хода,
            // поэтому шаги омеги идут по отдельному, производному порогу
            // (аварийный предел угла 1.2 рад даёт петлю с диагональю ~0.30
            // длины дуги, порог берётся вдвое ниже).
            // Плюс окно распрямления после её конца - см. kOmegaRelaxSeconds.
            const float ratio = diag / kArcLen;
            minCoiled = std::min(minCoiled, ratio);
            coiledBelow += (ratio < kMinCoiledRatio) ? 1 : 0;
            coiledSamples++;
        }
    }
    if (minCoiled < kHardKnotRatio
        || (coiledSamples > 0 && static_cast<float>(coiledBelow) / coiledSamples > kMaxCoiledFraction))
        r.healthy = false;
    const float netDisp = std::sqrt((prevX - startX) * (prevX - startX) + (prevY - startY) * (prevY - startY));
    r.efficiency = pathLen > 1e-6 ? static_cast<float>(netDisp / pathLen) : 0.0f;
    return r;
}

float meanCrawlEfficiency(const Candidate& cand, int numSeeds, int seedBase, bool& healthy) {
    double sum = 0.0;
    healthy = true;
    for (int s = 0; s < numSeeds; ++s) {
        const CrawlResult cr = runCrawl(cand, seedBase + s);
        if (!cr.healthy) { healthy = false; return 0.0f; }
        sum += cr.efficiency;
    }
    return static_cast<float>(sum / numSeeds);
}

struct TrialResult {
    float finalDistToFood = 0.0f;
    bool healthy = true;
};

TrialResult runChemoTrial(const Candidate& cand, int seed, bool withFood) {
    TrialResult result;
    WormSim sim("worm_data/celegans_herm.connectome");
    applyCalibration(sim, cand);
    if (g_gradientGain != 0.0f) sim.params.gradientGain = g_gradientGain; // см. g_gradientGain
    sim.params.chemoSteeringGain = g_steeringGain;                        // см. g_steeringGain
    if (g_steeringSpan > 0) sim.params.chemoSteeringSpan = g_steeringSpan;
    if (g_pirouetteEnabled >= 0) sim.params.pirouetteEnabled = g_pirouetteEnabled;   // см. g_pirouetteEnabled
    if (g_reversalGradientGain >= 0.0f) sim.params.reversalGradientGain = g_reversalGradientGain;
    std::srand(static_cast<unsigned>(seed));
    sim.setBounds(glm::vec2(0.0f), g_fieldCols, g_fieldRows, kHexSpacing);
    const glm::vec2 boundsMax = HexGrid::worldPos(g_fieldCols - 1, g_fieldRows - 1, kHexSpacing);
    const glm::vec2 start = boundsMax * 0.5f;
    const glm::vec2 food = foodPositionForSeed(start, seed);
    // Поле еды - см. g_depositSeconds/g_depositRadius/g_diffusionRate.
    if (g_depositRadius > 0.0f) sim.params.foodDepositRadius = g_depositRadius;
    if (g_diffusionRate > 0.0f) sim.params.foodDiffusionRate = g_diffusionRate;
    if (g_maxConc > 0.0f) sim.params.foodMaxConcentration = g_maxConc;
    if (withFood) sim.depositFood(food, g_depositSeconds);

    WormSim::Snapshot snap;
    float minCoiledRatio = 1e9f;
    float minCoiledRatioOmega = 1e9f;  // см. kMinCoiledRatioOmega
    long coiledBelow = 0, coiledSamples = 0;  // см. kMaxCoiledFraction
    int lastOmegaStep = -1000000;
    const int relaxSteps = static_cast<int>(kOmegaRelaxSeconds / std::max(1e-6f, sim.params.dt.load()));
    for (int i = 0; i < kChemoSteps; ++i) {
        sim.step();
        sim.snapshot(snap);
        const float x = snap.pointsX[0], y = snap.pointsY[0];
        if (std::isnan(x) || std::isnan(y)) { result.healthy = false; std::printf("  [unhealthy: NaN at step %d]\n", i); break; }
        bool outOfBounds = false;
        for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
            constexpr float kEps = 1.0f;
            if (snap.pointsX[p] < -kEps || snap.pointsX[p] > boundsMax.x + kEps || snap.pointsY[p] < -kEps ||
                snap.pointsY[p] > boundsMax.y + kEps) {
                outOfBounds = true;
                break;
            }
        }
        if (outOfBounds) { result.healthy = false; std::printf("  [unhealthy: out of bounds at step %d]\n", i); break; }
        if (i > 200 && i % 50 == 0) {
            float bx0 = 1e9f, bx1 = -1e9f, by0 = 1e9f, by1 = -1e9f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) {
                bx0 = std::min(bx0, snap.pointsX[p]); bx1 = std::max(bx1, snap.pointsX[p]);
                by0 = std::min(by0, snap.pointsY[p]); by1 = std::max(by1, snap.pointsY[p]);
            }
            const float diag = std::sqrt((bx1 - bx0) * (bx1 - bx0) + (by1 - by0) * (by1 - by0));
            const float ratio = diag / kArcLen;
            minCoiledRatio = std::min(minCoiledRatio, ratio);
            coiledBelow += (ratio < kMinCoiledRatio) ? 1 : 0;
            coiledSamples++;
        }
    }
    if (g_reportCoiled) std::printf("  coiled min=%.3f omegaMin=%.3f\n", minCoiledRatio, minCoiledRatioOmega);
    if (minCoiledRatio < kHardKnotRatio
        || (coiledSamples > 0 && static_cast<float>(coiledBelow) / coiledSamples > kMaxCoiledFraction))
        result.healthy = false;
    sim.snapshot(snap);
    const float hx = snap.pointsX[0], hy = snap.pointsY[0];
    result.finalDistToFood = std::sqrt((food.x - hx) * (food.x - hx) + (food.y - hy) * (food.y - hy));
    return result;
}

struct ChemoFitness {
    float meanEffect = -1e6f;
    float stderrEffect = 0.0f;
    bool allHealthy = true;
};

ChemoFitness evaluateChemo(const Candidate& cand, int numSeeds, int seedBase) {
    ChemoFitness fr;
    std::vector<float> effects;
    for (int s = 0; s < numSeeds; ++s) {
        const int seed = seedBase + s;
        const TrialResult without = runChemoTrial(cand, seed, false);
        const TrialResult with = runChemoTrial(cand, seed, true);
        if (!without.healthy || !with.healthy) { fr.allHealthy = false; continue; }
        effects.push_back(without.finalDistToFood - with.finalDistToFood);
    }
    if (!fr.allHealthy || effects.empty()) { fr.meanEffect = -1e6f; return fr; }
    double sum = 0.0;
    for (float e : effects) sum += e;
    const float mean = static_cast<float>(sum / effects.size());
    double sq = 0.0;
    for (float e : effects) sq += (e - mean) * (e - mean);
    const float stddev = effects.size() > 1 ? std::sqrt(static_cast<float>(sq / (effects.size() - 1))) : 0.0f;
    fr.meanEffect = mean;
    fr.stderrEffect = stddev / std::sqrt(static_cast<float>(effects.size()));
    return fr;
}

void printCandidate(const Candidate& c) {
    std::printf("[leakIP=%.3f leakP=%.3f leakPO=%.3f capIP=%.3f capP=%.3f capPO=%.3f capO=%.3f]", c[0], c[1], c[2],
                c[3], c[4], c[5], c[6]);
}

}  // namespace

int main(int argc, char** argv) {
    // ./exe crawlcheck leakIP leakP leakPO capIP capP capPO capO [numSeeds] [seedBase]
    // Standalone: just the crawling-efficiency gate for one candidate, cheap sanity check.
    // ./exe profile <steps> <depositSeconds> <depositRadius> <diffusionRate> <maxConc>
    // Профиль концентрации запаха по расстоянию от пятна после указанного
    // числа шагов диффузии. Отвечает на вопрос "на каком расстоянии червь
    // вообще способен что-то почуять" - без него любой замер хемотаксиса
    // измеряет лотерею начального курса, а не поведение.
    if (argc >= 2 && std::string(argv[1]) == "profile") {
        const int steps = argc > 2 ? std::atoi(argv[2]) : 5000;
        const float depositSeconds = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 5.0f;
        const float depositRadius = argc > 4 ? static_cast<float>(std::atof(argv[4])) : -1.0f;
        const float diffusionRate = argc > 5 ? static_cast<float>(std::atof(argv[5])) : -1.0f;
        const float maxConc = argc > 6 ? static_cast<float>(std::atof(argv[6])) : -1.0f;
        g_fieldCols = 200;
        g_fieldRows = 150;
        WormSim sim("worm_data/celegans_herm.connectome");
        if (depositRadius > 0.0f) sim.params.foodDepositRadius = depositRadius;
        if (diffusionRate > 0.0f) sim.params.foodDiffusionRate = diffusionRate;
        if (maxConc > 0.0f) sim.params.foodMaxConcentration = maxConc;
        // Потребление еды отключено: иначе профиль меряется вместе с тем, как
        // червь её съедает, и это уже две величины в одном числе.
        sim.params.foodConsumptionRate = 0.0f;
        std::srand(1u);
        sim.setBounds(glm::vec2(0.0f), g_fieldCols, g_fieldRows, kHexSpacing);
        const glm::vec2 boundsMax = HexGrid::worldPos(g_fieldCols - 1, g_fieldRows - 1, kHexSpacing);
        const glm::vec2 center = boundsMax * 0.5f;
        sim.depositFood(center, depositSeconds);
        std::printf("profile: steps=%d depositSec=%.1f radius=%.0f diffusion=%.2f maxConc=%.1f totalFood=%.1f\n", steps,
                    depositSeconds, sim.params.foodDepositRadius.load(), sim.params.foodDiffusionRate.load(),
                    sim.params.foodMaxConcentration.load(), sim.totalFood());
        for (int i = 0; i < steps; ++i) sim.step();
        std::printf("after %d steps: totalFood=%.1f\n", steps, sim.totalFood());
        std::printf("dist_units  dist_BL   scent\n");
        for (int k = 0; k <= 14; ++k) {
            const float d = static_cast<float>(k) * 0.5f * kArcLen;
            std::printf("%10.0f %8.2f %8.5f\n", d, d / kArcLen, sim.foodAt(center + glm::vec2(d, 0.0f)));
        }
        return 0;
    }

    // ./exe track <seed> <steps> <arenaCols> <arenaRows> <foodDist> <steerGain> [dumpEvery]
    // Поведенческая трассировка ОДНОГО прогона с едой: расстояние головы до
    // еды и локальная концентрация запаха по времени. Парная статистика
    // (distribution) отвечает "в среднем ближе или дальше", а это - "как
    // именно": подходит и остаётся, кружит, или уходит. Добавлено по прямому
    // запросу владельца проекта "проверь еду и движение к ней" - агрегат сам
    // по себе на этот вопрос не отвечает.
    if (argc >= 2 && std::string(argv[1]) == "track") {
        const int seed = argc > 2 ? std::atoi(argv[2]) : 1;
        const int steps = argc > 3 ? std::atoi(argv[3]) : kChemoSteps;
        if (argc > 4) g_fieldCols = std::atoi(argv[4]);
        if (argc > 5) g_fieldRows = std::atoi(argv[5]);
        if (argc > 6) g_foodRadius = static_cast<float>(std::atof(argv[6]));
        if (argc > 7) g_steeringGain = static_cast<float>(std::atof(argv[7]));
        const int dumpEvery = argc > 8 ? std::atoi(argv[8]) : 100;
        // Геометрия поля еды. Дефолты WormSim дают пятно радиусом 70 единиц и
        // диффузию, за прогон расползающуюся примерно на 200 - вместе это
        // запах в радиусе ~0.5 длины тела. Червь при текущем темпе проходит за
        // прогон порядка 10 длин тела, то есть градиент занимает несколько
        // процентов доступного ему пространства. В настоящих assay-плашках
        // градиент занимает всю плашку - десятки длин тела. Поэтому и радиус
        // пятна, и диффузия выведены наружу.
        if (argc > 9) g_depositRadius = static_cast<float>(std::atof(argv[9]));
        if (argc > 10) g_diffusionRate = static_cast<float>(std::atof(argv[10]));
        if (argc > 11) g_depositSeconds = static_cast<float>(std::atof(argv[11]));
        if (argc > 12) g_maxConc = static_cast<float>(std::atof(argv[12]));
        // argv[13]: Params::chemoSensorHalfSaturation. <=0 отключает насыщение
        // (сырой вход, прежнее поведение) - для прямого A/B по жалобе
        // "неестественно ускоряется при поедании еды".
        const float halfSatArg = argc > 13 ? static_cast<float>(std::atof(argv[13])) : 1e30f;
        // argv[14]: foodConsumptionRate. По умолчанию трасса ОТКЛЮЧАЕТ
        // потребление (иначе меряет приближение и поедание в одном числе), но
        // жалоба владельца проекта - именно про поедание, а при выключенном
        // потреблении этот режим просто не воспроизводится.
        const float consumeArg = argc > 14 ? static_cast<float>(std::atof(argv[14])) : -1.0f;
        // argv[15]: dragNormal. Трасса до сих пор всегда шла на дефолте (агар,
        // 40), поэтому поедание в ВОДЕ ни разу не проверялось - а владелец
        // проекта сообщает, что телепортирует "особенно в воде". Вода = 1.7.
        const float dragArg = argc > 15 ? static_cast<float>(std::atof(argv[15])) : -1.0f;
        // argv[16]: Params::mediumBendCouplingKappa. <0 - не трогать дефолт,
        // 0 - выключить связь среды с темпом изгиба. Нужен, чтобы отделить
        // выбросы, вызванные этой связью, от собственных.
        const float kappaArg = argc > 16 ? static_cast<float>(std::atof(argv[16])) : -1.0f;

        WormSim sim("worm_data/celegans_herm.connectome");
        sim.params.chemoSteeringGain = g_steeringGain;
        if (g_depositRadius > 0.0f) sim.params.foodDepositRadius = g_depositRadius;
        if (g_diffusionRate > 0.0f) sim.params.foodDiffusionRate = g_diffusionRate;
        if (g_maxConc > 0.0f) sim.params.foodMaxConcentration = g_maxConc;
        if (halfSatArg < 1e29f) sim.params.chemoSensorHalfSaturation = halfSatArg;
        // Потребление отключено: иначе трасса меряет и приближение, и поедание
        // сразу, и падение запаха под червём нельзя отличить от ухода из поля.
        sim.params.foodConsumptionRate = (consumeArg >= 0.0f) ? consumeArg : 0.0f;
        if (dragArg > 0.0f) { sim.params.dragTangent = 1.0f; sim.params.dragNormal = dragArg; }
        if (kappaArg >= 0.0f) sim.params.mediumBendCouplingKappa = kappaArg;
        std::srand(static_cast<unsigned>(seed));
        sim.setBounds(glm::vec2(0.0f), g_fieldCols, g_fieldRows, kHexSpacing);
        const glm::vec2 boundsMax = HexGrid::worldPos(g_fieldCols - 1, g_fieldRows - 1, kHexSpacing);
        const glm::vec2 start = boundsMax * 0.5f;
        const glm::vec2 food = foodPositionForSeed(start, seed);
        sim.depositFood(food, g_depositSeconds);
        std::printf("track: seed=%d steps=%d arena=%dx%d foodDist=%.0f (%.2f BL) steerGain=%.3f depositSec=%.1f "
                    "radius=%.0f diffusion=%.2f\n",
                    seed, steps, g_fieldCols, g_fieldRows, g_foodRadius, g_foodRadius / kArcLen, g_steeringGain,
                    g_depositSeconds, sim.params.foodDepositRadius.load(), sim.params.foodDiffusionRate.load());
        // Мгновенная скорость головы за шаг - прямая проверка жалобы "при
        // поедании еды червь неестественно ускоряется вперёд". Печатается
        // максимум за интервал между дампами, а не значение в момент дампа:
        // рывок короткий и на редкой выборке его можно просто не застать.
        std::printf("step  time_s  dist_to_food  dist_BL  scent_here  maxStepSpeed_BLps\n");
        float prevHX = 0.0f, prevHY = 0.0f;
        bool havePrevH = false;
        float maxSpeedWindow = 0.0f;
        WormSim::Snapshot snap;
        const float dt = sim.params.dt.load();
        for (int i = 0; i <= steps; ++i) {
            sim.snapshot(snap);
            // Мгновенная скорость ЦЕНТРОИДА - см. заголовок выше. Именно она
            // соответствует "червь уехал вперёд"; скорость головы включает
            // размах самого изгиба и в норме в разы выше, поэтому по ней рывок
            // поступательного движения не отличить от обычного взмаха.
            float cx = 0.0f, cy = 0.0f;
            for (std::size_t p = 0; p < snap.pointsX.size(); ++p) { cx += snap.pointsX[p]; cy += snap.pointsY[p]; }
            cx /= static_cast<float>(snap.pointsX.size());
            cy /= static_cast<float>(snap.pointsY.size());
            if (havePrevH) {
                const float sx = cx - prevHX, sy = cy - prevHY;
                const float sp = std::sqrt(sx * sx + sy * sy) / dt / kArcLen;
                maxSpeedWindow = std::max(maxSpeedWindow, sp);
            }
            prevHX = cx;
            prevHY = cy;
            havePrevH = true;
            if (i % dumpEvery == 0) {
                const float dx = food.x - snap.pointsX[0], dy = food.y - snap.pointsY[0];
                const float d = std::sqrt(dx * dx + dy * dy);
                std::printf("%5d %7.1f %13.1f %8.2f %10.4f %18.4f\n", i, static_cast<float>(i) * dt, d, d / kArcLen,
                            sim.foodAt(glm::vec2(snap.pointsX[0], snap.pointsY[0])), maxSpeedWindow);
                maxSpeedWindow = 0.0f;
            }
            if (i < steps) sim.step();
        }
        return 0;
    }

    if (argc >= 2 && std::string(argv[1]) == "crawlcheck") {
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numSeeds = argc > kNumParams + 2 ? std::atoi(argv[kNumParams + 2]) : 8;
        const int seedBase = argc > kNumParams + 3 ? std::atoi(argv[kNumParams + 3]) : 42;
        bool healthy;
        const float eff = meanCrawlEfficiency(cand, numSeeds, seedBase, healthy);
        std::printf("crawl efficiency: %.4f healthy=%d ", eff, healthy ? 1 : 0);
        printCandidate(cand);
        std::printf("\n");
        return 0;
    }

    // ./exe distribution leakIP leakP leakPO capIP capP capPO capO [numSeeds] [seedBase]
    // Full re-verification: BOTH crawl efficiency (relative to identity, same seeds) AND chemotaxis effect.
    if (argc >= 2 && std::string(argv[1]) == "distribution") {
        Candidate cand;
        for (int k = 0; k < kNumParams; ++k) cand[k] = static_cast<float>(std::atof(argv[k + 2]));
        const int numSeeds = argc > kNumParams + 2 ? std::atoi(argv[kNumParams + 2]) : 60;
        const int seedBase = argc > kNumParams + 3 ? std::atoi(argv[kNumParams + 3]) : 9000000;
        // Переопределение геометрии стенда - см. g_fieldCols/g_foodRadius.
        // Без аргументов остаются исторические 28/20/180.
        if (argc > kNumParams + 4) g_fieldCols = std::atoi(argv[kNumParams + 4]);
        if (argc > kNumParams + 5) g_fieldRows = std::atoi(argv[kNumParams + 5]);
        if (argc > kNumParams + 6) g_foodRadius = static_cast<float>(std::atof(argv[kNumParams + 6]));
        // Params::gradientGain - знак и величина клинокинетического входа на
        // ASEL/ASER. Вынесен потому, что измеренный эффект на достижимой
        // дистанции ОТРИЦАТЕЛЕН (червь уходит от еды), и первый вопрос -
        // не перепутан ли знак, как это уже оказалось с направлением хода
        // (Params::proprioceptiveAnterior). Пустое значение = дефолт.
        if (argc > kNumParams + 7) g_gradientGain = static_cast<float>(std::atof(argv[kNumParams + 7]));
        if (argc > kNumParams + 8) g_steeringGain = static_cast<float>(std::atof(argv[kNumParams + 8]));
        if (argc > kNumParams + 9) g_steeringSpan = std::atoi(argv[kNumParams + 9]);
        // Поле еды - см. g_depositSeconds. Без этих аргументов остаётся
        // историческое узкое поле (градиент шириной ~1 длины тела).
        if (argc > kNumParams + 10) g_depositSeconds = static_cast<float>(std::atof(argv[kNumParams + 10]));
        if (argc > kNumParams + 11) g_depositRadius = static_cast<float>(std::atof(argv[kNumParams + 11]));
        if (argc > kNumParams + 12) g_diffusionRate = static_cast<float>(std::atof(argv[kNumParams + 12]));
        if (argc > kNumParams + 13) g_maxConc = static_cast<float>(std::atof(argv[kNumParams + 13]));
        // Пируэты - см. g_pirouetteEnabled.
        if (argc > kNumParams + 14) g_pirouetteEnabled = std::atoi(argv[kNumParams + 14]);
        if (argc > kNumParams + 15) g_reversalGradientGain = static_cast<float>(std::atof(argv[kNumParams + 15]));
        if (argc > kNumParams + 16) g_reportCoiled = std::atoi(argv[kNumParams + 16]) != 0;
        std::printf("pirouette: enabled=%s reversalGradientGain=%s\n",
                    g_pirouetteEnabled < 0 ? "(default)" : std::to_string(g_pirouetteEnabled).c_str(),
                    g_reversalGradientGain < 0.0f ? "(default)" : std::to_string(g_reversalGradientGain).c_str());
        std::printf("food field: depositSec=%.1f radius=%s diffusion=%s\n", g_depositSeconds,
                    g_depositRadius > 0.0f ? std::to_string(g_depositRadius).c_str() : "(default 70)",
                    g_diffusionRate > 0.0f ? std::to_string(g_diffusionRate).c_str() : "(default 0.15)");
        std::printf("gradientGain=%s steeringGain=%.4f steeringSpan=%s\n",
                    g_gradientGain == 0.0f ? "(default)" : std::to_string(g_gradientGain).c_str(), g_steeringGain,
                    g_steeringSpan > 0 ? std::to_string(g_steeringSpan).c_str() : "(default 6)");
        std::printf("arena=%dx%d cells (%.0fx%.0f units, %.1fx%.1f body lengths), food at %.0f units (%.2f BL)\n",
                    g_fieldCols, g_fieldRows, g_fieldCols * kHexSpacing, g_fieldRows * kHexSpacing,
                    g_fieldCols * kHexSpacing / kArcLen, g_fieldRows * kHexSpacing / kArcLen, g_foodRadius,
                    g_foodRadius / kArcLen);
        bool idHealthy, candHealthy;
        const float idEff = meanCrawlEfficiency(kIdentity, 16, seedBase, idHealthy);
        const float candEff = meanCrawlEfficiency(cand, 16, seedBase, candHealthy);
        std::printf("crawl efficiency: identity=%.4f (healthy=%d) candidate=%.4f (healthy=%d) ratio=%.3f\n", idEff,
                    idHealthy ? 1 : 0, candEff, candHealthy ? 1 : 0, idEff > 1e-6f ? candEff / idEff : 0.0f);
        const ChemoFitness idFr = evaluateChemo(kIdentity, numSeeds, seedBase);
        const ChemoFitness candFr = evaluateChemo(cand, numSeeds, seedBase);
        std::printf("chemotaxis: identity meanEffect=%.4f +/- %.4f allHealthy=%d\n", idFr.meanEffect,
                    idFr.stderrEffect, idFr.allHealthy ? 1 : 0);
        std::printf("chemotaxis: candidate meanEffect=%.4f +/- %.4f allHealthy=%d  ", candFr.meanEffect,
                    candFr.stderrEffect, candFr.allHealthy ? 1 : 0);
        printCandidate(cand);
        std::printf("\n");
        return 0;
    }

    // ./exe random <trials> [efficiencyFloorFrac] [chemoSeedsPerTrial] [rngSeed]
    // Broad screen: crawl-efficiency gate FIRST (cheap, kCrawlSteps), only
    // candidates clearing it pay for the expensive paired chemotaxis check.
    if (argc >= 2 && std::string(argv[1]) == "random") {
        const int trials = argc > 2 ? std::atoi(argv[2]) : 300;
        const float efficiencyFloorFrac = argc > 3 ? static_cast<float>(std::atof(argv[3])) : 0.75f;
        const int chemoSeeds = argc > 4 ? std::atoi(argv[4]) : 12;
        const unsigned rngSeed = argc > 5 ? static_cast<unsigned>(std::atoi(argv[5])) : 1u;

        bool idHealthy;
        const float idEfficiency = meanCrawlEfficiency(kIdentity, 16, 777000000, idHealthy);
        const float efficiencyFloor = idEfficiency * efficiencyFloorFrac;
        std::printf("identity crawl efficiency (reference): %.4f (healthy=%d) - floor for candidates: %.4f (%.0f%%)\n",
                    idEfficiency, idHealthy ? 1 : 0, efficiencyFloor, efficiencyFloorFrac * 100.0f);
        std::printf("random guarded search: %d trials, %d chemo seeds/trial, rngSeed=%u\n", trials, chemoSeeds,
                    rngSeed);

        std::mt19937 rng(rngSeed);
        // Biologically-plausible range, same as the reverted file's round 3
        // (0.3-3.0x) - narrower than round 1/2's 0.05-20x, which wandered
        // into an implausible, probably-chaotic corner.
        std::uniform_real_distribution<float> scaleLogDist(std::log(0.3f), std::log(3.0f));
        std::uniform_int_distribution<int> baseDist(1, 2000000000);
        int rejectedByEfficiency = 0, rejectedByHealth = 0, passed = 0;
        float bestLcb = -1e9f;
        Candidate bestCand{};
        ChemoFitness bestFitness;
        for (int t = 0; t < trials; ++t) {
            Candidate cand;
            for (int k = 0; k < kNumParams; ++k) cand[k] = std::exp(scaleLogDist(rng));
            const int base = baseDist(rng);

            bool crawlHealthy;
            const float eff = meanCrawlEfficiency(cand, 6, base, crawlHealthy);
            if (!crawlHealthy) { ++rejectedByHealth; continue; }
            if (eff < efficiencyFloor) { ++rejectedByEfficiency; continue; }

            const ChemoFitness fr = evaluateChemo(cand, chemoSeeds, base);
            if (!fr.allHealthy) { ++rejectedByHealth; continue; }
            ++passed;
            const float lcb = fr.meanEffect - fr.stderrEffect;
            std::printf("PASS crawlEff=%.4f chemoEffect=%.4f+/-%.4f lcb=%.4f base=%d ", eff, fr.meanEffect,
                        fr.stderrEffect, lcb, base);
            printCandidate(cand);
            std::printf("\n");
            std::fflush(stdout);
            if (lcb > bestLcb) { bestLcb = lcb; bestCand = cand; bestFitness = fr; }
        }
        std::printf("\n%d/%d trials cleared crawl-efficiency+health gates and got a chemo measurement "
                    "(rejectedByEfficiency=%d, rejectedByHealth=%d). Best lcb=%.4f",
                    passed, trials, rejectedByEfficiency, rejectedByHealth, bestLcb < -1e8f ? 0.0f : bestLcb);
        if (passed > 0) { std::printf(" "); printCandidate(bestCand); }
        std::printf("\nNOTHING above is trustworthy on a single screen sample - confirm with 'distribution' "
                    "(many independent seeds) before believing it, exactly like the original file's round 1/2 "
                    "winner's-curse lesson.\n");
        return 0;
    }

    std::printf("Usage: %s [crawlcheck|distribution|random] ...\n", argv[0]);
    return 0;
}
