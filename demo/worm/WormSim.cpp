#include "WormSim.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <stdexcept>

namespace {
float frand() { return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX); }
float signedNoise(float amplitude) { return amplitude * (frand() * 2.0f - 1.0f); }

// DA/DB/DD - дорсальные, VA/VB/VD - вентральные, AS - тоже дорсальные
// (холинергические, проецируют на дорсальные мышцы) моторные нейроны
// вентрального тяжа; реальные анатомические числа нейронов в каждом классе
// (фиксированный факт биологии C. elegans, не подгоняемый параметр).
// Возвращает 0, если класс не распознан.
int motorClassCount(char c0, char c1) {
    if (c0 == 'D' && c1 == 'A') return 9;
    if (c0 == 'D' && c1 == 'B') return 7;
    if (c0 == 'D' && c1 == 'D') return 6;
    if (c0 == 'V' && c1 == 'A') return 12;
    if (c0 == 'V' && c1 == 'B') return 11;
    if (c0 == 'V' && c1 == 'D') return 13;
    if (c0 == 'A' && c1 == 'S') return 11;
    return 0;
}

bool motorClassDorsal(char c0, char c1) { return c0 == 'D' || (c0 == 'A' && c1 == 'S'); }

// Пытается разобрать имя как "<класс><число>" (например "VB11", "AS7"). При
// успехе возвращает true и заполняет position (1..24, по индексу внутри
// класса, пропорционально растянутому на длину тела) и dorsal.
bool parseMotorNeuron(const std::string& name, int& position, bool& dorsal) {
    if (name.size() < 3) return false;
    const char c0 = name[0], c1 = name[1];
    const int count = motorClassCount(c0, c1);
    if (count == 0) return false;
    for (std::size_t k = 2; k < name.size(); ++k)
        if (!std::isdigit(static_cast<unsigned char>(name[k]))) return false;
    const int idx = std::atoi(name.c_str() + 2);
    if (idx < 1 || idx > count) return false;
    position = (count > 1) ? static_cast<int>(std::lround(1.0 + (idx - 1) * 23.0 / (count - 1))) : 12;
    position = std::clamp(position, 1, 24);
    dorsal = motorClassDorsal(c0, c1);
    return true;
}
} // namespace

WormSim::WormSim(const std::string& connectomeDataPath)
    : m_loaded(connectome::load_connectome(connectomeDataPath)), m_body(kNumSegments, 24.0f) {
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    connectome::Network& net = m_loaded.network;
    net.set_activation_shape(0.0f, 1.0f);

    // Per-neuron-CLASS leak/capacitance calibration for GAIT SPEED - TRIED
    // AND REVERTED (second time this parameter axis has burned a search -
    // see the chemotaxis paragraph below for the first). Live measurement
    // (Demo_worm, default params) found the worm ~30-700x slower than real
    // C. elegans in body-lengths/second (Fang-Yen et al. 2010 and related
    // literature - see tests/worm_speed_calibration's header for exact
    // numbers/citations), and direct waveform tracing (dumping
    // lastCurvatureDeviation() over time, that file's "trace" mode) found
    // why: the emergent bend cycle at uncalibrated defaults takes on the
    // order of a MINUTE-PLUS per cycle against a real worm's 0.5-2 Hz - that
    // diagnosis stands, independent of everything below.
    //
    // tests/worm_speed_calibration's search (screen-then-confirm, same
    // methodology as the chemotaxis search, WITH an efficiency>=0.40 +
    // coiled-ratio>=0.30 health gate specifically to avoid repeating that
    // search's mistake) found a candidate that looked like a clean win on
    // its own final-verification check: identity 0.00221 BL/s vs calibrated
    // 0.01073 BL/s (4.85x), efficiency unchanged. It shipped. It was wrong.
    //
    // Re-running the SAME search fresh (same code, same RNG algorithm) gave
    // a DIFFERENT identity baseline (0.0108, not 0.0022) and found the
    // "4.85x" figure did not hold up: the winning candidate's speedup over
    // THAT baseline was only 1.44x. Chasing the discrepancy down (see
    // tests/worm_speed_calibration's "distribution" mode, added for this)
    // found the real problem: this network's dynamics are chaotic enough
    // that IDENTITY's own speed is not a stable number - it depends on
    // which attractor a given random seed happens to land the system in,
    // which is inherently unpredictable given the network's own chaos, not
    // literally random noise in the ordinary Monte-Carlo sense. A single
    // seed base's 24-seed sample (what the original search's "final
    // verification" used) is not enough to characterize this - one specific
    // base (900000000) happened to land unusually low. Measuring identity
    // across 20 FRESH seed bases (8 seeds each) never reproduced that low
    // reading again (0/20 came back slow) and gave a population mean of
    // 0.01079 BL/s. The SAME 20-base measurement on the shipped calibrated
    // candidate gave 0.01079 BL/s too - identical to 5 significant figures.
    // The calibration bought NOTHING on speed. What it did do: cut mean
    // efficiency roughly in half (0.55 -> 0.27, and far more scattered:
    // 0.12-0.37 vs identity's 0.38-0.66) while raising bend frequency
    // 4-6x (0.03-0.05Hz -> 0.10-0.24Hz) - a twitchier, less coherent gait
    // for zero net-distance benefit. Reverted to the flat uncalibrated
    // default (leak=1, capacitance=1 for every neuron, set by
    // load_connectome).
    //
    // Lesson for whoever revisits this: this network's chaotic sensitivity
    // makes ANY single-seed-base "final verification," no matter how many
    // seeds, an unreliable check for a global comparison like this - a
    // fitness/verification step needs to sample MULTIPLE independent seed
    // BASES (not just more seeds from one base) and look at the distribution
    // across bases, not a single pooled mean, or it can (and did) mistake a
    // one-off baseline reading for a real, reproducible effect.

    // Chemotaxis calibration - TRIED AND REVERTED (kept for the lesson).
    // tests/worm_chemotaxis_calibration found a calibration
    // (leakScale{IP=1.397,P=1.419,PO=0.520}, capScale{IP=1.680,P=0.560,
    // PO=1.212,O=0.400}) that produced a real, independently-confirmed
    // directed-chemotaxis bias (0.0275 +/- 0.0035, ~8 combined-stderr above
    // an identity baseline that itself reproduced the original
    // investigation's "indistinguishable from zero" finding - the search
    // methodology was sound). It shipped for a while. Then a live look at
    // Demo_worm (default params, no food) showed the worm settling into a
    // static shape after ~20s and staying there - and a headless check
    // (tests/worm_chemotaxis_calibration's "displacement" mode) confirmed it
    // numerically: this calibration cut plain crawling efficiency (net
    // displacement / path length, no food, 800 steps, 16 seeds) from 0.426
    // to 0.131 - MORE THAN 3x WORSE - and net displacement from 92.6 to 45.2
    // units. The search that found this calibration only screened for a
    // chemotaxis effect plus basic health (NaN/bounds/coiled-ratio) - it
    // never measured absolute crawling efficiency at all, so it happily
    // traded away most of the worm's ability to actually get anywhere in
    // exchange for a directional bias worth 0.03-0.06 units against a
    // 180-unit food distance - imperceptible next to a 3x mobility loss.
    // Reverted to the uncalibrated default for THIS axis (the speed
    // calibration above is unrelated and independently validated - see its
    // own efficiency guard). If chemotaxis calibration is revisited, the
    // fitness function MUST include a crawling-efficiency term (not just
    // health/no-crash), or it will keep finding exactly this kind of trade.

    // Synapse-SIGN calibration for GAIT SPEED - TRIED, briefly SHIPPED, then
    // REVERTED (third attempt at this axis - see tests/worm_synapse_speed_
    // calibration for the full search + validation history). Different lever
    // than the two reverted leak/capacitance attempts above: scales
    // excitatory (cholinergic) chemical synapses, inhibitory (GABAergic)
    // chemical synapses, and gap junctions SEPARATELY by sign, not by
    // neuron-class time constant - data/README.md confirms connectome
    // weights are raw EM synaptic CONTACT COUNTS (Cook et al. 2019), not
    // conductances, and the excitatory/inhibitory current ratio per contact
    // is a real missing physiological parameter, not a re-run of the same
    // knob.
    //
    // Search built the multi-independent-seed-base lesson in from round 1
    // (every fitness evaluation during the search itself already averaged
    // 3-8 independent bases, not more seeds from one) - the exact fix for
    // what shipped the false 4.85x leak/capacitance result above. Result:
    // identity 0.00241 BL/s vs winner 0.01268-0.01280 BL/s (independently
    // reproduced on a SECOND, differently-seeded 20-base run: 0.01280,
    // within 1% of the first) - a real, ~48-sigma-separated, reproducible
    // effect on ITS OWN metric, categorically unlike the leak/capacitance
    // false win (which converged to IDENTICAL population means once checked
    // this way). Adversarial multi-agent review (3 independent skeptics,
    // given only the raw numbers) confirmed the effect was real but caught
    // the headline number overstating it: bodyLengthsPerSec measures raw
    // path length, not net progress - true net-displacement speedup was
    // ~3.1x (0.00221 -> 0.00693 BL/s), not 5.25x, since efficiency (net
    // displacement / path length) dropped 0.918 -> 0.529 (-42%) and bend
    // frequency rose more (~4.5x) than net speed did. Coiled ratio ROSE
    // (0.632 -> 0.710) - ruled out the tight-coil degenerate mode
    // specifically. Chemotaxis re-checked (arena auto-scaled to the
    // candidate's measured speed, same paired with/without-food design as
    // the reverted calibration above): identity -0.0057+/-0.0021, winner
    // -0.0014+/-0.0011 - both indistinguishable from zero, not a regression.
    //
    // SHIPPED, then CAUGHT by tests/worm_locomotion - a check the search
    // itself never included: max |heading delta| in one simulation step. Ten
    // independent trials (real time()-seeded, 1.2s apart to force distinct
    // seeds) all landed 2.33-3.11 rad, right up against the test's own 3.2
    // rad implausibility ceiling - vs. this project's ~0.03 rad baseline
    // EVERY OTHER TIME this file has been run, all session. That is the
    // worm's heading nearly reversing (up to ~169 degrees) in a single 50ms
    // step, consistently, not a rare tail event - physically implausible,
    // and none of efficiency/coiled-ratio/bend-frequency-at-one-fixed-
    // position (everything the search AND the adversarial review actually
    // measured) are sensitive to it. Exactly the lesson tests/worm_
    // mechanosensation_calibration already learned the hard way, generalized:
    // a health gate only catches the failure modes it was built to look for -
    // "passed every check we ran" is not the same claim as "is healthy," and
    // a new, unrelated check can still catch something real. Reverted before
    // this session ended; do NOT re-ship this exact candidate without first
    // adding a single-step-heading-delta (or angular-velocity) gate to the
    // search/confirm/final-verification pipeline in tests/worm_synapse_speed_
    // calibration, run across many independent seeds, not the search's own
    // handful - this failure mode was invisible to a low seed count too.
    //
    // KNOWN CAVEATS beyond the one that killed it (for whoever revisits this
    // lever): water-preset speedup was weaker (2.46x) and less robust (2/16
    // bases failed the efficiency gate vs 0/20 on agar); did NOT fix, and
    // slightly worsened, the separate known swim-should-be-faster-than-crawl
    // bug (water/agar ratio 0.232 -> 0.108); network weights calibrate ONCE
    // at construction, before any later dragTangent/dragNormal preset choice,
    // so there is no way to apply this only to one preset with the current
    // architecture.
    // net.scale_synapse_sign(2.160f, 0.142f, 0.103f);  -- REVERTED, see above

    m_isMotorNeuron.assign(net.size(), false);

    // Реальные хемосенсорные нейроны C. elegans (амфидные нейроны запаха/еды).
    // ASEL/ASER кодируют РОСТ/ПАДЕНИЕ концентрации (клинокинез) - отдельно от
    // AWA/AWC, которые получают просто абсолютный уровень.
    for (connectome::NeuronId i = 0; i < net.size(); ++i) {
        const std::string& name = m_loaded.names[i];
        if (name == "ASEL") m_aseL = i;
        else if (name == "ASER") m_aseR = i;
        else if (name == "DVA") m_dva = i;
        else if (name == "AFDL") m_afdL = i;
        else if (name == "AFDR") m_afdR = i;
        else if (name == "ADFL") m_adfL = i;
        else if (name == "ADFR") m_adfR = i;
        else if (name == "NSML") m_nsmL = i;
        else if (name == "NSMR") m_nsmR = i;
        else if (name == "AWAL" || name == "AWAR" || name == "AWCL" || name == "AWCR")
            m_generalChemoIds.push_back(i);

        const auto& m = m_loaded.muscles[i];
        if (m.is_muscle && m.position >= 1 && m.position <= 24) {
            (m.side == 'D' ? m_dorsalByPos : m_ventralByPos)[static_cast<std::size_t>(m.position)].push_back(i);
        }

        int motorPos; bool motorDorsal;
        if (parseMotorNeuron(name, motorPos, motorDorsal)) {
            m_motorNeurons.push_back({i, motorPos, motorDorsal});
            m_isMotorNeuron[i] = true;
        }

        // B-класс (DB1-7 дорсальные, VB1-11 вентральные) - настоящие
        // локальные осцилляторы переднего хода (Fouad et al. 2018, eLife
        // 7:e29913) - см. Params::bClassOscillatorGain.
        if (name.size() >= 3 && (name[0] == 'D' || name[0] == 'V') && name[1] == 'B' &&
            std::isdigit(static_cast<unsigned char>(name[2])))
            m_classBMotorNeurons.push_back(i);
    }
    net.set_active_current_targets(m_classBMotorNeurons);
    if (m_generalChemoIds.empty() && m_aseL == kInvalidId && m_aseR == kInvalidId)
        throw std::runtime_error("connectome has no recognized chemosensory (AWA/AWC/ASE) neurons");

    // Раскладка узлов графа для отрисовки - по типу (столбцы), мышцы отдельно
    // справа по реальной стороне/позиции тела. Считается один раз, [0,1].
    const connectome::NeuronId n = net.size();
    m_nodeLayoutX.assign(n, 0.0f);
    m_nodeLayoutY.assign(n, 0.0f);
    std::array<int, 5> total{};
    for (connectome::NeuronId i = 0; i < n; ++i) total[static_cast<int>(net.type(i))]++;
    std::array<int, 5> seen{};
    const std::array<float, 5> colX = {0.05f, 0.30f, 0.55f, 0.80f, 0.0f};
    for (connectome::NeuronId i = 0; i < n; ++i) {
        const connectome::NeuronType t = net.type(i);
        if (t == connectome::NeuronType::Output) {
            const auto& m = m_loaded.muscles[i];
            const float posFrac = m.is_muscle ? (static_cast<float>(m.position) - 1.0f + 0.5f) / 24.0f : 0.5f;
            m_nodeLayoutX[i] = (m.is_muscle && m.side == 'D') ? 0.90f : 0.98f;
            m_nodeLayoutY[i] = (m.is_muscle && m.side == 'D') ? posFrac * 0.5f : 0.5f + posFrac * 0.5f;
            continue;
        }
        const int ti = static_cast<int>(t);
        const int idx = seen[ti]++;
        const int cnt = std::max(1, total[ti]);
        m_nodeLayoutX[i] = colX[ti];
        m_nodeLayoutY[i] = (static_cast<float>(idx) + 0.5f) / static_cast<float>(cnt);
    }
}

void WormSim::setBounds(glm::vec2 worldMin, int fieldCols, int fieldRows, float hexSpacing) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_boundsMin = worldMin;
    m_fieldCols = std::max(1, fieldCols);
    m_fieldRows = std::max(1, fieldRows);
    m_hexSpacing = std::max(1.0f, hexSpacing);
    // Та же формула, что рендер использует для последней клетки - гарантирует
    // boundsMax согласован с тем, что реально нарисовано на экране.
    m_boundsMax = worldMin + HexGrid::worldPos(m_fieldCols - 1, m_fieldRows - 1, m_hexSpacing);
    m_position = (m_boundsMin + m_boundsMax) * 0.5f;
    m_foodField.assign(static_cast<std::size_t>(m_fieldCols) * static_cast<std::size_t>(m_fieldRows), 0.0f);
}

// Ближайшая клетка НАСТОЯЩЕЙ гекс-решётки (нечётные ряды сдвинуты на пол-
// клетки вправо - см. HexGrid::worldPos) к мировой точке. Оценивает ряд по
// вертикальному шагу, затем перебирает соседние ряды/столбцы и берёт
// минимум по честному евклидову расстоянию до центра клетки - без гадания
// с формулой обратного преобразования, которую легко перепутать со сдвигом.
void WormSim::worldToHexCell(glm::vec2 worldPos, int& outCol, int& outRow) const {
    const float vert = HexGrid::vertSpacing(m_hexSpacing);
    const float horiz = HexGrid::horizSpacing(m_hexSpacing);
    const int rowGuess = static_cast<int>(std::lround((worldPos.y - m_boundsMin.y) / vert));

    float bestDist = std::numeric_limits<float>::max();
    outCol = 0;
    outRow = 0;
    for (int dr = -1; dr <= 1; ++dr) {
        const int row = std::clamp(rowGuess + dr, 0, m_fieldRows - 1);
        const float shift = (row & 1) ? 0.5f : 0.0f;
        const int colGuess = static_cast<int>(std::lround((worldPos.x - m_boundsMin.x) / horiz - shift));
        for (int dc = -1; dc <= 1; ++dc) {
            const int col = std::clamp(colGuess + dc, 0, m_fieldCols - 1);
            const glm::vec2 p = m_boundsMin + HexGrid::worldPos(col, row, m_hexSpacing);
            const float d = glm::dot(p - worldPos, p - worldPos);
            if (d < bestDist) { bestDist = d; outCol = col; outRow = row; }
        }
    }
}

// Взвешенная по расстоянию смесь ближайшей клетки и её 6 НАСТОЯЩИХ гекс-
// соседей (HexGrid::neighborOffsets) - сглаживает scent без ступенек на
// границах клеток (иначе d(scent)/dt на ASEL/ASER дёргалась бы), но по
// правильной геометрии, не по прямоугольному приближению.
float WormSim::sampleFood(glm::vec2 worldPos) const {
    if (m_fieldCols <= 0 || m_fieldRows <= 0 || m_foodField.empty()) return 0.0f;
    int col, row;
    worldToHexCell(worldPos, col, row);

    auto at = [&](int c, int r) { return m_foodField[static_cast<std::size_t>(r) * static_cast<std::size_t>(m_fieldCols) + static_cast<std::size_t>(c)]; };
    const float spacingSq = m_hexSpacing * m_hexSpacing;
    float weightSum = 0.0f, valueSum = 0.0f;
    auto accumulate = [&](int c, int r) {
        if (c < 0 || c >= m_fieldCols || r < 0 || r >= m_fieldRows) return;
        const glm::vec2 p = m_boundsMin + HexGrid::worldPos(c, r, m_hexSpacing);
        const glm::vec2 d = p - worldPos;
        const float w = 1.0f / (1.0f + glm::dot(d, d) / spacingSq);
        weightSum += w;
        valueSum += w * at(c, r);
    };
    accumulate(col, row);
    const int (*offsets)[2] = HexGrid::neighborOffsets(row);
    for (int k = 0; k < 6; ++k) accumulate(col + offsets[k][0], row + offsets[k][1]);
    return weightSum > 1e-6f ? valueSum / weightSum : 0.0f;
}

// Кисть с линейным затуханием к краю - рисует/стирает по клеткам НАСТОЯЩЕЙ
// гекс-решётки в радиусе radiusWorld (мировые единицы) вокруг worldPos,
// расстояние до каждой клетки - честное мировое (через HexGrid::worldPos),
// не индексное. amount может быть отрицательным (стирание/поедание). Каждая
// клетка ограничена [0, max].
void WormSim::depositAt(glm::vec2 worldPos, float amount, float radiusWorld) {
    if (m_fieldCols <= 0 || m_fieldRows <= 0 || m_foodField.empty()) return;
    int centerCol, centerRow;
    worldToHexCell(worldPos, centerCol, centerRow);

    // Пол в 1 клетку - иначе при небольшом radiusWorld кисть могла не задеть
    // ни одной клетки вообще ("радиус ничего не делает").
    const float radius = std::max(m_hexSpacing, radiusWorld);
    const float horiz = HexGrid::horizSpacing(m_hexSpacing);
    const float vert = HexGrid::vertSpacing(m_hexSpacing);
    const int ringCols = static_cast<int>(std::ceil(radius / horiz)) + 1;
    const int ringRows = static_cast<int>(std::ceil(radius / vert)) + 1;
    const float maxConc = std::max(0.0f, params.foodMaxConcentration.load());

    for (int r = std::max(0, centerRow - ringRows); r <= std::min(m_fieldRows - 1, centerRow + ringRows); ++r) {
        for (int c = std::max(0, centerCol - ringCols); c <= std::min(m_fieldCols - 1, centerCol + ringCols); ++c) {
            const glm::vec2 p = m_boundsMin + HexGrid::worldPos(c, r, m_hexSpacing);
            const float dist = glm::length(p - worldPos);
            if (dist > radius) continue;
            const float falloff = 1.0f - dist / radius;
            const std::size_t idx = static_cast<std::size_t>(r) * static_cast<std::size_t>(m_fieldCols) + static_cast<std::size_t>(c);
            m_foodField[idx] = std::clamp(m_foodField[idx] + amount * falloff, 0.0f, maxConc);
        }
    }
}

void WormSim::depositFood(glm::vec2 worldPos, float dtSeconds) {
    std::lock_guard<std::mutex> lock(m_mutex);
    depositAt(worldPos, params.foodDepositAmount.load() * dtSeconds, params.foodDepositRadius.load());
}

void WormSim::removeFood(glm::vec2 worldPos, float dtSeconds) {
    std::lock_guard<std::mutex> lock(m_mutex);
    depositAt(worldPos, -params.foodDepositAmount.load() * dtSeconds, params.foodDepositRadius.load());
}

void WormSim::clearFood() {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::fill(m_foodField.begin(), m_foodField.end(), 0.0f);
}

float WormSim::totalFood() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    float sum = 0.0f;
    for (float v : m_foodField) sum += v;
    return sum;
}

std::vector<float> WormSim::foodFieldSnapshot() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_foodField;
}

// Настоящий газон бактерий истощается там, где по нему ест червь - это не
// приближение, а факт биологии (еда и есть бактерии, которых он поглощает).
// Небольшой фиксированный радиус "рта", независимый от кисти пользователя.
void WormSim::consumeFood(float dt) {
    const float rate = params.foodConsumptionRate.load();
    if (rate <= 0.0f) return;
    // ~1.5 клетки - раньше было 0.6, слишком мало относительно того, как
    // быстро голова уходит от точки за счёт собственного шума/пропульсии:
    // эффект потребления был почти незаметен за обычную игровую сессию.
    depositAt(m_position, -rate * dt, m_hexSpacing * 1.5f);
}

// Явная диффузия по 4 соседям (дискретное уравнение теплопроводности) -
// газон медленно расползается/сглаживается, как настоящая бактериальная
// плёнка. rate ограничен снизу для устойчивости независимо от того, что
// выставлено в UI.
void WormSim::diffuseFood(float dt) {
    const float rate = std::clamp(params.foodDiffusionRate.load() * dt, 0.0f, 0.24f);
    if (rate <= 0.0f || m_foodField.empty()) return;
    std::vector<float> next = m_foodField;
    for (int r = 0; r < m_fieldRows; ++r) {
        for (int c = 0; c < m_fieldCols; ++c) {
            float sum = 0.0f;
            int cnt = 0;
            if (c > 0) { sum += m_foodField[static_cast<std::size_t>(r) * m_fieldCols + c - 1]; ++cnt; }
            if (c < m_fieldCols - 1) { sum += m_foodField[static_cast<std::size_t>(r) * m_fieldCols + c + 1]; ++cnt; }
            if (r > 0) { sum += m_foodField[static_cast<std::size_t>(r - 1) * m_fieldCols + c]; ++cnt; }
            if (r < m_fieldRows - 1) { sum += m_foodField[static_cast<std::size_t>(r + 1) * m_fieldCols + c]; ++cnt; }
            const std::size_t idx = static_cast<std::size_t>(r) * m_fieldCols + c;
            const float cur = m_foodField[idx];
            const float avg = cnt > 0 ? sum / static_cast<float>(cnt) : cur;
            next[idx] = cur + rate * (avg - cur);
        }
    }
    m_foodField.swap(next);
}

// AWA/AWC - абсолютный уровень запаха (плюс независимый шум - без него, при
// отсутствии еды, сеть просто оседает в покое и червь замирает: реальные
// нейроны никогда не молчат идеально, спонтанная активность/шум синапсов -
// не приближение, а факт). ASEL/ASER - РОСТ/ПАДЕНИЕ запаха с прошлого шага:
// это и есть клинокинез - настоящий механизм навигации C. elegans к еде
// (смещённое случайное блуждание через модуляцию частоты поворотов при
// ухудшении градиента), а не наведение на координаты.
float WormSim::applyFoodDrive() {
    const float scent = sampleFood(m_position);
    const float scentDelta = scent - m_prevScent;
    m_prevScent = scent;

    const float noiseAmp = params.spontaneousNoise.load();
    connectome::Network& net = m_loaded.network;
    for (connectome::NeuronId id : m_generalChemoIds) net.set_input(id, scent + signedNoise(noiseAmp));

    const float gradientGain = params.gradientGain.load();
    if (m_aseL != kInvalidId) net.set_input(m_aseL, scentDelta * gradientGain + signedNoise(noiseAmp));
    if (m_aseR != kInvalidId) net.set_input(m_aseR, -scentDelta * gradientGain + signedNoise(noiseAmp));

    return scent;
}

// Статичный линейный градиент в произвольном (настраиваемом) направлении -
// классический thermal-gradient assay (Hedgecock & Russell 1975), не
// дискретное поле: температуру не едят и не красят кистью, это гладкая
// заданная величина. Направление, а не жёстко +X - иначе (тот же урок, что
// уже стоил ложного "эффекта" в tests/worm_chemotaxis_calibration, где еду
// сперва клали ровно по +X, куда и так смотрит стартовый heading) любое
// смещение туда выглядело бы как термотаксис, даже будь оно чистым артефактом
// направления старта.
float WormSim::sampleTemperature(glm::vec2 worldPos) const {
    const glm::vec2 rel = worldPos - m_boundsMin;
    const float angle = params.tempGradientAngle.load();
    const float dirX = std::cos(angle), dirY = std::sin(angle);
    return params.tempBaseline.load() + params.tempGradientSlope.load() * (rel.x * dirX + rel.y * dirY);
}

// AFD - настоящий термосенсорный нейрон C. elegans (Mori & Ohshima 1995) -
// первый синапс которого в загруженном коннектоме и правда идёт на AIY
// (AFDL->AIYL вес 25, AFDR->AIYR вес 29 - того же порядка, что ASE/AWC->AIY),
// то есть контур навигации уже общий с хемотаксисом, ничего нового вниз по
// цепи подключать не нужно. AFDL/AFDR не разделены по функции, как ASEL/ASER
// (у настоящего AFD нет известной такой асимметрии) - получают одинаковый
// сигнал. Знак(T_c - T) перед производной - не прикладное решение, а
// упрощение реального свойства самого сенсора: AFD инвертирует ответ на
// потепление в зависимости от того, ниже или выше T_c текущая температура
// (Clark et al. 2006, Kimura et al. 2004) - без этой инверсии сеть видела бы
// "теплее" как всегда одно и то же (как у еды, где "больше" всегда хорошо),
// и термотаксис не мог бы вообще сходиться к T_c с обеих сторон.
float WormSim::applyTemperatureDrive() {
    const float temp = sampleTemperature(m_position);
    const float tempDelta = temp - m_prevTemp;
    m_prevTemp = temp;

    const float sign = (params.cultivationTemp.load() >= temp) ? 1.0f : -1.0f;
    const float gain = params.thermoGain.load();
    const float noiseAmp = params.spontaneousNoise.load();
    const float signal = sign * tempDelta * gain;

    connectome::Network& net = m_loaded.network;
    if (m_afdL != kInvalidId) net.set_input(m_afdL, signal + signedNoise(noiseAmp));
    if (m_afdR != kInvalidId) net.set_input(m_afdR, signal + signedNoise(noiseAmp));

    return temp;
}

// ADF/NSM - оба реально серотонинергические нейроны C. elegans. Roaming/
// dwelling переключение (Flavell et al. 2013, Cell): серотонин способствует
// dwelling (медленный ход, частые повороты - обычно на еде/рядом с ней),
// нейропептид PDF-1 - противоположному roaming (быстрый, прямой ход).
//
// ВТОРАЯ ВЕРСИЯ (первая - гладкая EMA scent, см. WormSim.h и tests/worm_
// roaming_dwelling_calibration за полной историей находок первой версии, не
// удалёнными - для честности). Настоящие ADF/NSM физиологически реагируют не
// на усреднённую концентрацию, а на ДИСКРЕТНЫЕ акты глотания через
// фарингеальную помпу - импульсный, не гладкий сигнал. Помпа считается
// включённой, только пока под головой РЕАЛЬНО есть еда - честная проверка
// через тот же sampleFood, что уже кормит AWA/AWC/ASE (см. applyFoodDrive),
// не придуманный порог. m_pumpPhase - фазовый аккумулятор такта помпы,
// сбрасывается вне еды (пампинг у настоящего червя быстро гаснет без
// контакта с едой - Raizen et al. - зачем и сбрасывать фазу, а не копить её
// впустую). Между качками ADF/NSM не получают ничего от этого пути, только
// обычный intrinsic-шум - сеть сама решает через свои реальные веса, что
// делать с импульсом, никакого прикладного переключения roaming<->dwelling в
// коде.
void WormSim::applySerotoninDrive() {
    const float scent = sampleFood(m_position);
    const float dt = params.dt.load();
    const float rateHz = std::max(0.0f, params.pharyngealPumpRateHz.load());

    bool pumpFired = false;
    if (scent > 0.0f) {
        m_pumpPhase += rateHz * dt;
        if (m_pumpPhase >= 1.0f) {
            m_pumpPhase -= std::floor(m_pumpPhase);
            pumpFired = true;
        }
    } else {
        m_pumpPhase = 0.0f;
    }

    const float gain = params.serotoninGain.load();
    const float noiseAmp = params.spontaneousNoise.load();
    const float signal = pumpFired ? gain : 0.0f;

    connectome::Network& net = m_loaded.network;
    if (m_adfL != kInvalidId) net.set_input(m_adfL, signal + signedNoise(noiseAmp));
    if (m_adfR != kInvalidId) net.set_input(m_adfR, signal + signedNoise(noiseAmp));
    if (m_nsmL != kInvalidId) net.set_input(m_nsmL, signal + signedNoise(noiseAmp));
    if (m_nsmR != kInvalidId) net.set_input(m_nsmR, signal + signedNoise(noiseAmp));
}

// Реальные нейроны никогда не абсолютно тихи - канальный шум и спонтанный
// выброс медиатора есть у ВСЕХ, не только у сенсорных. Без этого при сильной
// gap junction связи (линейная, электрическая - минует sigmoid активации
// совсем) сеть может стянуться в синхронизированную неподвижную точку, до
// которой шум всего 4-6 хемосенсорных клеток не всегда доходит через
// потенциально насыщенные синапсы - сеть перестаёт на что-либо реагировать.
// Мышцы (Output) пропускаем - у них drive принудительно 0 в Network::step
// независимо от set_input, интрinsic-шум там физически бессмыслен.
void WormSim::applyIntrinsicNoise() {
    const float amp = params.intrinsicNoise.load();
    if (amp <= 0.0f) return;
    connectome::Network& net = m_loaded.network;
    for (connectome::NeuronId i = 0; i < net.size(); ++i) {
        if (net.type(i) == connectome::NeuronType::Output) continue;
        if (i == m_aseL || i == m_aseR) continue; // уже получили scent+шум в applyFoodDrive
        if (i == m_dva) continue; // получит свой шум в applyMechanosensation - иначе один set_input затрёт другой
        if (i == m_afdL || i == m_afdR) continue; // уже получат temp+шум в applyTemperatureDrive
        if (i == m_adfL || i == m_adfR || i == m_nsmL || i == m_nsmR) continue; // уже получат tone+шум в applySerotoninDrive
        if (m_isMotorNeuron[i]) continue; // получат свой шум в applyProprioception - иначе один set_input затрёт другой
        bool isGeneralChemo = false;
        for (connectome::NeuronId id : m_generalChemoIds)
            if (id == i) { isGeneralChemo = true; break; }
        if (isGeneralChemo) continue;
        net.set_input(i, signedNoise(amp));
    }
}

// Проприоцептивная (stretch-receptor) обратная связь: моторный нейрон на
// позиции pos получает сигнал, усреднённый по РЕАЛЬНОМУ (уже прошедшему
// физику и кламп WormBody) углу изгиба на своей позиции и нескольких
// позициях К ХВОСТУ от неё - "локально и постериально", как у настоящих
// B-типа мотонейронов C. elegans (Boyle, Berri & Cohen 2012, PMC3296079:
// "each DB/VB integrates stretch-receptor currents...both locally and
// posteriorly, along its axon"). Знак - тот же, что и curvature (dorsal
// минус ventral): дорсальные моторные нейроны получают сигнал напрямую,
// вентральные - с обратным знаком (упрощение настоящей асимметричной
// GABA-эргической кросс-ингибиции D-класса между сторонами из той же
// работы, но та же суть - одна сторона не должна тянуть туда же, куда
// другая). Читает angles() ПОСЛЕ m_body.step() - это уже физически
// реализованный, ограниченный клампом изгиб (а не сырой, потенциально
// неограниченный внутренний сигнал сети), поэтому обратная связь физически
// не может каскадно разогнаться сама по себе.
//
// НАПРАВЛЕНИЕ ОКНА - ПРОВЕРЕНО, НЕ ПРОСТО ВЫВЕДЕНО ИЗ ЦИТАТЫ: реальные
// B-класс аксоны идут от тела клетки АНТЕРИОРНО (к голове) - при буквальном
// чтении "senses X, wave should propagate anterior-to-posterior for forward
// locomotion" (Boyle/Berri/Cohen, подтверждено отдельным поиском) выглядело,
// что окно должно смотреть К ГОЛОВЕ, а не к хвосту, как сейчас, и что
// нынешний код должен давать движение хвостом вперёд - ровно то, о чём
// сообщил пользователь ("жопой вперёд"). Прежде чем менять код по этой
// логике, гипотеза была проверена: временный режим "wavedir" в
// tests/worm_chemotaxis_calibration/main.cpp (без еды, чистое блуждание,
// метрика - скалярное произведение net-смещения центроида на направление
// points_[0]->points_[N] в конце прогона) на 80 независимых прогонах
// (agar/water, 1200 и 6000 шагов) дал dot = -0.96..-1.00 КАЖДЫЙ РАЗ - тело
// стабильно движется К points_[0], то есть головой вперёд (позиция 1 =
// голова по факту NEURONS.md/parseMotorNeuron: "число в конце моторных
// нейронов и мышц - позиция вдоль тела, голова->хвост" - анатомически не
// вопрос). Гипотеза (это окно причина заднего хода) НЕ подтвердилась -
// сделан вывод: реальная связность загруженного коннектома (401x401,
// настоящие веса Cook et al. 2019) не сводится к "мотонейрон двигает
// ближайшую к себе мышцу", поэтому предсказать направление волны из одной
// геометрии аксона (без учёта фактических синаптических весов) ненадёжно -
// измерение оказалось необходимо, не просто цитаты достаточно. Код НЕ
// менялся. Диагностика пользователя, скорее всего - визуальная: до этой
// правки голова была помечена лишь чуть теплее (та же гамма, что и
// activity-glow) и тонула в свечении хвоста при высокой там активности -
// см. Shaders/worm_body.frag, теперь голова - явный холодный (голубой)
// оттенок, не спутываемый с glow ни при какой его интенсивности.
void WormSim::applyProprioception(const std::vector<float>& bodyAngles) {
    const float gain = params.proprioceptiveGain.load();
    const float noiseAmp = params.intrinsicNoise.load();
    const int window = std::clamp(static_cast<int>(std::lround(params.proprioceptiveOffset.load())), 1, 24);
    // Локальная механосенсация (см. Params::localMechanoGain) - то же окно,
    // те же мотонейроны, независимый знако-нейтральный (не dorsal/ventral,
    // растяжение среды не различает сторону) вклад в тот же set_input, чтобы
    // не перезаписывать сигнал выше вторым вызовом на тот же нейрон.
    const float localMechanoGain = params.localMechanoGain.load();
    const float dragNormal = std::max(1e-6f, params.dragNormal.load());
    const std::vector<float>& segLoad = m_body.segment_load();
    connectome::Network& net = m_loaded.network;
    const int n = static_cast<int>(bodyAngles.size());
    for (const MotorNeuron& mn : m_motorNeurons) {
        const int start = std::clamp(mn.pos - 1, 0, std::max(0, n - 1));
        const int end = std::min(n, start + window);
        float sum = 0.0f;
        for (int i = start; i < end; ++i) sum += bodyAngles[static_cast<std::size_t>(i)];
        const float avgAngle = (end > start) ? sum / static_cast<float>(end - start) : 0.0f;
        float feedback = mn.dorsal ? (gain * avgAngle) : (-gain * avgAngle);

        float loadSum = 0.0f;
        const int loadEnd = std::min(static_cast<int>(segLoad.size()), end);
        for (int i = start; i < loadEnd; ++i) loadSum += segLoad[static_cast<std::size_t>(i)];
        const float avgLoad = (loadEnd > start) ? loadSum / static_cast<float>(loadEnd - start) : 0.0f;
        feedback += localMechanoGain * (avgLoad / dragNormal);

        feedback = std::clamp(feedback, -10.0f, 10.0f); // дешёвая страховка, не должно даже срабатывать
        net.set_input(mn.id, feedback + signedNoise(noiseAmp));
    }
}

// DVA - настоящий stretch-рецепторный нейрон C. elegans (Li, Feng & Xu 2006,
// PMC1500850), физиологически отдельный от B-класса проприоцепции выше:
// та пропагирует бегущую волну вдоль тела локально, DVA ощущает механическую
// нагрузку ЦЕЛИКОМ и модулирует амплитуду локомоции через премоторные
// интернейроны. Честный прокси "ощущаемой нагрузки" без хардкода под
// конкретную среду - m_body.mechanical_load() - реальная суммарная сила
// трения среды, уже посчитанная в WormBody::solve_propulsion при решении
// баланса сил НА ЭТОМ шаге (см. body.cpp), а не параметр среды (drag_normal)
// напрямую. Читается ПОСЛЕ m_body.step() - тот же принцип, что и
// applyProprioception (обратная связь идёт от уже физически случившегося
// результата, не от сырого внутреннего желания сети).
//
// Нормировка на params.dragNormal: сила трения в этой физике раскладывается
// на тангенциальную (ct*v_t) и нормальную (cn*v_n) составляющие (см. вывод в
// body.cpp::solve_propulsion) - при ct=1.0 фиксированном на обоих пресетах
// (см. main.cpp комментарий у кнопок Agar/Water) сырая mechanical_load()
// растёт примерно пропорционально cn=dragNormal (агар/вода отличаются в
// ~23.5 раза по cn - и примерно во столько же по сырой нагрузке, если
// нормальная составляющая доминирует, что для изгиба и есть основной
// случай). Без деления один и тот же gain бьёт по агару и воде с СОВСЕМ
// разной эффективной силой - screen в tests/worm_mechanosensation_calibration
// это прямо показал (агар ломался при gain=0.1, вода держалась до ~3).
// Деление на dragNormal - не подгонка под конкретное число, а обычная для
// механорецепторов относительная (Weber-Fechner-type) перенормировка входа
// под собственный динамический диапазон, тем же путём, что уже
// задокументирован в tests/worm_mechanosensation_calibration как следующий
// шаг.
void WormSim::applyMechanosensation() {
    if (m_dva == kInvalidId) return;
    const float gain = params.mechanoGain.load();
    const float noiseAmp = params.intrinsicNoise.load();
    const float dragNormal = std::max(1e-6f, params.dragNormal.load());
    const float normalizedLoad = m_body.mechanical_load() / dragNormal;
    connectome::Network& net = m_loaded.network;
    net.set_input(m_dva, gain * normalizedLoad + signedNoise(noiseAmp));
}

void WormSim::step() {
    std::lock_guard<std::mutex> lock(m_mutex);
    connectome::Network& net = m_loaded.network;
    net.set_activation_shape(params.activationTheta.load(), params.activationSlope.load());
    net.set_gains(params.chemGain.load(), params.gapGain.load(), params.leakScale.load());
    net.set_active_current(params.bClassOscillatorGain.load(), params.bClassOscillatorTauW.load());
    net.set_peptide_gain(params.pdf1Gain.load(), params.pdf1ReleaseTau.load());

    const float dt = params.dt.load();

    applyFoodDrive();
    applyTemperatureDrive();
    applySerotoninDrive();
    applyIntrinsicNoise();
    net.step(dt);
    consumeFood(dt);
    diffuseFood(dt);

    const float bodyGain = params.bodyGain.load();
    std::vector<float> curvature(kNumSegments, 0.0f);
    for (int pos = 1; pos <= 24; ++pos) {
        auto avg = [&](const std::vector<connectome::NeuronId>& ids) {
            if (ids.empty()) return 0.0f;
            float s = 0.0f;
            for (connectome::NeuronId id : ids) s += net.state(id);
            return s / static_cast<float>(ids.size());
        };
        const float d = avg(m_dorsalByPos[static_cast<std::size_t>(pos)]);
        const float v = avg(m_ventralByPos[static_cast<std::size_t>(pos)]);
        curvature[static_cast<std::size_t>(pos - 1)] = (d - v) * bodyGain;
    }

    // Пространственный high-pass: вычитаем среднее ПО ТЕЛУ curvature этого
    // шага - иначе устойчивый "весь корпус в одну сторону сразу" перекос
    // (не бегущая волна, где сумма по позициям и так ~0) проходит через
    // per-position временной baseline СЛИШКОМ МЕДЛЕННО (тот ловит дрейф в
    // КАЖДОЙ точке отдельно по времени, а не одновременный перекос по ВСЕМ
    // сразу) - тело успевает упереться в кламп на каждом из 24 сегментов за
    // пару шагов, раньше чем baseline его погасит за ~4с. 24 * 0.25 рад =
    // 6.0 рад ~ 2*pi - почти точно замкнутый круг. Подтверждено диагностикой
    // (см. KNOWN OPEN ISSUE в tests/worm_locomotion): под устойчивым
    // контактом с едой знак кривизны становится одинаковым на всех 24
    // позициях разом на сотни секунд - тело буквально замирает, потому что
    // форма перестаёт меняться (solve_propulsion решает нулевую скорость на
    // неизменной форме).
    float meanCurvature = 0.0f;
    for (float c : curvature) meanCurvature += c;
    meanCurvature /= static_cast<float>(curvature.size());
    for (float& c : curvature) c -= meanCurvature;

    // baseline - EMA с постоянной времени в несколько секунд, ловит именно
    // устойчивый перекос (то, что раньше навсегда скручивало тело/крутило
    // курс), а не текущую активность. deviation = curvature - baseline - то,
    // что реально меняется момент к моменту; именно она идёт в тело
    // (WormBody), которое из неё физически выводит перемещение.
    constexpr float kBaselineTau = 4.0f;
    const float alpha = std::clamp(dt / kBaselineTau, 0.0f, 1.0f);
    if (m_curvatureBaseline.size() != curvature.size()) m_curvatureBaseline.assign(curvature.size(), 0.0f);
    std::vector<float> deviation(curvature.size());
    for (std::size_t i = 0; i < curvature.size(); ++i) {
        m_curvatureBaseline[i] += (curvature[i] - m_curvatureBaseline[i]) * alpha;
        deviation[i] = curvature[i] - m_curvatureBaseline[i];
    }

    m_body.set_drag(params.dragTangent.load(), params.dragNormal.load());
    m_body.set_pose_decay_rate(params.bodyPoseDecayRate.load());
    m_body.step(deviation, dt);

    // Проприоцепция - готовит вход для СЛЕДУЮЩЕГО net.step() из РЕАЛЬНОГО
    // (уже прошедшего физику и кламп) изгиба, который тело только что
    // приняло - см. applyProprioception. Реальный механизм, которым
    // C. elegans превращает связность коннектома в бегущую волну (Boyle,
    // Berri & Cohen 2012).
    applyProprioception(m_body.angles());
    applyMechanosensation(); // DVA <- та же уже случившаяся физика, что и выше

    // Перемещение/поворот тела - РЕШЕНИЕ баланса сил анизотропного трения на
    // форме, которую только что породила сеть (см. connectome::WormBody), не
    // отдельная эвристика "знак/модуль deviation -> курс/скорость". Локальная
    // скорость тела поворачивается в мировые координаты текущим heading.
    m_heading += m_body.angular_velocity() * dt;
    const float c = std::cos(m_heading), s = std::sin(m_heading);
    const float lvx = m_body.local_velocity_x(), lvy = m_body.local_velocity_y();
    m_position.x += (lvx * c - lvy * s) * dt;
    m_position.y += (lvx * s + lvy * c) * dt;
    containBody();

    m_lastCurvature = std::move(deviation);
}

// Раньше о границу поля "спотыкалась" только точка отсчёта тела (points_[0]) -
// остальная его длина могла вылезать за край, пока она сама ещё в пределах.
// Настоящая коллизия - на всю длину: считаем мировой bbox ВСЕХ точек тела и,
// если он вылезает за границу, сдвигаем всё тело обратно целиком. При касании
// стены дополнительно отражаем курс (простой, но честный отскок) - иначе
// червь просто вжимался бы в стену снова и снова вместо того, чтобы от неё
// отвернуть.
void WormSim::containBody() {
    const int n = m_body.num_segments();
    const auto& lx = m_body.points_x();
    const auto& ly = m_body.points_y();

    // bbox тела в мировых координатах для ДАННОГО heading - используется и
    // до, и после возможного отражения курса (форма/точки не меняются, меняется
    // только его ориентация, поэтому bbox нужно пересчитать заново, если
    // heading поменялся, иначе сдвиг позиции окажется рассчитан для уже не
    // актуальной ориентации и тело всё равно вылезет за границу).
    auto worldBBox = [&](float heading, float& minX, float& maxX, float& minY, float& maxY) {
        const float c = std::cos(heading), s = std::sin(heading);
        minX = 1e30f; maxX = -1e30f; minY = 1e30f; maxY = -1e30f;
        for (int i = 0; i <= n; ++i) {
            const float lxp = lx[static_cast<std::size_t>(i)], lyp = ly[static_cast<std::size_t>(i)];
            const float wx = m_position.x + lxp * c - lyp * s;
            const float wy = m_position.y + lxp * s + lyp * c;
            minX = std::min(minX, wx);
            maxX = std::max(maxX, wx);
            minY = std::min(minY, wy);
            maxY = std::max(maxY, wy);
        }
    };

    float minX, maxX, minY, maxY;
    worldBBox(m_heading, minX, maxX, minY, maxY);

    const bool hitX = minX < m_boundsMin.x || maxX > m_boundsMax.x;
    const bool hitY = minY < m_boundsMin.y || maxY > m_boundsMax.y;
    if (hitX || hitY) {
        constexpr float kPi = 3.14159265f;
        if (hitX) m_heading = kPi - m_heading;
        if (hitY) m_heading = -m_heading;
        worldBBox(m_heading, minX, maxX, minY, maxY); // курс сменился - bbox тоже
    }

    if (minX < m_boundsMin.x) m_position.x += (m_boundsMin.x - minX);
    else if (maxX > m_boundsMax.x) m_position.x -= (maxX - m_boundsMax.x);
    if (minY < m_boundsMin.y) m_position.y += (m_boundsMin.y - minY);
    else if (maxY > m_boundsMax.y) m_position.y -= (maxY - m_boundsMax.y);
}

void WormSim::snapshot(Snapshot& out) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    const int n = m_body.num_segments();
    const auto& lx = m_body.points_x();
    const auto& ly = m_body.points_y();
    const float c = std::cos(m_heading), s = std::sin(m_heading);

    out.pointsX.resize(static_cast<std::size_t>(n) + 1);
    out.pointsY.resize(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const float x = lx[static_cast<std::size_t>(i)], y = ly[static_cast<std::size_t>(i)];
        out.pointsX[static_cast<std::size_t>(i)] = m_position.x + x * c - y * s;
        out.pointsY[static_cast<std::size_t>(i)] = m_position.y + x * s + y * c;
    }

    out.glow.assign(static_cast<std::size_t>(n) + 1, 0.0f);
    for (int i = 0; i <= n; ++i) {
        const float cc = m_lastCurvature.empty()
                              ? 0.0f
                              : m_lastCurvature[static_cast<std::size_t>(std::min(i, n - 1))];
        out.glow[static_cast<std::size_t>(i)] = std::tanh(std::fabs(cc));
    }

    out.nodeCount = static_cast<int>(m_loaded.network.size());
    out.nodeStates.resize(static_cast<std::size_t>(out.nodeCount));
    for (int i = 0; i < out.nodeCount; ++i)
        out.nodeStates[static_cast<std::size_t>(i)] = m_loaded.network.state(static_cast<connectome::NeuronId>(i));
}
