#include "WormSim.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <cstring>
#include <random>
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

// ГОЛОВНЫЕ МОТОРНЫЕ НЕЙРОНЫ. Вентральный тяж (классы выше) не иннервирует
// передние позиции ВООБЩЕ: взвешенный по синапсам центр иннервации ни одного
// его нейрона не попадает в 1-5, поэтому голова в этой модели не колебалась.
// У C. elegans головную мускулатуру ведёт отдельная группа - RMD, SMD, SMB,
// URA, RIV (White et al. 1986; Gray, Hill & Bargmann 2005 - именно эти клетки
// отвечают за head casting, поисковые взмахи головой). Все они присутствуют в
// загруженном коннектоме как ProcessingOutput, то есть их вход уже считается
// настоящей связностью - не использовался только их ВЫХОД.
//
// Дорсальность/вентральность читается из самого имени: буква D или V в
// позиции после класса - это анатомическая номенклатура (RMDD дорсальный,
// RMDV вентральный), а не соглашение этого кода. Имена без неё (RMDL/RMDR -
// латеральные) пропускаются: они не проецируют ни на дорсальную, ни на
// вентральную сторону.
//
// Позиция вдоль тела: класс раскладывается на переднюю часть тела. RMD и URA
// самые передние, SMD за ними, SMB ещё дальше - порядок соответствует
// анатомическому расположению их отростков.
bool parseHeadMotorNeuron(const std::string& name, int& position, bool& dorsal) {
    struct HeadClass { const char* prefix; int pos; };
    static const HeadClass kHead[] = {
        {"RMD", 1}, {"URA", 2}, {"SMD", 3}, {"RIV", 4}, {"SMB", 5},
    };
    for (const HeadClass& hc : kHead) {
        const std::size_t plen = std::strlen(hc.prefix);
        if (name.size() <= plen || name.compare(0, plen, hc.prefix) != 0) continue;
        const char side = name[plen];
        if (side != 'D' && side != 'V') return false;  // латеральные RMDL/RMDR - не наши
        position = hc.pos;
        dorsal = (side == 'D');
        return true;
    }
    return false;
}

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
    // ИСПРАВЛЕНО (был реальный баг, найден workflow'ом при верификации
    // WORM_V3, см. WORM_V3_RESULTS.md): раньше здесь стоял безусловный
    // std::srand(time(nullptr)) - затирал ЛЮБОЙ сид, который вызывающий код
    // выставил СВОИМ std::srand(seed) до создания WormSim. Каждый тестовый
    // харнесс в этом проекте (tests/worm_*) делает ровно это - srand(seed)
    // перед WormSim sim(...) - и ожидает, что этот сид реально подействует
    // на весь прогон. На деле же весь шум внутри симуляции (frand()/
    // signedNoise(), applyChemotaxis/applyThermosensation/
    // applyIntrinsicNoise/applyMechanosensation и т.д.) шёл по
    // последовательности, инициализированной ВРЕМЕНЕМ КОНСТРУКЦИИ, а не
    // переданным сидом - т.е. "сид" в CLI большинства тестов этой сессии
    // фактически ни на что не влиял, реальная случайность бралась из
    // wall-clock времени в момент создания WormSim. Агрегатная статистика
    // по многим прогонам (health-fraction, средние по 16+ базам) от этого не
    // страдает - реальная случайность там всё равно была, просто
    // неконтролируемая; НО любое утверждение вида "тот же сид -> тот же
    // прогон" (например, сравнение фазы agar/water трассировкой на
    // одинаковом seed=999 в этой сессии) было ошибочным - на деле сравнивались
    // два НЕЗАВИСИМЫХ случайных прогона, не один и тот же шум под разным
    // трением. Сидирование теперь целиком на совести вызывающего кода - как
    // и предполагали все тесты с самого начала; для живого Demo_worm сид
    // выставляется один раз при старте программы (main.cpp), не на каждое
    // создание WormSim.
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

    // Per-neuron-CLASS leak/capacitance, FOURTH attempt on this general axis
    // (after the three reverted ones above), THIS TIME SHIPPED - by direct
    // user request, after the shipped cpgGain/muscleBandwidthGain point
    // (below) was confirmed still ~14-21x below real C. elegans tempo and
    // three further searches on cpgGain/muscleBandwidthGain itself found
    // nothing better (see tests/worm_cpg_muscle_bandwidth_calibration and
    // WORM.md section 6). See tests/worm_leak_capacitance_tempo_calibration
    // for the full search + validation history - built specifically to avoid
    // repeating the two documented leak/capacitance failures above: an
    // efficiency>=0.40 floor from trial 1 (not added after a live surprise,
    // like the chemotaxis attempt), a maxHeadingDelta<=0.5 rad gate from
    // trial 1 (not added after the synapse-sign attempt's single-step
    // heading-reversal was caught by an unrelated test), and mandatory
    // 16-independent-base distribution confirmation of every candidate that
    // clears the screen gate (not deferred to a later verification step,
    // which is exactly how the speed-calibration attempt's false 4.85x
    // shipped in the first place).
    //
    // Searched ON TOP of the already-shipped cpgGain/muscleBandwidthGain
    // point (not isolated against zero, unlike the three attempts above) -
    // the goal was improving THAT configuration's tempo specifically. Broad
    // 8-class-param screen (1100 trials): only 3/1100 cleared the health+
    // ratio>=1.15 gate, none beat the shipped baseline after confirmation
    // (one was caught as a single-base illusion: healthy in only 2/16 bases).
    // Narrower follow-up (1100 more trials) held InputProcessing/Processing
    // at identity and searched only ProcessingOutput/Output leak+capacitance
    // - the one lever genuinely new this session (Output's leak used to be
    // mathematically dead with muscle_leak_scale_ always exactly 0; it is
    // live now that muscleBandwidthGain ships nonzero) - hit rate rose to
    // 14/1100, and one candidate held up under full 16-base confirmation:
    // leakPO=0.712/leakO=0.45/capPO=1.286/capO=0.68 -> agar healthy=16/16
    // (0.0283Hz, was 0.0192Hz, +47%), water healthy=16/16 (0.1232Hz, was
    // 0.1313Hz, -6%), ratio=2.908 (was 2.692, still inside the user's
    // requested 200-300% target), water>agar in all 16 bases. A real,
    // confirmed, if modest, step - agar frequency moved from ~4.8% of the
    // real target (0.4Hz) to ~7.1%, tempo is still far from solved.
    //
    // V2 REMOVAL (see WORM_V2_DESIGN.md section 2.3): both calls below are
    // REMOVED. This calibration was found ON TOP of the already-shipped
    // cpgGain/muscleBandwidthGain point (explicitly, see paragraph above) -
    // its entire justification was "improve THAT configuration's tempo",
    // and that configuration (CPG + load-dependent bandwidth gains) is
    // itself removed as part of the from-scratch rebuild (see
    // Params::muscleLeakScale/muscleCalciumTau in WormSim.h). Returning to
    // honest identity (leak_scale=1, capacitance_scale=1 for every type) is
    // not a separately-validated decision - it is the mandatory consequence
    // of the calibration's only cited justification disappearing along with
    // CPG. Kept here (not deleted) as the historical record of what was
    // tried and why it no longer applies.
    // net.scale_type_params(connectome::NeuronType::ProcessingOutput, 0.712f, 1.286f);  -- REMOVED, see above
    // net.scale_type_params(connectome::NeuronType::Output, 0.45f, 0.68f);  -- REMOVED, see above

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
        else if (name == "ADEL") m_adeL = i;
        else if (name == "ADER") m_adeR = i;
        else if (name == "PDEL") m_pdeL = i;
        else if (name == "PDER") m_pdeR = i;
        else if (name == "AWAL" || name == "AWAR" || name == "AWCL" || name == "AWCR")
            m_generalChemoIds.push_back(i);

        const auto& m = m_loaded.muscles[i];
        if (m.is_muscle && m.position >= 1 && m.position <= 24) {
            (m.side == 'D' ? m_dorsalByPos : m_ventralByPos)[static_cast<std::size_t>(m.position)].push_back(i);
        }

        int motorPos; bool motorDorsal;
        // Головные моторные нейроны - см. parseHeadMotorNeuron. Включаются
        // флагом, потому что это изменение состава привода, а не настройка:
        // при headMotorEnabled=0 состав m_motorNeurons побитово прежний.
        const bool isHead =
            params.headMotorEnabled.load() != 0 && parseHeadMotorNeuron(name, motorPos, motorDorsal);
        if (isHead) {
            // Позиция головного нейрона анатомическая, синтетическая
            // "растяжка по индексу класса" к нему неприменима, поэтому обе
            // позиции равны найденной.
            m_motorNeurons.push_back({i, motorPos, motorDorsal, motorPos, motorPos, true});
            m_isMotorNeuron[i] = true;
            m_headMotorNeurons.push_back(i);
        } else if (parseMotorNeuron(name, motorPos, motorDorsal)) {
            // posInnervation заполняется ниже (нужен полный m_loaded.muscles,
            // а он дочитывается этим же циклом) - пока дублируем синтетическую.
            m_motorNeurons.push_back({i, motorPos, motorDorsal, motorPos, motorPos});
            m_isMotorNeuron[i] = true;
        }

        // B-класс (DB1-7 дорсальные, VB1-11 вентральные) - настоящие
        // локальные осцилляторы переднего хода (Fouad et al. 2018, eLife
        // 7:e29913) - см. Params::bClassOscillatorGain.
        if (name.size() >= 3 && (name[0] == 'D' || name[0] == 'V') && name[1] == 'B' &&
            std::isdigit(static_cast<unsigned char>(name[2])))
            m_classBMotorNeurons.push_back(i);
    }
    // posInnervation: взвешенный по синаптическому весу центр РЕАЛЬНЫХ
    // мышечных мишеней каждого мотонейрона - см. WormSim::MotorNeuron и
    // Params::motorPositionSource. Обходим CSR обеих матриц (row=приёмник,
    // col_index[k]=источник, см. csr_matrix.hpp) и копим для источника
    // weight*position его мышечных приёмников.
    //
    // Веса берутся ПО МОДУЛЮ: у тормозных (D-класс, отрицательный вес)
    // синапсов позиция мишени такая же осмысленная, а знак здесь означает
    // направление действия, не "минус позицию". Нейроны без единой мышечной
    // мишени (интернейроны, случайно прошедшие parseMotorNeuron) сохраняют
    // синтетическую позицию - явный fallback, а не 0.
    {
        std::vector<float> posWeightSum(net.size(), 0.0f);
        std::vector<float> weightSum(net.size(), 0.0f);
        auto accumulate = [&](const connectome::CsrMatrix& m) {
            const auto& rowPtr = m.row_ptr();
            const auto& colIdx = m.col_index();
            const auto& w = m.weights();
            for (connectome::NeuronId target = 0; target + 1 < rowPtr.size(); ++target) {
                const auto& mi = m_loaded.muscles[target];
                if (!mi.is_muscle || mi.position < 1 || mi.position > 24) continue;
                for (std::uint32_t k = rowPtr[target]; k < rowPtr[target + 1]; ++k) {
                    const float aw = std::fabs(w[k]);
                    posWeightSum[colIdx[k]] += aw * static_cast<float>(mi.position);
                    weightSum[colIdx[k]] += aw;
                }
            }
        };
        accumulate(net.chemical());
        accumulate(net.gap());
        for (MotorNeuron& mn : m_motorNeurons) {
            if (weightSum[mn.id] > 1e-6f) {
                const int p = static_cast<int>(std::lround(posWeightSum[mn.id] / weightSum[mn.id]));
                mn.posInnervation = std::clamp(p, 1, 24);
            }
        }
    }

    net.set_active_current_targets(m_classBMotorNeurons);
    {
        std::vector<connectome::NeuronId> motorIds;
        motorIds.reserve(m_motorNeurons.size());
        for (const MotorNeuron& mn : m_motorNeurons) motorIds.push_back(mn.id);
        net.set_motor_leak_targets(std::move(motorIds));  // см. Params::motorLeakScale - множитель, а не гейн, дефолт 1.0
    }
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

void WormSim::setMedium(float dragTangent, float dragNormal) {
    // См. заголовочный комментарий в WormSim.h. step() держит m_mutex на
    // всю свою длительность (см. ниже) - беря тот же мьютекс здесь, эта
    // запись пары и любое чтение внутри step() становятся взаимно
    // исключающими, так что step() никогда не увидит наполовину
    // обновлённую пару (новый dragTangent со старым dragNormal или
    // наоборот).
    std::lock_guard<std::mutex> lock(m_mutex);
    params.dragTangent = dragTangent;
    params.dragNormal = dragNormal;
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

float WormSim::foodAt(glm::vec2 worldPos) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return sampleFood(worldPos);
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

    // НАСЫЩЕНИЕ СЕНСОРНОГО ОТВЕТА (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md
    // раздел 18). Раньше в нейроны уходила СЫРАЯ концентрация поля, а поле
    // масштабируется кистью: при foodMaxConcentration=1000 (значение,
    // необходимое для градиента шириной в несколько длин тела - см. раздел 15)
    // на AWA/AWC приходил вход величиной 400-900 при том, что всё остальное в
    // этой сети на два-три порядка меньше (intrinsicNoise 0.4, проприоцепция
    // зажата в +/-10, chemGain 0.02). Итог был виден живьём: при заходе на еду
    // червь неестественно дёргался вперёд.
    //
    // Ответ рецептора у настоящего животного - насыщающаяся функция
    // концентрации, а не линейная. Хилл/Михаэлис-Ментен первого порядка
    // scent/(scent+K) лежит в [0,1) при любой концентрации, поэтому масштаб
    // поля больше не может пробить сеть: он влияет только на то, как быстро
    // ответ выходит на насыщение.
    //
    // K = chemoSensorHalfSaturation. Дефолт 0.8 выбран так, чтобы на ДЕФОЛТНОЙ
    // кисти (пик концентрации ~0.2, см. измеренный профиль в разделе 15.2)
    // ответ 0.2/(0.2+0.8) = 0.2 численно совпал с прежним сырым входом - то
    // есть поведение на дефолтных настройках поля практически не меняется, а
    // ломается только патологический режим при большом поле.
    // K<=0 отключает насыщение (вход = сырая концентрация, побитово прежнее
    // поведение) - оставлено как способ прямого сравнения, а не как режим для
    // использования: именно сырой вход и давал рывок при заходе на еду.
    const float halfSat = params.chemoSensorHalfSaturation.load();
    const float response = (halfSat > 0.0f) ? (scent / (scent + halfSat)) : scent;
    // Производная берётся от ОТВЕТА, а не от сырой концентрации - иначе
    // градиентный канал остался бы непропорционально большим ровно там, где
    // абсолютный канал уже насыщен.
    const float responseDelta = response - m_prevScent;
    m_prevScent = response;
    // Тот же ответ рецептора, что уходит в ASE, но сохранённый как СКОРОСТЬ
    // (в секунду, не за шаг) для триггера пируэтов - см. updateLocomotionState.
    // Пируэт по построению должен читать ровно тот сигнал, который читает
    // животное, а не отдельный "прикладной" градиент.
    {
        const float dtNow = std::max(1e-6f, params.dt.load());
        // СГЛАЖИВАНИЕ ОБЯЗАТЕЛЬНО, и это не косметика. Поле еды лежит на
        // гексагональной сетке с шагом 36 единиц, а червь проходит за шаг около
        // 2.3 единицы (0.08 длины тела в секунду при dt=0.05). Значит на
        // большинстве шагов выборка поля не меняется ВООБЩЕ, а раз в ~15 шагов
        // прыгает на целую клетку: поразностная производная - это последовательность
        // импульсов от пересечения клеток, а не градиент. Триггер пируэта,
        // построенный на ней, срабатывает по квантованию сетки.
        //
        // Постоянная времени порядка секунд ещё и биологически верна: ASE
        // интегрирует изменение концентрации на масштабе сотен миллисекунд -
        // секунд, а не мгновенно.
        const float tau = std::max(1e-3f, params.reversalSensorTau.load());
        const float alpha = 1.0f - std::exp(-dtNow / tau);
        m_scentRate += (responseDelta / dtNow - m_scentRate) * alpha;
    }

    const float noiseAmp = params.spontaneousNoise.load();
    connectome::Network& net = m_loaded.network;
    for (connectome::NeuronId id : m_generalChemoIds) net.set_input(id, response + signedNoise(noiseAmp));

    const float gradientGain = params.gradientGain.load();
    if (m_aseL != kInvalidId) net.set_input(m_aseL, responseDelta * gradientGain + signedNoise(noiseAmp));
    if (m_aseR != kInvalidId) net.set_input(m_aseR, -responseDelta * gradientGain + signedNoise(noiseAmp));

    // Возвращается СЫРАЯ концентрация: вызывающий код использует её для
    // индикации/потребления, где нужна именно величина еды, а не ответ
    // рецептора.
    return scent;
}

// ПИРУЭТЫ - вторая, и у настоящего червя ГЛАВНАЯ, стратегия хемотаксиса.
//
// Pierce-Shimomura, Morse & Lockery 1999, J Neurosci 19(21):9557-9569, "The
// fundamental role of pirouettes in Caenorhabditis elegans chemotaxis":
// животное идёт прямыми пробегами, а вероятность начать пируэт - резкий
// разворот из одного или нескольких реверсов и омега-поворотов - зависит от
// dC/dt: растёт, когда червь идёт ВНИЗ по градиенту, и подавляется, когда
// вверх. Направление после пируэта почти случайно. Авторы показывают, что
// одного этого механизма достаточно, чтобы объяснить хемотаксис целиком.
//
// До этого момента в проекте был только клинотаксис (плавное подруливание,
// chemoSteeringGain) - вторая половина навигации отсутствовала, и это видно
// было прямо в метрике: эффективность пути 0.95, то есть червь не
// переориентировался ВООБЩЕ.
//
// МЕХАНИЗМ РЕВЕРСА, а не кинематический трюк. Направление бегущей волны
// изгиба задаётся направлением проприоцептивной связи: B-класс (DB/VB, ход
// вперёд, под AVB) читает кривизну в одну сторону вдоль тела, A-класс (DA/VA,
// ход назад, под AVA) - в противоположную (Chalfie et al. 1985; так же
// реализовано у Boyle, Berri & Cohen 2012). В этом коде направление окна уже
// вынесено в Params::proprioceptiveAnterior, и его переключение УЖЕ ИЗМЕРЕНО
// как переключение направления хода (16/16 задом наперёд против 0/16). Поэтому
// реверс здесь - ровно смена активного класса моторных нейронов, выраженная
// через направление их проприоцептивного окна. Никакого отдельного "поехали
// назад" в кинематике нет.
//
// МЕХАНИЗМ ОМЕГА-ПОВОРОТА - тоже не кинематический. Gray, Hill & Bargmann
// 2005, PNAS 102(9):3184-3191: омега это глубокий изгиб на одну сторону,
// голова достаёт до хвоста, животное выходит с большой сменой курса. Здесь это
// СМЕЩЕНИЕ КРИВИЗНЫ одного знака, добавляемое к приводу передней части тела -
// поворот дальше делает та же физика трения, что и всё остальное движение.
void WormSim::updateLocomotionState(float dt) {
    if (params.pirouetteEnabled.load() == 0) {
        m_locomotion = Locomotion::Forward;
        m_locomotionTimer = 0.0f;
        return;
    }

    m_locomotionTimer -= dt;

    switch (m_locomotion) {
        case Locomotion::Forward: {
            // Частота инициации пируэта как функция dC/dt. Экспоненциальная
            // форма, а не линейная: частота обязана оставаться положительной
            // при любом градиенте, и подавление вверх по градиенту у
            // Pierce-Shimomura выглядит именно как насыщающееся, а не как
            // выход в ноль. Пределы снизу/сверху - чтобы ступенька в поле
            // (поедание выгрызает яму, см. chemoSensorHalfSaturation) не могла
            // ни запереть червя в пируэте, ни выключить их совсем.
            const float rate = std::clamp(params.reversalBaseRate.load()
                                              * std::exp(-params.reversalGradientGain.load() * m_scentRate),
                                          params.reversalRateMin.load(), params.reversalRateMax.load());
            const float p = 1.0f - std::exp(-rate * dt);
            if (frand() < p) {
                m_locomotion = Locomotion::Reverse;
                const float lo = params.reversalDurationMin.load();
                const float hi = std::max(lo, params.reversalDurationMax.load());
                m_locomotionTimer = lo + frand() * (hi - lo);
            }
            break;
        }
        case Locomotion::Reverse: {
            if (m_locomotionTimer <= 0.0f) {
                // Часть реверсов переходит в омега-поворот, часть просто
                // заканчивается возвратом хода вперёд - Gray et al. 2005
                // описывают оба исхода.
                if (frand() < params.omegaProbability.load()) {
                    m_locomotion = Locomotion::Omega;
                    m_locomotionTimer = params.omegaDuration.load();
                    m_omegaSign = (frand() < 0.5f) ? -1.0f : 1.0f;
                } else {
                    m_locomotion = Locomotion::Forward;
                    m_locomotionTimer = 0.0f;
                }
            }
            break;
        }
        case Locomotion::Omega: {
            if (m_locomotionTimer <= 0.0f) {
                m_locomotion = Locomotion::Forward;
                m_locomotionTimer = 0.0f;
            }
            break;
        }
    }
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

    // ТЕРМАЛЬНЫЙ ИМПРИНТИНГ - единственная в этой модели НАСТОЯЩАЯ выученная
    // память, то есть состояние, которое переживает опыт, а не просто
    // релаксирует к нулю.
    //
    // У C. elegans комфортная температура T_c не врождённая: животное
    // запоминает температуру, при которой росло, и потом мигрирует к ней
    // (Hedgecock & Russell 1975). Механизм - изменение свойств AFD/AIY при
    // длительном содержании (Kimura et al. 2004; Clark et al. 2006: AFD
    // инвертирует знак ответа на потепление в зависимости от того, ниже или
    // выше T_c текущая температура - это и делает термотаксис сходящимся к
    // T_c, а не просто "теплее всегда лучше"). Память перезаписывается за
    // часы при переносе на другую температуру - то есть это медленный
    // след пережитого, а не константа.
    //
    // Здесь ровно это: T_c ползёт к средней пережитой температуре с большой
    // постоянной времени. thermalImprintTau<=0 (выключено) оставляет
    // cultivationTemp неизменным - побитово прежнее поведение.
    //
    // Величина постоянной времени: у настоящего червя перезапись занимает
    // часы, здесь же характерный прогон - сотни секунд, поэтому масштаб взят
    // от прогона, а не от животного. Это ЯВНОЕ несоответствие масштабов, а не
    // калибровка: сделать его биологическим значило бы, что в любом прогоне
    // этой модели память не успевает измениться вообще и механизм невозможно
    // ни увидеть, ни проверить.
    {
        const float tau = params.thermalImprintTau.load();
        if (tau > 0.0f) {
            const float dtNow = std::max(1e-6f, params.dt.load());
            const float alpha = 1.0f - std::exp(-dtNow / tau);
            params.cultivationTemp = params.cultivationTemp.load()
                                     + (temp - params.cultivationTemp.load()) * alpha;
        }
    }

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
        if (i == m_adeL || i == m_adeR || i == m_pdeL || i == m_pdeR) continue; // уже получат сигнал в applyDopamineDrive
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
    // См. Params::motorPositionSource - какая из двух позиций замыкает петлю.
    const bool useInnervationPos = params.motorPositionSource.load() != 0;
    // См. Params::proprioceptiveAnterior. В фазе реверса окно смотрит в
    // ПРОТИВОПОЛОЖНУЮ сторону - это и есть смена активного класса моторных
    // нейронов (B-класс вперёд, A-класс назад), см. updateLocomotionState.
    // Никакого отдельного механизма заднего хода нет, направление волны
    // задаётся только этим.
    const bool anteriorParam = params.proprioceptiveAnterior.load() != 0;
    const bool anteriorWindow = (m_locomotion == Locomotion::Reverse) ? !anteriorParam : anteriorParam;
    for (const MotorNeuron& mn : m_motorNeurons) {
        // Головные - вне проприоцептивной петли, см. MotorNeuron::head.
        if (mn.head) continue;
        const int mnPos = useInnervationPos ? mn.posInnervation : mn.posSynthetic;
        const int self = std::clamp(mnPos - 1, 0, std::max(0, n - 1));
        // См. Params::proprioceptiveAnterior. Оба окна включают собственную
        // позицию нейрона и имеют одинаковую ширину - отличается только
        // сторона, в которую они смотрят.
        const int start = anteriorWindow ? std::max(0, self - window + 1) : self;
        const int end = anteriorWindow ? (self + 1) : std::min(n, self + window);
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

// Генератор ритма ходьбы (CPG) - УДАЛЁН ЦЕЛИКОМ (v1, см. Params::cpgGain в
// WORM.md разделе 6 и WORM_V2_DESIGN.md разделе 1 за полным обоснованием
// удаления). Был единственным сознательным исключением из "поведение только
// из настоящей сети" в этом проекте - внешний фазовый генератор ритма,
// впрыскиваемый в мотонейроны в обход всей собственной динамики сети. По
// честной оценке WORM_V2_DESIGN.md, именно он тянул на себе большую часть
// прежнего ratio=3.523/agar=0.0637Hz - его удаление и есть ядро мандата "не
// искать темп снаружи, а вычислить его изнутри"; ожидаемая честная цена -
// заметная просадка темпа/ratio относительно старой точки, см.
// WORM_V2_RESULTS.md за измеренным исходом.

// Дофамин-подобное адаптивное состояние - см. Params::dopamineGain за полным
// обоснованием (второе, после CPG, сознательное исключение из "поведение
// только из сети"). Два независимых куска:
//  1. Реальный сигнал в ADEL/ADER/PDEL/PDER, пропорциональный реально
//     посчитанной нагрузке - честности ради: это ТОЛЬКО связность/
//     визуализация (настоящие дофаминергические нейроны реагируют на
//     реальную нагрузку, видно в диагностике), сеть эти нейроны никуда
//     дальше в моторный путь не проецирует (не в этом коннектоме/не в этой
//     редукции) - сигнал НЕ участвует в вычислении эффекта ниже. Отдельный
//     параметр (dopamineGain), не завязан на dopamineMotorLeakGain.
//  2. m_dopamineTone - медленный EMA той же нагрузки (постоянная времени
//     dopamineToneTau, на порядки медленнее быстрой синаптической передачи -
//     реальные нейромодуляторы действуют секунды-минуты, не миллисекунды),
//     который в step() домножает motorLeakScale на (1+dopamineMotorLeakGain*
//     tone) - НЕ константа, найденная поиском на обе среды сразу (как все
//     пять провалившихся осей, см. WORM.md раздел 6), а состояние, которое
//     само медленно подстраивается под то, в какой среде червь реально
//     находится ПРЯМО СЕЙЧАС.
void WormSim::applyDopamineDrive() {
    const float dragNormal = std::max(1e-6f, params.dragNormal.load());
    const float normalizedLoad = m_body.mechanical_load() / dragNormal;

    const float gain = params.dopamineGain.load();
    if (gain != 0.0f) {
        connectome::Network& net = m_loaded.network;
        const float signal = gain * normalizedLoad;
        if (m_adeL != kInvalidId) net.add_input(m_adeL, signal);
        if (m_adeR != kInvalidId) net.add_input(m_adeR, signal);
        if (m_pdeL != kInvalidId) net.add_input(m_pdeL, signal);
        if (m_pdeR != kInvalidId) net.add_input(m_pdeR, signal);
    }

    if (params.dopamineMotorLeakGain.load() != 0.0f) {
        const float dt = params.dt.load();
        const float tau = std::max(0.1f, params.dopamineToneTau.load());
        const float alpha = std::clamp(dt / tau, 0.0f, 1.0f);
        m_dopamineTone += (normalizedLoad - m_dopamineTone) * alpha;
    }
}

// Тестовый (test-only) диагностический хук - см. debugKickMotorNeurons()'s
// комментарий в WormSim.h за полным обоснованием. Локальный std::mt19937,
// засеянный явно переданным seed (не глобальный rand()/srand()) - случайный
// per-neuron знак не даёт толчку выродиться в единый DC-сдвиг, который
// step()'s пространственное вычитание среднего тут же уничтожило бы.
void WormSim::debugKickMotorNeurons(float magnitude, unsigned seed) {
    std::lock_guard<std::mutex> lock(m_mutex);
    connectome::Network& net = m_loaded.network;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> signDist(0, 1);
    for (int pos = 1; pos <= 24; ++pos) {
        for (connectome::NeuronId id : m_dorsalByPos[static_cast<std::size_t>(pos)]) {
            const float sign = signDist(rng) ? 1.0f : -1.0f;
            net.kick_state(id, sign * magnitude);
        }
        for (connectome::NeuronId id : m_ventralByPos[static_cast<std::size_t>(pos)]) {
            const float sign = signDist(rng) ? 1.0f : -1.0f;
            net.kick_state(id, sign * magnitude);
        }
    }
}

void WormSim::step() {
    std::lock_guard<std::mutex> lock(m_mutex);
    connectome::Network& net = m_loaded.network;
    net.set_activation_shape(params.activationTheta.load(), params.activationSlope.load());
    net.set_gains(params.chemGain.load(), params.gapGain.load(), params.leakScale.load());
    net.set_active_current(params.bClassOscillatorGain.load(), params.bClassOscillatorTauW.load());
    net.set_peptide_gain(params.pdf1Gain.load(), params.pdf1ReleaseTau.load());
    // V2 (см. WORM_V2_DESIGN.md раздел 2.3): muscleLeakScale больше НЕ
    // load-модулированный гейн (muscleBandwidthGain удалён вместе с CPG,
    // чью проблему он обслуживал) - прямое фиксированное физиологическое
    // значение утечки, см. Params::muscleLeakScale. muscleCalciumTau -
    // реальная кальциевая постоянная времени, теперь боевой параметр (0.3с
    // по дефолту), а не выключенная инфраструктура.
    net.set_muscle_leak(params.muscleLeakScale.load());
    net.set_muscle_calcium_tau(params.muscleCalciumTau.load());
    // motorLeakScale эффективно домножается на медленный дофаминовый тон
    // (m_dopamineTone, обновлён ПРОШЛЫМ шагом в applyDopamineDrive - см.
    // Params::dopamineMotorLeakGain) - множитель тождественно 1.0 при
    // gain=0, побитово прежнее поведение. motorBandwidthGain (нагрузочная
    // пропускная способность, обслуживавшая тот же CPG) удалён.
    net.set_motor_leak_scale(params.motorLeakScale.load() *
                              (1.0f + params.dopamineMotorLeakGain.load() * m_dopamineTone));

    const float dt = params.dt.load();

    applyFoodDrive();
    // Машина состояний пируэта читает m_scentRate, который только что обновил
    // applyFoodDrive - тот же сигнал, что уходит в ASE, а не отдельный.
    updateLocomotionState(dt);
    applyTemperatureDrive();
    applySerotoninDrive();
    applyIntrinsicNoise();
    net.step(dt);
    consumeFood(dt);
    diffuseFood(dt);

    const float bodyGain = params.bodyGain.load();
    std::vector<float> curvature(kNumSegments, 0.0f);
    // Диагностика (Phase 2, "где именно рождается распад") - ПАРАЛЛЕЛЬНЫЙ
    // снимок того же (d-v)*bodyGain, но из net.state(id) (сырое V
    // мотонейронов, ДО muscle_calcium_ фильтра), а не net.muscle_output(id) -
    // см. debugRawMotorVoltage(). Чисто аддитивно: rawMotorVoltage нигде
    // ниже не читается, только копируется в m_lastRawMotorVoltage.
    std::vector<float> rawMotorVoltage(kNumSegments, 0.0f);
    // Диагностика (Phase 3, гипотеза "sigmoid-насыщение на мотонейронах") -
    // см. debugRawDorsalVentralState(). Абсолютные (не d-v) avgState() по
    // каждой позиции, чисто аддитивно рядом с rawMotorVoltage.
    std::vector<float> rawDorsalState(kNumSegments, 0.0f);
    std::vector<float> rawVentralState(kNumSegments, 0.0f);
    for (int pos = 1; pos <= 24; ++pos) {
        auto avgOutput = [&](const std::vector<connectome::NeuronId>& ids) {
            if (ids.empty()) return 0.0f;
            float s = 0.0f;
            for (connectome::NeuronId id : ids) s += net.muscle_output(id);
            return s / static_cast<float>(ids.size());
        };
        auto avgState = [&](const std::vector<connectome::NeuronId>& ids) {
            if (ids.empty()) return 0.0f;
            float s = 0.0f;
            for (connectome::NeuronId id : ids) s += net.state(id);
            return s / static_cast<float>(ids.size());
        };
        const float d = avgOutput(m_dorsalByPos[static_cast<std::size_t>(pos)]);
        const float v = avgOutput(m_ventralByPos[static_cast<std::size_t>(pos)]);
        curvature[static_cast<std::size_t>(pos - 1)] = (d - v) * bodyGain;

        const float dRaw = avgState(m_dorsalByPos[static_cast<std::size_t>(pos)]);
        const float vRaw = avgState(m_ventralByPos[static_cast<std::size_t>(pos)]);
        rawMotorVoltage[static_cast<std::size_t>(pos - 1)] = (dRaw - vRaw) * bodyGain;
        rawDorsalState[static_cast<std::size_t>(pos - 1)] = dRaw;
        rawVentralState[static_cast<std::size_t>(pos - 1)] = vRaw;
    }
    m_lastRawMotorVoltage = rawMotorVoltage;
    m_lastRawDorsalState = rawDorsalState;
    m_lastRawVentralState = rawVentralState;

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

    // Диагностический снимок ДО baseline-вычитания - см. debugRawCurvature().
    // Чисто аддитивно: ничего ниже не читает m_lastRawCurvature, только
    // копирует то же curvature, что через пару строк пойдёт в EMA.
    m_lastRawCurvature = curvature;

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

    // Дешёвая страховка (тот же принцип, что у applyProprioception's feedback
    // clamp) - см. tests/worm_jerkiness_diagnostic: раз в ~300-500с на агаре
    // сеть даёт редкий выброс - это НЕ off-by-one, а собственная хаотичная
    // динамика сырой сети (см. tests/worm_locomotion: "states climb toward a
    // large self-sustained equilibrium"). Поэлементный кламп на 2.0 реально
    // снижает пик (1.2-1.4 рад -> 0.37-0.53 рад на тех же сидах до/после,
    // подтверждено) и НЕ трогает шипнутую точку (16/16 здоровых, те же
    // цифры). ПОПЫТКА добавить агрегатный кламп по сумме |deviation|
    // (потолок 15, пропорциональное сжатие всего вектора при превышении) -
    // ИСПРОБОВАНА И ОТКАЧЕНА: заморозила агар полностью (freq=0.0000Гц на
    // всех 16 базах) - нормальная амплитуда агара с сегодняшним стеком (шире
    // motorBandwidthGain) сама регулярно подходит к сумме ~9, потолок 15
    // оказался слишком близко и душил САМУ волну, не только аварийный
    // выброс. Не убирает первопричину (сеть остаётся хаотичной) - обрезает
    // то немногое, что от неё реально доходит до тела, без попытки понять
    // общую "энергию" сигнала (та попытка и сломала агар).
    // ПЕР-ПОЗИЦИОННАЯ НОРМИРОВКА ПРИВОДА (см. Params::driveEqualizationGain).
    // Стоит ЗДЕСЬ - после обоих фильтров кривизны, то есть работает с той
    // осциллирующей величиной, которая реально пойдёт в тело, и до рулевого
    // канала ниже (руль - аддитивная команда поворота, её выравнивать нельзя,
    // она не часть волны).
    {
        const float eqGain = params.driveEqualizationGain.load();
        if (eqGain > 0.0f && !deviation.empty()) {
            if (m_driveEnvelopeEma.size() != deviation.size())
                m_driveEnvelopeEma.assign(deviation.size(), 0.0f);
            const float tau = std::max(1e-3f, params.driveEqualizationTau.load());
            const float alpha = std::clamp(dt / tau, 0.0f, 1.0f);
            // EMA обновляется по СЫРОЙ (до нормировки) амплитуде. Это делает
            // коррекцию прямой, а не обратной связью: если бы EMA считалась по
            // уже отнормированному значению, равновесием стало бы
            // ema_i = sqrt(mean*raw_i), то есть выравнивание получилось бы
            // корневым и зависящим от собственного прошлого. По сырой величине
            // множитель однозначно определяется тем, что выдаёт сеть.
            float sum = 0.0f;
            for (std::size_t i = 0; i < deviation.size(); ++i) {
                m_driveEnvelopeEma[i] += (std::fabs(deviation[i]) - m_driveEnvelopeEma[i]) * alpha;
                sum += m_driveEnvelopeEma[i];
            }
            const float mean = sum / static_cast<float>(deviation.size());
            // Пока EMA не прогрелась (первые ~tau секунд) средняя близка к нулю
            // и множители были бы шумом - нормировка не применяется вообще.
            constexpr float kWarmupFloor = 1e-4f;
            if (mean > kWarmupFloor) {
                const float maxRatio = std::max(1.0f, params.driveEqualizationMaxRatio.load());
                for (std::size_t i = 0; i < deviation.size(); ++i) {
                    const float ema = std::max(kWarmupFloor, m_driveEnvelopeEma[i]);
                    const float scale = std::clamp(std::pow(mean / ema, eqGain), 1.0f / maxRatio, maxRatio);
                    deviation[i] *= scale;
                }
            }
        }
    }

    // РУЛЕВОЙ КАНАЛ (см. Params::chemoSteeringGain за полным обоснованием).
    // Стоит ЗДЕСЬ - после пространственного вычитания среднего и после
    // временной EMA, но ДО аварийного клампа ниже: смысл канала в том, чтобы
    // не быть стёртым фильтрами, но страховка на выброс на него всё равно
    // распространяется.
    // ОМЕГА-ПОВОРОТ (см. updateLocomotionState). Смещение кривизны одного знака
    // на передние суставы - глубокий изгиб на одну сторону; резкую смену курса
    // из него делает та же физика трения, что и обычный ход. Стоит в той же
    // точке, что и рулевой канал ниже, и по той же причине: после обоих
    // фильтров, иначе пространственное вычитание среднего стёрло бы его
    // полностью (одинаковая добавка ко всем позициям вычитается в ноль).
    //
    // Единицы: в режиме мышцы-как-момента deviation - это ЦЕЛЕВОЙ УГОЛ, поэтому
    // смещение задаётся прямо в радианах. В кинематическом режиме
    // (muscleTargetTau<=0) deviation - скорость изменения угла, и установившийся
    // угол равен c/r, поэтому там нужен множитель bodyPoseDecayRate. Тот же
    // разбор единиц, что и у рулевого канала ниже.
    if (m_locomotion == Locomotion::Omega && !deviation.empty()) {
        const float unitScale =
            (params.muscleTargetTau.load() > 0.0f) ? 1.0f : params.bodyPoseDecayRate.load();
        const float bias = m_omegaSign * params.omegaBendBias.load() * unitScale;
        const int span = std::clamp(params.omegaBendSpan.load(), 1, static_cast<int>(deviation.size()));
        // ПОДАВЛЕНИЕ ВОЛНЫ НА ВРЕМЯ ОМЕГИ. Омега-поворот - УДЕРЖИВАЕМАЯ поза, а
        // не волна со смещением: голова достаёт до хвоста и тело какое-то время
        // стоит витком (Gray, Hill & Bargmann 2005). Пока колебание
        // продолжалось на полной амплитуде, смещение лишь подпирало потолок, и
        // чтобы получить заметный поворот приходилось поднимать сам потолок -
        // а двенадцать суставов по 1.2 рад это 14.4 рад суммарной кривизны,
        // больше двух полных витков. Прямое измерение показало, чем это
        // кончалось: жёсткое вращение тела до 13.1 рад/с в фазе омеги при
        // скорости суставов всего 1.5 рад/с, то есть тело закручивало само
        // себя. Это и был весь implausible-heading-whip в воде.
        //
        // Один виток - это 2*pi суммарной кривизны, то есть 6.28/12 = 0.52 рад
        // на сустав при span=12. Локомоторный потолок 0.55 рад это уже
        // позволяет: поднимать его не нужно, нужно убрать колебание, которое
        // мешает смещению дойти до своей величины.
        const float keep = std::clamp(1.0f - params.omegaWaveSuppress.load(), 0.0f, 1.0f);
        for (int i = 0; i < span; ++i) {
            float& d = deviation[static_cast<std::size_t>(i)];
            d = d * keep + bias;
        }
    }

    // РУЛЕВОЙ КАНАЛ (см. Params::chemoSteeringGain за полным обоснованием).
    // Подавлен вне прямого хода: во время реверса и омега-поворота животное не
    // подруливает по градиенту, оно как раз меняет направление целиком.
    const float steerGain = (m_locomotion == Locomotion::Forward) ? params.chemoSteeringGain.load() : 0.0f;
    if (steerGain != 0.0f && !deviation.empty()) {
        const auto& lx = m_body.points_x();
        const auto& ly = m_body.points_y();
        if (lx.size() >= 2) {
            // Направление "вперёд" - от хвоста к голове (points_[0] - голова и
            // всегда локальный ноль, см. rebuild_points). Берётся по крайним
            // точкам, а не по первому сегменту: при сильном изгибе передний
            // сегмент смотрит вбок и ось начала бы шуметь - тот же выбор, что
            // в Measurement::signedForwardBLps.
            const std::size_t last = lx.size() - 1;
            float fx = lx[0] - lx[last], fy = ly[0] - ly[last];
            const float flen = std::sqrt(fx * fx + fy * fy);
            if (flen > 1e-6f) {
                fx /= flen;
                fy /= flen;
                const float ch = std::cos(m_heading), sh = std::sin(m_heading);
                const float wfx = fx * ch - fy * sh, wfy = fx * sh + fy * ch; // вперёд в мире
                const glm::vec2 leftNormal(-wfy, wfx);
                const float off = params.chemoLateralOffset.load();
                const float scentLeft = sampleFood(m_position + leftNormal * off);
                const float scentRight = sampleFood(m_position - leftNormal * off);
                // Поперечная производная запаха. Знак, при котором это даёт
                // приближение к еде, определён ИЗМЕРЕНИЕМ, а не выводом:
                // связь "знак angles_ -> сторона поворота" проходит через
                // rebuild_points и мировой поворот, и выводить её на бумаге
                // ровно тот способ, которым в этом проекте уже один раз
                // получили ход задом наперёд (см. Params::
                // proprioceptiveAnterior). См. WORM_V5_SPATIAL_ENVELOPE_
                // DIAGNOSIS.md раздел 13 за таблицей замера обоих знаков.
                // ОТНОСИТЕЛЬНАЯ поперечная разность (закон Вебера-Фехнера).
                // Абсолютная разность непригодна: она пропорциональна силе
                // поля, а поле в этом проекте задаётся кистью и может отличаться
                // на порядки (измеренные профили: пик 0.2 у дефолтной кисти
                // против 442 у широкого пятна - разница в 2000 раз). При
                // абсолютной разности одно и то же chemoSteeringGain означало
                // бы на слабом поле почти ноль, а на сильном - мгновенный
                // упор в кламп deviation (+/-2). Деление на локальный уровень
                // делает величину безразмерной и от силы поля не зависящей.
                //
                // Это не просто удобство: хемотаксис C. elegans и есть
                // ответ на ОТНОСИТЕЛЬНОЕ изменение концентрации, а не на
                // абсолютное - стандартное вебер-фехнеровское поведение
                // сенсорной системы.
                //
                // ЗНАМЕНАТЕЛЬ - локальный уровень ПЛЮС полунасыщение сенсора,
                // а не плюс микроскопическая эпсилон-заглушка. Раньше здесь
                // стояло +1e-4, и это давало реальный сбой, найденный владельцем
                // проекта живьём ("при поедании еды червь неестественно
                // ускоряется вперёд"): червь выгрызает под собой дыру
                // (consumeFood), локальный уровень под точками выборки падает,
                // а отношение (L-R)/(mean+1e-4) при малом mean уходит к своей
                // структурной границе 2 - смещение упирается в кламп deviation
                // и передние суставы перекладывает до упора. Изолировано
                // измерением: при steer=0 максимум мгновенной скорости
                // центроида 0.321 BL/s при ЛЮБОЙ скорости поедания (0/6/20/40),
                // при steer=-2 и скорости поедания 40 - 1.771 BL/s, то есть
                // 27x от средней. Нужны БЫЛИ оба условия сразу.
                //
                // chemoSensorHalfSaturation - это по смыслу и есть концентрация,
                // ниже которой сенсор почти не отвечает. Относительному
                // градиенту, измеренному на сигнале ниже этого масштаба,
                // доверять нельзя, и руль обязан там затухать, а не расти. С
                // этим знаменателем |lateral| ограничен уровнем
                // 2*mean/(mean+K) - то есть у слабого запаха стремится к нулю,
                // а не к границе.
                const float scentMean = 0.5f * (scentLeft + scentRight);
                const float lateral =
                    (scentLeft - scentRight) / (scentMean + std::max(1e-6f, params.chemoSensorHalfSaturation.load()));
                // НОРМИРОВКА НА bodyPoseDecayRate. deviation - это скорость
                // изменения угла (angles_ += c*dt), а затухание позы делает из
                // интегратора утечку, так что установившееся смещение угла
                // равно c/r, где r = bodyPoseDecayRate. Без множителя r здесь
                // смысл chemoSteeringGain зависел бы от темпа: подъём
                // bodyPoseDecayRate 0.5 -> 3.0 (раздел 14 отчёта) ослабил бы
                // руль ровно в 6 раз молча. С множителем chemoSteeringGain
                // означает РАДИАНЫ смещения передних суставов на единицу
                // поперечной разности запаха - величина физическая и от темпа
                // не зависящая. Ровно то сцепление параметров, из-за которого
                // в этой сессии уже пришлось откатывать целый набор
                // "компенсаций" (раздел 7).
                const float poseDecay = params.bodyPoseDecayRate.load();
                // ФИЗИЧЕСКОЕ ОГРАНИЧЕНИЕ СМЕЩЕНИЯ. Вебер-нормировка не может
                // ограничить |lateral|: поедание создаёт РЕЗКУЮ ступеньку в
                // поле (одна точка выборки в остатке еды, другая в выеденной
                // яме), а для ступеньки относительный градиент максимален при
                // любом уровне концентрации - подъём знаменателя с 1e-4 до
                // полунасыщения сбил рывок лишь с 1.771 до 1.393 BL/s.
                //
                // Ограничение ставится не на сигнал, а на РЕЗУЛЬТАТ: смещение
                // имеет право отклонить передние суставы максимум на их
                // собственный предел, не больше. Установившийся угол от
                // смещения равен bias/r, значит предел - jointAngleClamp*r.
                // При r=3 и клампе 0.25 это 0.75 против прежних 12 (полный
                // kDeviationClamp), то есть в 16 раз. Прежняя величина
                // соответствовала установившемуся углу 4 рад при пределе
                // сустава 0.25 - физически бессмысленно.
                const float maxSteerBias = params.jointAngleClamp.load() * poseDecay;
                const float steerBias =
                    std::clamp(steerGain * lateral * poseDecay, -maxSteerBias, maxSteerBias);
                const int span = std::clamp(params.chemoSteeringSpan.load(), 1,
                                            static_cast<int>(deviation.size()));
                for (int i = 0; i < span; ++i)
                    deviation[static_cast<std::size_t>(i)] += steerBias;
            }
        }
    }

    // МАСШТАБИРОВАНИЕ КЛАМПА (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 17).
    // Значение 2.0 было откалибровано при bodyPoseDecayRate=0.5; смысл клампа -
    // ограничить редкий выброс в единицах УГЛА, а установившийся угол равен
    // deviation/r (см. Params::bodyGain за выводом). Значит порог, чтобы
    // сохранить свой смысл, обязан масштабироваться вместе с r.
    //
    // Почему это пришлось исправлять: подъём ритма (раздел 14, r 0.5 -> 3.0 и
    // bodyGain 200 -> 1200) я сделал, НЕ пересмотрев этот порог, и он из
    // "страховки, которая не должна даже срабатывать" превратился в постоянно
    // активный ограничитель - замер на отгруженных дефолтах: 6.56% всех
    // отсчётов суставов прижаты к +/-2.0, у 88.8% шагов есть хотя бы один
    // срезанный сустав, sum|deviation| на шаг в среднем 17.8 против ~9,
    // которые комментарий ниже называет нормой для старого привода.
    //
    // 4.0f * r даёт ровно 2.0 при r=0.5 (побитово прежнее поведение на прежнем
    // ритме) и 12.0 при r=3.0.
    const float kDeviationClamp = 4.0f * params.bodyPoseDecayRate.load();
    for (float& d : deviation) d = std::clamp(d, -kDeviationClamp, kDeviationClamp);

    m_body.set_drag(params.dragTangent.load(), params.dragNormal.load());
    m_body.set_pose_decay_rate(params.bodyPoseDecayRate.load());
    // V3 (см. WORM_V3_DESIGN.md разделы 2-3): пространственная изгибная
    // жёсткость между соседними суставами - живёт целиком в кинематике
    // angles_ внутри WormBody::step(), не в solve_propulsion.
    m_body.set_bend_stiffness(params.bodyBendStiffness.load());
    // V2 (см. WORM_V2_DESIGN.md разделы 3-4): три мгновенные adhesion-формулы
    // + кламп на |u_k| заменены памятью проседания (penetration_depth_) и
    // ограничением угловой скорости ориентации сустава - см. body.hpp/.cpp.
    m_body.set_drag_settle(params.dragSettleTau.load(), params.dragSettleGain.load());
    constexpr float kTwoPi = 6.28318530717958647692f;
    m_body.set_max_frame_angular_rate(kTwoPi * params.bodyFrameRateLimitHz.load());
    // Связь среды с темпом изгиба - см. Params::mediumBendCouplingKappa за
    // выводом множителя и за тем, почему нормировка идёт на трение агара.
    {
        const float kappa = params.mediumBendCouplingKappa.load();
        float bendTimeScale = 1.0f;
        if (kappa > 0.0f) {
            const float refDrag = std::max(0.0f, params.mediumBendReferenceDrag.load());
            const float drag = std::max(0.0f, params.dragNormal.load());
            bendTimeScale = (1.0f + kappa * refDrag) / (1.0f + kappa * drag);
        }
        m_body.set_bend_time_scale(bendTimeScale);
    }
    // WORM_V5_JOINT_CLAMP_RESULTS.md: joint-angle hard clamp, formerly a
    // hardcoded local constant inside WormBody::step - see Params::
    // jointAngleClamp for full history. Default 0.25 == prior hardcode.
    // ЗАВИСИМОСТЬ АМПЛИТУДЫ ОТ СРЕДЫ (см. Params::mediumAmplitudeWaterRatio) -
    // масштабируется КЛАМП, а не команда.
    //
    // Почему именно кламп: попытка масштабировать команду измерена и провалена
    // (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 24) - при снижении привода
    // в воде амплитуда не убывает плавно, а обваливается через бифуркацию до
    // 0.03-0.09 рад с потерей survival и выбросами скорости до 26x. Это та же
    // бистабильность, что измерена в разделе 22: у петли нет устойчивой точки на
    // промежуточной амплитуде, амплитуду задаёт ПОТОЛОК ("пик садится ровно на
    // новый потолок"). Значит двигать надо потолок.
    {
        float clamp = params.jointAngleClamp.load();
        const float waterRatio = params.mediumAmplitudeWaterRatio.load();
        if (waterRatio > 0.0f && waterRatio != 1.0f) {
            const float lowDrag = std::max(1e-3f, params.mediumAmplitudeLowDrag.load());
            const float highDrag = std::max(lowDrag * 1.0001f, params.mediumBendReferenceDrag.load());
            const float drag = std::max(1e-3f, params.dragNormal.load());
            // Логарифмическая интерполяция между двумя якорями, зажатая по
            // концам: на lowDrag даёт waterRatio, на highDrag ровно 1.0.
            const float t = std::clamp(std::log(drag / lowDrag) / std::log(highDrag / lowDrag), 0.0f, 1.0f);
            clamp *= waterRatio + (1.0f - waterRatio) * t;
        }
        m_body.set_joint_angle_clamp(clamp);
    }
    // ЗАВИСИМОСТЬ АМПЛИТУДЫ ОТ СРЕДЫ ЧЕРЕЗ ЗАТУХАНИЕ, а не через потолок -
    // см. Params::mediumAmplitudeViaDecay. Устраняет ПРИЧИНУ релейного режима
    // (раздел 27 отчёта): равновесие сустава равно привод/затухание, и когда
    // потолок опущен НИЖЕ равновесия, сустав стоит у потолка постоянно, а при
    // смене знака привода проходит весь диапазон за считанные шаги - это и
    // давало скачки тяги. Поднятие затухания опускает само равновесие, так что
    // сустав к потолку не подходит вовсе.
    //
    // Побочно поднимает частоту в воде (затухание задаёт частоту среза, см.
    // раздел 14), что биологически верно: в воде выше частота и меньше
    // амплитуда (Fang-Yen et al. 2010).
    if (params.mediumAmplitudeViaDecay.load() != 0) {
        const float waterRatio = params.mediumAmplitudeWaterRatio.load();
        if (waterRatio > 0.0f && waterRatio != 1.0f) {
            const float lowDrag = std::max(1e-3f, params.mediumAmplitudeLowDrag.load());
            const float highDrag = std::max(lowDrag * 1.0001f, params.mediumBendReferenceDrag.load());
            const float drag = std::max(1e-3f, params.dragNormal.load());
            const float t = std::clamp(std::log(drag / lowDrag) / std::log(highDrag / lowDrag), 0.0f, 1.0f);
            const float scale = waterRatio + (1.0f - waterRatio) * t;
            // Амплитуда = привод/затухание, значит для множителя амплитуды
            // scale затухание делится на него.
            m_body.set_pose_decay_rate(params.bodyPoseDecayRate.load() / std::max(1e-3f, scale));
            // Потолок при этом НЕ опускается - он остаётся общим пределом
            // анатомии, а не инструментом задания амплитуды.
            m_body.set_joint_angle_clamp(params.jointAngleClamp.load());
        }
    }
    // ОМЕГА-ПОВОРОТ ГЛУБЖЕ ОБЫЧНОЙ ВОЛНЫ. Кламп сустава задан амплитудой
    // НОРМАЛЬНОЙ локомоции (0.55 рад - центр биологического диапазона пика
    // изгиба при ползании). Омега-поворот - другой режим: у Gray, Hill &
    // Bargmann 2005 голова достаёт до хвоста, то есть тело сворачивается
    // заметно круче, чем при ходе. Пока кламп остаётся локомоторным, смещение
    // кривизны просто упирается в потолок и средний изгиб почти не сдвигается -
    // измерено: поворот выходил 23-54 градуса вместо характерных для омеги
    // 150+.
    //
    // Поэтому на время фазы Omega потолок и предел скорости сустава
    // поднимаются: это разные режимы одного тела, а не подкрутка ради числа.
    // Верхняя граница - kJointHardSafetyLimit (1.2 рад) в body.cpp, тот же
    // аварийный предел, что и всегда.
    // СВЯЗЬ ИЗГИБА СО СРЕДОЙ (Fang-Yen, Wyart, Cockburn, Wasserman, Samuel
    // et al. 2010, PNAS 107(47):20323-20328, "Biomechanical analysis of gait
    // adaptation in the nematode C. elegans").
    //
    // До этого динамика изгиба НЕ ЗНАЛА ПРО СРЕДУ вообще: сустав шёл к цели с
    // постоянной времени 0.1с и на агаре, и в воде. Разница сред задавалась
    // снаружи, масштабированием потолка амплитуды - подобранным числом.
    // Отсюда отношение частот вода/агар 1.2 против биологических ~4.
    //
    // У Fang-Yen частота задаётся балансом мышечного момента против СУММАРНОГО
    // демпфирования: внутреннего (вязкоупругость самого тела) плюс внешнего
    // (среда). Они прямо заключают, что при низкой внешней вязкости доминирует
    // внутреннее - именно поэтому частота не уходит в бесконечность в воде.
    //
    //   tau(c_n) = tau_внутр + kappa * c_n
    //
    // Параметризуется ДОЛЕЙ ВНУТРЕННЕГО при трении агара (mediumBendInternal
    // Fraction = phi), с якорем на агаре:
    //
    //   tau(c_n) = tau_агар * [phi + (1-phi) * c_n / c_агар]
    //
    // При c_n == c_агар это тождественно tau_агар, то есть агар остаётся
    // побитово нетронутым, а меняется только вода. phi=1 - механизм выключен
    // тождественно.
    //
    // ПРЕДЕЛ СКОРОСТИ СУСТАВА МАСШТАБИРУЕТСЯ ТЕМ ЖЕ МНОЖИТЕЛЕМ, и это не
    // отдельное решение: предел есть отношение максимального мышечного момента
    // к демпфированию, r = T_max/b. Мышца одна и та же в обеих средах, значит
    // при меньшем демпфировании тот же момент даёт большую скорость. Обе
    // величины двигаются согласованно, поэтому f*A = r/4 сохраняется, а
    // меняется РАСПРЕДЕЛЕНИЕ между f и A - ровно то, что описывает Fang-Yen:
    // в воде частота выше в разы, амплитуда меняется умеренно.
    float mediumTauScale = 1.0f;
    {
        const float phi = std::clamp(params.mediumBendInternalFraction.load(), 0.0f, 1.0f);
        if (phi < 1.0f) {
            const float refDrag = std::max(1e-3f, params.mediumBendReferenceDrag.load());
            const float drag = std::max(1e-3f, params.dragNormal.load());
            mediumTauScale = std::max(1e-3f, phi + (1.0f - phi) * (drag / refDrag));
        }
    }
    const float mediumRateScale = 1.0f / mediumTauScale;
    if (m_locomotion == Locomotion::Omega) {
        m_body.set_joint_angle_clamp(std::min(1.2f, params.jointAngleClamp.load()
                                                        * params.omegaClampScale.load()));
        m_body.set_joint_angle_rate_limit(params.jointAngleRateLimit.load()
                                          * params.omegaRateLimitScale.load() * mediumRateScale);
    } else {
        m_body.set_joint_angle_rate_limit(params.jointAngleRateLimit.load() * mediumRateScale);
    }
    m_body.set_muscle_target_tau(params.muscleTargetTau.load() * mediumTauScale);
    // WORM_V5_SOFT_JOINT_RESISTANCE_RESULTS.md: additive, off-by-default
    // (0.0) alternatives to the hard clamp just above - see Params::
    // jointSoftSaturation/jointStiffeningGain for full derivation.
    m_body.set_joint_soft_saturation(params.jointSoftSaturation.load());
    m_body.set_joint_stiffening_gain(params.jointStiffeningGain.load());
    m_body.step(deviation, dt);

    // Проприоцепция - готовит вход для СЛЕДУЮЩЕГО net.step() из РЕАЛЬНОГО
    // (уже прошедшего физику и кламп) изгиба, который тело только что
    // приняло - см. applyProprioception. Реальный механизм, которым
    // C. elegans превращает связность коннектома в бегущую волну (Boyle,
    // Berri & Cohen 2012).
    //
    // Задержка - см. Params::proprioceptiveDelaySeconds. delaySeconds<=0
    // (дефолт) => history вообще не трогается, читаем m_body.angles()
    // напрямую - побитово прежнее поведение, ноль лишней работы.
    const float delaySeconds = params.proprioceptiveDelaySeconds.load();
    const std::vector<float>* rawAnglesPtr;
    if (delaySeconds > 1e-6f) {
        constexpr std::size_t kMaxHistorySteps = 400;  // ~20с при dt=0.05 - щедрый потолок
        m_angleHistory.push_back(m_body.angles());
        while (m_angleHistory.size() > kMaxHistorySteps) m_angleHistory.pop_front();
        const int delaySteps = std::clamp(static_cast<int>(std::lround(delaySeconds / dt)), 0,
                                           static_cast<int>(m_angleHistory.size()) - 1);
        rawAnglesPtr = &m_angleHistory[m_angleHistory.size() - 1 - static_cast<std::size_t>(delaySteps)];
    } else {
        rawAnglesPtr = &m_body.angles();
    }

    // Низкочастотный фильтр НА ВХОД проприоцепции - см. Params::
    // proprioceptiveFilterTau за полным обоснованием (почему это ВХОД, а не
    // ВЫХОД, как kDeviationClamp выше). filterTau<=0 (дефолт) => фильтр не
    // считается вообще, applyProprioception получает тот же raw/delayed
    // указатель, что и раньше - побитово прежнее поведение.
    const float filterTau = params.proprioceptiveFilterTau.load();
    if (filterTau > 1e-6f) {
        const std::vector<float>& rawAngles = *rawAnglesPtr;
        if (m_filteredAngles.size() != rawAngles.size()) m_filteredAngles.assign(rawAngles.size(), 0.0f);
        const float filterAlpha = std::clamp(dt / filterTau, 0.0f, 1.0f);
        for (std::size_t i = 0; i < rawAngles.size(); ++i)
            m_filteredAngles[i] += (rawAngles[i] - m_filteredAngles[i]) * filterAlpha;
        applyProprioception(m_filteredAngles);
    } else {
        applyProprioception(*rawAnglesPtr);
    }
    applyMechanosensation(); // DVA <- та же уже случившаяся физика, что и выше
    applyDopamineDrive(); // ADE/PDE + обновление m_dopamineTone на СЛЕДУЮЩИЙ шаг

    // Перемещение/поворот тела - РЕШЕНИЕ баланса сил анизотропного трения на
    // форме, которую только что породила сеть (см. connectome::WormBody), не
    // отдельная эвристика "знак/модуль deviation -> курс/скорость". Локальная
    // скорость тела поворачивается в мировые координаты текущим heading.
    // Собственное вращение тела - вокруг points_[0], потому что именно
    // относительно этой точки solve_propulsion параметризовал баланс сил
    // (r_k отсчитываются от неё), то есть (Vx,Vy,w) - это скорость points_[0] и
    // угловая скорость вокруг неё. Менять точку здесь было бы неверно.
    // Урезается тем же правилом, что и поворот у стены: стена не даёт телу
    // повернуться ГЛУБЖЕ в себя (см. rotateLimited).
    rotateLimited(m_body.angular_velocity() * dt, glm::vec2(0.0f));
    const float c = std::cos(m_heading), s = std::sin(m_heading);
    const float lvx = m_body.local_velocity_x(), lvy = m_body.local_velocity_y();
    float vwx = (lvx * c - lvy * s);
    float vwy = (lvx * s + lvy * c);
    // ГАШЕНИЕ СОСТАВЛЯЮЩЕЙ СКОРОСТИ В СТЕНУ ДО ИНТЕГРИРОВАНИЯ ПОЗИЦИИ - ровно
    // то, что комментарий в containBody ниже называл правильным решением и чего
    // там не было. Стена перестаёт быть источником движения: она отнимает
    // составляющую скорости, направленную наружу, а не двигает тело после
    // факта. Позиционная коррекция в containBody остаётся, но теперь она
    // страховка от остаточного проникновения (изгиб тела у стены), а не
    // основной механизм - её величина падает на порядок.
    {
        float minX, maxX, minY, maxY;
        bodyWorldBBox(m_position, m_heading, minX, maxX, minY, maxY);
        if (maxX >= m_boundsMax.x && vwx > 0.0f) vwx = 0.0f;
        if (minX <= m_boundsMin.x && vwx < 0.0f) vwx = 0.0f;
        if (maxY >= m_boundsMax.y && vwy > 0.0f) vwy = 0.0f;
        if (minY <= m_boundsMin.y && vwy < 0.0f) vwy = 0.0f;
    }
    m_position.x += vwx * dt;
    m_position.y += vwy * dt;
    containBody(dt);

    m_lastCurvature = std::move(deviation);
}

// Раньше о границу поля "спотыкалась" только точка отсчёта тела (points_[0]) -
// остальная его длина могла вылезать за край, пока она сама ещё в пределах.
// Настоящая коллизия - на всю длину: считаем мировой bbox ВСЕХ точек тела и,
// если он вылезает за границу, сдвигаем всё тело обратно целиком. При касании
// стены дополнительно отражаем курс (простой, но честный отскок) - иначе
// червь просто вжимался бы в стену снова и снова вместо того, чтобы от неё
// отвернуть.
// bbox тела в мировых координатах для ДАННЫХ якоря и курса. Форма (points_ в
// локальных координатах) при повороте не меняется, меняется только ориентация,
// поэтому bbox нужно пересчитывать заново после каждого изменения heading,
// иначе сдвиг позиции окажется рассчитан для уже неактуальной ориентации и тело
// всё равно вылезет за границу.
void WormSim::bodyWorldBBox(glm::vec2 anchor, float heading, float& minX, float& maxX, float& minY,
                            float& maxY) const {
    const int n = m_body.num_segments();
    const auto& lx = m_body.points_x();
    const auto& ly = m_body.points_y();
    const float c = std::cos(heading), s = std::sin(heading);
    minX = 1e30f; maxX = -1e30f; minY = 1e30f; maxY = -1e30f;
    for (int i = 0; i <= n; ++i) {
        const float lxp = lx[static_cast<std::size_t>(i)], lyp = ly[static_cast<std::size_t>(i)];
        const float wx = anchor.x + lxp * c - lyp * s;
        const float wy = anchor.y + lxp * s + lyp * c;
        minX = std::min(minX, wx);
        maxX = std::max(maxX, wx);
        minY = std::min(minY, wy);
        maxY = std::max(maxY, wy);
    }
}

// Суммарная глубина выхода bbox тела за границы поля для заданных якоря и
// курса. Ноль - тело целиком внутри. Мера, а не булев признак: нужна именно
// величина, чтобы гасить ту часть поворота, которая её УВЕЛИЧИВАЕТ, и не
// трогать ту, которая уменьшает (иначе тело у стены замирает вместо того, чтобы
// от неё отвернуть).
float WormSim::wallPenetration(glm::vec2 anchor, float heading) const {
    float minX, maxX, minY, maxY;
    bodyWorldBBox(anchor, heading, minX, maxX, minY, maxY);
    return std::max(0.0f, m_boundsMin.x - minX) + std::max(0.0f, maxX - m_boundsMax.x)
         + std::max(0.0f, m_boundsMin.y - minY) + std::max(0.0f, maxY - m_boundsMax.y);
}

// Поворот курса на dTheta вокруг pivotLocal (точка в ЛОКАЛЬНЫХ координатах
// тела), урезанный так, чтобы не увеличивать выход за границу. Половинным
// делением: поворот от стены проходит целиком (пенетрация падает), поворот в
// стену ужимается до безопасной доли.
//
// Зачем вообще: позиционная коррекция в containBody сдвигает тело на всю
// глубину проникновения за один шаг, и у стены каждый поворот сам это
// проникновение создаёт - отсюда боковой рывок. Гашение скорости в стену
// (см. step) убрало поступательную часть, это убирает поворотную. Обе правки -
// одно и то же: стена отнимает движение, а не создаёт его.
void WormSim::rotateLimited(float dTheta, glm::vec2 pivotLocal) {
    if (dTheta == 0.0f) return;
    auto anchorFor = [&](float heading) {
        const float c = std::cos(heading), s = std::sin(heading);
        const float px = pivotLocal.x, py = pivotLocal.y;
        const float c0 = std::cos(m_heading), s0 = std::sin(m_heading);
        const glm::vec2 pivotWorld(m_position.x + px * c0 - py * s0, m_position.y + px * s0 + py * c0);
        return glm::vec2(pivotWorld.x - (px * c - py * s), pivotWorld.y - (px * s + py * c));
    };
    const float before = wallPenetration(m_position, m_heading);
    float scale = 1.0f;
    for (int i = 0; i < 5; ++i) {
        const float h = m_heading + dTheta * scale;
        if (wallPenetration(anchorFor(h), h) <= before + 1e-4f) break;
        scale *= 0.5f;
        if (i == 4) scale = 0.0f;
    }
    if (scale == 0.0f) return;
    const float h = m_heading + dTheta * scale;
    m_position = anchorFor(h);
    m_heading = h;
}

void WormSim::containBody(float dt) {
    const int n = m_body.num_segments();
    const auto& lx = m_body.points_x();
    const auto& ly = m_body.points_y();

    float minX, maxX, minY, maxY;
    bodyWorldBBox(m_position, m_heading, minX, maxX, minY, maxY);

    const bool hitX = minX < m_boundsMin.x || maxX > m_boundsMax.x;
    const bool hitY = minY < m_boundsMin.y || maxY > m_boundsMax.y;
    if (hitX || hitY) {
        constexpr float kPi = 3.14159265f;
        // ОГРАНИЧЕНИЕ СКОРОСТИ ПОВОРОТА У СТЕНЫ (WORM_V5_SPATIAL_ENVELOPE_
        // DIAGNOSIS.md раздел 18). Раньше здесь стояло мгновенное
        // m_heading = kPi - m_heading, то есть весь червь за ОДИН шаг
        // разворачивался вокруг своей точки отсчёта - живьём это выглядело как
        // отскок/телепорт, а не как разворот животного. Форма тела при этом не
        // меняется, поэтому визуально всё тело прыгало целиком.
        //
        // Теперь курс поворачивается К отражённому значению с ограничением
        // скорости. Отдельного состояния не нужно: пока тело у стены, условие
        // hit сохраняется, цель пересчитывается каждый шаг и поворот
        // продолжается - процесс самозавершающийся, как только bbox перестаёт
        // выходить за границу.
        //
        // kWallTurnRate=3.0 рад/с: полный разворот на pi занимает ~1.05с
        // (~21 шаг при dt=0.05) - того же порядка, что и собственный период
        // изгиба (~8с), то есть читается как разворот, а не как удар. Значение
        // выбрано по этому соображению, не откалибровано против чего-либо.
        constexpr float kWallTurnRate = 3.0f;
        const float maxTurn = kWallTurnRate * std::max(1e-6f, dt);
        float desired = m_heading;
        if (hitX) desired = kPi - desired;
        if (hitY) desired = -desired;
        // Кратчайшая разность углов - без неё поворот мог пойти длинной дугой.
        float diff = desired - m_heading;
        while (diff > kPi) diff -= 2.0f * kPi;
        while (diff < -kPi) diff += 2.0f * kPi;
        const float turn = std::clamp(diff, -maxTurn, maxTurn);

        // ПОВОРОТ ВОКРУГ ЦЕНТРОИДА, А НЕ ВОКРУГ ГОЛОВЫ. m_position - это
        // points_[0], то есть голова; поворот курса вокруг неё разносит всё
        // остальное тело вбок. Центроид сидит на ~0.42 длины тела позади головы,
        // поэтому при kWallTurnRate=3 рад/с он ехал вбок со скоростью
        // 0.42*3 = 1.26 длины тела в секунду - ИЗМЕРЕНО ровно 1.259 (диагностика
        // burst, столбцы lat/dHead: dHead=-3.14 рад/с при lat=+1.259 и fwd=+0.04,
        // при неизменной скорости суставов). Это и был весь "выброс скорости":
        // на арене, до стен которой червь не доходит, выбросов нет вообще ни при
        // одном пределе скорости сустава вплоть до 1.5 рад/с.
        //
        // Свободное тело под моментом и без результирующей силы вращается вокруг
        // центра сопротивления, для однородного тонкого тела - примерно вокруг
        // геометрического центроида. Поворот вокруг головы порождал
        // поступательное движение без всякой силы, то есть был источником
        // энергии. Вокруг центроида центроид остаётся на месте по построению.
        //
        // Поворот дополнительно урезается так, чтобы не УВЕЛИЧИВАТЬ выход за
        // границу (см. rotateLimited): иначе тело у стены поворотом само себе
        // создаёт проникновение, а коррекция ниже мгновенно сдвигает его на всю
        // эту глубину - остаток того же бокового рывка, уже вдвое меньший, но
        // измеримый (0.5 длины тела в секунду против 1.26 до поворота вокруг
        // центроида; на арене без достижимых стен - ноль).
        float clx = 0.0f, cly = 0.0f;
        for (int i = 0; i <= n; ++i) {
            clx += lx[static_cast<std::size_t>(i)];
            cly += ly[static_cast<std::size_t>(i)];
        }
        clx /= static_cast<float>(n + 1);
        cly /= static_cast<float>(n + 1);
        rotateLimited(turn, glm::vec2(clx, cly));
        bodyWorldBBox(m_position, m_heading, minX, maxX, minY, maxY); // курс сменился - bbox тоже
    }

    // ОГРАНИЧЕНИЕ СКОРОСТИ ПОЗИЦИОННОЙ КОРРЕКЦИИ (WORM_V5_SPATIAL_ENVELOPE_
    // DIAGNOSIS.md раздел 19). Раньше тело возвращалось в границы одним
    // мгновенным сдвигом на всю глубину проникновения. Это и есть "граница
    // отталкивает червя": коррекция считается по bbox ВСЕГО тела, поэтому у
    // стены каждый взмах изгиба сам создаёт проникновение, и червь получает
    // толчок величиной со взмах - причём за один кадр, то есть на скорости,
    // которую он сам развить не может.
    //
    // Предел взят от собственной поступательной скорости тела: коррекция за шаг
    // не быстрее kCorrectionSpeedFactor раз этой скорости. То есть стена может
    // остановить червя и мягко вытолкнуть, но не может двигать его быстрее, чем
    // он плавает. Кратковременное проникновение при этом допускается - визуально
    // это несравнимо лучше рывка, а bbox-запас у арены большой.
    //
    // Множитель 2.0 - не калибровка: он лишь даёт стене возможность быть
    // немного "сильнее" червя, чтобы тот не мог продавить границу насовсем.
    // ПОПЫТКА И ОТКАТ (WORM_V5_SPATIAL_ENVELOPE_DIAGNOSIS.md раздел 19.3):
    // здесь стояло ограничение скорости позиционной коррекции, чтобы стена не
    // могла двигать червя быстрее, чем он плавает. Идея адресовала настоящую
    // жалобу ("граница отталкивает червя"), но ЛОМАЛА инвариант: коррекция
    // перестала гарантировать вложенность, и Test_worm_locomotion честно упал с
    // "some point of the body left the bounds". Инвариант важнее гладкости, тем
    // более что ограничитель поворота курса выше (мгновенный разворот всего
    // тела) убирал основную часть визуального рывка.
    //
    // Остаток проблемы РЕАЛЕН и не решён: коррекция считается по bbox ВСЕГО
    // тела, поэтому у стены каждый взмах изгиба сам создаёт проникновение, и
    // тело получает сдвиг величиной со взмах. Правильное решение - гасить
    // составляющую СКОРОСТИ в стену до интегрирования позиции, а не двигать
    // тело после факта; это заметно более инвазивная правка, и делать её
    // наспех, уже сломав один инвариант, неразумно.
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
