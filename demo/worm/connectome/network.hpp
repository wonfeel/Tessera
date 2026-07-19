#pragma once

#include <cassert>
#include <vector>

#include "csr_matrix.hpp"
#include "types.hpp"

namespace connectome {

// Редуцированная (не спайковая) модель нейронной сети на графе коннектома,
// в духе Wicks/Roehrig/Rankin 1996 и Kunert et al. 2014 (модель, на которой
// строится "простой" режим c302 из проекта OpenWorm):
//
//   C_i * dV_i/dt = -leak_i * (V_i - rest_i)
//                   + sum_j W_chem[i,j] * sigmoid(V_j)      (химические синапсы)
//                   + sum_j W_gap[i,j] * (V_j - V_i)         (электрические, gap junction)
//                   + external_i                              (внешний вход - см. ниже, кто его получает)
//
// Каждый шаг — это две разреженные matvec-операции (CsrMatrix) плюс
// поэлементная нелинейность. Тип узла определяет, что происходит с
// результатом (см. реализацию в network.cpp: единственная реальная развилка
// в интеграторе - Input и Output, остальные три типа в нём НЕотличимы):
//   Input             -> состояние равно внешнему входу напрямую (без динамики)
//   InputProcessing    -> полная динамика + внешний вход в drive-член
//   Processing           -> полная динамика + внешний вход в drive-член -
//                           отличие от InputProcessing только по смыслу/имени
//                           (интернейрон, а не сенсор), не по коду; WormSim
//                           умышленно пишет set_input и в Processing-нейроны
//                           тоже (см. applyIntrinsicNoise)
//   ProcessingOutput     -> то же самое, что Processing (см. выше) - тоже
//                           только смысловое отличие ("командный"/выходной
//                           интернейрон), состояние читается как выход
//   Output               -> утечка и внешний вход принудительно 0, но НЕ
//                           "без утечки/динамики" в общем: те же gap junction
//                           и chem-current идут в тот же экспоненциальный
//                           интегратор, что и у остальных типов. Это
//                           architectural leak=0 ЕСТЬ причина самой медленной
//                           коллективной моды сети (~755с, найдено анализом
//                           собственных чисел - tests/worm_network_
//                           eigenmodes) - но убирать его оказалось НЕ
//                           исправлением, см. "TRIED CHANGING, REVERTED" в
//                           network.cpp у Network::step за полной историей.
class Network {
public:
    Network(std::vector<NeuronType> types, std::vector<NeuronParams> params,
            CsrMatrix chemical, CsrMatrix gap);

    // Внешний вход держится (латчится) до следующего вызова set_input —
    // так вызывающий код может обновлять сенсоры реже, чем тикает сеть.
    void set_input(NeuronId id, float value);

    // Один шаг интегрирования методом Эйлера с шагом dt.
    void step(float dt);

    float state(NeuronId id) const { return state_[id]; }
    NeuronType type(NeuronId id) const { return types_[id]; }
    NeuronId size() const { return static_cast<NeuronId>(types_.size()); }
    const CsrMatrix& chemical() const { return chemical_; }
    const CsrMatrix& gap() const { return gap_; }

    // Параметры сигмоиды активации химического синапса: a = sigmoid((V-theta)/slope)
    void set_activation_shape(float theta, float slope) {
        activation_theta_ = theta;
        activation_slope_ = slope;
    }

    // Глобальные множители поверх весов из данных коннектома и утечки нейрона.
    // Сырой коннектом даёт связность и число синапсов, но не токовые
    // коэффициенты усиления/утечку -- это единственная точка, где их можно
    // подстроить в рантайме, не перестраивая CsrMatrix заново.
    void set_gains(float chem_gain, float gap_gain, float leak_scale) {
        chem_gain_ = chem_gain;
        gap_gain_ = gap_gain;
        leak_scale_ = leak_scale;
    }

    // Калибровка по КЛАССАМ нейронов, а не по отдельным нейронам (для 401
    // нейрона это слишком много свободных параметров - см. заключение в
    // tests/worm_locomotion: "real per-neuron-class ... calibration" как один
    // из двух оставшихся честных путей к направленному хемотаксису).
    // load_connectome сейчас даёт всем нейронам одинаковые NeuronParams
    // (leak=1, rest=0, capacitance=1) независимо от типа - множители здесь
    // применяются поверх этого дефолта, отдельно для каждого NeuronType.
    void scale_type_params(NeuronType type, float leak_scale, float capacitance_scale) {
        for (std::size_t i = 0; i < params_.size(); ++i) {
            if (types_[i] != type) continue;
            params_[i].leak *= leak_scale;
            params_[i].capacitance *= capacitance_scale;
        }
    }

    // Калибровка по ЗНАКУ синапса, не по классу нейрона - возбуждающие
    // (положительный вес - холинергические, Wang et al. 2024 типирование,
    // см. data/README.md) и тормозные (отрицательный - GABA-эргические)
    // химические синапсы отдельно, плюс gap junction отдельно. Сырые веса
    // коннектома - это число синаптических контактов (Cook et al. 2019), не
    // калиброванная проводимость (см. data/README.md: "все веса - сырое
    // число синаптических контактов"); отношение между возбуждающим и
    // тормозным постсинаптическим током НА ОДИН контакт - это то, что сама
    // методика EM-реконструкции в принципе не измеряет, поэтому это честный
    // недостающий параметр (3 числа), а не подгонка "под что угодно" - в
    // отличие от per-(pre,post)-type матрицы (25 комбинаций), которая была
    // бы неотличима от переподгонки на конкретную метрику. Множители - НА
    // ВЕРХ уже применённого chem_gain_/gap_gain_ (set_gains) - это разница
    // МЕЖДУ классами весов, не общий масштаб (тот уже есть отдельно).
    void scale_synapse_sign(float chem_excitatory_scale, float chem_inhibitory_scale, float gap_scale) {
        for (float& w : chemical_.weights_mutable()) w *= (w >= 0.0f) ? chem_excitatory_scale : chem_inhibitory_scale;
        for (float& w : gap_.weights_mutable()) w *= gap_scale;
    }

    // Активный (регенеративный) ток - минимальная двухпеременная редукция
    // кальциевого plateau-механизма (UNC-2/CaV2, Gao, Guan, Fouad et al. 2018,
    // eLife 7:e29915 "Excitatory motor neurons are local oscillators for
    // backward locomotion" - показано напрямую электрофизиологией и
    // абляцией, что мотонейроны сами являются автономными осцилляторами, не
    // просто реле сети). Применяется ТОЛЬКО к явно заданному списку нейронов
    // (см. set_active_current_targets - WormSim.cpp заводит туда B-класс
    // DB/VB, Fouad et al. 2018, eLife 7:e29913, задают ритм именно переднего
    // хода). В отличие от chem/gap (зависят от СОСЕДЕЙ) и от leak/scale-
    // множителей выше (масштабируют существующую линейную систему), это
    // САМОссылочный член - зависит от V_i того же нейрона - то есть
    // настоящая положительная обратная связь, способная на устойчивый
    // предельный цикл в целом диапазоне параметров, а не только на "лезвии
    // бритвы" одной точки (см. развёрнутый комментарий у Network::step).
    // w_i - вторая (медленная) переменная, "открытость" канала: релаксирует
    // к 1-activation с постоянной времени active_tau_w_, то есть закрывается
    // при деполяризации и снова открывается в покое - стандартная форма
    // activation+recovery для этого класса биофизики (Morris-Lecar 1981;
    // общая форма - Izhikevich, "Dynamical Systems in Neuroscience", 2007).
    // gain=0 (дефолт) - ток тождественно равен нулю для КАЖДОГО нейрона на
    // КАЖДОМ шаге, вне зависимости от target-списка - см. WormSim.cpp за
    // статусом калибровки (не откалибровано, выключено).
    void set_active_current_targets(std::vector<NeuronId> ids) { active_ids_ = std::move(ids); }
    void set_active_current(float gain, float tau_w) {
        active_gain_ = gain;
        active_tau_w_ = std::max(1e-3f, tau_w);
    }

    // Нейропептидная сигнализация (PDF-1/PDFR-1 - Ripoll-Sánchez, Watteyne et
    // al. 2023, Neuron 111:3570-3589, "The neuropeptidergic connectome of
    // C. elegans" - предсказанная по ко-экспрессии GPCR/лиганда связность,
    // mid-range модель). Структурно НЕ то же самое, что active_current выше
    // (та самоссылочная - зависит от V того же нейрона), и не то же самое,
    // что chem/gap (это токовые синапсы с реальным знаком/весом) - peptide_
    // это отдельная, честно бинарная (предсказан контакт или нет) связность,
    // загружаемая прямо из данных коннектома (см. PEPTIDE_EDGES, loader.cpp),
    // а не по имени нейрона в коде. r_j - медленный "запас на выброс" у
    // каждого нейрона-источника, релаксирует к его же текущей sigmoid-
    // активации (тот же индикатор "насколько сейчас сигнализирует", что и
    // chem/gap/active_current) с постоянной времени peptide_tau_release_ -
    // честное упрощение кинетики плотных гранул (реально зависит от паттерна
    // импульсов, не только от среднего уровня). gain=0 (дефолт) - вклад в f
    // тождественно 0 для каждого нейрона на каждом шаге, как и у
    // active_current. См. tests/worm_pdf1_calibration за статусом калибровки.
    void set_peptide_connectivity(CsrMatrix peptide, std::vector<NeuronId> source_ids) {
        assert(peptide.num_rows() == size() && peptide.num_cols() == size());
        peptide_ = std::move(peptide);
        peptide_source_ids_ = std::move(source_ids);
    }
    void set_peptide_gain(float gain, float tau_release) {
        peptide_gain_ = gain;
        peptide_tau_release_ = std::max(1e-3f, tau_release);
    }

private:
    std::vector<NeuronType> types_;
    std::vector<NeuronParams> params_;
    CsrMatrix chemical_;
    CsrMatrix gap_;
    std::vector<float> gap_row_sums_;

    std::vector<float> state_;
    std::vector<float> next_state_;
    std::vector<float> external_input_;

    // Скретч-буферы, переиспользуются между шагами, чтобы не аллоцировать
    // память в горячем цикле.
    mutable std::vector<float> activation_scratch_;
    mutable std::vector<float> chem_input_scratch_;
    mutable std::vector<float> gap_input_scratch_;

    float activation_theta_ = 0.0f;
    float activation_slope_ = 1.0f;
    float chem_gain_ = 1.0f;
    float gap_gain_ = 1.0f;
    float leak_scale_ = 1.0f;

    // Активный ток - см. set_active_current(_targets) выше.
    std::vector<NeuronId> active_ids_;
    mutable std::vector<float> active_current_scratch_;
    std::vector<float> active_w_;
    std::vector<float> next_active_w_;
    float active_gain_ = 0.0f;
    float active_tau_w_ = 1.0f;

    // Нейропептид (PDF-1/PDFR-1) - см. set_peptide_connectivity выше.
    // Дефолтная (пустая) CsrMatrix имеет num_rows()==num_cols()==0, так что
    // accumulate_matvec на ней - безопасный no-op, если сеттер ни разу не
    // вызывался (старый файл коннектома без PEPTIDE_EDGES).
    CsrMatrix peptide_;
    std::vector<NeuronId> peptide_source_ids_;
    mutable std::vector<float> peptide_current_scratch_;
    std::vector<float> peptide_release_;
    std::vector<float> next_peptide_release_;
    float peptide_gain_ = 0.0f;
    float peptide_tau_release_ = 20.0f;
};

} // namespace connectome
