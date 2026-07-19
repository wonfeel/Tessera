#include "network.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace connectome {

namespace {
float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
} // namespace

Network::Network(std::vector<NeuronType> types, std::vector<NeuronParams> params,
                  CsrMatrix chemical, CsrMatrix gap)
    : types_(std::move(types)),
      params_(std::move(params)),
      chemical_(std::move(chemical)),
      gap_(std::move(gap)) {
    assert(types_.size() == params_.size());
    const NeuronId n = size();
    assert(chemical_.num_rows() == n && chemical_.num_cols() == n);
    assert(gap_.num_rows() == n && gap_.num_cols() == n);

    gap_row_sums_ = gap_.row_sums();
    state_.assign(n, 0.0f);
    next_state_.assign(n, 0.0f);
    external_input_.assign(n, 0.0f);
    activation_scratch_.assign(n, 0.0f);
    chem_input_scratch_.assign(n, 0.0f);
    gap_input_scratch_.assign(n, 0.0f);
    active_current_scratch_.assign(n, 0.0f);
    active_w_.assign(n, 1.0f);   // канал полностью "открыт" в покое (см. set_active_current)
    next_active_w_.assign(n, 1.0f);
    peptide_current_scratch_.assign(n, 0.0f);
    peptide_release_.assign(n, 0.0f);
    next_peptide_release_.assign(n, 0.0f);
}

void Network::set_input(NeuronId id, float value) { external_input_[id] = value; }

void Network::step(float dt) {
    const NeuronId n = size();

    for (NeuronId i = 0; i < n; ++i) {
        activation_scratch_[i] = sigmoid((state_[i] - activation_theta_) / activation_slope_);
    }

    std::fill(chem_input_scratch_.begin(), chem_input_scratch_.end(), 0.0f);
    std::fill(gap_input_scratch_.begin(), gap_input_scratch_.end(), 0.0f);
    chemical_.accumulate_matvec(activation_scratch_, chem_input_scratch_);
    gap_.accumulate_matvec(state_, gap_input_scratch_);

    // Нейропептид (PDF-1/PDFR-1) - см. set_peptide_connectivity в network.hpp.
    // Как и chem/gap, gain применяется НИЖЕ (в f), не здесь - peptide_current_
    // scratch_ хранит "сырую" сумму по связности, до умножения на peptide_gain_.
    std::fill(peptide_current_scratch_.begin(), peptide_current_scratch_.end(), 0.0f);
    peptide_.accumulate_matvec(peptide_release_, peptide_current_scratch_);

    // Активный ток - только для явно заданных нейронов (active_ids_,
    // обычно десяток-другой из 401), не полный проход по n - см.
    // set_active_current в network.hpp. Использует activation_scratch_[id],
    // уже посчитанный выше (тот же sigmoid((V-theta)/slope), что и для
    // химических синапсов - разумно: и то, и другое - "насколько канал
    // открыт при этом V", просто chem-версия читается соседями, эта - самим
    // нейроном). w_i считается от СТАРОГО (пред-шаговое) activation, тем же
    // принципом, что и chem/gap - обратная связь идёт от уже случившегося
    // состояния, не от того, что только предстоит вычислить в этом шаге.
    for (NeuronId id : active_ids_) {
        const float act = activation_scratch_[id];
        active_current_scratch_[id] = active_gain_ * active_w_[id] * act;  // ==0 exactly when active_gain_==0
        const float w_inf = 1.0f - act;  // закрывается при деполяризации, открыт в покое
        const float alpha_w = std::clamp(dt / active_tau_w_, 0.0f, 1.0f);
        next_active_w_[id] = active_w_[id] + (w_inf - active_w_[id]) * alpha_w;
    }

    // Медленный "запас на выброс" пептида - только для явно заданных
    // нейронов-источников (peptide_source_ids_, из данных коннектома), тем же
    // принципом старого-состояния, что и active_w_ выше.
    for (NeuronId id : peptide_source_ids_) {
        const float act = activation_scratch_[id];
        const float alpha_r = std::clamp(dt / peptide_tau_release_, 0.0f, 1.0f);
        next_peptide_release_[id] = peptide_release_[id] + (act - peptide_release_[id]) * alpha_r;
    }

    for (NeuronId i = 0; i < n; ++i) {
        // gap_input_scratch_[i] = sum_j g_ij * V_j (соседи, БЕЗ члена -rowsum_i*V_i —
        // тот уходит в k ниже, а не остаётся форсингом, см. комментарий у k).
        const float gap_neighbor_current = gap_gain_ * gap_input_scratch_[i];
        const float chem_current = chem_gain_ * chem_input_scratch_[i];

        switch (types_[i]) {
            case NeuronType::Input:
                next_state_[i] = external_input_[i];
                break;
            case NeuronType::Output:
            case NeuronType::InputProcessing:
            case NeuronType::Processing:
            case NeuronType::ProcessingOutput: {
                // dV/dt = -k*V + f. Output не имеет утечки/внешнего входа (leak=0,
                // drive=0), но у мышц gap junction с соседями — тоже реальная физика
                // (соседние мышцы электрически спарены), и её самозатухающий член
                // (rowsum) идёт в k здесь же, а не отдельной веткой. Это важно: явный
                // Эйлер по x_{n+1} = f - k*x_n расходится при k>1, а мышцы плотно
                // gap-связаны друг с другом (rowsum легко даёт k>1 даже на дефолтном
                // gap_gain, что и производило inf/NaN). Экспоненциальный
                // (полу-неявный) шаг точно интегрирует линейную часть и НЕ расходится
                // ни при каком dt/k >= 0: V_new = V*exp(-k dt) + (f/k)*(1-exp(-k dt)).
                // При k=0 (изолированный узел без утечки и gap-соседей) для Output
                // это означает "нет память вообще" -> подставляем f напрямую, а не
                // копим его по dt (иначе безынерционный считыватель начал бы дрейфовать).
                //
                // leak=0 for Output - TRIED CHANGING, REVERTED. tests/worm_network_
                // eigenmodes found the network's slowest linear mode (~755s) is 100%
                // Output/muscle energy, exactly because leak=0 leaves muscles with
                // only weak gap-junction diffusion as a restoring force - correct,
                // confirmed diagnosis. Giving Output an ordinary nonzero leak (like
                // every other type) DID collapse that eigenvalue as predicted, and DID
                // measurably raise the real (nonlinear) network's oscillation
                // frequency (~0.03-0.05Hz -> ~0.15-0.18Hz, confirmed via the "trace"
                // mode). But real crawling speed (centroid path length/time, no food,
                // the same body-lengths/s metric as tests/worm_speed_calibration) got
                // ~28x WORSE (0.0108 -> 0.00039 BL/s, confirmed across 15 independent
                // seed bases, not a fluke), not better - a faster neural oscillation
                // doesn't help if the body can't turn it into a coherent propulsive
                // wave. A follow-up attempt to compensate by also speeding up
                // WormBody's own angle-decay time constant (body.cpp, was going to
                // let the body "keep up" with the faster network) made it MUCH worse
                // still (speed collapsed further, efficiency 0.52 -> 0.06) - the
                // body's decay-to-neutral term isn't just a low-pass filter to widen,
                // it's also how long a commanded bend PERSISTS to do propulsive work;
                // shortening that persistence starves solve_propulsion of anything to
                // push against. Reverted both changes. Net conclusion: this network's
                // slow eigenvalue-timescale is real and now well-understood (muscle
                // leak=0), but is NOT simply "a bug to fix" - the current slow network
                // + body pairing is, empirically, a working (if unrealistically slow)
                // balance, and naive attempts to speed up either side independently
                // broke the coupling between them rather than improving it. A real fix
                // would need to understand and preserve that coupling while speeding
                // both sides up together, not tested here.
                const NeuronParams& p = params_[i];
                const bool is_output = types_[i] == NeuronType::Output;
                const float leak = is_output ? 0.0f : p.leak * leak_scale_;
                const float drive = is_output ? 0.0f : external_input_[i];
                const float k = (leak + gap_gain_ * gap_row_sums_[i]) / p.capacitance;
                const float f = (leak * p.rest + chem_current + gap_neighbor_current + drive + active_current_scratch_[i]
                                  + peptide_gain_ * peptide_current_scratch_[i]) / p.capacitance;
                if (k > 1e-6f) {
                    const float decay = std::exp(-k * dt);
                    next_state_[i] = state_[i] * decay + (f / k) * (1.0f - decay);
                } else if (is_output) {
                    next_state_[i] = f;
                } else {
                    next_state_[i] = state_[i] + dt * f;
                }
                break;
            }
        }
    }

    state_.swap(next_state_);
    active_w_.swap(next_active_w_);
    peptide_release_.swap(next_peptide_release_);
}

} // namespace connectome
