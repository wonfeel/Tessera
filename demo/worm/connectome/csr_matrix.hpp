#pragma once

#include <cstdint>
#include <vector>

#include "types.hpp"

namespace connectome {

// Разреженная матрица весов синапсов в формате CSR (compressed sparse row).
// Строка i хранит входящие связи нейрона i: sum_j weights[k] * x[col_index[k]]
// для k в [row_ptr[i], row_ptr[i+1]).
//
// Коннектом червя имеет плотность связей ~2-5%, поэтому плотная матрица
// (302*302*4 байта = 365 КБ) не влезает в память Arduino, а CSR с int8/int16
// весами и индексами укладывается в единицы-десятки КБ и может целиком лежать
// во флеше (PROGMEM), не в ОЗУ.
class CsrMatrix {
public:
    // Синаптическая связь source -> target. Использовать именно эту структуру
    // (а не строки/столбцы матрицы напрямую) при построении сети из данных
    // коннектома или в демо-примерах — порядок "откуда/куда" в ней однозначен,
    // в отличие от row/col, где row исторически означает "приёмник".
    struct Edge {
        NeuronId source;
        NeuronId target;
        float weight;
    };

    struct Triplet {
        NeuronId row; // приёмник (post-synaptic) — строка CSR, по которой идёт суммирование
        NeuronId col; // источник (pre-synaptic)
        float weight;
    };

    // Строит CSR из списка синаптических связей source->target. Несколько
    // связей между одной и той же парой суммируются (несколько физических
    // синапсов = одна связь с суммарным весом).
    static CsrMatrix from_edges(NeuronId num_neurons, std::vector<Edge> edges);

    // Низкоуровневый вариант напрямую в терминах CSR (row=приёмник, col=источник).
    // Предпочитать from_edges, чтобы не перепутать направление связи.
    static CsrMatrix from_triplets(NeuronId num_rows, NeuronId num_cols,
                                    std::vector<Triplet> triplets);

    // y[i] += sum_j weights[i,j] * x[j]  для всех строк i.
    // y должен быть заранее обнулён (или содержать начальное значение,
    // которое нужно накопить) и иметь длину num_rows().
    void accumulate_matvec(const std::vector<float>& x, std::vector<float>& y) const;

    // Сумма весов по строке i (нужна, например, для проводимости gap junction:
    // ток_i = sum_j g_ij * (V_j - V_i) = (matvec)_i - row_sum[i] * V_i).
    std::vector<float> row_sums() const;

    NeuronId num_rows() const { return num_rows_; }
    NeuronId num_cols() const { return num_cols_; }
    std::size_t nnz() const { return weights_.size(); }

    // Прямой доступ к CSR-хранилищу для обхода связей (например, для
    // отрисовки графа): row=приёмник (target), col_index[k]=источник (source).
    const std::vector<std::uint32_t>& row_ptr() const { return row_ptr_; }
    const std::vector<NeuronId>& col_index() const { return col_index_; }
    const std::vector<float>& weights() const { return weights_; }
    // Изменяемый доступ - для калибровки поверх сырых данных коннектома в
    // рантайме (см. Network::scale_synapse_sign). Как и Network::
    // scale_type_params, предполагается ОДИН вызов сразу после конструктора
    // (мультипликативно поверх текущих значений - повторный вызов копит
    // эффект, не сбрасывает).
    std::vector<float>& weights_mutable() { return weights_; }

private:
    NeuronId num_rows_ = 0;
    NeuronId num_cols_ = 0;
    std::vector<float> weights_;
    std::vector<NeuronId> col_index_;
    std::vector<std::uint32_t> row_ptr_;
};

} // namespace connectome
