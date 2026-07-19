#include "csr_matrix.hpp"

#include <algorithm>
#include <cassert>

namespace connectome {

CsrMatrix CsrMatrix::from_edges(NeuronId num_neurons, std::vector<Edge> edges) {
    std::vector<Triplet> triplets;
    triplets.reserve(edges.size());
    for (const Edge& e : edges) {
        triplets.push_back(Triplet{/*row=target*/ e.target, /*col=source*/ e.source, e.weight});
    }
    return from_triplets(num_neurons, num_neurons, std::move(triplets));
}

CsrMatrix CsrMatrix::from_triplets(NeuronId num_rows, NeuronId num_cols,
                                    std::vector<Triplet> triplets) {
    std::sort(triplets.begin(), triplets.end(), [](const Triplet& a, const Triplet& b) {
        return a.row != b.row ? a.row < b.row : a.col < b.col;
    });

    CsrMatrix mat;
    mat.num_rows_ = num_rows;
    mat.num_cols_ = num_cols;
    mat.row_ptr_.assign(static_cast<std::size_t>(num_rows) + 1, 0);

    mat.weights_.reserve(triplets.size());
    mat.col_index_.reserve(triplets.size());

    std::size_t i = 0;
    for (NeuronId row = 0; row < num_rows; ++row) {
        mat.row_ptr_[row] = static_cast<std::uint32_t>(mat.weights_.size());
        while (i < triplets.size() && triplets[i].row == row) {
            NeuronId col = triplets[i].col;
            float weight = triplets[i].weight;
            ++i;
            // Дублирующиеся (row, col) суммируем в один вес.
            while (i < triplets.size() && triplets[i].row == row && triplets[i].col == col) {
                weight += triplets[i].weight;
                ++i;
            }
            mat.weights_.push_back(weight);
            mat.col_index_.push_back(col);
        }
    }
    mat.row_ptr_[num_rows] = static_cast<std::uint32_t>(mat.weights_.size());

    assert(i == triplets.size());
    return mat;
}

void CsrMatrix::accumulate_matvec(const std::vector<float>& x, std::vector<float>& y) const {
    assert(x.size() >= num_cols_);
    assert(y.size() >= num_rows_);
    for (NeuronId row = 0; row < num_rows_; ++row) {
        float sum = 0.0f;
        for (std::uint32_t k = row_ptr_[row]; k < row_ptr_[row + 1]; ++k) {
            sum += weights_[k] * x[col_index_[k]];
        }
        y[row] += sum;
    }
}

std::vector<float> CsrMatrix::row_sums() const {
    std::vector<float> sums(num_rows_, 0.0f);
    for (NeuronId row = 0; row < num_rows_; ++row) {
        float sum = 0.0f;
        for (std::uint32_t k = row_ptr_[row]; k < row_ptr_[row + 1]; ++k) {
            sum += weights_[k];
        }
        sums[row] = sum;
    }
    return sums;
}

} // namespace connectome
