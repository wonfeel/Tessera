// tests/worm_network_eigenmodes/main.cpp
//
// Research harness (kept in repo, same convention as tests/worm_speed_
// calibration and tests/worm_chemotaxis_calibration).
//
// Follow-up to tests/worm_speed_calibration: that investigation established
// the worm's emergent bend cycle is ~30-200+ simulated seconds (vs a real
// worm's 0.5-2s), that substrate friction and the proprioceptive body-
// feedback loop are NOT the cause (disabling proprioceptiveGain doesn't
// speed it up, if anything slows it further - see WormSim.cpp's ctor
// comment), and that per-neuron-class leak/capacitance scaling does not
// reliably fix it (that search's own "win" turned out to be a seed-base
// measurement artifact). What's left: the raw RECURRENT NETWORK dynamics
// (401 real C. elegans neurons, real Cook et al. 2019 chemical + gap
// junction wiring) themselves, independent of the body.
//
// This harness doesn't search parameters - it EXPORTS the network's actual
// structure (chemical/gap junction weight matrices, per the exact linear
// system network.cpp's Network::step integrates) so it can be analyzed with
// a real numerical linear algebra library (numpy/scipy - not available in
// C++ here, no vendored eigenvalue solver in this project). The question:
// what is the network's own slowest LINEAR mode (eigenvalue closest to 0),
// and which neurons dominate it? A large recurrent network's collective
// relaxation can be far slower than any single neuron's own time constant
// (tau=capacitance/leak=1s at defaults) - this is standard for coupled
// linear systems, not a bug, but *this specific connectome's* slowest mode
// timescale is an empirical question about Cook et al. 2019's real wiring
// data, not something to reason out by hand.
//
// Linearization (see network.cpp's Network::step for the exact non-linear
// version this approximates): around V=0 for every neuron (a defensible
// operating point - rest=0 default, and intrinsic noise/gradient inputs
// average to ~0 over time), sigmoid((V-theta)/slope) linearizes to
// sigmoid(0) + sigmoid'(0)/slope * V = 0.5 + 0.25*V at the default
// theta=0/slope=1, so the chemical-synapse contribution linearizes to a
// CONSTANT (0.5 * chem_gain * row_sum, folds into the fixed point, doesn't
// affect eigenvalues) plus 0.25*chem_gain*W_chem*V (does affect them). Gap
// junctions are already exactly linear (no sigmoid). Both effects, plus the
// leak/capacitance/gap-self-drain diagonal (see network.cpp's `k` term,
// replicated exactly here including the Output-type leak=0 special case),
// combine into one matrix M such that dV/dt ~= M*V + const - its
// eigenvalues' real parts ARE the network's decay-mode timescales
// (1/|Re(lambda)|, in seconds since dt/gains are already in real-second
// units throughout this codebase); imaginary parts, if any, are angular
// oscillation frequencies (Im(lambda)/(2*pi) Hz).
//
// Usage: run this exe (writes network_export.txt into the CURRENT
// directory), then run tests/worm_network_eigenmodes/analyze.py (needs
// numpy/scipy - `py -3 analyze.py` on this machine) against that file.
#include "demo/worm/connectome/csr_matrix.cpp"
#include "demo/worm/connectome/network.cpp"
#include "demo/worm/connectome/loader.cpp"

#include <cstdio>
#include <string>

int main() {
    connectome::LoadedConnectome loaded = connectome::load_connectome("worm_data/celegans_herm.connectome");
    connectome::Network& net = loaded.network;
    const connectome::NeuronId n = net.size();

    std::FILE* f = std::fopen("network_export.txt", "w");
    if (!f) {
        std::fprintf(stderr, "failed to open network_export.txt for write\n");
        return 1;
    }

    // Production defaults this analysis targets - see WormSim.h Params{}.
    // chem_gain/gap_gain are WormSim::Params defaults (0.02/0.02), NOT
    // Network's own internal defaults (1.0/1.0) - WormSim::step() always
    // calls set_gains() with the UI-facing values before net.step(), so
    // these (not Network's ctor defaults) are what actually runs live.
    constexpr float kChemGain = 0.02f, kGapGain = 0.02f, kLeakScale = 1.0f;
    constexpr float kActivationTheta = 0.0f, kActivationSlope = 1.0f;
    // load_connectome gives every neuron identical NeuronParams{leak=1,
    // rest=0, capacitance=1} regardless of type (see WormSim.cpp's ctor
    // comment, and network.hpp's scale_type_params doc) - Network has no
    // public getter for per-neuron params (only a setter to scale them), so
    // this hardcodes the known, verified default rather than adding one for
    // a one-off analysis.
    constexpr float kLeak = 1.0f, kRest = 0.0f, kCapacitance = 1.0f;

    std::fprintf(f, "%u\n", n);
    std::fprintf(f, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", kChemGain, kGapGain, kLeakScale, kActivationTheta,
                 kActivationSlope, kLeak, kCapacitance);
    (void)kRest;
    for (connectome::NeuronId i = 0; i < n; ++i) {
        std::fprintf(f, "%u\t%s\t%d\n", i, loaded.names[static_cast<std::size_t>(i)].c_str(),
                     static_cast<int>(net.type(i)));
    }

    const connectome::CsrMatrix& chem = net.chemical();
    std::fprintf(f, "%zu\n", chem.nnz());
    for (connectome::NeuronId row = 0; row < chem.num_rows(); ++row) {
        for (std::uint32_t k = chem.row_ptr()[row]; k < chem.row_ptr()[row + 1]; ++k) {
            std::fprintf(f, "%u\t%u\t%.8f\n", row, chem.col_index()[k], chem.weights()[k]);
        }
    }

    const connectome::CsrMatrix& gap = net.gap();
    std::fprintf(f, "%zu\n", gap.nnz());
    for (connectome::NeuronId row = 0; row < gap.num_rows(); ++row) {
        for (std::uint32_t k = gap.row_ptr()[row]; k < gap.row_ptr()[row + 1]; ++k) {
            std::fprintf(f, "%u\t%u\t%.8f\n", row, gap.col_index()[k], gap.weights()[k]);
        }
    }

    std::fclose(f);
    std::printf("wrote network_export.txt: %u neurons, %zu chem edges, %zu gap edges\n", n, chem.nnz(), gap.nnz());
    return 0;
}
