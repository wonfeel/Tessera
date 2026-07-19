# tests/worm_network_eigenmodes/analyze.py
#
# Consumes network_export.txt (written by Test_worm_network_eigenmodes.exe -
# see that file's header for the full derivation) and computes the
# eigenvalues of the linearized network dynamics at production gains
# (chemGain=0.02, gapGain=0.02, defaults from WormSim.h). Needs numpy - on
# this machine that's `py -3 analyze.py`, not bare `python`.
import numpy as np

TYPE_NAMES = ["Input", "InputProcessing", "Processing", "ProcessingOutput", "Output"]


def load(path="network_export.txt"):
    with open(path) as f:
        lines = f.read().splitlines()
    idx = 0
    n = int(lines[idx]); idx += 1
    chem_gain, gap_gain, leak_scale, theta, slope, leak_default, cap_default = (
        float(x) for x in lines[idx].split()
    )
    idx += 1
    names = [None] * n
    types = np.zeros(n, dtype=int)
    for _ in range(n):
        i, name, t = lines[idx].split("\t")
        idx += 1
        names[int(i)] = name
        types[int(i)] = int(t)
    nnz_chem = int(lines[idx]); idx += 1
    chem = np.zeros((n, n))
    for _ in range(nnz_chem):
        r, c, w = lines[idx].split("\t")
        idx += 1
        chem[int(r), int(c)] += float(w)
    nnz_gap = int(lines[idx]); idx += 1
    gap = np.zeros((n, n))
    for _ in range(nnz_gap):
        r, c, w = lines[idx].split("\t")
        idx += 1
        gap[int(r), int(c)] += float(w)
    return dict(
        n=n, chem_gain=chem_gain, gap_gain=gap_gain, leak_scale=leak_scale,
        theta=theta, slope=slope, leak_default=leak_default, cap_default=cap_default,
        names=names, types=types, chem=chem, gap=gap,
    )


def build_M(d):
    n = d["n"]
    leak = np.full(n, d["leak_default"] * d["leak_scale"])
    leak[d["types"] == 4] = 0.0  # Output: leak forced to 0, see network.cpp
    cap = np.full(n, d["cap_default"])
    gap_row_sums = d["gap"].sum(axis=1)
    k = (leak + d["gap_gain"] * gap_row_sums) / cap

    # sigmoid'(0) at the V=0 operating point (theta=0 default): 0.25/slope.
    sigmoid_slope = 0.25 / d["slope"]
    M = (d["chem_gain"] * sigmoid_slope * d["chem"] + d["gap_gain"] * d["gap"]) / cap[:, None]
    M -= np.diag(k)
    return M


def main():
    d = load()
    n = d["n"]
    print(f"n={n} chem_gain={d['chem_gain']} gap_gain={d['gap_gain']} "
          f"leak={d['leak_default']} capacitance={d['cap_default']}")

    M = build_M(d)
    eigvals, eigvecs = np.linalg.eig(M)

    unstable = eigvals[eigvals.real > 1e-9]
    print(f"\n{len(unstable)}/{n} eigenvalues have POSITIVE real part (would be linearly "
          f"unstable/growing around V=0, if any - sigmoid saturation is what actually "
          f"bounds it in the real nonlinear system).")

    order = np.argsort(-eigvals.real)  # least-negative (slowest decay) first
    print("\nSlowest 15 modes (closest to the imaginary axis = slowest decay):")
    print(f"{'#':>3} {'Re(lambda)':>12} {'Im(lambda)':>12} {'timescale(s)':>13} {'freq(Hz)':>10}")
    for rank, i in enumerate(order[:15]):
        lam = eigvals[i]
        timescale = 1.0 / abs(lam.real) if abs(lam.real) > 1e-12 else float("inf")
        freq = lam.imag / (2 * np.pi)
        print(f"{rank:>3} {lam.real:>12.5f} {lam.imag:>12.5f} {timescale:>13.2f} {freq:>10.4f}")

    slow_i = order[0]
    lam = eigvals[slow_i]
    vec = eigvecs[:, slow_i]
    mags = np.abs(vec)
    top = np.argsort(-mags)[:20]
    print(f"\nSlowest mode (lambda={lam.real:.5f}{lam.imag:+.5f}j, "
          f"timescale={1.0/abs(lam.real):.1f}s): top 20 contributing neurons by |eigenvector component|")
    print(f"{'neuron':>10} {'type':>18} {'|component|':>12} {'component (re,im)':>22}")
    for i in top:
        print(f"{d['names'][i]:>10} {TYPE_NAMES[d['types'][i]]:>18} {mags[i]:>12.5f} "
              f"({vec[i].real:>+.4f},{vec[i].imag:>+.4f})")

    # Breakdown by neuron type of contribution to the slow mode (sum of
    # |component|^2 per type, normalized) - which TYPE of neuron dominates,
    # not just which individual ones.
    print("\nSlow-mode energy by neuron type (sum |component|^2, normalized):")
    energy = mags ** 2
    energy /= energy.sum()
    for t in range(5):
        mask = d["types"] == t
        if mask.sum() == 0:
            continue
        print(f"  {TYPE_NAMES[t]:>18}: {energy[mask].sum():.4f}  ({mask.sum()} neurons)")


def sweep_output_leak(d):
    # Output (muscle) neurons have leak forced to 0 in network.cpp by design
    # - their only restoring force is gap-junction coupling to neighboring
    # muscles (see the slow-mode-is-100%-Output finding in main()). This
    # asks a purely linear-algebra question, no C++ changes: if Output rows
    # got their own nonzero leak (hypothetically), how much would the
    # slowest mode's timescale drop? Answers "is this worth implementing"
    # before touching network.cpp's actual integration logic.
    print("\n=== What if Output/muscle neurons had their own leak? (hypothetical, "
          "linear algebra only - not yet a code change) ===")
    n = d["n"]
    cap = np.full(n, d["cap_default"])
    gap_row_sums = d["gap"].sum(axis=1)
    sigmoid_slope = 0.25 / d["slope"]
    off_diag = (d["chem_gain"] * sigmoid_slope * d["chem"] + d["gap_gain"] * d["gap"]) / cap[:, None]
    is_output = d["types"] == 4
    for output_leak in [0.0, 0.05, 0.1, 0.2, 0.5, 1.0]:
        leak = np.full(n, d["leak_default"] * d["leak_scale"])
        leak[is_output] = output_leak
        k = (leak + d["gap_gain"] * gap_row_sums) / cap
        M = off_diag - np.diag(k)
        eigvals = np.linalg.eigvals(M)
        slow = eigvals[np.argsort(-eigvals.real)[0]]
        timescale = 1.0 / abs(slow.real) if abs(slow.real) > 1e-12 else float("inf")
        print(f"  output_leak={output_leak:<5} slowest lambda.real={slow.real:>10.5f}  "
              f"timescale={timescale:>9.2f}s")


if __name__ == "__main__":
    main()
    d = load()
    sweep_output_leak(d)
