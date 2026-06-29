# FieldEngine

A 2D cellular automaton engine written in C++17. Runs Conway's Game of Life (and
any life-like rule) on a chunked world with a multithreaded simulation loop and
an OpenGL renderer. Built as a portfolio project to learn GPU programming and
engine architecture.

![150 Gosper Glider Guns running on CUDA](docs/gosper_guns.png)
<!-- replace with an actual screenshot when you have one -->

---

## What it does

- **Chunked world** — the field is split into fixed-size chunks; only
  live/visible chunks are simulated
- **Parallel simulation** — custom thread pool (TaskScheduler) computes chunks
  in parallel; double-buffering prevents the renderer from reading a half-updated
  frame
- **Pluggable backends** — rule table is plain data (`uint8_t table[2][9]`), so
  the same rule runs identically on CPU and CUDA without duplication
- **CUDA backend** — shared-memory tiling (16×16 blocks, 18×18 tiles loaded once
  per block); lazy CUDA-GL interop writes results directly into the GL VBO,
  skipping the PCI-E upload on the way back
- **RLE loader** — parses standard `.rle` pattern files from
  [conwaylife.com](https://conwaylife.com); drop any pattern in `patterns/` and
  stamp it anywhere on the world with `stampPattern()`
- **Correctness tests** — headless `Demo_life_test`: block still-life, blinker
  (period 2), glider (4-step position), and a CPU-vs-CUDA byte-identical
  agreement check over 100 steps on a 256×256 grid

---

## Benchmark

Measured on RTX 3060 Ti, chunk 1024×1024, 200 iterations, Release build:

| Backend | Throughput     |
|---------|----------------|
| CPU     | 281 Mcells/s   |
| CUDA    | **1909 Mcells/s** |
| Speedup | **6.8×**       |

---

## Demos

| Executable | What it shows |
|---|---|
| `Demo_minimallife_demo` | 150 Gosper Glider Guns loaded from RLE, running on CUDA |
| `Demo_fractal_demo` | Mandelbrot set rendered through the same chunk pipeline — proves the engine isn't hardwired to automata |
| `Demo_benchmark_demo` | Headless throughput measurement, CPU vs GPU |
| `Demo_life_test` | Correctness tests — exits 0 on pass, 1 on failure |

---

## Build

**Requirements:** Windows, Visual Studio 2022 (MSVC), CMake ≥ 3.18, Ninja.
GLFW, GLAD, and GLM are bundled in `libs/` — nothing to install.
CUDA Toolkit is optional; the project builds CPU-only without it.

```bash
cmake --preset x64-release
cmake --build out/build/x64-release
```

If CUDA Toolkit is present, CMake picks it up automatically:
```
-- CUDA found – GPU simulation backend enabled (arch 86)
```

To force CPU-only: `-DFE_USE_CUDA=OFF`

To run the tests:
```bash
./out/build/x64-release/Demo_life_test.exe
# ALL TESTS PASSED
```

---

## Why I built this

This isn't my first graphics project — before this I wrote a console flower
field (WinAPI mouse input), a raw OpenGL renderer from scratch, and a naive
engine with no chunks or threads. This is the step where I wanted to actually
solve the hard parts:

- How to separate simulation and rendering into threads without races
- How to write a thread pool and get the synchronization right
- Whether GPU really speeds things up and by how much (answer: 6.8×)
- How CUDA-GL interop avoids the round-trip through system RAM

The architecture is deliberately simple — no ECS, no scripting, no editor. One
thing done properly beats five things done halfway.

---

## What's next

- Pan and zoom with the mouse
- Generation counter and FPS in the window title
- More RLE patterns (R-pentomino chaos run, spaceships)
