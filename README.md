# Tessera

![Build](https://github.com/wonfeel/Tessera/actions/workflows/build.yml/badge.svg)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat)
![CUDA](https://img.shields.io/badge/CUDA-optional-76b900?style=flat)
![OpenGL](https://img.shields.io/badge/OpenGL-4.6-white?style=flat)
![Platform](https://img.shields.io/badge/platform-Windows-0078d7?style=flat)
![License](https://img.shields.io/badge/license-MIT-green?style=flat)

**[Русский](README_RU.md)** · **[Architecture notes](https://wonfeel.github.io/Tessera/architecture.html)** · **[Adding a new demo](https://wonfeel.github.io/Tessera/new-demo.html)**

A 2D simulation engine in C++/CUDA/OpenGL, released under the [MIT license](LICENSE).
The world is chunked - only live chunks are simulated, simulation and rendering run on separate
threads, and the simulation backend is swappable (CPU or CUDA) behind one interface. Built to
actually understand threads, thread pools, CUDA and OpenGL, not just read about them.

**The problem:** simulate an effectively unbounded field, updating only the live regions, in
parallel across CPU/GPU, without ever stalling the render.
**The approach:** split the world into chunks; simulate the live ones on a custom thread pool;
keep compute and commit from overlapping with a phase barrier; run the same code on CPU or a
CUDA backend; give rendering its own thread so a heavy step never drops a frame.

How each piece works - ownership, the phase barrier, chunk lifecycle - is in the
[architecture notes](https://wonfeel.github.io/Tessera/architecture.html).

---

## Showcases

Standalone demos built on this engine. Each pulls Tessera in via CMake `FetchContent`, so there
is nothing to clone by hand:

- **[HexLife](https://github.com/wonfeel/HexLife)** - Conway's Game of Life on the chunked field:
  `.rle` pattern loading, interactive drawing, GIF export.
- **[WaveLight](https://github.com/wonfeel/WaveLight)** - light as three dispersive wave fields
  (R/G/B) on a hex grid: paintable prism, phased beam tool.
- **[Wormy](https://github.com/wonfeel/Wormy-c302)** - a C. elegans body driven by the real
  401-neuron connectome through resistive-force-theory physics.

<p align="center">
  <img src="https://raw.githubusercontent.com/wonfeel/HexLife/main/assets/gun_eater.gif" height="200" alt="Gosper gun" />
  &nbsp;&nbsp;
  <img src="https://raw.githubusercontent.com/wonfeel/HexLife/main/assets/random_field.gif" height="200" alt="Random field spreading across chunks" />
</p>

---

## Why I made this

Before this I had a flower field rendered in the Windows console, a raw OpenGL renderer built
from scratch, and a first attempt at an "engine" with no chunks and no threads that lagged badly.
This is where I fixed that last one, and answered three questions for myself: how to split
simulation and rendering across threads without getting races, how to write a thread pool that
actually works, and whether the GPU really makes it faster (it does, but only past a point - see
the benchmark).

---

## What it can do

- Chunked world - only live chunks are simulated, and state spreads into neighbour chunks as it
  reaches their borders (Game-of-Life gliders cross chunk boundaries intact).
- Simulation runs in parallel on a custom thread pool; rendering has its own thread, so a heavy
  step never stutters the picture.
- One backend interface (`ISimulationBackend`) - CPU and CUDA are interchangeable and the rest of
  the engine doesn't know which one it got. The CUDA backend uses shared-memory tiling to cut
  global-memory reads.
- Hex-grid rendering alongside the square-grid one, for demos that need isotropic neighbours
  (light, worm).
- Built into the base `Application`, so every demo gets them for free: screen recording (F9),
  GIF export, ImGui panels.

CUDA-GL interop (writing results straight into the GL vertex buffer to skip the PCI-E round trip)
is implemented, but falls back to the regular copy path on Windows WDDM - the simulation runs on
worker threads where the GL context isn't current.

---

## Benchmark

Conway's rule, one chunk, 200 iterations, median of 3 runs. Ryzen 5 5600 (6c/12t) + RTX 3060 Ti,
**Release build**. The CPU column is **single-threaded**: `CpuLifeBackend` uses no threads and no
SIMD, it walks one chunk start to finish on one thread. Cross-chunk parallelism is `TaskScheduler`'s
job and is measured separately.

| Chunk size | CPU (1 thread) | CUDA          | Speedup |
|------------|----------------|---------------|---------|
| 256²       | 271 Mcells/s   | 335 Mcells/s  | 1.2×    |
| 512²       | 281 Mcells/s   | 1085 Mcells/s | 3.9×    |
| 1024²      | 283 Mcells/s   | 2281 Mcells/s | 8.1×    |
| 2048²      | 281 Mcells/s   | 2967 Mcells/s | 10.6×   |

The interesting number here isn't the speedup, it's the break-even point: below ~512² the GPU
barely wins, because the launch and the PCI-E round trip cost more than the work itself.

This benchmark is **transfer-bound, not compute-bound** - the kernel only uses ~1.3% of the card's
memory bandwidth, while host↔device copies eat about two thirds of each iteration. Full arithmetic,
plus why an earlier version of this table claimed 109×, is in
[docs/BENCHMARKS.md](docs/BENCHMARKS.md).

Thread pool, many chunks across 12 threads: **5.2×** at 800 chunks, and **0.64×** (slower!) on a
single chunk - one chunk is one task is one thread, so the pool can only add overhead there.

Run it yourself: `Test_benchmark <chunkSize> <iterations>` and `Test_batch_benchmark`.

---

## Build

CMake 3.20+ and Ninja, on Windows or Linux. CI builds both on every push.

**Windows** (Visual Studio 2022 / MSVC). Dependencies ([GLFW](https://www.glfw.org/),
[GLAD](https://glad.dav1d.de/), [GLM](https://glm.g-truc.net/),
[Dear ImGui](https://github.com/ocornut/imgui)) are vendored in `libs/` - nothing else to install.

```bash
cmake --preset x64-release
cmake --build out/build/x64-release
```

**Linux** (GCC or Clang). Only GLFW comes from the system - `libs/` ships the Windows build of it;
GLAD, GLM and ImGui are vendored and used as-is.

```bash
sudo apt install ninja-build libglfw3-dev libgl1-mesa-dev xorg-dev
cmake --preset linux-release
cmake --build out/build/linux-release
```

CUDA is optional. Without it the project builds CPU-only, and CMake prints which one it picked:

```
-- CUDA found – GPU simulation backend enabled
-- CUDA not found – building CPU-only simulation backend
```

---

## Demos and tests in this repo

Small demos kept here for engine development and testing. The full interactive version of each
lives in its own showcase repo (see above):

```
Demo_life_minimal   Randomized field + Gosper gun (HexLife has the full one).
Demo_light          One LightField, click-to-pluck, no UI (WaveLight has the full one).
Demo_cloth          Spring-network cloth simulation.
```

Tests under `tests/` are headless, exit 0 = pass:

```
Test_correctness    rule + .rle parser, and CPU vs CUDA byte-identical after 100 steps
Test_propagation    a glider must cross a chunk boundary intact
Test_capture        deterministic GIF dump, used as a regression fingerprint
Test_benchmark      CPU vs GPU throughput
Test_batch_benchmark  thread-pool scaling: 1..800 chunks, single vs 12 threads
Test_light_frame_profile  where a frame goes: splits sync overhead into
                    field-mutex wait vs thread-pool competition, and measures
                    the zero-copy render path against the snapshot one
Test_thread_safety  thread-pool + chunk-store stress, meant to run under TSan
tests/light_field_* regression + benchmarks for the wave field
```

Measured output of all of these, with hardware and caveats, is committed in
[docs/BENCHMARKS.md](docs/BENCHMARKS.md) - including one optimization that turned out to change
nothing, and why.

`Test_propagation` is the one I added after finding gliders were being clipped at chunk borders -
it stamps a glider near a boundary and checks it comes out the other side with the right shape
and offset.

### Sanitizers

The races this engine actually had - inserting into the chunk store under a shared lock, starting
the pool before the running flag was set, a lost wakeup in shutdown - were found by reading the
code, which is not a method that keeps working. `Test_thread_safety` replays those scenarios so
ThreadSanitizer can check them mechanically. MSVC has no TSan, so this is Linux-only, and CI runs
it on every push:

```bash
cmake --preset linux-tsan
cmake --build out/build/linux-tsan --target Test_thread_safety
ctest --test-dir out/build/linux-tsan -R Test_thread_safety --output-on-failure
```

AddressSanitizer works on both platforms: add `-DFE_ENABLE_ASAN=ON` to any preset.

---

## What's not done yet

- Only 2-state, totalistic "life-like" rules so far (no multi-state automata).
- CUDA-GL interop falls back to the copy path on WDDM (see above).
