# Tessera

![Build](https://github.com/wonfeel/Tessera/actions/workflows/build.yml/badge.svg)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat)
![CUDA](https://img.shields.io/badge/CUDA-optional-76b900?style=flat)
![OpenGL](https://img.shields.io/badge/OpenGL-4.6-white?style=flat)
![Platform](https://img.shields.io/badge/platform-Windows-0078d7?style=flat)
![License](https://img.shields.io/badge/license-MIT-green?style=flat)

**[Русский](README_RU.md)** · **[Architecture notes](https://wonfeel.github.io/Tessera/architecture.html)** · **[Adding a new demo](https://wonfeel.github.io/Tessera/new-demo.html)**

A 2D simulation engine in C++/CUDA/OpenGL, released under the [MIT license](LICENSE).
The world is chunked — only live chunks are simulated, simulation and rendering run on
separate threads, and the simulation backend is swappable (CPU or CUDA) behind one
interface. Built to actually understand threads, thread pools, CUDA, and OpenGL — not
just read about them.

**The problem:** simulate an effectively unbounded field, updating only the live regions,
in parallel across CPU/GPU, without ever stalling the render.
**The approach:** split the world into chunks; simulate live chunks on a custom thread pool;
keep compute and commit from overlapping with a phase barrier; run the same rule unchanged on
CPU or a CUDA backend; give rendering its own thread so a heavy step never drops a frame.

---

## Contents

- [Showcases](#showcases)
- [Why I made this](#why-i-made-this)
- [What it can do](#what-it-can-do)
- [How it's put together](#how-its-put-together)
- [Benchmark](#benchmark)
- [Requirements](#requirements)
- [Dependencies](#dependencies)
- [Build](#build)
- [Demos in this repo](#demos-in-this-repo)
- [Tests](#tests)
- [What's not done yet](#whats-not-done-yet)

---

## Showcases

Full, standalone demos built on this engine (each pulls Tessera automatically
via CMake `FetchContent` — nothing to clone by hand):

**[HexLife](https://github.com/wonfeel/HexLife)** — Conway's Game of Life on the
chunked field, `.rle` pattern loading, interactive drawing, GIF export.

<p align="center">
  <img src="https://raw.githubusercontent.com/wonfeel/HexLife/main/assets/gun_eater.gif" height="200" alt="Gosper gun" />
  &nbsp;&nbsp;
  <img src="https://raw.githubusercontent.com/wonfeel/HexLife/main/assets/random_field.gif" height="200" alt="Random field spreading across chunks" />
</p>

**[WaveLight](https://github.com/wonfeel/WaveLight)** — light as three dispersive
wave fields (R/G/B) on a hex grid, paintable prism, phased beam tool.

**[Wormy](https://github.com/wonfeel/Wormy-c302)** — a C. elegans body + real
401-neuron connectome, driven by resistive-force-theory physics.

---

## Why I made this

Before this I had:
- a flower field rendered in the Windows console (mouse input via WinAPI)
- a raw OpenGL renderer built from scratch (shaders, buffers, the whole thing)
- a first attempt at an "engine" with no chunks and no threads — it lagged badly

This is where I tried to fix the things that were wrong with that last one.

The questions I wanted to answer for myself:
- How do you split simulation and rendering across threads without getting races?
- How do you write a thread pool that actually works?
- Does the GPU really make it faster, and by how much?

---

## What it can do

- The world is split into chunks — only live chunks are simulated, and state
  spreads into neighbour chunks as it reaches their borders (e.g. Game-of-Life
  gliders cross chunk boundaries correctly).
- Simulation runs in parallel on a custom thread pool; rendering runs on its own
  thread — the picture doesn't stutter when the simulation is heavy.
- The simulation backend sits behind one interface (`ISimulationBackend`) — CPU
  and CUDA are interchangeable, the rest of the engine doesn't know which it got.
- CUDA backend uses shared-memory tiling to cut global-memory reads.
- Tried CUDA-GL interop (writing results straight into the GL vertex buffer to
  skip the PCI-E round trip) — it falls back gracefully on Windows WDDM, where
  the GL context lives on a different thread.
- Hex-grid rendering alongside the square-grid one, for demos that need
  isotropic neighbors (light, worm).

---

## How it's put together

The two big classes used to do everything, so I split them into pieces that each
do one job:

- `ChunkStore` — owns the chunks, their lock, and the active-chunk list
- `SimulationCoordinator` — runs one generation: schedules chunk work on the
  thread pool, then commits the results. A small phase machine
  (`Idle → Computing → ReadyToCommit → Committing`) makes sure computing and
  committing never overlap, so neighbour reads always see one consistent
  generation.
- `ChunkMapRenderer` — draws the chunks that are on screen
- `ChunkGrid` — world ↔ chunk-local coordinate math
- `CameraController` — WASD / mouse-wheel / middle-drag camera
- `ChunkedTileMap` — a thin layer that ties those together

The simulation backend is behind an interface (`ISimulationBackend`), so CPU and
CUDA are interchangeable and the rest of the engine doesn't know which one it got.

---

## Benchmark

Conway's rule (via `demo/life`), RTX 30-series, one chunk, 100 iterations. The
GPU only pulls ahead once the field is big enough to hide the kernel-launch
overhead:

| Chunk size | CPU          | CUDA           | Speedup |
|------------|--------------|----------------|---------|
| 256²       | 37 Mcells/s  | 351 Mcells/s   | 9.5×    |
| 512²       | 35 Mcells/s  | 1230 Mcells/s  | 35×     |
| 1024²      | 37 Mcells/s  | 2366 Mcells/s  | 64×     |
| 2048²      | 37 Mcells/s  | 4034 Mcells/s  | 109×    |

Run it yourself: `Test_benchmark <chunkSize> <iterations>`.

---

## Requirements

- Windows
- Visual Studio 2022 (MSVC toolchain)
- CMake 3.20+ and Ninja
- CUDA Toolkit — optional, only needed for the GPU backend
- Python 3 + Pillow (`pip install pillow`) — only needed for `tools/capture_gif.py`

## Dependencies

Vendored in `libs/`, nothing else to install:

- [GLFW](https://www.glfw.org/) — window/context/input
- [GLAD](https://glad.dav1d.de/) — OpenGL function loading
- [GLM](https://glm.g-truc.net/) — vector/matrix math
- [Dear ImGui](https://github.com/ocornut/imgui) — debug/interactive UI panels

> [!Tip]
> Building without CUDA installed is fine — CMake detects it and falls back to the
> CPU-only backend automatically (see [Build](#build)).

---

## Build

Windows, Visual Studio 2022, CMake + Ninja.

```bash
cmake --preset x64-release
cmake --build out/build/x64-release
```

CUDA is optional — without it the project builds CPU-only. CMake prints which one
it picked:
```
-- CUDA found – GPU simulation backend enabled
-- CUDA not found – building CPU-only simulation backend
```

---

## Demos in this repo

Small demos kept here for engine development and testing — the full
interactive experience for each is in its own showcase repo (see
[Showcases](#showcases) above):

```
Demo_life_minimal   Randomized field + Gosper gun, nothing else (HexLife has the full one).
Demo_light          One LightField, click-to-pluck, no UI (WaveLight has the full one).
Demo_worm           C. elegans body + connectome (Wormy has the polished build/README).
Demo_cloth          Spring-network cloth simulation.
```

---

## Tests

Headless, exit 0 = pass. They live under `tests/`:

```
Test_correctness   rule + RLE-parser tests
Test_propagation   a glider must cross a chunk boundary intact
Test_capture       deterministic GIF dump — used as a regression fingerprint
Test_benchmark     CPU vs GPU throughput
```

`Test_correctness` checks:
- block still-life doesn't change
- blinker oscillates with period 2
- glider moves to the correct position after 4 steps
- CPU and CUDA produce byte-identical output after 100 steps on the same input
- the `.rle` parser decodes run-lengths, row jumps and whitespace correctly

`Test_propagation` is the one I added after finding gliders were being clipped at
chunk borders — it stamps a glider near a boundary and checks it comes out the
other side with the right shape and offset.

`tests/light_field_*` are the equivalent regression/benchmark tests for
`LightField` (used by the light demo).

---

## What's not done yet

> [!Warning]
> CUDA-GL interop still falls back to the regular copy path on WDDM (the
> simulation runs on worker threads where the GL context isn't current).

- Only 2-state, totalistic "life-like" rules so far (no multi-state automata).
