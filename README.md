# FieldEngine

A small 2D engine that runs cellular automata (like Conway's Game of Life) on a
chunked world and renders it with OpenGL. I built this to actually understand the
stuff that doesn't click from books alone.

---

## Why I made this

Before this I had:
- a flower field rendered in the Windows console (mouse input via WinAPI)
- a raw OpenGL renderer built from scratch (shaders, buffers, the whole thing)
- a first attempt at an "engine" with no chunks and no threads — it lagged badly

This is where I tried to fix the things that were wrong with that last one.

The main questions I wanted to answer:
- How do you separate simulation and rendering into different threads without
  getting races?
- How do you write a thread pool that actually works?
- Does the GPU really make it faster, and by how much?

---

## What it can do

- The world is split into chunks — only live/visible ones are simulated
- Simulation runs in parallel (custom thread pool), rendering in a separate
  thread — the picture doesn't stutter when the simulation is heavy
- The automaton rule is stored as a lookup table, not hardcoded — so the same
  rule works on both CPU and CUDA without rewriting anything
- CUDA backend uses shared memory tiling to reduce global memory reads
- Tried to implement CUDA-GL interop (writing results directly into the GL
  vertex buffer to skip the PCI-E round trip) — it falls back gracefully on
  Windows WDDM where GL context threading gets in the way
- Loads `.rle` pattern files — the standard format from conwaylife.com

---

## Benchmark

RTX 3060 Ti, chunk 1024×1024, 200 iterations, Release:

| Backend | Speed          |
|---------|----------------|
| CPU     | 281 Mcells/s   |
| CUDA    | 1909 Mcells/s  |
| Speedup | **6.8×**       |

---

## Build

Windows, Visual Studio 2022, CMake + Ninja. GLFW/GLAD/GLM are in `libs/`,
nothing to install separately.

```bash
cmake --preset x64-release
cmake --build out/build/x64-release
```

CUDA is optional — without it the project builds CPU-only. CMake prints which
one it picked:
```
-- CUDA found – GPU simulation backend enabled
-- CUDA not found – building CPU-only simulation backend
```

---

## Demos

```
Demo_minimallife_demo   150 Gosper Glider Guns loaded from RLE, runs on GPU
Demo_fractal_demo       Mandelbrot set through the same chunk pipeline
Demo_benchmark_demo     CPU vs GPU throughput, no window
Demo_life_test          Correctness tests — exit 0 = all pass
```

The fractal demo exists mostly to prove the engine isn't hardwired to Game of
Life — any chunk-based data source works.

---

## Tests

`Demo_life_test` is headless and checks:
- block still-life doesn't change
- blinker oscillates with period 2
- glider moves to the correct position after 4 steps
- CPU and CUDA produce byte-identical output after 100 steps on the same input

All 9 pass on both backends.

---

## What's not done yet

- No pan/zoom with the mouse — camera is fixed
- No generation counter or FPS display
- CUDA-GL interop currently falls back to the regular path on WDDM (the
  simulation runs on worker threads where the GL context isn't current)
