# Tessera

![C++](https://img.shields.io/badge/C%2B%2B-17-blue?style=flat)
![CUDA](https://img.shields.io/badge/CUDA-optional-76b900?style=flat)
![OpenGL](https://img.shields.io/badge/OpenGL-4.6-white?style=flat)
![Platform](https://img.shields.io/badge/platform-Windows-0078d7?style=flat)
![License](https://img.shields.io/badge/license-MIT-green?style=flat)

**[Русский](README_RU.md)**

A 2D cellular-automaton engine in C++/CUDA/OpenGL. The world is chunked — only live
chunks are simulated, gliders cross chunk boundaries correctly, simulation and rendering
run on separate threads. Built to actually understand threads, thread pools, CUDA, and
OpenGL — not just read about them.

---

## Previews

**Gosper gun period 30:**

![gun + eater](assets/gun_eater.gif)

**Random 64×64 starting field — shows how life spreads into neighbouring chunks:**

![random field](assets/random_field.gif)

**Capture UI — ImGui interface for configuring and launching headless GIF exports:**

![capture ui](assets/interface.png)

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

- The world is split into chunks — only live chunks are simulated, and life
  spreads into neighbour chunks as it reaches their borders (gliders cross chunk
  boundaries correctly).
- Simulation runs in parallel on a custom thread pool; rendering runs on its own
  thread — the picture doesn't stutter when the simulation is heavy.
- The automaton rule is a lookup table, not hardcoded — the same rule runs on
  CPU and CUDA without rewriting anything.
- CUDA backend uses shared-memory tiling to cut global-memory reads.
- Tried CUDA-GL interop (writing results straight into the GL vertex buffer to
  skip the PCI-E round trip) — it falls back gracefully on Windows WDDM, where
  the GL context lives on a different thread.
- Loads `.rle` pattern files (the standard format from conwaylife.com) and you
  can pick any of them from the in-app panel.
- Interactive editing: draw/erase cells, pan, zoom, pause/step, change speed, and
  record a region of the field straight to a GIF.

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

Conway's rule, RTX 30-series, one chunk, 100 iterations. The GPU only pulls ahead
once the field is big enough to hide the kernel-launch overhead:

| Chunk size | CPU          | CUDA           | Speedup |
|------------|--------------|----------------|---------|
| 256²       | 37 Mcells/s  | 351 Mcells/s   | 9.5×    |
| 512²       | 35 Mcells/s  | 1230 Mcells/s  | 35×     |
| 1024²      | 37 Mcells/s  | 2366 Mcells/s  | 64×     |
| 2048²      | 37 Mcells/s  | 4034 Mcells/s  | 109×    |

Run it yourself: `Test_benchmark <chunkSize> <iterations>`.

---

## Build

Windows, Visual Studio 2022, CMake + Ninja. GLFW / GLAD / GLM / ImGui are vendored
in `libs/`, nothing to install separately.

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

## Demos

```
Demo_life_full      Full interactive Game of Life: draw, pan/zoom, pause/step,
                    pick patterns, record GIFs (ImGui panel).
Demo_life_minimal   The smallest possible setup — a randomized field, nothing else.
```

Controls (full demo):

```
LMB        draw cells           Space      pause / resume
RMB        erase cells          Step >     one step while paused
MMB drag   pan                  WASD       pan with the keyboard
Scroll     zoom to cursor       slider     simulation speed
```

The left panel also has a pattern picker (any `patterns/*.rle`), Clear / Randomize,
and a GIF recorder (select a region, hit record).

---

## Capture GIFs

**Option 1 — GUI (`Demo_life_capture_ui`):**
Run `Demo_life_capture_ui.exe` — an ImGui window where you pick the scene,
set resolution, output path, and hit **Capture!**. The default output path
is `%USERPROFILE%\Pictures\Tessera\capture.gif`.

**Option 2 — command line:**
```bash
# gun + eater scene -> Pictures\Tessera\capture.gif
python tools\capture_gif.py --exe out\build\x64-release\Test_capture.exe ^
    --scene guns --stop 60 --res 600x360 --region 14 14 80 60

# random field
python tools\capture_gif.py --exe out\build\x64-release\Test_capture.exe ^
    --scene random --stop 40 --res 320x320 --region 0 0 80 80 --out random.gif

# single glider
python tools\capture_gif.py --exe out\build\x64-release\Test_capture.exe ^
    --scene glider --stop 60 --res 420x420 --region 0 0 35 35 --out glider.gif
```

Key flags:

| Flag | What it does |
|------|--------------|
| `--scene` | `guns` (default), `random`, `glider` |
| `--stop N` | how many steps to capture |
| `--res WxH` | output resolution in pixels |
| `--region X0 Y0 X1 Y1` | grid region to show (tile coords) |
| `--guns GX GY` | GX×GY grid of Gosper guns (scene=guns only) |
| `--delay ms` | ms between GIF frames |
| `--out path` | output path (default: Pictures\Tessera\capture.gif) |

Every step is captured (stride = 1 — not configurable). Requires Pillow: `pip install pillow`.

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

---

## What's not done yet

- CUDA-GL interop still falls back to the regular copy path on WDDM (the
  simulation runs on worker threads where the GL context isn't current).
- Only 2-state, totalistic "life-like" rules so far (no multi-state automata).
- The world is a fixed size, not truly infinite.
