# demo/light

`LightField` — a scalar wave field on a hex grid, solving the discrete wave
equation `d^2h/dt^2 = c^2*lap(h)` per color channel. `LightField.h/.cpp`
live here because the engine's regression/benchmark tests
(`tests/light_field_wave`, `tests/light_field_benchmark`,
`tests/light_field_cuda_benchmark`, `tests/light_frame_profile`) include
them directly from this path.

`main.cpp` here is the smallest possible showcase: one field, click-to-pluck,
plain decaying-glow rendering, no UI.

The full interactive demo — chromatic R/G/B dispersion, paintable prism,
phased beam tool, ImGui controls, built-in prism presets — lives in its own
repository: [wonfeel/WaveLight](https://github.com/wonfeel/WaveLight)
(same `LightField` class, pulls this engine via `FetchContent`).
