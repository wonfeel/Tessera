# demo/light

Light as a scalar wave field on a fixed hex grid: three independent
R/G/B fields, each solving the discrete wave equation `d^2h/dt^2 = c^2*lap(h)`
on its own lattice. Different wave speed per color gives chromatic
dispersion; a paintable "prism" region slows each color by a different
amount and bends/splits the beam the way real glass does.

## Where the idea came from

Inspired by [Artem Onigiri](https://github.com/ArtemOnigiri)'s
[Light-Simulation-JS](https://github.com/ArtemOnigiri/Light-Simulation-JS)
and his video ["Сделал СИМУЛЯЦИЮ ВОЛН СВЕТА на КЛЕТОЧНЫХ АВТОМАТАХ"](https://www.youtube.com/watch?v=noUpBKY2rIg) -
light as a cellular-automaton wave, not raytraced.

His `logic.js` (single 600x600 canvas, no libraries) runs the same core
loop this demo does:

- a height field per color channel, updated by a discrete Laplacian
  (`waveVelocity += (avg(4 neighbors) - height) * speed`)
- a "glass" region where `speed` differs per pixel, causing refraction
  and chromatic separation for free (no explicit raytracing/Snell's law -
  the wave equation does it)
- a long-exposure accumulation buffer (`accumulatedLight += |height| *
  EXPOSURE`, clamped and squared) instead of showing the instantaneous
  wave - this is what turns a jittery wavefront into the smooth "light
  painting" look in his demo

This project reuses that same accumulation trick (`Rendering > Accumulate`
below, on by default) and now shares its rendering register with the
default decaying glow. What's different here: a hex grid instead of square
(6 equidistant neighbors instead of 4, isotropic point sources without
square-lattice artifacts, see `LightField.h`), a proper chunk-sleep/wake
system for a much larger field, a phased-line-source Beam tool, CPU
multithreading via the engine's task scheduler instead of a single-threaded
canvas loop, and an adaptive accumulation rate: instead of a fixed
`EXPOSURE` on raw `|height|`, it accumulates the same avgSpeed-normalized
energy fraction that feeds the decaying glow, so saturation speed doesn't
depend on whatever `waveSpeedSq`/pluck strength happen to be dialed to.

## Controls

- **Pluck** - LMB click, one-shot amplitude impulse.
- **Brush** - LMB hold injects energy, RMB hold damps it.
- **Prism** - LMB paints the slow-medium region, RMB erases it.
- **Beam** - LMB press, drag to aim, hold to fire a phased directional pulse.
- WASD/scroll/MMB - camera.

## Rendering modes

- **Accumulate** (on by default) - the exact color formula from
  `logic.js`: `color = min(accum, 1)^2`, no ambient floor, and glass gets a
  flat additive tint (their `GLASS_COLORS`) instead of being darkened.
  Brightness never fades, only grows with total energy a node has seen. Use
  "Reset accumulation" to clear the trace without resetting the wave itself.
- Turn it off for the instantaneous view instead - this one's our own
  addition, not in the reference: a constant ambient floor plus decaying
  glow (Reinhard-style, see `LightField::step()`), with the prism darkened
  and tinted instead of glass-tinted.

## Maps

Built-in presets (`Maps` panel) drop a pre-shaped prism region into an
otherwise empty, reset field - no mouse painting required:

- **Prism (small)** / **Prism (large)** - the classic triangular prism, two
  sizes.
