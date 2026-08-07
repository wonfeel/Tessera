# WORM_V2_MECHANOSENSATION_JOINT_RESULTS.md - joint mechanoGain/localMechanoGain search, honest report

Status: REPORT ON A COMPLETED SEARCH (not a design doc - see `WORM_V2_DESIGN.md`
and `WORM_V2_RESULTS.md` section 12.4 for the motivating context). Written
after triage across six parallel shards plus targeted follow-up and 16-base
confirmation passes, on this machine (`H:\workspace\projects\Tessera`, branch
`worm-v2-architecture`). Same convention as `WORM_V2_RESULTS.md`: positive and
negative results reported with equal care, seed bases stated honestly, no
marketing language.

---

## 1. What was searched and why

`WORM_V2_RESULTS.md` section 5 established that, by construction, `freqHz`
(the zero-crossing frequency of curvature deviation) is structurally
independent of medium in the current v2 stack: `WormBody::step()`'s angle
update reads only the network's `curvature[]` output, never `drag_tangent_`/
`drag_normal_`. The only two channels through which the medium could still
feed back into frequency emergently are `Params::mechanoGain` (drives the DVA
stretch-receptor neuron: `net.set_input(m_dva, gain*normalizedLoad+noise)` in
`WormSim.cpp::applyMechanosensation`) and `Params::localMechanoGain` (a direct
additive term inside `applyProprioception`'s feedback sum). Both ship at
`0.0` and have never been calibrated - they are the last channel in the
project that could plausibly restore a realistic water/agar frequency ratio
without reintroducing the removed, non-biological `cpgGain` rhythm generator.

Hand exploration earlier in this project (section 12.4 of
`WORM_V2_RESULTS.md`, ~30 manually-checked points) had already found:

- `mechanoGain` alone hard-locks water to exactly 0.0000 Hz once gain rises
  above roughly 0.1 (the same "hard lock" failure mode documented elsewhere
  in the project on this axis), but at very small gain (0.02-0.05) water
  oscillates at a genuinely different frequency than agar (ratio 1.01-1.14)
  without freezing - it just fails on path efficiency (0.16-0.34 vs. the 0.40
  floor).
- `localMechanoGain` alone is a much bigger, more promising effect: at
  0.05-0.2, water's frequency jumps to 0.35-0.62 Hz, giving a freq ratio of
  3.3-4.5x - the closest this project has ever come to the real ~2-6x target
  without `cpgGain`. Agar can be fully healthy (4/4) at these settings. But
  water was 0/4 healthy at every single value tried (0.02-2.0), always
  failing specifically on efficiency, never on coiled ratio or heading
  (both of which were *better* than agar's at the same point).
- One small combined point (`mechanoGain=0.02, localMechanoGain=0.1`) got
  water to 1/4 healthy for the first time, with a good ratio (4.377) - a
  hint, not a result, that the joint region might unlock something neither
  channel gives alone.

That single hint - and the large, unexplored joint region around it
(`mechanoGain` ~0.01-0.03 x `localMechanoGain` ~0.08-0.15) - is what this
search was launched to resolve: is there a point in the joint
`mechanoGain`/`localMechanoGain` space (optionally combined with nearby
`dragSettleGain`/`bodyGain`/`bodyPoseDecayRate` re-tuning) where **both**
media are simultaneously health-gate healthy, with a realistic frequency
ratio and without a collapsed absolute speed?

---

## 2. What each screen region found, in aggregate

Six shards plus this session's own supplementary sweeps ran roughly 470
point-evaluations at 4-base screen scale, covering:

- `mechanoGain` in [0, 0.1]
- `localMechanoGain` in [0.02, 0.35]
- `dragSettleGain` in [5, 150] (re-tuned jointly with the above, not just
  held at the shipped 20)
- `bodyGain` in [180, 220]
- `bodyPoseDecayRate` in [0.5, 0.7]

The pattern is uniform across every shard and every sub-region, with no
exceptions found anywhere:

- **Agar is easy to keep healthy.** Once `localMechanoGain` climbs above
  roughly 0.12-0.15, agar reaches 3/4-4/4 at screen scale (12/16-15/16 at
  16-base confirmation) almost regardless of the other axes tried.
- **Water's frequency ratio lands in the biologically correct band (2.5-5x)
  almost everywhere in this region** - this part of the original hand
  exploration replicates cleanly at scale. Water's coiled-ratio and heading
  metrics are consistently as good as or better than agar's at the same
  points, confirming this is not a raw-instability problem.
- **Water's path efficiency is the one metric that essentially never
  sustains above the 0.40 floor at confirmation scale.** Every screen-level
  "water healthy 2/4 or 3/4" hit found by any shard, when re-tested by either
  reseeding at 4 bases or expanding to 16 bases, collapsed to 0/16-4/16
  (0-25%) healthy - i.e., it was small-sample noise, not a real effect.
- Widening `dragSettleGain` away from the shipped anchor of 20 (both up to
  150 and down to 0) did not help and in most cases made things worse -
  consistent with the hand-exploration finding already on record that 20 is
  a reasonable anchor for this axis.
- Varying `bodyGain` (180-220) and `bodyPoseDecayRate` (0.5-0.7) around the
  shipped values did not open a healthy-water point anywhere either; these
  axes shift the agar/water balance slightly but do not touch the underlying
  water-efficiency ceiling.

No region tried - mechanoGain alone, localMechanoGain alone, the two
combined, or either combined with a re-tuned dragSettleGain/bodyGain/
bodyPoseDecayRate - produced a joint-healthy point that survived scaling.
This rules out the hypothesis that the earlier hint
(`mechanoGain=0.02, localMechanoGain=0.1`, water 1/4) was a real, small,
underexplored pocket of joint health; at the sampling density used here, no
such pocket was found nearby or elsewhere.

---

## 3. Confirmation results

Per the search protocol, every screen-level "hit" that looked promising was
re-checked either by reseeding at the same 4-base scale or by expanding to a
16-base (or wider) confirmation with a fresh, explicit `baseRngSeed`
distinct from the shipped default (31337) and from every other trial's seed.

| # | Screen hit (params) | Screen result | Confirmation | Confirmed result | Verdict |
|---|---|---|---|---|---|
| 1 | mechanoGain=0.005, localMechanoGain=0.15 | agar 4/4, water 3/4, ratio 3.32 | reseed (x2) + 16-base | water 0/4 (twice); 16-base: agar 13/16, water 2/16, ratio 3.66 | Collapsed - winner's-curse noise |
| 2 | localMechanoGain=0.16, bodyGain=180, mechanoGain=0 | screen hit (shard 6) | 16-base | agar 12/16, water 2/16, ratio 3.58 | Collapsed |
| 3 | localMechanoGain=0.16, bodyGain=220, mechanoGain=0 | screen hit (shard 6) | 16-base | agar 15/16, water 0/16, ratio 3.21 | Collapsed |
| 4 | mechanoGain x localMechanoGain corner, 0.02-0.1 x 0.1-0.2 (shard 4's unfinished cells) | partial screen, incomplete | completed (15 trials) | 0/15 water-healthy | No candidate emerged |
| 5 | mechanoGain x localMechanoGain corner, 0.005-0.04 x 0.05-0.2 (shard 1's unfinished cells) | partial screen, incomplete | completed | only hit #1 above (already collapsed); otherwise water capped at 0-1/4 | No candidate emerged |
| 6 | Full grid, mechanoGain 0-0.05 x localMechanoGain 0.15-0.35 (shard 2, 55 trials) | best: water 1/4 | n/a - never exceeded 1/4 in 55 trials | - | No candidate to confirm |
| - | Shard 3 / shard 5 best hits (reported independently) | agar 7-15/16 | already 16-base | water only 1-4/16 paired at same points | Collapsed |

**Triage's own conclusion, reproduced verbatim for the record:** the triage
report's final line reads `CANDIDATE: none - no genuinely promising
candidate survived reseed/16-base confirmation in any of the six explored
regions.` No numbered candidate list was ever produced by the triage
pass, because nothing cleared triage's own bar for "worth numbering as a
candidate." Six independent confirmation sub-passes were subsequently
requested against that report; all six correctly report that there is no
"CANDIDATE 1" through "CANDIDATE 6" to run, because the report never
enumerated any - it explicitly closed with "none." No further executable
runs were performed in this final stage, because there was nothing left
unconfirmed: every screen hit that triage flagged as even provisionally
interesting had already been reseeded or expanded to 16 bases before this
stage began, and all of them are captured in the table above.

---

## 4. Bottom line

**No.** There is not, as of this search, a confirmed, healthy-on-both-media
candidate with a realistic frequency ratio anywhere in the
`mechanoGain`/`localMechanoGain` joint space (nor in the neighboring
`dragSettleGain`/`bodyGain`/`bodyPoseDecayRate` re-tunings tried alongside
it).

Every point that looked promising at screen scale (4 bases) either:

- collapsed on reseeding or 16-base expansion to water healthy fractions of
  0-4/16 (table above), or
- never exceeded 1/16-1/4 water-healthy at any scale in the first place.

Where water's frequency ratio does land in the realistic 2-6x biological band
(which happens over a fairly wide swath of `localMechanoGain` >= ~0.12), water
is not healthy - it fails specifically and consistently on path efficiency,
never on coiled ratio or heading (both of which are typically *better* than
agar's at the same point). This is exactly the same failure mode already on
record from the original hand exploration; the joint search neither escaped
it nor found a smaller pocket where it doesn't apply. This is a clean,
complete negative result on the specific question asked ("does the joint
space unlock a point neither channel gives alone?") - not an inconclusive or
partial one.

---

## 5. No shipped defaults were changed

`WormSim.h`'s `Params` defaults were not modified as part of this search.
`mechanoGain` and `localMechanoGain` remain at `0.0`, exactly as shipped
before this work began. This was a search-and-report task only; whether to
ship any point found here (none is being recommended) is a decision for the
project owner, not this session.

---

## 6. Concrete next steps

1. **The blocker is water's path efficiency specifically, not frequency,
   shape, or heading.** Every region tried shows the same signature: water's
   coiled ratio and heading are as good as or better than agar's, and its
   frequency is genuinely medium-dependent and in the right biological band
   - but net displacement over path length stays low. This points at a
   phase-coherence / traveling-wave problem at low drag: the body may be
   oscillating "in place" (good local curvature dynamics, poor coordination
   of the wave's spatial phase along the body) rather than propagating a
   coherent wave that translates into forward motion once drag is low
   enough that the wave's own reaction forces stop constraining it. This is
   a hypothesis suggested by the pattern across shards, not something
   directly instrumented in this search - it was not diagnosed at the level
   of individual segment phases or wave-speed measurements.
2. **Unexplored corners:** the joint search covered `mechanoGain` up to 0.1
   and `localMechanoGain` up to 0.35; it did not extend to
   `localMechanoGain` above 0.35, nor to negative or antagonistic sign
   combinations of the two gains (e.g. `mechanoGain` negative to counteract
   an over-amplified stretch signal specifically in low-drag water). Neither
   was tried.
3. **A direct instrumentation step**, analogous to `WORM_V2_RESULTS.md`
   section 10's `trace` mode, would be the natural next move before further
   blind sweeping: log per-segment curvature phase along the body length in
   agar vs. water at a point like `localMechanoGain=0.2` (agar 4/4 healthy,
   water 0/4, always on efficiency) and check directly whether the traveling
   wave's spatial phase gradient collapses or reverses in water relative to
   agar. If it does, that confirms the phase-coherence hypothesis above and
   points at a concrete, non-blind fix (e.g. a phase-locking or
   wave-coordination term) rather than another parameter sweep.
4. Per section 12.4 of `WORM_V2_RESULTS.md`, `mechanoGain`/`localMechanoGain`
   remain the only channel in the project that could restore water/agar
   frequency medium-dependence without reintroducing `cpgGain`. This search
   shows that channel alone (or jointly with nearby anchors) is not
   sufficient to also fix water's path efficiency - the two problems
   (frequency realism and path-efficiency realism) appear to be separable,
   and solving one does not solve the other.
