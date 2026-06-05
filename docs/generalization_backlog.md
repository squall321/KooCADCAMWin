<!-- @lat: [[engine/reverse-route#generalization-backlog]] -->
# Generalization backlog — removing band-aids from the RE→adapt loop

Source: automated band-aid audit (2026-06-06) over the recover→adapt→re-execute
subsystems. Each item is a special-case / hardcoded assumption that works for
the current tests but fails the general real-world case. Ranked by payoff.

## Done

- **Datum matching → AABB containment, axis-aware** (`3be1d14`). Was
  "nearest part *centre* within 3D tolerance" (forced `tol=50` and faked `z=0`).
  Now a feature is owned by the smallest part whose AABB contains it on the axes
  the step specifies; omitted axes ignored.
- **Reframe → rigid-body (translation + rotation)** (`8286740`). Was
  translation-only; `Part.placement` was unused. Now features move through the
  owner part's full pose delta `f' = newCentre + ΔR·(f − oldCentre)`.

## High payoff (next)

- **Reframe: handle `resized` parts** — `PartsLayout::diff` emits "resized" but
  `reframePlanForMoves` ignores it. Scale a feature's offset-from-centre by the
  part's per-axis size ratio (and scale radius/depth params where present).
  *parts/DatumGraph.cpp.*
- **Datum extraction scans only position/center/offset fields** — features
  anchored via face datums (`entry_face_id`, `FaceByNormal`, …) or angle/path
  params are never linked to a part, so they are never adapted. Extend
  `extractHeuristicDependencies` to also read face/edge/angle anchors.
  *parts/DatumGraph.cpp:107.*
- **drill_hole entry-side + axis heuristic is Z-locked** — "higher Z = entry"
  and `abs(dot−1) < 1e-3` reject angled/side drills. Use the outward-surface
  circle as entry and a `cos(θ_max)` axis tolerance. *skills/drill_hole.cpp:133,259,286.*
- **chamfer_edge clusters by Z-midpoint** (`< 0.5 mm`) — fails on side-wall or
  tilted chamfers. Cluster along the bevel faces' dominant normal axis instead
  of world Z. *skills/chamfer_edge.cpp:346.*
- **Executor `parseFaceDatum` only knows "top"/"bottom"/id** — blocks arbitrary
  `FaceByNormal`/`FaceByRay`/`FaceTopAtXY`/`CylinderByAxis` from JSON, so tilted
  or non-axis entries can't round-trip. Parse the full datum-object form.
  *process/Executor.cpp:322; `parseCylinderAxisDatum` also hardcodes origin (0,0,0) at :409.*
- **Metadata-replay returns confidence 0.99 without checking geometry**
  (bolt_hole_metric_spec:377, tap_thread, ream) — trusts a possibly-stale
  signature. Validate recovered params against the shape before replaying;
  drop confidence on mismatch.

## Medium / low payoff

- Magic radius tolerances differ per skill (`1e-3` drill vs `1e-2` pocket) and
  don't scale — use `max(1e-4, 0.001·radius)` everywhere.
- counterbore `sameAxis`/junction tolerances (`0.5°`, `1e-3`, `1e-2`) are fixed
  — scale by bbox / feature size.
- bolt_hole ISO-273 match tol `0.15 mm` — widen to `0.25` for STEP imports;
  return ranked candidates not a single best.
- `Recognizer::dedupe` fingerprints by face-ID (unstable across STEP round-trip)
  — use a geometric fingerprint (skill_id + diameter + position).
- `inferProcessPlan` sets `depends_on = {}` (linear only) and `classify()` has
  no rules for the 110+ compound skills (all land in "Unknown") — build a real
  feature-family dependency DAG.
- recovered params omit `entry_face` (drill_hole/counterbore) — emit the entry
  datum so a re-executed plan targets the same face after topology change.
