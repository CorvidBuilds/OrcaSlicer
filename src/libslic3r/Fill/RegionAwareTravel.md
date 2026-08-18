# Region-aware travel optimization for irregular infills

Handoff / design spec for a future implementer. Target consumers today:
[FillSymmetricWave](FillSymmetricWave.cpp) and forced-order [FillScales](FillScales.cpp).
The same helper should stay useful for other decorative / oriented surface fills that
cannot freely reverse paths.

This is **not** a request to reuse Monotonic’s ant-colony code inside
`FillRectilinear`. That machinery is welded to the vertical hatch intersection
graph. What we want is the *idea* of Monotonic travel (ordered regions under a
partial order), expressed over ordinary oriented open paths.

## Problem

Decorative surface fills often need:

1. A fixed extrusion direction (left→right, or a consistent arc sweep), for
   surface quality — start-of-extrusion blobs and shear must land on the same
   side of neighbouring beads.
2. `no_sort() == true` (or an equivalent collection flag) so the G-code path
   planner cannot re-chain or reverse the result.

Together those force a naïve emission order: walk the lattice in generation
order, clip, emit. On a solid rectangle that is fine. On surfaces broken by
holes or concavities it repeatedly travels across gaps:

- **Symmetric Wave:** each row is clipped into several L→R fragments. Naïve
  order finishes every fragment of row `n` left-to-right, then row `n+1`, so
  the nozzle crosses the hole on every row.
- **Scales (Inward / Outward fill order):** each radius pass is clipped into
  many same-direction arcs. Naïve order walks the whole lattice once per
  radius, so a hole is crossed once per arc depth. Default Scales already
  calls `chain_or_connect_infill` and is out of scope for this work unless a
  caller opts into oriented chaining later.

Monotonic / Monotonic Lines avoid that by grouping hatch into contiguous
vertical **regions**, requiring left regions before right ones, then chaining
regions to cut travel. Irregular fills need the same *region* concept without
the rectilinear graph.

## What already exists (and why it is not enough)

| Facility | Location | Behaviour | Fit |
|---|---|---|---|
| `chain_polylines` | [`ShortestPath.cpp`](../ShortestPath.cpp) | Greedy NN; **may reverse** each polyline | Rejected by Scales forced-order (explicit comment). Wrong for Wave. |
| `chain_or_connect_infill` / `connect_infill` | [`FillBase.cpp`](FillBase.cpp) | Chain or stitch along boundaries | Used by Default Scales. May reverse / reconnect in ways that break forced orientation. |
| `reorder_by_shortest_traverse` | [`ShortestPath.hpp`](../ShortestPath.hpp) | Order by start points only | Ignores end→start of oriented open paths; no partial order. |
| `chain_extrusion_entities` | ShortestPath | Planner-level reordering | Defeated by `no_sort`. |
| `chain_monotonic_regions` + ant colony | [`FillRectilinear.cpp`](FillRectilinear.cpp) | Region order under left-before-right | Correct *idea*, not reusable API — tied to `SegmentedIntersectionLine`. |

**There is no generic “oriented region chain” helper today.** That is the gap.

## Goals

1. Add a small, fill-agnostic helper that reorders **pre-built oriented path
   groups** (“regions”) to reduce travel while:
   - never reversing an individual path (or only reversing when the caller
     explicitly marks a region as flippable as a unit — default off);
   - respecting a caller-supplied partial order (e.g. left strip before right,
     or “same depth before next depth”);
   - preserving path order *inside* a region (caller already sorted that).
2. Wire it into Symmetric Wave and forced-order Scales as the first clients.
3. Keep solid-rectangle behaviour effectively unchanged (same monotonic look;
   travels between consecutive full-width rows stay ~one row pitch at the
   ends — Monotonic pays the same cost).
4. Leave Default Scales on `chain_or_connect_infill` unless a deliberate follow-up
   opts it in.

## Non-goals

- Porting or calling `generate_montonous_regions` /
  `chain_monotonic_regions` from FillRectilinear.
- Serpentine (alternate R→L) rows — shorter travel, different surface product.
- User-facing settings. Hard-coded on for Wave; on for Scales only when
  `fill_order != Default`.
- Connecting fragments with extruded perimeter links (`connect_infill`). This
  work is travel *between* oriented groups, not merging geometry.
- Variable-width / Arachne specifics beyond accepting `Polyline` or a thin
  path handle type. Wave still builds `ThickPolyline` widths *after* order is
  fixed (or carries widths with the polyline — see Open questions).
- Optimizing gap fill order.

## Proposed abstraction

### Region

A region is a contiguous set of oriented open paths that must print as a block:

```
struct OrientedPathRegion {
    Polylines paths;          // already oriented; order inside is final
    // Optional metadata the chainer uses only for constraints / heuristics:
    BoundingBox bbox;         // union of paths
    Point start() const;      // paths.front().first_point()
    Point end()   const;      // paths.back().last_point()
};
```

Callers build regions. The helper only reorders the region list (and,
optionally later, chooses a flip of a *whole* region if `flippable` is set).

### Partial order

Caller supplies dependencies, same spirit as Monotonic’s `left_neighbors`:

```
// region[i] may not start until every region in preds[i] has finished.
std::vector<std::vector<size_t>> predecessors;
```

Examples:

- **Wave:** bucket clipped fragments into vertical strips by infill-frame `x`
  (see below). Strip `k` lists strip `k-1` as predecessor when their y-ranges
  overlap (or always, for a strict left-to-right sweep of strips).
- **Scales forced order:** one region per (lattice cell × current radius
  pass), or one region per lattice cell containing all arcs of that pass in
  paint order. Depth passes stay outer loops; within a pass, cells get
  left-before-right (or lattice NN under a weaker order). Minimum viable:
  within one radius pass, chain cells with no reverse; keep pass order.

### Chainer API (sketch)

Place next to ShortestPath (implementation) and declare from FillBase or a
small `OrientedPathChain.hpp` — prefer ShortestPath so Fill stays free of a
new dependency cycle.

```
// Reorder regions to cut end→start travel while honouring predecessors.
// Never reverses paths inside a region.
// Empty predecessors ⇒ pure greedy among remaining regions (still no reverse).
std::vector<size_t> chain_oriented_regions(
    const std::vector<OrientedPathRegion> &regions,
    const std::vector<std::vector<size_t>> &predecessors,
    const Point *start_near = nullptr);
```

Algorithm (v1 — keep it boring):

1. Ready set = regions with no unsatisfied predecessors.
2. Pick next = argmin distance(current_end, candidate.start) among ready
   (tie-break: stable index). Optional: allow end→end only if the caller marked
   the region flippable (v1: do not flip).
3. Append region, update current_end = region.end(), unlock successors.
4. If ready set empties with regions left, the predecessor graph has a bug —
   assert in debug, fall back to original order in release.

v1 does **not** need ant colony. Monotonic uses ants because the region graph
is large and perimeter-link lengths are asymmetric. Wave/Scales region counts
are small (strips × islands). Upgrade to pheromone only if a profile shows a
real gap; leave a `ponytail:` comment naming that ceiling.

Flatten:

```
Polylines flatten(const std::vector<OrientedPathRegion> &regions,
                  const std::vector<size_t> &order);
```

### Invariants the helper must document

- Input paths are non-empty; regions are non-empty.
- Predecessor indices are in range and the graph is a DAG.
- Output permutation is total: every region appears once.
- No polyline is reversed.
- `no_sort` remains the caller’s responsibility on the
  `ExtrusionEntityCollection`.

## Client: Symmetric Wave

Current behaviour ([FillSymmetricWave.cpp](FillSymmetricWave.cpp)):

- Clip **per row** (so Clipper cannot scramble bottom→top).
- `sort_row_ltr` + `orient_ltr` per fragment.
- Append in row order; `no_sort() == true`.

Desired:

1. Still clip per row and orient L→R.
2. Assign each fragment a strip id from infill-frame geometry, e.g.

   ```
   x_mid = 0.5 * (first.x + last.x)   // after rotate into infill frame
   strip = floor((x_mid - origin_x) / strip_width)
   ```

   Start with `strip_width ≈ few * P` (one or two periods) or adaptive: cluster
   fragments whose x-ranges overlap into connected components per “column”.
   Prefer **overlap connectivity** over a fixed pitch — holes define strips
   naturally.

3. Build one region per (strip × contiguous run of rows), or simpler v1:
   **one region = all fragments of one strip, bottom→top, each row L→R**.
4. Predecessors: strip `s` depends on strip `s-1` when their y-ranges overlap
   (Monotonic left-before-right). Disjoint y-ranges may be independent
   (parallel “channels”) — optional v1 simplification: always depend on
   `s-1` for a total left-to-right strip order.
5. `chain_oriented_regions` → flatten → `to_thick` / `variable_width` as now.

Tests to extend in `tests/fff_print/test_fill.cpp`:

- Existing monotonic L→R / bottom→top checks stay.
- New: surface with a central hole; count long travels (end→start distance
  greater than ~half the hole width) and assert the chained order produces
  fewer than the naïve row-major order (or assert strip completion: all
  fragments of strip 0 appear before strip 1 when y-ranges overlap).

## Client: Scales (forced fill order only)

Current behaviour ([FillScales.cpp](FillScales.cpp)):

- Default: `chain_or_connect_infill` (unchanged by this project).
- Inward/Outward: per-radius pass, orient arcs, append; **deliberately not
  chained** because `chain_polylines` reverses.

Desired for forced order:

1. Keep per-radius outer loop (depth order is the product feature).
2. Inside one pass, group arcs into regions by lattice cell (centre `x,y`
   before clip — clip per cell or tag by nearest lattice point after clip).
3. Orient as today (`sweeps_ccw` fixups).
4. Chain cells inside the pass with `chain_oriented_regions`. Predecessors:
   optional left-before-right in infill frame; or empty preds + greedy NN
   with **no reverse** (already a win vs lattice walk). Prefer empty preds
   for v1 unless a visual artefact shows up.
5. Still never call `chain_polylines` on these paths.

Tests: forced-order Scales on a holed square; all arcs still same sweep
direction; travel sum decreases vs current lattice order; depth order of
radii unchanged.

## Suggested file layout

```
src/libslic3r/ShortestPath.hpp/.cpp     # chain_oriented_regions (+ types)
  or src/libslic3r/Fill/OrientedPathChain.hpp/.cpp
src/libslic3r/Fill/FillSymmetricWave.cpp  # build strips → call helper
src/libslic3r/Fill/FillScales.cpp         # forced-order path only
tests/fff_print/test_fill.cpp             # Wave + Scales cases
tests/libslic3r/test_oriented_path_chain.cpp  # unit DAG / no-reverse pins
```

Prefer a dedicated unit test file with synthetic polylines (no full Fill) so
the DAG / greedy logic can fail fast without slicing.

CMake: list new `.cpp` next to ShortestPath or in the Fill block of
`src/libslic3r/CMakeLists.txt`.

## Implementation plan (checklist)

1. Spec sign-off on strip definition for Wave (fixed pitch vs overlap
   clustering) and Scales cell tagging after clip.
2. Implement `OrientedPathRegion` + `chain_oriented_regions` + unit tests
   (empty preds, chain preds, assert no reverse, assert permutation).
3. Wave: build regions, wire helper, keep `no_sort`, extend Fill tests with a
   holed surface.
4. Scales forced-order: wire helper inside each radius pass; leave Default
   alone.
5. Profile a large holed top surface; confirm travel drop and no direction
   regressions in a width- or direction-colorized preview.
6. Doc touch-ups in this file’s Status section and any pattern handoff notes.

## Open questions

1. **Strip construction for Wave:** fixed multiples of `P` vs connected
   components of x-overlapping fragments. **Resolved:** neither — use one
   fragment per region with an x-interval frontier predecessor DAG (plus
   same-row L→R). Overlap clustering is wrong because full-width rows
   reconnect both sides of a hole.
2. **Flip whole regions?** Monotonic flips zig-zag strips as a unit. Wave
   strips that are single-direction rows gain little from flipping (start and
   end sit on opposite sides). Default off.
3. **ThickPolyline timing for Wave:** chain `Polyline`s then `to_thick`, or
   chain after thickness? Prefer chain before `to_thick` so the helper stays
   geometry-only.
4. **Should Default Scales ever use this?** Only if someone wants oriented
   chaining without boundary stitches; not required for v1.
5. **Ant colony later?** Only if greedy region NN leaves obvious long
   jumps on real models; measure first.

## Status

- Implemented. Helper lives in `ShortestPath.hpp` / `ShortestPath.cpp`
  (`OrientedPathRegion`, `chain_oriented_regions`, `flatten_oriented_regions`).
- **Wave:** one oriented fragment per region; same-row L→R edges plus an
  **x-interval frontier DAG** for bottom→top (not fixed pitch, not global
  overlap clustering — full-width rows would merge both sides of a hole into
  one component). Chain before `to_thick`. `no_sort` unchanged.
- **Scales forced order:** per-radius pass, one region per oriented clipped
  arc, empty predecessors, greedy no-reverse chain. Default still uses
  `chain_or_connect_infill`.
- Unit tests: oriented-region cases in `tests/libslic3r/test_geometry.cpp`
  (`[OrientedPathChain]`). Fill coverage: holed Wave + forced-order Scales in
  `tests/fff_print/test_fill.cpp`.
- v1 is greedy NN only (`ponytail:` on the helper); no whole-region flips.
## References

- Monotonic entry points: `FillMonotonic::fill_surface`,
  `FillMonotonicLines::fill_surface` → `params.monotonic = true` →
  `fill_surface_by_lines` branch around `generate_montonous_regions` /
  `chain_monotonic_regions` in `FillRectilinear.cpp`.
- Scales refusal of reversing chainers: comment above the forced-order append
  loop in `FillScales.cpp`.
- Wave monotonic direction: `orient_ltr` / `sort_row_ltr` /
  `no_sort()` in `FillSymmetricWave.*`.
- Existing free chainers: `ShortestPath.hpp`.
