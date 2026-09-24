# Sawtooth support interface — implementation plan

## 1. What this repo has today

### Support interface generation

Three entry points produce support toolpaths; only two of them emit user-visible interface
extrusions:

| Generator | File | Interface filler |
|---|---|---|
| Classic supports **and** organic tree (`smsTreeOrganic`) | [SupportCommon.cpp:1421](../src/libslic3r/Support/SupportCommon.cpp) `generate_support_toolpaths` | `support_params.contact_fill_pattern` |
| Legacy tree (slim / strong / hybrid) | [TreeSupport.cpp:1502](../src/libslic3r/Support/TreeSupport.cpp) | `support_params.contact_fill_pattern` |
| TreeSupport3D area planning | [TreeSupport3D.cpp:656](../src/libslic3r/Support/TreeSupport3D.cpp) `generate_support_infill_lines` | `interface_fill_pattern` — **planning only**, its polylines never reach G-code |

The pattern is chosen once in
[SupportParameters.hpp:140](../src/libslic3r/Support/SupportParameters.hpp), mapping
`support_interface_pattern` (`smipAuto / Rectilinear / Concentric / RectilinearInterlaced / Grid`)
onto an `InfillPattern` in `contact_fill_pattern`. Angle per layer comes from
`SupportParameters::support_interface_angle()`.

Both live generators funnel through the same shape:

```
filler->fill_surface(surface, params)   ->  Polylines      (2D, no Z, no width variation)
extrusion_entities_append_paths(...)    ->  ExtrusionPath  (one mm3_per_mm / width / height per path)
```

— [SupportCommon.cpp:417](../src/libslic3r/Support/SupportCommon.cpp)
`fill_expolygon_generate_paths`, and TreeSupport.cpp's own copy at line 1230.

**The support path never calls `Fill::fill_surface_extrusion`.** That virtual exists
([FillBase.hpp:187](../src/libslic3r/Fill/FillBase.hpp)) and is how `FillSymmetricWave`,
`FillLockedZag` and `FillConcentricInternal` emit non-uniform geometry, but its only caller is
`Layer::make_fills` ([Fill.cpp:1448](../src/libslic3r/Fill/Fill.cpp)) — region fills, not supports.

### Per-point Z is already solved here

This fork carries Z-contouring machinery that upstream OrcaSlicer and SuperSlicer do not:

- `ExtrusionPath::polyline` is a **`Polyline3`** (`Points3`) already —
  [ExtrusionEntity.hpp:164](../src/libslic3r/ExtrusionEntity.hpp). Every path can carry per-point Z
  today; nothing new needs to be typed.
- `ExtrusionPath::z_contoured` flags a path whose Z is meaningful.
- [GCode.cpp:8176](../src/libslic3r/GCode.cpp) and :8387 emit `z_contoured` paths with
  `extrude_to_xyz`, and **auto-compensate flow**:
  `extrusion_ratio = (path.height + z_diff) / path.height`.
- [GCode.cpp:7501](../src/libslic3r/GCode.cpp) sets the entry Z, and :7523 restores nominal Z after a
  contoured path — self-healing, so a contoured support path cannot leave Z wrong for the next
  feature.
- `ExtrusionPath::simplify` and `simplify_by_fitting_arc` bail out on `z_contoured`
  ([ExtrusionEntity.cpp:33](../src/libslic3r/ExtrusionEntity.cpp)), and `split_at` propagates the
  flag — so teeth survive path post-processing.
- `Line3::length()` is 3-D, so ramped segments get correct E.

Role gating for Z-contouring lives in `ContourZ.cpp` (`zaa_*` config), **not** in `GCode.cpp`. The
G-code writer will happily emit a `z_contoured` path with role `erSupportMaterialInterface`.

### Variable width is already solved too

`FillSymmetricWave::fill_surface_extrusion`
([FillSymmetricWave.cpp:205](../src/libslic3r/Fill/FillSymmetricWave.cpp)) builds `ThickPolylines`
and calls `variable_width(...)` — the existing route for per-segment width, if it is ever needed
here.

## 2. The SuperSlicer sawtooth, reviewed

`FillRectilinearSawtooth::fill_surface_extrusion`
([refs/superslicer-sawtooth/FillRectilinear.cpp:3324](../refs/superslicer-sawtooth/FillRectilinear.cpp)).

What it does: run the normal rectilinear fill, then walk each resulting polyline and, at
pseudo-random intervals, splice in a **tooth** — a spike that lifts the nozzle so the interface
touches the object above at discrete points instead of along the whole line.

Per tooth it emits five `ExtrusionPath3D` segments:

1. vertical hop up at full nozzle-round flow (`d²·π/4`),
2. a `nozzle_diameter`-long move at the top with **zero** flow and a hairline width,
3. a descent at `mm3_per_mm / √2`,
4. back to normal flow.

Constants, all derived from nozzle diameter: `clearance = 2.5·d`, tooth spacing random in `[d, 3d]`,
`tooth_zhop = d`. Teeth are skipped on segments shorter than `clearance` and on the "return" legs
(a sign test on x or y depending on the fill angle).

### What does not port

- **`ExtrusionPath3D` / `ExtrusionMultiPath3D` do not exist here**, and we do not need them: a single
  `ExtrusionPath` with a `Polyline3` and `z_contoured = true` expresses the whole tooth. The only
  thing we give up is SuperSlicer's per-segment flow dance — and `GCode.cpp`'s `extrusion_ratio`
  already scales flow with Z, which is most of what that dance bought.
- **`rand()`** — called from inside a TBB-parallel region. That is a data race and makes slicing
  non-reproducible. Replace with a hash of the segment's start point (deterministic, lock-free, and
  still breaks up tooth alignment between rows).
- **Instantaneous vertical hops** (two points sharing XY, differing Z). They stall XY motion, print a
  blob, and make the 3-D preview degenerate. Use short ramps instead.
- **AGPL.** The refs tree is read-only reference (see
  [refs README](../refs/superslicer-sawtooth/README.md)). Implement the geometry from the description
  above; do not copy the function body.

## 3. Plan

The teeth are a **post-process on already-generated interface extrusions**, not a new infill pattern.
That keeps the whole feature out of the `Fill` registration surface (no `InfillPattern` enum value, no
`Fill::new_from_type` case, no `Layer::make_fills` case, no GUI infill pattern lists, no
`is_separable_infill_pattern`, no `use_bridge_flow` table entry) and out of `Fill.cpp` entirely.

### Step 1 — config

`PrintConfig.hpp` — append to `enum SupportMaterialInterfacePattern`:

```cpp
enum SupportMaterialInterfacePattern {
    smipAuto, smipRectilinear, smipConcentric, smipRectilinearInterlaced, smipGrid, smipSawtooth
};
```

`PrintConfig.cpp`:

- `s_keys_map_SupportMaterialInterfacePattern`: `{ "sawtooth", smipSawtooth }`.
- `support_interface_pattern` def (~line 6931): append `enum_values` `"sawtooth"` and `enum_labels`
  `L("Sawtooth")`.
- New option, the one calibration knob:

```cpp
def = this->add("support_interface_tooth_height", coFloat);
// 0 = auto: min(support_top_z_distance, nozzle diameter)
def->min = 0; def->max = 1.0; def->mode = comAdvanced;
def->set_default_value(new ConfigOptionFloat(0));
```

  Register in `PrintObjectConfig` (`PrintConfig.hpp` ~line 1135, next to `support_interface_pattern`)
  and on the Support page of `src/slic3r/GUI/Tab.cpp`.

Appending the enum value is 3mf/profile safe — serialization goes through the string key.

### Step 2 — pattern selection

`SupportParameters.hpp`:

- in the `contact_fill_pattern` chain (~line 140), add `smipSawtooth -> ipRectilinear` before the
  `smipAuto` branch, so sawtooth is plain rectilinear geometry plus the post-process.
- in `support_interface_angle()` (~line 300), give `smipSawtooth` the same case as `smipRectilinear`.

### Step 3 — the tooth post-process

New file pair `src/libslic3r/Support/SawtoothInterface.{hpp,cpp}` (add to
`src/libslic3r/CMakeLists.txt`, `Support/` block). One public function plus one testable helper:

```cpp
struct SawtoothParams {
    coord_t tooth_height;   // scaled Z rise at the tip
    coord_t spacing_min;    // scaled, along the path
    coord_t spacing_max;
    coord_t ramp;           // scaled run of each up/down ramp
    coord_t clearance;      // scaled; segments shorter than this get no tooth
};

// Rewrites `pl` into a Polyline3 with teeth. Pure geometry — unit testable.
Polyline3 add_sawtooth_teeth(const Polyline &pl, const SawtoothParams &p);

// Walks entities appended at or after `first_entity` and applies teeth in place.
void apply_sawtooth_teeth(ExtrusionEntitiesPtr &entities, size_t first_entity,
                          const SawtoothParams &p);
```

`add_sawtooth_teeth` walks the polyline accumulating arc length. On each segment longer than
`clearance`, at each tooth position it inserts four points by interpolation:

```
... ── base(z=0) ─ramp─ tip(z=h) ── flat ── tip(z=h) ─ramp─ base(z=0) ── ...
```

with the flat top one `ramp` long. All other points get `z = 0`. Tooth phase within a segment is
`hash(segment start point) % (spacing_max - spacing_min) + spacing_min` — deterministic, TBB-safe,
and still de-aligns teeth between adjacent rows. If the remaining segment length after a tooth is
less than `clearance`, stop teething that segment.

`apply_sawtooth_teeth` `dynamic_cast`s each entity (`ExtrusionPath`, `ExtrusionMultiPath`,
`ExtrusionEntityCollection`, recursing the way `ContourZ.cpp`'s `contour_extrusion_entity` does),
filters on `role() == erSupportMaterialInterface`, rewrites `path.polyline` and sets
`path.z_contoured = true`. A path whose rewrite produced no tooth is left untouched (no flag), so the
G-code stays on the fast 2-D path wherever possible.

Derivation of the params, in the caller:

```
gap = object_config.support_top_z_distance
h   = tooth_height > 0 ? tooth_height : std::min(gap, nozzle_diameter)
```

If `h <= EPSILON` (zero Z gap — soluble / zero-gap interfaces) the whole feature is a no-op: there is
nowhere to hop into. `spacing_min = d`, `spacing_max = 3d`, `ramp = 0.5d`, `clearance = 2.5d`, with
`d = support_material_interface_flow.nozzle_diameter()`, matching the reference's proportions.

### Step 4 — call sites

**SupportCommon.cpp**, inside the `extrude_interface` lambda (~line 1750): record
`layer_ex.extrusions.size()` before `fill_expolygons_generate_paths`, and after it call
`apply_sawtooth_teeth` when `support_params.support_interface_pattern == smipSawtooth` **and**
`interface_layer_type` is `TopContact`, or `Interface` whose `layer_type` is not `BottomInterface`.
Bottom contacts, raft contacts and `InterfaceAsBase` are excluded — teeth point up, at the object.

**TreeSupport.cpp** (~line 1580): same two lines around the `RoofType` and `Roof1stLayer`
`fill_expolygons_generate_paths` / `make_perimeter_and_infill` calls. Skip `FloorType`.

Leave `TreeSupport3D::generate_support_infill_lines` alone — planning only.

### Step 5 — interactions to check

- **Ironing.** `support_params.ironing` irons the top contact surface
  ([SupportCommon.cpp:1646](../src/libslic3r/Support/SupportCommon.cpp)). Ironing a toothed surface
  will shear the teeth off. Force ironing off when `smipSawtooth` is selected, in
  `SupportParameters`, and say so in the tooltip.
- **Bridging flow.** Bottom contacts use `Flow::bridging_flow`; they are excluded anyway.
- **`support_top_z_distance == 0`** and soluble interfaces: no-op by the `h <= EPSILON` guard above.
- **Arc fitting / simplification**: already guarded by `z_contoured`.
- **Preview**: `GCodeProcessor` re-parses the emitted G-code, so teeth show up in preview with no
  extra work.
- **Spiral vase / `m_nominal_z`**: `m_nominal_z` is set per layer in `GCode.cpp:6820` and supports are
  extruded inside that layer, so the Z base is correct.

### Step 6 — verification

Geometry first, before any build (a cold build is hours — see
[Fill/AGENTS.md](../src/libslic3r/Fill/AGENTS.md) §6): a scratchpad script reimplementing
`add_sawtooth_teeth` and asserting tooth count, tip Z, and that XY is unchanged.

Then:

- `tests/libslic3r/test_support_material_sawtooth.cpp` (or folded into an existing file), tagged
  `[SupportMaterial]`: feed `add_sawtooth_teeth` a straight 50 mm polyline and assert
  (a) tooth count is within the spacing bounds, (b) every tip is exactly `tooth_height`,
  (c) all non-tooth points have `z == 0`, (d) the 2-D projection is unchanged,
  (e) a 1 mm polyline (< clearance) comes back with no teeth,
  (f) the same input twice gives byte-identical output (determinism / no `rand()`).
- `tests/fff_print/test_support_material.cpp`: slice a small overhang with
  `support_interface_pattern = sawtooth` and assert some interface path has `z_contoured == true`,
  and that with `support_top_z_distance = 0` none does.

```bash
cmake --build build --config RelWithDebInfo --target all --
```

```bash
ctest --test-dir build/tests -C RelWithDebInfo -R "Support|Fill"
```

## 4. Alternative considered and rejected

**Route the support interface through `Fill::fill_surface_extrusion`** (add `ipSawtooth`, a
`FillRectilinearSawtooth` subclass, and a branch in `fill_expolygon_generate_paths` that calls
`fill_surface_extrusion` when the filler asks for it).

This is how SuperSlicer and this repo's own `FillSymmetricWave` do it, and it would let the pattern
also be offered on `top_surface_pattern`. It costs: a new `InfillPattern` enum value and every
registration edit in [Fill/AGENTS.md](../src/libslic3r/Fill/AGENTS.md) §2 — including the
`use_bridge_flow` initializer, which iterates all `ipCount` values and throws on a missing factory
case — plus wiring `FillParams::flow` and `FillParams::extrusion_role` at both support call sites,
which are currently left default there. All of that to reach a pattern that is only meaningful on a
support interface. Revisit if sawtooth is ever wanted as a top-surface finish.

## 5. Diff summary

| File | Change |
|---|---|
| `src/libslic3r/PrintConfig.hpp` | `smipSawtooth`; `support_interface_tooth_height` in `PrintObjectConfig` |
| `src/libslic3r/PrintConfig.cpp` | enum key, GUI enum value + label, new option def |
| `src/libslic3r/Support/SupportParameters.hpp` | `contact_fill_pattern` + `support_interface_angle` cases; force ironing off |
| `src/libslic3r/Support/SawtoothInterface.{hpp,cpp}` | **new** — the whole algorithm, ~150 lines |
| `src/libslic3r/Support/SupportCommon.cpp` | 3 lines in `extrude_interface` |
| `src/libslic3r/Support/TreeSupport.cpp` | 3 lines each at `RoofType` / `Roof1stLayer` |
| `src/libslic3r/CMakeLists.txt` | new file pair |
| `src/slic3r/GUI/Tab.cpp` | expose `support_interface_tooth_height` |
| `tests/…` | two test files |

No changes to `Fill/`, `GCode.cpp`, `ExtrusionEntity.*`, or `ContourZ.cpp` — the Z-contour machinery
this fork already has does the emission.
