# Project Overview

A Fusion 360 add-in for fitting primitive surfaces to mesh bodies. The primary use case is interactively reconstructing parts that are only available in exported STL/mesh format — selecting regions of the mesh and generating exact BRep surfaces (planes, cylinders, spheres, cones, etc.) that can be assembled into a solid via Boundary Fill.

## Goals

- Replicate and improve on the original [Reverse](https://github.com/phossystems/Reverse) add-in
- Remove all external dependencies (numpy, scipy) — pure Python only
- Extend surface type support beyond planes and cylinders
- Improve or replace the vertex selection mechanism
- Explore semi-automatic surface type detection from mesh normals/curvature

## v1.0 — Released

- Pure Python fitting algorithms (plane, cylinder) — Nelder-Mead + grid search, Graham scan convex hull
- Surface preview while picking — `executePreview` + `isValidResult=True`; fits once min pts reached (Plane ≥ 3, Cylinder ≥ 4)
- Expansion distance input — FloatSliderCommandInput 'expDist', 0–5 mm, 0.1 mm steps
- BRep output via `TemporaryBRepManager` + `BaseFeature` (parametric mode safe)
- Multi-surface table — add/remove rows, each row independently typed and vertex-selected
- Screen-space pixel radius vertex selection — `modelToViewSpaceTransform` bulk projection, shift-click deselect
- Custom graphics vertex highlighting — per-row point sprites (12 colours)
- Panel fallback — if `ParaMeshCreatePanel` not found, falls back to Scripts/Add-ins panel

## v2.0 — Released

- Cone fitting — Gauss-Newton 4-parameter solver; handles frustums, pointed cones, oblique cuts
- Sphere fitting — algebraic 4-parameter linear solve (no optimisation needed)
- Auto surface type detection — fits all types per row, picks lowest RMSE with absolute improvement threshold and engineering-prior evaluation order (plane → cylinder → cone → sphere)
- Fit result caching — skips redundant fitting when only expansion slider or row selection changes; invalidated on vertex or type change
- Outlier detection — per-point error flagging (mean + k·std with absolute floor); outlier vertices shown with distinct sprite
- Centroid centering — applied to all fitters for numerical stability when mesh is far from world origin
- Plausibility guards — cylinder, cone, and sphere fits rejected in Auto mode if implied radius exceeds 5× point-cloud scale
- Failed-fit sentinel — row label shows "failed" on degenerate manual fits; cache prevents re-running
- N hotkey — add new surface row without clicking the toolbar button
- Auto-select sole visible mesh body on command open
- Debug test mesh command (6 reference primitives, registered only when DEBUG=True)

## Next Release

### Planned

*(nothing currently planned)*

### Deferred

- Cursor circle (selection radius visualisation) — see `exploration/notes/custom-graphics.md`
- `CustomFeature` wrapper for timeline editability — see `exploration/notes/ui-design.md`
- Auto-trim/join surfaces where they intersect (`booleanOperation` on open sheets)
- Watertight detection → auto-convert to solid
- Semi-automatic surface type detection from face groups / mesh normals
- Torus fitting
- Batch reconstruction mode

## Implementation

### Algorithm Overview (from Reverse)

**Plane fitting** — searches over all possible plane normals (two spherical angles) via a coarse grid + Nelder-Mead refinement. For each candidate normal, the mean orthogonal offset is the direct solution. Residual is sum of squared deviations from the mean offset. Once the best normal is found, the boundary is the convex hull of points projected onto the plane, optionally expanded outward.

**Cylinder fitting** — same outer grid search over axis orientation (two spherical angles). For each candidate axis, projects points to 2D (collapsing along axis), runs a direct linear least squares circle fit (non-dimensionalized form). Best axis minimizes the 2D circle fit residual. Origin is mapped back to 3D; radius comes from the 2D fit. Extent of the cylinder along its axis comes from min/max projected distances.

**Vertex selection (old hack)** — uses a raw `mouseClick` command event handler. On each click, converts the viewport 2D click position to a 3D ray (perspective: camera-eye to click point; orthographic: ray parallel to view direction through click point). Selects all mesh vertices whose distance to that ray is below a configurable radius. Shift-click deselects. Custom graphics (point sprites) are used to highlight selected vertices during executePreview.

### Key external dependencies to replace

| Old | Replacement |
|-----|------------|
| `numpy` arrays + ops | Pure Python lists + math helpers |
| `numpy.linalg.lstsq` | Normal equations via Gaussian elimination |
| `numpy.linalg.norm` | `math.sqrt(sum(x**2 for x in v))` |
| `numpy.cross` | Explicit 3-component formula |
| `scipy.optimize.brute` | Explicit angle grid iteration |
| `scipy.optimize.fmin` | Custom Nelder-Mead |
| `scipy.spatial.ConvexHull` | 2D Graham scan |
| `CustomGraphicsCoordinates.create(np.asarray(...))` | Flat list of doubles |
| numpy model-space ray distance for vertex selection | `modelToViewSpaceTransform.asArray()` bulk transform in pure Python (screen-space px radius) |
| `displayMesh.nodeCoordinates` (Point3D array) | `displayMesh.nodeCoordinatesAsDouble` (flat double array) |

### Fusion 360 API notes

- `MeshBody.displayMesh` → `TriangleMesh` → `.nodeCoordinatesAsDouble` (flat x,y,z array, no numpy needed)
- `TemporaryBRepManager` for building BRep surfaces without touching the timeline
- `BaseFeature` needed for parametric design mode
- `customGraphicsGroups.addPointSet(coords, indices, ...)` for vertex highlighting

## Progress Log

### 2026-04-06 — UI design decided, table prototype validated

**UI design:** Single command with a `TableCommandInput` listing surfaces (number |
type dropdown | vertex count). One execution = one `CustomFeature` in the timeline
containing N surfaces. No auto-trim in v1 — Boundary Fill handles assembly.
Full design rationale and API constraints in `exploration/notes/ui-design.md`.

**Table prototype:** `exploration/scratch/FusionTesting/commands/tableTest/entry.py`.
Validated: table layout, add/remove rows, row selection, label renumbering after
delete. Key lessons:
- Input IDs must use a monotonically increasing counter — never reuse IDs after
  deletion or label numbers collide on re-add.
- Use `table.getInputAtPosition(row, col)` for position-based lookup; ID-suffix
  lookup breaks after rows are deleted.
- `SelectionCommandInput` cannot go inside a table (not needed — mouseClick is custom).
- `BoolValueInput` toolbar buttons require `''` as resource folder if no icon assets exist.

Next: resolve open graphics questions via exploration (see `exploration/notes/custom-graphics.md`), then implement fitting and BRep output on OK.

---

### 2026-04-06 — src/ bootstrapped: vertex selection + custom graphics wired into table UI

**`src/` is now live.** Structure:
```
src/
  SurfaceFitting.py              # entry point (run/stop)
  config.py
  lib/fusionAddInUtils/          # log, handle_error, add_handler, clear_handlers
  commands/fitSurfaces/entry.py  # main command
```

**What's implemented in `entry.py`:**
- `SelectionInput` for the mesh body — loads `nodeCoordinatesAsDouble`, applies world
  transform via `_get_world_matrix` (ported from original `GetRootMatrix`), stores flat
  double list in `_mesh_pts`; clears `hasFocus` so mouseClick fires freely
- `TableCommandInput` (3 cols: row# | type dropdown | vertex count), add/remove rows,
  monotonic ID counter — same mechanics as the validated tableTest prototype
- `mouseClick` handler using screen-space pixel radius (`modelToViewSpaceTransform.asArray()`);
  accumulates into `_row_verts[active_row]`; shift-click deselects
- `executePreview` rebuilds a `CustomGraphicsGroup` with one `CustomGraphicsPointSet` per
  row (distinct colors from `_ROW_COLORS`; `PointCloudCustomGraphicsPointType` = single pixel,
  no image asset required); group is deleted and rebuilt on each preview call
- `destroy` cleans up the graphics group

**Custom graphics notes:**
- Using `PointCloudCustomGraphicsPointType` (pixel dots) — visible but small.
  Upgrade path: add a `Resources/point.png` and switch to `UserDefinedCustomGraphicsPointType`
  for larger, more visible sprites.
- Graphics are rebuilt in `executePreview`, which fires after any input change.
  The `_update_status` call at the end of `mouseClick` sets `tbStatus.text`, which triggers
  `inputChanged` → `executePreview`. This is the same trigger pattern as the original.

**Not yet implemented:** fitting algorithms in `entry.py`; `execute` handler is a no-op.

---

### 2026-04-06 — Exploration complete, ready to build

**Fitting algorithms:** Pure Python plane + cylinder fitting validated in `exploration/scratch/pure_python_fitting.py`. All tests pass. Replaces numpy/scipy entirely.

**Vertex selection:** Tested in Fusion via `exploration/scratch/FusionTesting/`. No native mesh vertex filter exists (`MeshVertices`/`BRepVertices` throw; `Vertices` only highlights full mesh body). mouseClick approach confirmed working. Upgraded to screen-space pixel radius using `modelToViewSpaceTransform.asArray()` — 0.23ms/930 verts, ~25ms/100K verts, 0 pixel error vs per-vertex API calls. `nodeCoordinatesAsDouble` confirmed as the fast vertex access path.

---

### 2026-04-06 — v1.0 release prep

**Fitting + BRep output implemented** in `entry.py`. Both plane and cylinder fitting are live; `executePreview` with `isValidResult=True` commits the result directly.

**Pre-release fixes:**
- `config.py`: `DEBUG = False` — verbose logging no longer written to Text Command window
- Cylinder degenerate case: `fit_cylinder_to_points` now returns `(None, None, None, inf)` if `_fit_cylinder_on_axis` fails post-optimization; `_create_cylinder_body` guards and logs instead of crashing
- `start()` panel fallback: if `ParaMeshCreatePanel` not found, falls back to `SolidScriptsAddinsPanel` instead of silently dropping the command
- `_SHIFT_KEY = 0x2000000` constant replaces inline magic number
- Verbose dev logging stripped throughout (per-click, per-slider, per-row, graphics rebuild, destroy); fit results and error paths retained

---

### 2026-04-07 — Auto mode, fit result caching, and two stale-cache bug fixes

**Auto surface type:** Added "Auto" as a dropdown option. Fits all implemented types once per `executePreview` call and picks the lowest-RMSE result that passes the cylinder plausibility check (`radius ≤ _MAX_RADIUS_RATIO * point_cloud_scale`). No double-fitting — the winning params are used directly for BRep creation.

**Fit result cache:** `_row_fit_results` stores `verts_snapshot` (a `frozenset` of selected vertex indices), the fit `params`, RMSE, and outlier set. On each `executePreview`, if the snapshot matches the current vertex set the fit step is skipped and BRep creation proceeds directly from cached params. Eliminates redundant fitting when only the expansion slider, active row selection, or status text changes.

**Bug 1 — Type change not invalidating cache:**
The cache hit check only compared `verts_snapshot`. Changing the surface type dropdown (e.g. "Auto" → "Plane") with the same vertex set produced a false cache hit, reusing the old type's fit params until a new vertex was selected. Fixed by adding `type_name` to both the cache hit predicate and the stored entry.

**Bug 2 — Mesh deselection leaving stale RMSE labels:**
When the mesh body was deselected, `_row_verts` was cleared but `_row_fit_results` was not. RMSE values in col 0 of the table stayed visible until new vertices were selected from the next mesh. Fixed by nulling out `_row_fit_results[i]` in the deselection branch of `command_input_changed`, causing `_update_row_labels` to revert col 0 to row numbers immediately.

**Also fixed:** Manual test plan had the default type dropdown listed as "Plane" — it has always defaulted to "Auto". Corrected.

**Per-row vertex colour:** Implemented from the start using 12 pre-coloured PNG images
(`point_00.png`–`point_11.png`) — one image per row, cycling modulo 12. Outliers use a
separate `point_outlier.png`. `SolidColorEffect` does not tint image-based sprites (confirmed
dead end); the pre-coloured PNG approach sidesteps this entirely. See
`exploration/notes/custom-graphics.md`.

**Bug 2 correction:** The original fix nulled `_row_fit_results` in the deselect branch but
never actually cleared col 0 — `executePreview` returns early on `_mesh_pts is None` before
reaching `_update_row_labels`. Fixed by calling `_update_row_labels(table)` directly in the
deselect branch.

---

### 2026-04-13 — Cone fitting + centroid centering for all fitters

**Cone fitting implemented** (`fitting.py`, `entry.py`). Algorithm: algebraic 5-parameter
linearization — for a fixed axis, the cone condition `r_i = s·|t_i - t_apex|` is squared and
expanded into a linear system `H·x = y` (5 unknowns: A=2cx, B=2cy, C=s², D=2sb, E=b²-cx²-cy²).
Solved via 5×5 normal equations — O(n) per axis candidate, same cost structure as the algebraic
circle fit for cylinders. Outer optimization: 9×5 grid search + Nelder-Mead on spherical angles
(θ, φ), same pattern as cylinder/plane. Derivation in `exploration/notes/cone-fitting.md`;
validated in `exploration/scratch/cone_fitting.py`.

BRep construction uses `createCylinderOrCone(pt1, r1, pt2, r2)` (same API as cylinder, r1 ≠ r2).
The axis is oriented so data is on the positive side; a small epsilon floor prevents r=0 at the
narrow end from potentially being rejected by the API.

**Hourglass rejection:** If the fitted apex falls strictly inside the data's axial range (with
1% span tolerance for floating-point), the solution is a double-nappe (hourglass) — rejected with
cost=inf. Only single-nappe patches are returned.

**Centroid centering applied to all fitters** (`fit_plane_to_points`, `fit_cylinder_to_points`,
`fit_cone_to_points`, `fit_sphere_to_points`). A shared `_center_points(pts) → (cpts, centroid)`
helper subtracts the centroid before fitting and the caller shifts geometry back to world space.
This prevents large-magnitude columns (`xi²+yi²`, `t_i²`) from dominating the normal equations
when the mesh is far from the world origin. The pattern is reused by all fitters.

---

### 2026-04-13 — Debug test mesh command

**`Add Test Meshes` command** added (`src/Reverse/commands/addTestMeshes/entry.py`), registered only when `config.DEBUG = True`.

Generates six BRep reference solids and six matching mesh bodies in the active design:
- Sphere (r=20 mm)
- Full cylinder (r=15 mm, h=40 mm)
- Partial cylinder (~270°, one quadrant removed via boolean subtract)
- Flat plane slab (80×80×4 mm, select top-face vertices for plane fitting)
- Cone with apex (r=20 mm base, h=50 mm)
- Frustum / truncated cone (r=20 mm → r=10 mm, h=40 mm)

Bodies are spaced 100 mm apart along X. BRep solids (`Ref_*`) are the ground truth; mesh bodies (`Mesh_*`) are the inputs for Fit Surfaces. Both are created inside a single `BaseFeature` using `TemporaryBRepManager` shapes tessellated with `HighQualityTriangleMesh` and `meshBodies.addByTriangleMeshData`.

---

### 2026-04-13 — Sphere fitting

**Sphere fitting implemented** (`fitting.py`, `entry.py`). Algorithm: algebraic 4-parameter
linearization — the sphere condition `||p - c||² = r²` expands to a linear system
`y_i = A·px + B·py + C·pz + E` where y_i=px²+py²+pz², unknowns A=2cx, B=2cy, C=2cz,
E=r²-cx²-cy²-cz². Solved via 4×4 normal equations — **no axis to optimise over**, so
no grid search or Nelder-Mead needed. This is the simplest of all the fitters: a single O(n)
linear solve.

BRep construction uses `createSphere(center, radius)`. Expansion is not applied — a sphere is
a closed solid with no boundary edge to push outward. Auto mode applies the same
`_MAX_RADIUS_RATIO` plausibility guard as cylinder.

**Cone degenerate early-exit:** If the cone grid search finds no valid solution (all-inf costs),
Nelder-Mead is skipped entirely. Previously all 500 iterations would run with NaN comparisons
(inf - inf) and never converge, causing ~1–3s stalls on spherical geometry in Auto mode.

**Cone fitter: Nelder-Mead fallback to grid best:** After Nelder-Mead optimisation, the final
`_fit_cone_on_axis` call could return DEGENERATE if the optimiser drifted to an angle where C≤0
or the hourglass check triggers — even though the grid search found a valid starting direction.
The fitter now falls back to the grid's best direction when this happens, preventing spurious
DEGENERATE returns on real cone geometry.

---

### 2026-04-13 — Cone fitting rewrite: Gauss-Newton 4-parameter

**Root cause:** The algebraic 5-parameter linearization introduced a redundant parameter
(E = b² − cx² − cy²), making the 5×5 normal equations rank-deficient when data has ≤ 2
distinct t-values. This is the common case for CAD-exported frustums (two vertex rings at
different heights). The solver either returned singular or produced silently wrong results
(e.g. 36.87° half-angle instead of the correct 14.04°, with misleadingly low RMSE).

**Fix:** Replaced with a direct Gauss-Newton fit on the 4 geometric parameters (cx, cy, s, b)
where `r_i = s·t_i + b`. The residual `e_i = r_i − (s·t_i + b)` is minimized via
`δθ = −(JᵀJ)⁻¹ Jᵀe` iterations (4×4 system). Initialization: cx/cy from centroid, s/b from
linear regression of r vs t. Converges in 2–3 iterations.

This approach has no rank-deficiency issues because there are no redundant parameters. Verified
on the Ref_Frustum STL data: 2-ring subsets (where the old method failed completely) now produce
exact results (14.04° half-angle, RMSE ≈ 0). Also handles oblique-cut frustums and multi-ring
data correctly.

Also removed the temporary `fusionAddInUtils` import that was added for cone-fit diagnostics.

Derivation: `exploration/notes/cone-fitting-gauss-newton.md`.
Experimental verification: `exploration/scratch/cone_gauss_newton.py`.

**Manual fit failure: row label and cache sentinel:** When a manually-typed fit (Cylinder, Cone,
Sphere) returns DEGENERATE, the row label now shows "failed" instead of retaining the previous
fit's RMSE. Previously the cache was left untouched on failure, so the old result would display
until a new vertex was added. A sentinel `{'failed': True}` is stored so subsequent cache hits also
skip BRep creation without re-running the fitter.

**Auto mode: absolute improvement threshold (`_AUTO_MIN_IMPROVEMENT = 0.001 cm`):** Each surface
type in the evaluation chain (plane → cylinder → cone → sphere) must beat the current best by at
least 0.01 mm to take priority. Without this, the algebraically-exact sphere solver (which reaches
~1e-12 cm RMSE on perfect data) would displace the Nelder-Mead cylinder fitter (~1e-5 cm on perfect
data) on degenerate inputs where both fits are numerically near-zero — for example, selecting the
two end rings of a cylinder, which are equidistant from a sphere centre. The relative factor
approach fails here because the ratio can be 10^6+, making any finite factor insufficient; an
absolute threshold works because real fitting errors are always >> 0.01 mm. The evaluation order
encodes the engineering prior: planes and cylinders are far more common than cones and spheres.
