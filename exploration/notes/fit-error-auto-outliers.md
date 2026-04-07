# Fit Error, Auto Surface Type, and Outlier Highlighting

## Goals

Three related features all building on a common primitive: **per-point signed error** for each fitted surface type.

1. **Fit error per row** — display RMSE in mm in the table after fitting.
2. **Auto surface type** — run all fitters on the selected points, pick the one with lowest RMSE.
3. **Outlier point highlighting** — flag points whose individual error exceeds mean + k·σ.

---

## Per-point error functions

### Plane

Plane defined by unit normal **n** and offset d: `dot(n, p) = d`.

Point-to-plane signed distance: `e_i = dot(n, p_i) - d`

Unsigned error: `|e_i|`

RMSE: `sqrt( sum(e_i²) / N )`

This is already what `fit_plane_on_axis` computes as `resid` (sum of squared e_i). So:

```
plane_rmse = sqrt(resid / N)
```

The existing `resid` from `fit_plane_to_points` is directly usable.

---

### Cylinder

Cylinder defined by axis direction **w** (unit), axis origin **o**, and radius r.

The axis origin **o** returned by the fitter is `cx2·u + cy2·v + mean_ax·w` where `(cx2, cy2)` is the 2D circle center and `mean_ax` is the mean axial projection. So `dot(o, u) = cx2` and `dot(o, v) = cy2`.

Per-point radial distance from axis:

```
px2 = dot(p_i, u)
py2 = dot(p_i, v)
dist_2d_i = sqrt( (px2 - dot(o, u))² + (py2 - dot(o, v))² )
e_i = dist_2d_i - r       # signed: positive = outside cylinder
```

Unsigned error: `|e_i|`

RMSE: `sqrt( sum(e_i²) / N )`

**Important:** the existing cylinder `resid` from `fit_circle_2d` is `sum((dist² - r²)²)`, not `sum(e_i²)`. This is optimized for the linear fit but is NOT a physical distance and NOT comparable to plane RMSE. Must recompute proper RMSE after fitting.

---

## Auto surface type selection

Run all available fitters on the selected points, compute RMSE for each. Pick the type with the lowest RMSE.

Minimum point requirements still apply per type.

```
candidates = []
for type in ['Plane', 'Cylinder']:
    if len(pts) >= MIN_PTS[type]:
        fit, rmse = fit_and_rmse(type, pts)
        if fit is not None:
            candidates.append((rmse, type, fit))
best = min(candidates, key=lambda x: x[0])
```

RMSE comparison is valid because both are in the same units (cm internally, displayed as mm).

### Critical: cylinder radius plausibility check

**Without a radius check, auto-fit always picks Cylinder.** A cylinder with a very large radius degenerates to a plane — it can fit flat points with RMSE as low as a plane fit, because the cylinder surface can be placed tangent to the flat cluster. Since cylinder has more degrees of freedom than a plane, it never does *worse* on RMSE.

Fix: reject cylinder fits where `r > max_ratio * point_cloud_scale`, where `point_cloud_scale` is the RMS distance from the point cloud centroid. `max_ratio=5` works in practice — legitimate cylinders in mechanical parts have radii comparable to the part feature size.

```python
scale = rms_distance_from_centroid(pts)
if r > 5.0 * scale:
    # degenerate — cylinder is approximating a plane, skip
    pass
```

Validated in `exploration/scratch/fit_error_and_auto.py`: plane data → Plane selected; cylinder data → Cylinder selected (5.88mm plane RMSE vs 0.0089mm cylinder RMSE).

---

## Outlier detection

After fitting, compute per-point errors `e_i = |point_to_surface_distance(p_i)|`.

Statistics:
```
mean_e = sum(e_i) / N
var_e  = sum((e_i - mean_e)²) / N
std_e  = sqrt(var_e)
threshold = mean_e + k * std_e   # k = 2.0 default
outliers = {i for i, e in enumerate(errors) if e > threshold}
```

Outlier indices can then be split off from the normal point set for different-colored graphics.

**Choice of k:** k=2 catches roughly the top ~5% of a normal distribution. For messy scans this may be too aggressive; k=3 is more conservative (~0.3%). Start with k=2, tune later.

**Caveat:** if all points are genuinely noisy (bad selection — mixing two surfaces), the outlier detection will still flag some subset. The RMSE number is the better signal in that case.

---

## Graphics integration

Current graphics: one `CustomGraphicsPointSet` per row, all points same color (per-row image).

For outlier highlighting, per row we need two point sets:
- Normal points: existing row color
- Outlier points: a fixed "warning" color (e.g. red/orange sprite)

The current sprite images are `resources/point_00.png` through `point_11.png`. Add a dedicated `point_outlier.png` (or reuse a high-index color as the outlier indicator).

Alternatively, use a second set with `PointCloudCustomGraphicsPointType` (pixel dots) for outliers — no image asset needed, smaller/less obtrusive.

---

## Implementation notes

- `fit_and_rmse_plane(pts)` → `(n, d, rmse)` — reuse existing fit, compute `sqrt(resid/N)`
- `fit_and_rmse_cylinder(pts)` → `(origin, axis, radius, rmse, u, v)` — reuse existing fit, recompute proper RMSE, expose frame vectors for per-point error
- `point_errors_plane(pts, n, d)` → list of per-point `|e_i|`
- `point_errors_cylinder(pts, origin, axis, radius)` → list of per-point `|e_i|`
- `outlier_indices(errors, k=2.0)` → set of indices where `e_i > mean + k*std`

All pure Python, no Fusion API. Validate in `exploration/scratch/fit_error_and_auto.py` before touching `src/`.
