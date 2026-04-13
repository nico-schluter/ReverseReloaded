# Cone Fitting — Mathematical Derivation

## Parameterization

A circular cone is defined by:
- **Apex**: point A in 3D
- **Axis**: unit vector ŵ (2 DOF via spherical angles θ, φ)
- **Slope**: s = tan(α), where α is the half-angle

A point p lies on the cone iff its radial distance from the axis equals s times its
axial distance from the apex. Concretely, with perpendicular frame (û, v̂) to ŵ:

```
t_i     = dot(p_i, ŵ)                         — axial coordinate of p_i
q_xi    = dot(p_i, û),  q_yi = dot(p_i, v̂)    — perpendicular coords of p_i
t_apex  = dot(A, ŵ)                             — axial coordinate of apex
cx, cy  = dot(A, û), dot(A, v̂)                 — perp coords of apex (= axis center)

Cone condition: sqrt((q_xi − cx)² + (q_yi − cy)²) = s · (t_i − t_apex)
```

Squaring:
```
(q_xi − cx)² + (q_yi − cy)² = (s·t_i − s·t_apex)²
```

Define `b = −s·t_apex` (the linear-regression intercept). Then:
```
(q_xi − cx)² + (q_yi − cy)² = (s·t_i + b)²
```

---

## Algebraic Linearization

Expand both sides:

```
LHS: q_xi² − 2cx·q_xi + cx² + q_yi² − 2cy·q_yi + cy²

RHS: s²·t_i² + 2sb·t_i + b²
```

Rearrange (move all knowns to LHS, collect unknowns on RHS):

```
q_xi² + q_yi² = 2cx·q_xi + 2cy·q_yi + s²·t_i² + 2sb·t_i + (b² − cx² − cy²)
```

Define 5 unknowns:
```
A = 2·cx
B = 2·cy
C = s²
D = 2sb
F = b² − cx² − cy²
```

This is a **linear system**: for each point i, row of H is `[q_xi, q_yi, t_i², t_i, 1]`
and the target value is `y_i = q_xi² + q_yi²`. Solve min ||H·x − y||² (5×5 normal equations).

Recovery from solution [A, B, C, D, F]:
```
cx = A/2,  cy = B/2
s  = sqrt(C)       if C > 0; else degenerate (cylinder, s=0)
b  = D / (2s)
t_apex = −b/s = −D/(2C)
apex = cx·û + cy·v̂ + t_apex·ŵ   (all in world coords)
half_angle = atan(s)
```

This is O(n) per axis direction — identical cost structure to the algebraic circle fit for
cylinders. The outer optimization over axis direction (grid search + Nelder-Mead on θ, φ)
therefore has the same cost as cylinder fitting.

### Validity checks

After solving, reject (return cost=inf) if:
- C ≤ 0 (would imply imaginary half-angle — not a real cone with this axis)
- |s| < epsilon and b ≈ 0 (cylinder, handled separately)
- Half-angle outside plausible range (e.g. > 80° or < 0.5° suggests bad fit)

### Residual

Use the geometric radial residual (per-point error = |r_i − s·(t_i − t_apex)|):
```python
r_i     = sqrt((q_xi − cx)² + (q_yi − cy)²)
t_rel_i = t_i − t_apex
error_i = |r_i − s · t_rel_i|
residual = sum(error_i²)
```

This is what we optimize over axis direction, and what entry.py will use for RMSE.

---

## Full Fitting Algorithm

```
fit_cone_to_points(pts):
  1. Grid search: for each (θ, φ) in 9×5 grid:
       axis = spherical_to_dir(θ, φ)
       solve 5×5 system → cx, cy, s, b, residual
       track best (θ, φ, residual)

  2. Nelder-Mead: refine (θ, φ) from best grid point
       same analytical inner solve at each evaluation

  3. From best (θ, φ): solve for apex, half_angle

  4. Compute axial bounds: t_min, t_max = extent of data along axis
     (same as cylinder_axial_bounds, but relative to apex for radius computation)

  Returns: (apex, axis, half_angle, residual)
```

---

## Relationship to Cylinder

A cylinder is the degenerate cone with half_angle = 0 (s = 0).

In the algebraic solve: C = s² = 0 and D = 2sb = 0. The system becomes:
```
q_xi² + q_yi² = A·q_xi + B·q_yi + F
```
which is exactly the algebraic circle fit. So the cone fit degenerates smoothly.

When C ≈ 0 (within numerical tolerance), we should fall back to cylinder fit.

---

## BRep Creation (Fusion 360)

Use `TemporaryBRepManager.createCylinderOrCone(pointOne, r1, pointTwo, r2)`.

Given fitting result (apex, axis ŵ, half-angle α, slope s = tan(α)):

```
t_min, t_max = axial extent of data along ŵ (in absolute dot(p, ŵ) coordinates)
t_apex_abs   = dot(apex, ŵ)

t1 = t_min − expansion_dist  (optionally expanded)
t2 = t_max + expansion_dist

r1 = s · (t1 − t_apex_abs)   # radius at the lower end
r2 = s · (t2 − t_apex_abs)   # radius at the upper end

pointOne = apex + (t1 − t_apex_abs) · ŵ   (lower end center)
pointTwo = apex + (t2 − t_apex_abs) · ŵ   (upper end center)
```

If r1 or r2 < 0, clamp to 0 (apex is within the data extent — true cone, not frustum).
The Fusion API accepts r=0 at one end (apex point).

Note: the `Cone.create(origin, axis, radius, halfAngle)` transient geometry object is a
different thing (infinite surface); we use `createCylinderOrCone` for the bounded BRep.

---

## Numerical Conditioning

The t_i² column in H has much larger magnitude than the linear columns when data is far
from origin. Centering the data (subtract centroid before fitting, add back after) reduces
this: t_i become centered near 0, and t_i² are smaller. Apply the same centering as used
for cylinder fitting (subtract centroid from all pts before passing to the fitter).

---

## Edge Cases

- **All points at same axial height**: t_i all equal → D column degenerate. The cone is
  underdetermined from a single ring of points (infinitely many apex positions).
  Minimum required data: at least 2 distinct axial heights.

- **Truncated frustum** (apex outside data range): normal case. r1 > 0 and r2 > 0, the
  output is a frustum body.

- **Data includes apex** (r ≈ 0 at one end): r_apex_end ≈ 0, createCylinderOrCone handles it.

- **Very flat cone** (α close to 90°): s → ∞. C = s² very large. Numerical issues.
  Clamp max half-angle to ~80°.

- **Cylinder disguised as cone**: s close to 0. Auto mode compares cylinder RMSE vs cone
  RMSE and picks the winner. A near-cylinder cone fit won't be much better than a cylinder
  fit (both will have low RMSE), so plausibility checks and RMSE comparison suffice.
