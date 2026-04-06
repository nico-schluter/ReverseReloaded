# Surface Fitting Algorithms — Pure Python Approach

## Problem

Given a set of 3D points selected from a mesh surface, find the best-fit primitive surface (plane, cylinder, sphere, cone) and produce a BRep surface in Fusion 360.

The original Reverse add-in used numpy and scipy. We need identical algorithms in pure Python (no external dependencies).

---

## Vector Math Replacements

All operations that were done with numpy arrays can be replaced with simple Python functions operating on plain lists or tuples of 3 floats.

```
dot(a, b)       = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
norm(v)         = sqrt(dot(v, v))
normalize(v)    = [x/norm(v) for x in v]
cross(a, b)     = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
add(a, b)       = [a[i]+b[i] for i in range(3)]
sub(a, b)       = [a[i]-b[i] for i in range(3)]
scale(v, s)     = [x*s for x in v]
average(pts)    = [sum(p[i] for p in pts)/len(pts) for i in range(3)]
```

---

## Linear Least Squares (replacing numpy.linalg.lstsq)

Used in `fitCircle2D`. The problem is: minimize ||Hx - b||² where H is N×3 and b is N×1.

The normal equations: (H^T H) x = H^T b

This is a 3×3 system, solved directly by Gaussian elimination with partial pivoting.

### Circle 2D fitting (linear form)

Given points (xi, yi) on a circle, reparameterize: let x = [A, B, C] and H = [[xi²+yi², xi, yi]]. The system Hx = -1 (ones) gives:

```
cx = -B / (2A)
cy = -C / (2A)
r  = sqrt(mean((xi-cx)² + (yi-cy)²))
```

This avoids any iterative optimization for circle fitting — it's a direct closed-form linear solve.

---

## Optimization: Grid Search + Nelder-Mead (replacing scipy.optimize.brute + fmin)

Both plane and cylinder fitting search over two spherical angles (θ, φ) that define the surface normal / cylinder axis.

### Spherical parameterization

A unit vector is defined by two angles:
```
direction(θ, φ) = [cos(θ)*sin(φ), sin(θ)*sin(φ), cos(φ)]
```
where θ ∈ [0, 2π) is the azimuth and φ ∈ [0, π) is the polar angle from Z.

Using this parameterization:
- The search space is bounded (no unbounded optimization)
- Only two parameters need iterative search; the rest are solved analytically
- The whole orientation space can be coarsely grid-searched

### Grid search

Enumerate a regular grid over (θ, φ). The original used π/4 steps (8×4 = 32 evaluations). Evaluate the cost function at each point, return the best.

### Nelder-Mead (downhill simplex)

Standard implementation starting from the best grid point. Steps:
1. Build initial simplex of 3 points around the best grid point
2. Reflect worst vertex through centroid of others
3. If improved: try expansion; else if better than 2nd worst: accept reflection; else try contraction; else shrink entire simplex toward best
4. Terminate when simplex size < tolerance or max iterations reached

This replaces `scipy.optimize.fmin`.

---

## 2D Convex Hull (replacing scipy.spatial.ConvexHull)

Used in plane fitting to determine the boundary of the reconstructed planar face.

Algorithm: **Graham scan**

1. Find the point with lowest y (then lowest x as tiebreak)
2. Sort all other points by polar angle relative to that anchor point
3. Walk through sorted points: if turning right (clockwise), pop the stack; otherwise push
4. The stack is the convex hull in CCW order

Then we need to produce line segments (pairs of consecutive hull vertices), which feed directly into `TemporaryBRepManager.createWireFromCurves`.

---

## Plane Fitting

### Parameters

A plane is defined by:
- Normal direction: spherical angles (θ, φ) → unit vector n
- Offset: scalar d such that n·p = d for all points p on the plane

### Direct solution for offset given normal

```python
d = mean(dot(n, p) for p in pts)
residual = sum((dot(n, p) - d)**2 for p in pts)
```

### Full fitting procedure

1. Grid search over (θ, φ) with step π/4 — evaluate residual at each
2. Nelder-Mead from best grid point
3. Direct solve for d from optimal normal
4. Result: n = direction(θ_opt, φ_opt), d_opt

### Creating the BRep face

1. Project points onto plane: p_proj = p - (dot(n, p) - d) * n
2. Build local 2D coordinate system (u, v) orthogonal to n
3. Express projections as 2D (u, v) coords
4. Compute Graham scan convex hull in 2D
5. Map hull vertices back to 3D (all at the plane)
6. Create Line3D segments, build wire, optionally offset outward, create face

---

## Cylinder Fitting

### Parameters

A cylinder is defined by:
- Axis direction: unit vector n (from spherical angles)
- Axis origin: point o on the axis
- Radius: r

### Direct solution for o and r given axis

1. Build orthonormal frame: u, v orthogonal to n
2. Project all points to 2D: xi = dot(p-anything, u), yi = dot(p-anything, v)  
   (the along-axis component drops out)
3. Fit circle in 2D (linear least squares) → cx, cy, r
4. Map 2D center back to 3D: o = cx*u + cy*v + mean_axial*n

The residual is the 2D circle fit residual.

### Full fitting procedure

1. Grid search over (θ, φ) — 32 evaluations
2. Nelder-Mead from best
3. Direct solve for o and r from optimal axis
4. Compute axial bounds: project all points onto axis, take min/max

### Creating the BRep surface

1. Compute p1 = o + n*min_bound, p2 = o + n*max_bound (optionally expanded)
2. Create Circle3D at p1 with normal n and radius r
3. Create Circle3D at p2 with normal n and radius r
4. Build two wire bodies from circles
5. `TemporaryBRepManager.createRuledSurface(wire1, wire2)` → cylindrical shell

---

## Vertex Selection — Old vs Potential New Approach

### Old approach (Reverse add-in)

- `MeshBodies` SelectionCommandInput selects the mesh body
- On selection, extract `displayMesh.nodeCoordinates`, transform to world space
- `mouseClick` CommandEventHandler fires on every click
- Convert 2D viewport position to 3D: `viewport.viewToModelSpace(viewportPosition)`
- For perspective: ray from camera eye through click point; for ortho: ray parallel to view direction
- Select all vertices closer than `fsRadius` to the ray (using line-to-point distance)
- Shift-click = deselect
- Highlight selected vertices via `CustomGraphicsPointSet` in `executePreview`

### Key issue with old approach

`cmd.mouseClick` is a raw viewport mouse event — not a "selection" in the Fusion API sense. The selection radius is screen-independent (it's in model space), so it's sensitive to zoom level. The approach is fragile because it bypasses Fusion's selection system entirely.

### Confirmed findings (2026-04-06, tested in Fusion)

Filter probe results from `addSelectionFilter`:
- **Accepted:** `Vertices`, `MeshBodies`, `Bodies`, `SolidBodies`, `Faces`, `Edges`, `SketchPoints`, `ConstructionPoints`, `Occurrences`
- **Rejected (throws):** `TSplineBodies`, `MeshVertices`, `BRepVertices`

`'Vertices'` is accepted but only highlights `adsk::fusion::MeshBody` when hovering over a mesh — it does not resolve individual mesh vertices. There is no filter for mesh vertices. **The mouseClick approach is still required.**

### Improved approach: screen-space selection

Instead of the original model-space ray distance, use the viewport's projection matrix:

```python
m = vp.modelToViewSpaceTransform.asArray()  # 16 floats, row-major 4x4
# For each vertex (x, y, z) from nodeCoordinatesAsDouble:
sx = m[0]*x + m[1]*y + m[2]*z + m[3]
sy = m[4]*x + m[5]*y + m[6]*z + m[7]
# For perspective only (camType != 0): divide by w
# w = m[12]*x + m[13]*y + m[14]*z + m[15]; sx /= w; sy /= w
dist_sq = (sx - click_vp_x)**2 + (sy - click_vp_y)**2
```

Verified: this matches `viewport.modelToViewSpace()` to sub-pixel accuracy (0 mismatches).

**Performance:** 0.23ms / 930 verts in pure Python → ~25ms / 100K verts, ~250ms / 1M verts. Acceptable for a click event. Real-world meshes from STL imports at typical quality settings are usually 50K–500K vertices.

**Advantages over old model-space approach:**
- Selection radius is in pixels — consistent feel at any zoom level
- Single matrix extraction per click instead of per-vertex API calls
- No camera type branching needed for the main path (orthographic = perspective with w≈1)

---

## Edge Cases & Notes

- Points must be in world space (component-local transforms must be applied)
- Very few points (< 3 for plane, < 6 for cylinder) should be rejected early
- Bad fits: reject if any parameter > 100000 or < -100000 (from original)
- The convex hull orientation (CW/CCW) matters for `offsetPlanarWire` direction
- `BaseFeature` required when `design.designType != 0` (parametric mode)
- All Fusion 360 units are centimeters internally
