# Cone Fitting — Gauss-Newton 4-Parameter Approach

## Motivation

The algebraic 5-parameter linearization (see cone-fitting.md) introduces a redundant
parameter E = b² − cx² − cy². This over-parameterization causes rank deficiency when
the data has ≤ 2 distinct t-values (e.g., two rings of a frustum from CAD export).

This document derives a direct 4-parameter fit that avoids the issue entirely.

## Setup

Given a fixed axis direction ŵ (with orthonormal frame û, v̂):

```
t_i  = dot(p_i, ŵ)       — axial coordinate
qx_i = dot(p_i, û)       — perpendicular x
qy_i = dot(p_i, v̂)       — perpendicular y
```

The cone surface condition (single nappe):

```
r_i = s · t_i + b
```

where:
- `r_i = sqrt((qx_i − cx)² + (qy_i − cy)²)` — radial distance from axis
- `s = tan(half_angle)` — slope
- `b = −s · t_apex` — intercept
- `cx, cy` — perpendicular coordinates of the axis (apex projected onto û, v̂ plane)

Four unknowns: **θ = (cx, cy, s, b)**.

## Residual

```
e_i(θ) = r_i(cx, cy) − (s · t_i + b)
```

where `r_i(cx, cy) = sqrt((qx_i − cx)² + (qy_i − cy)²)`.

Minimize: `Σ e_i²`

## Jacobian

```
∂e_i/∂cx = −(qx_i − cx) / r_i
∂e_i/∂cy = −(qy_i − cy) / r_i
∂e_i/∂s  = −t_i
∂e_i/∂b  = −1
```

So the Jacobian row for point i is:

```
J_i = [ −(qx_i − cx)/r_i,  −(qy_i − cy)/r_i,  −t_i,  −1 ]
```

## Gauss-Newton Update

```
δθ = −(JᵀJ)⁻¹ Jᵀe
θ  ← θ + δθ
```

This is a 4×4 system per iteration, O(n) to build. Converges in ~3-5 iterations
from a reasonable initial guess.

## Initialization

1. **cx₀, cy₀**: centroid of projected (qx, qy) coordinates. For a frustum this is
   biased toward the larger ring (more points at larger radius push centroid outward),
   but close enough for convergence.

2. **s₀, b₀**: with cx₀, cy₀ fixed, compute r_i for each point, then solve the 2×2
   linear regression `r_i = s · t_i + b` via normal equations.

## Why This Works for All Cases

- **Two flat rings** (2 distinct t-values): The regression `r = s·t + b` has 2 unknowns
  and 2 groups of data — perfectly constrained. cx, cy are constrained by the circular
  structure. No rank deficiency.

- **Oblique cuts** (varying t within each ring): Each point individually satisfies
  `r_i = s·t_i + b`. No need to group into rings or fit circles per ring.

- **Multiple rings**: More data → better conditioning. Works trivially.

- **Chamfer** (small axial extent): s is well-constrained by the radii difference
  between rings. The short t-range doesn't cause issues because we're fitting a
  2-parameter line, not separating t² from t from 1.

## Conditioning Analysis

The JᵀJ matrix has the structure (roughly):

```
        cx       cy       s        b
cx  [ Σ dx²/r²   ...     ...      ...  ]
cy  [  ...      Σ dy²/r²  ...      ...  ]
s   [  ...       ...     Σ t_i²   Σ t_i ]
b   [  ...       ...     Σ t_i      n   ]
```

where dx = (qx − cx)/r, dy = (qy − cy)/r.

The upper-left 2×2 block is well-conditioned as long as points span a range of angles
around the axis (they always do for a ring or partial ring).

The lower-right 2×2 block is the normal-equations matrix for `r = s·t + b`. With 2
distinct t-values, this has rank 2. With 1 t-value, rank 1 (can't separate s from b —
genuinely underdetermined, correct behavior).

The cross terms (cx/cy with s/b) are small for well-centered data.

## Sign Convention and abs(t)

The cone condition `r = s·t + b` assumes the data is on one nappe. For a frustum where
all t_i are on one side of the apex, s·t_i + b > 0 for all i. If the axis is flipped,
s and b both flip sign, and the model still works.

The hourglass check from the old code still applies: after solving, verify that
`s·t_i + b` doesn't change sign across the data range.

## Recovery

```
t_apex = −b/s
half_angle = atan(|s|)
apex = cx·û + cy·v̂ + t_apex·ŵ
```
