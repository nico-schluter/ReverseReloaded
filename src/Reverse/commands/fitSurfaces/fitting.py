# Pure Python fitting algorithms — no numpy, no scipy, no Fusion API.
# Ported from exploration/scratch/pure_python_fitting.py and validated there.
#
# Public API used by entry.py:
#   fit_plane_to_points(pts)            → (n, d, resid)
#   fit_cylinder_to_points(pts)         → (origin, axis, radius, resid)
#   cylinder_axial_bounds(pts, o, ax)   → (t_min, t_max)
#   convex_hull_3d_on_plane(pts, n, d)  → list of 3D hull points
#   plane_point_errors(pts, n, d)       → list[float]
#   cylinder_point_errors(pts, o, ax, r)→ list[float]
#   compute_rmse(errors)                → float
#   outlier_indices(errors)             → set[int]
#   point_cloud_scale(pts)              → float

import math as _math

# Outlier detection thresholds
_OUTLIER_K       = 2.0    # flag if error > mean + k*std
_OUTLIER_MIN_ABS = 1e-4   # absolute floor (0.001 mm in cm units) — prevents noise
                           # from triggering outliers when all points fit nearly perfectly


# ---------------------------------------------------------------------------
# Vector math
# ---------------------------------------------------------------------------

def _dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def _norm(v):
    return _math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def _normalize(v):
    n = _norm(v)
    return [v[0]/n, v[1]/n, v[2]/n]

def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def _sub(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]

def _make_frame(w):
    """Return orthonormal (u, v) such that (u, v, w) is a right-handed frame."""
    ref = [w[1], w[2], w[0]]
    u = _normalize(_cross(w, ref))
    v = _cross(w, u)
    return u, v

def _spherical_to_dir(theta, phi):
    return [
        _math.cos(theta) * _math.sin(phi),
        _math.sin(theta) * _math.sin(phi),
        _math.cos(phi),
    ]


# ---------------------------------------------------------------------------
# Linear algebra helpers
# ---------------------------------------------------------------------------

def _solve3x3(A, b):
    M = [[A[i][j] for j in range(3)] + [b[i]] for i in range(3)]
    for col in range(3):
        max_row = max(range(col, 3), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]
        pivot = M[col][col]
        if abs(pivot) < 1e-15:
            return None
        for row in range(col+1, 3):
            f = M[row][col] / pivot
            for j in range(col, 4):
                M[row][j] -= f * M[col][j]
    x = [0.0, 0.0, 0.0]
    for i in range(2, -1, -1):
        x[i] = M[i][3]
        for j in range(i+1, 3):
            x[i] -= M[i][j] * x[j]
        x[i] /= M[i][i]
    return x


def _lstsq3(H, b_vec):
    HtH = [[0.0]*3 for _ in range(3)]
    Htb = [0.0]*3
    for row, bi in zip(H, b_vec):
        for i in range(3):
            Htb[i] += row[i] * bi
            for j in range(3):
                HtH[i][j] += row[i] * row[j]
    x = _solve3x3(HtH, Htb)
    if x is None:
        return None, float('inf')
    resid = sum((sum(H[k][j]*x[j] for j in range(3)) - b_vec[k])**2 for k in range(len(H)))
    return x, resid


# ---------------------------------------------------------------------------
# Optimisation primitives
# ---------------------------------------------------------------------------

def _grid_search(cost_fn, theta_steps=9, phi_steps=5):
    best_cost, best = float('inf'), [0.0, 0.0]
    for ti in range(theta_steps):
        theta = 2*_math.pi*ti/theta_steps
        for pi_ in range(phi_steps):
            phi = _math.pi*pi_/phi_steps
            c = cost_fn(theta, phi)
            if c < best_cost:
                best_cost, best = c, [theta, phi]
    return best, best_cost


def _nelder_mead(cost_fn, x0, tol=1e-8, max_iter=500):
    step = 0.1
    simplex = [list(x0), [x0[0]+step, x0[1]], [x0[0], x0[1]+step]]
    costs = [cost_fn(s) for s in simplex]
    for _ in range(max_iter):
        order = sorted(range(3), key=lambda i: costs[i])
        simplex = [simplex[i] for i in order]
        costs   = [costs[i]   for i in order]
        if max(abs(costs[i]-costs[0]) for i in range(1, 3)) < tol:
            break
        xc = [(simplex[0][j]+simplex[1][j])/2.0 for j in range(2)]
        xr = [xc[j]+1.0*(xc[j]-simplex[2][j]) for j in range(2)]; cr = cost_fn(xr)
        if cr < costs[0]:
            xe = [xc[j]+2.0*(xr[j]-xc[j]) for j in range(2)]; ce = cost_fn(xe)
            simplex[2], costs[2] = (xe, ce) if ce < cr else (xr, cr)
        elif cr < costs[1]:
            simplex[2], costs[2] = xr, cr
        else:
            xcon = [xc[j]+0.5*(simplex[2][j]-xc[j]) for j in range(2)]; ccon = cost_fn(xcon)
            if ccon < costs[2]:
                simplex[2], costs[2] = xcon, ccon
            else:
                for i in range(1, 3):
                    simplex[i] = [simplex[0][j]+0.5*(simplex[i][j]-simplex[0][j]) for j in range(2)]
                    costs[i] = cost_fn(simplex[i])
    return simplex[0]


# ---------------------------------------------------------------------------
# Plane fitting
# ---------------------------------------------------------------------------

def _fit_plane_on_axis(pts, n):
    projs = [_dot(n, p) for p in pts]
    d = sum(projs) / len(projs)
    return d, sum((p-d)**2 for p in projs)


def fit_plane_to_points(pts):
    """Returns (n, d, resid) — unit normal, offset, sum-of-squared residuals."""
    best, _ = _grid_search(lambda t, p: _fit_plane_on_axis(pts, _spherical_to_dir(t, p))[1])
    opt = _nelder_mead(lambda a: _fit_plane_on_axis(pts, _spherical_to_dir(a[0], a[1]))[1], best)
    n = _normalize(_spherical_to_dir(opt[0], opt[1]))
    d, resid = _fit_plane_on_axis(pts, n)
    return n, d, resid


# ---------------------------------------------------------------------------
# Cylinder fitting
# ---------------------------------------------------------------------------

def _fit_circle_2d(pts2d):
    H = [(xi**2+yi**2, xi, yi) for xi, yi in pts2d]
    xe, _ = _lstsq3(H, [-1.0]*len(pts2d))
    if xe is None:
        return None
    cx = -xe[1] / (2.0 * xe[0])
    cy = -xe[2] / (2.0 * xe[0])
    r = _math.sqrt(sum((xi-cx)**2+(yi-cy)**2 for xi, yi in pts2d) / len(pts2d))
    resid = sum(((xi-cx)**2+(yi-cy)**2-r**2)**2 for xi, yi in pts2d)
    return cx, cy, r, resid


def _fit_cylinder_on_axis(pts, axis):
    w = _normalize(axis)
    u, v = _make_frame(w)
    pts2d = [(_dot(p, u), _dot(p, v)) for p in pts]
    result = _fit_circle_2d(pts2d)
    if result is None:
        return None
    cx2, cy2, r, resid = result
    mean_ax = sum(_dot(p, w) for p in pts) / len(pts)
    cx3 = [cx2*u[i]+cy2*v[i]+mean_ax*w[i] for i in range(3)]
    return cx3, r, resid


def fit_cylinder_to_points(pts):
    """Returns (origin, axis, radius, resid)."""
    def cost(t, p):
        res = _fit_cylinder_on_axis(pts, _spherical_to_dir(t, p))
        return float('inf') if res is None else res[2]
    best, _ = _grid_search(cost)
    opt = _nelder_mead(lambda a: cost(a[0], a[1]), best)
    axis = _normalize(_spherical_to_dir(opt[0], opt[1]))
    result = _fit_cylinder_on_axis(pts, axis)
    if result is None:
        return None, None, None, float('inf')
    origin, radius, resid = result
    return origin, axis, radius, resid


def cylinder_axial_bounds(pts, origin, axis):
    """Returns (t_min, t_max) — extent of pts along axis from origin."""
    projs = [_dot(_sub(p, origin), axis) for p in pts]
    return min(projs), max(projs)


# ---------------------------------------------------------------------------
# Convex hull
# ---------------------------------------------------------------------------

def _convex_hull_2d(pts2d):
    pts = list(pts2d)
    if len(pts) < 3:
        return pts
    anchor = min(pts, key=lambda p: (p[1], p[0]))
    def polar(p):
        return _math.atan2(p[1]-anchor[1], p[0]-anchor[0])
    def dsq(p):
        return (p[0]-anchor[0])**2+(p[1]-anchor[1])**2
    sorted_pts = sorted(pts, key=lambda p: (polar(p), dsq(p)))
    def cross2(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    hull = []
    for p in sorted_pts:
        while len(hull) >= 2 and cross2(hull[-2], hull[-1], p) <= 0:
            hull.pop()
        hull.append(p)
    return hull


def convex_hull_3d_on_plane(pts, n, d):
    """Project pts to plane (n, d), compute hull, return 3D hull points."""
    u, v = _make_frame(n)
    pts2d = [(_dot(p, u), _dot(p, v)) for p in pts]
    hull2d = _convex_hull_2d(pts2d)
    return [[hull2d[i][0]*u[j]+hull2d[i][1]*v[j]+d*n[j] for j in range(3)]
            for i in range(len(hull2d))]


# ---------------------------------------------------------------------------
# Per-point errors, RMSE, outlier detection
# ---------------------------------------------------------------------------

def plane_point_errors(pts, n, d):
    """Per-point unsigned distance from the plane dot(n,p)=d."""
    return [abs(_dot(n, p) - d) for p in pts]


def cylinder_point_errors(pts, origin, axis, radius):
    """Per-point unsigned radial distance error from cylinder surface."""
    w = _normalize(axis)
    u, v = _make_frame(w)
    cx2 = _dot(origin, u)
    cy2 = _dot(origin, v)
    errors = []
    for p in pts:
        px2 = _dot(p, u)
        py2 = _dot(p, v)
        dist = _math.sqrt((px2 - cx2) ** 2 + (py2 - cy2) ** 2)
        errors.append(abs(dist - radius))
    return errors


def compute_rmse(errors):
    n = len(errors)
    return _math.sqrt(sum(e * e for e in errors) / n) if n else 0.0


def outlier_indices(errors, k=_OUTLIER_K):
    """Indices where error > mean + k*std AND error > _OUTLIER_MIN_ABS.
    The absolute floor prevents floating-point noise from triggering outliers
    when all points fit nearly perfectly (RMSE ≈ 0).
    """
    n = len(errors)
    if n < 2:
        return set()
    mean_e = sum(errors) / n
    var_e  = sum((e - mean_e) ** 2 for e in errors) / n
    std_e  = _math.sqrt(var_e)
    threshold = max(mean_e + k * std_e, _OUTLIER_MIN_ABS)
    return {i for i, e in enumerate(errors) if e > threshold}


def point_cloud_scale(pts):
    """RMS distance from centroid — scale measure for plausibility check."""
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    cz = sum(p[2] for p in pts) / n
    return _math.sqrt(sum((p[0]-cx)**2 + (p[1]-cy)**2 + (p[2]-cz)**2 for p in pts) / n)
