# Pure Python fitting algorithms — no numpy, no scipy, no Fusion API.
# Ported from exploration/scratch/pure_python_fitting.py and validated there.
#
# Public API used by entry.py:
#   fit_plane_to_points(pts)                    → (n, d, resid)
#   fit_cylinder_to_points(pts)                 → (origin, axis, radius, resid)
#   cylinder_axial_bounds(pts, o, ax)           → (t_min, t_max)
#   fit_cone_to_points(pts)                     → (apex, axis, half_angle, resid)
#   cone_axial_bounds(pts, apex, ax)            → (t_min, t_max)  [relative to apex]
#   fit_sphere_to_points(pts)                   → (center, radius, resid)
#   convex_hull_3d_on_plane(pts, n, d)          → list of 3D hull points
#   plane_point_errors(pts, n, d)               → list[float]
#   cylinder_point_errors(pts, o, ax, r)        → list[float]
#   cone_point_errors(pts, apex, ax, half_angle)→ list[float]
#   sphere_point_errors(pts, center, radius)    → list[float]
#   compute_rmse(errors)                        → float
#   outlier_indices(errors)                     → set[int]
#   point_cloud_scale(pts)                      → float

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
    # Choose the standard-basis vector least parallel to w — avoids zero cross product
    # when all components are equal (e.g. w = [1,1,1]/√3).
    ax, ay, az = abs(w[0]), abs(w[1]), abs(w[2])
    if ax <= ay and ax <= az:
        ref = [1.0, 0.0, 0.0]
    elif ay <= az:
        ref = [0.0, 1.0, 0.0]
    else:
        ref = [0.0, 0.0, 1.0]
    u = _normalize(_cross(w, ref))
    v = _cross(w, u)
    return u, v

def _spherical_to_dir(theta, phi):
    return [
        _math.cos(theta) * _math.sin(phi),
        _math.sin(theta) * _math.sin(phi),
        _math.cos(phi),
    ]


def _center_points(pts):
    """Subtract centroid from all points. Returns (centered_pts, centroid).

    Used by every fitter to keep coordinates small and near origin, which
    prevents large-magnitude columns (e.g. xi², t_i²) from dominating the
    normal equations and causing ill-conditioning.  The caller is responsible
    for shifting the fitted geometry back to world space afterwards.
    """
    n = len(pts)
    gcx = sum(p[0] for p in pts) / n
    gcy = sum(p[1] for p in pts) / n
    gcz = sum(p[2] for p in pts) / n
    gc  = [gcx, gcy, gcz]
    return [[p[0]-gcx, p[1]-gcy, p[2]-gcz] for p in pts], gc


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


def _solve_nxn(A, b):
    """Gaussian elimination with partial pivoting for an N×N system. Returns x or None."""
    n = len(b)
    M = [[A[i][j] for j in range(n)] + [b[i]] for i in range(n)]
    for col in range(n):
        max_row = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]
        pivot = M[col][col]
        if abs(pivot) < 1e-15:
            return None
        for row in range(col+1, n):
            f = M[row][col] / pivot
            for j in range(col, n+1):
                M[row][j] -= f * M[col][j]
    x = [0.0]*n
    for i in range(n-1, -1, -1):
        x[i] = M[i][n]
        for j in range(i+1, n):
            x[i] -= M[i][j]*x[j]
        x[i] /= M[i][i]
    return x


def _lstsq_n(H, b_vec):
    """Normal equations for an over-determined system with any number of columns."""
    nc = len(H[0])
    HtH = [[0.0]*nc for _ in range(nc)]
    Htb = [0.0]*nc
    for row, bi in zip(H, b_vec):
        for i in range(nc):
            Htb[i] += row[i] * bi
            for j in range(nc):
                HtH[i][j] += row[i] * row[j]
    x = _solve_nxn(HtH, Htb)
    if x is None:
        return None, float('inf')
    resid = sum((sum(H[k][j]*x[j] for j in range(nc)) - b_vec[k])**2 for k in range(len(H)))
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
    cpts, gc = _center_points(pts)
    best, _ = _grid_search(lambda t, p: _fit_plane_on_axis(cpts, _spherical_to_dir(t, p))[1])
    opt = _nelder_mead(lambda a: _fit_plane_on_axis(cpts, _spherical_to_dir(a[0], a[1]))[1], best)
    n = _normalize(_spherical_to_dir(opt[0], opt[1]))
    d_c, resid = _fit_plane_on_axis(cpts, n)
    d = d_c + _dot(n, gc)
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
    cpts, gc = _center_points(pts)

    def cost(t, p):
        res = _fit_cylinder_on_axis(cpts, _spherical_to_dir(t, p))
        return float('inf') if res is None else res[2]
    best, _ = _grid_search(cost)
    opt = _nelder_mead(lambda a: cost(a[0], a[1]), best)
    axis = _normalize(_spherical_to_dir(opt[0], opt[1]))
    result = _fit_cylinder_on_axis(cpts, axis)
    if result is None:
        return None, None, None, float('inf')
    origin_c, radius, resid = result
    origin = [origin_c[i] + gc[i] for i in range(3)]
    return origin, axis, radius, resid


def cylinder_axial_bounds(pts, origin, axis):
    """Returns (t_min, t_max) — extent of pts along axis from origin."""
    projs = [_dot(_sub(p, origin), axis) for p in pts]
    return min(projs), max(projs)


# ---------------------------------------------------------------------------
# Cone fitting
# ---------------------------------------------------------------------------
# Derivation in exploration/notes/cone-fitting-gauss-newton.md.
#
# Direct 4-parameter Gauss-Newton fit for a fixed axis direction.
# Parameters: (cx, cy, s, b) where r_i = s*t_i + b is the cone surface condition
# and (cx, cy) is the axis offset in the perpendicular plane.
#
# This avoids the rank-deficiency of the old algebraic 5-parameter linearization
# which introduced a redundant parameter and failed on data with ≤ 2 distinct
# t-values (e.g. two-ring frustums from CAD export).
# ---------------------------------------------------------------------------

def _fit_cone_on_axis(pts, axis, _max_iter=20, _tol=1e-12):
    """Gauss-Newton 4-parameter cone fit for a fixed axis direction.
    Returns (apex, half_angle_rad, residual) or (None, None, inf) on failure.
    """
    w = _normalize(axis)
    u, v = _make_frame(w)
    n = len(pts)

    t_v = [_dot(p, w) for p in pts]
    q_x = [_dot(p, u) for p in pts]
    q_y = [_dot(p, v) for p in pts]

    # Initialize cx, cy from centroid of projected coords
    cx = sum(q_x) / n
    cy = sum(q_y) / n

    # Initialize s, b from linear regression of r_i vs t_i
    r_v = [_math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2) for i in range(n)]
    sum_t  = sum(t_v)
    sum_r  = sum(r_v)
    sum_tt = sum(t * t for t in t_v)
    sum_tr = sum(t_v[i] * r_v[i] for i in range(n))
    det = n * sum_tt - sum_t * sum_t
    if abs(det) < 1e-30:
        # All points at same axial height — cone is underdetermined.
        return None, None, float('inf')
    s = (n * sum_tr - sum_t * sum_r) / det
    b = (sum_r * sum_tt - sum_t * sum_tr) / det

    # Gauss-Newton iterations
    for _iteration in range(_max_iter):
        r_v = [_math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2) for i in range(n)]
        e = [r_v[i] - (s * t_v[i] + b) for i in range(n)]

        # Build J^T J (4x4) and J^T e (4x1)
        JtJ = [[0.0] * 4 for _ in range(4)]
        Jte = [0.0] * 4

        for i in range(n):
            ri = r_v[i]
            if ri < 1e-15:
                continue  # point on axis — Jacobian undefined, skip
            dx = -(q_x[i] - cx) / ri
            dy = -(q_y[i] - cy) / ri
            row = [dx, dy, -t_v[i], -1.0]
            for a in range(4):
                Jte[a] += row[a] * e[i]
                for bb in range(4):
                    JtJ[a][bb] += row[a] * row[bb]

        delta = _solve_nxn(JtJ, Jte)
        if delta is None:
            return None, None, float('inf')

        cx -= delta[0]
        cy -= delta[1]
        s  -= delta[2]
        b  -= delta[3]

        if sum(d * d for d in delta) < _tol:
            break

    # Reject hourglass: s*t + b must not change sign across the data range.
    t_min_data = min(t_v)
    t_max_data = max(t_v)
    val_lo = s * t_min_data + b
    val_hi = s * t_max_data + b
    if val_lo * val_hi < 0:
        return None, None, float('inf')

    if abs(s) < 1e-15:
        # Degenerate: cylinder, not cone.
        return None, None, float('inf')

    t_apex = -b / s
    apex = [cx * u[i] + cy * v[i] + t_apex * w[i] for i in range(3)]
    half_angle = _math.atan(abs(s))

    # Geometric radial residual.
    resid = 0.0
    for i in range(n):
        r_i = _math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2)
        err = r_i - abs(s) * abs(t_v[i] - t_apex)
        resid += err * err

    return apex, half_angle, resid


def fit_cone_to_points(pts):
    """Returns (apex, axis, half_angle_rad, resid).
    Grid search over axis direction + Nelder-Mead refinement.
    Per-axis evaluation uses Gauss-Newton on 4 geometric parameters.

    Data is centered at its centroid before fitting to keep coordinates small.
    The apex is shifted back to world space after solving.
    """
    cpts, gc = _center_points(pts)

    def cost(theta, phi):
        result = _fit_cone_on_axis(cpts, _spherical_to_dir(theta, phi))
        return float('inf') if result[0] is None else result[2]

    best, best_cost = _grid_search(cost)
    if best_cost == float('inf'):
        return None, None, None, float('inf')
    opt = _nelder_mead(lambda a: cost(a[0], a[1]), best)

    axis   = _normalize(_spherical_to_dir(opt[0], opt[1]))
    result = _fit_cone_on_axis(cpts, axis)
    if result[0] is None:
        # Nelder-Mead drifted to a degenerate direction — fall back to best grid direction.
        axis   = _normalize(_spherical_to_dir(best[0], best[1]))
        result = _fit_cone_on_axis(cpts, axis)
    if result[0] is None:
        return None, None, None, float('inf')

    apex_c, half_angle, resid = result
    apex = [apex_c[i] + gc[i] for i in range(3)]
    return apex, axis, half_angle, resid


def cone_axial_bounds(pts, apex, axis):
    """Returns (t_min, t_max) — extent of pts along axis, measured from apex."""
    projs = [_dot(_sub(p, apex), axis) for p in pts]
    return min(projs), max(projs)


# ---------------------------------------------------------------------------
# Sphere fitting
# ---------------------------------------------------------------------------
# A point p lies on a sphere with center c and radius r iff ||p - c||² = r².
# Expanding and rearranging:
#   px² + py² + pz² = A·px + B·py + C·pz + E
# where A=2cx, B=2cy, C=2cz, E=r²-cx²-cy²-cz².
# This is a direct 4-parameter linear system — no axis to optimise over.
# Solve via 4×4 normal equations (O(n)), recover:
#   cx=A/2, cy=B/2, cz=C/2,  r=sqrt(cx²+cy²+cz²+E)
# ---------------------------------------------------------------------------

def fit_sphere_to_points(pts):
    """Returns (center, radius, resid) or (None, None, inf) on failure."""
    cpts, gc = _center_points(pts)

    y_v = [p[0]**2 + p[1]**2 + p[2]**2 for p in cpts]
    H   = [[p[0], p[1], p[2], 1.0] for p in cpts]
    x, _ = _lstsq_n(H, y_v)
    if x is None:
        return None, None, float('inf')

    cx_c = x[0] / 2.0
    cy_c = x[1] / 2.0
    cz_c = x[2] / 2.0
    r_sq = cx_c**2 + cy_c**2 + cz_c**2 + x[3]
    if r_sq <= 0:
        return None, None, float('inf')

    r      = _math.sqrt(r_sq)
    center = [cx_c + gc[0], cy_c + gc[1], cz_c + gc[2]]

    resid = sum((_math.sqrt((p[0]-center[0])**2 + (p[1]-center[1])**2 + (p[2]-center[2])**2) - r)**2
                for p in pts)
    return center, r, resid


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


def cone_point_errors(pts, apex, axis, half_angle):
    """Per-point unsigned radial error from cone surface."""
    w  = _normalize(axis)
    u, v = _make_frame(w)
    s  = _math.tan(half_angle)
    t_apex_abs = _dot(apex, w)
    cx = _dot(apex, u)
    cy = _dot(apex, v)
    errors = []
    for p in pts:
        r_i   = _math.sqrt((_dot(p, u) - cx)**2 + (_dot(p, v) - cy)**2)
        t_rel = _dot(p, w) - t_apex_abs
        # abs(t_rel): correct for any axis orientation relative to apex.
        errors.append(abs(r_i - s * abs(t_rel)))
    return errors


def sphere_point_errors(pts, center, radius):
    """Per-point unsigned distance error from sphere surface."""
    return [abs(_math.sqrt((p[0]-center[0])**2 + (p[1]-center[1])**2 + (p[2]-center[2])**2) - radius)
            for p in pts]


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
