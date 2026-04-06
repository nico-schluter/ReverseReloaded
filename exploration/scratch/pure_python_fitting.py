# Pure Python surface fitting — no numpy, no scipy
# Replicates the algorithms from exploration/Reverse/Reverse.py
# Run standalone to validate on toy inputs.

import math
import random


# ============================================================
# Vector math (pure Python, operating on lists/tuples of floats)
# ============================================================

def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def normalize(v):
    n = norm(v)
    return [v[0]/n, v[1]/n, v[2]/n]

def cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]

def add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]

def sub(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]

def scale(v, s):
    return [v[0]*s, v[1]*s, v[2]*s]

def mean3(pts):
    n = len(pts)
    return [sum(p[i] for p in pts)/n for i in range(3)]

def dist_pt_to_line(p, a, b):
    """Distance from point p to line through a and b."""
    ab = sub(b, a)
    ap = sub(p, a)
    c = cross(ab, ap)
    return norm(c) / norm(ab)


# ============================================================
# Linear algebra: 3x3 Gaussian elimination (replaces numpy.linalg.lstsq
# for our specific use — normal equations give a 3x3 system)
# ============================================================

def solve3x3(A, b):
    """Solve Ax = b for 3x3 A via Gaussian elimination with partial pivoting.
    Returns x as a list of 3 floats, or None if singular.
    """
    # Augmented matrix
    M = [[A[i][j] for j in range(3)] + [b[i]] for i in range(3)]

    for col in range(3):
        # Partial pivot
        max_row = max(range(col, 3), key=lambda r: abs(M[r][col]))
        M[col], M[max_row] = M[max_row], M[col]

        pivot = M[col][col]
        if abs(pivot) < 1e-15:
            return None  # singular

        for row in range(col+1, 3):
            factor = M[row][col] / pivot
            for j in range(col, 4):
                M[row][j] -= factor * M[col][j]

    # Back substitution
    x = [0.0, 0.0, 0.0]
    for i in range(2, -1, -1):
        x[i] = M[i][3]
        for j in range(i+1, 3):
            x[i] -= M[i][j] * x[j]
        x[i] /= M[i][i]

    return x


def lstsq3(H, b_vec):
    """Least squares: minimize ||Hx - b||^2.
    H is N x 3 (list of N rows of 3 floats).
    b_vec is list of N floats.
    Returns x (3 floats) and residual norm squared.
    """
    # Normal equations: (H^T H) x = H^T b
    HtH = [[0.0]*3 for _ in range(3)]
    Htb = [0.0]*3
    for row, b_i in zip(H, b_vec):
        for i in range(3):
            Htb[i] += row[i] * b_i
            for j in range(3):
                HtH[i][j] += row[i] * row[j]

    x = solve3x3(HtH, Htb)
    if x is None:
        return None, float('inf')

    resid_sq = sum((sum(H[k][j]*x[j] for j in range(3)) - b_vec[k])**2 for k in range(len(H)))
    return x, resid_sq


# ============================================================
# Spherical angle <-> direction vector
# ============================================================

def spherical_to_direction(theta, phi):
    """Two spherical angles to unit vector.
    theta: azimuth in [0, 2pi)
    phi:   polar angle from Z in [0, pi)
    Returns [x, y, z] unit vector.
    Same convention as the original Reverse add-in.
    """
    return [
        math.cos(theta) * math.sin(phi),
        math.sin(theta) * math.sin(phi),
        math.cos(phi),
    ]

def make_orthonormal_frame(w):
    """Given a unit vector w, return orthonormal u, v such that u, v, w form a right-handed frame."""
    # Perturb w slightly to get a non-parallel reference
    ref = [w[1], w[2], w[0]]
    u = normalize(cross(w, ref))
    v = cross(w, u)
    return u, v


# ============================================================
# 2D circle fitting (linear least squares)
# Replaces fitCircle2D from Reverse
# ============================================================

def fit_circle_2d(pts2d):
    """Fit a circle to N >= 3 2D points.
    pts2d: list of (x, y) tuples.
    Returns (cx, cy, r, residual_sq).
    Uses the non-dimensionalized linear form: H @ [A,B,C] = -1
    where H_i = [xi²+yi², xi, yi], and circle params are:
        cx = -B/(2A), cy = -C/(2A), r = mean radial distance
    """
    H = [(xi**2 + yi**2, xi, yi) for (xi, yi) in pts2d]
    b_vec = [-1.0] * len(pts2d)

    xe, _ = lstsq3(H, b_vec)
    if xe is None:
        return None

    cx = -xe[1] / (2.0 * xe[0])
    cy = -xe[2] / (2.0 * xe[0])
    r = math.sqrt(sum((xi - cx)**2 + (yi - cy)**2 for (xi, yi) in pts2d) / len(pts2d))

    resid_sq = sum(((xi - cx)**2 + (yi - cy)**2 - r**2)**2 for (xi, yi) in pts2d)
    return cx, cy, r, resid_sq


# ============================================================
# Plane fitting
# Replaces fitPlaneOnAxis + fitPlaneToPoints2 from Reverse
# ============================================================

def fit_plane_on_axis(pts, n):
    """Given plane normal n (unit vector) and points, find best offset d.
    The plane is: dot(n, p) = d
    Returns (d, residual_sq).
    """
    projections = [dot(n, p) for p in pts]
    d = sum(projections) / len(projections)
    resid_sq = sum((proj - d)**2 for proj in projections)
    return d, resid_sq


def fit_plane_to_points(pts):
    """Fit a plane to a set of 3D points.
    Returns (n, d) where n is the unit normal and d is the offset.
    The plane is: dot(n, p) = d
    """
    best_angles, best_resid = _grid_search_2d(
        lambda theta, phi: fit_plane_on_axis(pts, spherical_to_direction(theta, phi))[1],
        theta_steps=9, phi_steps=5
    )
    opt_angles = _nelder_mead_2d(
        lambda angles: fit_plane_on_axis(pts, spherical_to_direction(angles[0], angles[1]))[1],
        best_angles
    )
    n = normalize(spherical_to_direction(opt_angles[0], opt_angles[1]))
    d, resid = fit_plane_on_axis(pts, n)
    return n, d, resid


# ============================================================
# Cylinder fitting
# Replaces fitCylinderOnAxis + fitCylinderToPoints from Reverse
# ============================================================

def fit_cylinder_on_axis(pts, axis):
    """Given a candidate axis direction, project points to 2D and fit circle.
    Returns (cx, cy, cz, r, resid_sq) where (cx,cy,cz) is the 3D axis origin.
    """
    w = normalize(axis)
    u, v = make_orthonormal_frame(w)

    # Project points to 2D (collapse along axis)
    pts2d = [(dot(p, u), dot(p, v)) for p in pts]

    result = fit_circle_2d(pts2d)
    if result is None:
        return None

    cx2, cy2, r, resid_sq = result

    # Map 2D circle center back to 3D
    mean_axial = sum(dot(p, w) for p in pts) / len(pts)
    cx3 = [cx2*u[i] + cy2*v[i] + mean_axial*w[i] for i in range(3)]

    return cx3, r, resid_sq


def fit_cylinder_to_points(pts):
    """Fit a cylinder to a set of 3D points.
    Returns (origin, axis, radius) where origin is a point on the axis,
    axis is the unit direction vector, and radius is the cylinder radius.
    """
    def cost(theta, phi):
        axis = spherical_to_direction(theta, phi)
        result = fit_cylinder_on_axis(pts, axis)
        if result is None:
            return float('inf')
        _, _, resid_sq = result
        return resid_sq

    best_angles, _ = _grid_search_2d(cost, theta_steps=9, phi_steps=5)
    opt_angles = _nelder_mead_2d(
        lambda angles: cost(angles[0], angles[1]),
        best_angles
    )

    axis = normalize(spherical_to_direction(opt_angles[0], opt_angles[1]))
    origin, radius, resid = fit_cylinder_on_axis(pts, axis)
    return origin, axis, radius, resid


def cylinder_axial_bounds(pts, origin, axis):
    """Return (min_t, max_t) — extent of points along the axis."""
    projections = [dot(sub(p, origin), axis) for p in pts]
    return min(projections), max(projections)


# ============================================================
# Optimization: Grid search + Nelder-Mead
# Replaces scipy.optimize.brute + scipy.optimize.fmin
# ============================================================

def _grid_search_2d(cost_fn, theta_steps=9, phi_steps=5):
    """Coarse grid search over theta in [0, 2pi) and phi in [0, pi).
    Returns (best_angles, best_cost) where best_angles = [theta, phi].
    """
    best_cost = float('inf')
    best_angles = [0.0, 0.0]

    for ti in range(theta_steps):
        theta = 2 * math.pi * ti / theta_steps
        for pi_ in range(phi_steps):
            phi = math.pi * pi_ / phi_steps
            c = cost_fn(theta, phi)
            if c < best_cost:
                best_cost = c
                best_angles = [theta, phi]

    return best_angles, best_cost


def _nelder_mead_2d(cost_fn, x0, tol=1e-8, max_iter=500):
    """Nelder-Mead minimization for a 2D function.
    x0: initial point [x, y]
    Returns optimal [x, y].
    """
    step = 0.1
    simplex = [
        list(x0),
        [x0[0] + step, x0[1]],
        [x0[0], x0[1] + step],
    ]
    costs = [cost_fn(s) for s in simplex]

    alpha = 1.0  # reflection
    gamma = 2.0  # expansion
    rho = 0.5    # contraction
    sigma = 0.5  # shrink

    for _ in range(max_iter):
        # Sort
        order = sorted(range(3), key=lambda i: costs[i])
        simplex = [simplex[i] for i in order]
        costs = [costs[i] for i in order]

        # Convergence check
        spread = max(abs(costs[i] - costs[0]) for i in range(1, 3))
        if spread < tol:
            break

        # Centroid of best two
        xc = [(simplex[0][j] + simplex[1][j]) / 2.0 for j in range(2)]

        # Reflection
        xr = [xc[j] + alpha * (xc[j] - simplex[2][j]) for j in range(2)]
        cr = cost_fn(xr)

        if cr < costs[0]:
            # Try expansion
            xe = [xc[j] + gamma * (xr[j] - xc[j]) for j in range(2)]
            ce = cost_fn(xe)
            if ce < cr:
                simplex[2], costs[2] = xe, ce
            else:
                simplex[2], costs[2] = xr, cr
        elif cr < costs[1]:
            simplex[2], costs[2] = xr, cr
        else:
            # Contraction
            xcon = [xc[j] + rho * (simplex[2][j] - xc[j]) for j in range(2)]
            ccon = cost_fn(xcon)
            if ccon < costs[2]:
                simplex[2], costs[2] = xcon, ccon
            else:
                # Shrink toward best
                for i in range(1, 3):
                    simplex[i] = [simplex[0][j] + sigma*(simplex[i][j] - simplex[0][j]) for j in range(2)]
                    costs[i] = cost_fn(simplex[i])

    return simplex[0]


# ============================================================
# 2D Convex Hull — Graham scan
# Replaces scipy.spatial.ConvexHull
# ============================================================

def convex_hull_2d(pts2d):
    """Compute 2D convex hull using Graham scan.
    pts2d: list of (x, y) tuples.
    Returns list of (x, y) tuples in CCW order forming the hull.
    """
    pts = list(pts2d)
    if len(pts) < 3:
        return pts

    # Find lowest-y, then lowest-x anchor
    anchor = min(pts, key=lambda p: (p[1], p[0]))

    def polar_angle(p):
        dx, dy = p[0] - anchor[0], p[1] - anchor[1]
        return math.atan2(dy, dx)

    def dist_sq(p):
        dx, dy = p[0] - anchor[0], p[1] - anchor[1]
        return dx*dx + dy*dy

    sorted_pts = sorted(pts, key=lambda p: (polar_angle(p), dist_sq(p)))

    def cross2d(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

    hull = []
    for p in sorted_pts:
        while len(hull) >= 2 and cross2d(hull[-2], hull[-1], p) <= 0:
            hull.pop()
        hull.append(p)

    return hull


def convex_hull_3d_on_plane(pts, n, d):
    """Project 3D points onto a plane defined by (n, d), compute 2D convex hull,
    return hull vertices back in 3D (all at the plane height).
    Returns list of [x, y, z] points in CCW hull order.
    """
    u, v = make_orthonormal_frame(n)

    # Project to 2D local frame
    pts2d = [(dot(p, u), dot(p, v)) for p in pts]

    hull2d = convex_hull_2d(pts2d)

    # Project back to 3D at the plane
    hull3d = [
        [hull2d[i][0]*u[j] + hull2d[i][1]*v[j] + d*n[j] for j in range(3)]
        for i in range(len(hull2d))
    ]
    return hull3d


# ============================================================
# Tests
# ============================================================

def _make_plane_points(n, d, count=50, noise=1e-4):
    """Generate points on a plane with optional noise."""
    n = normalize(n)
    u, v = make_orthonormal_frame(n)
    pts = []
    rng = random.Random(42)
    for _ in range(count):
        s = rng.uniform(-1, 1)
        t = rng.uniform(-1, 1)
        # point on plane: n*d + s*u + t*v
        base = [d*n[i] + s*u[i] + t*v[i] for i in range(3)]
        nz = rng.gauss(0, noise)
        p = add(base, scale(n, nz))
        pts.append(p)
    return pts


def _make_cylinder_points(origin, axis, radius, count=100, noise=1e-4):
    """Generate points on a cylinder surface."""
    axis = normalize(axis)
    u, v = make_orthonormal_frame(axis)
    rng = random.Random(42)
    pts = []
    for _ in range(count):
        theta = rng.uniform(0, 2*math.pi)
        t = rng.uniform(-1, 1)
        r_noise = rng.gauss(0, noise)
        r = radius + r_noise
        p = [
            origin[i] + t*axis[i] + r*math.cos(theta)*u[i] + r*math.sin(theta)*v[i]
            for i in range(3)
        ]
        pts.append(p)
    return pts


def test_circle_2d():
    print("--- test_circle_2d ---")
    cx_true, cy_true, r_true = 2.0, -1.5, 3.0
    rng = random.Random(7)
    pts = []
    for _ in range(30):
        theta = rng.uniform(0, 2*math.pi)
        pts.append((cx_true + r_true*math.cos(theta), cy_true + r_true*math.sin(theta)))
    cx, cy, r, resid = fit_circle_2d(pts)
    print(f"  True:  cx={cx_true:.4f}, cy={cy_true:.4f}, r={r_true:.4f}")
    print(f"  Found: cx={cx:.4f}, cy={cy:.4f}, r={r:.4f}, resid={resid:.2e}")
    assert abs(cx - cx_true) < 1e-6, f"cx error: {cx}"
    assert abs(cy - cy_true) < 1e-6, f"cy error: {cy}"
    assert abs(r - r_true) < 1e-6, f"r error: {r}"
    print("  PASS")


def test_plane_fit():
    print("--- test_plane_fit ---")
    n_true = normalize([1.0, 2.0, 3.0])
    d_true = 1.5
    pts = _make_plane_points(n_true, d_true, count=60, noise=1e-5)

    n_fit, d_fit, resid = fit_plane_to_points(pts)

    # Normal could be flipped — check both
    angle = math.acos(max(-1.0, min(1.0, abs(dot(n_fit, n_true)))))
    print(f"  True:  n={[f'{x:.4f}' for x in n_true]}, d={d_true:.4f}")
    print(f"  Found: n={[f'{x:.4f}' for x in n_fit]}, d={d_fit:.4f}, resid={resid:.2e}")
    print(f"  Normal angle error: {math.degrees(angle):.4f} deg")
    assert angle < 0.01, f"normal angle error too large: {math.degrees(angle):.2f} deg"
    print("  PASS")


def test_cylinder_fit():
    print("--- test_cylinder_fit ---")
    origin_true = [0.5, -0.3, 0.0]
    axis_true = normalize([0.0, 0.0, 1.0])
    r_true = 2.5

    pts = _make_cylinder_points(origin_true, axis_true, r_true, count=80, noise=1e-4)

    origin_fit, axis_fit, r_fit, resid = fit_cylinder_to_points(pts)

    angle = math.acos(max(-1.0, min(1.0, abs(dot(axis_fit, axis_true)))))
    print(f"  True:  axis={[f'{x:.4f}' for x in axis_true]}, r={r_true:.4f}")
    print(f"  Found: axis={[f'{x:.4f}' for x in axis_fit]}, r={r_fit:.4f}, resid={resid:.2e}")
    print(f"  Axis angle error: {math.degrees(angle):.4f} deg")
    assert angle < 1.0, f"axis angle too large: {math.degrees(angle):.2f} deg"
    assert abs(r_fit - r_true) < 0.01, f"radius error: {r_fit - r_true}"
    print("  PASS")


def test_convex_hull():
    print("--- test_convex_hull ---")
    # Square centered at origin
    pts = [(-1,-1),(1,-1),(1,1),(-1,1),(0,0),(0.5,0.3),(-0.2,0.8)]
    hull = convex_hull_2d(pts)
    print(f"  Hull: {hull}")
    # Should have 4 points (the square corners)
    assert len(hull) == 4, f"Expected 4 hull points, got {len(hull)}"
    print("  PASS")


def test_tilted_cylinder():
    """Test fitting a cylinder with a tilted axis."""
    print("--- test_tilted_cylinder ---")
    axis_true = normalize([1.0, 1.0, 2.0])
    origin_true = [1.0, 0.5, -0.5]
    r_true = 1.2

    pts = _make_cylinder_points(origin_true, axis_true, r_true, count=100, noise=1e-4)

    origin_fit, axis_fit, r_fit, resid = fit_cylinder_to_points(pts)

    angle = math.acos(max(-1.0, min(1.0, abs(dot(axis_fit, axis_true)))))
    print(f"  True:  axis={[f'{x:.4f}' for x in axis_true]}, r={r_true:.4f}")
    print(f"  Found: axis={[f'{x:.4f}' for x in axis_fit]}, r={r_fit:.4f}, resid={resid:.2e}")
    print(f"  Axis angle error: {math.degrees(angle):.4f} deg")
    assert angle < 1.0, f"axis angle too large: {math.degrees(angle):.2f} deg"
    assert abs(r_fit - r_true) < 0.05, f"radius error: {r_fit - r_true}"
    print("  PASS")


if __name__ == "__main__":
    test_circle_2d()
    test_plane_fit()
    test_cylinder_fit()
    test_convex_hull()
    test_tilted_cylinder()
    print("\nAll tests passed.")
