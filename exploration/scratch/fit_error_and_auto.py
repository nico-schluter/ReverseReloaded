# Experiment: fit error (RMSE), auto surface type selection, outlier detection.
# All pure Python — validates the math before any changes to src/.
# See exploration/notes/fit-error-auto-outliers.md for derivations.
#
# Run: python3 exploration/scratch/fit_error_and_auto.py

import math
import random


# ============================================================
# Shared vector math (duplicated from pure_python_fitting.py
# so this script is self-contained)
# ============================================================

def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def normalize(v):
    n = norm(v)
    return [v[0]/n, v[1]/n, v[2]/n]

def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

def sub(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]

def add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]

def scale(v, s):
    return [v[0]*s, v[1]*s, v[2]*s]


def make_frame(w):
    ref = [w[1], w[2], w[0]]
    u = normalize(cross(w, ref))
    v = cross(w, u)
    return u, v


def spherical_to_dir(theta, phi):
    return [
        math.cos(theta) * math.sin(phi),
        math.sin(theta) * math.sin(phi),
        math.cos(phi),
    ]


def solve3x3(A, b):
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


def lstsq3(H, b_vec):
    HtH = [[0.0]*3 for _ in range(3)]
    Htb = [0.0]*3
    for row, bi in zip(H, b_vec):
        for i in range(3):
            Htb[i] += row[i] * bi
            for j in range(3):
                HtH[i][j] += row[i] * row[j]
    x = solve3x3(HtH, Htb)
    if x is None:
        return None, float('inf')
    resid = sum((sum(H[k][j]*x[j] for j in range(3)) - b_vec[k])**2 for k in range(len(H)))
    return x, resid


def fit_circle_2d(pts2d):
    H = [(xi**2+yi**2, xi, yi) for xi, yi in pts2d]
    xe, _ = lstsq3(H, [-1.0]*len(pts2d))
    if xe is None:
        return None
    cx = -xe[1] / (2.0 * xe[0])
    cy = -xe[2] / (2.0 * xe[0])
    r = math.sqrt(sum((xi-cx)**2+(yi-cy)**2 for xi, yi in pts2d) / len(pts2d))
    return cx, cy, r


def grid_search(cost_fn, theta_steps=9, phi_steps=5):
    best_cost, best = float('inf'), [0.0, 0.0]
    for ti in range(theta_steps):
        theta = 2*math.pi*ti/theta_steps
        for pi_ in range(phi_steps):
            phi = math.pi*pi_/phi_steps
            c = cost_fn(theta, phi)
            if c < best_cost:
                best_cost, best = c, [theta, phi]
    return best, best_cost


def nelder_mead(cost_fn, x0, tol=1e-8, max_iter=500):
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


# ============================================================
# Fitting with proper RMSE
# ============================================================

def _plane_cost(pts, n):
    projs = [dot(n, p) for p in pts]
    d = sum(projs) / len(projs)
    return d, sum((p - d)**2 for p in projs)


def fit_plane(pts):
    """Returns (n, d, rmse, per_point_errors).
    n: unit normal, d: offset, rmse: sqrt(mean squared distance), errors: list of |e_i|.
    """
    best, _ = grid_search(lambda t, p: _plane_cost(pts, spherical_to_dir(t, p))[1])
    opt = nelder_mead(lambda a: _plane_cost(pts, spherical_to_dir(a[0], a[1]))[1], best)
    n = normalize(spherical_to_dir(opt[0], opt[1]))
    d, resid_sum = _plane_cost(pts, n)
    errors = [abs(dot(n, p) - d) for p in pts]
    rmse = math.sqrt(resid_sum / len(pts))
    return n, d, rmse, errors


def _cyl_cost_on_axis(pts, axis):
    w = normalize(axis)
    u, v = make_frame(w)
    pts2d = [(dot(p, u), dot(p, v)) for p in pts]
    result = fit_circle_2d(pts2d)
    if result is None:
        return float('inf')
    cx2, cy2, r = result
    # Use the optimizer's cost: sum of squared (dist - r)^2 for Nelder-Mead
    return sum(((math.sqrt((xi-cx2)**2+(yi-cy2)**2) - r)**2) for xi, yi in pts2d)


def fit_cylinder(pts):
    """Returns (origin, axis, radius, rmse, per_point_errors) or None on failure.
    rmse is sqrt(mean squared radial distance error), errors are per-point |dist_2d - r|.
    """
    def cost(t, p):
        return _cyl_cost_on_axis(pts, spherical_to_dir(t, p))

    best, _ = grid_search(cost)
    opt = nelder_mead(lambda a: cost(a[0], a[1]), best)
    axis = normalize(spherical_to_dir(opt[0], opt[1]))

    w = axis
    u, v = make_frame(w)
    pts2d = [(dot(p, u), dot(p, v)) for p in pts]
    result = fit_circle_2d(pts2d)
    if result is None:
        return None

    cx2, cy2, r = result
    mean_ax = sum(dot(p, w) for p in pts) / len(pts)
    origin = [cx2*u[i] + cy2*v[i] + mean_ax*w[i] for i in range(3)]

    errors = [abs(math.sqrt((xi-cx2)**2 + (yi-cy2)**2) - r) for xi, yi in pts2d]
    rmse = math.sqrt(sum(e**2 for e in errors) / len(pts))
    return origin, axis, r, rmse, errors


# ============================================================
# Outlier detection
# ============================================================

def outlier_indices(errors, k=2.0):
    """Return set of indices where |error| > mean + k*std."""
    n = len(errors)
    if n == 0:
        return set()
    mean_e = sum(errors) / n
    var_e  = sum((e - mean_e)**2 for e in errors) / n
    std_e  = math.sqrt(var_e)
    threshold = mean_e + k * std_e
    return {i for i, e in enumerate(errors) if e > threshold}


# ============================================================
# Auto surface type selection
# ============================================================

MIN_PTS = {'Plane': 3, 'Cylinder': 4}


def _point_cloud_scale(pts):
    """Return the RMS distance from the centroid — a scale measure for the point cloud."""
    n = len(pts)
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    cz = sum(p[2] for p in pts) / n
    return math.sqrt(sum((p[0]-cx)**2 + (p[1]-cy)**2 + (p[2]-cz)**2 for p in pts) / n)


def auto_fit(pts, max_radius_ratio=5.0):
    """Run all fitters, return (best_type, fit_result, rmse) for the best match.

    max_radius_ratio: if cylinder radius > max_radius_ratio * point_cloud_scale,
    the cylinder fit is rejected as degenerate (it's approximating a plane).
    """
    candidates = []
    scale = _point_cloud_scale(pts)

    if len(pts) >= MIN_PTS['Plane']:
        n, d, rmse, errors = fit_plane(pts)
        candidates.append(('Plane', (n, d, errors), rmse))

    if len(pts) >= MIN_PTS['Cylinder']:
        result = fit_cylinder(pts)
        if result is not None:
            origin, axis, r, rmse, errors = result
            if r <= max_radius_ratio * scale:
                candidates.append(('Cylinder', (origin, axis, r, errors), rmse))
            # else: radius is implausibly large — cylinder degenerating to a plane, skip

    if not candidates:
        return None
    return min(candidates, key=lambda x: x[2])


# ============================================================
# Synthetic test data
# ============================================================

def make_plane_pts(n, d, count=60, noise=1e-4, rng=None):
    rng = rng or random.Random(42)
    n = normalize(n)
    u, v = make_frame(n)
    pts = []
    for _ in range(count):
        s, t = rng.uniform(-1, 1), rng.uniform(-1, 1)
        base = [d*n[i] + s*u[i] + t*v[i] for i in range(3)]
        nz = rng.gauss(0, noise)
        pts.append(add(base, scale(n, nz)))
    return pts


def make_cylinder_pts(origin, axis, radius, count=80, noise=1e-4, rng=None):
    rng = rng or random.Random(42)
    axis = normalize(axis)
    u, v = make_frame(axis)
    pts = []
    for _ in range(count):
        theta = rng.uniform(0, 2*math.pi)
        t = rng.uniform(-1, 1)
        r = radius + rng.gauss(0, noise)
        p = [origin[i] + t*axis[i] + r*math.cos(theta)*u[i] + r*math.sin(theta)*v[i]
             for i in range(3)]
        pts.append(p)
    return pts


def inject_outliers(pts, n_outliers=5, spread=0.5, rng=None):
    """Add n_outliers points randomly scattered away from the point cloud."""
    rng = rng or random.Random(99)
    out_indices = list(range(len(pts), len(pts) + n_outliers))
    extra = []
    for _ in range(n_outliers):
        # Pick a random existing point and displace it significantly
        base = pts[rng.randint(0, len(pts)-1)]
        disp = [rng.gauss(0, spread) for _ in range(3)]
        extra.append(add(base, disp))
    return pts + extra, out_indices


# ============================================================
# Tests
# ============================================================

def test_plane_rmse():
    print("--- test_plane_rmse ---")
    n_true = normalize([1.0, 2.0, 3.0])
    d_true = 1.5
    noise = 0.005  # 0.05 mm if units are cm
    pts = make_plane_pts(n_true, d_true, count=60, noise=noise)

    n_fit, d_fit, rmse, errors = fit_plane(pts)

    angle = math.acos(max(-1.0, min(1.0, abs(dot(n_fit, n_true)))))
    print(f"  Noise sigma = {noise*10:.3f} mm")
    print(f"  RMSE        = {rmse*10:.4f} mm   (expected ≈ {noise*10:.3f} mm)")
    print(f"  Normal angle error = {math.degrees(angle):.4f} deg")
    print(f"  Per-point errors: min={min(errors)*10:.4f} max={max(errors)*10:.4f} mm")
    assert rmse < noise * 3, f"RMSE too high: {rmse}"
    assert angle < 0.1
    print("  PASS")


def test_cylinder_rmse():
    print("--- test_cylinder_rmse ---")
    origin_true = [0.5, -0.3, 0.0]
    axis_true = normalize([0.0, 0.0, 1.0])
    r_true = 2.5
    noise = 0.005

    pts = make_cylinder_pts(origin_true, axis_true, r_true, count=80, noise=noise)
    result = fit_cylinder(pts)
    assert result is not None, "cylinder fit returned None"
    origin_fit, axis_fit, r_fit, rmse, errors = result

    angle = math.acos(max(-1.0, min(1.0, abs(dot(axis_fit, axis_true)))))
    print(f"  r_true={r_true:.4f}  r_fit={r_fit:.4f}  err={abs(r_fit-r_true)*10:.4f} mm")
    print(f"  Noise sigma = {noise*10:.3f} mm")
    print(f"  RMSE        = {rmse*10:.4f} mm   (expected ≈ {noise*10:.3f} mm)")
    print(f"  Axis angle error = {math.degrees(angle):.4f} deg")
    print(f"  Per-point errors: min={min(errors)*10:.4f} max={max(errors)*10:.4f} mm")
    assert rmse < noise * 5, f"RMSE too high: {rmse}"
    assert angle < 1.0
    print("  PASS")


def test_outlier_detection():
    print("--- test_outlier_detection ---")
    n_true = normalize([0.0, 0.0, 1.0])
    d_true = 0.0
    pts_clean = make_plane_pts(n_true, d_true, count=50, noise=0.001)
    pts_with_outliers, true_out_indices = inject_outliers(pts_clean, n_outliers=5, spread=0.3)

    n_fit, d_fit, rmse, errors = fit_plane(pts_with_outliers)
    detected = outlier_indices(errors, k=2.0)

    print(f"  True outlier indices:    {sorted(true_out_indices)}")
    print(f"  Detected outlier indices: {sorted(detected)}")
    print(f"  True positives:  {len(detected & set(true_out_indices))}/{len(true_out_indices)}")
    print(f"  False positives: {len(detected - set(true_out_indices))}")
    # Expect most true outliers to be detected
    tp = len(detected & set(true_out_indices))
    assert tp >= 3, f"Too few true outliers detected: {tp}"
    print("  PASS")


def test_auto_fit_plane():
    print("--- test_auto_fit_plane (plane pts → should pick Plane) ---")
    n_true = normalize([1.0, 0.0, 0.0])
    d_true = 2.0
    pts = make_plane_pts(n_true, d_true, count=60, noise=0.001)

    result = auto_fit(pts)
    assert result is not None
    best_type, fit_params, rmse = result
    print(f"  Best type: {best_type}  RMSE: {rmse*10:.4f} mm")
    assert best_type == 'Plane', f"Expected Plane, got {best_type}"
    print("  PASS")


def test_auto_fit_cylinder():
    print("--- test_auto_fit_cylinder (cylinder pts → should pick Cylinder) ---")
    pts = make_cylinder_pts([0,0,0], [0,0,1], radius=1.5, count=80, noise=0.001)

    result = auto_fit(pts)
    assert result is not None
    best_type, fit_params, rmse = result
    print(f"  Best type: {best_type}  RMSE: {rmse*10:.4f} mm")
    assert best_type == 'Cylinder', f"Expected Cylinder, got {best_type}"
    print("  PASS")


def test_auto_fit_reports_rmse_both():
    """Print RMSE for both fitters on plane and cylinder data, for manual inspection."""
    print("--- test_auto_fit_reports_rmse_both ---")
    plane_pts = make_plane_pts(normalize([1,0,0]), 2.0, count=60, noise=0.001)
    cyl_pts   = make_cylinder_pts([0,0,0], [0,0,1], radius=1.5, count=80, noise=0.001)

    for label, pts in [('PLANE data', plane_pts), ('CYLINDER data', cyl_pts)]:
        print(f"  {label}:")
        if len(pts) >= MIN_PTS['Plane']:
            _, _, rmse_p, _ = fit_plane(pts)
            print(f"    Plane    RMSE = {rmse_p*10:.4f} mm")
        if len(pts) >= MIN_PTS['Cylinder']:
            result = fit_cylinder(pts)
            if result:
                _, _, _, rmse_c, _ = result
                print(f"    Cylinder RMSE = {rmse_c*10:.4f} mm")
    print("  (verify plane data has lower plane RMSE; cylinder data has lower cylinder RMSE)")
    print("  PASS (manual verification)")


def test_outlier_cylinder():
    print("--- test_outlier_detection_cylinder ---")
    pts_clean = make_cylinder_pts([0,0,0], [0,0,1], radius=2.0, count=60, noise=0.001)
    pts_with_outliers, true_out_indices = inject_outliers(pts_clean, n_outliers=5, spread=0.5)

    result = fit_cylinder(pts_with_outliers)
    assert result is not None
    _, _, _, rmse, errors = result
    detected = outlier_indices(errors, k=2.0)

    print(f"  True outlier indices:    {sorted(true_out_indices)}")
    print(f"  Detected outlier indices: {sorted(detected)}")
    tp = len(detected & set(true_out_indices))
    print(f"  True positives: {tp}/{len(true_out_indices)}")
    print(f"  False positives: {len(detected - set(true_out_indices))}")
    assert tp >= 3, f"Too few true outliers detected: {tp}"
    print("  PASS")


if __name__ == '__main__':
    test_plane_rmse()
    test_cylinder_rmse()
    test_outlier_detection()
    test_auto_fit_plane()
    test_auto_fit_cylinder()
    test_auto_fit_reports_rmse_both()
    test_outlier_cylinder()
    print("\nAll tests passed.")
