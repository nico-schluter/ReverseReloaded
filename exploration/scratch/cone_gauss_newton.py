# Experimental verification: Gauss-Newton 4-param cone fit vs algebraic 5-param.
# Tests on the Ref_Frustum_r20-10mm.stl data (r1=20mm@z=0, r2=10mm@z=40mm, offset ~500mm in x).
# Verifies that the new approach works on 2-ring subsets where the old one fails.

import math
import re
import os

# ---------------------------------------------------------------------------
# Vector helpers (same as fitting.py)
# ---------------------------------------------------------------------------

def dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def norm(v): return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
def normalize(v):
    n = norm(v)
    return [v[0]/n, v[1]/n, v[2]/n]
def cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def sub(a, b): return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]

def make_frame(w):
    ax, ay, az = abs(w[0]), abs(w[1]), abs(w[2])
    if ax <= ay and ax <= az:   ref = [1,0,0]
    elif ay <= az:              ref = [0,1,0]
    else:                       ref = [0,0,1]
    u = normalize(cross(w, ref))
    v = cross(w, u)
    return u, v

def center_points(pts):
    n = len(pts)
    gc = [sum(p[i] for p in pts)/n for i in range(3)]
    return [[p[i]-gc[i] for i in range(3)] for p in pts], gc

def spherical_to_dir(theta, phi):
    return [math.cos(theta)*math.sin(phi), math.sin(theta)*math.sin(phi), math.cos(phi)]

# ---------------------------------------------------------------------------
# Linear algebra
# ---------------------------------------------------------------------------

def solve_nxn(A, b):
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

def lstsq_n(H, b_vec):
    nc = len(H[0])
    HtH = [[0.0]*nc for _ in range(nc)]
    Htb = [0.0]*nc
    for row, bi in zip(H, b_vec):
        for i in range(nc):
            Htb[i] += row[i] * bi
            for j in range(nc):
                HtH[i][j] += row[i] * row[j]
    x = solve_nxn(HtH, Htb)
    if x is None:
        return None, float('inf')
    resid = sum((sum(H[k][j]*x[j] for j in range(nc)) - b_vec[k])**2 for k in range(len(H)))
    return x, resid

# ---------------------------------------------------------------------------
# OLD approach: algebraic 5-param (from fitting.py)
# ---------------------------------------------------------------------------

def fit_cone_on_axis_OLD(pts, axis):
    w = normalize(axis)
    u, v = make_frame(w)
    t_v = [dot(p, w) for p in pts]
    q_x = [dot(p, u) for p in pts]
    q_y = [dot(p, v) for p in pts]
    y_v = [q_x[i]**2 + q_y[i]**2 for i in range(len(pts))]

    H = [[q_x[i], q_y[i], t_v[i]**2, t_v[i], 1.0] for i in range(len(pts))]
    x, _ = lstsq_n(H, y_v)
    if x is None:
        return None, None, float('inf'), "lstsq singular"

    A_c, B_c, C_c, D_c = x[0], x[1], x[2], x[3]
    if C_c <= 0:
        return None, None, float('inf'), f"C_c={C_c:.6g} <= 0"

    cx = A_c / 2.0
    cy = B_c / 2.0
    s  = math.sqrt(C_c)
    b  = D_c / (2.0 * s)
    t_apex = -b / s

    apex = [cx*u[i] + cy*v[i] + t_apex*w[i] for i in range(3)]

    resid = 0.0
    for i in range(len(pts)):
        r_i = math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2)
        err = r_i - s * abs(t_v[i] - t_apex)
        resid += err * err

    return apex, math.atan(s), resid, "ok"

# ---------------------------------------------------------------------------
# NEW approach: Gauss-Newton 4-param
# ---------------------------------------------------------------------------

def fit_cone_on_axis_GN(pts, axis, max_iter=20, tol=1e-12):
    w = normalize(axis)
    u, v = make_frame(w)
    n = len(pts)

    t_v = [dot(p, w) for p in pts]
    q_x = [dot(p, u) for p in pts]
    q_y = [dot(p, v) for p in pts]

    # Initialize cx, cy from centroid of projected coords
    cx = sum(q_x) / n
    cy = sum(q_y) / n

    # Initialize s, b from linear regression of r_i vs t_i
    r_v = [math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2) for i in range(n)]
    # r = s*t + b  =>  least squares 2x2
    sum_t = sum(t_v)
    sum_r = sum(r_v)
    sum_tt = sum(t*t for t in t_v)
    sum_tr = sum(t_v[i]*r_v[i] for i in range(n))
    det = n * sum_tt - sum_t * sum_t
    if abs(det) < 1e-30:
        return None, None, float('inf'), "degenerate t-values (all identical)"
    s = (n * sum_tr - sum_t * sum_r) / det
    b = (sum_r * sum_tt - sum_t * sum_tr) / det

    # Gauss-Newton iterations
    for iteration in range(max_iter):
        # Compute residuals and Jacobian
        r_v = [math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2) for i in range(n)]
        e = [r_v[i] - (s * t_v[i] + b) for i in range(n)]

        # Build J^T J (4x4) and J^T e (4x1)
        JtJ = [[0.0]*4 for _ in range(4)]
        Jte = [0.0]*4

        for i in range(n):
            ri = r_v[i]
            if ri < 1e-15:
                # Point is exactly on the axis — skip (Jacobian undefined)
                continue
            dx = -(q_x[i] - cx) / ri
            dy = -(q_y[i] - cy) / ri
            row = [dx, dy, -t_v[i], -1.0]

            for a in range(4):
                Jte[a] += row[a] * e[i]
                for bb in range(4):
                    JtJ[a][bb] += row[a] * row[bb]

        delta = solve_nxn(JtJ, Jte)
        if delta is None:
            return None, None, float('inf'), "GN solve singular"

        cx -= delta[0]
        cy -= delta[1]
        s  -= delta[2]
        b  -= delta[3]

        if sum(d*d for d in delta) < tol:
            break

    # Check for valid cone (s·t + b must not change sign = hourglass)
    t_min, t_max = min(t_v), max(t_v)
    val_lo = s * t_min + b
    val_hi = s * t_max + b
    if val_lo * val_hi < 0:
        # Apex inside data range — hourglass
        t_apex = -b / s if abs(s) > 1e-15 else float('inf')
        return None, None, float('inf'), f"hourglass: t_apex={t_apex:.4g} in [{t_min:.4g},{t_max:.4g}]"

    if abs(s) < 1e-15:
        return None, None, float('inf'), "s≈0 (cylinder, not cone)"

    t_apex = -b / s
    apex = [cx*u[i] + cy*v[i] + t_apex*w[i] for i in range(3)]
    half_angle = math.atan(abs(s))

    # Geometric residual
    resid = 0.0
    for i in range(n):
        r_i = math.sqrt((q_x[i] - cx)**2 + (q_y[i] - cy)**2)
        err = r_i - abs(s) * abs(t_v[i] - t_apex)
        resid += err * err

    return apex, half_angle, resid, f"ok (iter={iteration+1})"

# ---------------------------------------------------------------------------
# Grid search + Nelder-Mead (same structure as fitting.py)
# ---------------------------------------------------------------------------

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

def fit_cone_full(pts, method='GN'):
    """Full cone fit: grid search + Nelder-Mead over axis, inner solve per-axis."""
    cpts, gc = center_points(pts)

    inner = fit_cone_on_axis_GN if method == 'GN' else fit_cone_on_axis_OLD

    def cost(theta, phi):
        result = inner(cpts, spherical_to_dir(theta, phi))
        return float('inf') if result[0] is None else result[2]

    best, best_cost = grid_search(cost)
    if best_cost == float('inf'):
        return None, None, None, float('inf'), "all grid directions degenerate"

    opt = nelder_mead(lambda a: cost(a[0], a[1]), best)
    axis = normalize(spherical_to_dir(opt[0], opt[1]))
    result = inner(cpts, axis)
    if result[0] is None:
        axis = normalize(spherical_to_dir(best[0], best[1]))
        result = inner(cpts, axis)
    if result[0] is None:
        return None, None, None, float('inf'), f"failed: {result[3]}"

    apex_c, half_angle, resid, msg = result
    apex = [apex_c[i] + gc[i] for i in range(3)]
    rmse = math.sqrt(resid / len(pts))
    return apex, axis, half_angle, rmse, msg

# ---------------------------------------------------------------------------
# Load STL data
# ---------------------------------------------------------------------------

def load_stl(path):
    with open(path) as f:
        text = f.read()
    verts = set()
    for m in re.finditer(r'vertex\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', text):
        verts.add((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return [list(v) for v in sorted(verts, key=lambda p: (round(p[2],3), p[0], p[1]))]

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_case(name, pts, expected_half_angle_deg=None, expected_apex_z=None):
    print(f"\n{'='*60}")
    print(f"TEST: {name}  ({len(pts)} points)")
    print(f"{'='*60}")

    # Group by z to show structure
    from collections import Counter
    z_groups = Counter(round(p[2], 2) for p in pts)
    print(f"  Z-groups: {dict(sorted(z_groups.items()))}")

    for method, label in [('OLD', 'Algebraic 5-param'), ('GN', 'Gauss-Newton 4-param')]:
        apex, axis, half_angle, rmse, msg = fit_cone_full(pts, method=method)
        if apex is None:
            print(f"  {label:25s}: FAILED — {msg}")
        else:
            ha_deg = math.degrees(half_angle)
            print(f"  {label:25s}: half_angle={ha_deg:6.2f}°  RMSE={rmse:.6f}mm  "
                  f"apex=({apex[0]:.2f},{apex[1]:.2f},{apex[2]:.2f})  "
                  f"axis=({axis[0]:.3f},{axis[1]:.3f},{axis[2]:.3f})  [{msg}]")
            if expected_half_angle_deg is not None:
                print(f"    expected half_angle={expected_half_angle_deg:.2f}°  "
                      f"error={abs(ha_deg - expected_half_angle_deg):.4f}°")
            if expected_apex_z is not None:
                print(f"    expected apex_z={expected_apex_z:.2f}  "
                      f"error={abs(apex[2] - expected_apex_z):.4f}")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    stl_path = os.path.join(os.path.dirname(__file__), '..', 'Ref_Frustum_r20-10mm.stl')
    all_pts = load_stl(stl_path)
    print(f"Loaded {len(all_pts)} vertices from STL")

    # Ground truth: r1=20mm at z=0, r2=10mm at z=40mm, axis = +z
    # slope s = (20-10)/(0-40) = -0.25  (radius decreases with z)
    # |s| = 0.25, half_angle = atan(0.25) = 14.036°
    # apex at z where r=0: z_apex = 0 + 20/0.25 = 80mm
    # (center of cone is at x≈500, y=0 based on the STL data)
    expected_ha = math.degrees(math.atan(0.25))
    expected_apex_z = 80.0

    # Test 1: Full data (4 rings — should work for both)
    test_case("Full STL (4 rings)", all_pts, expected_ha, expected_apex_z)

    # Test 2: Only bottom + top rings (2 rings — old approach fails)
    two_ring_pts = [p for p in all_pts if round(p[2], 1) in (0.0, 40.0)]
    test_case("Two rings only (z=0, z=40)", two_ring_pts, expected_ha, expected_apex_z)

    # Test 3: Small selection — first 20 pts from bottom ring only (should be underdetermined)
    bottom_ring = [p for p in all_pts if round(p[2], 1) == 0.0][:20]
    test_case("Single ring (should fail — underdetermined)", bottom_ring)

    # Test 4: ~20 pts — 10 from each ring (mimics small selection in Fusion)
    bottom_10 = [p for p in all_pts if round(p[2], 1) == 0.0][:10]
    top_10 = [p for p in all_pts if round(p[2], 1) == 40.0][:10]
    small_2ring = bottom_10 + top_10
    test_case("Small 2-ring (10+10 pts)", small_2ring, expected_ha, expected_apex_z)

    # Test 5: 3 rings (bottom, middle, top)
    mid_ring = [p for p in all_pts if round(p[2], 1) == 13.33]
    three_ring = bottom_10 + mid_ring[:10] + top_10
    test_case("Three rings (10+10+10)", three_ring, expected_ha, expected_apex_z)

    # Test 6: Simulate oblique cut — rotate all points 10° around x-axis
    # This mixes z-values within each "ring"
    angle_rad = math.radians(10)
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    rotated = [[p[0], p[1]*cos_a - p[2]*sin_a, p[1]*sin_a + p[2]*cos_a] for p in two_ring_pts]
    test_case("Rotated 10° (oblique cuts, 2 original rings)", rotated, expected_ha)
