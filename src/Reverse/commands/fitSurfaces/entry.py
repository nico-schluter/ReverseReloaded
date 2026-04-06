# Fit Surfaces — main command.
#
# Dialog layout:
#   [SelectionInput: Mesh body]
#   [TableCommandInput: row# | type dropdown | vertex count]  (+/- toolbar)
#   [TextBox: status]
#   [IntegerSlider: Radius (px) — screen-space selection radius]
#   [FloatSlider:   Expansion (mm) — surface overextension for watertight intersections]
#
# Vertex selection event flow:
#   mesh selected → inputChanged → load nodeCoordinatesAsDouble + world transform → hasFocus=False
#   click viewport → mouseClick → update _row_verts[active_row] → update status/count
#                 → triggers inputChanged → triggers executePreview
#
# executePreview event flow:
#   rebuild vertex highlight graphics (CustomGraphicsGroup, one point set per row)
#   for each row with enough points: fit surface, create temporary BRep body
#   add all bodies to design (BaseFeature wrapper in parametric mode)
#   set isValidResult=True → preview IS the committed result, execute is a no-op
#
# Known open issues (deferred):
#   - Per-row vertex colour: SolidColorEffect does not tint user-defined point sprites —
#     all rows show in the image's native colour. See exploration/notes/custom-graphics.md.
#   - Cursor circle showing selection radius: needs screen-space graphics on mouseMove,
#     not yet validated. See exploration/notes/custom-graphics.md.
#   - Duplicate vertex indices: STL meshes store one vertex per triangle corner, so shared
#     positions appear multiple times. Click counts are inflated; fit quality is unaffected.

import adsk.core
import adsk.fusion
import math as _math
import os
from ...lib import fusionAddInUtils as futil
from ... import config

app = adsk.core.Application.get()
ui  = app.userInterface

CMD_ID   = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_fitSurfaces'
CMD_NAME = 'Fit Surfaces'
CMD_DESC = 'Fit primitive surfaces to selected mesh vertices'
IS_PROMOTED = True

WORKSPACE_ID      = 'FusionSolidEnvironment'
PANEL_ID          = 'ParaMeshCreatePanel'
COMMAND_BESIDE_ID = 'ParaMeshTessellateCommand'

ICON_FOLDER  = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')
_RES         = os.path.dirname(os.path.abspath(__file__))
POINT_IMAGES = [
    os.path.join(_RES, 'resources', f'point_{i:02d}.png') for i in range(12)
]

local_handlers = []

SURFACE_TYPES = ['Plane', 'Cylinder']  # Sphere, Cone: added when fitting is implemented

# ---------------------------------------------------------------------------
# Module-level state (reset on commandCreated)
# ---------------------------------------------------------------------------

_row_verts: list = []   # list[set[int]] — per-row selected vertex indices
_next_id: int    = 0    # monotonic counter for unique input IDs
_mesh_pts        = None # flat list of doubles [x0,y0,z0, ...] in world space
_mesh_count: int = 0    # vertex count
_px_radius: int  = 20   # selection radius in screen pixels
_expansion: float = 0.1 # surface overextension distance in cm (internal units); default 1 mm
_cg_group        = None # adsk.fusion.CustomGraphicsGroup for vertex highlights

# Minimum vertex count before fitting is attempted per surface type
_MIN_PTS = {'Plane': 3, 'Cylinder': 4}

_SHIFT_KEY = 0x2000000  # keyboard modifier bit for Shift


def _new_id(prefix: str) -> str:
    global _next_id
    uid = f'{prefix}_{_next_id}'
    _next_id += 1
    return uid


# ---------------------------------------------------------------------------
# Fitting algorithms (pure Python — no numpy, no scipy)
# Ported from exploration/scratch/pure_python_fitting.py
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


def _fit_plane_on_axis(pts, n):
    projs = [_dot(n, p) for p in pts]
    d = sum(projs) / len(projs)
    return d, sum((p-d)**2 for p in projs)


def fit_plane_to_points(pts):
    """Returns (n, d, resid) — unit normal, offset, residual."""
    best, _ = _grid_search(lambda t, p: _fit_plane_on_axis(pts, _spherical_to_dir(t, p))[1])
    opt = _nelder_mead(lambda a: _fit_plane_on_axis(pts, _spherical_to_dir(a[0], a[1]))[1], best)
    n = _normalize(_spherical_to_dir(opt[0], opt[1]))
    d, resid = _fit_plane_on_axis(pts, n)
    return n, d, resid


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
    """Returns (t_min, t_max) along axis from origin."""
    projs = [_dot(_sub(p, origin), axis) for p in pts]
    return min(projs), max(projs)


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
# Lifecycle
# ---------------------------------------------------------------------------

def start():
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC, ICON_FOLDER)
    futil.add_handler(cmd_def.commandCreated, command_created)

    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID) if workspace else None
    if panel is None:
        # Fall back to the Scripts/Add-ins panel which always exists
        futil.log(f'{CMD_NAME}: {PANEL_ID!r} not found — falling back to Scripts panel')
        workspace = ui.workspaces.itemById('FusionSolidEnvironment')
        panel = workspace.toolbarPanels.itemById('SolidScriptsAddinsPanel')

    control = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)
    control.isPromoted = IS_PROMOTED


def stop():
    # Search all workspaces for our control rather than assuming placement succeeded
    for ws_id in (WORKSPACE_ID, 'FusionSolidEnvironment'):
        workspace = ui.workspaces.itemById(ws_id)
        if workspace is None:
            continue
        for i in range(workspace.toolbarPanels.count):
            panel = workspace.toolbarPanels.item(i)
            ctrl  = panel.controls.itemById(CMD_ID)
            if ctrl:
                ctrl.deleteMe()
    defn = ui.commandDefinitions.itemById(CMD_ID)
    if defn:
        defn.deleteMe()


# ---------------------------------------------------------------------------
# Command creation — build the dialog
# ---------------------------------------------------------------------------

def command_created(args: adsk.core.CommandCreatedEventArgs):
    global _row_verts, _next_id, _mesh_pts, _mesh_count, _cg_group
    _row_verts  = []
    _next_id    = 0
    _mesh_pts   = None
    _mesh_count = 0
    _cg_group   = None
    # _px_radius and _expansion are intentionally NOT reset — they persist between
    # command invocations within the same Fusion session.

    cmd    = args.command
    inputs = cmd.commandInputs

    # ---- Mesh body selection ----
    sel = inputs.addSelectionInput('selMesh', 'Mesh body', 'Select mesh body to pick vertices from')
    sel.addSelectionFilter('MeshBodies')
    sel.setSelectionLimits(0, 1)

    # ---- Surface table ----
    table = inputs.addTableCommandInput('surfaceTable', 'Surfaces', 3, '1:5:2')
    table.minimumVisibleRows = 3
    table.maximumVisibleRows = 7
    table.columnSpacing      = 2
    table.rowSpacing         = 2
    table.hasGrid            = False
    table.tablePresentationStyle = (
        adsk.core.TablePresentationStyles.itemBorderTablePresentationStyle
    )
    table.isFullWidth = True

    btn_add    = inputs.addBoolValueInput('btnAddSurface',    'Add surface',    False, '', False)
    btn_remove = inputs.addBoolValueInput('btnRemoveSurface', 'Remove surface', False, '', False)
    table.addToolbarCommandInput(btn_add)
    table.addToolbarCommandInput(btn_remove)

    _add_row(inputs, table)

    # ---- Status text (updating this triggers inputChanged → executePreview) ----
    inputs.addTextBoxCommandInput(
        'tbStatus', 'Status',
        '(select a mesh body, then click the viewport to add vertices)',
        2, True
    )

    # ---- Selection radius slider ----
    sldr = inputs.addIntegerSliderCommandInput('sldrRadius', 'Radius (px)', 5, 200)
    sldr.valueOne = _px_radius

    # ---- Expansion distance slider (0–5 mm, 0.1 mm steps) ----
    exp_sldr = inputs.addFloatSliderCommandInput('expDist', 'Expansion', 'mm', 0.0, 0.5)
    exp_sldr.spinStep = 0.1   # 0.1 mm (spinStep is in unitType units)
    exp_sldr.valueOne = _expansion

    futil.add_handler(cmd.inputChanged,   command_input_changed,   local_handlers=local_handlers)
    futil.add_handler(cmd.mouseClick,     command_mouse_click,     local_handlers=local_handlers)
    futil.add_handler(cmd.execute,        command_execute,         local_handlers=local_handlers)
    futil.add_handler(cmd.executePreview, command_execute_preview, local_handlers=local_handlers)
    futil.add_handler(cmd.destroy,        command_destroy,         local_handlers=local_handlers)


# ---------------------------------------------------------------------------
# Table helpers
# ---------------------------------------------------------------------------

def _add_row(inputs: adsk.core.CommandInputs, table: adsk.core.TableCommandInput):
    """Append one row using unique IDs, then renumber labels."""
    row = table.rowCount

    lbl = inputs.addStringValueInput(_new_id('surfLabel'), '', '')
    lbl.isReadOnly = True

    dd = inputs.addDropDownCommandInput(
        _new_id('surfType'), '',
        adsk.core.DropDownStyles.TextListDropDownStyle
    )
    for t in SURFACE_TYPES:
        dd.listItems.add(t, t == 'Plane', '')

    count = inputs.addStringValueInput(_new_id('surfVerts'), '', '0 pts')
    count.isReadOnly = True

    table.addCommandInput(lbl,   row, 0)
    table.addCommandInput(dd,    row, 1)
    table.addCommandInput(count, row, 2)

    _row_verts.append(set())
    _renumber_labels(table)
    table.selectedRow = row


def _renumber_labels(table: adsk.core.TableCommandInput):
    for r in range(table.rowCount):
        lbl = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(r, 0))
        if lbl:
            lbl.value = str(r + 1)


def _update_status(inputs: adsk.core.CommandInputs, table: adsk.core.TableCommandInput):
    """Refresh status text. Setting tb.text triggers inputChanged → executePreview."""
    tb = adsk.core.TextBoxCommandInput.cast(inputs.itemById('tbStatus'))
    if tb is None:
        return

    if _mesh_pts is None:
        tb.text = '(select a mesh body, then click the viewport to add vertices)'
        return

    row = table.selectedRow
    if row < 0 or row >= len(_row_verts):
        tb.text = f'Mesh loaded: {_mesh_count} vertices  |  (no row selected)'
        return

    dd        = adsk.core.DropDownCommandInput.cast(table.getInputAtPosition(row, 1))
    type_name = dd.selectedItem.name if dd and dd.selectedItem else '?'
    total     = sum(len(s) for s in _row_verts)
    tb.text   = (
        f'Mesh: {_mesh_count} pts  |  Surface {row + 1} ({type_name}): '
        f'{len(_row_verts[row])} pts\n'
        f'Total selected: {total} pts across {table.rowCount} surface(s)'
    )


# ---------------------------------------------------------------------------
# World transform helpers
# ---------------------------------------------------------------------------

def _get_world_matrix(mesh_body: adsk.fusion.MeshBody) -> adsk.core.Matrix3D:
    """Return the Matrix3D that transforms mesh-body local coords to world/root space."""
    comp = mesh_body.parentComponent
    root = comp.parentDesign.rootComponent
    mat  = adsk.core.Matrix3D.create()  # identity
    if comp == root:
        return mat
    occs = root.allOccurrencesByComponent(comp)
    if not occs or len(occs) == 0:
        return mat
    occ       = occs[0]
    occ_names = occ.fullPathName.split('+')
    path_occs = [root.allOccurrences.itemByName(name) for name in occ_names]
    path_occs.reverse()
    for path_occ in path_occs:
        if path_occ:
            mat.transformBy(path_occ.transform)
    return mat


def _apply_world_matrix(pts_flat, count: int, mat: adsk.core.Matrix3D) -> list:
    """Apply Matrix3D to flat [x,y,z,...] array. Returns a new flat list."""
    m      = mat.asArray()  # 16 floats, row-major 4x4
    result = [0.0] * (count * 3)
    for i in range(count):
        x = pts_flat[i * 3]
        y = pts_flat[i * 3 + 1]
        z = pts_flat[i * 3 + 2]
        result[i * 3]     = m[0]*x + m[1]*y + m[2]*z + m[3]
        result[i * 3 + 1] = m[4]*x + m[5]*y + m[6]*z + m[7]
        result[i * 3 + 2] = m[8]*x + m[9]*y + m[10]*z + m[11]
    return result


# ---------------------------------------------------------------------------
# Input changed handler
# ---------------------------------------------------------------------------

def command_input_changed(args: adsk.core.InputChangedEventArgs):
    global _mesh_pts, _mesh_count, _px_radius, _expansion

    inp    = args.input
    inputs = args.inputs
    table  = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))

    if inp.id == 'btnAddSurface':
        if table.rowCount >= 12:
            return
        _add_row(inputs, table)
        _update_status(inputs, table)

    elif inp.id == 'btnRemoveSurface':
        row = table.selectedRow
        if row < 0 or table.rowCount <= 1:
            return
        table.deleteRow(row)
        _row_verts.pop(row)
        _renumber_labels(table)
        table.selectedRow = max(0, row - 1)
        _update_status(inputs, table)

    elif inp.id == 'selMesh':
        sel_inp = adsk.core.SelectionCommandInput.cast(inp)
        if sel_inp.selectionCount == 1:
            mesh_body = adsk.fusion.MeshBody.cast(sel_inp.selection(0).entity)
            sel_inp.hasFocus = False

            dm    = mesh_body.displayMesh
            count = dm.nodeCount
            raw   = dm.nodeCoordinatesAsDouble
            futil.log(f'{CMD_NAME}: mesh selected — {count} vertices, applying world transform')

            mat       = _get_world_matrix(mesh_body)
            world_pts = _apply_world_matrix(raw, count, mat)

            _mesh_pts   = world_pts
            _mesh_count = count
        else:
            _mesh_pts   = None
            _mesh_count = 0
            for s in _row_verts:
                s.clear()
            for r in range(table.rowCount):
                c = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(r, 2))
                if c:
                    c.value = '0 pts'

        _update_status(inputs, table)

    elif inp.id == 'sldrRadius':
        _px_radius = adsk.core.IntegerSliderCommandInput.cast(inp).valueOne

    elif inp.id == 'expDist':
        _expansion = adsk.core.FloatSliderCommandInput.cast(inp).valueOne

    elif inp.id.startswith('surfType_'):
        _update_status(inputs, table)


# ---------------------------------------------------------------------------
# Mouse click handler — screen-space vertex selection
# ---------------------------------------------------------------------------

def command_mouse_click(args: adsk.core.MouseEventArgs):
    cmd    = adsk.core.Command.cast(args.firingEvent.sender)
    inputs = cmd.commandInputs
    table  = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))
    if table is None:
        return

    row = table.selectedRow
    if row < 0 or row >= len(_row_verts):
        return

    if _mesh_pts is None:
        return

    vp = args.viewport
    if vp is None:
        return

    cx, cy   = args.viewportPosition.x, args.viewportPosition.y
    m        = vp.modelToViewSpaceTransform.asArray()
    cam_type = vp.camera.cameraType
    r_sq     = _px_radius * _px_radius
    # Shift = modifier bit 0x2000000 (33554432)
    shift    = bool(args.keyboardModifiers & _SHIFT_KEY)

    pts = _mesh_pts
    n   = _mesh_count
    hit = set()
    for i in range(n):
        x, y, z = pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]
        sx = m[0]*x + m[1]*y + m[2]*z + m[3]
        sy = m[4]*x + m[5]*y + m[6]*z + m[7]
        if cam_type != 0:
            w = m[12]*x + m[13]*y + m[14]*z + m[15]
            if w != 0.0:
                sx /= w
                sy /= w
        dx, dy = sx - cx, sy - cy
        if dx*dx + dy*dy <= r_sq:
            hit.add(i)

    if shift:
        _row_verts[row] -= hit
    else:
        _row_verts[row] |= hit

    # Update count label in table col 2
    count_inp = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(row, 2))
    if count_inp:
        count_inp.value = f'{len(_row_verts[row])} pts'

    # Update status — this triggers inputChanged → executePreview
    _update_status(inputs, table)


# ---------------------------------------------------------------------------
# Execute — fit surfaces and add BRep bodies to the design
# ---------------------------------------------------------------------------

def command_execute(args: adsk.core.CommandEventArgs):
    # executePreview with isValidResult=True handles the commit — nothing to do here.
    futil.log(f'{CMD_NAME}: execute (preview already committed)')


def _create_surface_body(tbm, type_name, pts, expansion):
    if type_name == 'Plane':
        return _create_plane_body(tbm, pts, expansion)
    elif type_name == 'Cylinder':
        return _create_cylinder_body(tbm, pts, expansion)
    else:
        futil.log(f'{CMD_NAME}: "{type_name}" not yet implemented')
        return None


def _create_plane_body(tbm, pts, expansion):
    n, d, resid = fit_plane_to_points(pts)
    futil.log(f'{CMD_NAME}: plane n=[{n[0]:.4f},{n[1]:.4f},{n[2]:.4f}] d={d:.4f} resid={resid:.3e}')

    hull = convex_hull_3d_on_plane(pts, n, d)
    if len(hull) < 3:
        futil.log(f'{CMD_NAME}: plane hull degenerate ({len(hull)} pts)')
        return None

    nv = len(hull)
    lines = [
        adsk.core.Line3D.create(
            adsk.core.Point3D.create(hull[i][0],        hull[i][1],        hull[i][2]),
            adsk.core.Point3D.create(hull[(i+1)%nv][0], hull[(i+1)%nv][1], hull[(i+1)%nv][2])
        )
        for i in range(nv)
    ]

    wire_body, _ = tbm.createWireFromCurves(lines)
    if wire_body is None:
        futil.log(f'{CMD_NAME}: plane wire creation failed')
        return None

    if expansion > 0:
        # Get the actual face normal so the offset direction is always outward.
        temp_face = tbm.createFaceFromPlanarWires([wire_body])
        if temp_face is not None:
            _, face_normal = temp_face.faces[0].evaluator.getNormalAtParameter(
                adsk.core.Point2D.create(0, 0)
            )
            offset_wire = wire_body.wires[0].offsetPlanarWire(face_normal, expansion, 2)
            wire_body = offset_wire if offset_wire is not None else wire_body

    surface = tbm.createFaceFromPlanarWires([wire_body])
    if surface is None:
        futil.log(f'{CMD_NAME}: plane face creation failed')
    return surface


def _create_cylinder_body(tbm, pts, expansion):
    origin, axis, radius, resid = fit_cylinder_to_points(pts)
    if origin is None:
        futil.log(f'{CMD_NAME}: cylinder fit degenerate — skipping')
        return None
    t_min, t_max = cylinder_axial_bounds(pts, origin, axis)
    futil.log(
        f'{CMD_NAME}: cylinder r={radius:.4f} '
        f'axis=[{axis[0]:.4f},{axis[1]:.4f},{axis[2]:.4f}] '
        f'extent=[{t_min:.4f},{t_max:.4f}] resid={resid:.3e}'
    )

    p1 = [origin[i] + (t_min - expansion) * axis[i] for i in range(3)]
    p2 = [origin[i] + (t_max + expansion) * axis[i] for i in range(3)]
    ax_vec = adsk.core.Vector3D.create(axis[0], axis[1], axis[2])

    c1 = adsk.core.Circle3D.createByCenter(
        adsk.core.Point3D.create(p1[0], p1[1], p1[2]), ax_vec, radius
    )
    c2 = adsk.core.Circle3D.createByCenter(
        adsk.core.Point3D.create(p2[0], p2[1], p2[2]), ax_vec, radius
    )

    wire1, _ = tbm.createWireFromCurves([c1])
    wire2, _ = tbm.createWireFromCurves([c2])
    if wire1 is None or wire2 is None:
        futil.log(f'{CMD_NAME}: cylinder wire creation failed')
        return None

    surface = tbm.createRuledSurface(wire1.wires.item(0), wire2.wires.item(0))
    if surface is None:
        futil.log(f'{CMD_NAME}: cylinder ruled surface creation failed')
    return surface


# ---------------------------------------------------------------------------
# Execute preview — rebuild graphics + fit surfaces + commit via isValidResult
# ---------------------------------------------------------------------------

def command_execute_preview(args: adsk.core.CommandEventArgs):
    _rebuild_graphics()

    if _mesh_pts is None:
        return

    cmd    = adsk.core.Command.cast(args.firingEvent.sender)
    inputs = cmd.commandInputs
    table  = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))
    if table is None:
        return

    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        return

    root   = design.rootComponent
    bodies = root.bRepBodies
    tbm    = adsk.fusion.TemporaryBRepManager.get()

    temp_bodies = []

    for row in range(table.rowCount):
        verts = _row_verts[row] if row < len(_row_verts) else set()
        dd = adsk.core.DropDownCommandInput.cast(table.getInputAtPosition(row, 1))
        type_name = dd.selectedItem.name if dd and dd.selectedItem else 'Plane'

        min_pts = _MIN_PTS.get(type_name, 3)
        if len(verts) < min_pts:
            continue

        pts  = [[_mesh_pts[i*3], _mesh_pts[i*3+1], _mesh_pts[i*3+2]] for i in verts]
        body = _create_surface_body(tbm, type_name, pts, _expansion)

        if body is not None:
            temp_bodies.append(body)

    if not temp_bodies:
        return

    # Add surfaces; wrap in BaseFeature for parametric mode
    if design.designType:
        base_feature = root.features.baseFeatures.add()
        base_feature.startEdit()
    else:
        base_feature = None

    for body in temp_bodies:
        if base_feature:
            bodies.add(body, base_feature)
        else:
            bodies.add(body)

    if base_feature:
        base_feature.finishEdit()

    # Mark this preview result as the committed result — execute becomes a no-op
    args.isValidResult = True
    futil.log(f'{CMD_NAME}: preview committed — {len(temp_bodies)} surface(s)')


def _rebuild_graphics():
    global _cg_group

    if _cg_group is not None and _cg_group.isValid:
        _cg_group.deleteMe()
    _cg_group = None

    if _mesh_pts is None or not any(_row_verts):
        return

    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        return

    root  = design.rootComponent
    group = root.customGraphicsGroups.add()

    total = 0
    for row_idx, verts in enumerate(_row_verts):
        if not verts:
            continue

        flat = []
        pts  = _mesh_pts
        for vi in verts:
            flat.append(pts[vi * 3])
            flat.append(pts[vi * 3 + 1])
            flat.append(pts[vi * 3 + 2])

        coords = adsk.fusion.CustomGraphicsCoordinates.create(flat)
        ps     = group.addPointSet(
            coords, [],
            adsk.fusion.CustomGraphicsPointTypes.UserDefinedCustomGraphicsPointType,
            POINT_IMAGES[row_idx % len(POINT_IMAGES)]
        )
        ps.depthPriority = 1
        ps.isSelectable  = False
        total += len(verts)

    _cg_group = group


# ---------------------------------------------------------------------------
# Destroy handler — clean up graphics
# ---------------------------------------------------------------------------

def command_destroy(args: adsk.core.CommandEventArgs):
    global _cg_group, local_handlers
    if _cg_group is not None and _cg_group.isValid:
        _cg_group.deleteMe()
    _cg_group      = None
    local_handlers = []
