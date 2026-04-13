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
#   - Cursor circle showing selection radius: needs screen-space graphics on mouseMove,
#     not yet validated. See exploration/notes/custom-graphics.md.
#   - Duplicate vertex indices: STL meshes store one vertex per triangle corner, so shared
#     positions appear multiple times. Click counts are inflated; fit quality is unaffected.

import adsk.core
import adsk.fusion
import math
import os
import time as _time
from ...lib import fusionAddInUtils as futil
from ... import config
from . import fitting

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
OUTLIER_IMAGE = os.path.join(_RES, 'resources', 'point_outlier.png')

local_handlers = []

SURFACE_TYPES = ['Auto', 'Plane', 'Cylinder', 'Cone', 'Sphere']

_MAX_RADIUS_RATIO  = 5.0    # cylinder/sphere r / point-cloud scale; above this → degenerate
_AUTO_MIN_IMPROVEMENT = 0.001  # cm (= 0.01 mm); a surface type must beat the current best by
                                # at least this much to take priority. Prevents floating-point
                                # noise in the Nelder-Mead optimiser (which reaches ~1e-5 cm on
                                # perfect data) from letting sphere or cone displace plane/cylinder
                                # on degenerate inputs like two collinear rings.

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
_row_fit_results: list = []  # list[tuple[float, set[int]] | None] — per-row (rmse_cm, outlier_mesh_indices)

# Minimum vertex count before fitting is attempted per surface type
_MIN_PTS = {'Plane': 3, 'Cylinder': 4, 'Cone': 5, 'Sphere': 4, 'Auto': 4}

_SHIFT_KEY = 0x2000000  # keyboard modifier bit for Shift
_KEY_N     = 78         # adsk.core.KeyCodes.NKeyCode


def _new_id(prefix: str) -> str:
    global _next_id
    uid = f'{prefix}_{_next_id}'
    _next_id += 1
    return uid


# ---------------------------------------------------------------------------
# Fit dispatchers — call fitting.py, package results into params dicts
# ---------------------------------------------------------------------------

def _fit_plane(pts):
    """Fit plane to pts. Returns (params_dict, rmse_cm, per_point_errors)."""
    t0 = _time.perf_counter()
    n, d, _ = fitting.fit_plane_to_points(pts)
    t1 = _time.perf_counter()
    errors = fitting.plane_point_errors(pts, n, d)
    rmse   = fitting.compute_rmse(errors)
    futil.log(f'{CMD_NAME}: fit_plane  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  RMSE={rmse*10:.3f}mm')
    return {'type': 'Plane', 'n': n, 'd': d}, rmse, errors


def _fit_cylinder(pts):
    """Fit cylinder to pts. Returns (params_dict, rmse_cm, per_point_errors) or (None, None, None)."""
    t0 = _time.perf_counter()
    origin, axis, radius, _ = fitting.fit_cylinder_to_points(pts)
    t1 = _time.perf_counter()
    if origin is None:
        futil.log(f'{CMD_NAME}: fit_cylinder  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  DEGENERATE')
        return None, None, None
    errors = fitting.cylinder_point_errors(pts, origin, axis, radius)
    rmse   = fitting.compute_rmse(errors)
    futil.log(f'{CMD_NAME}: fit_cylinder  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  RMSE={rmse*10:.3f}mm')
    return {'type': 'Cylinder', 'origin': origin, 'axis': axis, 'radius': radius}, rmse, errors


def _fit_cone(pts):
    """Fit cone to pts. Returns (params_dict, rmse_cm, per_point_errors) or (None, None, None)."""
    t0 = _time.perf_counter()
    apex, axis, half_angle, _ = fitting.fit_cone_to_points(pts)
    t1 = _time.perf_counter()
    if apex is None:
        futil.log(f'{CMD_NAME}: fit_cone  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  DEGENERATE')
        return None, None, None
    errors = fitting.cone_point_errors(pts, apex, axis, half_angle)
    rmse   = fitting.compute_rmse(errors)
    futil.log(f'{CMD_NAME}: fit_cone  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  '
              f'half_angle={math.degrees(half_angle):.1f}deg  RMSE={rmse*10:.3f}mm')
    return {'type': 'Cone', 'apex': apex, 'axis': axis, 'half_angle': half_angle}, rmse, errors


def _fit_sphere(pts):
    """Fit sphere to pts. Returns (params_dict, rmse_cm, per_point_errors) or (None, None, None)."""
    t0 = _time.perf_counter()
    center, radius, _ = fitting.fit_sphere_to_points(pts)
    t1 = _time.perf_counter()
    if center is None:
        futil.log(f'{CMD_NAME}: fit_sphere  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  DEGENERATE')
        return None, None, None
    errors = fitting.sphere_point_errors(pts, center, radius)
    rmse   = fitting.compute_rmse(errors)
    futil.log(f'{CMD_NAME}: fit_sphere  {len(pts)}pts  fit={1000*(t1-t0):.1f}ms  '
              f'radius={radius*10:.2f}mm  RMSE={rmse*10:.3f}mm')
    return {'type': 'Sphere', 'center': center, 'radius': radius}, rmse, errors


def _auto_fit(pts):
    """Fit all surface types once, return the best (params_dict, rmse_cm, per_point_errors).
    No double-fitting: the returned params are used directly for BRep creation.

    Evaluation order is plane → cylinder → cone → sphere, encoding a preference for
    simpler/more-common types.  A later type only displaces the current best if it
    improves RMSE by at least _AUTO_MIN_IMPROVEMENT (0.01 mm).  This prevents the
    algebraically-exact sphere solver from winning over Nelder-Mead cylinder on
    degenerate inputs where both fits are numerically near zero (e.g. two edge rings
    of a cylinder that happen to be equidistant from a sphere centre).
    """
    thresh = _AUTO_MIN_IMPROVEMENT
    scale  = fitting.point_cloud_scale(pts)

    plane_params, plane_rmse, plane_errors = _fit_plane(pts)
    best_params, best_rmse, best_errors = plane_params, plane_rmse, plane_errors

    if len(pts) >= _MIN_PTS['Cylinder']:
        cyl_params, cyl_rmse, cyl_errors = _fit_cylinder(pts)
        if cyl_params is not None and cyl_rmse is not None:
            if cyl_params['radius'] <= _MAX_RADIUS_RATIO * scale and cyl_rmse < best_rmse - thresh:
                best_params, best_rmse, best_errors = cyl_params, cyl_rmse, cyl_errors

    if len(pts) >= _MIN_PTS['Cone']:
        cone_params, cone_rmse, cone_errors = _fit_cone(pts)
        if cone_params is not None and cone_rmse is not None:
            # Plausibility: reject if the base radius at the data's far end exceeds
            # _MAX_RADIUS_RATIO * point-cloud scale (same guard as cylinder/sphere).
            apex, axis, ha = cone_params['apex'], cone_params['axis'], cone_params['half_angle']
            t_min, t_max = fitting.cone_axial_bounds(pts, apex, axis)
            max_r = math.tan(ha) * max(abs(t_min), abs(t_max))
            if max_r <= _MAX_RADIUS_RATIO * scale and cone_rmse < best_rmse - thresh:
                best_params, best_rmse, best_errors = cone_params, cone_rmse, cone_errors

    if len(pts) >= _MIN_PTS['Sphere']:
        sph_params, sph_rmse, sph_errors = _fit_sphere(pts)
        if sph_params is not None and sph_rmse is not None:
            if sph_params['radius'] <= _MAX_RADIUS_RATIO * scale and sph_rmse < best_rmse - thresh:
                best_params, best_rmse, best_errors = sph_params, sph_rmse, sph_errors

    futil.log(f'{CMD_NAME}: auto → {best_params["type"]}')
    return best_params, best_rmse, best_errors


def _make_plane_brep(tbm, pts, params, expansion):
    """Build plane BRep from pre-computed fit params. Returns body or None."""
    n, d = params['n'], params['d']
    hull = fitting.convex_hull_3d_on_plane(pts, n, d)
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
    t0 = _time.perf_counter()
    wire_body, _ = tbm.createWireFromCurves(lines)
    if wire_body is None:
        futil.log(f'{CMD_NAME}: plane wire creation failed')
        return None
    if expansion > 0:
        temp_face = tbm.createFaceFromPlanarWires([wire_body])
        if temp_face is not None:
            _, face_normal = temp_face.faces[0].evaluator.getNormalAtParameter(
                adsk.core.Point2D.create(0, 0)
            )
            offset_wire = wire_body.wires[0].offsetPlanarWire(face_normal, expansion, 2)
            wire_body = offset_wire if offset_wire is not None else wire_body
    surface = tbm.createFaceFromPlanarWires([wire_body])
    futil.log(f'{CMD_NAME}: plane  brep={1000*(_time.perf_counter()-t0):.1f}ms')
    if surface is None:
        futil.log(f'{CMD_NAME}: plane face creation failed')
    return surface


def _make_cylinder_brep(tbm, pts, params, expansion):
    """Build cylinder BRep from pre-computed fit params. Returns body or None."""
    origin, axis, radius = params['origin'], params['axis'], params['radius']
    t_min, t_max = fitting.cylinder_axial_bounds(pts, origin, axis)
    p1 = [origin[i] + (t_min - expansion) * axis[i] for i in range(3)]
    p2 = [origin[i] + (t_max + expansion) * axis[i] for i in range(3)]
    ax_vec = adsk.core.Vector3D.create(axis[0], axis[1], axis[2])
    c1 = adsk.core.Circle3D.createByCenter(adsk.core.Point3D.create(*p1), ax_vec, radius)
    c2 = adsk.core.Circle3D.createByCenter(adsk.core.Point3D.create(*p2), ax_vec, radius)
    t0 = _time.perf_counter()
    wire1, _ = tbm.createWireFromCurves([c1])
    wire2, _ = tbm.createWireFromCurves([c2])
    if wire1 is None or wire2 is None:
        futil.log(f'{CMD_NAME}: cylinder wire creation failed')
        return None
    surface = tbm.createRuledSurface(wire1.wires.item(0), wire2.wires.item(0))
    futil.log(f'{CMD_NAME}: cylinder  brep={1000*(_time.perf_counter()-t0):.1f}ms')
    if surface is None:
        futil.log(f'{CMD_NAME}: cylinder ruled surface creation failed')
    return surface


def _make_cone_brep(tbm, pts, params, expansion):
    """Build cone/frustum BRep from pre-computed fit params. Returns body or None.

    Uses createCylinderOrCone(pointOne, r1, pointTwo, r2) — same API as cylinder
    but r1 ≠ r2. The apex is found from the fit; axial bounds are relative to it.
    """
    apex       = params['apex']
    axis       = params['axis']
    half_angle = params['half_angle']
    s          = math.tan(half_angle)

    t_min, t_max = fitting.cone_axial_bounds(pts, apex, axis)

    # Normalise axis so data is on the positive side.
    # Use abs comparison: the fitter's abs(t_rel) residual is orientation-symmetric,
    # so the optimizer may converge to either direction.  The condition t_max < 0
    # handles the "all-negative" case, but misses "apex exactly at t_max=0" (pointed
    # cone where the apex vertex is the outermost data point in that direction).
    # abs(t_min) > abs(t_max) catches both: if most data is in the negative half, flip.
    futil.log(f'{CMD_NAME}: cone axial_bounds  t_min={t_min*10:.3f}mm  t_max={t_max*10:.3f}mm')
    if abs(t_min) > abs(t_max):
        axis = [-axis[0], -axis[1], -axis[2]]
        t_min, t_max = -t_max, -t_min   # both now positive, min < max
        futil.log(f'{CMD_NAME}: cone axis flipped  t_min={t_min*10:.3f}mm  t_max={t_max*10:.3f}mm')

    # Extend by expansion, clamped so we don't go past the apex (t1 can't go below 0).
    t1 = max(0.0, t_min - expansion)
    t2 = t_max + expansion

    r1 = s * t1
    r2 = s * t2

    # When expansion pushes t1 to 0 the cone comes to a true apex point.
    # createCylinderOrCone may or may not accept r=0; use a tiny epsilon to be safe.
    _R_MIN = 1e-4  # 0.001 mm — imperceptible but avoids potential API rejection of r=0
    r1 = max(_R_MIN, r1)

    pt1 = adsk.core.Point3D.create(
        apex[0] + t1 * axis[0],
        apex[1] + t1 * axis[1],
        apex[2] + t1 * axis[2],
    )
    pt2 = adsk.core.Point3D.create(
        apex[0] + t2 * axis[0],
        apex[1] + t2 * axis[1],
        apex[2] + t2 * axis[2],
    )

    length = math.sqrt((pt2.x-pt1.x)**2 + (pt2.y-pt1.y)**2 + (pt2.z-pt1.z)**2)
    futil.log(f'{CMD_NAME}: cone params  half_angle={math.degrees(half_angle):.2f}deg  '
              f't1={t1*10:.3f}mm  t2={t2*10:.3f}mm  r1={r1*10:.3f}mm  r2={r2*10:.3f}mm  '
              f'length={length*10:.3f}mm  '
              f'pt1=({pt1.x*10:.2f},{pt1.y*10:.2f},{pt1.z*10:.2f})mm  '
              f'pt2=({pt2.x*10:.2f},{pt2.y*10:.2f},{pt2.z*10:.2f})mm')

    if length < 1e-6:
        futil.log(f'{CMD_NAME}: cone degenerate — zero length body')
        return None

    t0 = _time.perf_counter()
    try:
        body = tbm.createCylinderOrCone(pt1, r1, pt2, r2)
    except RuntimeError as e:
        futil.log(f'{CMD_NAME}: cone createCylinderOrCone failed: {e}')
        return None
    futil.log(f'{CMD_NAME}: cone brep={1000*(_time.perf_counter()-t0):.1f}ms')
    if body is None:
        futil.log(f'{CMD_NAME}: cone body creation failed (returned None)')
    return body


def _make_sphere_brep(tbm, params):
    """Build sphere BRep from pre-computed fit params. Returns body or None.

    Expansion is not applied — a sphere is a closed surface with no boundary
    edge to push outward; adding to the radius would resize it, not extend it.
    """
    center = params['center']
    radius = params['radius']
    pt = adsk.core.Point3D.create(center[0], center[1], center[2])
    futil.log(f'{CMD_NAME}: sphere params  radius={radius*10:.3f}mm  '
              f'center=({center[0]*10:.2f},{center[1]*10:.2f},{center[2]*10:.2f})mm  (no expansion)')
    t0 = _time.perf_counter()
    try:
        body = tbm.createSphere(pt, radius)
    except RuntimeError as e:
        futil.log(f'{CMD_NAME}: sphere createSphere failed: {e}')
        return None
    futil.log(f'{CMD_NAME}: sphere brep={1000*(_time.perf_counter()-t0):.1f}ms')
    if body is None:
        futil.log(f'{CMD_NAME}: sphere body creation failed (returned None)')
    return body


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
    global _row_verts, _next_id, _mesh_pts, _mesh_count, _cg_group, _row_fit_results
    _row_verts       = []
    _next_id         = 0
    _mesh_pts        = None
    _mesh_count      = 0
    _cg_group        = None
    _row_fit_results = []
    # _px_radius and _expansion are intentionally NOT reset — they persist between
    # command invocations within the same Fusion session.

    cmd    = args.command
    inputs = cmd.commandInputs

    # ---- Mesh body selection ----
    sel = inputs.addSelectionInput('selMesh', 'Mesh body', 'Select mesh body to pick vertices from')
    sel.addSelectionFilter('MeshBodies')
    sel.setSelectionLimits(0, 1)

    # ---- Surface table ----
    table = inputs.addTableCommandInput('surfaceTable', 'Surfaces', 3, '3:5:2')
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

    # ---- Selection radius slider ----
    sldr = inputs.addIntegerSliderCommandInput('sldrRadius', 'Radius (px)', 5, 200)
    sldr.valueOne = _px_radius

    # ---- Expansion distance slider (0–5 mm, 0.1 mm steps) ----
    exp_sldr = inputs.addFloatSliderCommandInput('expDist', 'Expansion', 'mm', 0.0, 0.5)
    exp_sldr.spinStep = 0.1   # 0.1 mm (spinStep is in unitType units)
    exp_sldr.valueOne = _expansion

    # ---- Status text — full-width, at the bottom ----
    # Updating this triggers inputChanged → executePreview.
    # isFullWidth requires an empty label to span the full dialog width.
    tb = inputs.addTextBoxCommandInput(
        'tbStatus', '',
        '(select a mesh body, then click the viewport to add vertices)',
        2, True
    )
    tb.isFullWidth = True

    futil.add_handler(cmd.activate,       command_activate,        local_handlers=local_handlers)
    futil.add_handler(cmd.keyDown,        command_key_down,        local_handlers=local_handlers)
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
        dd.listItems.add(t, t == 'Auto', '')

    count = inputs.addStringValueInput(_new_id('surfVerts'), '', '0 pts')
    count.isReadOnly = True

    table.addCommandInput(lbl,   row, 0)
    table.addCommandInput(dd,    row, 1)
    table.addCommandInput(count, row, 2)

    _row_verts.append(set())
    _row_fit_results.append(None)
    _update_row_labels(table)
    table.selectedRow = row


def _update_row_labels(table: adsk.core.TableCommandInput):
    """Refresh col 0 (RMSE when fit exists, row number otherwise) and col 2 (count + outlier count)."""
    for r in range(table.rowCount):
        lbl = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(r, 0))
        cnt = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(r, 2))

        fit = _row_fit_results[r] if r < len(_row_fit_results) else None
        n_verts = len(_row_verts[r]) if r < len(_row_verts) else 0

        if lbl:
            if fit is not None and fit.get('failed'):
                lbl.value = 'failed'
            elif fit is not None:
                rmse_mm = fit['rmse'] * 10.0  # cm → mm
                lbl.value = f'{rmse_mm:.3f} mm'
            else:
                lbl.value = str(r + 1)

        if cnt:
            if fit is not None and not fit.get('failed') and fit['outliers']:
                cnt.value = f'{n_verts} pts · {len(fit["outliers"])}⚠'
            else:
                cnt.value = f'{n_verts} pts'


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
# Activate handler — auto-select the mesh body if exactly one is visible
# ---------------------------------------------------------------------------
# Note: command.activate fires not just on dialog open but also when Fusion
# re-activates the command after browser interactions (e.g. showing/hiding
# bodies). As a result, unhiding a mesh while the dialog is open can trigger
# an auto-selection. This is intentional — the logic is "if exactly one
# visible mesh and nothing selected, select it", which is valid regardless of
# what triggered the check. The selectionCount guard prevents overriding an
# existing selection. The one known edge case is: user manually clears the
# selection, then toggles browser visibility → re-auto-selects. Accepted as
# low-probability and low-harm.

def command_activate(args: adsk.core.CommandEventArgs):
    cmd    = adsk.core.Command.cast(args.firingEvent.sender)
    inputs = cmd.commandInputs
    sel    = adsk.core.SelectionCommandInput.cast(inputs.itemById('selMesh'))
    if sel is None or sel.selectionCount > 0:
        return

    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        return

    root    = design.rootComponent
    visible = []

    for mb in root.meshBodies:
        if mb.isVisible:
            visible.append(mb)

    for occ in root.allOccurrences:
        for mb in occ.component.meshBodies:
            proxy = mb.createForAssemblyContext(occ)
            if proxy is not None and proxy.isVisible:
                visible.append(proxy)

    if len(visible) == 1:
        sel.addSelection(visible[0])
        futil.log(f'{CMD_NAME}: auto-selected sole visible mesh body')


# ---------------------------------------------------------------------------
# Key down handler — hotkeys while the command dialog is open
# ---------------------------------------------------------------------------

def command_key_down(args: adsk.core.KeyboardEventArgs):
    if args.keyCode != _KEY_N or args.modifierMask != 0:
        return
    cmd    = adsk.core.Command.cast(args.firingEvent.sender)
    inputs = cmd.commandInputs
    table  = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))
    if table is None or table.rowCount >= 12:
        return
    _add_row(inputs, table)
    _update_status(inputs, table)


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
        _row_fit_results.pop(row)
        _update_row_labels(table)
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
            for i in range(len(_row_fit_results)):
                _row_fit_results[i] = None
            # executePreview returns early when _mesh_pts is None, so _update_row_labels
            # would never be reached — call it directly to clear col 0 RMSE labels.
            _update_row_labels(table)
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

    # Invalidate fit result — col 0 reverts to row number until next executePreview
    if row < len(_row_fit_results):
        _row_fit_results[row] = None

    # Update count label in table col 2 (outlier count cleared since fit is now stale)
    count_inp = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(row, 2))
    if count_inp:
        count_inp.value = f'{len(_row_verts[row])} pts'

    # Revert col 0 to row number now that fit is stale
    lbl = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(row, 0))
    if lbl:
        lbl.value = str(row + 1)

    # Update status — this triggers inputChanged → executePreview
    _update_status(inputs, table)


# ---------------------------------------------------------------------------
# Execute — fit surfaces and add BRep bodies to the design
# ---------------------------------------------------------------------------

def command_execute(args: adsk.core.CommandEventArgs):
    # executePreview with isValidResult=True handles the commit — nothing to do here.
    futil.log(f'{CMD_NAME}: execute (preview already committed)')


# ---------------------------------------------------------------------------
# Execute preview — rebuild graphics + fit surfaces + commit via isValidResult
# ---------------------------------------------------------------------------

def command_execute_preview(args: adsk.core.CommandEventArgs):
    t_start = _time.perf_counter()
    _rebuild_graphics()
    t_graphics = _time.perf_counter()

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
        type_name = dd.selectedItem.name if dd and dd.selectedItem else 'Auto'

        min_pts = _MIN_PTS.get(type_name, 3)
        if len(verts) < min_pts:
            continue

        t_row0 = _time.perf_counter()

        # --- Cache check: skip fitting if vertex set AND surface type are unchanged ---
        verts_frozen = frozenset(verts)
        cache = _row_fit_results[row] if row < len(_row_fit_results) else None
        if (cache is not None
                and cache['verts_snapshot'] == verts_frozen
                and cache.get('type_name') == type_name):
            if cache.get('failed'):
                futil.log(f'{CMD_NAME}: row {row}  cache_hit  (failed, {len(verts)} pts unchanged)')
                continue
            params      = cache['params']
            rmse        = cache['rmse']
            outlier_mesh = cache['outliers']
            futil.log(f'{CMD_NAME}: row {row}  cache_hit  ({len(verts)} pts unchanged)')
        else:
            # Full fit — rebuild pts list and run fitter
            verts_list = list(verts)
            pts = [[_mesh_pts[i*3], _mesh_pts[i*3+1], _mesh_pts[i*3+2]] for i in verts_list]

            if type_name == 'Auto':
                params, rmse, errors = _auto_fit(pts)
            elif type_name == 'Plane':
                params, rmse, errors = _fit_plane(pts)
            elif type_name == 'Cylinder':
                params, rmse, errors = _fit_cylinder(pts)
                if params is None:
                    _row_fit_results[row] = {'verts_snapshot': verts_frozen, 'type_name': type_name, 'failed': True}
                    continue
            elif type_name == 'Cone':
                params, rmse, errors = _fit_cone(pts)
                if params is None:
                    _row_fit_results[row] = {'verts_snapshot': verts_frozen, 'type_name': type_name, 'failed': True}
                    continue
            elif type_name == 'Sphere':
                params, rmse, errors = _fit_sphere(pts)
                if params is None:
                    _row_fit_results[row] = {'verts_snapshot': verts_frozen, 'type_name': type_name, 'failed': True}
                    continue
            else:
                futil.log(f'{CMD_NAME}: "{type_name}" not yet implemented')
                continue

            outlier_local = fitting.outlier_indices(errors)
            outlier_mesh  = {verts_list[j] for j in outlier_local}
            futil.log(f'{CMD_NAME}: row {row}  outliers={len(outlier_mesh)}')

            if row < len(_row_fit_results):
                _row_fit_results[row] = {
                    'verts_snapshot': verts_frozen,
                    'type_name':      type_name,
                    'params':         params,
                    'rmse':           rmse,
                    'outliers':       outlier_mesh,
                }

        # --- BRep creation from params (always needed — preview bodies don't persist) ---
        # pts may not be built yet on a cache hit; build it now (O(N), ~0ms)
        pts = [[_mesh_pts[i*3], _mesh_pts[i*3+1], _mesh_pts[i*3+2]] for i in verts]

        if params['type'] == 'Plane':
            body = _make_plane_brep(tbm, pts, params, _expansion)
        elif params['type'] == 'Cylinder':
            body = _make_cylinder_brep(tbm, pts, params, _expansion)
        elif params['type'] == 'Cone':
            body = _make_cone_brep(tbm, pts, params, _expansion)
        elif params['type'] == 'Sphere':
            body = _make_sphere_brep(tbm, params)
        else:
            futil.log(f'{CMD_NAME}: BRep creation not implemented for type "{params["type"]}"')
            body = None

        t_row1 = _time.perf_counter()
        futil.log(f'{CMD_NAME}: row {row}  total_row={1000*(t_row1-t_row0):.1f}ms')

        if body is not None:
            temp_bodies.append(body)

    t_fit = _time.perf_counter()
    _update_row_labels(table)

    if not temp_bodies:
        futil.log(f'{CMD_NAME}: preview  graphics={1000*(t_graphics-t_start):.1f}ms  '
                  f'fitting={1000*(t_fit-t_graphics):.1f}ms  (no bodies)')
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

    t_add = _time.perf_counter()
    args.isValidResult = True
    futil.log(f'{CMD_NAME}: preview  graphics={1000*(t_graphics-t_start):.1f}ms  '
              f'fitting={1000*(t_fit-t_graphics):.1f}ms  '
              f'add_bodies={1000*(t_add-t_fit):.1f}ms  '
              f'total={1000*(t_add-t_start):.1f}ms  ({len(temp_bodies)} surfaces)')


def _rebuild_graphics():
    global _cg_group

    t0 = _time.perf_counter()
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

    pts = _mesh_pts  # captured after the None-guard above; never None here

    def _add_point_set(group, vis, image):
        if not vis:
            return
        flat = []
        for vi in vis:
            flat.append(pts[vi * 3])
            flat.append(pts[vi * 3 + 1])
            flat.append(pts[vi * 3 + 2])
        coords = adsk.fusion.CustomGraphicsCoordinates.create(flat)
        ps = group.addPointSet(
            coords, [],
            adsk.fusion.CustomGraphicsPointTypes.UserDefinedCustomGraphicsPointType,
            image
        )
        ps.depthPriority = 1
        ps.isSelectable  = False

    for row_idx, verts in enumerate(_row_verts):
        if not verts:
            continue

        fit = _row_fit_results[row_idx] if row_idx < len(_row_fit_results) else None
        outlier_mesh = fit['outliers'] if (fit and fit['outliers']) else set()

        normal_verts  = verts - outlier_mesh
        outlier_verts = verts & outlier_mesh  # only outliers that are still selected

        _add_point_set(group, normal_verts,  POINT_IMAGES[row_idx % len(POINT_IMAGES)])
        _add_point_set(group, outlier_verts, OUTLIER_IMAGE)

    _cg_group = group
    t1 = _time.perf_counter()
    futil.log(f'{CMD_NAME}: _rebuild_graphics  total={1000*(t1-t0):.1f}ms')


# ---------------------------------------------------------------------------
# Destroy handler — clean up graphics
# ---------------------------------------------------------------------------

def command_destroy(args: adsk.core.CommandEventArgs):
    global _cg_group, local_handlers
    if _cg_group is not None and _cg_group.isValid:
        _cg_group.deleteMe()
    _cg_group      = None
    local_handlers = []
