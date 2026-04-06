# Fit Surfaces — main command.
#
# Dialog layout:
#   [SelectionInput: Mesh body]
#   [TableCommandInput: row# | type dropdown | vertex count]  (+/- toolbar)
#   [TextBox: status]
#   [IntegerSpinner: selection radius in pixels]
#
# Vertex selection event flow:
#   mesh selected → inputChanged → load nodeCoordinatesAsDouble + world transform → hasFocus=False
#   click viewport → mouseClick → update _row_verts[active_row] → update status text
#                 → triggers inputChanged → triggers executePreview → redraws point graphics

import adsk.core
import adsk.fusion
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
PANEL_ID          = 'SolidScriptsAddinsPanel'
COMMAND_BESIDE_ID = 'ScriptsManagerCommand'

ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

local_handlers = []

SURFACE_TYPES = ['Plane', 'Cylinder', 'Sphere', 'Cone']

# One color per row (R, G, B) — up to 12 rows
_ROW_COLORS = [
    (255, 140,   0),
    ( 30, 180, 255),
    ( 50, 220,  50),
    (255, 255,   0),
    (255,  60, 255),
    (  0, 230, 200),
    (255,  80,  80),
    (160, 100, 255),
    (255, 180, 100),
    (100, 255, 160),
    (220, 220, 220),
    (255, 160, 200),
]

# ---------------------------------------------------------------------------
# Module-level state (reset on commandCreated)
# ---------------------------------------------------------------------------

_row_verts: list  = []   # list[set[int]] — per-row selected vertex indices
_next_id: int     = 0    # monotonic counter for unique input IDs
_mesh_pts         = None # flat list of doubles [x0,y0,z0, ...] in world space
_mesh_count: int  = 0    # vertex count
_cg_group         = None # adsk.fusion.CustomGraphicsGroup for point highlights
_px_radius: int   = 20   # selection radius in screen pixels


def _new_id(prefix: str) -> str:
    global _next_id
    uid = f'{prefix}_{_next_id}'
    _next_id += 1
    return uid


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

def start():
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC, ICON_FOLDER)
    futil.add_handler(cmd_def.commandCreated, command_created)

    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel     = workspace.toolbarPanels.itemById(PANEL_ID)
    control   = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)
    control.isPromoted = IS_PROMOTED


def stop():
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel     = workspace.toolbarPanels.itemById(PANEL_ID)
    ctrl = panel.controls.itemById(CMD_ID)
    defn = ui.commandDefinitions.itemById(CMD_ID)
    if ctrl:
        ctrl.deleteMe()
    if defn:
        defn.deleteMe()


# ---------------------------------------------------------------------------
# Command creation — build the dialog
# ---------------------------------------------------------------------------

def command_created(args: adsk.core.CommandCreatedEventArgs):
    global _row_verts, _next_id, _mesh_pts, _mesh_count, _cg_group, _px_radius
    _row_verts  = []
    _next_id    = 0
    _mesh_pts   = None
    _mesh_count = 0
    _cg_group   = None
    _px_radius  = 20

    futil.log(f'{CMD_NAME}: command_created')
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

    # ---- Selection radius ----
    inputs.addIntegerSpinnerCommandInput('spnRadius', 'Radius (px)', 5, 200, 1, _px_radius)

    futil.add_handler(cmd.inputChanged,   command_input_changed,   local_handlers=local_handlers)
    futil.add_handler(cmd.mouseClick,     command_mouse_click,     local_handlers=local_handlers)
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
    futil.log(f'{CMD_NAME}: added row {row}, total={table.rowCount}')


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
    global _mesh_pts, _mesh_count, _px_radius

    inp    = args.input
    inputs = args.inputs
    table  = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))

    if inp.id == 'btnAddSurface':
        if table.rowCount >= 12:
            futil.log(f'{CMD_NAME}: max rows reached')
            return
        _add_row(inputs, table)
        _update_status(inputs, table)

    elif inp.id == 'btnRemoveSurface':
        row = table.selectedRow
        if row < 0:
            futil.log(f'{CMD_NAME}: remove — no row selected')
            return
        if table.rowCount <= 1:
            futil.log(f'{CMD_NAME}: remove — cannot delete last row')
            return
        table.deleteRow(row)
        _row_verts.pop(row)
        _renumber_labels(table)
        table.selectedRow = max(0, row - 1)
        futil.log(f'{CMD_NAME}: removed row {row}, remaining={table.rowCount}')
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
            futil.log(f'{CMD_NAME}: transform done, mesh ready')

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
            futil.log(f'{CMD_NAME}: mesh deselected, all selections cleared')

        _update_status(inputs, table)

    elif inp.id == 'spnRadius':
        spn = adsk.core.IntegerSpinnerCommandInput.cast(inp)
        _px_radius = spn.value
        futil.log(f'{CMD_NAME}: radius = {_px_radius} px')

    elif inp.id.startswith('surfType_'):
        _update_status(inputs, table)


# ---------------------------------------------------------------------------
# Mouse click handler — screen-space vertex selection
# ---------------------------------------------------------------------------

def command_mouse_click(args: adsk.core.MouseEventArgs):
    import time

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
    shift    = bool(args.keyboardModifiers & 0x2000000)

    t0  = time.time()
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

    elapsed = (time.time() - t0) * 1000
    futil.log(
        f'{CMD_NAME}: click ({cx:.0f},{cy:.0f})px r={_px_radius}px '
        f'hit={len(hit)}/{n} in {elapsed:.1f}ms shift={shift}'
    )

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
# Execute preview — rebuild custom graphics point highlights
# ---------------------------------------------------------------------------

def command_execute_preview(args: adsk.core.CommandEventArgs):
    _rebuild_graphics()


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
            adsk.fusion.CustomGraphicsPointTypes.PointCloudCustomGraphicsPointType,
            ''
        )
        r, g, b = _ROW_COLORS[row_idx % len(_ROW_COLORS)]
        ps.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(
            adsk.core.Color.create(r, g, b, 255)
        )
        ps.depthPriority = 1
        ps.isSelectable  = False
        total += len(verts)

    _cg_group = group
    futil.log(f'{CMD_NAME}: graphics rebuilt — {total} pts across {len(_row_verts)} row(s)')


# ---------------------------------------------------------------------------
# Destroy handler — clean up graphics
# ---------------------------------------------------------------------------

def command_destroy(args: adsk.core.CommandEventArgs):
    global _cg_group, local_handlers
    futil.log(f'{CMD_NAME}: destroy')
    if _cg_group is not None and _cg_group.isValid:
        _cg_group.deleteMe()
    _cg_group     = None
    local_handlers = []
