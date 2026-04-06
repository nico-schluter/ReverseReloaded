import adsk.core
import os
from ...lib import fusionAddInUtils as futil
from ... import config

app = adsk.core.Application.get()
ui = app.userInterface

CMD_ID = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_vertexSelTest'
CMD_NAME = 'Vertex Selection Test'
CMD_Description = 'Tests mesh vertex selection filter strings and mouseClick approach'
IS_PROMOTED = True

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'
COMMAND_BESIDE_ID = 'ScriptsManagerCommand'

ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

local_handlers = []

CANDIDATE_FILTERS = [
    'Vertices',
    'MeshBodies',
    'Bodies',
    'SolidBodies',
    'Faces',
    'Edges',
    'SketchPoints',
    'ConstructionPoints',
    'TSplineBodies',
    'MeshVertices',
    'BRepVertices',
    'Occurrences',
]


def probe_filters(inputs):
    """Try each candidate filter string on a throwaway selection input.
    Returns dict {filter_string: bool}.
    """
    results = {}
    probe = inputs.addSelectionInput('_filterProbe', '_probe', '_probe')
    for f in CANDIDATE_FILTERS:
        probe.clearSelectionFilter()
        try:
            ok = probe.addSelectionFilter(f)
        except Exception as ex:
            futil.log(f'{CMD_NAME}: probe "{f}" threw: {ex}')
            ok = False
        results[f] = ok
    probe.deleteMe()
    return results


def start():
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_Description, ICON_FOLDER)
    futil.add_handler(cmd_def.commandCreated, command_created)

    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
    control = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)
    control.isPromoted = IS_PROMOTED


def stop():
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
    command_control = panel.controls.itemById(CMD_ID)
    command_definition = ui.commandDefinitions.itemById(CMD_ID)
    if command_control:
        command_control.deleteMe()
    if command_definition:
        command_definition.deleteMe()


def command_created(args: adsk.core.CommandCreatedEventArgs):
    futil.log(f'{CMD_NAME}: command_created fired')
    inputs = args.command.commandInputs

    # Confirmed from log1.txt: 'Vertices' is accepted but only works for BRep vertices,
    # not mesh body vertices. No mesh-vertex filter exists. mouseClick is still the way.
    # This session tests screen-space selection and nodeCoordinatesAsDouble.

    # Mesh body selection — after selecting, hasFocus is cleared so mouseClick fires freely
    sel_mesh = inputs.addSelectionInput('selMesh', 'Mesh body', 'Select mesh body first')
    sel_mesh.addSelectionFilter('MeshBodies')
    sel_mesh.setSelectionLimits(0, 1)

    inputs.addTextBoxCommandInput('tbMeshInfo', 'Mesh info', '(select a mesh body)', 2, True)
    inputs.addTextBoxCommandInput('tbClickInfo', 'Last click', '(click viewport after mesh selected)', 2, True)

    futil.add_handler(args.command.inputChanged, command_input_changed, local_handlers=local_handlers)
    futil.add_handler(args.command.mouseClick,   command_mouse_click,   local_handlers=local_handlers)
    futil.add_handler(args.command.destroy,      command_destroy,       local_handlers=local_handlers)


# Tracks mesh vertices loaded from the selected mesh body
_mesh_pts_double = None   # flat double array [x0,y0,z0, x1,y1,z1, ...]
_mesh_pt_count = 0


def command_input_changed(args: adsk.core.InputChangedEventArgs):
    global _mesh_pts_double, _mesh_pt_count
    inp = args.input
    if inp.id != 'selMesh':
        return

    if inp.selectionCount == 0:
        _mesh_pts_double = None
        _mesh_pt_count = 0
        args.inputs.itemById('tbMeshInfo').text = '(mesh deselected)'
        return

    mesh_body = adsk.fusion.MeshBody.cast(inp.selection(0).entity)

    # Take focus away so mouseClick fires without deselecting the mesh
    inp.hasFocus = False

    # Load vertices via nodeCoordinatesAsDouble (flat array, no Point3D overhead)
    dm = mesh_body.displayMesh
    _mesh_pts_double = dm.nodeCoordinatesAsDouble   # [x0,y0,z0, x1,y1,z1, ...]
    _mesh_pt_count = dm.nodeCount

    tb = args.inputs.itemById('tbMeshInfo')
    tb.text = (
        f'nodeCount: {_mesh_pt_count}\n'
        f'nodeCoordinatesAsDouble len: {len(_mesh_pts_double)}\n'
        f'first vertex: ({_mesh_pts_double[0]:.4f}, {_mesh_pts_double[1]:.4f}, {_mesh_pts_double[2]:.4f})'
    )
    futil.log(f'{CMD_NAME}: mesh loaded — nodeCount={_mesh_pt_count}, '
              f'doubles len={len(_mesh_pts_double)}')


def command_mouse_click(args: adsk.core.MouseEventArgs):
    import math
    import time

    vp = args.viewport
    if vp is None or _mesh_pts_double is None:
        return

    click_vp = args.viewportPosition  # Point2D — viewport pixel coords
    cx, cy = click_vp.x, click_vp.y

    # ----------------------------------------------------------------
    # Approach A: pure Python bulk transform via asArray()
    # Extract the 4x4 matrix once, apply to all vertices manually.
    # Row-major: x' = m[0]*x + m[1]*y + m[2]*z + m[3]
    #            y' = m[4]*x + m[5]*y + m[6]*z + m[7]
    #            w' = m[12]*x + m[13]*y + m[14]*z + m[15]  (for perspective divide)
    # ----------------------------------------------------------------
    m = vp.modelToViewSpaceTransform.asArray()
    cam_type = vp.camera.cameraType  # 0 = orthographic, non-zero = perspective

    RADIUS_PX = 20  # selection radius in screen pixels

    t0 = time.time()
    pts = _mesh_pts_double
    selected = []
    for i in range(_mesh_pt_count):
        x, y, z = pts[i*3], pts[i*3+1], pts[i*3+2]
        sx = m[0]*x + m[1]*y + m[2]*z + m[3]
        sy = m[4]*x + m[5]*y + m[6]*z + m[7]
        if cam_type != 0:
            w = m[12]*x + m[13]*y + m[14]*z + m[15]
            sx /= w
            sy /= w
        dx, dy = sx - cx, sy - cy
        if dx*dx + dy*dy <= RADIUS_PX*RADIUS_PX:
            selected.append(i)
    t_bulk = time.time() - t0

    # ----------------------------------------------------------------
    # Approach B: per-point API call via modelToViewSpace() — sample 5 verts
    # to verify approach A gives matching results
    # ----------------------------------------------------------------
    t1 = time.time()
    mismatches = 0
    for i in range(min(5, _mesh_pt_count)):
        x, y, z = pts[i*3], pts[i*3+1], pts[i*3+2]
        p = adsk.core.Point3D.create(x, y, z)
        api_view = vp.modelToViewSpace(p)
        sx = m[0]*x + m[1]*y + m[2]*z + m[3]
        sy = m[4]*x + m[5]*y + m[6]*z + m[7]
        err = math.sqrt((sx - api_view.x)**2 + (sy - api_view.y)**2)
        if err > 1.0:
            mismatches += 1
            futil.log(f'{CMD_NAME}: MISMATCH v{i}: bulk=({sx:.1f},{sy:.1f}) api=({api_view.x:.1f},{api_view.y:.1f}) err={err:.2f}px')
    t_api_sample = time.time() - t1

    msg = (
        f'click=({cx:.0f},{cy:.0f})px  radius={RADIUS_PX}px\n'
        f'selected {len(selected)}/{_mesh_pt_count} vertices\n'
        f'bulk transform: {t_bulk*1000:.2f}ms  '
        f'(API sample 5 pts: {t_api_sample*1000:.2f}ms)\n'
        f'matrix mismatches (>1px): {mismatches}/5\n'
        f'first 5 selected indices: {selected[:5]}'
    )
    futil.log(f'{CMD_NAME}: mouseClick: {msg.replace(chr(10), "  ")}')

    cmd = args.firingEvent.sender
    if cmd:
        tb = adsk.core.Command.cast(cmd).commandInputs.itemById('tbClickInfo')
        if tb:
            tb.text = msg


def command_destroy(args: adsk.core.CommandEventArgs):
    futil.log(f'{CMD_NAME}: command destroyed')
    global local_handlers
    local_handlers = []
