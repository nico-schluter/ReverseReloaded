# Add Test Meshes — debug command.
#
# Creates a set of primitive BRep solids and matching mesh bodies in the active
# design.  Intended to be run in a blank/scrap Fusion file.  The BRep bodies
# serve as ground truth; the mesh bodies are the inputs for the Fit Surfaces
# command.
#
# Surface types produced:
#   1. Sphere
#   2. Full cylinder
#   3. Partial cylinder (~270 degrees, one quadrant removed)
#   4. Flat plane (thin rectangular slab)
#   5. Cone with apex
#   6. Frustum (truncated cone, no apex)
#
# Only registered when config.DEBUG is True.

import adsk.core
import adsk.fusion
from ...lib import fusionAddInUtils as futil
from ... import config

app = adsk.core.Application.get()
ui  = app.userInterface

CMD_ID   = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_addTestMeshes'
CMD_NAME = 'Add Test Meshes'
CMD_DESC = 'Add primitive mesh bodies for testing Fit Surfaces (debug only)'

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID     = 'ParaMeshCreatePanel'

local_handlers = []


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

def start():
    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC, '')
    futil.add_handler(cmd_def.commandCreated, command_created)

    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID) if workspace else None
    if panel is None:
        workspace = ui.workspaces.itemById('FusionSolidEnvironment')
        panel = workspace.toolbarPanels.itemById('SolidScriptsAddinsPanel')

    panel.controls.addCommand(cmd_def)


def stop():
    for ws_id in (WORKSPACE_ID, 'FusionSolidEnvironment'):
        workspace = ui.workspaces.itemById(ws_id)
        if workspace is None:
            continue
        for i in range(workspace.toolbarPanels.count):
            p = workspace.toolbarPanels.item(i)
            ctrl = p.controls.itemById(CMD_ID)
            if ctrl:
                ctrl.deleteMe()
    defn = ui.commandDefinitions.itemById(CMD_ID)
    if defn:
        defn.deleteMe()


# ---------------------------------------------------------------------------
# Command events
# ---------------------------------------------------------------------------

def command_created(args: adsk.core.CommandCreatedEventArgs):
    cmd = args.command
    # No dialog inputs needed — just execute immediately.
    futil.add_handler(cmd.execute, command_execute, local_handlers=local_handlers)


def command_execute(args: adsk.core.CommandEventArgs):
    try:
        _create_test_bodies()
        futil.log(f'{CMD_NAME}: all bodies created successfully')
    except Exception:
        import traceback
        msg = traceback.format_exc()
        futil.log(f'{CMD_NAME}: ERROR\n{msg}')
        ui.messageBox(f'Add Test Meshes failed:\n{msg}')


# ---------------------------------------------------------------------------
# Body creation
# ---------------------------------------------------------------------------

def _tessellate(body):
    """Tessellate a BRepBody to high quality. Returns (coords, indices, normals)
    as plain Python lists ready for meshBodies.addByTriangleMeshData."""
    calc = body.meshManager.createMeshCalculator()
    calc.setQuality(adsk.fusion.TriangleMeshQualityOptions.HighQualityTriangleMesh)
    mesh = calc.calculate()
    if mesh is None:
        raise RuntimeError('tessellation returned None')
    coords  = list(mesh.nodeCoordinatesAsDouble)
    indices = list(mesh.nodeIndices)
    normals = list(mesh.normalVectorsAsDouble)
    futil.log(f'{CMD_NAME}:   tessellated  nodes={mesh.nodeCount}  tris={mesh.triangleCount}')
    return coords, indices, normals


def _create_test_bodies():
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        raise RuntimeError('no active Fusion design')

    comp = design.rootComponent
    tbm  = adsk.fusion.TemporaryBRepManager.get()

    P = adsk.core.Point3D.create
    V = adsk.core.Vector3D.create
    OBB = adsk.core.OrientedBoundingBox3D.create

    # Horizontal spacing between shape centres along X (cm).
    STEP = 10.0

    # Collect (temp_body, brep_label, mesh_label) before touching the design.
    shapes = []
    cx = 0.0

    # 1. Sphere  r=2 cm
    futil.log(f'{CMD_NAME}: building Sphere')
    body = tbm.createSphere(P(cx, 0, 2.0), 2.0)
    shapes.append((body, 'Ref_Sphere_r20mm', 'Mesh_Sphere_r20mm'))

    # 2. Full cylinder  r=1.5 cm  h=4 cm  axis Z
    cx += STEP
    futil.log(f'{CMD_NAME}: building Cylinder (full)')
    body = tbm.createCylinderOrCone(P(cx, 0, 0), 1.5, P(cx, 0, 4.0), 1.5)
    shapes.append((body, 'Ref_Cylinder_Full_r15mm', 'Mesh_Cylinder_Full_r15mm'))

    # 3. Partial cylinder (~270°): full cylinder minus one quadrant (+X,+Y)
    cx += STEP
    futil.log(f'{CMD_NAME}: building Cylinder (partial, 270 deg)')
    cyl = tbm.createCylinderOrCone(P(cx, 0, 0), 1.5, P(cx, 0, 4.0), 1.5)
    # Cutting box covers the (+X, +Y) quadrant of the cylinder cross-section.
    # Box centre at (cx+0.75, 0.75, 2.0), half-extents 0.75 x 0.75 x 2.5 cm.
    cut_box = tbm.createBox(OBB(
        P(cx + 0.75, 0.75, 2.0),
        V(1, 0, 0), V(0, 1, 0),
        1.5, 1.5, 5.0
    ))
    tbm.booleanOperation(cyl, cut_box, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    shapes.append((cyl, 'Ref_Cylinder_Partial_r15mm', 'Mesh_Cylinder_Partial_r15mm'))

    # 4. Flat plane slab  8×8×0.4 cm (use top face for plane fitting)
    cx += STEP
    futil.log(f'{CMD_NAME}: building Plane slab')
    body = tbm.createBox(OBB(
        P(cx, 0, 0.2),
        V(1, 0, 0), V(0, 1, 0),
        8.0, 8.0, 0.4
    ))
    shapes.append((body, 'Ref_Plane_80x80mm', 'Mesh_Plane_80x80mm'))

    # 5. Cone with apex  base r=2 cm at Z=0, apex at Z=5 cm
    cx += STEP
    futil.log(f'{CMD_NAME}: building Cone (with apex)')
    body = tbm.createCylinderOrCone(P(cx, 0, 0), 2.0, P(cx, 0, 5.0), 0.0)
    shapes.append((body, 'Ref_Cone_Full_r20mm', 'Mesh_Cone_Full_r20mm'))

    # 6. Frustum (truncated cone, no apex)  r1=2 cm at Z=0, r2=1 cm at Z=4 cm
    cx += STEP
    futil.log(f'{CMD_NAME}: building Frustum (no apex)')
    body = tbm.createCylinderOrCone(P(cx, 0, 0), 2.0, P(cx, 0, 4.0), 1.0)
    shapes.append((body, 'Ref_Frustum_r20-10mm', 'Mesh_Frustum_r20-10mm'))

    # Tessellate all temp bodies while they are still accessible outside the design.
    mesh_data = []
    for temp_body, brep_label, mesh_label in shapes:
        futil.log(f'{CMD_NAME}: tessellating {brep_label}')
        coords, indices, normals = _tessellate(temp_body)
        mesh_data.append((temp_body, brep_label, mesh_label, coords, indices, normals))

    # Add everything to the design inside a single BaseFeature.
    # Per the API docs, mesh bodies created in a parametric design must be
    # created while a BaseFeature is in its edit state.
    base = comp.features.baseFeatures.add()
    base.startEdit()
    try:
        for temp_body, brep_label, mesh_label, coords, indices, normals in mesh_data:
            # Reference solid
            brep = comp.bRepBodies.add(temp_body, base)
            brep.name = brep_label
            futil.log(f'{CMD_NAME}: added BRep   {brep_label}')

            # Corresponding mesh body (normalIndexList=[] → normals are 1-1 with coords)
            mesh_body = comp.meshBodies.addByTriangleMeshData(
                coords, indices, normals, []
            )
            if mesh_body is None:
                futil.log(f'{CMD_NAME}: WARNING mesh creation failed for {mesh_label}')
            else:
                mesh_body.name = mesh_label
                futil.log(f'{CMD_NAME}: added mesh   {mesh_label}')
    finally:
        base.finishEdit()
