# CustomFeature prototype for the surface fitting add-in.
#
# Goals:
#   - Validate CustomFeatureDefinition registration and lifecycle
#   - Validate create → customNamedValues → timeline round-trip
#   - Validate edit (double-click in timeline) pre-populates dialog from customNamedValues
#   - Validate customFeatureCompute fires on recompute (logged only, no geometry)
#   - Table UI reused from tableTest learnings
#
# Not included yet: vertex selection, fitting, BRep geometry output.
#
# Handler lifetime note:
#   _permanent_handlers — registered in start(), cleared only in stop().
#                         Holds commandCreated and customFeatureCompute handlers.
#   local_handlers      — registered per command session, cleared in on_destroy().
#                         Holds inputChanged, execute, destroy handlers.
#   Mixing these into a single list causes the permanent handlers to be garbage-
#   collected after the first command session ends, silently breaking edit and compute.

import adsk.core
import adsk.fusion
import json
import os
from ...lib import fusionAddInUtils as futil
from ... import config

app = adsk.core.Application.get()
ui = app.userInterface

# Unique ID for this custom feature type. Must be stable across sessions.
FEATURE_DEF_ID = f'{config.COMPANY_NAME}.{config.ADDIN_NAME}.SurfaceFit'

# Two separate command IDs: one for create (toolbar), one for edit (timeline).
CREATE_CMD_ID = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_surfaceFitCreate'
EDIT_CMD_ID   = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_surfaceFitEdit'

CREATE_CMD_NAME = 'Surface Fit'
CREATE_CMD_DESC = 'Fit primitive surfaces to mesh vertices'

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'
COMMAND_BESIDE_ID = 'ScriptsManagerCommand'

ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

SURFACE_TYPES = ['Plane', 'Cylinder', 'Sphere', 'Cone']

# customNamedValues key under which the surface list JSON is stored.
SURFACES_KEY = 'surfaces'

# Permanent handlers live for the entire add-in session (cleared only in stop()).
_permanent_handlers = []
# Per-session handlers live for one command execution (cleared in on_destroy).
local_handlers = []

# Stored during start() so execute handlers can create inputs without
# re-calling CustomFeatureDefinition.create(), which would make a duplicate.
_feature_def: adsk.fusion.CustomFeatureDefinition = None

# Monotonically increasing counter for unique input IDs within one dialog session.
_next_id = 0
# Per-row simulated vertex counts for the current dialog session.
_row_vertex_counts: list[int] = []


def _new_id(prefix: str) -> str:
    global _next_id
    uid = f'{prefix}_{_next_id}'
    _next_id += 1
    return uid


# ---------------------------------------------------------------------------
# Lifecycle — called by commands/__init__.py
# ---------------------------------------------------------------------------

def start():
    # Register both CommandDefinitions before setting editCommandId — Fusion
    # validates that the edit command ID exists at assignment time.
    create_def = ui.commandDefinitions.addButtonDefinition(
        CREATE_CMD_ID, CREATE_CMD_NAME, CREATE_CMD_DESC, ICON_FOLDER
    )
    futil.add_handler(create_def.commandCreated, on_create_command_created,
                      local_handlers=_permanent_handlers)

    # Edit command — invoked by Fusion when the user double-clicks the feature
    # in the timeline. Not shown in the toolbar.
    edit_def = ui.commandDefinitions.addButtonDefinition(
        EDIT_CMD_ID, f'{CREATE_CMD_NAME} (Edit)', '', ICON_FOLDER
    )
    futil.add_handler(edit_def.commandCreated, on_edit_command_created,
                      local_handlers=_permanent_handlers)

    # Register the custom feature type. editCommandId is validated immediately,
    # so both CommandDefinitions must exist first.
    global _feature_def
    _feature_def = adsk.fusion.CustomFeatureDefinition.create(
        FEATURE_DEF_ID, CREATE_CMD_NAME, ICON_FOLDER
    )
    _feature_def.editCommandId = EDIT_CMD_ID
    futil.add_handler(_feature_def.customFeatureCompute, on_compute,
                      local_handlers=_permanent_handlers)

    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
    ctrl = panel.controls.addCommand(create_def, COMMAND_BESIDE_ID, False)
    ctrl.isPromoted = True

    futil.log(f'{CREATE_CMD_NAME}: started, feature def ID = {FEATURE_DEF_ID}')


def stop():
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
    for cmd_id in (CREATE_CMD_ID, EDIT_CMD_ID):
        ctrl = panel.controls.itemById(cmd_id)
        defn = ui.commandDefinitions.itemById(cmd_id)
        if ctrl:
            ctrl.deleteMe()
        if defn:
            defn.deleteMe()
    global _permanent_handlers, local_handlers
    _permanent_handlers = []
    local_handlers = []


# ---------------------------------------------------------------------------
# Compute handler — fires when Fusion recomputes this custom feature
# ---------------------------------------------------------------------------

def on_compute(args: adsk.fusion.CustomFeatureEventArgs):
    feat = args.customFeature
    surfaces = _load_surfaces(feat)
    futil.log(
        f'{CREATE_CMD_NAME}: compute fired for "{feat.name}" — '
        f'{len(surfaces)} surface(s): {[s["type"] for s in surfaces]}'
    )
    # No geometry output yet. Future: re-run fitting and rebuild BRep faces here.


# ---------------------------------------------------------------------------
# Create path
# ---------------------------------------------------------------------------

def on_create_command_created(args: adsk.core.CommandCreatedEventArgs):
    global _next_id, _row_vertex_counts
    _next_id = 0
    _row_vertex_counts = []

    futil.log(f'{CREATE_CMD_NAME}: create command opened')
    cmd = args.command
    _build_dialog(cmd.commandInputs, initial_surfaces=[])

    futil.add_handler(cmd.inputChanged, on_input_changed,  local_handlers=local_handlers)
    futil.add_handler(cmd.execute,      on_create_execute, local_handlers=local_handlers)
    futil.add_handler(cmd.destroy,      on_destroy,        local_handlers=local_handlers)


def on_create_execute(args: adsk.core.CommandEventArgs):
    inputs = args.command.commandInputs
    table = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))
    surfaces = _read_surfaces_from_table(table)

    design = adsk.fusion.Design.cast(app.activeProduct)

    if design.designType == adsk.fusion.DesignTypes.DirectDesignType:
        # Direct mode has no timeline — no CustomFeature possible.
        # Future: output BRep faces directly here without feature wrapper.
        futil.log(
            f'{CREATE_CMD_NAME}: direct design mode — CustomFeature not supported. '
            f'Would output {len(surfaces)} surface(s) as plain bodies.'
        )
        return

    comp = design.rootComponent
    feat_input = comp.features.customFeatures.createInput(_feature_def)
    feat = comp.features.customFeatures.add(feat_input)
    _save_surfaces(feat, surfaces)

    futil.log(
        f'{CREATE_CMD_NAME}: created "{feat.name}" with '
        f'{len(surfaces)} surface(s): {[s["type"] for s in surfaces]}'
    )


# ---------------------------------------------------------------------------
# Edit path
# ---------------------------------------------------------------------------

def on_edit_command_created(args: adsk.core.CommandCreatedEventArgs):
    global _next_id, _row_vertex_counts
    _next_id = 0
    _row_vertex_counts = []

    feat = _find_feature_at_marker()
    if feat is None:
        futil.log(f'{CREATE_CMD_NAME}: edit command fired but no feature found at marker')
        return

    futil.log(f'{CREATE_CMD_NAME}: edit command opened for "{feat.name}"')
    surfaces = _load_surfaces(feat)

    cmd = args.command
    _build_dialog(cmd.commandInputs, initial_surfaces=surfaces)

    # Stash the feature token so on_edit_execute can find the same feature.
    tok = cmd.commandInputs.addStringValueInput('_featureToken', '', feat.entityToken)
    tok.isVisible = False

    futil.add_handler(cmd.inputChanged, on_input_changed, local_handlers=local_handlers)
    futil.add_handler(cmd.execute,      on_edit_execute,  local_handlers=local_handlers)
    futil.add_handler(cmd.destroy,      on_destroy,       local_handlers=local_handlers)


def on_edit_execute(args: adsk.core.CommandEventArgs):
    inputs = args.command.commandInputs
    table = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))
    surfaces = _read_surfaces_from_table(table)

    token_input = adsk.core.StringValueCommandInput.cast(inputs.itemById('_featureToken'))
    design = adsk.fusion.Design.cast(app.activeProduct)
    feat = adsk.fusion.CustomFeature.cast(design.findEntityByToken(token_input.value))
    if feat is None:
        futil.log(f'{CREATE_CMD_NAME}: edit execute — could not find feature by token')
        return

    _save_surfaces(feat, surfaces)
    futil.log(
        f'{CREATE_CMD_NAME}: updated "{feat.name}" → '
        f'{len(surfaces)} surface(s): {[s["type"] for s in surfaces]}'
    )


# ---------------------------------------------------------------------------
# Shared input changed / destroy handlers
# ---------------------------------------------------------------------------

def on_input_changed(args: adsk.core.InputChangedEventArgs):
    inp = args.input
    inputs = args.inputs
    table = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))

    if inp.id == 'btnAddSurface':
        if table.rowCount >= 12:
            return
        _add_row(inputs, table)

    elif inp.id == 'btnRemoveSurface':
        row = table.selectedRow
        if row < 0 or table.rowCount <= 1:
            return
        table.deleteRow(row)
        _row_vertex_counts.pop(row)
        _renumber_labels(table)
        table.selectedRow = max(0, row - 1)


def on_destroy(_: adsk.core.CommandEventArgs):
    futil.log(f'{CREATE_CMD_NAME}: command destroyed')
    global local_handlers
    local_handlers = []


# ---------------------------------------------------------------------------
# Dialog builder — shared by create and edit paths
# ---------------------------------------------------------------------------

def _build_dialog(inputs: adsk.core.CommandInputs,
                  initial_surfaces: list[dict]):
    """Build the surface table dialog. initial_surfaces is [] for create,
    or a list of {"type": str} dicts loaded from customNamedValues for edit."""
    table = inputs.addTableCommandInput('surfaceTable', 'Surfaces', 3, '1:5:2')
    table.minimumVisibleRows = 3
    table.maximumVisibleRows = 7
    table.columnSpacing = 2
    table.rowSpacing = 2
    table.hasGrid = False
    table.tablePresentationStyle = (
        adsk.core.TablePresentationStyles.itemBorderTablePresentationStyle
    )
    table.isFullWidth = True

    btn_add = inputs.addBoolValueInput('btnAddSurface', 'Add surface', False, '', False)
    btn_rem = inputs.addBoolValueInput('btnRemoveSurface', 'Remove surface', False, '', False)
    table.addToolbarCommandInput(btn_add)
    table.addToolbarCommandInput(btn_rem)

    if initial_surfaces:
        for s in initial_surfaces:
            _add_row(inputs, table, preset_type=s.get('type', 'Plane'))
    else:
        _add_row(inputs, table)  # start with one blank row


# ---------------------------------------------------------------------------
# Table row helpers
# ---------------------------------------------------------------------------

def _add_row(inputs: adsk.core.CommandInputs,
             table: adsk.core.TableCommandInput,
             preset_type: str = 'Plane'):
    visual_row = table.rowCount

    lbl = inputs.addStringValueInput(_new_id('lbl'), '', '')
    lbl.isReadOnly = True

    dd = inputs.addDropDownCommandInput(
        _new_id('type'), '',
        adsk.core.DropDownStyles.TextListDropDownStyle
    )
    for t in SURFACE_TYPES:
        dd.listItems.add(t, t == preset_type, '')

    count = inputs.addStringValueInput(_new_id('verts'), '', '0 pts')
    count.isReadOnly = True

    table.addCommandInput(lbl,   visual_row, 0)
    table.addCommandInput(dd,    visual_row, 1)
    table.addCommandInput(count, visual_row, 2)

    _row_vertex_counts.append(0)
    _renumber_labels(table)
    table.selectedRow = visual_row


def _renumber_labels(table: adsk.core.TableCommandInput):
    for r in range(table.rowCount):
        lbl = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(r, 0))
        if lbl:
            lbl.value = str(r + 1)


# ---------------------------------------------------------------------------
# Serialisation helpers
# ---------------------------------------------------------------------------

def _read_surfaces_from_table(table: adsk.core.TableCommandInput) -> list[dict]:
    surfaces = []
    for r in range(table.rowCount):
        dd = adsk.core.DropDownCommandInput.cast(table.getInputAtPosition(r, 1))
        type_name = dd.selectedItem.name if dd and dd.selectedItem else 'Plane'
        surfaces.append({'type': type_name})
    return surfaces


def _save_surfaces(feat: adsk.fusion.CustomFeature, surfaces: list[dict]):
    feat.customNamedValues.addOrSetValue(SURFACES_KEY, json.dumps(surfaces))


def _load_surfaces(feat: adsk.fusion.CustomFeature) -> list[dict]:
    raw = feat.customNamedValues.value(SURFACES_KEY)
    if not raw:
        return []
    try:
        return json.loads(raw)
    except Exception as ex:
        futil.log(f'{CREATE_CMD_NAME}: failed to parse surfaces JSON: {ex}')
        return []


# ---------------------------------------------------------------------------
# Timeline helpers
# ---------------------------------------------------------------------------

def _find_feature_at_marker() -> adsk.fusion.CustomFeature | None:
    """Return the CustomFeature whose timeline position matches the current
    marker. Fusion moves the marker to immediately after the feature when
    edit is triggered."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        return None
    marker = design.timeline.markerPosition
    custom_features = design.rootComponent.features.customFeatures
    # Try marker - 1 first (marker sits just after the feature), then marker itself.
    for offset in (-1, 0):
        for i in range(custom_features.count):
            cf = custom_features.item(i)
            if cf.timelineObject.index == marker + offset:
                return cf
    return None
