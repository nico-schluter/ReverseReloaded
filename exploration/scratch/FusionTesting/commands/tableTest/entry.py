# Table UI prototype for the surface fitting add-in.
#
# Goals:
#   - Validate TableCommandInput layout: label | type dropdown | vertex count
#   - Test add/remove row mechanics and toolbar buttons
#   - Test row selection: which row is "active" drives vertex accumulation
#   - Explore tablePresentationStyle and hasGrid options
#   - No geometry, no fitting — pure dialog mechanics only.

import adsk.core
import os
from ...lib import fusionAddInUtils as futil
from ... import config

app = adsk.core.Application.get()
ui = app.userInterface

CMD_ID = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_tableTest'
CMD_NAME = 'Table UI Test'
CMD_DESC = 'Prototype: TableCommandInput layout and interaction'
IS_PROMOTED = True

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel'
COMMAND_BESIDE_ID = 'ScriptsManagerCommand'

ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

local_handlers = []

SURFACE_TYPES = ['Plane', 'Cylinder', 'Sphere', 'Cone']

# Per-row simulated vertex counts (stand-in for real selection data).
# Index matches the current visual row order.
_row_vertex_counts: list[int] = []

# Monotonically increasing counter used to generate unique input IDs.
# Never resets within a session — prevents ID collisions after rows are deleted
# and new ones added.
_next_id = 0


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
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
    control = panel.controls.addCommand(cmd_def, COMMAND_BESIDE_ID, False)
    control.isPromoted = IS_PROMOTED


def stop():
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(PANEL_ID)
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
    global _row_vertex_counts, _next_id
    _row_vertex_counts = []
    _next_id = 0

    futil.log(f'{CMD_NAME}: command_created')
    cmd = args.command
    inputs = cmd.commandInputs

    # ---- Surface table ----
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

    # Toolbar buttons
    btn_add = inputs.addBoolValueInput('btnAddSurface', 'Add surface', False, '', False)
    btn_remove = inputs.addBoolValueInput('btnRemoveSurface', 'Remove surface', False, '', False)
    table.addToolbarCommandInput(btn_add)
    table.addToolbarCommandInput(btn_remove)

    # Start with one row so the dialog isn't empty
    _add_row(inputs, table)

    # ---- Status area ----
    inputs.addTextBoxCommandInput('tbStatus', 'Active surface', '(no row selected)', 2, True)

    futil.add_handler(cmd.inputChanged, command_input_changed, local_handlers=local_handlers)
    futil.add_handler(cmd.destroy,      command_destroy,       local_handlers=local_handlers)


# ---------------------------------------------------------------------------
# Input helpers
# ---------------------------------------------------------------------------

def _add_row(inputs: adsk.core.CommandInputs,
             table: adsk.core.TableCommandInput):
    """Append one row to the table using unique IDs, then renumber all labels."""
    visual_row = table.rowCount  # position this row will occupy

    # Col 0: row number label — narrow click target
    lbl = inputs.addStringValueInput(_new_id('surfLabel'), '', '')
    lbl.isReadOnly = True

    # Col 1: surface type dropdown
    dd = inputs.addDropDownCommandInput(
        _new_id('surfType'), '',
        adsk.core.DropDownStyles.TextListDropDownStyle
    )
    for t in SURFACE_TYPES:
        dd.listItems.add(t, t == 'Plane', '')

    # Col 2: vertex count (read-only)
    count = inputs.addStringValueInput(_new_id('surfVerts'), '', '0 pts')
    count.isReadOnly = True

    table.addCommandInput(lbl,   visual_row, 0)
    table.addCommandInput(dd,    visual_row, 1)
    table.addCommandInput(count, visual_row, 2)

    _row_vertex_counts.append(0)

    _renumber_labels(table)
    table.selectedRow = visual_row
    futil.log(f'{CMD_NAME}: added row, total rows = {table.rowCount}')


def _renumber_labels(table: adsk.core.TableCommandInput):
    """Update col-0 label text to match current visual row order (1-based)."""
    for r in range(table.rowCount):
        lbl = adsk.core.StringValueCommandInput.cast(table.getInputAtPosition(r, 0))
        if lbl:
            lbl.value = str(r + 1)


def _update_status(inputs: adsk.core.CommandInputs,
                   table: adsk.core.TableCommandInput):
    """Refresh the status text box to reflect the active row."""
    row = table.selectedRow
    tb = adsk.core.TextBoxCommandInput.cast(inputs.itemById('tbStatus'))
    if tb is None:
        return
    if row < 0 or row >= len(_row_vertex_counts):
        tb.text = '(no row selected)'
        return

    dd = adsk.core.DropDownCommandInput.cast(table.getInputAtPosition(row, 1))
    type_name = dd.selectedItem.name if dd and dd.selectedItem else '?'

    tb.text = (
        f'Active: {row + 1}  |  Type: {type_name}  |  '
        f'Vertices: {_row_vertex_counts[row]}\n'
        f'(click viewport here to accumulate vertices into this surface)'
    )


# ---------------------------------------------------------------------------
# Input changed handler
# ---------------------------------------------------------------------------

def command_input_changed(args: adsk.core.InputChangedEventArgs):
    inp = args.input
    inputs = args.inputs
    table = adsk.core.TableCommandInput.cast(inputs.itemById('surfaceTable'))

    # ---- Add row ----
    if inp.id == 'btnAddSurface':
        if table.rowCount >= 12:
            futil.log(f'{CMD_NAME}: max rows reached')
            return
        _add_row(inputs, table)
        _update_status(inputs, table)

    # ---- Remove row ----
    elif inp.id == 'btnRemoveSurface':
        row = table.selectedRow
        if row < 0:
            futil.log(f'{CMD_NAME}: remove clicked but no row selected')
            return
        if table.rowCount <= 1:
            futil.log(f'{CMD_NAME}: cannot remove last row')
            return
        table.deleteRow(row)
        _row_vertex_counts.pop(row)
        futil.log(f'{CMD_NAME}: removed row {row}, remaining = {table.rowCount}')
        _renumber_labels(table)
        table.selectedRow = max(0, row - 1)
        _update_status(inputs, table)

    # ---- Type dropdown changed — update status ----
    elif inp.id.startswith('surfType_'):
        _update_status(inputs, table)

    # ---- Simulate vertex accumulation (stand-in for mouseClick) ----
    elif inp.id == 'btnSimClick':
        row = table.selectedRow
        if 0 <= row < len(_row_vertex_counts):
            _row_vertex_counts[row] += 5
            count_input = adsk.core.StringValueCommandInput.cast(
                table.getInputAtPosition(row, 2)
            )
            if count_input:
                count_input.value = f'{_row_vertex_counts[row]} pts'
            _update_status(inputs, table)


def command_destroy(args: adsk.core.CommandEventArgs):
    futil.log(f'{CMD_NAME}: destroyed')
    global local_handlers
    local_handlers = []
