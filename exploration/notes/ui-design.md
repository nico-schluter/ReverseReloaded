# Add-In UI Design

## Core decisions

### Single command, surface type dropdown
One toolbar button opens one command dialog. A dropdown selects the surface type
(Plane / Cylinder / Sphere / Cone). Later, an "Auto" option at the top will pick
the best type for the selected vertices — same workflow, no new command needed.

Changing the type dropdown re-fits the already-selected vertices rather than
clearing them. This lets the user quickly compare how different surface types fit
the same region.

### Multi-surface per execution
One command execution produces N surfaces. The dialog uses a `TableCommandInput`
to manage the list:

| Column | Content | Type |
|--------|---------|------|
| 0 | Row number (1, 2, 3…) | `StringValueInput` (read-only) |
| 1 | Surface type | `DropDownCommandInput` |
| 2 | Vertex count | `StringValueInput` (read-only) |

Toolbar (bottom of table): "Add surface" / "Remove surface" `BoolValueInput` buttons.

The **selected row** (via `table.selectedRow`) is the active surface. All vertex
clicks (mouseClick handler) accumulate into that surface's vertex set. Switching
rows immediately switches which set is being populated.

### No CustomFeature in v1
Wrapping the output in a `CustomFeature` (timeline node, re-editable) proved to
have significant integration friction and is not required for core functionality.
The first version will produce plain BRep bodies without a timeline feature wrapper.
CustomFeature support is deferred to a future release.

### No auto-trimming in v1
Surfaces are output as independent untrimmed BRep faces. The user runs Boundary
Fill to assemble them into a shell or solid. This is Fusion's job.

`booleanOperation(target, tool, UnionBooleanType)` on open sheet bodies is
undocumented behaviour — needs experimentation before committing. Deferred.

Watertight detection (all edges valence 2 → attempt solid stitch) also deferred.

## CustomFeature notes (deferred — DO NOT REVISIT)

> **Update 2026-04-28:** the "validated as correct in principle" claim below was
> wrong. A follow-up investigation in PhosEDM-WCAM
> (`exploration/scratch/customfeature_spike/`) confirmed the Custom Feature
> API's edit lifecycle is broken in current Fusion: the right-click "Edit
> Feature" menu item is missing entirely and double-click on the timeline node
> does nothing — true even for Autodesk's own canonical CustomPocket sample.
> The API has been "Preview" for 5+ years with no public adoption and no path
> to general availability. Autodesk's currently-shipping add-ins (e.g.
> SinterBox) avoid it. See
> [phosedm-wcam findings](../../../../phosedm-wcam/repo/exploration/notes/customfeature_findings.md)
> for the full investigation.
>
> If a future ReverseReloaded release wants timeline persistence, take a
> different path — `Document.attributes` + `entityToken` for storage with
> auto-remapping entity references, or `adsk.cam.Setups` if the workflow fits
> the Manufacture workspace.

The design below is validated as correct in principle — the friction was in
integrating all the pieces, not in any individual API call being broken. When this
is revisited, start from the prototype in `exploration/scratch/FusionTesting/commands/customFeatureTest/`.

Key storage design: each `CustomFeature` stores per-surface data in
`customNamedValues` as a JSON string:
```json
[
  {"type": "plane",    "vertices": [0, 3, 7, 12, ...], "params": {...}},
  {"type": "cylinder", "vertices": [44, 45, ...],       "params": {...}}
]
```

The source `MeshBody` is declared as a dependency via `addDependency` so
`customFeatureCompute` fires whenever the mesh changes. The compute handler
re-runs fitting from the stored vertex indices and rebuilds the BRep faces.

`CustomFeatureDefinition.editCommandId` points back to the edit command, which
re-opens the table dialog pre-populated from `customNamedValues`.

### CustomFeature gotchas

- **`CustomFeatures.add()` throws `make params invalid`** if the add-in manifest has no
  `id` field. The Fusion docs say the id is optional — it is not. Add a UUID and
  **restart Fusion** (reload add-in is not enough). See `exploration/forumPostAboutCustomFeature.txt`.
- **`CustomFeature` requires parametric design mode.** `designType == 1` is direct mode;
  the feature cannot be created there. Detect this and show a clear error — do not
  auto-switch, which would silently destroy the user's timeline.
- **Handler lifetime:** `customFeatureCompute` and the edit command's `commandCreated`
  handler must stay alive for the entire add-in session. Store them in a `_permanent_handlers`
  list cleared only in `stop()` — not in a per-command `local_handlers` list cleared by
  `on_destroy`, or Fusion silently stops calling them after the first session.
- **Compute fires before `customNamedValues` are written:** `customFeatures.add()` triggers
  `customFeatureCompute` immediately. The compute handler must handle empty/missing data.

## API constraints to keep in mind

- `SelectionCommandInput` cannot be placed inside a `TableCommandInput`.
  Vertex selection is via the mouseClick handler anyway, so this is not an issue.
- `TableCommandInput` is `adsk.core`, not `adsk.fusion`.
- Column widths: `columnRatio` string e.g. `"1:5:2"`. Setting a width to `0` hides
  the column without deleting inputs.
- `table.selectedRow` returns -1 if no row selected; set it programmatically after
  adding a row to auto-select the new one.
- Input IDs must use a monotonically increasing counter — never reuse IDs after
  deletion, or label numbers collide on re-add.
- Use `table.getInputAtPosition(row, col)` for position-based lookup; ID-suffix
  lookup breaks after rows are deleted and new ones are added.
- `BoolValueInput` toolbar buttons require `''` as the resource folder if no icon
  assets exist (a relative path that doesn't resolve throws a runtime error).

## Prototype plan

Step 1 ✓ — Table UI validated: layout, add/remove rows, row selection, label
renumbering. See `exploration/scratch/FusionTesting/commands/tableTest/`.

Step 2 (next) — Wire vertex selection into the active row. mouseClick populates
per-row vertex lists; switching rows switches the active set.

Step 3 — Run fitting on OK. Output untrimmed BRep faces to document via
`TemporaryBRepManager` → `BaseFeature` (parametric mode) or direct body
(direct mode).

Step 4 (deferred) — Wrap in `CustomFeature` for timeline editability.
