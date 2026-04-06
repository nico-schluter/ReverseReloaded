# Manual Test Plan — Fit Surfaces Add-In

Fusion 360 API code cannot be unit-tested automatically. Run these checks inside Fusion after any significant change to `src/`.

Load the add-in via **Scripts & Add-Ins (Shift+S) → Add-Ins → point to `src/Reverse/`**.

---

## 1. Add-in lifecycle

- [ ] Add-in loads without errors in the Text Command window
- [ ] **Fit Surfaces** button appears in the toolbar (Mesh tab → Create panel, or Scripts/Add-Ins panel as fallback)
- [ ] Add-in unloads cleanly — button disappears, no errors
- [ ] Reload after unload works without stale state

---

## 2. Dialog — basic UI

- [ ] Dialog opens when the button is clicked
- [ ] Mesh body selection input is present and active
- [ ] Table starts with one row (labelled "1"), type dropdown defaulting to "Plane"
- [ ] **+** toolbar button adds a row; label renumbers correctly
- [ ] **−** toolbar button removes the selected row; labels renumber; cannot delete the last row
- [ ] Radius (px) slider present, moves, range 5–200
- [ ] Expansion (mm) slider present, moves, range 0–5 mm
- [ ] Status text updates as inputs change
- [ ] Cancel dismisses the dialog with no changes to the design

---

## 3. Mesh selection

- [ ] Clicking a mesh body in the viewport selects it; status shows vertex count
- [ ] Mesh in a sub-component (not root) loads correctly — world transform applied (vertex highlights appear in the right place)
- [ ] Deselecting the mesh clears all vertex selections and resets all row counts to "0 pts"

---

## 4. Vertex selection

- [ ] Clicking the viewport while a mesh is loaded adds vertices to the active row; row count and status update
- [ ] Selection radius slider visibly changes how many vertices are captured per click
- [ ] Shift-click removes vertices from the active row
- [ ] Switching table rows and clicking accumulates into the newly active row without affecting other rows
- [ ] Custom graphics (coloured point sprites) appear at selected vertex positions
- [ ] Graphics update immediately after each click; previous frame is not left behind

---

## 5. Plane fitting

Select ≥ 3 vertices on a clearly planar region.

- [ ] Preview surface appears as a flat planar face covering the selected vertices
- [ ] Preview updates live as more vertices are added
- [ ] Expansion slider extends the face outward beyond the convex hull
- [ ] Expansion = 0 produces a face that tightly bounds the selected points
- [ ] OK commits the surface; it appears as a body in the browser
- [ ] Parametric mode: surface is wrapped in a Base Feature in the timeline

---

## 6. Cylinder fitting

Select ≥ 4 vertices on a clearly cylindrical region (ideally with good radial distribution).

- [ ] Preview surface appears as a ruled cylinder of the correct radius and orientation
- [ ] Expansion slider extends the cylinder axially beyond the selected extents
- [ ] OK commits the surface

---

## 7. Multi-surface

Add two or more rows and select vertices for each.

- [ ] Each row fits independently; all surfaces appear in the preview simultaneously
- [ ] Rows with fewer than the minimum vertex count (Plane < 3, Cylinder < 4) are silently skipped — no crash
- [ ] OK commits all surfaces; each is a separate body

---

## 8. Camera modes

- [ ] Switch to **Perspective** camera mode (View menu → Camera → Perspective) — dialog remains open, viewport interaction continues to work
- [ ] Switch to **Orthographic** camera mode — vertex selection and preview rendering remain correct
- [ ] Toggling between perspective and orthographic during an active session does not lose vertex selections or reset the preview

---

## 9. Edge cases

- [ ] Clicking the viewport before a mesh is selected does nothing
- [ ] Very few points (e.g. exactly 3 for Plane, exactly 4 for Cylinder) — fit runs without error
- [ ] Collinear or near-collinear points for Cylinder — degenerate fit logged, surface skipped, no crash
- [ ] Removing a row that has vertices — subsequent rows' selections are not affected

---

## 10. Session persistence

- [ ] Closing and re-opening the dialog (without reloading the add-in) resets vertex selections
- [ ] Radius and Expansion sliders restore their last-used values across invocations within the same session
