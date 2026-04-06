# Custom Graphics — Open Problems

Two custom graphics features are needed for the Fit Surfaces command but are not yet
understood well enough to implement correctly in production code.

---

## 1. Per-row vertex colour differentiation

### What we want
Each surface row's selected vertices displayed in a distinct colour so the user can
visually distinguish which vertices belong to which surface.

### What was tried
`CustomGraphicsPointSet` with `UserDefinedCustomGraphicsPointType` + `point.png` from
the original Reverse add-in, with `CustomGraphicsSolidColorEffect` applied per row.

### What happened / confirmed
`point.png` is a red circle. All rows displayed as red regardless of the
`SolidColorEffect` colour assigned. The PNG pixel colour wins; `SolidColorEffect` does
not tint or override image pixels for `UserDefinedCustomGraphicsPointType`.

### Root cause (confirmed)
`SolidColorEffect` controls the entity's *material* colour, not its texture pixels.
A user-defined point image is rendered as a texture billboard; its own RGBA values are
used directly. The effect has no influence over those pixels.

### Confirmed dead ends
- `UserDefinedCustomGraphicsPointType` + red `point.png` + `SolidColorEffect` → all red (image wins)
- `UserDefinedCustomGraphicsPointType` + white `point_white.png` + `SolidColorEffect` → all white (image still wins)

**`SolidColorEffect` does not tint image-based point sprites regardless of the PNG colour.**

### Remaining candidates (not yet tested)
Two approaches still to try, in order of simplicity:

**A. `PointCloudCustomGraphicsPointType` + `SolidColorEffect`**
No image involved — points are single pixels rendered as geometry. `SolidColorEffect`
may work correctly here since there is no texture to override it. The earlier observation
that pixel points were "invisible" may have been a separate issue (they are genuinely
tiny — 1px — and may need `depthPriority` to show above mesh faces).

**B. `CustomGraphicsCoordinates.colors` + `CustomGraphicsVertexColorEffect`**
Set per-coordinate RGBA via `coords.colors = [r,g,b,a, r,g,b,a, ...]` and apply
`CustomGraphicsVertexColorEffect` to the point set. The API doc describes this for
mesh vertex colouring but it may also apply to `CustomGraphicsPointSet`.

### Suggested experiment
In `exploration/scratch/FusionTesting/`, add a command that places two groups of 5
points at known 3D positions and tries approach A then B, logging what is visible.
The key questions are: (A) do pixel points appear at all, and do they colour correctly?
(B) does vertex colour effect apply to point sets?

---

## 2. Cursor circle — selection radius visualisation

### What we want
A circle following the mouse cursor showing the current pixel-radius selection region,
so the user has real-time feedback of how large each click will select.

### What was tried (production code — reverted)
- `CustomGraphicsLines` unit-circle line strip
- `viewScale(pixelScale = _px_radius)` to size it in pixels
- `viewPlacement(upperLeft, cursor_pos)` to position it at the cursor
- `viewPoint` updated in a `mouseMove` handler

### What happened
- The circle appeared but was oriented along the XY model plane, not facing the screen.
  `viewScale` + `viewPlacement` together may not produce a proper screen-space overlay.
- The position only updated when the viewport was panned/rotated, not on raw mouse movement.
  This suggests `mouseMove` on the command either does not fire on cursor motion alone, or
  the custom graphics do not refresh until a viewport redraw is triggered by a camera event.
- Viewport felt choppy while the command was active, suggesting the `mouseMove` handler
  was firing and causing overhead even when nothing was drawn.

### Key unknowns
1. Does `cmd.mouseMove` fire on cursor movement inside the viewport, or only on
   movement that drags/pans the view? Need to confirm with a simple log-only handler.
2. Does assigning `entity.viewPlacement` require a manual viewport refresh call
   (`app.activeViewport.refresh()` or equivalent) to take effect immediately?
3. Do `viewPlacement` and `viewScale` compose as expected, or does one override the other?

### Suggested experiment
In `exploration/scratch/FusionTesting/`, add a command that:
1. Registers `mouseMove` and logs the viewport position to the text command window —
   confirms whether the event fires at all and at what frequency.
2. Draws a single point at (0,0,0), gives it `viewPlacement` pointing to (100, 100)
   from the upper-left corner, and checks if it appears at pixel (100,100) in the view.
3. Updates `viewPlacement.viewPoint` inside `mouseMove` and checks if the position
   changes live (without a pan/zoom event).
4. Tests `viewScale` separately — draws a line 1 unit long, applies `viewScale(10)`,
   checks if it appears 10 pixels long regardless of zoom.

### Alternative approach if screen-space doesn't work
Draw the circle in model space at the 3D position under the cursor
(`vp.viewToModelSpace(viewportPosition)` returns a Point3D on the camera near-plane).
The circle radius would need to be computed from `_px_radius` and the current
zoom/field-of-view — non-trivial for perspective cameras, straightforward for
orthographic. This approach avoids the screen-space API entirely but the circle would
appear at the wrong depth (on the near-plane) and would not match the mesh surface.
