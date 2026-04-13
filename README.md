# ReverseReloaded

Add-In for Autodesk Fusion 360.  
Reconstructs BRep surfaces from mesh bodies.  
Generates geometry very close to the original source file, generally within 1e-6mm.  
Pure Python — no external dependencies (numpy, scipy, etc.).

Inspired by and based on the original [Reverse](https://github.com/phossystems/Reverse) add-in.

# Use

1. Import the mesh to be reconstructed
2. Run **Fit Surfaces** from the Mesh tab → Create panel
3. Select the mesh body
4. Add a row for each surface you want to reconstruct (press **N** or the **+** button), choose its type (Auto, Plane, Cylinder, Cone, or Sphere), and select mesh points belonging to that feature:
   - All points within the selection radius around the click position will be selected, even hidden ones.
   - Hold shift to deselect points.
   - Adjust the selection radius slider to select just the points you want.
   - Points not belonging to the feature you are reconstructing significantly degrade the accuracy. Outlier points are highlighted automatically.
   - A preview surface updates live as you select points.
   - **Auto** mode fits all surface types and picks the best match.
5. Repeat step 4 for all features of the part — multiple surfaces can be set up before pressing OK
6. Press OK to commit all surfaces
7. Use Boundary Fill to turn the enclosed volume into a solid

# Supported Features

- Planes
- Cylinders
- Cones (pointed or truncated)
- Spheres

# Installation

- Download the project as a ZIP and extract it somewhere convenient, or clone it with git
- Open Fusion 360 and press **Shift+S** to open Scripts & Add-Ins
- Select the **Add-Ins** tab and click the green **+** next to "My Add-Ins"
- Navigate to the `src/Reverse/` folder inside the extracted project and hit Open
- The add-in will appear in the "My Add-Ins" list — select it, optionally check "Run on Startup", and click Run
- The command appears as **Mesh → Create → Fit Surfaces**

# Changelog

## 2.0 — Cone, Sphere & Auto Mode

- Cone fitting (Gauss-Newton solver) — pointed and truncated cones
- Sphere fitting (algebraic linear solve)
- Auto surface type: fits all types, picks the best match automatically
- Outlier detection: highlights points that don't fit the surface well
- Fit result caching: skips redundant fitting when only the expansion slider changes
- N hotkey to add new surface rows
- Auto-selects the mesh body when only one is visible

## 1.0 — Plane & Cylinder

- Pure Python fitting algorithms — no numpy or scipy required
- Multi-surface table: add and configure multiple surfaces before committing
- Live preview surface updates as points are selected
- Expansion distance slider: extend surfaces beyond the selected point boundary
- Screen-space pixel radius vertex selection with shift-click deselect
- Custom graphics: per-row coloured point highlights in the viewport
