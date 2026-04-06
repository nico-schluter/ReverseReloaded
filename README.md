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
4. Add a row for each surface you want to reconstruct, choose its type (Plane or Cylinder), and select mesh points belonging to that feature:
   - All points within the selection radius around the click position will be selected, even hidden ones.
   - Hold shift to deselect points.
   - Adjust the selection radius slider to select just the points you want.
   - Points not belonging to the feature you are reconstructing significantly degrade the accuracy.
   - A preview surface updates live as you select points.
5. Repeat step 4 for all features of the part — multiple surfaces can be set up before pressing OK
6. Press OK to commit all surfaces
7. Use Boundary Fill to turn the enclosed volume into a solid

# Supported Features

- Planes
- Cylinders

# Installation

- Download the project as a ZIP and extract it somewhere convenient, or clone it with git
- Open Fusion 360 and press **Shift+S** to open Scripts & Add-Ins
- Select the **Add-Ins** tab and click the green **+** next to "My Add-Ins"
- Navigate to the `src/Reverse/` folder inside the extracted project and hit Open
- The add-in will appear in the "My Add-Ins" list — select it, optionally check "Run on Startup", and click Run
- The command appears as **Mesh → Create → Fit Surfaces**

# Changelog

## 1.0 — Plane & Cylinder

- Pure Python fitting algorithms — no numpy or scipy required
- Multi-surface table: add and configure multiple surfaces before committing
- Live preview surface updates as points are selected
- Expansion distance slider: extend surfaces beyond the selected point boundary
- Screen-space pixel radius vertex selection with shift-click deselect
- Custom graphics: per-row coloured point highlights in the viewport
