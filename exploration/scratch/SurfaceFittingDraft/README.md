# SurfaceFittingDraft

Intermediate prototype of the full add-in command structure — written after `tableTest` validated the dialog mechanics but before the clean implementation landed in `src/`.

Used to work out the end-to-end flow: mesh selection → mouseClick vertex accumulation → executePreview graphics → fitting → BRep output. The structure mirrors `src/` deliberately so lessons could transfer directly.

Do not import from or promote this to `src/`. Superseded by `src/Reverse/commands/fitSurfaces/entry.py`.
