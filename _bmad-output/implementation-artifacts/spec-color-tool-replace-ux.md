---
title: 'Color Tool — clearer replace-color workflow'
type: 'feature'
created: '2026-05-11'
status: 'done'
route: 'one-shot'
---

# Color Tool — clearer replace-color workflow

## Intent

**Problem:** In the FreeCAD Color Tool, clicking "replace color" opened a generic `QColorDialog`. The picked color was used as the *new* color and the currently-selected 3D panel as the *source*, but nothing in the UI communicated this. A hard `assert` silently crashed the action when no panel was selected.

**Approach:** Replaced the direct `QColorDialog` wire-up with a custom `ReplaceColorDialog` that explicitly shows two swatches ("Color to replace" / "New color") and a scope selector (selected panels only vs. all panels with this color). Added a friendly empty-selection guard and a canonical 8-bit RGB key (`_color_key`) to avoid float-equality drift between `QColorDialog` and `hex_to_rgb`. The geometric-adjacency option was deferred — `(cell_idx, panel_idx)` adjacency doesn't model split panels correctly.

## Suggested Review Order

1. [Dialog skeleton](../../freecad/glider/tools/color_tool.py#L63-L121) — `ReplaceColorDialog` UI: swatches + scope radios + buttons.
2. [Replace-color flow](../../freecad/glider/tools/color_tool.py#L193-L223) — selection guard, modal `exec_`, scope dispatch, canonical color match.
3. [`_color_key` helper](../../freecad/glider/tools/color_tool.py#L57-L60) — why 8-bit canonicalization beats float `==` on `std_col`.
4. [`set_color` guard](../../freecad/glider/tools/color_tool.py#L28-L31) — `if col is not None` (was `col or self.std_col`, which dropped truthy-but-falsy lists).
