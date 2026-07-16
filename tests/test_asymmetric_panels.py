"""Asymmetric decoupe: left/right wings may carry different panel cuts and
colours while the structural geometry stays a spanwise mirror.

The engine keys asymmetry off ParametricGlider.elements:
  * a cut dict tagged ``"side": "left" | "right"`` applies to that wing only;
  * ``materials_left_by_name`` overrides panel colours on the left wing.
When no such data is present the glider builds exactly as before (pure mirror).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import openglider

DEMOKITE = os.path.join(os.path.dirname(__file__), "common", "demokite.json")


def _full(g2d):
    return g2d.get_glider_3d().copy_complete()


def _cells_with(full, prefix):
    """Physical cell indices whose panels come from a given half-cell (by
    panel name prefix, e.g. 'c3p')."""
    return [
        ci
        for ci, cell in enumerate(full.cells)
        if any(p.name.startswith(prefix) for p in cell.panels)
    ]


def test_clean_glider_is_symmetric():
    g2d = openglider.load(DEMOKITE)
    assert g2d.is_asymmetric is False
    full = _full(g2d)
    ncenter = g2d.shape.half_cell_num - 1  # centre cell index in full glider
    left = {
        (g2d.shape.half_cell_num - 1 - ci): [p.material_code for p in full.cells[ci].panels]
        for ci in range(ncenter)
    }
    right = {
        (ci - ncenter): [p.material_code for p in full.cells[ci].panels]
        for ci in range(ncenter + 1, len(full.cells))
    }
    assert left == right


def test_side_tagged_cut_is_symmetric_flag():
    g2d = openglider.load(DEMOKITE)
    g2d.elements.setdefault("cuts", [])
    g2d.elements["cuts"].append(
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "right"}
    )
    assert g2d.is_asymmetric is True


def test_right_only_cut_diverges_geometry():
    g2d = openglider.load(DEMOKITE)
    g2d.elements.setdefault("cuts", [])
    g2d.elements["cuts"].append(
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "right"}
    )
    full = _full(g2d)
    counts = {ci: len(full.cells[ci].panels) for ci in _cells_with(full, "c4p")}
    # the two physical c4 cells (left + right) now have different panel counts
    assert len(set(counts.values())) == 2


def test_left_only_cut_targets_left_wing():
    g2d = openglider.load(DEMOKITE)
    g2d.elements.setdefault("cuts", [])
    g2d.elements["cuts"].append(
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "left"}
    )
    full = _full(g2d)
    c4 = sorted(_cells_with(full, "c4p"))
    # lower physical index == left wing; it gets the extra cut
    assert len(full.cells[c4[0]].panels) > len(full.cells[c4[1]].panels)


def test_asymmetric_colours_diverge():
    g2d = openglider.load(DEMOKITE)
    g2d.elements["materials_by_name"] = {"c3p1": "rightred"}
    g2d.elements["materials_left_by_name"] = {"c3p1": "leftblue"}
    assert g2d.is_asymmetric is True
    full = _full(g2d)
    cols = {}
    for ci in _cells_with(full, "c3p"):
        for p in full.cells[ci].panels:
            if p.name == "c3p1":
                cols[ci] = p.material_code
    assert set(cols.values()) == {"leftblue", "rightred"}


def test_json_roundtrip_preserves_asymmetry(tmp_path=None):
    import tempfile

    g2d = openglider.load(DEMOKITE)
    g2d.elements.setdefault("cuts", [])
    g2d.elements["cuts"].append(
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "left"}
    )
    g2d.elements["materials_by_name"] = {"c3p1": "rightred"}
    g2d.elements["materials_left_by_name"] = {"c3p1": "leftblue"}

    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        openglider.save(g2d, path)
        g2 = openglider.load(path)
    finally:
        os.remove(path)

    assert g2.is_asymmetric is True
    full = _full(g2)
    counts = {ci: len(full.cells[ci].panels) for ci in _cells_with(full, "c4p")}
    assert len(set(counts.values())) == 2
    cols = set()
    for ci in _cells_with(full, "c3p"):
        for p in full.cells[ci].panels:
            if p.name == "c3p1":
                cols.add(p.material_code)
    assert cols == {"leftblue", "rightred"}


# --- Full-span design-tool DATA PATH (simulated with the real pure helpers) ---
# The FreeCAD design tool's full-span canvas is exercised here at the data level
# (rendering/mouse interaction still needs FreeCAD).  We replay the exact helper
# chain the tool uses: expand -> signed ribs -> resolve -> finalize.
from openglider.glider.parametric.asymmetric import (  # noqa: E402
    expand_cut_placements,
    finalize_cut_lines,
    resolve_signed_pair,
    signed_ribs_for,
)

_HAS_CENTER = True  # demokite is odd


def _stored_to_canvas(cuts):
    lines = []
    for cut in cuts:
        for cell, side in expand_cut_placements(cut, _HAS_CENTER):
            si, so = signed_ribs_for(cell, side, _HAS_CENTER)
            lines.append((si, so, cut["left"], cut["right"], cut["type"]))
    return lines


def _canvas_to_stored(lines):
    dicts = []
    for (si, so, pos_i, pos_o, t) in lines:
        inner_is_r1, cell, wing = resolve_signed_pair(si, so, _HAS_CENTER)
        left, right = (pos_i, pos_o) if inner_is_r1 else (pos_o, pos_i)
        dicts.append({"cells": [cell], "left": left, "right": right,
                      "type": t, "side": wing})
    return finalize_cut_lines(dicts)


def _norm(cuts):
    return sorted((tuple(sorted(c["cells"])), c["left"], c["right"], c["type"],
                   c.get("side")) for c in cuts)


def test_fullspan_symmetric_roundtrip_idempotent():
    cuts = [
        {"cells": [2], "left": -0.30, "right": -0.40, "type": "orthogonal"},
        {"cells": [0], "left": -0.20, "right": -0.20, "type": "orthogonal"},
    ]
    assert _norm(_canvas_to_stored(_stored_to_canvas(cuts))) == _norm(cuts)


def test_fullspan_left_edit_diverges_engine_geometry():
    cuts = [{"cells": [2], "left": -0.30, "right": -0.40, "type": "orthogonal"}]
    lines = _stored_to_canvas(cuts)
    target = signed_ribs_for(2, "left", _HAS_CENTER)
    edited = [(si, so, pi, (-0.60 if (si, so) == target else po), t)
              for (si, so, pi, po, t) in lines]
    stored = _canvas_to_stored(edited)

    g2d = openglider.load(DEMOKITE)
    g2d.elements["cuts"] = stored
    assert g2d.is_asymmetric
    full = _full(g2d)
    c3 = sorted(_cells_with(full, "c3p"))
    left_lefts = sorted(round(p.cut_front["left"], 2) for p in full.cells[c3[0]].panels)
    right_lefts = sorted(round(p.cut_front["left"], 2) for p in full.cells[c3[-1]].panels)
    assert left_lefts != right_lefts  # left wing edited, right untouched


if __name__ == "__main__":
    import inspect

    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and inspect.isfunction(fn):
            fn()
            print("ok  ", name)
    print("all asymmetric-panel tests passed")
