"""Unit tests for the pure asymmetric-decoupe helpers (no FreeCAD needed)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from openglider.glider.parametric.asymmetric import (
    expand_cut_placements,
    finalize_cut_lines,
    wing_of_x,
)


def test_wing_of_x():
    assert wing_of_x(-1.2) == "left"
    assert wing_of_x(0.8) == "right"
    assert wing_of_x(0.0) == "both"


def test_collapse_identical_pair_to_symmetric():
    lines = [
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "left"},
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "right"},
    ]
    out = finalize_cut_lines(lines)
    assert len(out) == 1
    assert "side" not in out[0]  # collapsed to symmetric, marker stripped
    assert out[0]["cells"] == [3]


def test_divergent_pair_stays_asymmetric():
    lines = [
        {"cells": [3], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "left"},
        {"cells": [3], "left": -0.5, "right": -0.5, "type": "orthogonal", "side": "right"},
    ]
    out = finalize_cut_lines(lines)
    sides = sorted(c["side"] for c in out)
    assert sides == ["left", "right"]


def test_lone_side_preserved():
    lines = [
        {"cells": [2], "left": -0.4, "right": -0.4, "type": "orthogonal", "side": "left"},
    ]
    out = finalize_cut_lines(lines)
    assert len(out) == 1 and out[0]["side"] == "left"


def test_contiguous_cells_merge():
    lines = [
        {"cells": [1], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "both"},
        {"cells": [2], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "both"},
    ]
    out = finalize_cut_lines(lines)
    assert len(out) == 1
    assert sorted(out[0]["cells"]) == [1, 2]
    assert "side" not in out[0]


def test_symmetric_only_input_unchanged_format():
    lines = [
        {"cells": [1], "left": -0.3, "right": -0.4, "type": "orthogonal", "side": "both"},
    ]
    out = finalize_cut_lines(lines)
    assert out == [{"cells": [1], "left": -0.3, "right": -0.4, "type": "orthogonal"}]


def test_expand_symmetric_draws_both_wings():
    cut = {"cells": [2, 3], "left": -0.3, "right": -0.3, "type": "orthogonal"}
    placements = set(expand_cut_placements(cut, has_center=True))
    assert placements == {(2, "left"), (2, "right"), (3, "left"), (3, "right")}


def test_expand_center_cell_single():
    cut = {"cells": [0], "left": -0.3, "right": -0.3, "type": "orthogonal"}
    placements = list(expand_cut_placements(cut, has_center=True))
    assert placements == [(0, "both")]


def test_expand_side_tagged():
    cut = {"cells": [4], "left": -0.3, "right": -0.3, "type": "orthogonal", "side": "left"}
    placements = list(expand_cut_placements(cut, has_center=True))
    assert placements == [(4, "left")]


if __name__ == "__main__":
    import inspect

    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and inspect.isfunction(fn):
            fn()
            print("ok  ", name)
    print("all asymmetric-helper tests passed")
