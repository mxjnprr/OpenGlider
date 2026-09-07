"""Leading-edge panel split.

The LE panel of a half-cell is the extrados strip bounded by the design's LE
cut and the air intake.  ``elements["le_panel_splits"]`` lists the cells whose
LE panel is halved spanwise (seam at mid-cell); nothing else is touched and no
cut is created.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import openglider

DEMOKITE = os.path.join(os.path.dirname(__file__), "common", "demokite.json")
LE_CUT = 0.10  # chord fraction of the extrados LE cut added by the tests
TIP = 6  # demokite has 7 half-cells (0 = centre, 6 = tip)


def _glider_with_le_cut(cells):
    g2d = openglider.load(DEMOKITE)
    g2d.elements["cuts"].append(
        {"cells": list(cells), "left": -LE_CUT, "right": -LE_CUT, "type": "cut_3d"}
    )
    return g2d


def _halves(panels):
    return [
        p
        for p in panels
        if (getattr(p, "y_start", 0.0), getattr(p, "y_end", 1.0)) != (0.0, 1.0)
    ]


class TestLEPanelSplit(unittest.TestCase):
    def test_bounds_follow_the_design_le_cut(self):
        g2d = _glider_with_le_cut([4, 5, 6])
        front, back = g2d.get_le_panel_bounds(5)
        self.assertAlmostEqual(front, -LE_CUT)
        self.assertAlmostEqual(back, 0.06)  # demokite intake cut on cells 5, 6
        # no extrados cut: the upper surface is one panel, never an LE panel
        self.assertIsNone(g2d.get_le_panel_bounds(0))

    def test_only_selected_cells_get_split(self):
        g2d = _glider_with_le_cut([4, 5, 6])
        g2d.set_le_split_cells([5, 6])
        panels = g2d.get_panels()
        for cell_no in range(5):
            self.assertEqual(_halves(panels[cell_no]), [], f"cell {cell_no} must stay whole")
        for cell_no in (5, 6):
            halves = _halves(panels[cell_no])
            self.assertEqual(len(halves), 2)
            self.assertEqual(sorted(p.name[-2:] for p in halves), ["_L", "_R"])
            self.assertEqual(
                {(p.y_start, p.y_end) for p in halves}, {(0.0, 0.5), (0.5, 1.0)}
            )

    def test_split_stops_at_the_le_cut(self):
        g2d = _glider_with_le_cut([TIP])
        g2d.set_le_split_cells([TIP])
        panels = g2d.get_panels()[TIP]
        # trailing edge -> LE cut, two LE halves, intake -> trailing edge
        self.assertEqual(len(panels), 4)
        for p in _halves(panels):
            self.assertAlmostEqual(p.cut_front["left"], -LE_CUT)
            self.assertGreaterEqual(p.cut_back["left"], 0.0)
        whole = [p for p in panels if p not in _halves(panels)]
        self.assertEqual(sorted(p.cut_front["left"] for p in whole), [-1, 0.06])

    def test_cell_without_le_cut_is_never_split(self):
        g2d = openglider.load(DEMOKITE)
        g2d.set_le_split_cells([0, TIP])
        panels = g2d.get_panels()
        self.assertEqual(sum(len(_halves(c)) for c in panels), 0)

    def test_split_ignores_le_cut_position_mismatch_of_legacy_entries(self):
        # an old-style entry with a cut_limit that does not match the real LE
        # cut still splits the actual LE panel
        g2d = _glider_with_le_cut([TIP])
        g2d.elements["le_panel_splits"] = [
            {"cut_limit": 0.25, "material_code": "", "cells": [TIP]}
        ]
        self.assertEqual(g2d.le_split_cells(), {TIP})
        halves = _halves(g2d.get_panels()[TIP])
        self.assertEqual(len(halves), 2)
        self.assertAlmostEqual(halves[0].cut_front["left"], -LE_CUT)

    def test_set_le_split_cells_normalises(self):
        g2d = openglider.load(DEMOKITE)
        g2d.set_le_split_cells([TIP, 3, 99, -1, 3])
        self.assertEqual(
            g2d.elements["le_panel_splits"], [{"cells": [3, TIP], "material_code": ""}]
        )
        g2d.set_le_split_cells([])
        self.assertNotIn("le_panel_splits", g2d.elements)

    def test_full_glider_builds_with_split(self):
        g2d = _glider_with_le_cut([5, TIP])
        g2d.set_le_split_cells([5, TIP])
        glider = g2d.get_glider_3d()
        glider.copy_complete()
        self.assertEqual(len(glider.cells[TIP].le_closures), 1)
        self.assertAlmostEqual(glider.cells[TIP].le_closures[0].cut_back_x, LE_CUT)
        self.assertFalse(getattr(glider.cells[0], "le_closures", []))
        # the half panels can be meshed (FreeCAD preview)
        halves = _halves(glider.cells[TIP].panels)
        self.assertEqual(len(halves), 2)
        for p in halves:
            p.get_mesh(glider.cells[TIP], 4, with_numpy=True)


if __name__ == "__main__":
    unittest.main()
