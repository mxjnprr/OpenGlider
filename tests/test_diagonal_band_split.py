"""Band-split ("T") diagonals: bands as wide as the intrados base that rise
straight, then flare out from the join height to meet on the extrados.

Covers the DiagonalRib geometry (trumpet outline + holes in the flattened
frame, parametric mapping, 3D mesh, 2D pattern) and the Edit Cells auto-fill
that produces them (headless, FreeCAD/pivy stubbed as in
test_le_panel_split_tool.py).
"""
import importlib
import os
import sys
import unittest

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import test_le_panel_split_tool as tool_stubs

import openglider
from openglider.glider.cell.elements import (
    DiagonalRib,
    _flat_to_parametric,
    _point_in_polygon,
)
from openglider.glider.parametric.glider import DIAGONAL_AUTOFILL_DEFAULT_PARAMS
from openglider.plots.glider.cell import DribPlot
from openglider.plots.glider.config import PatternConfig

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")
CFG = {"num": 3, "flare_angle": 40.0, "margin_top": 0.01}
BASE = ((0.53, -1), (0.57, -1))
FAR = ((0.35, 1), (0.75, 1))
NARROW = ((0.50, 1), (0.62, 1))
BAND_CFG = {"num": 1, "flare_angle": 40.0, "strip_width": 0.04}


def _polygon_area(points):
    area = 0.0
    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


def _dist_to_polyline(point, polyline):
    """Distance from a point to a polyline (segments, not only vertices)."""
    pts = np.asarray(polyline.data)
    p = np.asarray(point)
    best = float("inf")
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        t = float(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-18))
        t = min(max(t, 0.0), 1.0)
        best = min(best, float(np.linalg.norm(a + t * ab - p)))
    return best


def _real_cuts(plotpart):
    return [c for c in plotpart.layers["cuts"] if len(c) >= 3]


def _width_at(points, origin, up, h):
    """Extent of a closed polygon across the line at height h above origin."""
    hits = []
    n = len(points)
    for i in range(n):
        p, q = np.asarray(points[i]), np.asarray(points[(i + 1) % n])
        hp, hq = np.dot(p - origin, up) - h, np.dot(q - origin, up) - h
        if (hp > 0) != (hq > 0):
            t = hp / (hp - hq)
            hits.append(p + (q - p) * t)
    if len(hits) < 2:
        return 0.0
    hits = np.array(hits)
    across = np.array([-up[1], up[0]])
    proj = hits @ across
    return float(proj.max() - proj.min())


class TestBandSplitGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.glider_3d = openglider.load(DEMOKITE).get_glider_3d()
        cls.cell = cls.glider_3d.cells[3]

    def _drib(self, mirrored=False, **overrides):
        cfg = dict(CFG, **overrides)
        if mirrored:
            return DiagonalRib(FAR[0], FAR[1], BASE[0], BASE[1], band_split=cfg)
        return DiagonalRib(BASE[0], BASE[1], FAR[0], FAR[1], band_split=cfg)

    def _shape(self, drib):
        flat_left, flat_right = drib.get_flattened(self.cell)
        return flat_left, flat_right, drib.get_band_split_shape_flat(flat_left, flat_right)

    def test_outline_is_a_stem_then_a_flare(self):
        for mirrored in (False, True):
            drib = self._drib(mirrored=mirrored, num=1)
            flat_left, flat_right, (outline, holes) = self._shape(drib)
            base = flat_right if mirrored else flat_left
            far = flat_left if mirrored else flat_right
            trapezoid = list(base.data) + list(far.data)[::-1]
            width = base.get_length()
            p_ap = np.asarray(base[base.walk(0, width / 2)])
            up = np.asarray(far[far.walk(0, far.get_length() / 2)]) - p_ap
            height = np.linalg.norm(up)
            up = up / height
            # near the base the piece is as wide as the base (stem)
            for h in (0.1, 0.2):
                self.assertAlmostEqual(_width_at(outline, p_ap, up, h * height), width, delta=0.25 * width)
            # then it widens to the full extrados range
            self.assertGreater(_width_at(outline, p_ap, up, 0.9 * height), 2 * width)
            self.assertLess(_polygon_area(outline), 0.85 * _polygon_area(trapezoid))
            # the outline stays inside the plain trapezoid
            for pt in outline:
                self.assertTrue(_point_in_polygon(pt, trapezoid) or
                                min(np.linalg.norm(np.asarray(trapezoid) - pt, axis=1)) < 1e-6)
            # with three bands the outline is the union of three such stems
            outline3 = self._shape(self._drib(mirrored=mirrored))[2][0]
            width3 = _width_at(outline3, p_ap, up, 0.4 * height)
            self.assertGreater(width3, 1.5 * width)
            self.assertLess(width3, _width_at(trapezoid, p_ap, up, 0.4 * height) + 1e-9)
            self.assertLess(_polygon_area(outline), _polygon_area(outline3))
            self.assertLess(_polygon_area(outline3), _polygon_area(trapezoid))

    def test_holes_between_bands(self):
        drib = self._drib()
        flat_left, flat_right, (outline, holes) = self._shape(drib)
        self.assertEqual(len(holes), 2)
        for hole in holes:
            self.assertGreaterEqual(len(hole), 3)
            for pt in hole:
                self.assertTrue(_point_in_polygon(pt, outline))
            # the apex keeps at least margin_top of fabric along the extrados
            # (a little more because the apex corner is rounded)
            dist = min(_dist_to_polyline(p, flat_right) for p in hole)
            self.assertGreaterEqual(dist, CFG["margin_top"] - 1e-3)
            self.assertLess(dist, CFG["margin_top"] + 0.02)

    def test_flare_angle_is_measured_from_the_extrados(self):
        # sharp corners so that the hole vertices are the true corners
        drib = self._drib(corner_radius=0.0)
        flat_left, flat_right, (outline, holes) = self._shape(drib)
        far_len = flat_right.get_length()
        self.assertEqual(len(holes), 2)
        for j, hole in enumerate(holes):
            pts = [np.asarray(p) for p in hole]
            i_apex = min(range(len(pts)), key=lambda i: _dist_to_polyline(pts[i], flat_right))
            t_b = (j + 1) / 3
            tang = (np.asarray(flat_right[flat_right.walk(0, (t_b + 0.02) * far_len)])
                    - np.asarray(flat_right[flat_right.walk(0, (t_b - 0.02) * far_len)]))
            tang = tang / np.linalg.norm(tang)
            # both roof edges leave the apex at 40 deg from the extrados line
            for neighbour in (pts[i_apex - 1], pts[(i_apex + 1) % len(pts)]):
                roof = neighbour - pts[i_apex]
                cos_a = abs(np.dot(roof, tang)) / np.linalg.norm(roof)
                self.assertAlmostEqual(np.degrees(np.arccos(cos_a)), 40.0, delta=1.0)

    def test_flare_angle_zero_is_the_plain_trapezoid(self):
        drib = self._drib(flare_angle=0.0)
        flat_left, flat_right, (outline, holes) = self._shape(drib)
        trapezoid = list(flat_left.data) + list(flat_right.data)[::-1]
        self.assertEqual(holes, [])
        self.assertAlmostEqual(_polygon_area(outline), _polygon_area(trapezoid), delta=1e-9)

    def test_single_band_is_a_t(self):
        drib = self._drib(num=1)
        flat_left, flat_right, (outline, holes) = self._shape(drib)
        trapezoid = list(flat_left.data) + list(flat_right.data)[::-1]
        self.assertEqual(holes, [])
        self.assertLess(_polygon_area(outline), 0.85 * _polygon_area(trapezoid))

    def test_too_narrow_range_drops_bands(self):
        # three 35 mm bands do not fit in a 10 cm range: two bands (one hole),
        # same count for the connecting band on that rib; a 6 cm range keeps
        # a single band and no hole
        drib = DiagonalRib(BASE[0], BASE[1], NARROW[0], NARROW[1], band_split=CFG)
        flat_left, flat_right = drib.get_flattened(self.cell)
        outline, holes = drib.get_band_split_shape_flat(flat_left, flat_right)
        self.assertEqual(len(holes), 1)
        # (same strip width as the flattened base of the diagonal: 35 mm)
        band = DiagonalRib(NARROW[0], NARROW[1], NARROW[0], NARROW[1],
                           band_split=dict(BAND_CFG, num=3, strip_width=0.035))
        flat_left, flat_right = band.get_flattened(self.cell)
        outline, holes = band.get_band_split_shape_flat(flat_left, flat_right)
        self.assertEqual(len(holes), 1)
        very_narrow = ((0.50, 1), (0.57, 1))
        drib = DiagonalRib(BASE[0], BASE[1], very_narrow[0], very_narrow[1], band_split=CFG)
        fl, fr = drib.get_flattened(self.cell)
        self.assertEqual(drib.get_band_split_shape_flat(fl, fr)[1], [])
        band = DiagonalRib(very_narrow[0], very_narrow[1], very_narrow[0], very_narrow[1],
                           band_split=dict(BAND_CFG, num=3, strip_width=0.035))
        flat_left, flat_right = band.get_flattened(self.cell)
        outline, holes = band.get_band_split_shape_flat(flat_left, flat_right)
        self.assertEqual(holes, [])
        m_l = np.asarray(flat_left[flat_left.walk(0, flat_left.get_length() / 2)])
        m_r = np.asarray(flat_right[flat_right.walk(0, flat_right.get_length() / 2)])
        u = (m_r - m_l) / np.linalg.norm(m_r - m_l)
        self.assertAlmostEqual(_width_at(outline, m_l, u, 0.5 * np.linalg.norm(m_r - m_l)), 0.035, delta=0.004)

    def test_connecting_band_is_shoes_and_strip(self):
        band = DiagonalRib(NARROW[0], NARROW[1], NARROW[0], NARROW[1], band_split=BAND_CFG)
        flat_left, flat_right, (outline, holes) = self._shape(band)
        self.assertEqual(holes, [])
        rectangle = list(flat_left.data) + list(flat_right.data)[::-1]
        m_l = np.asarray(flat_left[flat_left.walk(0, flat_left.get_length() / 2)])
        m_r = np.asarray(flat_right[flat_right.walk(0, flat_right.get_length() / 2)])
        span = np.linalg.norm(m_r - m_l)
        u = (m_r - m_l) / span
        # thin strip of the intrados width in the middle, full range at the ribs
        for f in (0.3, 0.5, 0.7):
            self.assertAlmostEqual(_width_at(outline, m_l, u, f * span), 0.04, delta=0.004)
        # shoe length = (half range - half strip) * tan(40 deg from the rib line)
        half_gap = flat_left.get_length() / 2 - 0.02
        shoe = half_gap * np.tan(np.radians(40))
        self.assertGreater(_width_at(outline, m_l, u, shoe * 0.5), 0.04 + half_gap * 0.5)
        self.assertAlmostEqual(_width_at(outline, m_l, u, shoe * 1.3), 0.04, delta=0.004)
        # the shoe tapers linearly from the full range at the rib down to the strip
        full = _width_at(rectangle, m_l, u, 0.03 * span)
        near_rib = _width_at(outline, m_l, u, 0.03 * span)
        self.assertGreater(near_rib, 0.8 * full)
        self.assertLess(near_rib, full)
        self.assertLess(_polygon_area(outline), 0.7 * _polygon_area(rectangle))
        for pt in outline:
            self.assertTrue(_point_in_polygon(pt, rectangle) or
                            min(np.linalg.norm(np.asarray(rectangle) - pt, axis=1)) < 1e-6)
        # shoe angle 0 -> plain rectangle
        plain = DiagonalRib(NARROW[0], NARROW[1], NARROW[0], NARROW[1], band_split=dict(BAND_CFG, flare_angle=0.0))
        outline0, _ = plain.get_band_split_shape_flat(flat_left, flat_right)
        self.assertAlmostEqual(_polygon_area(outline0), _polygon_area(rectangle), delta=1e-9)
        # shoes that would meet (wide range, one strip) keep 10 % of straight strip
        wide = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1], band_split=BAND_CFG)
        fl, fr, (outline_w, _) = self._shape(wide)
        m_lw = np.asarray(fl[fl.walk(0, fl.get_length() / 2)])
        m_rw = np.asarray(fr[fr.walk(0, fr.get_length() / 2)])
        span_w = np.linalg.norm(m_rw - m_lw)
        u_w = (m_rw - m_lw) / span_w
        self.assertAlmostEqual(_width_at(outline_w, m_lw, u_w, 0.5 * span_w), 0.04, delta=0.004)
        self.assertGreater(_width_at(outline_w, m_lw, u_w, 0.4 * span_w), 0.06)
        # mesh and pattern
        mesh = band.get_mesh(self.cell, insert_points=4)
        plain_mesh = DiagonalRib(NARROW[0], NARROW[1], NARROW[0], NARROW[1]).get_mesh(self.cell, insert_points=4)
        self.assertLess(_mesh_area(mesh), 0.7 * _mesh_area(plain_mesh))
        cuts = _real_cuts(DribPlot(band, self.cell, PatternConfig()).flatten([]))
        self.assertEqual(len(cuts), 1)

    def test_connecting_band_with_three_strips(self):
        band = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1], band_split=dict(BAND_CFG, num=3))
        flat_left, flat_right, (outline, holes) = self._shape(band)
        self.assertEqual(len(holes), 2)
        len_l = flat_left.get_length()
        m_l = np.asarray(flat_left[flat_left.walk(0, len_l / 2)])
        m_r = np.asarray(flat_right[flat_right.walk(0, flat_right.get_length() / 2)])
        span = np.linalg.norm(m_r - m_l)
        u = (m_r - m_l) / span
        # three 40 mm strips centred on the thirds of the range, 2 gaps between them
        share = len_l / 3
        self.assertAlmostEqual(_width_at(outline, m_l, u, 0.5 * span), 2 * share + 0.04, delta=0.01)
        for hole in holes:
            self.assertAlmostEqual(_width_at(hole, m_l, u, 0.5 * span), share - 0.04, delta=0.01)
            for pt in hole:
                self.assertTrue(_point_in_polygon(pt, outline))
            # the holes point at the ribs, margin_top away (a bit more: rounded)
            for rib in (flat_left, flat_right):
                dist = min(_dist_to_polyline(p, rib) for p in hole)
                self.assertGreaterEqual(dist, 0.01 - 1e-3)
                self.assertLess(dist, 0.01 + 0.03)
        # the strips line up with the bands of the diagonal reaching this rib:
        # the hole apex sits at the share boundary, like the diagonal's
        for j, hole in enumerate(holes):
            apex = min(hole, key=lambda p: _dist_to_polyline(p, flat_left))
            boundary = np.asarray(flat_left[flat_left.walk(0, (j + 1) / 3 * len_l)])
            self.assertLess(np.linalg.norm(apex - boundary), 0.02)
        mesh = band.get_mesh(self.cell, insert_points=4)
        self.assertGreater(len(mesh.all_polygons), 50)
        cuts = _real_cuts(DribPlot(band, self.cell, PatternConfig()).flatten([]))
        self.assertEqual(len(cuts), 3)

    def test_not_a_full_diagonal(self):
        band = DiagonalRib((0.3, 1), (0.5, 1), (0.3, 1), (0.5, 1), band_split=CFG)
        flat_left, flat_right = band.get_flattened(self.cell)
        self.assertIsNone(band.get_band_split_shape_flat(flat_left, flat_right))

    def test_parametric_mapping_round_trip(self):
        drib = self._drib()
        flat_left, flat_right, (outline, holes) = self._shape(drib)
        outline_param, holes_param = drib._band_split_shape_parametric(self.cell)
        self.assertEqual(len(holes_param), len(holes))
        nl, nr = len(flat_left) - 1, len(flat_right) - 1

        def check(contour_param, contour):
            for (x, y), q in zip(contour_param, contour):
                self.assertTrue(0 <= x <= 1 and 0 <= y <= 1)
                p = np.asarray(flat_left[x * nl]) * (1 - y) + np.asarray(flat_right[x * nr]) * y
                self.assertLess(np.linalg.norm(p - q), 1e-6)

        check(outline_param, outline)
        ys = [p[1] for p in outline_param]
        self.assertAlmostEqual(min(ys), 0.0, places=6)
        self.assertAlmostEqual(max(ys), 1.0, places=6)
        for (contour, centre), hole in zip(holes_param, holes):
            check(contour, hole)
            self.assertTrue(_point_in_polygon(centre, contour))
        q = np.asarray(flat_left[0.3 * nl]) * 0.6 + np.asarray(flat_right[0.3 * nr]) * 0.4
        x, y = _flat_to_parametric(flat_left, flat_right, q)
        self.assertAlmostEqual(x, 0.3, places=6)
        self.assertAlmostEqual(y, 0.4, places=6)

    def test_mesh_and_pattern(self):
        drib = self._drib()
        mesh = drib.get_mesh(self.cell, insert_points=4)
        self.assertGreater(len(mesh.all_polygons), 50)
        # the mesh only covers the trumpet: its area is well below the trapezoid
        plain = DiagonalRib(BASE[0], BASE[1], FAR[0], FAR[1])
        self.assertLess(_mesh_area(mesh), 0.8 * _mesh_area(plain.get_mesh(self.cell, insert_points=4)))
        self.assertGreater(len(self._drib(flare_angle=0.0).get_mesh(self.cell).all_polygons), 50)

        plain_plot = DribPlot(plain, self.cell, PatternConfig()).flatten([])
        split_plot = DribPlot(drib, self.cell, PatternConfig()).flatten([])
        cuts = _real_cuts(split_plot)
        self.assertEqual(len(cuts), 3)  # outline + 2 holes
        outline_cut = cuts[0]
        self.assertTrue(np.allclose(outline_cut[0], outline_cut[len(outline_cut) - 1]))
        self.assertLess(_polygon_area(list(outline_cut.data)[:-1]),
                        _polygon_area(list(_real_cuts(plain_plot)[0].data)))
        # stitches to the ribs are kept
        self.assertEqual(len(split_plot.layers["stitches"]), len(plain_plot.layers["stitches"]))

    def test_json_and_cone_holes(self):
        drib = self._drib()
        self.assertEqual(drib.__json__()["band_split"], CFG)
        self.assertNotIn("band_split", DiagonalRib((0.5, -1), (0.55, -1), (0.4, 1), (0.6, 1)).__json__())
        g2d = openglider.load(DEMOKITE)
        g2d.elements["diagonals"] = [
            {"left_front": BASE[0], "left_back": BASE[1],
             "right_front": FAR[0], "right_back": FAR[1],
             "cells": [3], "band_split": CFG},
            {"left_front": (0.08, -1), "left_back": (0.12, -1),
             "right_front": (0.05, 1), "right_back": (0.15, 1), "cells": [3]},
            {"left_front": FAR[0], "left_back": FAR[1],
             "right_front": FAR[0], "right_back": FAR[1],
             "cells": [4], "band_split": BAND_CFG},
            {"left_front": (0.05, 1), "left_back": (0.15, 1),
             "right_front": (0.05, 1), "right_back": (0.15, 1), "cells": [4]},
        ]
        g2d.diag_holes_enabled = True
        g3d = g2d.get_glider_3d()
        split, plain = g3d.cells[3].diagonals
        if split.band_split is None:
            split, plain = plain, split
        self.assertEqual(split.band_split, CFG)
        self.assertIsNone(getattr(split, "cone_hole_config", None))
        self.assertIsNotNone(getattr(plain, "cone_hole_config", None))
        shaped_band, plain_band = g3d.cells[4].diagonals
        if shaped_band.band_split is None:
            shaped_band, plain_band = plain_band, shaped_band
        self.assertIsNone(getattr(shaped_band, "band_hole_config", None))
        self.assertIsNotNone(getattr(plain_band, "band_hole_config", None))
        reloaded = openglider.jsonify.loads(openglider.jsonify.dumps(g2d))["data"]
        self.assertEqual(reloaded.elements["diagonals"][0]["band_split"], CFG)


def _mesh_area(mesh):
    area = 0.0
    for poly in mesh.all_polygons:
        pts = [np.asarray(list(v)) for v in poly]
        for i in range(1, len(pts) - 1):
            area += np.linalg.norm(np.cross(pts[i] - pts[0], pts[i + 1] - pts[0])) / 2
    return area


@unittest.skipUnless(tool_stubs.HAVE_QT, "PySide6 not available")
class TestAutoFillBandSplit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6 import QtWidgets

        cls._added = tool_stubs._install_stubs()
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        for name in (
            "freecad.glider.tools.cell_tool",
            "freecad.glider.tools.glider",
            "freecad.glider.tools.tools",
            "freecad.glider.tools.table",
            "freecad.glider.tools.pull_axis_utils",
        ):
            sys.modules.pop(name, None)
        cls.module = importlib.import_module("freecad.glider.tools.cell_tool")
        cls.module.draw_glider = lambda *a, **k: None
        cls.module.draw_lines = lambda *a, **k: None
        cls.QtWidgets = QtWidgets

    @classmethod
    def tearDownClass(cls):
        for name in list(sys.modules):
            if name.startswith("freecad"):
                sys.modules.pop(name, None)
        for name in cls._added:
            sys.modules.pop(name, None)

    def setUp(self):
        self.obj = tool_stubs._FreeCADObject(openglider.load(DEMOKITE))
        self.tool = self.module.CellTool(self.obj)
        # the tool works on its own copy of the glider until accept()
        self.g2d = self.tool.parametric_glider
        self.dialogs = []
        QDialog = self.QtWidgets.QDialog
        self._orig_exec = QDialog.exec_
        tests = self

        def fake_exec(dialog):
            tests.dialogs.append(dialog)
            return QDialog.Accepted

        QDialog.exec_ = fake_exec

    def tearDown(self):
        self.QtWidgets.QDialog.exec_ = self._orig_exec

    def _rows(self):
        table = self.tool.diagonals_table
        rows = []
        for r in range(table.table.rowCount()):
            row = table.get_row(r)
            if row:
                rows.append((row, table.get_row_bands(r)))
        return rows

    def _param_table(self):
        return self.dialogs[-1].findChild(self.QtWidgets.QTableWidget)

    def test_percent_values_are_used_as_entered(self):
        params = {
            "A": (4.0, 4.0, 21.0, 1), "B": (4.0, 20.0, 57.0, 1),
            "C": (4.0, 56.0, 80.0, 1), "D": (4.0, 50.0, 75.0, 1),
        }
        self.g2d.diagonal_autofill_params = params
        self.g2d.diagonal_autofill_flare_angle = 0.0
        self.g2d.diagonal_autofill_shoe_angle = 0.0
        self.tool.auto_fill_diagonals()
        table = self._param_table()
        for row, layer in enumerate("ABC"):
            self.assertEqual(table.cellWidget(row, 0).value(), 40.0)  # 4 cm shown as 40 mm
            self.assertEqual(table.cellWidget(row, 1).value(), params[layer][1])
            self.assertEqual(table.cellWidget(row, 2).value(), params[layer][2])
        # the preview echoes the entered range, no pull-axis shift
        self.assertEqual(table.item(0, 5).text(), "4 – 21 %")
        self.assertEqual(table.item(1, 5).text(), "20 – 57 %")
        wanted = {(p[1] / 100, p[2] / 100) for p in params.values()}
        for r, _ in self._rows():
            if r[1] == r[7]:
                continue  # connecting band
            ext = (r[0], r[2]) if r[1] > 0 else (r[6], r[4])
            self.assertIn((round(ext[0], 4), round(ext[1], 4)), wanted, ext)
        # intrados width is stored back in cm
        self.assertEqual(self.g2d.diagonal_autofill_params["A"][0], 4.0)

    def test_one_t_diagonal_per_attachment_point(self):
        self.g2d.diagonal_autofill_params = {
            k: (v[0], v[1], v[2], 3) for k, v in DIAGONAL_AUTOFILL_DEFAULT_PARAMS.items()
        }
        self.g2d.diagonal_autofill_flare_angle = 35.0
        self.g2d.diagonal_autofill_shoe_angle = 50.0
        self.tool.auto_fill_diagonals()
        rows = self._rows()
        diags = [(r, b) for r, b in rows if r[1] != r[7]]
        bands = [(r, b) for r, b in rows if r[1] == r[7]]
        self.assertTrue(diags)
        # one diagonal per (AP, cell, side): no fan of several trapezoids
        bases = []
        for r, _ in diags:
            side, base_x = ("L", r[6]) if r[7] == -1 else ("R", r[0])
            bases += [(side, round(base_x, 3), cell) for cell in r[-1]]
        self.assertEqual(len(bases), len(set(bases)))
        self.assertTrue(all(b == 3 for _, b in diags))
        # connecting bands: as many strips as diagonal bands, shoes on the ribs
        self.assertTrue(bands)
        self.assertTrue(all(b == 3 for _, b in bands))
        # extrados ranges are exactly the configured absolute positions
        wanted = {(p[1] / 100, p[2] / 100) for p in DIAGONAL_AUTOFILL_DEFAULT_PARAMS.values()}
        for r, _ in diags:
            ext = (r[0], r[2]) if r[1] > 0 else (r[6], r[4])
            self.assertIn((round(ext[0], 4), round(ext[1], 4)), wanted)

        self.tool.diagonals_table.apply_to_glider(self.g2d)
        elements = self.g2d.elements["diagonals"]
        split = [e for e in elements if e.get("band_split")]
        self.assertEqual(len(split), len(rows))
        diag_elements = [e for e in split if e["right_front"][1] != e["left_front"][1]]
        band_elements = [e for e in split if e["right_front"][1] == e["left_front"][1]]
        self.assertEqual(len(diag_elements), len(diags))
        self.assertEqual(len(band_elements), len(bands))
        self.assertTrue(all(e["band_split"]["num"] == 3 for e in split))
        self.assertAlmostEqual(diag_elements[0]["band_split"]["flare_angle"], 35.0)
        self.assertNotIn("strip_width", diag_elements[0]["band_split"])
        self.assertAlmostEqual(band_elements[0]["band_split"]["strip_width"], 0.04)  # 4 cm intrados
        self.assertAlmostEqual(band_elements[0]["band_split"]["flare_angle"], 50.0)  # shoe angle
        # the 3D glider builds with the trumpet outline and holes
        g3d = self.g2d.get_glider_3d()
        drib = next(d for c in g3d.cells for d in c.diagonals if d.band_split)
        cell = next(c for c in g3d.cells if drib in c.diagonals)
        outline, holes = drib._band_split_shape_parametric(cell)
        self.assertGreater(len(outline), 10)
        self.assertGreater(len(drib.get_mesh(cell).all_polygons), 20)

    def test_single_band_with_flare_angle_is_a_t(self):
        self.tool.auto_fill_diagonals()  # defaults: 40 deg / 40 deg
        rows = self._rows()
        self.assertTrue(all(b == 1 for _, b in rows))
        self.tool.diagonals_table.apply_to_glider(self.g2d)
        split = [e for e in self.g2d.elements["diagonals"] if e.get("band_split")]
        self.assertEqual(len(split), len(rows))
        self.assertEqual(split[0]["band_split"]["num"], 1)
        self.assertAlmostEqual(split[0]["band_split"]["flare_angle"], 40.0)

    def test_zero_angles_keep_plain_diagonals(self):
        self.g2d.diagonal_autofill_flare_angle = 0.0
        self.g2d.diagonal_autofill_shoe_angle = 0.0
        self.tool.auto_fill_diagonals()
        rows = self._rows()
        self.assertTrue(rows)
        self.assertTrue(all(b is None for _, b in rows))
        self.tool.diagonals_table.apply_to_glider(self.g2d)
        self.assertTrue(all("band_split" not in e for e in self.g2d.elements["diagonals"]))

    def test_range_preview_flags_overlaps(self):
        self.g2d.diagonal_autofill_params = {
            "A": (4.0, 4.0, 21.0, 1), "B": (4.0, 20.0, 57.0, 1),
            "C": (4.0, 60.0, 80.0, 1), "D": (4.0, 82.0, 90.0, 1),
        }
        self.tool.auto_fill_diagonals()
        table = self._param_table()
        texts = [table.item(row, 5).text() for row in range(4)]
        self.assertTrue(all("%" in t for t in texts), texts)
        # A and B overlap between 20 and 21 % -> both flagged, C and D are clean
        self.assertIn("B", table.item(0, 5).toolTip())
        self.assertIn("A", table.item(1, 5).toolTip())
        self.assertEqual(table.item(2, 5).toolTip(), "")
        self.assertEqual(table.item(3, 5).toolTip(), "")

    def test_bands_column_round_trip(self):
        table = self.tool.diagonals_table
        self.g2d.elements["diagonals"] = [
            {"left_front": BASE[0], "left_back": BASE[1],
             "right_front": FAR[0], "right_back": FAR[1],
             "cells": [3], "band_split": dict(CFG), "material_code": "red"},
        ]
        table.get_from_ParametricGlider(self.g2d)
        self.assertEqual(table.get_row_bands(0), 3)
        table.table.item(0, 9).setText("4")
        table.apply_to_glider(self.g2d)
        element = self.g2d.elements["diagonals"][0]
        self.assertEqual(element["band_split"]["num"], 4)
        self.assertEqual(element["band_split"]["flare_angle"], CFG["flare_angle"])
        self.assertEqual(element["material_code"], "red")
        # one band keeps the T (angle > 0), only angle 0 drops it
        table.table.item(0, 9).setText("1")
        table.apply_to_glider(self.g2d)
        self.assertEqual(self.g2d.elements["diagonals"][0]["band_split"]["num"], 1)
        self.g2d.elements["diagonals"][0]["band_split"]["flare_angle"] = 0.0
        table.get_from_ParametricGlider(self.g2d)
        table.table.item(0, 9).setText("1")
        table.apply_to_glider(self.g2d)
        self.assertNotIn("band_split", self.g2d.elements["diagonals"][0])
        # a hand-typed connecting band gets the shoe angle and a strip width
        self.g2d.diagonal_autofill_shoe_angle = 55.0
        self.g2d.elements["diagonals"] = [
            {"left_front": FAR[0], "left_back": FAR[1],
             "right_front": FAR[0], "right_back": FAR[1], "cells": [3]},
        ]
        table.get_from_ParametricGlider(self.g2d)
        table.table.item(0, 9).setText("2")
        table.apply_to_glider(self.g2d)
        band_split = self.g2d.elements["diagonals"][0]["band_split"]
        self.assertEqual(band_split["num"], 2)
        self.assertEqual(band_split["flare_angle"], 55.0)
        self.assertAlmostEqual(band_split["strip_width"], 0.04)


if __name__ == "__main__":
    unittest.main()
