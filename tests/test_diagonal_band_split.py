"""Band-split ("T") diagonals: independent bands as wide as the intrados
base that rise straight, then flare out at the flare angle to their share
of the extrados; connecting bands made of one shoe-and-strip piece per band.

Covers the DiagonalRib geometry (pieces in the flattened frame, parametric
mapping, 3D mesh), the 2D pattern (one plot part per piece with seam and
hem allowances) and the Edit Cells auto-fill that produces them (headless,
FreeCAD/pivy stubbed as in test_le_panel_split_tool.py).
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
    _edge_kinds,
    _flat_to_parametric,
    _point_in_polygon,
    _sub_polyline,
)
from openglider.glider.parametric.glider import DIAGONAL_AUTOFILL_DEFAULT_PARAMS
from openglider.plots.glider.cell import CellPlotMaker, DribPlot
from openglider.plots.glider.config import PatternConfig

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")
CFG = {"num": 3, "flare_angle": 40.0}
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


def _dist_to_points(point, pts):
    """Distance from a point to an open polyline given as an array of points."""
    pts = np.asarray(pts)
    p = np.asarray(point)
    best = float("inf")
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        t = float(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-18))
        t = min(max(t, 0.0), 1.0)
        best = min(best, float(np.linalg.norm(a + t * ab - p)))
    return best


def _dist_to_polyline(point, polyline):
    return _dist_to_points(point, polyline.data)


def _dist_to_polygon(point, polygon):
    return _dist_to_points(point, list(polygon) + [polygon[0]])


def _inside_or_on(point, polygon, tol=1e-6):
    return _point_in_polygon(point, polygon) or _dist_to_polygon(point, polygon) < tol


def _real_cuts(plotpart):
    return [c for c in plotpart.layers["cuts"] if len(c) >= 3]


def _self_intersections(points):
    """Number of crossings between non-adjacent edges of a closed polygon."""
    pts = np.asarray(points)
    n = len(pts)

    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])

    count = 0
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        for j in range(i + 2, n):
            if (j + 1) % n == i:
                continue
            c, d = pts[j], pts[(j + 1) % n]
            if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
                count += 1
    return count


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


def _mesh_area(mesh):
    area = 0.0
    for poly in mesh.all_polygons:
        pts = [np.asarray(list(v)) for v in poly]
        for i in range(1, len(pts) - 1):
            area += np.linalg.norm(np.cross(pts[i] - pts[0], pts[i + 1] - pts[0])) / 2
    return area


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

    def _pieces(self, drib):
        flat_left, flat_right = drib.get_flattened(self.cell)
        return flat_left, flat_right, drib.get_band_split_pieces_flat(flat_left, flat_right)

    def _frame(self, base, far):
        width = base.get_length()
        p_ap = np.asarray(base[base.walk(0, width / 2)])
        up = np.asarray(far[far.walk(0, far.get_length() / 2)]) - p_ap
        height = np.linalg.norm(up)
        return width, p_ap, up / height, height

    def test_single_band_is_a_stem_then_a_flare(self):
        for mirrored in (False, True):
            drib = self._drib(mirrored=mirrored, num=1)
            flat_left, flat_right, pieces = self._pieces(drib)
            self.assertEqual(len(pieces), 1)
            outline = pieces[0]
            base = flat_right if mirrored else flat_left
            far = flat_left if mirrored else flat_right
            trapezoid = list(base.data) + list(far.data)[::-1]
            width, p_ap, up, height = self._frame(base, far)
            # near the base the piece is as wide as the base (stem)
            for h in (0.1, 0.2):
                self.assertAlmostEqual(_width_at(outline, p_ap, up, h * height), width, delta=0.25 * width)
            # then it widens to the full extrados range
            self.assertGreater(_width_at(outline, p_ap, up, 0.9 * height), 2 * width)
            self.assertLess(_polygon_area(outline), 0.85 * _polygon_area(trapezoid))
            for pt in outline:
                self.assertTrue(_inside_or_on(pt, trapezoid))

    def test_three_independent_bands(self):
        drib = self._drib()
        flat_left, flat_right, pieces = self._pieces(drib)
        self.assertEqual(len(pieces), 3)
        far_len = flat_right.get_length()
        base_pts = np.asarray(flat_left.data)
        width, p_ap, up, height = self._frame(flat_left, flat_right)
        trapezoid = list(flat_left.data) + list(flat_right.data)[::-1]
        for k, piece in enumerate(pieces):
            # every band carries the full intrados base ...
            for corner in (base_pts[0], base_pts[-1]):
                self.assertLess(min(np.linalg.norm(np.asarray(piece) - corner, axis=1)), 1e-6)
            # ... is as wide as the base near it, and stays inside the trapezoid
            share = _sub_polyline(flat_right, k / 3, (k + 1) / 3)
            _, _, axis, h_share = self._frame(flat_left, share)
            self.assertAlmostEqual(_width_at(piece, p_ap, axis, 0.15 * h_share), width, delta=0.3 * width)
            for pt in piece:
                self.assertTrue(_inside_or_on(pt, trapezoid))
            # its extrados seam is exactly its share (k-th third of the range)
            for t in (k / 3 + 0.02, (k + 0.5) / 3, (k + 1) / 3 - 0.02):
                q = np.asarray(flat_right[flat_right.walk(0, t * far_len)])
                self.assertLess(_dist_to_polygon(q, piece), 1e-6)
            outside = np.asarray(flat_right[flat_right.walk(0, ((k + 1.5) % 3) / 3 * far_len)])
            self.assertGreater(_dist_to_polygon(outside, piece), 0.01)
        # neighbouring bands touch on the extrados at the share boundary
        boundary = np.asarray(flat_right[flat_right.walk(0, far_len / 3)])
        self.assertLess(_dist_to_polygon(boundary, pieces[0]), 1e-6)
        self.assertLess(_dist_to_polygon(boundary, pieces[1]), 1e-6)

    def test_flare_angle_is_measured_from_the_extrados(self):
        drib = self._drib(corner_radius=0.0)
        flat_left, flat_right, pieces = self._pieces(drib)
        far_len = flat_right.get_length()
        for k, piece in enumerate(pieces):
            pts = [np.asarray(p) for p in piece]
            for t_end in (k / 3, (k + 1) / 3):
                end = np.asarray(flat_right[flat_right.walk(0, t_end * far_len)])
                i_end = min(range(len(pts)), key=lambda i: np.linalg.norm(pts[i] - end))
                tang = (np.asarray(flat_right[flat_right.walk(0, min(t_end + 0.05, 1) * far_len)])
                        - np.asarray(flat_right[flat_right.walk(0, max(t_end - 0.05, 0) * far_len)]))
                tang = tang / np.linalg.norm(tang)
                # of the two neighbours of the share end, the one off the
                # extrados starts the flare edge: 40 deg from the extrados line
                neighbours = [pts[i_end - 1], pts[(i_end + 1) % len(pts)]]
                flare = max(neighbours, key=lambda q: _dist_to_polyline(q, flat_right))
                edge = flare - pts[i_end]
                cos_a = abs(np.dot(edge, tang)) / np.linalg.norm(edge)
                self.assertAlmostEqual(np.degrees(np.arccos(cos_a)), 40.0, delta=1.0)

    def test_flare_angle_zero_is_the_plain_trapezoid(self):
        drib = self._drib(flare_angle=0.0, num=1)
        flat_left, flat_right, pieces = self._pieces(drib)
        trapezoid = list(flat_left.data) + list(flat_right.data)[::-1]
        self.assertAlmostEqual(_polygon_area(pieces[0]), _polygon_area(trapezoid), delta=1e-9)

    def test_too_narrow_range_drops_bands(self):
        # three 35 mm bands do not fit in a 10 cm range: two pieces; a 6 cm
        # range keeps a single one; the connecting band follows the same rule
        drib = DiagonalRib(BASE[0], BASE[1], NARROW[0], NARROW[1], band_split=CFG)
        flat_left, flat_right = drib.get_flattened(self.cell)
        self.assertEqual(len(drib.get_band_split_pieces_flat(flat_left, flat_right)), 2)
        band = DiagonalRib(NARROW[0], NARROW[1], NARROW[0], NARROW[1],
                           band_split=dict(BAND_CFG, num=3, strip_width=0.035))
        flat_left, flat_right = band.get_flattened(self.cell)
        self.assertEqual(len(band.get_band_split_pieces_flat(flat_left, flat_right)), 2)
        very_narrow = ((0.50, 1), (0.57, 1))
        drib = DiagonalRib(BASE[0], BASE[1], very_narrow[0], very_narrow[1], band_split=CFG)
        fl, fr = drib.get_flattened(self.cell)
        self.assertEqual(len(drib.get_band_split_pieces_flat(fl, fr)), 1)

    def test_connecting_band_pieces(self):
        band = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1], band_split=dict(BAND_CFG, num=3))
        flat_left, flat_right, pieces = self._pieces(band)
        self.assertEqual(len(pieces), 3)
        len_l = flat_left.get_length()
        rectangle = list(flat_left.data) + list(flat_right.data)[::-1]
        for k, piece in enumerate(pieces):
            l_share = _sub_polyline(flat_left, k / 3, (k + 1) / 3)
            r_share = _sub_polyline(flat_right, k / 3, (k + 1) / 3)
            m_l = np.asarray(l_share[l_share.walk(0, l_share.get_length() / 2)])
            m_r = np.asarray(r_share[r_share.walk(0, r_share.get_length() / 2)])
            span = np.linalg.norm(m_r - m_l)
            u = (m_r - m_l) / span
            # thin strip of the intrados width in the middle, centred on the share
            self.assertAlmostEqual(_width_at(piece, m_l, u, 0.5 * span), 0.04, delta=0.004)
            # shoes cover the share on both ribs
            for share in (l_share, r_share):
                for t in (0.05, 0.5, 0.95):
                    q = np.asarray(share[share.walk(0, t * share.get_length())])
                    self.assertLess(_dist_to_polygon(q, piece), 1e-6)
            # shoe length = (half share - half strip) * tan(40 deg from the rib line)
            half_gap = len_l / 6 - 0.02
            shoe = half_gap * np.tan(np.radians(40))
            self.assertGreater(_width_at(piece, m_l, u, shoe * 0.5), 0.04 + half_gap * 0.5)
            self.assertAlmostEqual(_width_at(piece, m_l, u, shoe * 1.3), 0.04, delta=0.004)
            for pt in piece:
                self.assertTrue(_inside_or_on(pt, rectangle))
        # a single wide strip whose shoes would meet keeps 10 % of straight strip
        wide = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1], band_split=BAND_CFG)
        fl, fr, (outline_w,) = self._pieces(wide)
        m_lw = np.asarray(fl[fl.walk(0, fl.get_length() / 2)])
        m_rw = np.asarray(fr[fr.walk(0, fr.get_length() / 2)])
        span_w = np.linalg.norm(m_rw - m_lw)
        u_w = (m_rw - m_lw) / span_w
        self.assertAlmostEqual(_width_at(outline_w, m_lw, u_w, 0.5 * span_w), 0.04, delta=0.004)
        self.assertGreater(_width_at(outline_w, m_lw, u_w, 0.4 * span_w), 0.06)
        # shoe angle 0 -> plain rectangle
        plain = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1], band_split=dict(BAND_CFG, flare_angle=0.0))
        (outline0,) = plain.get_band_split_pieces_flat(fl, fr)
        self.assertAlmostEqual(_polygon_area(outline0), _polygon_area(rectangle), delta=1e-9)

    def test_not_a_full_diagonal(self):
        band = DiagonalRib((0.3, 1), (0.5, 1), (0.3, 1), (0.5, 1), band_split=CFG)
        flat_left, flat_right = band.get_flattened(self.cell)
        self.assertIsNone(band.get_band_split_pieces_flat(flat_left, flat_right))

    def test_parametric_mapping_round_trip(self):
        drib = self._drib()
        flat_left, flat_right, pieces = self._pieces(drib)
        param = drib._band_split_pieces_parametric(self.cell)
        self.assertEqual(len(param), len(pieces))
        nl, nr = len(flat_left) - 1, len(flat_right) - 1
        for contour_param, contour in zip(param, pieces):
            for (x, y), q in zip(contour_param, contour):
                self.assertTrue(0 <= x <= 1 and 0 <= y <= 1)
                p = np.asarray(flat_left[x * nl]) * (1 - y) + np.asarray(flat_right[x * nr]) * y
                self.assertLess(np.linalg.norm(p - q), 1e-6)
            ys = [p[1] for p in contour_param]
            self.assertAlmostEqual(min(ys), 0.0, places=6)
            self.assertAlmostEqual(max(ys), 1.0, places=6)
        q = np.asarray(flat_left[0.3 * nl]) * 0.6 + np.asarray(flat_right[0.3 * nr]) * 0.4
        x, y = _flat_to_parametric(flat_left, flat_right, q)
        self.assertAlmostEqual(x, 0.3, places=6)
        self.assertAlmostEqual(y, 0.4, places=6)

    def test_mesh(self):
        drib = self._drib()
        mesh = drib.get_mesh(self.cell, insert_points=4)
        self.assertGreater(len(mesh.all_polygons), 50)
        plain = DiagonalRib(BASE[0], BASE[1], FAR[0], FAR[1])
        plain_area = _mesh_area(plain.get_mesh(self.cell, insert_points=4))
        # three stems + flares: less fabric than the trapezoid (the stems overlap)
        self.assertLess(_mesh_area(mesh), 0.95 * plain_area)
        self.assertGreater(_mesh_area(mesh), 0.3 * plain_area)
        self.assertGreater(len(self._drib(flare_angle=0.0).get_mesh(self.cell).all_polygons), 50)
        band = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1], band_split=dict(BAND_CFG, num=3))
        band_mesh = band.get_mesh(self.cell, insert_points=4)
        plain_band = DiagonalRib(FAR[0], FAR[1], FAR[0], FAR[1]).get_mesh(self.cell, insert_points=4)
        self.assertLess(_mesh_area(band_mesh), 0.7 * _mesh_area(plain_band))

    def test_pattern_one_piece_per_band(self):
        config = PatternConfig()
        drib = self._drib()
        plot = DribPlot(drib, self.cell, config)
        parts = plot.flatten_pieces([])
        self.assertEqual(len(parts), 3)
        self.assertEqual([p.name for p in parts], ["unnamed-1", "unnamed-2", "unnamed-3"])
        flat_left, flat_right, pieces = self._pieces(drib)
        far_len = flat_right.get_length()
        for k, (part, piece) in enumerate(zip(parts, pieces)):
            cuts = _real_cuts(part)
            self.assertEqual(len(cuts), 1)
            cut = list(cuts[0].data)[:-1]
            self.assertGreater(_polygon_area(cut), _polygon_area(piece))
            # seam allowance along the extrados share and the base
            q = np.asarray(flat_right[flat_right.walk(0, (k + 0.5) / 3 * far_len)])
            self.assertAlmostEqual(_dist_to_polygon(q, cut), config.allowance_general, delta=0.001)
            base_mid = np.asarray(flat_left[flat_left.walk(0, flat_left.get_length() / 2)])
            self.assertAlmostEqual(_dist_to_polygon(base_mid, cut), config.allowance_general, delta=0.001)
            # hem allowance along the free stem edge
            kinds = _edge_kinds(piece, [flat_left, flat_right])
            self.assertIn("seam", kinds)
            self.assertIn("free", kinds)
            i_free = next(i for i, kind in enumerate(kinds) if kind == "free")
            mid_free = (np.asarray(piece[i_free]) + np.asarray(piece[(i_free + 1) % len(piece)])) / 2
            self.assertAlmostEqual(_dist_to_polygon(mid_free, cut), config.drib_allowance_folds, delta=0.001)
            # the cut line never loops or crosses itself
            self.assertEqual(_self_intersections(cut), 0)
            # stitch lines on the sewn edges only, front -> back along the rib
            stitches = list(part.layers["stitches"])
            self.assertEqual(len(stitches), 2)
            for stitch in stitches:
                for p in stitch.data:
                    self.assertLess(min(_dist_to_polyline(p, flat_left), _dist_to_polyline(p, flat_right)), 1e-6)
            # the name sits in the seam allowance of the longest seam (the
            # extrados share): outside the piece, centred in the allowance
            text_pts = [np.asarray(t.data[0]) for t in part.layers["text"]] + \
                       [np.asarray(c.data[0]) for c in part.layers["cuts"] if len(c) < 3]
            self.assertGreater(len(text_pts), 0)
            for tp in text_pts:
                self.assertFalse(_point_in_polygon(tp, piece))
                dist = _dist_to_polyline(tp, flat_right)
                self.assertGreater(dist, config.allowance_general * 0.15)
                self.assertLess(dist, config.allowance_general * 0.85)
            # assembly marks: ticks across the allowance at both ends of both
            # seams, front arrow / back double line, laser dots
            marks = list(part.layers["marks"])
            ticks = [m for m in marks if m.name == "band_split_seam"]
            self.assertEqual(len(ticks), 4)
            for tick in ticks:
                a, b = np.asarray(tick.data[0]), np.asarray(tick.data[-1])
                self.assertAlmostEqual(np.linalg.norm(b - a), config.allowance_general, delta=1e-6)
                self.assertLess(min(_dist_to_polyline(a, flat_left), _dist_to_polyline(a, flat_right)), 1e-6)
            self.assertEqual(len([m for m in marks if m.name == "diagonal_front"]), 2)
            self.assertEqual(len([m for m in marks if m.name == "diagonal_back"]), 2)
            self.assertGreaterEqual(len(list(part.layers["L0"])), 4)
        # plain diagonals still give a single part, and the cell collects all pieces
        plain = DiagonalRib(BASE[0], BASE[1], FAR[0], FAR[1])
        self.assertEqual(len(DribPlot(plain, self.cell, config).flatten_pieces([])), 1)
        self.cell.diagonals = [drib, plain]
        try:
            dribs = CellPlotMaker(self.cell, [], config).get_dribs()
        finally:
            self.cell.diagonals = []
        self.assertEqual(len(dribs), 4)

    def test_piece_names_upright_and_wing_labelled(self):
        from openglider.vector.text import Text

        class Cfg(PatternConfig):
            laser_text_mode = False

        config = Cfg()

        def strokes(part):
            return [np.asarray(t.data) for t in part.layers["text"]]

        def signed_area(pts):
            return 0.5 * float(np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]))

        # mirror-image pieces (base on rib1 / base on rib2): every glyph stroke
        # keeps the orientation of the same text drawn horizontally (no mirror),
        # and the text reads left to right / bottom to top on the sheet
        for mirrored in (False, True):
            drib = self._drib(mirrored=mirrored, num=1)
            part = DribPlot(drib, self.cell, config).flatten_pieces([])[0]
            texts = strokes(part)
            self.assertTrue(texts)
            ref = [np.asarray(t.data) for t in Text(
                f" {part.name} ", np.array([0.0, 0.0]), np.array([0.05, 0.0]),
                size=0.006, align="center", height=0.8, valign=0.0).get_vectors()]
            self.assertEqual(len(texts), len(ref))
            for stroke, ref_stroke in zip(texts, ref):
                a, b = signed_area(stroke), signed_area(ref_stroke)
                if abs(b) > 1e-9:
                    self.assertEqual(np.sign(a), np.sign(b))
            # reading direction: first letter left of (or below) the last one
            first, last = texts[0].mean(axis=0), texts[-1].mean(axis=0)
            self.assertTrue(last[0] > first[0] - 1e-9 or (abs(last[0] - first[0]) < 1e-6 and last[1] > first[1]))

        # wing side label on complete-glider exports only
        plot = DribPlot(self._drib(num=1), self.cell, config)
        self.assertEqual(plot._piece_label("c4d1-2"), "c4d1-2")
        full = self.glider_3d.copy_complete()
        left_cell, right_cell = full.cells[0], full.cells[-1]
        self.assertEqual(left_cell.wing_side, "left")
        self.assertEqual(right_cell.wing_side, "right")
        drib = DiagonalRib(BASE[0], BASE[1], FAR[0], FAR[1], band_split=CFG)
        self.assertEqual(DribPlot(drib, left_cell, config)._piece_label("c1d1-1"), "c1d1-1 G")
        self.assertEqual(DribPlot(drib, right_cell, config)._piece_label("c9d1-1"), "c9d1-1 D")

    def test_rib_and_panel_marks_at_band_boundaries(self):
        from openglider.plots.glider.ribs import RibPlot

        g2d = openglider.load(DEMOKITE)
        g2d.elements["diagonals"] = [
            {"left_front": BASE[0], "left_back": BASE[1],
             "right_front": FAR[0], "right_back": FAR[1],
             "cells": [3], "band_split": CFG},
            {"left_front": FAR[0], "left_back": FAR[1],
             "right_front": FAR[0], "right_back": FAR[1],
             "cells": [4], "band_split": dict(BAND_CFG, num=3)},
        ]
        g3d = g2d.get_glider_3d()
        cell3, cell4 = g3d.cells[3], g3d.cells[4]
        drib = cell3.diagonals[0]
        band = cell4.diagonals[0]
        # the diagonal: two boundaries on the extrados of rib2, none on its base
        self.assertEqual(drib.get_band_split_marks(cell3, right=False), [])
        marks = drib.get_band_split_marks(cell3, right=True)
        self.assertEqual(len(marks), 2)
        xs = [x for x, h in marks]
        self.assertTrue(all(h == 1 for x, h in marks))
        self.assertTrue(FAR[0][0] < xs[0] < xs[1] < FAR[1][0])
        # they sit at the thirds of the extrados arc length
        rib = cell3.rib2
        flat_left, flat_right = drib.get_flattened(cell3)
        for k, x in enumerate(xs):
            # compare through the rib: distance along the extrados from the share start
            self.assertAlmostEqual(
                _sub_polyline(flat_right, 0, (k + 1) / 3).get_length(),
                rib.profile_2d[rib.profile_2d(-FAR[0][0]):rib.profile_2d(-x)].get_length() * rib.chord,
                delta=0.003,
            )
        # the connecting band: boundaries on both ribs
        self.assertEqual(len(band.get_band_split_marks(cell4, right=False)), 2)
        self.assertEqual(len(band.get_band_split_marks(cell4, right=True)), 2)
        # rib template of rib 4 (rib2 of cell 3, rib1 of cell 4): 4 band marks
        config = PatternConfig()
        plot = RibPlot(rib, config)
        plot.flatten(g3d)
        names = [m.name for m in plot.plotpart.layers["marks"]]
        self.assertEqual(names.count("band_split"), 4)
        # panels of cell 3: the upper panel carries the marks on its right edge
        panels = CellPlotMaker(cell3, g3d.lineset.attachment_points, config).get_panels()
        panel_marks = sum(
            [m.name for m in panel.layers["marks"]].count("band_split") for panel in panels
        )
        self.assertGreaterEqual(panel_marks, 2)

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

    def test_one_element_per_attachment_point_with_three_pieces(self):
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
        # one element per (AP, cell, side): no fan of several trapezoids
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
        # the 3D glider builds with three pieces on the widest range (layer D),
        # fewer where the range is too narrow for three 40 mm bands
        g3d = self.g2d.get_glider_3d()
        candidates = [(d, c) for c in g3d.cells for d in c.diagonals if d.band_split]

        def range_length(dc):
            flat_left, flat_right = dc[0].get_flattened(dc[1])
            return max(flat_left.get_length(), flat_right.get_length())

        drib, cell = max(candidates, key=range_length)
        self.assertEqual(len(drib._band_split_pieces_parametric(cell)), 3)
        self.assertGreater(len(drib.get_mesh(cell).all_polygons), 20)
        for d, c in candidates:
            self.assertGreaterEqual(len(d._band_split_pieces_parametric(c)), 1)

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
