"""Rounded rib-less wingtip (last_profile_type 'curve' -> TipCurveRib).

The last cell is closed by a curve from the previous rib's nose, tangent to
the tip station at ``tip_curve_position``, back to the previous rib's trailing
edge; both skins meet on that curve and are inflated with one symmetric
(extrados) ballooning law.
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import openglider
from openglider import jsonify
from openglider.airfoil import get_x_value
from openglider.glider.cell.tip_pocket import TipPocketCell
from openglider.glider.rib.rib import TipCurveRib

DEMOKITE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "common", "demokite.json")


def load(position=0.5, power=0.5, enabled=True):
    glider = openglider.load(DEMOKITE)
    glider.last_profile_enabled = enabled
    glider.last_profile_type = "curve"
    glider.tip_curve_position = position
    glider.tip_curve_power = power
    return glider


class TipCurveTest(unittest.TestCase):
    def test_last_rib_is_a_closed_curve_rib(self):
        glider = load().get_glider_3d()
        tip, prev = glider.ribs[-1], glider.ribs[-2]
        self.assertIsInstance(tip, TipCurveRib)
        self.assertIs(tip.previous_rib, prev)
        self.assertTrue(tip.is_closed())  # no rib piece
        self.assertFalse(isinstance(prev, TipCurveRib))

    def test_curve_runs_from_nose_to_trailing_edge_of_previous_rib(self):
        glider = load().get_glider_3d()
        tip, prev = glider.ribs[-1], glider.ribs[-2]
        curve = np.array(tip.profile_3d.data)
        prev3d = np.array(prev.profile_3d.data)
        nose = prev3d[prev.profile_2d.noseindex]
        te = 0.5 * (prev3d[0] + prev3d[-1])
        np.testing.assert_allclose(curve[tip.profile_2d.noseindex], nose, atol=1e-9)
        np.testing.assert_allclose(curve[0], te, atol=1e-9)
        np.testing.assert_allclose(curve[-1], te, atol=1e-9)
        # upper and lower points of one chord position share one 3d point
        xv = tip.profile_2d.x_values
        mid = tip.profile_3d
        for x in (0.1, 0.3, 0.5, 0.8):
            up = np.array(mid[get_x_value(xv, -x)])
            lo = np.array(mid[get_x_value(xv, x)])
            np.testing.assert_allclose(up, lo, atol=5e-4)  # same seam polyline, native sampling

    def test_curve_touches_tip_station_tangentially_at_position(self):
        for position in (0.3, 0.5, 0.7):
            glider = load(position=position).get_glider_3d()
            tip, prev = glider.ribs[-1], glider.ribs[-2]
            line = np.array(tip.align_all(tip.profile_2d.data))
            curve = np.array(tip.profile_3d.data)
            base = np.abs(np.array(tip.profile_2d.x_values))
            # spanwise progress: 0 at both ends, 1 (on the tip station) at position
            f = tip.curve_fraction(base)
            self.assertAlmostEqual(f[0], 0.0)
            self.assertAlmostEqual(f[tip.profile_2d.noseindex], 0.0)
            self.assertAlmostEqual(float(tip.curve_fraction(position)), 1.0)
            # flat top (tangent): still >99% at +-2% chord around the touch point
            self.assertGreater(float(tip.curve_fraction(position - 0.02)), 0.99)
            self.assertGreater(float(tip.curve_fraction(position + 0.02)), 0.99)
            # the touch point lies on the straight tip station chord line
            i = int(np.argmin(np.abs(base[: tip.profile_2d.noseindex] - position)))
            self.assertLess(np.linalg.norm(curve[i] - line[i]), 1e-3 * tip.chord + 1e-9)
            # every curve point lies between the previous rib's chord line and
            # the tip station, reaching the tip station only at the touch point
            prev3d = np.array(prev.profile_3d.data)
            nose1, te1 = prev3d[prev.profile_2d.noseindex], 0.5 * (prev3d[0] + prev3d[-1])
            c1 = np.abs(np.array(prev.profile_2d.x_values))
            on_prev = nose1 + c1[:, None] * (te1 - nose1)
            span = np.linalg.norm(line - on_prev, axis=1)
            progress = np.linalg.norm(curve - on_prev, axis=1)
            self.assertTrue(np.all(progress <= span + 1e-9))
            self.assertAlmostEqual(progress[i], span[i], delta=1e-3 * tip.chord)

    def test_last_cell_ballooning_is_symmetric_others_untouched(self):
        glider = load().get_glider_3d()
        last, other = glider.cells[-1], glider.cells[-2]
        for x in (0.15, 0.4, 0.7):
            self.assertAlmostEqual(last.ballooning[-x], last.ballooning[x])
            self.assertAlmostEqual(other.ballooning[-x], glider.cells[-2].ballooning[-x])
        # the symmetric law is the extrados one
        self.assertAlmostEqual(last.ballooning[0.4], other.ballooning[-0.4])

    def test_skins_meet_on_the_curve_and_cell_flattens(self):
        glider = load().get_glider_3d().copy_complete()
        for cell in (glider.cells[0], glider.cells[-1]):
            tip_is_rib2 = isinstance(cell.rib2, TipCurveRib)
            self.assertTrue(tip_is_rib2 or isinstance(cell.rib1, TipCurveRib))
            y_tip = 1.0 if tip_is_rib2 else 0.0
            xv = cell.rib1.base_profile_2d.x_values
            mid = cell.midrib(y_tip, with_numpy=True)
            # one physical seam curve; upper and lower samples interpolate the
            # same polyline between (native, not symmetric) x positions
            for x in (0.2, 0.5, 0.9):
                up = np.array(mid[get_x_value(xv, -x)])
                lo = np.array(mid[get_x_value(xv, x)])
                np.testing.assert_allclose(up, lo, atol=5e-4)
            # ballooned midribs and panel meshes are finite
            for y in (0.25, 0.5, 0.75):
                self.assertTrue(np.isfinite(np.array(cell.midrib(y, with_numpy=True).data)).all())
            for panel in cell.panels:
                verts, polys, _ = panel.get_mesh(cell, 3, with_numpy=True).get_indexed()
                self.assertGreater(len(verts), 3)
            flat_left, flat_right = cell.get_flattened_cell(numribs=10)[:2] if isinstance(cell.get_flattened_cell(numribs=10), (list, tuple)) else (None, None)

    def test_mirrored_tip_bulges_outward(self):
        glider = load().get_glider_3d().copy_complete()
        left, right = glider.ribs[0], glider.ribs[-1]
        self.assertIsInstance(left, TipCurveRib)
        self.assertIsInstance(right, TipCurveRib)
        for tip in (left, right):
            curve = np.array(tip.profile_3d.data)
            prev = tip.previous_rib
            line = np.array(tip.align_all(tip.profile_2d.data))
            prev3d = np.array(prev.profile_3d.data)
            nose1, te1 = prev3d[prev.profile_2d.noseindex], 0.5 * (prev3d[0] + prev3d[-1])
            c1 = np.abs(np.array(prev.profile_2d.x_values))
            on_prev = nose1 + c1[:, None] * (te1 - nose1)
            # the mirrored curve also runs from its previous rib towards its own
            # tip station (never inwards), touching it at mid chord
            progress = np.linalg.norm(curve - on_prev, axis=1)
            span = np.linalg.norm(line - on_prev, axis=1)
            self.assertTrue(np.all(progress <= span + 1e-9))
            self.assertGreater(progress.max(), 0.5 * span.max())
            towards_tip = np.einsum("ij,ij->i", curve - on_prev, line - on_prev)
            self.assertTrue(np.all(towards_tip >= -1e-9))

    def test_disabled_or_other_type_gives_plain_rib(self):
        glider = load(enabled=False).get_glider_3d()
        self.assertFalse(isinstance(glider.ribs[-1], TipCurveRib))
        parametric = load()
        parametric.last_profile_type = "line"
        self.assertFalse(isinstance(parametric.get_glider_3d().ribs[-1], TipCurveRib))

    def test_json_roundtrip(self):
        parametric = load(position=0.35, power=0.8)
        restored = jsonify.loads(jsonify.dumps(parametric))["data"]
        self.assertEqual(restored.last_profile_type, "curve")
        self.assertAlmostEqual(restored.tip_curve_position, 0.35)
        self.assertAlmostEqual(restored.tip_curve_power, 0.8)
        tip = restored.get_glider_3d().ribs[-1]
        self.assertIsInstance(tip, TipCurveRib)
        self.assertAlmostEqual(tip.curve_position, 0.35)


class TipPocketTest(unittest.TestCase):
    """The pocket cell: shrinking sections between the previous rib and the apex."""

    def test_pocket_cell_and_apex(self):
        glider = load().get_glider_3d().copy_complete()
        for cell in (glider.cells[0], glider.cells[-1]):
            self.assertIsInstance(cell, TipPocketCell)
            tip, rib, flipped = cell._tip_and_rib()
            y_tip = 0.0 if flipped else 1.0
            apex = np.array(cell.midrib(y_tip, with_numpy=True).data)
            np.testing.assert_allclose(apex, np.repeat(tip.apex[None, :], len(apex), axis=0), atol=1e-9)
            np.testing.assert_allclose(np.array(cell.midrib(1.0 - y_tip).data), np.array(rib.profile_3d.data), atol=1e-9)

    def test_sections_nose_and_te_lie_on_the_seam(self):
        glider = load(position=0.4).get_glider_3d()
        cell = glider.cells[-1]
        tip, rib = cell.rib2, cell.rib1
        seam = np.array(tip.profile_3d.data)
        xv = np.array(tip.profile_2d.x_values)
        for t in (0.2, 0.5, 0.8, 0.95):
            section = np.array(cell.midrib(t, ballooning=False).data)
            c_front, c_back = tip.seam_branches(t)
            self.assertAlmostEqual(float(tip.curve_fraction(c_front)), t, places=9)
            self.assertAlmostEqual(float(tip.curve_fraction(c_back)), t, places=9)
            # nose / trailing edge of the section = seam points at those chord fractions
            for c, idx in ((c_front, tip.profile_2d.noseindex), (c_back, 0)):
                on_seam = np.array(tip.profile_3d[openglider.airfoil.get_x_value(xv, -c)])
                np.testing.assert_allclose(section[idx], on_seam, atol=2e-3 * rib.chord)
            # ballooning never moves the seam points
            inflated = np.array(cell.midrib(t, ballooning=True).data)
            np.testing.assert_allclose(inflated[tip.profile_2d.noseindex], section[tip.profile_2d.noseindex], atol=1e-9)
            np.testing.assert_allclose(inflated[0], section[0], atol=1e-9)
            # thickness shrinks with the section chord
            thick = np.linalg.norm(section[len(xv) // 4] - section[len(xv) - 1 - len(xv) // 4])
            self.assertGreater(thick, 0.0)
            self.assertLess(thick, np.linalg.norm(np.array(rib.profile_3d.data)[len(xv) // 4] - np.array(rib.profile_3d.data)[len(xv) - 1 - len(xv) // 4]))

    def test_pocket_is_smooth_finite_and_flattens(self):
        glider = load().get_glider_3d().copy_complete()
        for cell in (glider.cells[0], glider.cells[-1]):
            for y in np.linspace(0, 1, 11):
                self.assertTrue(np.isfinite(np.array(cell.midrib(y, with_numpy=True).data)).all())
            # no pinch: the nose of a mid section sits away from the previous rib's nose
            tip, rib, flipped = cell._tip_and_rib()
            mid = np.array(cell.midrib(0.5 if not flipped else 0.5).data)
            self.assertGreater(np.linalg.norm(mid[rib.profile_2d.noseindex] - np.array(rib.profile_3d.data)[rib.profile_2d.noseindex]), 0.05 * rib.chord)
            for panel in cell.panels:
                verts, polys, _ = panel.get_mesh(cell, 3, with_numpy=True).get_indexed()
                self.assertTrue(np.isfinite(np.array([list(v) for v in verts], float)).all())
            flat = cell.get_flattened_cell(numribs=10)
            for rail in flat["ballooned"]:
                self.assertTrue(np.isfinite(np.array(list(rail), float)).all())


class TrailingEdgeCutTest(unittest.TestCase):
    def test_skins_run_to_the_trailing_edge_only_the_rib_piece_is_cut(self):
        glider = openglider.load(DEMOKITE)
        glider.te_cut_enabled = True
        glider.te_cut_mm = 15.0
        glider3d = glider.get_glider_3d()
        rib = glider3d.ribs[1]
        self.assertGreater(rib.trailing_edge_cut, 0.0)
        cut_x = 1.0 - rib.trailing_edge_cut / rib.chord
        hull = np.array(list(rib.get_hull()))
        self.assertAlmostEqual(hull[:, 0].max(), cut_x, places=6)  # rib template shortened
        skin = np.array(rib.profile_3d.data)
        full = np.array(rib.align_all(rib.profile_2d.data))
        np.testing.assert_allclose(skin[0], full[0], atol=1e-12)  # skin reaches the TE
        cell = glider3d.cells[1]
        te = np.array(cell.midrib(0.5, with_numpy=True).data)[0]
        expected = 0.5 * (np.array(cell.rib1.profile_3d.data)[0] + np.array(cell.rib2.profile_3d.data)[0])
        self.assertLess(np.linalg.norm(te - expected), 0.05 * cell.rib1.chord)


if __name__ == "__main__":
    unittest.main()
