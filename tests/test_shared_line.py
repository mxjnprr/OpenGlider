"""
Shared lines: one leg of a symmetric pair (e.g. the central brake lines tied
to both brake handles).  The knot sits on the symmetry plane and the tension
of the leg comes from the mirrored force balance.
"""
import os
import re
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import TestCase  # noqa: E402

from openglider import jsonify  # noqa: E402
from openglider.glider.parametric.lines import (  # noqa: E402
    BatchNode2D,
    Line2D,
    LowerNode2D,
    UpperNode2D,
)
from openglider.lines import Line, LineSet, Node  # noqa: E402
from openglider.lines.lineset import SYMMETRY_PLANE_MASK  # noqa: E402
from openglider.vector.functions import norm, normalize  # noqa: E402

HANDLE_Y = 0.3


def build_lineset(shared, handle_y=HANDLE_Y, leg_length=5.0):
    """brake handle -> knot -> two trailing-edge points of the right half"""
    handle = Node(0, [1.0, handle_y, -7.0], name="brake")
    knot = Node(1, name="knot")
    ap1 = Node(2, [1.5, 0.4, 0.0], name="F1")
    ap1.force = np.array([0.0, 0.0, 8.0])
    ap2 = Node(2, [1.6, 1.2, -0.1], name="F2")
    ap2.force = np.array([0.0, 0.5, 6.0])
    leg = Line(handle, knot, None, target_length=leg_length, name="leg", shared=shared)
    lines = [
        leg,
        Line(knot, ap1, None, name="u1"),
        Line(knot, ap2, None, name="u2"),
    ]
    lineset = LineSet(lines, v_inf=[10.0, 0.0, 1.0])
    lineset._set_line_indices()
    return lineset, leg, knot


def solve(lineset, iterations=8):
    for _ in range(iterations):
        lineset.recalc(calculate_sag=False)
    return lineset


class SharedLineForceTest(unittest.TestCase):
    def test_knot_stays_on_symmetry_plane(self):
        lineset, leg, knot = build_lineset(shared=True)
        solve(lineset)
        self.assertAlmostEqual(knot.vec[1], 0.0, places=9)
        # the leg keeps its length
        self.assertAlmostEqual(norm(knot.vec - leg.lower_node.vec), 5.0, places=6)
        # a plain line would let the knot drift off the plane
        lineset_plain, _, knot_plain = build_lineset(shared=False)
        solve(lineset_plain)
        self.assertGreater(knot_plain.vec[1], 0.1)

    def test_leg_tension_balances_in_plane_resultant(self):
        lineset, leg, knot = build_lineset(shared=True)
        solve(lineset)
        upper_force = sum(
            line.force * line.diff_vector
            for line in lineset.get_upper_connected_lines(knot)
        )
        in_plane = upper_force * SYMMETRY_PLANE_MASK
        direction = normalize(leg.diff_vector) * SYMMETRY_PLANE_MASK
        residual = in_plane - leg.force * direction
        self.assertLess(norm(residual), 1e-3 * norm(in_plane))
        # the leg is splayed (y from 0.3 to 0), so it carries more than the
        # in-plane resultant, by 1/cos of the splay angle
        cos_splay = norm(direction)
        self.assertLess(cos_splay, 1.0)
        self.assertAlmostEqual(leg.force, norm(in_plane) / cos_splay, places=6)
        # the residual reported for the knot is in-plane and vanishes
        knot_residual = lineset.get_residual_force(knot)
        self.assertEqual(knot_residual[1], 0.0)
        self.assertLess(norm(knot_residual), 1e-3 * norm(in_plane))

    def test_shared_uppermost_line(self):
        # a shared line tied directly to an attachment point on the plane
        handle = Node(0, [1.0, HANDLE_Y, -7.0], name="brake")
        point = Node(2, [1.4, 0.0, 0.0], name="F0")
        point.force = np.array([0.0, 0.0, 10.0])
        leg = Line(handle, point, None, name="leg", shared=True)
        lineset = LineSet([leg], v_inf=[10.0, 0.0, 1.0])
        lineset._set_line_indices()
        lineset.recalc(calculate_sag=False)
        direction = normalize(leg.diff_vector) * SYMMETRY_PLANE_MASK
        # the leg must give the in-plane force its full magnitude along z
        self.assertAlmostEqual(leg.force * direction[2], 10.0, places=6)

    def test_sag_computation_runs(self):
        lineset, leg, knot = build_lineset(shared=True)
        solve(lineset)
        lineset.recalc(calculate_sag=True)
        self.assertIsNotNone(leg.sag_par_1)
        self.assertAlmostEqual(knot.vec[1], 0.0, places=9)
        self.assertGreater(leg.length_with_sag, 4.9)

    def test_leg_too_short_warns_and_falls_back(self):
        lineset, leg, knot = build_lineset(
            shared=True, handle_y=HANDLE_Y, leg_length=0.2
        )
        with self.assertLogs("openglider.lines.lineset", level="WARNING") as logs:
            lineset.recalc(calculate_sag=False)
        self.assertTrue(any("too short" in msg for msg in logs.output))
        self.assertAlmostEqual(norm(knot.vec - leg.lower_node.vec), 0.2, places=6)
        self.assertGreater(knot.vec[1], 0.0)

    def test_has_geo_with_zero_coordinate(self):
        lineset, leg, knot = build_lineset(shared=True)
        self.assertFalse(leg.has_geo)  # knot not yet placed
        solve(lineset)
        self.assertEqual(knot.vec[1], 0.0)
        self.assertTrue(leg.has_geo)


class SharedLineJsonTest(unittest.TestCase):
    def test_lineset_3d_roundtrip(self):
        lineset, leg, knot = build_lineset(shared=True)
        solve(lineset)
        dumped = jsonify.dumps(lineset)
        loaded = jsonify.loads(dumped)["data"]
        self.assertEqual([line.shared for line in loaded.lines], [True, False, False])
        # files written before the flag existed load as plain lines
        legacy = re.sub(r',\s*"shared":\s*(true|false)', "", dumped)
        self.assertNotIn("shared", legacy)
        loaded_legacy = jsonify.loads(legacy)["data"]
        self.assertFalse(any(line.shared for line in loaded_legacy.lines))

    def test_line2d_roundtrip(self):
        handle = LowerNode2D([0.3, -7.0], [1.0, HANDLE_Y, -7.0], name="brake")
        knot = BatchNode2D([0.0, -2.0], name="knot")
        line = Line2D(handle, knot, target_length=5.0, name="leg", shared=True)
        loaded = jsonify.loads(jsonify.dumps(line))["data"]
        self.assertTrue(loaded.shared)
        self.assertEqual(loaded.name, "leg")
        plain = jsonify.loads(jsonify.dumps(Line2D(handle, knot)))["data"]
        self.assertFalse(plain.shared)


class SharedLineParametricTest(TestCase):
    """Flag a lowest line of the demo kite and build the 3d glider."""

    def pick_line(self, glider2d):
        lineset = glider2d.lineset
        for node in lineset.get_lower_attachment_points():
            for line in lineset.get_upper_connected_lines(node):
                if not isinstance(line.upper_node, UpperNode2D) and (
                    line.target_length or 0
                ) > abs(node.pos_3D[1]) + 0.1:
                    return line
        self.skipTest("no lowest line long enough in the demo kite")

    def test_shared_line_in_glider_3d(self):
        glider2d = self.import_glider_2d()
        line2d = self.pick_line(glider2d)
        line2d.shared = True
        line2d.name = "shared_test"
        handle_y = line2d.lower_node.pos_3D[1]

        glider = glider2d.get_glider_3d()
        leg = glider.lineset["shared_test"]
        self.assertTrue(leg.shared)
        self.assertAlmostEqual(leg.upper_node.vec[1], 0.0, places=9)
        self.assertAlmostEqual(leg.lower_node.vec[1], handle_y, places=9)
        self.assertGreater(leg.force, 0.0)
        # lines above the knot are computed as usual
        for upper in glider.lineset.get_upper_connected_lines(leg.upper_node):
            self.assertGreater(upper.force, 0.0)

        # both legs show up in the mirrored glider, meeting on the plane
        complete = glider.copy_complete()
        legs = [line for line in complete.lineset.lines if line.name == "shared_test"]
        self.assertEqual(len(legs), 2)
        for line in legs:
            self.assertTrue(line.shared)
            self.assertAlmostEqual(line.upper_node.vec[1], 0.0, places=9)
        self.assertAlmostEqual(
            sorted(line.lower_node.vec[1] for line in legs)[0], -handle_y, places=9
        )
        self.assertAlmostEqual(legs[0].force, legs[1].force, places=6)
        self.assertLess(norm(legs[0].upper_node.vec - legs[1].upper_node.vec), 1e-6)

        # the flag survives the parametric json roundtrip
        reloaded = jsonify.loads(jsonify.dumps(glider2d))["data"]
        shared_lines = [line for line in reloaded.lineset.lines if line.shared]
        self.assertEqual([line.name for line in shared_lines], ["shared_test"])

        # and shows up in the production table
        table = glider.lineset.get_table_grouped_by_riser()
        cells = [str(v) for v in table.dct.values()] if hasattr(table, "dct") else []
        if cells:
            self.assertTrue(any("partagée" in c for c in cells))


if __name__ == "__main__":
    unittest.main(verbosity=2)
