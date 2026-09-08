import os
import tempfile
import unittest

from common import *

from openglider import jsonify
from openglider.glider import ParametricGlider

TEMPDIR =  tempfile.gettempdir()

class GliderTestCase2D(TestCase):
    def setUp(self):
        self.glider2d = self.import_glider_2d()

    def test_fit(self):
        glider_3d = self.import_glider()
        self.assertEqualGlider2D(ParametricGlider.fit_glider_3d(glider_3d), self.glider2d)

    def test_create_glider(self):
        glider = self.glider2d.get_glider_3d()
        self.assertAlmostEqual(glider.span, 2*self.glider2d.shape.span, 2)

    def test_export(self):
        exp = jsonify.dumps(self.glider2d)
        imp = jsonify.loads(exp)['data']
        self.assertEqualGlider2D(self.glider2d, imp)

    def test_export_ods(self):
        exp = self.glider2d.export_ods(os.path.join(TEMPDIR, "test.ods"))

    def test_set_area(self):
        self.glider2d.shape.set_area(10)
        self.assertAlmostEqual(self.glider2d.shape.area, 10)

    def test_distribute_forces(self):
        lineset = self.glider2d.lineset
        result = lineset.distribute_forces(self.glider2d)

        # every upper attachment point gets a strictly positive force
        self.assertEqual(set(result), set(lineset.get_upper_nodes()))
        self.assertTrue(all(f > 0 for f in result.values()))
        self.assertAlmostEqual(max(result.values()), 1.0)  # normalized to peak

        # chord-wise: on a rib, the front point carries more than the rear one
        by_rib = {}
        for node, force in result.items():
            by_rib.setdefault(node.cell_no, []).append((node.rib_pos, force))
        for points in by_rib.values():
            points.sort()  # by rib_pos (front -> rear)
            # skip wing-tip ribs whose points are all clamped to the floor
            if len(points) > 1 and points[0][1] > 0.05:
                self.assertGreater(points[0][1], points[-1][1])

        # forces propagate cleanly down the 3D lineset (no None / degenerate)
        glider = self.glider2d.get_glider_3d()
        glider.lineset.recalc(calculate_sag=False)
        forces = [l.force for l in glider.lineset.lines]
        self.assertTrue(all(f is not None for f in forces))

    def test_distribute_forces_nearest_point_tributary(self):
        """A brake point alone on its rib must not carry the whole rib's lift:
        the A/B/C share of that rib goes to the neighbouring ribs' points."""
        from openglider.glider.parametric.lines import Line2D, UpperNode2D

        lineset = self.glider2d.lineset
        # the demo glider has no attachment point on rib 5 -> add a lone
        # trailing-edge (brake) point there
        brake = UpperNode2D(5, 1.0, force=1.0, name="brake5", layer="brake")
        knot = lineset.lines[0].lower_node
        lineset.lines.append(Line2D(knot, brake, name="brake5", layer="brake"))

        result = lineset.distribute_forces(self.glider2d)
        by_name = {node.name: force for node, force in result.items()}
        self.assertIn("brake5", by_name)
        # far from the peak (the old per-rib grouping gave it ~1.0)
        self.assertLess(by_name["brake5"], 0.2)
        # the rear point of the neighbouring rib still carries more than the
        # trailing-edge point, and the front point of that rib much more
        self.assertLess(by_name["brake5"], by_name["D5"])
        self.assertLess(by_name["brake5"], by_name["A5"] / 3)

    def test_distribute_forces_total_load(self):
        """normalize="load": forces in newtons, the half wing carries half of
        the all-up weight (vertical components)."""
        lineset = self.glider2d.lineset
        result = lineset.distribute_forces(
            self.glider2d, normalize="load", total_load=100.0
        )
        total = sum(result.values())
        half_weight = 100.0 * 9.81 / 2
        # rib inclination (arc) only increases the sum, the floor adds a bit
        self.assertGreaterEqual(total, half_weight - 1)
        self.assertLess(total, half_weight * 1.5)
        self.assertGreater(max(result.values()), 10.0)
        with self.assertRaises(ValueError):
            lineset.distribute_forces(self.glider2d, normalize="load")

    def test_force_distribution_config_roundtrip(self):
        self.glider2d.force_distribution_config = {
            "spanwise": "chord", "normalize": "load", "total_load": 95.0,
        }
        exp = jsonify.dumps(self.glider2d)
        imp = jsonify.loads(exp)["data"]
        self.assertEqual(imp.force_distribution_config["total_load"], 95.0)
        self.assertEqual(imp.force_distribution_config["spanwise"], "chord")

if __name__ == '__main__':
    unittest.main(verbosity=2)