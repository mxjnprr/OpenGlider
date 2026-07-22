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

if __name__ == '__main__':
    unittest.main(verbosity=2)