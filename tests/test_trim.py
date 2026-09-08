import unittest

import numpy as np

from common import TestCase
from openglider.glider.trim import TrimModel


class TestTrim(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parametric = cls.import_glider_2d()

    def setUp(self):
        self.model = TrimModel(self.parametric)

    def _deltas(self, predicate, value=0.03):
        """Build line.number-keyed deltas for every line feeding a matching AP."""
        deltas = {}
        for ap in self.model.attachment_points:
            if predicate(ap):
                for line in self.model.path_to_ground(ap):
                    deltas[line.number] = value
        return deltas

    # ------------------------------------------------------------------ #
    # solve / fit                                                        #
    # ------------------------------------------------------------------ #
    def test_identity(self):
        """Zero length change must leave the geometry untouched."""
        res = self.model.solve({})
        for r in res.ribs.values():
            self.assertAlmostEqual(r.d_aoa, 0.0, places=12)
            self.assertLess(np.linalg.norm(r.d_pos), 1e-12)
            self.assertLess(r.max_residual, 1e-12)

    def test_front_back_opposite_aoa(self):
        """Lengthening front (A) vs back (D) lines pitches the ribs oppositely."""
        front = self.model.solve(self._deltas(lambda ap: getattr(ap, "rib_pos", 1) < 0.15))
        back = self.model.solve(self._deltas(lambda ap: getattr(ap, "rib_pos", 0) > 0.6))
        common = set(front.ribs) & set(back.ribs)
        self.assertTrue(common)
        for i in common:
            fa, ba = front.ribs[i].d_aoa, back.ribs[i].d_aoa
            if abs(fa) > np.radians(1) and abs(ba) > np.radians(1):
                self.assertLess(fa * ba, 0.0, msg=f"rib {i}: same sign {fa} / {ba}")

    def test_layers_inferred(self):
        layers = self.model.infer_layers()
        self.assertTrue(layers)
        self.assertGreaterEqual(len(set(layers.values())), 2)

    # ------------------------------------------------------------------ #
    # to_curves                                                          #
    # ------------------------------------------------------------------ #
    def test_curves_identity(self):
        """Zero trim -> proposed curves reproduce the baseline, no overrides."""
        prop = self.model.to_curves(self.model.solve({}))
        np.testing.assert_allclose(prop.new_aoa, prop.base_aoa, atol=1e-9)
        self.assertEqual(prop.profiles, {})

    def test_curves_roundtrip(self):
        """Proposed arch + AoA curves evaluate back close to their targets."""
        prop = self.model.to_curves(
            self.model.solve(self._deltas(lambda ap: getattr(ap, "rib_pos", 1) < 0.15)),
            arc_numpoints=5,
            aoa_numpoints=5,
        )
        aoa_int = prop.aoa_curve.interpolation(num=100)
        got = np.array([aoa_int(x) for x in prop.xv])
        self.assertLess(np.max(np.abs(got - prop.new_aoa)), np.radians(2))

        arc = np.array([np.asarray(p) for p in prop.arc_curve.get_arc_positions(prop.xv)])
        self.assertTrue(np.all(np.isfinite(arc)))
        self.assertLess(np.max(np.abs(arc)), 100.0)  # no degenerate blow-up

    def test_bake_and_apply_rebuilds(self):
        """Baked profiles + curves apply cleanly and the glider rebuilds."""
        prop = self.model.to_curves(
            self.model.solve(self._deltas(lambda ap: getattr(ap, "rib_pos", 1) < 0.15))
        )
        self.assertTrue(prop.profiles)  # deformation was baked

        target = self.import_glider_2d()
        n_before = len(target.profiles)
        prop.apply_to(target)
        self.assertGreater(len(target.profiles), n_before)
        self.assertTrue(target.profile_overrides_enabled)

        rebuilt = target.get_glider_3d()  # must not raise
        self.assertEqual(len(rebuilt.ribs), len(self.model.glider.ribs))

    # ------------------------------------------------------------------ #
    # iterative solve                                                    #
    # ------------------------------------------------------------------ #
    def test_iterative_stable(self):
        """The guarded iteration never returns worse than the single pass."""
        deltas = self._deltas(lambda ap: getattr(ap, "rib_pos", 1) < 0.15)
        prop, info = self.model.solve_iterative(deltas, iterations=6)
        self.assertIsNotNone(prop)
        self.assertLessEqual(info["best_gap"], info["gaps"][0] + 1e-9)


if __name__ == "__main__":
    unittest.main()
