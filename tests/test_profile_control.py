"""Profile control on the parametric glider: rib overrides, procedural shark
nose / thickness and the profile distribution (morphing)."""
import io
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import openglider
from openglider import jsonify
from openglider.airfoil import Profile2D
from openglider.vector.spline import SymmetricBSpline

DEMOKITE = os.path.join(ROOT, "tests", "common", "demokite.json")


def nose_shelf_level(profile, start=0.03, end=0.08):
    """Mean intrados y between ``start`` and ``end`` (raised by a shark nose)."""
    p = Profile2D(profile.data)
    xs = p.data[p.noseindex :, 0]
    ys = p.data[p.noseindex :, 1]
    mask = (xs >= start) & (xs <= end)
    return float(np.mean(ys[mask]))


def resampled(profile, x_values):
    p = Profile2D(profile.data)
    p.x_values = x_values
    return p


class TestProfileOverrides(unittest.TestCase):
    def setUp(self):
        self.pg = openglider.load(DEMOKITE)
        plain = Profile2D(self.pg.profiles[0].data)
        plain.name = "plain"
        self.pg.sharknose_enabled = True
        self.pg.sharknose_start, self.pg.sharknose_end, self.pg.sharknose_angle = 0.03, 0.08, 5.0
        shark = Profile2D(self.pg._apply_sharknose(plain.copy(), 1.0).data)
        shark.name = "shark"
        self.pg.sharknose_enabled = False
        self.pg.profiles = [shark, plain]
        self.shark, self.plain = shark, plain
        # parametric rib i is 3d rib i + offset (mirrored centre rib)
        self.offset = 1 if self.pg.shape.has_center_cell else 0
        self.n = len(self.pg.shape.rib_x_values)
        self.tip = [self.n - 3, self.n - 2]  # the very last rib is collapsed

    def rib_profile(self, glider, rib_index):
        return glider.ribs[rib_index + self.offset].profile_2d

    def assert_is_plain(self, glider, rib_index):
        expected = resampled(self.plain, self.pg.profiles[0].x_values)
        got = self.rib_profile(glider, rib_index)
        self.assertTrue(np.allclose(got.data, expected.data, atol=1e-9), f"rib {rib_index}")

    def test_override_replaces_distribution_result(self):
        self.pg.profile_overrides = {str(i): 1 for i in self.tip}
        self.pg.profile_overrides_enabled = True
        glider = self.pg.get_glider_3d()
        for i in self.tip:
            self.assert_is_plain(glider, i)
        shark_level = nose_shelf_level(self.shark)
        for i in range(self.n - 3):
            self.assertAlmostEqual(nose_shelf_level(self.rib_profile(glider, i)), shark_level, places=4)

    def test_overrides_ignored_when_disabled(self):
        self.pg.profile_overrides = {str(i): 1 for i in self.tip}
        self.pg.profile_overrides_enabled = False
        glider = self.pg.get_glider_3d()
        for i in self.tip:
            self.assertAlmostEqual(
                nose_shelf_level(self.rib_profile(glider, i)), nose_shelf_level(self.shark), places=4
            )

    def test_override_is_used_as_is_with_sharknose_and_thickness(self):
        """An overridden rib gets neither the procedural shark nose nor the
        thickness scaling (a plain profile on closed tip cells stays plain)."""
        self.pg.profile_overrides = {str(i): 1 for i in self.tip}
        self.pg.profile_overrides_enabled = True
        self.pg.sharknose_enabled = True
        self.pg.sharknose_cells = None  # all cells
        span = self.pg.shape.span
        self.pg.thickness_curve = SymmetricBSpline([[0.0, 0.8], [span, 0.8]])
        self.pg.thickness_curve_enabled = True
        glider = self.pg.get_glider_3d()
        for i in self.tip:
            self.assert_is_plain(glider, i)
        # the other ribs are still processed
        inner = self.rib_profile(glider, 0)
        self.assertLess(inner.thickness, self.plain.thickness * 0.9)
        self.assertGreater(nose_shelf_level(inner), nose_shelf_level(self.plain) + 0.005)

    def test_override_out_of_range_falls_back_to_distribution(self):
        self.pg.profile_overrides = {str(self.tip[0]): 7}
        self.pg.profile_overrides_enabled = True
        glider = self.pg.get_glider_3d()
        self.assertAlmostEqual(
            nose_shelf_level(self.rib_profile(glider, self.tip[0])), nose_shelf_level(self.shark), places=4
        )

    def test_get_profile_override_accepts_int_keys_and_returns_a_copy(self):
        self.pg.profile_overrides = {self.tip[0]: 1}
        self.pg.profile_overrides_enabled = True
        override = self.pg.get_profile_override(self.tip[0])
        self.assertIsNotNone(override)
        self.assertIsNot(override, self.plain)
        self.assertIsNone(self.pg.get_profile_override(0))
        self.assertIsNone(self.pg.get_profile_override(None))

    def test_overrides_survive_json_roundtrip(self):
        self.pg.profile_overrides = {str(i): 1 for i in self.tip}
        self.pg.profile_overrides_enabled = True
        buf = io.StringIO()
        jsonify.dump(self.pg, buf)
        buf.seek(0)
        loaded = jsonify.load(buf)["data"]
        self.assertEqual(loaded.profile_overrides, {str(i): 1 for i in self.tip})
        self.assertTrue(loaded.profile_overrides_enabled)
        glider = loaded.get_glider_3d()
        self.plain = Profile2D(loaded.profiles[1].data)
        self.pg = loaded
        for i in self.tip:
            self.assert_is_plain(glider, i)


class TestProceduralSharkNoseSwitch(unittest.TestCase):
    """The procedural shark nose must not touch any rib unless it is enabled."""

    def setUp(self):
        self.pg = openglider.load(DEMOKITE)
        plain = Profile2D(self.pg.profiles[0].data)
        plain.name = "plain"
        self.pg.profiles = [plain]
        self.plain = plain
        self.offset = 1 if self.pg.shape.has_center_cell else 0
        self.n = len(self.pg.shape.rib_x_values)

    def assert_all_ribs_plain(self, glider):
        expected = resampled(self.plain, self.pg.profiles[0].x_values)
        for i in range(self.n - 1):  # the last rib is collapsed to a line
            got = glider.ribs[i + self.offset].profile_2d
            self.assertTrue(np.allclose(got.data, expected.data, atol=1e-12), f"rib {i}")

    def test_disabled_by_default(self):
        self.assertFalse(self.pg.sharknose_enabled)
        self.assert_all_ribs_plain(self.pg.get_glider_3d())

    def test_disabled_even_with_cells_selected_and_parameters_set(self):
        self.pg.sharknose_enabled = False
        self.pg.sharknose_cells = list(range(self.n - 1))
        self.pg.sharknose_start, self.pg.sharknose_end, self.pg.sharknose_angle = 0.02, 0.10, 10.0
        self.pg.sharknose_curve = SymmetricBSpline([[0.0, 1.0], [self.pg.shape.span, 1.0]])
        self.assertEqual(self.pg._get_sharknose_factor(0.5), 0.0)
        self.assert_all_ribs_plain(self.pg.get_glider_3d())
        for i in range(self.n):
            self.assertIs(self.pg.get_profile_override(i), None)

    def test_enable_then_disable_restores_plain_profiles(self):
        self.pg.sharknose_enabled = True
        self.pg.sharknose_cells = None
        deformed = self.pg.get_glider_3d()
        level = nose_shelf_level(deformed.ribs[self.offset].profile_2d)
        self.assertGreater(level, nose_shelf_level(self.plain) + 0.005)
        self.pg.sharknose_enabled = False
        self.assert_all_ribs_plain(self.pg.get_glider_3d())
        # the stored profile itself was never modified
        self.assertTrue(np.allclose(self.pg.profiles[0].data, self.plain.data))

    def test_disabled_state_survives_json_roundtrip(self):
        self.pg.sharknose_enabled = False
        self.pg.sharknose_cells = list(range(self.n - 1))
        buf = io.StringIO()
        jsonify.dump(self.pg, buf)
        buf.seek(0)
        loaded = jsonify.load(buf)["data"]
        self.assertFalse(loaded.sharknose_enabled)
        self.pg = loaded
        self.assert_all_ribs_plain(loaded.get_glider_3d())


class TestProfileDistribution(unittest.TestCase):
    def test_morph_between_profiles_with_different_point_counts(self):
        pg = openglider.load(DEMOKITE)
        thin = Profile2D(pg.profiles[0].data)
        thin.name = "thin"
        thick = thin.copy()
        thick.numpoints = 61  # different sampling than profile 0
        thick.data = np.array([[x, 1.5 * y] for x, y in thick.data])
        thick.name = "thick"
        pg.profiles = [thin, thick]
        span = pg.profile_merge_curve.controlpoints[-1][0]
        pg.profile_merge_curve.controlpoints = [[0.0, 0.0], [0.5 * span, 0.5], [span, 1.0]]

        factors = pg.get_profile_merge()
        self.assertTrue(all(b >= a for a, b in zip(factors, factors[1:])))
        self.assertAlmostEqual(factors[-1], 1.0, places=3)

        glider = pg.get_glider_3d()
        offset = 1 if pg.shape.has_center_cell else 0
        thick_values = [glider.ribs[i + offset].profile_2d.thickness for i in range(len(factors) - 1)]
        self.assertTrue(all(b >= a for a, b in zip(thick_values, thick_values[1:])))
        self.assertGreater(thick_values[0], thin.thickness * 0.99)
        self.assertLess(thick_values[-1], thick.thickness * 1.01)
        self.assertGreater(thick_values[-1], thick.thickness * 0.95)


if __name__ == "__main__":
    unittest.main()
