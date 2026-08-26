import unittest

import numpy as np

from common import TestCase
from openglider.glider.twist import ThinAirfoilPolar, TwistModel
from openglider.glider.twist.model import shift_curve


class TestThinAirfoilPolar(unittest.TestCase):
    def test_parabolic_camber(self):
        """Parabolic camber h: alpha_0 = -2h, Cm_c4 = -pi h (thin-airfoil theory)."""
        h = 0.04
        camber = [(x, 4 * h * x * (1 - x)) for x in np.linspace(0, 1, 200)]
        alpha_0, cm = ThinAirfoilPolar.from_camber(camber)
        self.assertAlmostEqual(alpha_0, -2 * h, places=3)
        self.assertAlmostEqual(cm, -np.pi * h, places=3)

    def test_flat_plate(self):
        polar = ThinAirfoilPolar(camber_line=[(0, 0), (0.5, 0), (1, 0)])
        cl, cd, cm = polar(np.radians(5))
        self.assertAlmostEqual(cl, 2 * np.pi * np.radians(5), places=6)
        self.assertAlmostEqual(cm, 0.0)
        self.assertAlmostEqual(polar.centre_of_pressure(np.radians(5)), 0.25)
        self.assertGreater(cd, 0)


class TestTwistModel(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parametric = cls.import_glider_2d()

    def setUp(self):
        self.model = TwistModel(self.parametric)

    # ------------------------------------------------------------------ #
    # geometry                                                           #
    # ------------------------------------------------------------------ #
    def test_frames_match_3d_ribs(self):
        """Rib origin / chord axis rebuilt from the splines == the 3d glider's."""
        glider = self.parametric.get_glider_3d()
        ribs = glider.ribs[glider.has_center_cell:]
        for station in self.model.stations:
            rib = ribs[station.index]
            o, e_c, e_t, e_n, chord = self.model.frame(station)
            self.assertLess(np.linalg.norm(o - rib.pos), 2e-3)
            e_c_rib = (np.array(rib.align([1, 0])) - rib.pos) / rib.chord
            self.assertLess(np.linalg.norm(e_c - e_c_rib), 1e-6)
            self.assertAlmostEqual(chord, rib.chord, places=6)

    def test_alpha_equals_relative_aoa(self):
        """In-plane wind angle reproduces Rib.aoa_relative (arc projection)."""
        glider = self.parametric.get_glider_3d()
        ribs = glider.ribs[glider.has_center_cell:]
        for station, state in zip(self.model.stations, self.model.diagnose()):
            self.assertAlmostEqual(state.alpha, ribs[station.index].aoa_relative, places=6)

    def test_lines_read(self):
        stations = self.model.stations
        self.assertTrue(all(s.apex is not None for s in stations))
        suspended = [s for s in stations if s.suspended]
        self.assertGreater(len(suspended), 0)
        for s in suspended:
            self.assertGreaterEqual(len(s.line_attachments), 1)
        borrowed = [s for s in stations if s.apex_from is not None]
        for s in borrowed:
            self.assertTrue(stations[s.apex_from].suspended)

    # ------------------------------------------------------------------ #
    # moment arm                                                         #
    # ------------------------------------------------------------------ #
    def test_arm_sign_convention(self):
        """Moving the apex backwards puts the resultant ahead of it: arm grows."""
        station = self.model.stations[1]
        base = self.model.state(station).moment_arm
        station.apex = station.apex + [0.10, 0.0, 0.0]
        moved = self.model.state(station).moment_arm
        self.assertGreater(moved, base)
        self.assertAlmostEqual(moved - base, 0.10, delta=0.02)

    def test_sweep_zeroes_arm(self):
        for station in self.model.stations:
            dx = self.model.sweep_for_arm(station, station.aoa_rel)
            arm = self.model.state(station, dx=dx).moment_arm
            self.assertAlmostEqual(arm, 0.0, places=6)

    def test_sweep_direction(self):
        """Resultant behind the apex (arm < 0) -> rib must move forward (dx < 0)."""
        station = self.model.stations[0]
        arm = self.model.state(station).moment_arm
        dx = self.model.sweep_for_arm(station, station.aoa_rel)
        self.assertEqual(np.sign(dx), np.sign(arm))

    def test_twist_moves_arm(self):
        station = self.model.stations[0]
        a0 = self.model.state(station).moment_arm
        a1 = self.model.state(station, station.aoa_rel + np.radians(5)).moment_arm
        self.assertNotAlmostEqual(a0, a1, places=4)

    # ------------------------------------------------------------------ #
    # solve / propose                                                    #
    # ------------------------------------------------------------------ #
    def test_solve_mixes_reach_target(self):
        for mix in (0.0, 0.5, 1.0):
            sol = self.model.solve(mix)
            for after, reachable in zip(sol.after, sol.reachable):
                self.assertAlmostEqual(after.moment_arm, sol.target, places=5)
            if mix == 0.0:
                self.assertTrue(np.all(sol.d_aoa == 0))
            if mix == 1.0:
                # sweep only where twist alone could not reach the target
                self.assertTrue(np.all(sol.dx[sol.reachable] == 0))

    def test_center_target_keeps_center_rib(self):
        sol = self.model.solve(1.0, target="center")
        self.assertAlmostEqual(sol.d_aoa[0], 0.0, places=9)
        self.assertAlmostEqual(sol.dx[0], 0.0, places=9)

    def test_zero_target(self):
        sol = self.model.solve(0.0, target="zero")
        self.assertEqual(sol.target, 0.0)
        for after in sol.after:
            self.assertAlmostEqual(after.moment_arm, 0.0, places=6)

    def test_propose_smooth_and_apply(self):
        proposal = self.model.propose(0.5)
        self.assertIsNotNone(proposal.aoa_curve)
        self.assertIsNotNone(proposal.front_curve)
        self.assertEqual(len(proposal.aoa_curve.controlpoints),
                         len(self.parametric.aoa.controlpoints))
        self.assertEqual(len(proposal.front_curve.controlpoints),
                         len(self.parametric.shape.front_curve.controlpoints))
        # smoothing keeps the residual small relative to the raw deviation
        raw = max(abs(b.moment_arm - proposal.solution.target) for b in proposal.solution.before)
        residual = max(abs(a.moment_arm - proposal.solution.target) for a in proposal.after)
        self.assertLess(residual, 0.2 * raw)

        target = self.parametric.copy()
        span, area = target.shape.span, target.shape.area
        chords = [abs(r[0][1] - r[1][1]) for r in target.shape.ribs]
        proposal.apply_to(target)
        self.assertAlmostEqual(target.shape.span, span, places=6)
        # chord-fixed translation: chords and area preserved
        for c0, rib in zip(chords, target.shape.ribs):
            self.assertAlmostEqual(abs(rib[0][1] - rib[1][1]), c0, places=3)
        self.assertAlmostEqual(target.shape.area, area, places=2)
        # the glider still builds
        target.get_glider_3d()
        # and the new model agrees with the proposal's own re-evaluation
        new = TwistModel(target)
        for a, b in zip(new.diagnose(), proposal.after):
            self.assertAlmostEqual(a.moment_arm, b.moment_arm, places=3)

    def test_shift_curve_identity(self):
        curve = self.parametric.shape.front_curve
        same = shift_curve(curve, lambda x: 0.0)
        for a, b in zip(curve.get_sequence(num=30), same.get_sequence(num=30)):
            self.assertLess(np.linalg.norm(np.array(a) - np.array(b)), 1e-6)
        moved = shift_curve(curve, lambda x: 0.05)
        for a, b in zip(curve.get_sequence(num=30), moved.get_sequence(num=30)):
            self.assertAlmostEqual(b[1] - a[1], 0.05, places=6)
            self.assertAlmostEqual(b[0], a[0], places=6)

    def test_ghost_planform(self):
        proposal = self.model.propose(0.0)
        ribs = proposal.ribs_2d()
        self.assertEqual(len(ribs), len(self.model.stations))
        for (front, back), s in zip(ribs, self.model.stations):
            self.assertAlmostEqual(back[1] - front[1], s.chord, places=9)

    def test_set_aoa(self):
        curve = self.parametric.aoa.copy()
        curve.controlpoints = [[x, y + np.radians(2)] for x, y in curve.controlpoints]
        before = [s.aoa_rel for s in self.model.stations]
        self.model.set_aoa(curve)
        for b, s in zip(before, self.model.stations):
            self.assertAlmostEqual(s.aoa_rel - b, np.radians(2), places=3)

    def test_table(self):
        rows = self.model.table()
        self.assertEqual(len(rows), len(self.model.stations))
        self.assertIn("arm_mm", rows[0])


if __name__ == "__main__":
    unittest.main()
