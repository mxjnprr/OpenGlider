"""Twist (vrillage) model: per-rib 2d "kite" equilibrium.

Each rib is treated as a 2d kite in its own plane.  Its aerodynamic resultant
(direction from the section polar, applied at the centre of pressure) is
compared with the *apex* of its line cone -- the riser the rib's attachment
points feed.  A free rib is moment-free only when the resultant line passes
through the apex; the signed distance between the two is the **moment arm**,
the tool's error curve (positive = resultant ahead of the apex = nose-up
tendency, front lines overloaded relative to the rear ones).

Two levers zero that arm:

* **twist** (``d_aoa``) -- moves the centre of pressure through ``x_cp(alpha)``
  and tilts the resultant through ``Cd/Cl``;
* **sweep** (``dx``) -- translates the rib along the flight axis, chord kept.

The ``mix`` parameter (0 = all sweep, 1 = all twist) blends them: the twist
share is applied first, the sweep closes whatever arm is left.

Geometry comes straight from the parametric glider (shape, arc, aoa splines,
2d line plan), no 3d glider build: a full solve over the half wing takes a few
milliseconds, fast enough for a live slider.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from openglider.glider.parametric.lines import LowerNode2D, UpperNode2D
from openglider.glider.rib.rib import Rib, rib_transformation
from openglider.vector.spline import SymmetricBSpline

from .polar import Polar, ThinAirfoilPolar

X_AXIS = np.array([1.0, 0.0, 0.0])
BRAKE_POS = 0.95  # attachment points at/after this chord fraction are brakes


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def fit_symmetric(points, numpoints):
    """Fit a :class:`SymmetricBSpline` to half-wing ``[(x, value), ...]``."""
    pts = [np.asarray(p, dtype=float) for p in points]
    mirrored = [p * [-1.0, 1.0] for p in pts[::-1]] + pts
    return SymmetricBSpline.fit(mirrored, numpoints=numpoints)


def shift_curve(curve, delta, num=200):
    """Copy of a symmetric shape spline with ``y += delta(|x|)``.

    The curve is resampled at uniform parameter (what ``fit`` assumes) and
    refitted with its own number of control points, so ``delta == 0`` returns
    the same curve and front/back curves shifted by the same ``delta`` keep
    their chord.
    """
    pts = np.asarray(curve.get_sequence(num=num), dtype=float)
    pts = [np.array([x, y + delta(abs(x))]) for x, y in pts]
    return SymmetricBSpline.fit(pts, numpoints=len(curve.controlpoints))


# --------------------------------------------------------------------------- #
# data                                                                        #
# --------------------------------------------------------------------------- #
@dataclass
class Station:
    """One half-wing rib station, as read from the parametric glider."""

    index: int
    x: float                     # span station [m]
    chord: float                 # [m]
    front_y: float               # shape-2d y of the leading edge
    arc_pos: np.ndarray          # (y, z) of the leading edge in the front view
    arcang: float                # [rad]
    zrot: float                  # z-rotation spline value at this station
    aoa_rel: float               # [rad] designed aoa (relative to the wind)
    profile: object              # Profile2D
    polar: Polar
    attachments: List[tuple] = field(default_factory=list)  # (name, rib_pos)
    apex: Optional[np.ndarray] = None       # riser position (3d) [m]
    apex_from: Optional[int] = None         # station the apex was borrowed from

    @property
    def suspended(self):
        return self.apex_from is None and self.apex is not None

    @property
    def line_attachments(self):
        return [a for a in self.attachments if a[1] < BRAKE_POS]


@dataclass
class StationState:
    """Kite equilibrium of a station for a given aoa / sweep."""

    index: int
    aoa_rel: float               # [rad] aoa used
    dx: float                    # [m] sweep used (+ = towards trailing edge)
    alpha: float                 # [rad] chord vs in-plane wind (== aoa_rel)
    cl: float
    cd: float
    cm: float
    x_cp: float                  # chord fraction of the centre of pressure
    moment_arm: float            # [m] signed, + = resultant ahead of the apex
    spanwise_deg: float          # apex direction vs rib plane, + = leans inboard
    cp_split: Optional[float]    # x_cp within [front .. rear attachment], 0..1
    lift: float                  # Cl * chord, relative section load
    frame: tuple                 # (origin, e_chord, e_thick, e_normal) 3d
    resultant: np.ndarray        # unit resultant direction (3d)
    cp: np.ndarray               # centre of pressure (3d)

    @property
    def arm_chord(self):
        return self.moment_arm / self.frame[4] if self.frame[4] else 0.0


@dataclass
class TwistSolution:
    mix: float
    target: float                        # [m] arm every station was driven to
    stations: List[Station]
    before: List[StationState]
    after: List[StationState]            # per-station exact solve (unsmoothed)
    d_aoa: np.ndarray                    # [rad] per station
    dx: np.ndarray                       # [m] per station, + = towards TE
    reachable: np.ndarray                # twist root found inside polar window

    @property
    def x(self):
        return np.array([s.x for s in self.stations])


@dataclass
class TwistProposal:
    """Smoothed curves ready to be written into a parametric glider."""

    solution: TwistSolution
    aoa_curve: Optional[object]          # SymmetricBSpline or None (mix == 0)
    front_curve: Optional[object]        # SymmetricBSpline or None (no sweep)
    back_curve: Optional[object]
    dx_curve: Optional[object]           # SymmetricBSpline of dx(x) [m]
    after: List[StationState]            # states with the *smoothed* curves
    aoa_values: np.ndarray               # [rad] per station after smoothing
    dx_values: np.ndarray                # [m] effective 3d shift per station
    dx_shape: np.ndarray                 # [m] shift written into the shape curves
    riser_dx: float = 0.0                # [m] common shift the shape cannot carry

    def ribs_2d(self):
        """Ghost planform: ``[[front, back], ...]`` in shape-2d coordinates."""
        return [
            [[s.x, s.front_y - dx], [s.x, s.front_y - dx + s.chord]]
            for s, dx in zip(self.solution.stations, self.dx_shape)
        ]

    @staticmethod
    def _assign(owner, attr, curve):
        """Write ``curve`` into ``owner.attr``, in place when the spline
        objects are compatible (open tools keep references to them)."""
        old = getattr(owner, attr, None)
        if (old is not None and type(old) is type(curve)
                and len(old.controlpoints) == len(curve.controlpoints)):
            old.controlpoints = np.array(curve.controlpoints).tolist()
        else:
            setattr(owner, attr, curve)

    def apply_to(self, parametric_glider):
        if self.aoa_curve is not None:
            self._assign(parametric_glider, "aoa", self.aoa_curve)
        if self.front_curve is not None and self.back_curve is not None:
            self._assign(parametric_glider.shape, "front_curve", self.front_curve)
            self._assign(parametric_glider.shape, "back_curve", self.back_curve)


# --------------------------------------------------------------------------- #
# model                                                                       #
# --------------------------------------------------------------------------- #
class TwistModel:
    """Per-rib kite model of a parametric glider.

    :param parametric_glider: the ``ParametricGlider`` (not modified)
    :param polar_factory: ``callable(profile) -> Polar``; defaults to the
        thin-airfoil estimate from the merged profile's camber line
    """

    num_interpolate = 50
    d_aoa_max = np.radians(20.0)     # search window for the twist root

    def __init__(self, parametric_glider, polar_factory=None):
        self.parametric = parametric_glider
        self.polar_factory = polar_factory or ThinAirfoilPolar.from_profile
        self.glide = float(parametric_glider.glide)
        gamma = np.arctan(1.0 / self.glide)
        self.wind = np.array([np.cos(gamma), 0.0, np.sin(gamma)])  # unit v_inf
        self.stations = self._read_stations()

    # ------------------------------------------------------------------ #
    # setup                                                              #
    # ------------------------------------------------------------------ #
    def _read_stations(self):
        pg = self.parametric
        x_values = list(pg.shape.rib_x_values)
        shape_ribs = list(pg.shape.ribs)
        arc_pos = [np.asarray(p, dtype=float) for p in pg.arc.get_arc_positions(x_values)]
        # get_glider_3d rescales the arc so the centre rib sits at z = 0
        arc_pos = [p - [0.0, arc_pos[0][1]] for p in arc_pos]
        arc_ang = pg.arc.get_rib_angles(x_values)
        aoa_int = pg.aoa.interpolation(num=self.num_interpolate)
        zrot_int = pg.zrot.interpolation(num=self.num_interpolate)
        merge_int = pg.profile_merge_curve.interpolation(num=self.num_interpolate)
        self.offset_x = shape_ribs[0][0][1]

        stations = []
        for i, x in enumerate(x_values):
            front, back = shape_ribs[i]
            chord = abs(front[1] - back[1])
            factor = merge_int(abs(x))
            try:
                profile = pg.get_merge_profile(factor, pos_x=x, rib_index=i)
            except TypeError:  # older signature
                profile = pg.get_merge_profile(factor)
            try:
                polar = self.polar_factory(profile)
            except Exception:
                polar = ThinAirfoilPolar()
            stations.append(
                Station(
                    index=i, x=float(x), chord=float(chord), front_y=float(front[1]),
                    arc_pos=arc_pos[i], arcang=float(arc_ang[i]), zrot=float(zrot_int(x)),
                    aoa_rel=float(aoa_int(x)), profile=profile, polar=polar,
                )
            )

        self._read_lines(stations)
        return stations

    def _riser_of(self, node):
        """Follow the 2d line plan down from ``node`` to its lower attachment."""
        lineset = self.parametric.lineset
        seen = set()
        while not isinstance(node, LowerNode2D):
            if id(node) in seen:
                return None
            seen.add(id(node))
            lowers = [ln for ln in lineset.lines if ln.upper_node is node]
            if not lowers:
                return None
            node = lowers[0].lower_node
        return node

    def _read_lines(self, stations):
        n = len(stations)
        risers: Dict[int, list] = {}
        for node in self.parametric.lineset.nodes:
            if not isinstance(node, UpperNode2D):
                continue
            idx = int(node.cell_no + round(node.cell_pos))
            if idx < 0 or idx >= n:
                continue
            stations[idx].attachments.append((node.name, float(node.rib_pos)))
            if node.rib_pos < BRAKE_POS:
                riser = self._riser_of(node)
                if riser is not None:
                    risers.setdefault(idx, []).append(np.asarray(riser.pos_3D, dtype=float))
        for idx, pts in risers.items():
            stations[idx].apex = np.mean(pts, axis=0)
        # ribs without lines borrow the cone of the nearest suspended rib
        suspended = [s for s in stations if s.apex is not None]
        for s in stations:
            if s.apex is None and suspended:
                donor = min(suspended, key=lambda d: abs(d.x - s.x))
                s.apex = donor.apex.copy()
                s.apex_from = donor.index
                if not s.line_attachments:
                    s.attachments = list(donor.attachments)

    def set_aoa(self, aoa_curve=None):
        """Re-read the designed aoa per station (after the spline was edited)."""
        curve = aoa_curve or self.parametric.aoa
        aoa_int = curve.interpolation(num=self.num_interpolate)
        for s in self.stations:
            s.aoa_rel = float(aoa_int(s.x))

    # ------------------------------------------------------------------ #
    # kinematics                                                         #
    # ------------------------------------------------------------------ #
    def aoa_absolute(self, station, aoa_rel):
        return aoa_rel - Rib._aoa_diff(station.arcang, self.glide)

    def frame(self, station, aoa_rel=None, dx=0.0):
        """Rib frame ``(origin, e_chord, e_thick, e_normal, chord)`` in 3d."""
        aoa_rel = station.aoa_rel if aoa_rel is None else aoa_rel
        pos = np.array([-station.front_y + self.offset_x + dx,
                        station.arc_pos[0], station.arc_pos[1]])
        zrot = np.arctan(station.arcang) / self.glide * station.zrot
        trafo = rib_transformation(
            self.aoa_absolute(station, aoa_rel), station.arcang, zrot, 0.0,
            station.chord, pos,
        )
        o = np.asarray(trafo([0.0, 0.0, 0.0]), dtype=float)
        e_c = _unit(np.asarray(trafo([1.0, 0.0, 0.0]), dtype=float) - o)
        e_t = _unit(np.asarray(trafo([0.0, 1.0, 0.0]), dtype=float) - o)
        e_n = np.cross(e_c, e_t)
        return o, e_c, e_t, e_n, station.chord

    def state(self, station, aoa_rel=None, dx=0.0):
        aoa_rel = station.aoa_rel if aoa_rel is None else aoa_rel
        o, e_c, e_t, e_n, chord = self.frame(station, aoa_rel, dx)

        # wind in the rib plane
        w2 = np.array([self.wind @ e_c, self.wind @ e_t])
        alpha = float(np.arctan2(w2[1], w2[0]))
        w2 = _unit(w2)
        n2 = np.array([-w2[1], w2[0]])          # +90 deg: towards the extrados

        cl, cd, cm = station.polar(alpha)
        x_cp = station.polar.centre_of_pressure(alpha)
        r2 = _unit(cl * n2 + cd * w2)
        cp2 = np.array([x_cp * chord, 0.0])
        resultant = r2[0] * e_c + r2[1] * e_t
        cp = o + cp2[0] * e_c

        if station.apex is not None:
            a = station.apex - o
            a2 = np.array([a @ e_c, a @ e_t])
            arm = float(_cross2(a2 - cp2, r2))
            d = station.apex - cp
            span_deg = float(np.degrees(np.arcsin(np.clip((d @ e_n) / np.linalg.norm(d), -1, 1))))
        else:
            arm, span_deg = 0.0, 0.0

        att = sorted(p for _, p in station.line_attachments)
        if len(att) >= 2 and att[-1] - att[0] > 1e-6:
            cp_split = float((x_cp - att[0]) / (att[-1] - att[0]))
        else:
            cp_split = None

        return StationState(
            index=station.index, aoa_rel=aoa_rel, dx=dx, alpha=alpha,
            cl=cl, cd=cd, cm=cm, x_cp=x_cp, moment_arm=arm, spanwise_deg=span_deg,
            cp_split=cp_split, lift=cl * chord, frame=(o, e_c, e_t, e_n, chord),
            resultant=resultant, cp=cp,
        )

    def diagnose(self, aoa_rel=None, dx=None):
        """States of every station (optionally with per-station overrides)."""
        out = []
        for i, s in enumerate(self.stations):
            a = None if aoa_rel is None else aoa_rel[i]
            d = 0.0 if dx is None else dx[i]
            out.append(self.state(s, a, d))
        return out

    # ------------------------------------------------------------------ #
    # solve                                                              #
    # ------------------------------------------------------------------ #
    def target_arm(self, target):
        """Arm every station is driven to.

        ``"zero"``: moment-free ribs.  ``"center"`` (default): the centre rib's
        current arm -- twist/sweep only remove the *spanwise variation*, the
        common offset is a pitch-trim matter (riser position, glide number).
        """
        if target == "zero":
            return 0.0
        if target == "center":
            return self.state(self.stations[0]).moment_arm
        return float(target)

    def sweep_for_arm(self, station, aoa_rel, dx0=0.0, target=0.0):
        """Sweep increment (from ``dx0``) driving the arm at ``aoa_rel`` to ``target``."""
        st = self.state(station, aoa_rel, dx0)
        _, e_c, e_t, _, _ = st.frame
        w2 = np.array([self.wind @ e_c, self.wind @ e_t])
        n2 = _unit(np.array([-w2[1], w2[0]]))
        r2 = _unit(st.cl * n2 + st.cd * _unit(w2))
        x2 = np.array([X_AXIS @ e_c, X_AXIS @ e_t])
        denom = _cross2(x2, r2)
        if abs(denom) < 1e-9:
            return 0.0
        # translating the rib by dx shifts (apex - cp) by -dx * x2
        return (st.moment_arm - target) / denom

    def twist_for_arm(self, station, dx=0.0, target=0.0):
        """Aoa (relative) driving the arm to ``target``; ``(aoa, reachable)``."""
        polar = station.polar
        lo = max(polar.alpha_min, station.aoa_rel - self.d_aoa_max)
        hi = min(polar.alpha_max, station.aoa_rel + self.d_aoa_max)
        if hi <= lo:
            return station.aoa_rel, False
        grid = np.linspace(lo, hi, 41)
        arms = np.array([self.state(station, a, dx).moment_arm - target for a in grid])
        # sign changes, nearest to the current aoa first
        idx = [i for i in range(len(grid) - 1) if arms[i] * arms[i + 1] <= 0]
        if not idx:
            return float(grid[int(np.argmin(np.abs(arms)))]), False
        i = min(idx, key=lambda j: abs(grid[j] - station.aoa_rel))
        a, b, fa = grid[i], grid[i + 1], arms[i]
        for _ in range(40):  # bisection
            m = 0.5 * (a + b)
            fm = self.state(station, m, dx).moment_arm - target
            if fa * fm <= 0:
                b = m
            else:
                a, fa = m, fm
        return float(0.5 * (a + b)), True

    def solve(self, mix=1.0, target="center"):
        """Per-station exact solve.

        ``mix``: 1 = all twist, 0 = all sweep.  ``target``: see :meth:`target_arm`.
        """
        mix = float(np.clip(mix, 0.0, 1.0))
        goal = self.target_arm(target)
        before, after, d_aoa, dx, reach = [], [], [], [], []
        for s in self.stations:
            st0 = self.state(s)
            before.append(st0)
            if mix > 0.0:
                a_star, ok = self.twist_for_arm(s, target=goal)
                da = mix * (a_star - s.aoa_rel)
            else:
                da, ok = 0.0, True
            if mix >= 1.0 and ok:
                d = 0.0
            else:
                d = self.sweep_for_arm(s, s.aoa_rel + da, target=goal)
            d_aoa.append(da)
            dx.append(d)
            reach.append(ok)
            after.append(self.state(s, s.aoa_rel + da, d))
        return TwistSolution(
            mix=mix, target=goal, stations=self.stations, before=before, after=after,
            d_aoa=np.array(d_aoa), dx=np.array(dx), reachable=np.array(reach),
        )

    # ------------------------------------------------------------------ #
    # smoothing / proposal                                               #
    # ------------------------------------------------------------------ #
    def propose(self, mix=1.0, target="center", dx_numpoints=4, aoa_numpoints=None):
        """Solve, then fit smooth spanwise curves and re-evaluate the arm."""
        sol = self.solve(mix, target)
        pg = self.parametric
        x = sol.x

        aoa_curve = None
        aoa_values = np.array([s.aoa_rel for s in self.stations])
        if mix > 0.0 and np.any(np.abs(sol.d_aoa) > 1e-9):
            n_aoa = aoa_numpoints or len(pg.aoa.controlpoints)
            aoa_curve = fit_symmetric(
                list(zip(x, aoa_values + sol.d_aoa)), numpoints=n_aoa
            )
            aoa_int = aoa_curve.interpolation(num=self.num_interpolate)
            aoa_values = np.array([aoa_int(xi) for xi in x])

        dx_curve = front_curve = back_curve = None
        dx_values = np.zeros(len(x))
        dx_shape = np.zeros(len(x))
        riser_dx = 0.0
        if np.any(np.abs(sol.dx) > 1e-6):
            n_dx = max(2, min(dx_numpoints, len(x)))
            dx_curve = fit_symmetric(list(zip(x, sol.dx)), numpoints=n_dx)
            dx_int = dx_curve.interpolation(num=self.num_interpolate)
            # shape-2d y runs opposite to the flight axis: x_3d = -y + offset
            front_curve = shift_curve(pg.shape.front_curve, lambda xi: -dx_int(xi))
            back_curve = shift_curve(pg.shape.back_curve, lambda xi: -dx_int(xi))
            # what the shape will actually carry (refit is not exact), sampled
            # the way ParametricShape.get_half_shape reads the curves
            num = pg.shape.num_shape_interpolation
            old_front = pg.shape.front_curve.interpolation(num=num)
            new_front = front_curve.interpolation(num=num)
            dx_shape = np.array([-(new_front(xi) - old_front(xi)) for xi in x])
            # the 3d glider always puts the centre rib's nose at x = 0 (see
            # get_glider_3d): a common shift of the planform moves nothing
            # relative to the risers -- it is a riser (pilot) position change.
            riser_dx = float(dx_shape[0])
            dx_values = dx_shape - riser_dx

        after = self.diagnose(aoa_values, dx_values)
        return TwistProposal(
            solution=sol, aoa_curve=aoa_curve, front_curve=front_curve,
            back_curve=back_curve, dx_curve=dx_curve, after=after,
            aoa_values=aoa_values, dx_values=dx_values, dx_shape=dx_shape,
            riser_dx=riser_dx,
        )

    # ------------------------------------------------------------------ #
    # reporting                                                          #
    # ------------------------------------------------------------------ #
    def table(self, states=None):
        states = states or self.diagnose()
        rows = []
        for s, st in zip(self.stations, states):
            rows.append(dict(
                rib=s.index, x=s.x, chord=s.chord,
                aoa_deg=np.degrees(st.aoa_rel), arcang_deg=np.degrees(s.arcang),
                cl=st.cl, x_cp=st.x_cp, arm_mm=st.moment_arm * 1000.0,
                arm_pct=100.0 * st.moment_arm / s.chord if s.chord else 0.0,
                spanwise_deg=st.spanwise_deg, cp_split=st.cp_split,
                suspended=s.suspended,
            ))
        return rows
