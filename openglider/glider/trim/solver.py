"""
Core (FreeCAD-independent) trimming solver.

The solver works on a :class:`ParametricGlider`.  It builds the 3d glider and its
lineset once as a *baseline*, records every attachment-point position and every
line length, and then answers the question:

    "If the pilot changes these line lengths by these amounts, where does the wing
     want to go, and what arch / AoA / profile change does that imply?"

Model (v1, first-order / linearised)
------------------------------------
* Each attachment point ``k`` is connected to a ground node (riser) through an
  ordered cascade of lines ``path(k)``.  Lengthening a line pushes the wing point
  away from the pilot along that line's current tension direction.  A line shared
  by several attachment points moves the whole bundle together::

      P'_k = P_k + sum_{line in path(k)} delta_line * u_line

* Per rib we fit a rigid body transform (Kabsch/SVD) to the displaced attachment
  points, then decompose it into a pitch component about the span axis (= dAoA)
  and a roll component about the chord axis (= d-arc).  The residual of the fit is
  the profile deformation to be baked into a ``profile_override``.

The force re-solve (outer fixed-point iteration) is intentionally left as a thin
wrapper hook (:meth:`TrimModel.solve` ``iterations`` argument) so it can be added
once the linear core is validated.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

from openglider.airfoil import Profile2D
from openglider.glider.parametric.arc import ArcCurve
from openglider.vector.spline import SymmetricBSpline


# --------------------------------------------------------------------------- #
# small linear-algebra helpers                                                #
# --------------------------------------------------------------------------- #
def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros_like(v)


def kabsch(P: np.ndarray, Q: np.ndarray):
    """Best-fit rigid transform mapping the point cloud ``P`` onto ``Q``.

    Returns ``(R, t)`` (3x3 rotation, 3 translation) minimising
    ``sum_i |R @ P_i + t - Q_i|**2``.
    """
    Pc = P.mean(axis=0)
    Qc = Q.mean(axis=0)
    H = (P - Pc).T @ (Q - Qc)
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = Qc - R @ Pc
    return R, t


def kabsch_2d(p: np.ndarray, q: np.ndarray):
    """Best-fit 2d rotation angle + translation mapping ``p`` onto ``q``.

    Returns ``(theta, t)``.  Well-posed as long as ``p`` spans at least one
    dimension (i.e. two distinct points) -- unlike a full 3d fit on the
    near-colinear attachment points of a single rib.
    """
    pc = p.mean(axis=0)
    qc = q.mean(axis=0)
    a = p - pc
    b = q - qc
    # 2d Kabsch: theta = atan2(sum(ax*by - ay*bx), sum(ax*bx + ay*by))
    num = float(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]))
    den = float(np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    theta = np.arctan2(num, den)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = qc - R @ pc
    return theta, R, t


def fit_symmetric(points, numpoints):
    """Fit a :class:`SymmetricBSpline` to half-wing ``points`` [(x, value), ...].

    The curve is mirrored about x=0 (as the model's arc/aoa splines are), then
    fitted with ``numpoints`` control points -- fewer points == smoother proposal.
    """
    pts = [np.asarray(p, dtype=float) for p in points]
    mirrored = [p * [-1.0, 1.0] for p in pts[::-1]] + pts
    return SymmetricBSpline.fit(mirrored, numpoints=numpoints)


def arc_from_positions(positions, x_values, numpoints):
    """Build an :class:`ArcCurve` from half-wing arc (y, z) positions.

    The arc spline lives in an arc-length-parameterised space, so we cannot fit
    the absolute positions directly (``get_arc_positions`` re-walks the curve and
    blows up).  Instead we convert positions to per-station *cell angles* -- the
    same quantity :meth:`ArcCurve.get_cell_angles` produces -- and integrate them
    back, mirroring :meth:`ArcCurve.from_cell_angles` but with a tunable
    ``numpoints`` for smoothing.
    """
    positions = [np.asarray(p, dtype=float) for p in positions]
    angles = [0.0]  # center cell is straight (has_center_cell convention)
    for left, right in zip(positions[:-1], positions[1:]):
        d = right - left
        angles.append(float(np.arctan2(-d[1], d[0])))

    last_pos = np.array([0.0, 0.0])
    last_x = 0.0
    curve = []
    for i, x in enumerate(x_values):
        d = np.array([np.cos(angles[i]), -np.sin(angles[i])])
        last_pos = last_pos + d * (x - last_x)
        last_x = x
        curve.append(last_pos)

    mirrored = [p * [-1.0, 1.0] for p in curve[::-1]] + curve
    return ArcCurve(SymmetricBSpline.fit(mirrored, numpoints=numpoints))


def deform_profile(profile, nose_index, samples, name=None):
    """Deform a normalized profile by a set of local displacements.

    ``samples`` is ``[(point_index, dx, dy), ...]`` in normalized profile
    coordinates (residual metres / chord).  The displacement is interpolated over
    the profile point index and anchored to zero at the trailing edges and the
    nose, so the leading edge and the un-attached surface stay put while the
    attachment region (intrados) deforms smoothly.
    """
    data = np.asarray(profile.data, dtype=float)
    n = len(data)
    anchors = {0.0: (0.0, 0.0), float(nose_index): (0.0, 0.0), float(n - 1): (0.0, 0.0)}
    for pf, dx, dy in samples:
        anchors[float(pf)] = (dx, dy)  # samples override end/nose anchors if colocated
    xs = np.array(sorted(anchors))
    dxs = np.array([anchors[x][0] for x in xs])
    dys = np.array([anchors[x][1] for x in xs])
    idx = np.arange(n)
    new = data.copy()
    new[:, 0] += np.interp(idx, xs, dxs)
    new[:, 1] += np.interp(idx, xs, dys)
    return Profile2D(new.tolist(), name=name)


def rib_local_frame(rib):
    """Orthonormal local frame of a rib: (origin, chord axis, thickness axis).

    ``e_chord`` points leading->trailing, ``e_thick`` along profile thickness;
    their cross product is the spanwise rib normal (rotation about it == AoA).
    """
    o = np.array(rib.align([0.0, 0.0]), dtype=float)
    e_chord = _unit(np.array(rib.align([1.0, 0.0]), dtype=float) - o)
    e_thick = _unit(np.array(rib.align([0.0, 1.0]), dtype=float) - o)
    return o, e_chord, e_thick


# --------------------------------------------------------------------------- #
# results                                                                     #
# --------------------------------------------------------------------------- #
@dataclass
class RibTrim:
    """Per-rib outcome of a trim solve.

    ``d_aoa`` and the residual come from a 2d fit *in the rib's own plane* (the
    only well-posed per-rib DOF).  ``d_pos`` is the rib's centroid displacement in
    global coordinates and feeds the spanwise arch reconstruction; the arc angle
    is *not* fitted per rib (it is unobservable from near-colinear attachment
    points) but derived from the arch curve tangent in :meth:`TrimResult.arch`.
    """

    rib_index: int
    d_aoa: float                       # [rad] incremental angle of attack
    d_pos: np.ndarray                  # [3] centroid displacement (x, y, z)
    residual: Dict[str, np.ndarray]    # attachment name -> [d_chord, d_thick] (chord units)
    max_residual: float                # worst in-plane residual magnitude [m]


@dataclass
class CurveProposal:
    """Proposed new arch + AoA curves derived from a trim result."""

    aoa_curve: object                  # SymmetricBSpline (span-x -> aoa [rad])
    arc_curve: object                  # ArcCurve
    xv: np.ndarray                     # half-wing station x-values
    new_aoa: np.ndarray                # target aoa per station [rad]
    new_arc: List[np.ndarray]          # target (y, z) per station
    base_aoa: np.ndarray
    base_arc: List[np.ndarray]
    profiles: Dict[int, object] = field(default_factory=dict)  # station -> Profile2D

    def apply_to(self, parametric_glider, bake_profiles: bool = True) -> None:
        """Write the proposed arch + AoA curves (+ baked profiles) into a glider."""
        parametric_glider.arc = self.arc_curve
        parametric_glider.aoa = self.aoa_curve
        if bake_profiles and self.profiles:
            overrides = dict(getattr(parametric_glider, "profile_overrides", {}))
            for station, profile in self.profiles.items():
                parametric_glider.profiles.append(profile)
                overrides[str(station)] = len(parametric_glider.profiles) - 1
            parametric_glider.profile_overrides = overrides
            parametric_glider.profile_overrides_enabled = True


@dataclass
class TrimResult:
    ribs: Dict[int, RibTrim] = field(default_factory=dict)
    # displaced attachment-point positions, by name
    attachment_positions: Dict[str, np.ndarray] = field(default_factory=dict)
    # baseline rib centroids (global 3d), by rib index -- for arch reconstruction
    rib_centroid: Dict[int, np.ndarray] = field(default_factory=dict)

    def arch(self) -> Dict[int, np.ndarray]:
        """New arch positions (y, z) per rib in the front view.

        Baseline centroid + displacement, projected to the spanwise plane.
        """
        out = {}
        for i, r in self.ribs.items():
            base = self.rib_centroid.get(i)
            if base is None:
                continue
            new = base + r.d_pos
            out[i] = np.array([new[1], new[2]])  # (span y, vertical z)
        return out

    def as_table(self) -> List[dict]:
        rows = []
        for i in sorted(self.ribs):
            r = self.ribs[i]
            rows.append(
                dict(
                    rib=i,
                    d_aoa_deg=np.degrees(r.d_aoa),
                    dz=float(r.d_pos[2]),
                    dy=float(r.d_pos[1]),
                    max_residual_mm=r.max_residual * 1000.0,
                )
            )
        return rows


# --------------------------------------------------------------------------- #
# model                                                                       #
# --------------------------------------------------------------------------- #
class TrimModel:
    """Baseline snapshot + inverse solve for a parametric glider."""

    def __init__(self, parametric_glider, calculate_sag: bool = False):
        self.parametric = parametric_glider
        self.glider = parametric_glider.get_glider_3d()
        self.lineset = self.glider.lineset
        # make sure geometry + line forces are populated
        self.lineset.recalc(calculate_sag=calculate_sag)

        # baseline snapshots ------------------------------------------------- #
        self.attachment_points = list(self.lineset.attachment_points)
        self.baseline_pos: Dict[int, np.ndarray] = {
            id(ap): np.array(ap.vec, dtype=float) for ap in self.attachment_points
        }
        self.baseline_length: Dict[int, float] = {
            id(ln): float(np.linalg.norm(ln.upper_node.vec - ln.lower_node.vec))
            for ln in self.lineset.lines
        }
        # rib -> index (only ribs carrying rib-attachment points)
        self._rib_index = {id(rib): i for i, rib in enumerate(self.glider.ribs)}

        # half-wing stations (the arc/aoa splines are symmetric, defined over the
        # half span).  Map every 3d rib to its nearest station by |span|.
        self.xv = np.asarray(self.parametric.shape.rib_x_values, dtype=float)
        self.base_arc = [
            np.asarray(p, dtype=float)
            for p in self.parametric.arc.get_arc_positions(self.xv)
        ]
        base_arc_y = np.array([p[0] for p in self.base_arc])
        self._rib_station = {
            i: int(np.argmin(np.abs(base_arc_y - abs(rib.pos[1]))))
            for i, rib in enumerate(self.glider.ribs)
        }
        aoa_int = self.parametric.aoa.interpolation(
            num=self.parametric.num_interpolate
        )
        self.base_aoa = np.array([aoa_int(x) for x in self.xv])

    # ------------------------------------------------------------------ #
    # topology                                                           #
    # ------------------------------------------------------------------ #
    def path_to_ground(self, attachment_point) -> List:
        """Ordered list of lines from an attachment point down to its riser."""
        path = []
        node = attachment_point
        seen = set()
        while getattr(node, "type", None) != 0:
            lowers = self.lineset.get_lower_connected_lines(node)
            if not lowers:
                break
            line = lowers[0]
            if id(line) in seen:  # cycle guard
                break
            seen.add(id(line))
            path.append(line)
            node = line.lower_node
        return path

    def infer_layers(self) -> Dict[str, str]:
        """Group attachment points into A/B/C/D... layers.

        Attachment-point names normally encode the layer in their alphabetic
        prefix (A1, B3, C2, BR1 ...), which is the reliable grouping -- one point
        per rib per row.  Only when the names carry no such prefix do we fall
        back to clustering the chord position ``rib_pos``.
        """
        import re

        prefix_re = re.compile(r"^([A-Za-z]+)")
        by_prefix = {}
        for ap in self.attachment_points:
            match = prefix_re.match(ap.name or "")
            if match:
                by_prefix[ap.name] = match.group(1).upper()
        # accept the name-based grouping if it yields at least two rows and covers
        # essentially every attachment point
        if len(set(by_prefix.values())) >= 2 and len(by_prefix) >= 0.8 * len(
            self.attachment_points
        ):
            return by_prefix

        aps = [ap for ap in self.attachment_points if hasattr(ap, "rib_pos")]
        positions = sorted({round(float(ap.rib_pos), 4) for ap in aps})
        if not positions:
            return {}
        # split on the largest gaps between consecutive chord positions
        gaps = [(positions[i + 1] - positions[i], i) for i in range(len(positions) - 1)]
        gaps.sort(reverse=True)
        # heuristic: a new layer starts wherever the gap exceeds 1.5x the median gap
        if gaps:
            median_gap = np.median([g for g, _ in gaps]) or 1e-9
            cuts = sorted(i for g, i in gaps if g > 1.5 * median_gap)
        else:
            cuts = []
        band_of = {}
        band = 0
        for i, pos in enumerate(positions):
            band_of[pos] = band
            if i in cuts:
                band += 1
        labels = "ABCDEFGH"
        return {
            ap.name: labels[min(band_of[round(float(ap.rib_pos), 4)], len(labels) - 1)]
            for ap in aps
        }

    # ------------------------------------------------------------------ #
    # solve                                                              #
    # ------------------------------------------------------------------ #
    def propagate(self, deltas: Dict[int, float]) -> Dict[int, np.ndarray]:
        """Displace each attachment point given per-line length deltas [m].

        ``deltas`` is keyed by ``id(line)``.  Positive = longer line.
        """
        new_pos = {}
        for ap in self.attachment_points:
            disp = np.zeros(3)
            for line in self.path_to_ground(ap):
                d = deltas.get(id(line), 0.0)
                if d:
                    disp += d * _unit(line.upper_node.vec - line.lower_node.vec)
            new_pos[id(ap)] = self.baseline_pos[id(ap)] + disp
        return new_pos

    def fit(self, new_pos: Dict[int, np.ndarray]) -> TrimResult:
        """Per-rib fit of the displaced attachment points.

        Per rib we solve a *2d* rigid fit in the rib's own plane -> AoA change +
        in-plane residual (profile deformation).  The rib centroid's global
        displacement is kept for the spanwise arch reconstruction.
        """
        result = TrimResult()
        for ap in self.attachment_points:
            result.attachment_positions[ap.name] = new_pos[id(ap)]

        by_rib: Dict[int, list] = {}
        for ap in self.attachment_points:
            rib = getattr(ap, "rib", None)
            if rib is None or id(rib) not in self._rib_index:
                continue  # CellAttachmentPoint: handled in a later revision
            by_rib.setdefault(id(rib), []).append(ap)

        for rib_id, aps in by_rib.items():
            i = self._rib_index[rib_id]
            rib = self.glider.ribs[i]
            o, e_chord, e_thick = rib_local_frame(rib)

            P = np.array([self.baseline_pos[id(ap)] for ap in aps])
            Q = np.array([new_pos[id(ap)] for ap in aps])
            result.rib_centroid[i] = P.mean(axis=0)

            # project into the rib plane (chord, thickness)
            p2 = np.column_stack([(P - o) @ e_chord, (P - o) @ e_thick])
            q2 = np.column_stack([(Q - o) @ e_chord, (Q - o) @ e_thick])

            if len(aps) >= 2 and np.ptp(p2[:, 0]) + np.ptp(p2[:, 1]) > 1e-9:
                theta, R2, t2 = kabsch_2d(p2, q2)
            else:  # single attachment point: pure translation, no rotation
                theta, R2, t2 = 0.0, np.eye(2), (q2[0] - p2[0])

            residual = {
                ap.name: (q2[j] - (R2 @ p2[j] + t2)) for j, ap in enumerate(aps)
            }
            max_res = max(
                (float(np.linalg.norm(v)) for v in residual.values()), default=0.0
            )
            result.ribs[i] = RibTrim(
                rib_index=i,
                d_aoa=float(theta),
                d_pos=Q.mean(axis=0) - P.mean(axis=0),
                residual=residual,
                max_residual=max_res,
            )
        return result

    def to_curves(
        self, result: TrimResult, arc_numpoints: int = 3, aoa_numpoints: int = 3
    ) -> "CurveProposal":
        """Turn a per-rib trim result into proposed (smoothed) arch + AoA curves.

        Folds left/right ribs onto the half-wing stations (the curves are
        symmetric), averages, then fits symmetric B-splines.  ``*_numpoints`` is
        the smoothing knob (fewer == smoother).  NB: keep ``arc_numpoints`` low
        (3-6); >=8 control points make ``ArcCurve.get_arc_positions`` degenerate.
        """
        arc_numpoints = max(3, min(int(arc_numpoints), 6))
        aoa_numpoints = max(2, int(aoa_numpoints))
        n = len(self.xv)
        d_aoa = np.zeros(n)
        d_yz = np.zeros((n, 2))
        count = np.zeros(n)
        for i, rib in enumerate(self.glider.ribs):
            trim = result.ribs.get(i)
            if trim is None:
                continue
            k = self._rib_station[i]
            fold = -1.0 if rib.pos[1] < 0 else 1.0  # mirror left onto right half
            d_aoa[k] += trim.d_aoa
            d_yz[k] += [fold * trim.d_pos[1], trim.d_pos[2]]
            count[k] += 1
        count[count == 0] = 1.0
        d_aoa /= count
        d_yz /= count[:, None]

        new_aoa = self.base_aoa + d_aoa
        new_arc = [self.base_arc[k] + d_yz[k] for k in range(n)]

        aoa_curve = fit_symmetric(
            [[self.xv[k], new_aoa[k]] for k in range(n)], aoa_numpoints
        )
        arc_curve = arc_from_positions(new_arc, self.xv, arc_numpoints)
        profiles = self._bake_profiles(result)
        return CurveProposal(
            aoa_curve=aoa_curve,
            arc_curve=arc_curve,
            profiles=profiles,
            xv=self.xv,
            new_aoa=new_aoa,
            new_arc=new_arc,
            base_aoa=self.base_aoa.copy(),
            base_arc=[p.copy() for p in self.base_arc],
        )

    def _bake_profiles(self, result: TrimResult, min_residual: float = 1e-3):
        """Deform each station's profile from the fit residuals (baked override).

        Returns ``{station -> Profile2D}`` for stations whose worst residual
        exceeds ``min_residual`` (metres).
        """
        # representative rib per station: rightmost fitted rib
        rep: Dict[int, int] = {}
        for i, rib in enumerate(self.glider.ribs):
            if i not in result.ribs:
                continue
            k = self._rib_station[i]
            if k not in rep or rib.pos[1] > self.glider.ribs[rep[k]].pos[1]:
                rep[k] = i

        profiles: Dict[int, object] = {}
        for k, i in rep.items():
            trim = result.ribs[i]
            if trim.max_residual < min_residual:
                continue
            rib = self.glider.ribs[i]
            chord = float(rib.chord)
            aps = [
                ap
                for ap in self.attachment_points
                if getattr(ap, "rib", None) is rib and ap.name in trim.residual
            ]
            samples = []
            for ap in aps:
                pf = rib.profile_2d(ap.rib_pos)  # fractional point index
                res = trim.residual[ap.name] / chord
                samples.append((pf, float(res[0]), float(res[1])))
            if samples:
                profiles[k] = deform_profile(
                    rib.profile_2d,
                    rib.profile_2d.noseindex,
                    samples,
                    name=f"trim_station_{k}",
                )
        return profiles

    def deltas_by_id(self, deltas: Dict[int, float]) -> Dict[int, float]:
        """Map ``line.number``-keyed deltas onto this model's ``id(line)`` keys.

        Line object identity changes on every rebuild, so the stable public key is
        ``line.number`` (deterministic for a given parametric glider).
        """
        by_number = {ln.number: ln for ln in self.lineset.lines}
        return {
            id(by_number[num]): value
            for num, value in deltas.items()
            if num in by_number
        }

    def solve(self, deltas: Dict[int, float]) -> TrimResult:
        """Single linear inverse pass from ``line.number``-keyed deltas [m]."""
        return self.fit(self.propagate(self.deltas_by_id(deltas)))

    def compensate(
        self, deltas: Dict[int, float], comp_layers, layers=None
    ) -> Dict[int, float]:
        """Length changes on ``comp_layers`` that keep each profile rigid.

        Given a set of trims (e.g. the C row shortened), solve -- per rib, by
        least squares -- for the extra length change on each compensation layer's
        line so that every attachment of that rib moves as close as possible to a
        single rigid body (rotation + translation), i.e. minimal deformation.

        Note: because lines pull radially, a pure AoA change (rotation) can never
        be perfectly rigid -- some residual deformation is inherent; this only
        minimises it.  ``comp_layers`` may be a single layer name or a list.

        Returns ``{line.number: delta}`` to *add* to ``deltas``.
        """
        if isinstance(comp_layers, str):
            comp_layers = [comp_layers]
        layers = layers or self.infer_layers()
        disp = self.propagate(self.deltas_by_id(deltas))

        by_rib: Dict[int, list] = {}
        for ap in self.attachment_points:
            rib = getattr(ap, "rib", None)
            if rib is None or id(rib) not in self._rib_index:
                continue
            by_rib.setdefault(id(rib), []).append(ap)

        compensation: Dict[int, float] = {}
        for rib_id, aps in by_rib.items():
            rib = self.glider.ribs[self._rib_index[rib_id]]
            o, e_chord, e_thick = rib_local_frame(rib)

            def to2d(vec):
                return np.array([float(vec @ e_chord), float(vec @ e_thick)])

            # compensation lines on this rib (uppermost segment of each layer)
            comp_lines = {}  # ap -> (line, u_2d)
            for ap in aps:
                if layers.get(ap.name) in comp_layers:
                    path = self.path_to_ground(ap)
                    if path:
                        comp_lines[id(ap)] = (
                            path[0],
                            to2d(_unit(path[0].upper_node.vec - path[0].lower_node.vec)),
                        )
            if not comp_lines:
                continue
            comp_ap_ids = list(comp_lines)  # ordered unknowns

            p2 = {id(ap): to2d(self.baseline_pos[id(ap)] - o) for ap in aps}
            centroid = np.mean(list(p2.values()), axis=0)

            # unknowns: [theta, tx, ty, dm_0, dm_1, ...]
            ncomp = len(comp_ap_ids)
            rows, rhs = [], []
            for ap in aps:
                p = p2[id(ap)]
                perp = np.array([-(p - centroid)[1], (p - centroid)[0]])
                d0 = to2d(disp[id(ap)] - self.baseline_pos[id(ap)])
                comp_coef = np.zeros((2, ncomp))
                if id(ap) in comp_lines:
                    k = comp_ap_ids.index(id(ap))
                    comp_coef[:, k] = -comp_lines[id(ap)][1]
                rows.append([perp[0], 1.0, 0.0] + list(comp_coef[0]))
                rhs.append(d0[0])
                rows.append([perp[1], 0.0, 1.0] + list(comp_coef[1]))
                rhs.append(d0[1])
            sol, *_ = np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)
            for k, ap_id in enumerate(comp_ap_ids):
                line = comp_lines[ap_id][0]
                compensation[line.number] = compensation.get(line.number, 0.0) + float(
                    sol[3 + k]
                )
        return compensation

    def solve_iterative(
        self,
        deltas: Dict[int, float],
        iterations: int = 5,
        tol: float = 2e-3,
        arc_numpoints: int = 3,
        aoa_numpoints: int = 3,
    ):
        """Gain-corrected inverse solve (Newton on the prescribed scale).

        The single linear pass prescribes attachment displacements ``d_des`` but
        the forward map (arc reconstruction + symmetric-spline smoothing + rib
        rotation) realises them with a gain ``G != 1``, so the rebuilt wing over-
        or under-shoots.  We measure ``G`` from the rebuilt geometry and rescale
        the prescribed displacement by ``1/G`` until the wing lands on the trimmed
        attachment targets.  Fitting always uses the original tension directions,
        which keeps the iteration stable (unlike a naive re-linearising fixed
        point, which diverges here).

        Returns ``(proposal, info)`` with ``info['gaps']`` the per-iteration worst
        attachment gap [m] and ``info['scale']`` the final gain compensation.
        """
        from copy import deepcopy

        d_des = {}  # desired displacement from original baseline, by name
        target0 = self.propagate(self.deltas_by_id(deltas))
        for ap in self.attachment_points:
            d = target0[id(ap)] - self.baseline_pos[id(ap)]
            if np.linalg.norm(d) > 0:
                d_des[ap.name] = d
        den = sum(float(v @ v) for v in d_des.values()) or 1.0

        best = (float("inf"), None)
        gaps = []
        scale = 1.0
        for it in range(iterations):
            new_pos = {
                id(ap): self.baseline_pos[id(ap)]
                + scale * d_des.get(ap.name, np.zeros(3))
                for ap in self.attachment_points
            }
            proposal = self.to_curves(
                self.fit(new_pos),
                arc_numpoints=arc_numpoints,
                aoa_numpoints=aoa_numpoints,
            )
            # apply and rebuild to measure what the forward map actually realised
            parametric = deepcopy(self.parametric)
            proposal.apply_to(parametric)
            rebuilt = TrimModel(parametric)
            achieved = {
                ap.name: rebuilt.baseline_pos[id(ap)]
                - self._baseline_pos_by_name(ap.name)
                for ap in rebuilt.attachment_points
                if ap.name in d_des
            }
            gap = max(
                (
                    float(np.linalg.norm(d_des[name] - achieved[name]))
                    for name in achieved
                ),
                default=0.0,
            )
            gaps.append(gap)
            improved = gap < best[0]
            if improved:
                best = (gap, proposal)
            if gap < tol or not improved:
                # representation-limited: a symmetric low-DOF arch/AoA cannot get
                # closer, so gain rescaling only makes it worse -> stop at best.
                break
            # global forward gain G = <achieved, desired> / (scale * <desired, desired>)
            num = sum(float(achieved[name] @ d_des[name]) for name in achieved)
            gain = num / (scale * den) if den else 1.0
            if abs(gain) < 1e-3:
                break
            scale = float(np.clip(scale / gain, 0.3, 2.0))  # deconvolve, clamped

        return best[1], {"gaps": gaps, "iterations": len(gaps), "best_gap": best[0]}

    def _baseline_pos_by_name(self, name):
        for ap in self.attachment_points:
            if ap.name == name:
                return self.baseline_pos[id(ap)]
        return np.zeros(3)
