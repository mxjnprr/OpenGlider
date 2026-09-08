"""Rounded rib-less wingtip cell, swept by shrinking airfoil sections.

The seam is described by :class:`openglider.glider.rib.rib.TipCurveRib`: a
curve from the previous rib's nose, tangent to the tip station, back to its
trailing edge, on which the extrados and intrados skins are sewn together.

A ruled surface between the rib's skins and that seam pinches at the nose
(the seam leaves the nose spanwise while the nose "rail" has zero length),
so the pocket is modelled the way an inflated bag actually looks: it is swept
by sections of the previous rib's airfoil whose nose and trailing edge slide
along the two branches of the seam, shrinking with the remaining chord until
they meet at the seam's tangent point (the apex).  The cell's symmetric
ballooning then inflates every section like an arc between the rib and the
apex, the bulge vanishing on the seam.

Index structure is kept: point ``i`` of every section corresponds to point
``i`` of the previous rib, so panels, cuts, meshing and flattening work
unchanged.  ``midrib(1.0)`` is the apex (all points coincide): the patterns of
the pocket's panels are fan-shaped pieces converging there, their two side
edges being the seam.
"""

from __future__ import annotations

import numpy as np

from openglider.airfoil import Profile3D
from openglider.glider.cell.cell import Cell
from openglider.utils.cache import cached_function
from openglider.vector import normalize


class _SectionAdapter:
    """Stand-in for ``BasicCell`` for callers of ``cell.basic_cell.midrib``
    (minirib meshes): the pocket has no rib-to-rib basic cell."""

    def __init__(self, cell):
        self.cell = cell

    def midrib(self, y_value, ballooning=True, *args, **kwargs):
        return self.cell.midrib(y_value, ballooning=ballooning)


class TipPocketCell(Cell):
    def _tip_and_rib(self):
        """(tip rib, previous rib, flipped): ``flipped`` when the tip is rib1,
        i.e. on the mirrored (left) wing where Cell.mirror swapped the ribs."""
        from openglider.glider.rib.rib import TipCurveRib

        if isinstance(self.rib2, TipCurveRib):
            return self.rib2, self.rib1, False
        if isinstance(self.rib1, TipCurveRib):
            return self.rib1, self.rib2, True
        raise TypeError("TipPocketCell needs a TipCurveRib on one side")

    @property
    def basic_cell(self):
        return _SectionAdapter(self)

    def _make_profile3d_from_minirib(self, minirib):
        return self.midrib(minirib.y_value)

    @cached_function("self")
    def midrib(
        self,
        y,
        ballooning=True,
        arc_argument=True,
        with_numpy=False,
        close_trailing_edge=False,
    ):
        tip, rib, flipped = self._tip_and_rib()
        t = 1.0 - float(y) if flipped else float(y)
        t = min(max(t, 0.0), 1.0)
        if t <= 0.0:
            return rib.profile_3d

        prev3d = np.array(rib.profile_3d.data, dtype=float)
        nose1 = prev3d[rib.profile_2d.noseindex]
        te1 = 0.5 * (prev3d[0] + prev3d[-1])
        nose2, te2 = tip.tip_line()
        profile = rib.profile_2d
        chord_frac = np.abs(np.array(profile.x_values, dtype=float))
        height = np.array(profile.data, dtype=float)[:, 1]
        nose_index = profile.noseindex

        # section chord: from the seam's front branch to its back branch
        c_front, c_back = tip.seam_branches(t)
        front = (1 - t) * (nose1 + c_front * (te1 - nose1)) + t * (nose2 + c_front * (te2 - nose2))
        back = (1 - t) * (nose1 + c_back * (te1 - nose1)) + t * (nose2 + c_back * (te2 - nose2))
        chord = back - front
        length = float(np.linalg.norm(chord))
        if length < 1e-9:
            return Profile3D(np.repeat(tip.apex[None, :], len(chord_frac), axis=0))

        # thickness axis: blend of both ribs' "up", orthogonal to the chord
        up = (1 - t) * np.array(rib.rotation_matrix([0.0, 1.0, 0.0]), dtype=float)
        up = up + t * np.array(tip.rotation_matrix([0.0, 1.0, 0.0]), dtype=float)
        chord_dir = chord / length
        up = up - np.dot(up, chord_dir) * chord_dir
        up = normalize(up)

        # self-similar section: the airfoil scaled to the remaining chord
        points = front[None, :] + chord_frac[:, None] * chord[None, :]
        points = points + (height * length)[:, None] * up[None, :]

        if ballooning:
            # inflate along the rail from the rib point to the apex, like the
            # ballooning arcs of a normal cell; zero on the seam (nose/TE)
            phi = np.array(self.ballooning_phi, dtype=float)
            rail = np.linalg.norm(prev3d - tip.apex[None, :], axis=1)
            with np.errstate(divide="ignore", invalid="ignore"):
                radius = np.where(phi > 0, rail / (2.0 * np.sin(phi)), 0.0)
            psi = 2.0 * phi * t
            bulge = radius * (np.cos(phi - psi) - np.cos(phi))
            weight = np.abs(height) / max(float(np.abs(height).max()), 1e-12)
            side = np.where(np.arange(len(height)) < nose_index, 1.0, -1.0)
            side[nose_index] = 0.0
            points = points + (weight * bulge * side)[:, None] * up[None, :]

        return Profile3D(points)
