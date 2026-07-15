"""A cell panel bounded by an arbitrary polygon in ``(y, chord)`` parameter
space — used for regions produced by crossing cuts, which the strip ``Panel``
cannot represent (see ``docs/crossing-cuts-design.md``).

3D meshing generalises ``DiagonalRib.get_mesh``: sample a parametric grid that
follows the region's chord interval per ``y``, triangulate (constrained), and
lift every 2D point to the ballooned surface via ``cell.midrib(y)[ik(chord)]``.

Flattening reuses the existing isometric two-rail developer
(``vector.projection.flatten_list``); the spike in the design doc showed it
develops both bent (pointe) and converging (triangle) boundaries cleanly.

Watertightness / seam-matching between neighbouring regions is achieved by
sampling every region at the **same cell-global spanwise ``y`` values**
(``mesh_ys``): a shared cut edge is then sampled identically on both sides, so
its vertices coincide in 3D and its developed length matches in 2D.
"""

from __future__ import annotations

import numpy as np

from openglider.airfoil import get_x_value
from openglider.mesh import Mesh, triangulate
from openglider.vector.projection import flatten_list

_WIDTH_EPS = 1e-6


class PolygonPanel:
    def __init__(self, region, material_code="", name="unnamed", mesh_ys=None,
                 chord_points=6):
        self.region = region
        self.material_code = material_code or ""
        self.name = name
        # cell-global spanwise sample values (shared by all regions of the cell)
        self.mesh_ys = list(mesh_ys) if mesh_ys is not None else None
        self.chord_points = chord_points

    # ------------------------------------------------------------------ helpers
    def mean_x(self):
        return self.region.centroid()[1]

    def is_lower(self):
        return self.mean_x() > 0

    def mirror(self):
        for s in self.region.substrips:
            _, f, b, _, _ = s
            f.left, f.right = -f.left, -f.right
            b.left, b.right = -b.left, -b.right
        self.region.boundary = [(y, -x) for (y, x) in self.region.boundary]

    def __json__(self):
        return {
            "boundary": [list(p) for p in self.region.boundary],
            "material_code": self.material_code,
            "name": self.name,
        }

    def _ys(self):
        """Spanwise sample values for this region: the cell-global set clipped
        to the region's span, unioned with the region's own kink y-values."""
        y0, y1 = self.region.y_range
        seg = self.region.segment_ys()
        if self.mesh_ys is not None:
            base = [y for y in self.mesh_ys if y0 - 1e-9 <= y <= y1 + 1e-9]
        else:
            base = list(np.linspace(y0, y1, 12))
        ys = sorted(set(round(float(y), 9) for y in base) | set(seg))
        # guarantee the endpoints are present
        if abs(ys[0] - y0) > 1e-9:
            ys.insert(0, round(float(y0), 9))
        if abs(ys[-1] - y1) > 1e-9:
            ys.append(round(float(y1), 9))
        return ys

    # -------------------------------------------------------------------- 3D
    def get_mesh(self, cell, numribs=0, with_numpy=True):
        xvalues = cell.rib1.base_profile_2d.x_values
        nx = self.chord_points
        ys = self._ys()

        pts2d = []
        pts3d = []
        rows = []
        count = 0
        for y in ys:
            lo, hi = self.region.chord_interval(y)
            midrib = cell.midrib(y, with_numpy=with_numpy)
            if hi - lo < _WIDTH_EPS:
                # degenerate (apex) row → a single shared point
                chord = 0.5 * (lo + hi)
                pts2d.append([y, chord])
                pts3d.append(np.array(midrib[get_x_value(xvalues, chord)]))
                rows.append([count])
                count += 1
                continue
            row = []
            for xt in np.linspace(0.0, 1.0, nx):
                chord = lo + xt * (hi - lo)
                pts2d.append([y, chord])
                pts3d.append(np.array(midrib[get_x_value(xvalues, chord)]))
                row.append(count)
                count += 1
            rows.append(row)

        edge = self._outline(rows)
        group = "panel_" + self.material_code

        tri = triangulate.Triangulation(pts2d, [edge + [edge[0]]])
        mesh = None
        for opts in ("QzpY", "Qzp", "Qz"):
            try:
                m = tri.triangulate(options=opts)
                if len(m.elements) > 0:
                    mesh = m
                    break
            except Exception:
                continue

        if mesh is None or len(mesh.elements) == 0:
            # last-resort fan triangulation of the outline
            tris = [[edge[0], edge[i], edge[i + 1]] for i in range(1, len(edge) - 1)]
            return Mesh.from_indexed(np.array(pts3d), {group: tris},
                                     boundaries={group: edge})

        # lift ALL mesh points (incl. any Steiner) back to 3D
        mesh_pts_3d = []
        for pt2d in mesh.points:
            y = float(np.clip(pt2d[0], ys[0], ys[-1]))
            chord = float(pt2d[1])
            mesh_pts_3d.append(np.array(cell.midrib(y, with_numpy=with_numpy)[get_x_value(xvalues, chord)]))
        return Mesh.from_indexed(np.array(mesh_pts_3d),
                                 {group: list(mesh.elements)})

    @staticmethod
    def _outline(rows):
        """CCW index outline of a (possibly variable-length / apex-collapsed)
        grid of rows — bottom row L→R, up the right, top row R→L, down the left."""
        edge = list(rows[0])
        edge += [r[-1] for r in rows[1:]]
        if len(rows[-1]) > 1:
            edge += rows[-1][-2::-1]
        edge += [r[0] for r in rows[1:-1]][::-1]
        # drop consecutive duplicates (apex rows share their single index)
        out = []
        for i in edge:
            if not out or out[-1] != i:
                out.append(i)
        if len(out) > 1 and out[0] == out[-1]:
            out.pop()
        return out

    # ------------------------------------------------------------------ 2D
    def flatten_rails(self, cell, with_numpy=True):
        """Develop the region into two 2D rails via the isometric developer.
        Returns ``(flat_lo, flat_hi)`` as ``PolyLine2D``."""
        xvalues = cell.rib1.base_profile_2d.x_values
        ys = self._ys()
        rail_lo = []
        rail_hi = []
        for y in ys:
            lo, hi = self.region.chord_interval(y)
            midrib = cell.midrib(y, with_numpy=with_numpy)
            rail_lo.append(np.array(midrib[get_x_value(xvalues, lo)]))
            rail_hi.append(np.array(midrib[get_x_value(xvalues, hi)]))
        return flatten_list(rail_lo, rail_hi)

    def get_flattened(self, cell, with_numpy=True):
        """2D contour (``PolyLine2D``) of the developed region — no seam
        allowance yet (that wiring is a later integration step)."""
        from openglider.vector.polyline import PolyLine2D
        flat_lo, flat_hi = self.flatten_rails(cell, with_numpy=with_numpy)
        contour = list(np.array(flat_lo)) + list(np.array(flat_hi))[::-1]
        return PolyLine2D(contour)
