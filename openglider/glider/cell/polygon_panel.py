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
    def __init__(self, region, material_code="", name="unnamed", crossings=None,
                 mesh_ys=None):
        self.region = region
        self.material_code = material_code or ""
        self.name = name
        # the cell's crossing y-values (0<y<1). Every region samples through
        # these so shared cut edges coincide across regions (watertight mesh +
        # matching flattened seams). ``mesh_ys`` kept as a back-compat alias.
        if crossings is None and mesh_ys is not None:
            crossings = [y for y in mesh_ys if 0.0 < y < 1.0]
        self.crossings = list(crossings) if crossings else []

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

    def _ys(self, numribs):
        """Spanwise sample values for this region.

        ``numribs+1`` evenly spaced stations (matching the strip
        ``Panel.get_mesh`` spanwise density, so a crossing cell renders with the
        same flatness/smoothness as its neighbours at any midrib setting) unioned
        with every cell crossing and the region's own kink y-values that fall in
        the region's span (needed so shared cut edges coincide and the apex is a
        vertex). All regions use the same formula → shared edges stay consistent.
        """
        y0, y1 = self.region.y_range
        n = max(int(numribs) + 1, 1)
        base = list(np.linspace(y0, y1, n + 1))
        extra = [y for y in self.crossings if y0 + 1e-9 < y < y1 - 1e-9]
        extra += self.region.segment_ys()
        ys = sorted(set(round(float(y), 9) for y in base) | set(round(float(y), 9) for y in extra))
        if abs(ys[0] - y0) > 1e-9:
            ys.insert(0, round(float(y0), 9))
        if abs(ys[-1] - y1) > 1e-9:
            ys.append(round(float(y1), 9))
        return ys

    # -------------------------------------------------------------------- 3D
    def get_mesh(self, cell, numribs=0, with_numpy=True):
        # Parametric domain is (y, ik) where ik is the profile arc index. The
        # chord direction is sampled at the profile's NATIVE resolution between
        # the region's lo/hi cut (via ``get_positions``) exactly like
        # ``Panel.get_mesh`` — a fixed number of chord points would under-sample
        # a wide region (e.g. one that wraps the nose in a cell with no entry
        # cut) and fold the mesh onto itself.
        xvalues = cell.rib1.base_profile_2d.x_values
        nxv = len(xvalues)
        ys = self._ys(numribs)

        pts2d = []
        pts3d = []
        rows = []
        count = 0
        for y in ys:
            lo, hi = self.region.chord_interval(y)
            midrib = cell.midrib(y, with_numpy=with_numpy)
            ik_lo = get_x_value(xvalues, lo)
            ik_hi = get_x_value(xvalues, hi)
            if abs(ik_hi - ik_lo) < 1e-6:
                # degenerate (apex) row → a single shared point
                pts2d.append([y, ik_lo])
                pts3d.append(np.array(midrib[ik_lo]))
                rows.append([count])
                count += 1
                continue
            row = []
            for ik in midrib.get_positions(ik_lo, ik_hi):
                pts2d.append([y, ik])
                pts3d.append(np.array(midrib[ik]))
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

        # lift ALL mesh points (incl. any Steiner) back to 3D via (y, ik)
        mesh_pts_3d = []
        for pt2d in mesh.points:
            y = float(np.clip(pt2d[0], ys[0], ys[-1]))
            ik = float(np.clip(pt2d[1], 0.0, nxv - 1))
            mesh_pts_3d.append(np.array(cell.midrib(y, with_numpy=with_numpy)[ik]))
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
        ys = self._ys(30)  # flatten quality is independent of the 3d view's midribs
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
