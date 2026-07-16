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
        """Spanwise mirror, matching ``Panel.mirror`` semantics (used by
        ``Cell.mirror`` when building the mirrored wing). ``Panel.mirror`` swaps
        each cut's left/right (rib1<->rib2, chord VALUES unchanged) — NOT a chord
        negation. So we swap left/right on (deep-copied, since cuts are shared
        between regions) cuts and reflect the parametric span y -> 1-y. After
        this, ``chord_interval(y)`` equals the original ``chord_interval(1-y)``,
        which matches ``cell.midrib`` after Cell.mirror swaps rib1/rib2.
        """
        from openglider.glider.cell.cut_arrangement import Cut, Region

        cutmap = {}

        def mir(c):
            if id(c) not in cutmap:
                cutmap[id(c)] = Cut(c.right, c.left, c.type, c.id)  # swap L/R
            return cutmap[id(c)]

        new_subs = [
            (seg, mir(f), mir(b), 1.0 - y_hi, 1.0 - y_lo)
            for (seg, f, b, y_lo, y_hi) in self.region.substrips
        ]
        new_boundary = [(1.0 - y, chord) for (y, chord) in self.region.boundary]
        self.region = Region(new_boundary, new_subs)

    def __json__(self):
        return {
            "boundary": [list(p) for p in self.region.boundary],
            "material_code": self.material_code,
            "name": self.name,
        }

    def _ys(self, numribs):
        """Spanwise sample values for this region.

        Built from a **cell-global** grid ``linspace(0, 1, numribs+2)`` clipped
        to the region's span (NOT a region-local linspace — that would give
        different y-values to regions of different span and crack their shared
        cut edges with T-junctions). Unioned with every cell crossing and the
        region's own kink y-values. Because every region samples the *same*
        global grid, a shared cut edge gets identical vertices on both sides.
        Density follows ``numribs`` so a crossing cell matches its strip
        neighbours' flatness at midribs=0 and smooths with them as it rises.
        """
        y0, y1 = self.region.y_range
        n = max(int(numribs) + 1, 1)
        global_grid = [i / n for i in range(n + 1)]  # linspace(0, 1, n+1), GLOBAL
        base = [y for y in global_grid if y0 - 1e-9 <= y <= y1 + 1e-9]
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

        # Triangulate in a NORMALISED parameter box, not raw (y, ik). y spans
        # [0,1] but ik spans [0, nxv-1] (~150), so raw-space Delaunay is wildly
        # anisotropic and tiles the region with zigzag slivers; lifted onto the
        # curved surface their normals flip between neighbours (dihedral up to
        # ~145°), so coin3d shades the crossing as a creased/dark fold. Scaling
        # each axis to its own extent (as DiagonalRib.get_mesh does with a unit
        # [0,1]x[0,1] box) yields well-shaped triangles → smooth normals. The
        # region's boundary vertices are preserved (``Y`` keeps them, no Steiner),
        # so shared cut edges still weld across regions (watertight unchanged).
        pts2d_arr = np.asarray(pts2d, float)
        y_lo_n, y_hi_n = pts2d_arr[:, 0].min(), pts2d_arr[:, 0].max()
        ik_lo_n, ik_hi_n = pts2d_arr[:, 1].min(), pts2d_arr[:, 1].max()
        sy = 1.0 / max(y_hi_n - y_lo_n, _WIDTH_EPS)
        sik = 1.0 / max(ik_hi_n - ik_lo_n, _WIDTH_EPS)
        norm_pts = [[(p[0] - y_lo_n) * sy, (p[1] - ik_lo_n) * sik] for p in pts2d]

        tri = triangulate.Triangulation(norm_pts, [edge + [edge[0]]])
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

        # lift ALL mesh points (incl. any Steiner) back to 3D — invert the
        # normalisation to recover the true (y, ik) before mapping to the surface.
        mesh_pts_3d = []
        for pt2d in mesh.points:
            y = float(np.clip(pt2d[0] / sy + y_lo_n, ys[0], ys[-1]))
            ik = float(np.clip(pt2d[1] / sik + ik_lo_n, 0.0, nxv - 1))
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
    def _flatten_boundaries(self, cell, numribs=14):
        """Develop the region using the cell's isometric ``inner`` development
        (which carries the profile ARC), returning
        ``(lo_rail, hi_rail, bottom_arc, top_arc)`` as lists of 2D points.

        The lo/hi rails are the region's two chord boundaries (the cuts); the
        bottom/top arcs are the developed profile arc across the region at its
        first/last spanwise station. Using the cell-global ``inner`` lines (same
        for every region) makes a shared cut edge develop to the SAME length in
        both neighbours, and the width lo->hi is the true profile arc (not a
        straight chord — the fix for nose-wrapping regions).
        """
        inner = cell.get_flattened_cell(numribs)["inner"]
        n = len(inner)
        xvalues = cell.rib1.base_profile_2d.x_values
        y0, y1 = self.region.y_range

        idxs = [i for i in range(n) if y0 - 1e-9 <= i / (n - 1) <= y1 + 1e-9]
        if len(idxs) < 2:
            idxs = sorted({
                max(0, min(n - 1, int(round(y0 * (n - 1))))),
                max(0, min(n - 1, int(round(y1 * (n - 1))))),
            })
            if len(idxs) < 2:
                idxs = [idxs[0], min(n - 1, idxs[0] + 1)]

        lo_rail, hi_rail = [], []
        lo_types, hi_types = [], []
        station_ys = []
        iks = []
        for i in idxs:
            y = min(max(i / (n - 1), y0), y1)
            station_ys.append(y)
            lo, hi = self.region.chord_interval(y)
            ik_lo = get_x_value(xvalues, lo)
            ik_hi = get_x_value(xvalues, hi)
            iks.append((i, ik_lo, ik_hi))
            lo_rail.append(np.array(inner[i][ik_lo]))
            hi_rail.append(np.array(inner[i][ik_hi]))
            # cut TYPE bounding the lo / hi chord at this station, so the plot
            # can apply the correct per-cut seam allowance on each rail.
            _, f, b, _, _ = self.region._substrip_at(y)
            fa, ba = f.at(y), b.at(y)
            lo_cut, hi_cut = (f, b) if fa <= ba else (b, f)
            lo_types.append(lo_cut.type)
            hi_types.append(hi_cut.type)

        (i0, ik_lo0, ik_hi0) = iks[0]
        (i1, ik_lo1, ik_hi1) = iks[-1]
        bottom_arc = [np.array(p) for p in inner[i0].get(ik_lo0, ik_hi0)]
        top_arc = [np.array(p) for p in inner[i1].get(ik_lo1, ik_hi1)]
        meta = {
            "lo_types": lo_types,
            "hi_types": hi_types,
            "station_ys": station_ys,  # spanwise y aligned with lo_rail/hi_rail
            # the spanwise ends are rib seams only when they reach rib1 / rib2;
            # an interior crossing (apex) end carries no seam allowance.
            "touches_rib1": abs(y0) < 1e-6,
            "touches_rib2": abs(y1 - 1.0) < 1e-6,
        }
        return lo_rail, hi_rail, bottom_arc, top_arc, meta

    def get_flattened(self, cell, numribs=14, with_numpy=True):
        """Arc-correct 2D sewing contour (``PolyLine2D``) of the developed
        region."""
        from openglider.vector.polyline import PolyLine2D
        lo_rail, hi_rail, bottom_arc, top_arc, _ = self._flatten_boundaries(cell, numribs)
        # closed loop: up the lo rail, across the top arc, down the hi rail,
        # back across the bottom arc.
        contour = (
            list(lo_rail)
            + list(top_arc)
            + list(hi_rail[::-1])
            + list(bottom_arc[::-1])
        )
        return PolyLine2D(_dedup(contour))


def _dedup(points, tol=1e-9):
    """Drop consecutive (and closing) duplicate points from a 2D loop."""
    out = []
    for p in points:
        p = np.asarray(p, float)
        if not out or np.linalg.norm(out[-1] - p) > tol:
            out.append(p)
    while len(out) > 1 and np.linalg.norm(out[0] - out[-1]) < tol:
        out.pop()
    return out
