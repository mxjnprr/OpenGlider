"""Integration tests: crossing cuts -> PolygonPanel regions (3D mesh + flatten)."""
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import openglider
from openglider.glider.cell.elements import Panel
from openglider.glider.cell.polygon_panel import PolygonPanel

DEMOKITE = os.path.join(os.path.dirname(__file__), "common", "demokite.json")
CROSS_CUTS = [
    {"left": -0.3031669129322903, "right": -0.4095345223836025, "type": "orthogonal"},
    {"left": -0.4315038450197704, "right": -0.22565283792995963, "type": "orthogonal"},
]
CELL = 2


def _glider_with_crossing():
    g2d = openglider.load(DEMOKITE)
    g2d.elements.setdefault("cuts", [])
    for c in CROSS_CUTS:
        g2d.elements["cuts"].append(dict(c, cells=[CELL]))
    return g2d.get_glider_3d()


def _nonmanifold(cell):
    pos = {}
    faces = []

    def key(p):
        return (round(float(p[0]), 5), round(float(p[1]), 5), round(float(p[2]), 5))

    for p in cell.panels:
        m = p.get_mesh(cell, 0, with_numpy=True)
        v, groups, _ = m.get_indexed()
        v = [np.array([x.x, x.y, x.z]) for x in v]
        for _, fs in groups.items():
            for f in fs:
                idx = list(f)
                for k in range(1, len(idx) - 1):
                    tri = [idx[0], idx[k], idx[k + 1]]
                    gi = [pos.setdefault(key(v[t]), len(pos)) for t in tri]
                    if len(set(gi)) == 3:
                        faces.append(gi)
    edge = defaultdict(int)
    for f in faces:
        for a, b in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])]:
            edge[frozenset((a, b))] += 1
    return sum(1 for c in edge.values() if c > 2), len(faces)


def _self_intersections(contour):
    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def cross(p, p2, q, q2):
        d = [cr(q, q2, p), cr(q, q2, p2), cr(p, p2, q), cr(p, p2, q2)]
        return ((d[0] > 0) != (d[1] > 0)) and ((d[2] > 0) != (d[3] > 0))

    n = len(contour)
    hits = 0
    for i in range(n):
        a, a2 = contour[i], contour[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or j == (i + 1) % n or (j + 1) % n == i:
                continue
            b, b2 = contour[j], contour[(j + 1) % n]
            if cross(a, a2, b, b2):
                hits += 1
    return hits


def test_r6_non_crossing_all_strip_panels():
    g2d = openglider.load(DEMOKITE)
    cells = g2d.get_panels()
    assert all(isinstance(p, Panel) for cell in cells for p in cell)


def test_crossing_cell_builds_polygon_panels():
    cell = _glider_with_crossing().cells[CELL]
    polys = [p for p in cell.panels if isinstance(p, PolygonPanel)]
    assert len(polys) >= 4  # >=2 pointes + 2 triangles (+ entry-bounded pieces)
    assert all(not isinstance(p, Panel) for p in cell.panels)


def test_crossing_cell_mesh_watertight():
    g3 = _glider_with_crossing()
    # meshing every cell must not crash (mirrors the FreeCAD viewer)
    for cell in g3.cells:
        for p in cell.panels:
            p.get_mesh(cell, 0, with_numpy=True)
    nm, ntris = _nonmanifold(g3.cells[CELL])
    assert ntris > 0
    assert nm == 0, f"{nm} non-manifold (overlap/crack) edges in crossing cell"


def test_crossing_cell_flatten_clean():
    cell = _glider_with_crossing().cells[CELL]
    for p in cell.panels:
        if not isinstance(p, PolygonPanel):
            continue
        contour = [np.asarray(pt, float) for pt in np.array(p.get_flattened(cell))]
        assert _self_intersections(contour) == 0, f"{p.name} flatten self-intersects"


def test_nose_wrapping_region_watertight_and_dense():
    # a cell with NO entry cut: the region between the aft cut and the intrados
    # TE wraps around the nose (extrados -> LE -> intrados). It must be sampled
    # at the profile's native resolution (not a fixed few points) and stay
    # watertight, otherwise the mesh folds through itself.
    from openglider.glider.cell.cut_arrangement import build_regions

    g2d = openglider.load(DEMOKITE)
    cell_cuts = [
        {"left": -1.0, "right": -1.0, "type": "parallel"},
        *CROSS_CUTS,
        {"left": 1.0, "right": 1.0, "type": "parallel"},
    ]
    g2d.elements["cuts"] = [dict(c, cells=[CELL]) for c in cell_cuts]
    cell = g2d.get_glider_3d().cells[CELL]
    regions = [r for r in build_regions(cell_cuts) if not r.is_entry()]
    mesh_ys = sorted(set([round(i / 24.0, 9) for i in range(25)] + [0.411]))
    panels = [PolygonPanel(r, "ff8800", f"r{i}", mesh_ys=mesh_ys)
              for i, r in enumerate(regions)]
    # widest region must be densely sampled (native profile res, not ~6/row)
    widest = max(panels, key=lambda p: p.region.chord_interval(0.5)[1] - p.region.chord_interval(0.5)[0])
    span = widest.region.chord_interval(0.5)[1] - widest.region.chord_interval(0.5)[0]
    assert span > 1.0, "test setup: expected a nose-wrapping region"
    assert len(widest.get_mesh(cell, 0, with_numpy=True).vertices) > 1000, (
        "nose-wrap region under-sampled (mesh would fold)"
    )

    # combined watertightness of all regions
    cell.panels = panels
    nm, _ = _nonmanifold(cell)
    assert nm == 0, f"{nm} non-manifold edges in nose-wrapping crossing cell"


def test_plotmaker_runs_with_crossing():
    from openglider.plots import PlotMaker

    g3 = _glider_with_crossing()
    PlotMaker(g3).get_panels()  # must not raise


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS", name)
    print("all ok")
