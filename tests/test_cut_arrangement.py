"""Tests for the per-cell cut arrangement / region builder."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from openglider.glider.cell.cut_arrangement import Cut, build_regions, find_crossings


def _pos(cut, y):
    return cut["left"] + y * (cut["right"] - cut["left"])


def _region_interval_at(region, y):
    """chord interval [lo, hi] the region covers at spanwise y, or None."""
    y0, y1 = region.y_range
    if not (y0 - 1e-9 <= y <= y1 + 1e-9):
        return None
    # find the sub-strip covering this y
    for (_, f, b, y_lo, y_hi) in region.substrips:
        if y_lo - 1e-9 <= y <= y_hi + 1e-9:
            lo, hi = sorted((f.at(y), b.at(y)))
            if hi - lo > 1e-9:
                return (lo, hi)
    return None


def _assert_partition(regions, cuts, chord_min=-1.0, chord_max=1.0):
    """At many y, the non-degenerate region intervals must tile [min,max] with
    no overlap and full coverage."""
    for k in range(1, 40):
        y = k / 40.0
        ivs = sorted(
            iv for r in regions if (iv := _region_interval_at(r, y)) is not None
        )
        # no overlap
        for a, b in zip(ivs, ivs[1:]):
            assert a[1] <= b[0] + 1e-6, f"overlap at y={y}: {a} & {b}"
        # full coverage
        assert ivs, f"no coverage at y={y}"
        assert abs(ivs[0][0] - chord_min) < 1e-6, f"gap at chord_min y={y}: {ivs[0]}"
        assert abs(ivs[-1][1] - chord_max) < 1e-6, f"gap at chord_max y={y}: {ivs[-1]}"
        for a, b in zip(ivs, ivs[1:]):
            assert abs(a[1] - b[0]) < 1e-6, f"interior gap at y={y}: {a}->{b}"


def _cell22_cuts():
    # real cell-22 X (extrados) + implicit TE up/down. No entry cut (closed cell).
    return [
        {"left": -1.0, "right": -1.0, "type": "parallel"},
        {"left": -0.3031669129322903, "right": -0.4095345223836025, "type": "orthogonal"},
        {"left": -0.4315038450197704, "right": -0.22565283792995963, "type": "orthogonal"},
        {"left": 1.0, "right": 1.0, "type": "parallel"},
    ]


def test_no_crossing_returns_none():
    cuts = [
        {"left": -1.0, "right": -1.0, "type": "parallel"},
        {"left": -0.5, "right": -0.5, "type": "orthogonal"},
        {"left": 1.0, "right": 1.0, "type": "parallel"},
    ]
    assert build_regions(cuts) is None


def test_x_crossing_yields_four_regions():
    cuts = _cell22_cuts()
    xs = find_crossings([Cut(c["left"], c["right"], c["type"], i) for i, c in enumerate(cuts)])
    assert len(xs) == 1, xs
    y_cross = xs[0]
    assert 0.40 < y_cross < 0.42, y_cross

    regions = build_regions(cuts)
    assert regions is not None
    non_entry = [r for r in regions if not r.is_entry()]
    # 2 pointes (full span) + 2 triangles (half span) = 4 fabric regions
    assert len(non_entry) == 4, [str(r) for r in non_entry]

    full_span = [r for r in non_entry if abs(r.y_range[1] - r.y_range[0] - 1.0) < 1e-6]
    half = [r for r in non_entry if r.y_range[1] - r.y_range[0] < 0.99]
    assert len(full_span) == 2, "expected 2 pointes spanning the whole cell"
    assert len(half) == 2, "expected 2 triangles spanning up to the crossing"
    # triangles converge: one goes [0, y_cross], the other [y_cross, 1]
    spans = sorted(r.y_range for r in half)
    assert abs(spans[0][0] - 0.0) < 1e-6 and abs(spans[0][1] - y_cross) < 1e-6, spans
    assert abs(spans[1][0] - y_cross) < 1e-6 and abs(spans[1][1] - 1.0) < 1e-6, spans


def test_x_partition_no_overlap_full_coverage():
    cuts = _cell22_cuts()
    regions = build_regions(cuts)
    _assert_partition(regions, cuts)


def test_pointe_boundary_bends_at_crossing():
    """A full-span pointe must have >4 boundary vertices (its back edge bends at
    the apex), proving it is a single region with a bent boundary (not a strip)."""
    cuts = _cell22_cuts()
    regions = [r for r in build_regions(cuts) if not r.is_entry()]
    pointes = [r for r in regions if abs(r.y_range[1] - r.y_range[0] - 1.0) < 1e-6]
    for p in pointes:
        # rib1(2) + rib2(2) + apex bend => at least 5 distinct vertices
        assert len(p.boundary) >= 5, f"pointe not bent: {len(p.boundary)} pts"


def test_near_rib_crossing_partition():
    # crossing lands near a rib (y ~ 0.1) — still must partition cleanly
    cuts = [
        {"left": -1.0, "right": -1.0, "type": "parallel"},
        {"left": -0.55, "right": -0.5, "type": "orthogonal"},
        {"left": -0.5, "right": -0.9, "type": "orthogonal"},
        {"left": 1.0, "right": 1.0, "type": "parallel"},
    ]
    xs = find_crossings([Cut(c["left"], c["right"], c["type"], i) for i, c in enumerate(cuts)])
    assert len(xs) == 1 and 0.05 < xs[0] < 0.2, xs
    regions = build_regions(cuts)
    _assert_partition(regions, cuts)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS", name)
    print("all ok")
