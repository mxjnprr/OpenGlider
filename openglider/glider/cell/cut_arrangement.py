"""Planar arrangement of panel cuts inside a single cell.

A panel *cut* is a straight line in the cell's ``(y, chord)`` parameter space,
running from ``(0, left)`` at rib1 to ``(1, right)`` at rib2. When two cuts
*cross* inside the cell (at an interior ``y`` that is not on a rib) the ordinary
strip-panel model can no longer tile the cell (see
``docs/crossing-cuts-design.md``). This module turns a list of cuts into the
**regions** bounded by them, so each region can be meshed / flattened as one
piece.

Algorithm (merge of spanwise sub-strips):

1. Find every interior crossing ``y`` between two cuts.
2. Split the span at those ``y`` values into *segments*. Inside a segment no two
   cuts cross, so the cuts have a fixed chord order → the segment is tiled by
   *sub-strips* (bands between consecutive cuts).
3. Merge sub-strips of adjacent segments that share a **positive-width** chord
   interval across the segment boundary. A "pointe" merges through a crossing
   (its band keeps positive width at the apex); the two triangles of an X do not
   merge (their shared band has zero width at the apex) → they stay separate
   regions meeting at the apex point.
4. Each merged component is one region; its boundary polygon is the forward rail
   (following each sub-strip's forward cut) plus the reversed back rail.

The output lives entirely in ``(y, chord)`` parameter space; lifting to 3D /
flattening is the consumer's job.
"""

from __future__ import annotations

EPS = 1e-9


class Cut:
    """A straight cut in (y, chord) space: chord = left + y*(right-left)."""

    __slots__ = ("left", "right", "type", "id")

    def __init__(self, left, right, type="orthogonal", id=None):
        self.left = float(left)
        self.right = float(right)
        self.type = type
        self.id = id

    def at(self, y):
        return self.left + y * (self.right - self.left)

    def __repr__(self):
        return f"Cut(#{self.id} {self.left:+.3f}/{self.right:+.3f} {self.type})"


class Region:
    """A region bounded by cuts, in (y, chord) space.

    boundary : list[(y, chord)] closed CCW-ish polygon (no repeated last point).
    forward_cut / back_cut : the innermost/outermost bounding cut at the widest
        segment (used for material / naming / entry detection). For a merged
        pointe the back boundary bends, so ``back_cut`` is only representative.
    substrips : the (segment_index, forward_cut, back_cut, y_lo, y_hi) tuples that
        compose the region (kept for meshing / debugging).
    """

    def __init__(self, boundary, substrips):
        self.boundary = boundary
        self.substrips = substrips

    @property
    def y_range(self):
        ys = [p[0] for p in self.boundary]
        return min(ys), max(ys)

    def centroid(self):
        # polygon centroid (area-weighted); falls back to vertex mean for
        # degenerate (zero-area) polygons.
        n = len(self.boundary)
        a = cx = cy = 0.0
        for i in range(n):
            y0, x0 = self.boundary[i]
            y1, x1 = self.boundary[(i + 1) % n]
            cross = y0 * x1 - y1 * x0
            a += cross
            cy += (y0 + y1) * cross
            cx += (x0 + x1) * cross
        if abs(a) < 1e-12:
            my = sum(p[0] for p in self.boundary) / n
            mx = sum(p[1] for p in self.boundary) / n
            return my, mx
        a *= 0.5
        return cy / (6 * a), cx / (6 * a)

    def _substrip_at(self, y):
        for s in self.substrips:
            _, _, _, y_lo, y_hi = s
            if y_lo - 1e-9 <= y <= y_hi + 1e-9:
                return s
        # y outside coverage (numerical) → nearest sub-strip
        return min(self.substrips, key=lambda s: min(abs(s[3] - y), abs(s[4] - y)))

    def chord_interval(self, y):
        """(lo, hi) chord positions bounding the region at spanwise ``y``."""
        _, f, b, _, _ = self._substrip_at(y)
        a, c = f.at(y), b.at(y)
        return (a, c) if a <= c else (c, a)

    def segment_ys(self):
        """Spanwise y values at which the region's boundary may kink (sub-strip
        segment boundaries) — must be sampled to capture bent boundaries."""
        ys = set()
        for _, _, _, y_lo, y_hi in self.substrips:
            ys.add(round(y_lo, 9))
            ys.add(round(y_hi, 9))
        return sorted(ys)

    def is_entry(self):
        """True if every sub-strip is an open entry (folded/singleskin between
        two same-type cuts) — no fabric panel there."""
        return all(
            fwd.type == back.type and fwd.type in ("folded", "singleskin")
            for _, fwd, back, _, _ in self.substrips
        )

    def __repr__(self):
        (y0, y1) = self.y_range
        return f"Region(y[{y0:.3f},{y1:.3f}] {len(self.substrips)} substrips, {len(self.boundary)} pts)"


def find_crossings(cuts):
    """Sorted, de-duplicated interior (0<y<1) crossing y-values among cuts."""
    ys = set()
    n = len(cuts)
    for i in range(n):
        a = cuts[i]
        slope_a = a.right - a.left
        for j in range(i + 1, n):
            b = cuts[j]
            slope_b = b.right - b.left
            denom = slope_a - slope_b
            if abs(denom) < EPS:
                continue  # parallel in (y, chord) space
            y = (b.left - a.left) / denom
            if EPS < y < 1.0 - EPS:
                ys.add(round(y, 9))
    return sorted(ys)


class _UnionFind:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, x):
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


def build_regions(cuts):
    """Build the regions of a cell from its cuts.

    ``cuts`` is an iterable of ``Cut`` (or ``{left,right,type}`` dicts). Returns
    ``None`` if there is **no interior crossing** (caller should use the ordinary
    strip path — R6). Otherwise returns a list of ``Region`` covering the cell,
    including entry regions (mark via ``Region.is_entry()``).
    """
    cut_objs = []
    for i, c in enumerate(cuts):
        if isinstance(c, Cut):
            c.id = i if c.id is None else c.id
            cut_objs.append(c)
        else:
            cut_objs.append(Cut(c["left"], c["right"], c.get("type", "orthogonal"), i))

    crossings = find_crossings(cut_objs)
    if not crossings:
        return None  # no crossing → ordinary strip decomposition applies

    bounds = [0.0] + crossings + [1.0]
    segments = list(zip(bounds[:-1], bounds[1:]))

    # Per segment: order cuts by chord at mid-y, list sub-strips (consecutive
    # pairs). substrip = (seg_index, fwd_cut, back_cut, y_lo, y_hi).
    seg_substrips = []
    for si, (y_lo, y_hi) in enumerate(segments):
        y_mid = 0.5 * (y_lo + y_hi)
        ordered = sorted(cut_objs, key=lambda c: c.at(y_mid))
        strips = []
        for k in range(len(ordered) - 1):
            strips.append((si, ordered[k], ordered[k + 1], y_lo, y_hi))
        seg_substrips.append(strips)

    # Flatten with global indices for union-find.
    flat = [s for strips in seg_substrips for s in strips]
    index_of = {id(s): i for i, s in enumerate(flat)}
    uf = _UnionFind(len(flat))

    # Merge sub-strips of adjacent segments that share a positive-width chord
    # interval across the shared segment boundary y.
    for si in range(len(segments) - 1):
        y_b = segments[si][1]  # == segments[si+1][0]
        for s_low in seg_substrips[si]:
            _, f0, b0, _, _ = s_low
            lo0, hi0 = sorted((f0.at(y_b), b0.at(y_b)))
            for s_high in seg_substrips[si + 1]:
                _, f1, b1, _, _ = s_high
                lo1, hi1 = sorted((f1.at(y_b), b1.at(y_b)))
                overlap = min(hi0, hi1) - max(lo0, lo1)
                if overlap > EPS:
                    uf.union(index_of[id(s_low)], index_of[id(s_high)])

    # Group sub-strips into components → regions.
    comps = {}
    for i, s in enumerate(flat):
        comps.setdefault(uf.find(i), []).append(s)

    regions = []
    for members in comps.values():
        members.sort(key=lambda s: s[3])  # by y_lo
        boundary = _region_boundary(members)
        regions.append(Region(boundary, members))
    # deterministic order: by centroid (y, chord)
    regions.sort(key=lambda r: r.centroid())
    return regions


def _region_boundary(substrips):
    """Closed (y, chord) polygon of a vertically-stacked column of sub-strips.

    Forward rail = each sub-strip's forward cut sampled at (y_lo, y_hi);
    back rail = back cut. Polygon = forward rail (up) + back rail (down).
    Consecutive sub-strips share their boundary y-point so the rails are
    continuous; duplicate consecutive points are collapsed.
    """
    fwd = []
    back = []
    for (_, f, b, y_lo, y_hi) in substrips:
        fwd.append((y_lo, f.at(y_lo)))
        fwd.append((y_hi, f.at(y_hi)))
        back.append((y_lo, b.at(y_lo)))
        back.append((y_hi, b.at(y_hi)))
    poly = fwd + back[::-1]
    # collapse consecutive duplicates (within tolerance)
    out = []
    for p in poly:
        if not out or abs(out[-1][0] - p[0]) > 1e-9 or abs(out[-1][1] - p[1]) > 1e-9:
            out.append(p)
    if len(out) > 1 and abs(out[0][0] - out[-1][0]) < 1e-9 and abs(out[0][1] - out[-1][1]) < 1e-9:
        out.pop()
    return out
