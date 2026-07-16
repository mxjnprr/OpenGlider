"""Pure helpers for asymmetric panel decoupe.

These convert between the FreeCAD design tool's per-cut-line view and the stored
``elements["cuts"]`` format (half-cell index + optional ``"side"`` key consumed
by :meth:`ParametricGlider.get_panels`).  They contain **no FreeCAD imports** so
the correctness-critical logic can be unit-tested headless.

Storage convention (matches the engine, verified against copy_complete):
  * a cut dict has ``"cells": [half_cell_index, ...]`` and, optionally,
    ``"side": "left" | "right"``.  Absent/"both" = mirrored onto both wings.
  * ``"left"``/``"right"`` chord positions are always inner (centre-side) /
    outer (tip-side); the engine mirrors the left wing automatically.
"""


def wing_of_x(x, eps=1e-9):
    """Physical wing a planform x-coordinate belongs to."""
    if x < -eps:
        return "left"
    if x > eps:
        return "right"
    return "both"


def finalize_cut_lines(line_dicts):
    """Merge per-cut-line dicts into the stored ``elements["cuts"]`` list.

    Each input dict is single-cell:
        {"cells": [cell], "left": l, "right": r, "type": t, "side": s}
    with ``s`` in {"both", "left", "right"}.

    Steps:
      1. collapse - a left+right pair with identical (cell, left, right, type)
         becomes one "both" cut, so a symmetric paint/design stays symmetric.
      2. merge - contiguous cells sharing (type, left, right, side) collapse to
         one dict with several cells (mirrors the legacy merge).
      3. strip - the neutral "both" marker is removed so legacy/symmetric files
         are byte-for-byte unchanged and ``is_asymmetric`` stays False.
    """
    from collections import defaultdict

    # 1. collapse identical left+right into "both", per (cell, geometry)
    sides_by_key = defaultdict(set)
    order = []
    for d in line_dicts:
        key = (d["cells"][0], d["left"], d["right"], d["type"])
        if key not in sides_by_key:
            order.append(key)
        sides_by_key[key].add(d.get("side", "both"))

    collapsed = []
    for key in order:
        cell, left, right, cut_type = key
        sides = sides_by_key[key]
        if "both" in sides or {"left", "right"} <= sides:
            out_sides = ["both"]
        else:
            out_sides = sorted(sides)  # a lone "left" or "right"
        for side in out_sides:
            collapsed.append(
                {"cells": [cell], "left": left, "right": right,
                 "type": cut_type, "side": side}
            )

    if not collapsed:
        return []

    # 2. merge contiguous cells with matching geometry+side
    collapsed.sort(key=lambda c: c["right"])
    merged = [collapsed[0]]
    for cut in collapsed[1:]:
        last = merged[-1]
        same = (cut["type"], cut["left"], cut["right"], cut["side"]) == (
            last["type"], last["left"], last["right"], last["side"])
        if same:
            last["cells"].append(cut["cells"][0])
        else:
            merged.append(cut)

    # 3. strip the neutral marker
    for cut in merged:
        if cut.get("side") == "both":
            cut.pop("side", None)
    return merged


def signed_ribs_for(cell, side, has_center):
    """(inner_signed, outer_signed) rib indices for a (cell, side) placement.

    Signed ribs encode the full-span canvas: +n = right wing, -n = left wing,
    |n| grows towards the tip.  The single cross-centre pair (-1, +1) is the
    centre cell of an odd glider.
    """
    if has_center:
        if cell == 0:
            return (-1, 1)
        if side == "left":
            return (-cell, -(cell + 1))
        return (cell, cell + 1)
    # even cell count: no centre cell
    if side == "left":
        return (-(cell + 1), -(cell + 2))
    return (cell + 1, cell + 2)


def resolve_signed_pair(r1, r2, has_center):
    """Resolve a full-span cut line between two signed ribs.

    Returns ``(inner_is_r1, cell, wing)`` where the inner (centre-side) rib's
    chord position becomes the stored ``"left"`` (the engine mirrors the left
    wing, so inner==centre-side holds for both wings).
    """
    off = 0 if has_center else 1
    if (r1 < 0) != (r2 < 0):
        # centre cell (odd gliders): the -x0 rib is rib1 / inner
        return (r1 < 0), 0, "both"
    if r1 > 0:  # right wing: inner = smaller signed rib
        return (r1 < r2), min(r1, r2) - off, "right"
    # left wing: inner = rib closest to centre (larger signed, e.g. -1 > -2)
    return (r1 > r2), min(abs(r1), abs(r2)) - off, "left"


def expand_cut_placements(cut, has_center):
    """Yield ``(cell, side)`` canvas placements for one stored cut dict.

    A symmetric ("both") cut is drawn on both wings (so the user sees the whole
    span and can diverge one side), except the single centre cell which is drawn
    once.  A side-tagged cut is drawn only on its wing.
    """
    side = cut.get("side", "both")
    for cell in cut["cells"]:
        if side == "both":
            if has_center and cell == 0:
                yield (cell, "both")
            else:
                yield (cell, "left")
                yield (cell, "right")
        else:
            yield (cell, side)
