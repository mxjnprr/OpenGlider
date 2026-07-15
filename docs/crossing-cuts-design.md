# Design: panel cuts that cross inside a cell

Status: **draft — synthesized from 4-agent brainstorm (2026-07-15)**
Branch: `feature/crossing-cuts-interior-apex`

## 1. Problem

In the Design tool a user can draw two (or more) panel cuts that **cross inside a
single cell** — e.g. an "X". The crossing point lies at an interior spanwise
position `y_cross ∈ (0,1)` and an interior chord position, **not on a rib**.

The user wants the cell split into the regions bounded by the cuts — for a simple
X, **4 pieces: 2 triangles + 2 "pointes"** — where the **only seams are the cut
lines** (plus the pre-existing cell seams: LE entry, TE, ribs). No extra seam.

**Ground truth = the flattened sewing patterns (mise à plat), not the 3D view.**
A decomposition is only valid if the 2D patterns are correct (no extra seam, no
self-intersection, clean contours).

## 2. Why the current model can't do it

An OpenGlider `Panel` (`openglider/glider/cell/elements.py:774-1057`) is a **strip**
from rib1 to rib2, defined by `cut_front`/`cut_back` (`{left,right}` chord
positions) interpolated **linearly along the span** (`x = left + y*(right-left)`).
`get_3d`, `get_mesh`, `_get_ik_values` all compute **one front-ik and one back-ik
per spanwise row** — so a boundary that *switches which cut it follows* at an
interior `y` is structurally inexpressible.

A crossing introduces two things the strip model cannot express:
1. an **interior apex vertex** (the crossing point), shared by all regions;
2. a **bent boundary**: a "pointe" is bounded by cutA on one side of the crossing
   and cutB on the other.

`get_panels` already *silently mis-tiles* crossings today: it pairs consecutive
cuts with `ZipCmp` after sorting by mean chord and explicitly `pass`es on crossing
pairs (`glider.py:2103-2109`) → **users get wrong patterns with no warning.**

Two fallbacks tried on `develop` (reverted), both wrong:
- **Segment split** (split the whole cell spanwise at `y_cross`): watertight in 3D
  but adds an unwanted lengthwise seam; flattened patterns crenellated/self-
  intersecting.
- **Surgical bow-tie split** (split only the crossing strip; keep neighbours
  full-span): no extra seam, but the pointes **overlap** past the crossing because
  cutA/cutB swap chord order.

## 3. Requirements

- R1. A crossing yields exactly the regions bounded by the cuts (4 for an X),
  sharing the crossing point as a vertex.
- R2. **No extra seam** beyond the cut lines and existing cell seams.
- R3. Correct **3D mesh**: watertight, no overlap, no T-junction crack, at any
  midrib count (incl. default `midribs=0`).
- R4. Correct **2D flattening**: each region one fabric piece, clean non-self-
  intersecting contour, seam allowance correct, **shared cut edge same length on
  both neighbours** (else pieces don't sew).
- R5. Generalise: polylines, ≥2 crossings/cell, crossing near rib/LE/TE.
- R6. No regression of the non-crossing case (byte-identical output).
- R7. Materials/colours per region; stable naming for color tool + pattern export.

## 4. Key finding: the hard numerics ALREADY exist

The brainstorm's strongest, cross-confirmed result: **no new numerical solver is
needed.** The scary parts (isometric surface development, constrained meshing of a
non-strip region) are already implemented and already applied to non-strip pieces:

| Need | Existing, reusable building block |
|---|---|
| Mesh an arbitrary parametric `(y,chord)` region, lifted to the ballooned surface | **`DiagonalRib.get_mesh`** (`elements.py:610-645`): builds a PSLG in param space → `triangulate.Triangulation(pts, boundaries, holes)` → lifts every 2D point (incl. Steiner) to 3D via the same bilinear/`midrib` map. Generalise from quad → arbitrary polygon. |
| Develop a non-strip region into a 2D sewing piece, length-preserving | **`LeadingEdgeClosure.get_flattened` / `get_flattened_plotpart`** (`elements.py:1419-1546`) and **`Cell.get_flattened_cell`→`get_point`** (`cell.py:551-638`): isometric two-rail / law-of-cosines developer, already handling a non-full-span piece **with seam allowance** (`add_stuff`). Also `flatten_list`/`point2d` (`vector/projection.py:39-89`), used by `DiagonalRib.get_flattened`. |
| Param→3D point | `cell.midrib(y)[get_x_value(xvalues, x_chord)]` — cached (`cell.py:296`), handles ballooning + miniribs. |
| Constrained Delaunay w/ holes + boundary preservation | `openglider/mesh/triangulate.py` `Triangulation` (use the **`Y` option** to forbid Steiner points on shared edges). |
| Seam allowance / cut-corner handling | `PolyLine2D.add_stuff`, `openglider/plots/cuts.py` (`DesignCut`, `robust_cut`). |

**Avoid** `openglider/mesh/least_square_conformal_mapping.py` (LSCM): dead py2 code
using removed numpy APIs (`np.complex`, `np.bmat`), conformal (area-distorting),
and gives **no** shared-edge length matching → keep only as a research fallback.

So **the real new work is geometry + glue, not math:**
- a **per-cell 2D planar arrangement** turning crossing cuts into region polygons;
- a **canonical shared-edge sampler** (one 3D polyline per cut half-edge / rib
  edge, handed to *both* neighbouring regions → bit-identical vertices → watertight
  mesh AND matching flattened seam lengths, for free);
- **integration** (data model, materials, naming, plots).

## 5. Chosen direction

### 5.1 Region representation — new `PolygonPanel`, not an extended `Panel`
A distinct `PolygonPanel` class alongside `Panel` (do **not** add a nullable field
to `Panel`). Rationale: keeps the strip path byte-identical (R6), old files never
contain the new type so serialization stays back-compat (`jsonify` reconstructs
`Class(**data)`, `jsonify/__init__.py:59-68`), and enables clean `isinstance`
guards at the few incompatible call-sites instead of branching everywhere.

Both types implement a shared duck-typed interface used by consumers
(`for panel in cell.panels`): `get_mesh(cell, numribs, with_numpy)`,
`get_flattened(cell, …)`, `mean_x()`, `is_lower()`, `mirror()`, `material_code`,
`name`, `__json__`. Sketch:

```python
class PolygonPanel:
    def __init__(self, vertices, material_code="", name="unnamed", crossing_nodes=None):
        self.vertices = vertices          # list[(y, chord)], CCW, in param space
        self.crossing_nodes = crossing_nodes or []
        self.material_code = material_code or ""
        self.name = name
    def __json__(self):                    # self-contained (store absolute verts)
        return {"vertices": self.vertices, "material_code": self.material_code,
                "name": self.name, "crossing_nodes": self.crossing_nodes}
    def mean_x(self): ...                   # area-centroid chord (method, like Panel)
    def is_lower(self): return self.mean_x() > 0
    def mirror(self): ...                    # negate chord of each vertex
    def get_mesh(self, cell, numribs=0, with_numpy=False): ...   # §5.3
    def get_flattened(self, cell, ...): ...                       # §5.4
```

### 5.2 Where crossings are handled — a pure pre-pass inside `get_panels`
Per cell, branch on detection so the common path is literally the old code:

```
elements["cuts"] (per cell)
   └─ detect_crossings(cuts)
        ├─ none → <existing ZipCmp strip-Panel block, verbatim>     # R6 no-op
        └─ crossing → build_cell_arrangement(cuts) → [RegionPolygon] → PolygonPanel(...)
                                                             ↓
                                          cell.panels  (mixed strip + polygon)
```
The arrangement is **per-cell and local** (cuts are stored per-cell). Keep it in
the parametric layer (`ParametricGlider.get_panels`), region geometry in `(y,
chord)` param space (matches `Panel` and `DiagonalRib` hole contours).

### 5.3 3D meshing (per region) — generalise `DiagonalRib.get_mesh`
For each region: boundary as a closed param-space polyline (rib slices at
y∈{y_start,y_end} + cut half-edges + apex node). Densify each boundary edge via the
**canonical shared-edge sampler**. Triangulate with `Triangulation(..., Y-option)`
so Steiner points stay interior. Lift every returned `(y,x)` → 3D via
`cell.midrib(y)[get_x_value(xvalues, x)]`. One poly-group `"panel_"+material_code`
per region → **the FreeCAD render path is unchanged** (`draw_glider` just iterates
`cell.panels` and colours by `material_code`, `glider.py:635-641`). Correct at
`midribs=0` because triangulation is param-driven, not rib-count-driven (sidesteps
the flagged `Panel.get_mesh` `numribs=0` bug, `elements.py:889`).

Watertightness by construction: single shared apex vertex + identical shared-edge
samples + `Y` option; optional belt-and-braces `Mesh.delete_duplicates` on named
boundary nodes (`mesh.py:509`). Include any minirib `y` in the edge sampler to
avoid kinks.

### 5.4 Flattening (per region) — reuse the isometric developer
Develop each region with `LeadingEdgeClosure.get_flattened`'s length-preserving
walk (or `flatten_list`) over two rails traced across the region (a rail may switch
cutA→cutB at `y_cross`; collapse a rail to the apex for a triangle — the developer
already guards `l_base < 1e-10`, `elements.py:1450-1454`). **Sample each shared cut
once and feed the identical 3D points to both neighbours** → arc-length preserved →
seam lengths match automatically (the property LSCM lacks). Seam allowance via the
existing `DesignCut`/`add_stuff` path, per boundary edge with its allowance type.
Fallback for ≥5-sided regions (≥2 crossings): triangulate + fan-develop with the
same `get_point` law-of-cosines, pinning shared edges to neighbour samples.

### 5.5 Materials, naming, back-compat
- **Colour channel = `materials_by_name`** (extend the existing `le_panel_splits`
  escape hatch, `glider.py:2158-2177`); keep positional `materials` list only for
  legacy strip panels.
- **Deterministic region names** from geometry, not creation order: e.g.
  `c{cell}p{base}r{k}` with `k` from sorting regions by `(centroid_y, centroid_x)`,
  so names are stable across reload (→ `materials_by_name` survives save/load).
- **`rename_panels` must skip / not clobber `PolygonPanel` names** (it sorts by
  `mean_x()` and overwrites `.name`, `cell.py:119-124`; two X-triangles share a
  chord centroid → collisions). Primary regression risk.
- `elements["cuts"]` schema **unchanged** — crossings are *derived*, no migration.
- Old files: no `PolygonPanel` blobs → deserialize exactly as today.

### 5.6 Call-sites needing an `isinstance(panel, Panel)` guard
Consumers that read `.cut_front`/`.cut_back` will `AttributeError` on regions —
guard these (primary integration cost):
- `Cell.get_connected_panels` (`cell.py:176-177`), `Panel.__add__`
  (`elements.py:833-843`) — regions can't merge.
- `Cell.calculate_3d_shaping` (`cell.py:670-687`) — **regions opt out of 3D
  shaping in v1** (crossing cuts are a design feature, not a 3d-cut feature).
- `PanelRigidFoil.get_flattened` iterating `cell.panels` (`elements.py:1235-1238`).

### 5.7 Killed alternatives (record so they aren't re-litigated)
- **Insert a full rib at the crossing** → splits fabric = **adds a seam** (R2 fail),
  and changes the structural/aero model.
- **Insert a minirib at the crossing** → drawn as *marks* on continuous fabric
  (`plots/glider/cell.py:474-533`), so it does **not** split into regions (R1 fail),
  and is partial-chord (can't reach LE/TE).
- **Snap the crossing onto a rib** → moves the apex off where the user drew it
  (R1 fail), silent authoring lie.
- **Mesh-whole-cell-then-classify (for 3D)** → seam becomes a staircase unless the
  cut is inserted as a constraint (⇒ collapses into §5.3); fails at `midribs=0`.
  Keep only as a **test oracle**.

## 6. Test strategy (learned: centroid heuristics give false positives — don't use them as the check)
- **Watertight**: combine region meshes, dedup vertices (~1e-9), build undirected
  edge multiset — assert every interior edge shared by exactly 2 faces, outer-
  boundary edges by exactly 1 (count 1 interior = crack; ≥3 = overlap).
- **Apex incidence**: the apex coordinate appears as one shared vertex in all
  incident regions after dedup.
- **No-overlap (param space, exact)**: for each sample `y`, region chord intervals
  `[x_front(y), x_back(y)]` partition the cell extent with disjoint interiors.
- **Area conservation**: Σ region 3D areas == un-split region area.
- **Flatten validity**: each region contour non-self-intersecting; shared cut edge
  identical length in both neighbours.
- **R6 regression**: non-crossing cell → new path not taken, mesh identical to
  `Panel.get_mesh`.
- **Oracle**: mesh-whole-cell-classify region areas vs constrained-triangulation
  areas.

## 7. Highest-value spike — DONE ✅ (2026-07-15), result: CLEAN
Fed real cell-22 cuts through the isometric developer `flatten_list`
(`vector/projection.py:39`) for all four regions, both a **bent boundary** (pointe:
aft rail = `min(cutA,cutB)` switching cut at the apex) and a **converging boundary**
(triangle: rails meet at the apex). Result (spike script
`scratchpad/spike_flatten.py`, run under pixi/numpy 2.2):

| region | boundary | self-intersections | NaN | 2D area | 2D width |
|---|---|---|---|---|---|
| A pointe (TE_up) | bent | 0 | no | 0.145 | 0.52–0.59 |
| D pointe (TE_low) | bent | 0 | no | 0.167 | 0.59–0.68 |
| B triangle (y<apex) | converging | 0 | no | 0.007 | 0.117→0.000 |
| C triangle (y>apex) | converging | 0 | no | 0.013 | 0.000→0.160 |

**Verdict: the existing isometric developer handles bent AND converging region
boundaries with clean, non-self-intersecting contours.** The feature is therefore
"**reuse developer + build region polygons + shared-edge sampling + integration**",
NOT "new flattener". Shared-edge seam-length matching (R4) is free by construction:
region A's aft rail and triangle B share the same `min(cut)` trait sampled at the
same `y`s → identical 3D points → identical developed lengths (isometry).

Caveats: the spike exercised the core developer directly (not the full
`PanelPlot.flatten` envelope + seam-allowance path), and used the simple 2-cut
`min/max` arrangement. Both are integration details; the *numerical* unknown is
resolved. Remaining flatten risk = wiring seam allowance + the general arrangement,
not the development math.

## 8. Incremental, testable plan
0. **Spike** (§7) — de-risk the flattener reuse.
1. **Crossing detector + warning** in `get_panels` (replaces the silent `pass` at
   `glider.py:2103-2109`). Pure, unit-testable; ships value immediately (stops
   silent pattern corruption) and is the arrangement's entry point.
2. **Per-cell 2D arrangement** → region polygons (pure data; param-interval &
   area tests). Must fast-path to identical strips when no crossing (R6).
3. **`PolygonPanel.get_mesh`** (generalise `DiagonalRib.get_mesh`) + canonical
   shared-edge sampler; watertight/apex/no-overlap tests; render path unchanged.
4. **`PolygonPanel.get_flattened`** (reuse isometric developer) + seam allowance;
   contour-validity & shared-edge-length tests.
5. **Integration**: `materials_by_name` + deterministic naming, `isinstance`
   guards (§5.6), color-tool/plot wiring, `_remap` colour carry (note latent bug:
   `_remap` doesn't remap `materials_by_name`, `glider.py:1488-1524`).

## 9. Open questions still to resolve
- Q-spike (§7) outcome → confirms flattener path.
- Rail-tracing for regions with ≥2 crossings (5+ sides): two-rail switch vs
  triangulate+fan — pick after the spike.
- Authoring UX: should the Design tool visualise the crossing node / warn on
  interior crossings, or fully support arbitrary crossings silently? (`design_tool.py`)
- `_remap` / `rename_panels` latent bugs (mean_x property-vs-method at
  `cell.py:104-106`; `materials_by_name` not remapped) — fix deliberately as part
  of step 5, not incidentally.

## 10. Appendix — key code references
`elements.py`: `Panel` 774-1057, `PanelCut` 752-771, `DiagonalRib.get_mesh`
610-645, `LeadingEdgeClosure.get_flattened(_plotpart)` 1419-1546. `cell.py`:
`midrib` 296, `get_flattened_cell`/`get_point` 551-638, `rename_panels` 102-124,
`get_connected_panels` 170-183, `calculate_3d_shaping` 640-688. `parametric/
glider.py`: `get_panels`/`ZipCmp` 2047-2187, materials/naming 2118-2127,
le_panel_splits+`materials_by_name` 2131-2183, `_remap` 1488-1525.
`mesh/triangulate.py` `Triangulation`; `mesh/mesh.py` `delete_duplicates` 509,
`from_indexed`; `vector/projection.py` `flatten_list`/`point2d` 39-89.
`plots/glider/cell.py` `PanelPlot.flatten` 118-171 (spitzer-schnitt 136-159),
minirib marks 474-533. `plots/cuts.py` `DesignCut`. `jsonify/__init__.py` 26-31,
59-68. `freecad/glider/tools/{design_tool,color_tool,glider}.py`.
