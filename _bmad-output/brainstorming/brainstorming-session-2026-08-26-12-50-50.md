---
stepsCompleted: [1, 2, 3, 4]
inputDocuments: []
session_topic: 'Wing twist (vrillage) feature in OpenGlider: automatic optimum twist + alternative "fixed absolute angle, move ribs / planform" mode with preview'
session_goals: '1) Define what an optimum twist criterion is (constant AoA vs elliptic lift vs stall safety/washout) and how to apply it automatically. 2) Explore the alternative mode: keep absolute rib angle, move ribs (sweep/planform) so they hang correctly relative to the pilot, preview resulting planform. 3) Map couplings with panel method, shape, lines and the in-progress trimming tool.'
selected_approach: 'progressive-flow'
techniques_used: ['First Principles Thinking', 'What If Scenarios', 'Assumption Reversal', 'Constraint Mapping', 'Morphological Analysis', 'Decision Tree Mapping']
ideas_generated: [42]
context_file: ''
---

# Brainstorming Session Results

**Facilitator:** Max
**Date:** 2026-08-26

## Session Overview

**Topic:** Wing twist (vrillage) feature in OpenGlider
**Goals:** Define an "optimum twist" criterion and auto-apply it; explore a planform-moving alternative (fixed absolute angle, ribs relocated relative to pilot) with preview; map couplings (panel method, shape, lines, trimming tool).

### Context Guidance

Current state: twist only exists implicitly through the AoA spline (`ParametricGlider.aoa`). Per rib: `aoa_relative = aoa_absolute + arctan(cos(arcang)/glide)`. The AoA tool shows red = relative (user-defined), blue = absolute (geometric). No explicit twist entity, no lift-distribution feedback loop, no coupling with panel method or lines.

### Session Setup

Fresh session. Approach: Progressive Technique Flow (broad divergence → systematic narrowing). Session conducted in French, document in English.

## Technique Selection

**Approach:** Progressive Technique Flow
**Journey Design:** Systematic development from exploration to action

**Progressive Techniques:**

- **Phase 1 - Exploration:** First Principles Thinking + What If Scenarios — decompose "twist" to what a rib actually sees in flight; 30+ raw ideas
- **Phase 2 - Pattern Recognition:** Assumption Reversal + Constraint Mapping — invert the implicit assumptions of the current model (AoA spline, fixed arc, single global glide, planar ribs)
- **Phase 3 - Development:** Morphological Analysis — grid {optimum criterion} × {adjusted variable} × {data source} × {UI mode}
- **Phase 4 - Action Planning:** Decision Tree Mapping — MVP, dependencies (trimming tool, panel method), success criteria

**Journey Rationale:** The topic mixes flight physics, a parametric model with hidden assumptions, and tool UX. Starting from first principles avoids anchoring on the existing AoA curve; assumption reversal targets the model's structural choices; morphological analysis forces combinations across physics/data/UI axes before committing to a plan.

## Technique Execution Results

### Phase 1 — First Principles Thinking + What If Scenarios

**Interactive focus:** what a rib really "sees" in flight; the kite analogy (line angle vs chord/wind angle); the vertical tip rib limit case; pilot ≠ point (riser spacing); twist as input vs output; planform as second degree of freedom.

**Key breakthroughs:**
- Two angles, two roles: α (chord/wind) sets lift; β (chord/line) sets local moment. Twist alone can only satisfy one.
- Lift points radially **outward** from the arc centre; lines pull inward. Tip lift keeps tip lines taut (no tip lift → tip folds). When pilot = arc centre, each rib is self-balanced by its line and fabric carries no spanwise tension; the pilot/arc-centre offset is therefore a fabric-tension setting.
- Planform sweep (δx per rib, chord fixed) is the second degree of freedom to align the line cone with the resultant without touching α. δx and δα are linked by line length L (1° ≈ L·tan 1°).

**Ideas captured:**

- **[Physics #1] Two angles, two roles** — α (chord/wind) sets lift, β (chord/line) sets local moment; twist can only satisfy one. Ends the "constant or not" debate.
- **[Physics #2] Misalignment becomes A/B/C load** — the rib isn't free; resultant/line-cone mismatch shows up as A/B/C load split and internal torsion, computable per rib (already near-available via attachment-force distribution).
- **[Physics #3] Planform as 2nd DoF** — moving the rib realigns the line cone on the resultant without touching α; planform becomes an output of an equilibrium.
- **[Physics #4] In-rib-plane wind is shallower at the tip** — cos(arcang) projection: same α gives a different resultant direction in rib plane at centre vs tip.
- **[Physics #5] Lift = carry + tension** — L·cos(arcang) carries the pilot, L·sin(arcang) loads the tip lines (outward) and keeps the arc open.
- **[Physics #6] Arc + twist + lines = one equilibrium** — same solver family as the trimming tool, different free variable.
- **[Physics #7] Optimum twist is a 3-term compromise** — induced drag / stall safety (washout) / tip line tension. Suggests priority sliders rather than an "optimum" button.
- **[Physics #8] Tip as fin** — beyond some arcang the rib behaves as a lateral stabiliser; its α is a sideslip angle. Two regimes, not one spline.
- **[Physics #9] Tip α is set by line tension, not aero** — the α that gives exactly the required outward force at minimum drag; computable from line tensions without panel method.
- **[Physics #10] Tip drag = yaw stability** — drag far from the axis is a stabilising yaw moment; a criterion never in the AoA curve today.
- **[Physics #11] Hinge rib** — spanwise position where carry = tension; optimum criterion changes nature on each side. Justifies a two-regime twist curve with an arc-dependent switch point.
- **[Physics #12] Lifting tip = taut tip lines** — measurable criterion: minimum tip-line tension, readable in the line solver.
- **[Physics #13] Arc centre ≠ pilot = fabric tension** — non-radial lines load the fabric spanwise; a wing-stiffness setting coupled to twist. Indicator: radial offset per rib.
- **[Physics #14] Decompose per-rib resultant radial/tangential** — radial carried by the line, tangential by fabric (or a moment). One spanwise diagram for line health and fabric tension.
- **[Model #15] Each half-wing has its own centre** — compute #14 from the relevant riser, not the axis; chest-strap spacing (mandatory harness parameter) becomes a twist parameter; show "tight/open" curves.
- **[Model #16] Twist "seen" changes in turns** — offset pilot → different β on inner/outer wing; twist optimum is a straight/turn compromise.
- **[Tool #17] Twist ↔ sweep slider** — for the same alignment objective, compute the all-twist (δα) and all-sweep (δx) solutions; a 0–100 % slider mixes them; planform preview and AoA curve update together.
- **[Tool #18] Lines → twist (read)** — input line lengths + arc + attachments; output implied AoA curve via the trimming solver. Matches the field gesture.
- **[Tool #19] Twist → lines (reciprocal)** — already what line computation does; what's missing is the per-rib diagnostic (#14).
- **[Tool #20] Both directions in one view with a lock** — choose which side is master (curve or lines), like a bidirectional spreadsheet.
- **[Tool #21] Movable attachments as 3rd lever** — shift A/B/C % chord instead of the rib; realigns the cone with no planform or α change.
- **[Model #22] Displacement convention: chord-fixed x-translation** — user decision. Area and aero preserved; a single δx(y) vector describes the change.
- **[Tool #23] Show the twist/sweep exchange rate** — δα ⇄ δx via L per rib; explains why 50 % isn't "half everywhere".
- **[UX #24] Ghost planform** — current planform solid, slider result dashed, δx arrows per rib.
- **[UX #25] Blue curve becomes the error curve** — replace/complement the absolute curve with per-rib resultant/cone misalignment (deg or A/C ratio); zero = aligned.
- **[UX #26] View from under the wing** — pilot's viewpoint: each rib with resultant direction and line cone.
- **[UX #27] Dedicated "Twist" tool** — user decision: separate FreeCAD tool grouping AoA, error curve, slider, ghost planform; "Apply" writes to `aoa` and/or `shape` (δx).

**User creative strengths:** strong physical intuition (kite analogy, outward tip lift, riser spacing), fast decisive choices (convention 1, separate tool).
**Energy level:** high on physics, terse on UX questions.

### Phase 2 — Assumption Reversal + Constraint Mapping

**Assumptions of the current model and verdicts** (H2, H7 by user; others decided by facilitator on user's request "avance tout seul"):

| # | Current assumption | Verdict | Consequence |
|---|---|---|---|
| H1 | Twist is one spline over the span | **Reverse (later)** | Two-regime curve with hinge rib; MVP keeps one spline but shows the hinge |
| H2 | `glide` is one global scalar | **Broken** (user: "je bidouillais avec, pas précis") | Replace by per-rib effective glide from the per-rib resultant |
| H3 | Arc is frozen during twist computation | **Keep for MVP** | Twist tool reads arc; radial-offset diagnostic only. Joint solve later via trimming tool |
| H4 | Pilot is a point on the axis | **Reverse now** | Per-half-wing centre at the riser (chest-strap spacing already in `lines`) |
| H5 | Rib is planar, angle = in-plane rotation | **Keep** | Tip sideslip treated as a diagnostic, not a new DoF |
| H6 | Attachments at fixed % chord | **Later** | Third lever adds DoF and confusion; keep out of v1 |
| H7 | User controls relative AoA directly | **Reverse, with fast surrogate** (user: panel method "trop long") | 2D per-rib kite model live; panel method on demand |
| H8 | Planform is an input | **Reverse, smoothed** | δx(y) output fitted to a 3–5 control-point spline (least squares) |

**Ideas from this phase:**

- **[Model #28] Live 2D "kite" model per rib** — airfoil polar (Cl, Cd, Cm from XFoil, already in `openglider/airfoil/xfoil.py`) → resultant direction + centre of pressure in rib plane → misalignment vs line cone. Milliseconds for the whole wing.
- **[Model #29] Panel method as a "Verify" button** — slider runs on the 2D model; Verify runs `panel_method` and shows the 2D↔3D correction as an uncertainty band.
- **[Model #30] Sensitivity cache** — one panel-method run yields ∂L/∂α per rib; slider then works linearly on that influence matrix until refreshed.
- **[Model #31] Per-rib effective glide** — `arctan(cos(arcang)/glide)` becomes `arctan(cos(arcang)·Cd_i/Cl_i)`-style local term; fixes the blue curve at the tips.
- **[Model #32] Smoothed δx(y)** — sweep correction fitted through a low-order spline; residual misalignment shown so the user sees what smoothing cost.

**Constraint map:**

| Constraint | Type | Effect on design |
|---|---|---|
| Panel method too slow for interactive use | compute | forces #28/#30 architecture |
| XFoil is an external binary, may be absent | environment | need a fallback polar (thin-airfoil Cl=2π(α−α0), flat-plate Cd) |
| `aoa` spline has 2–9 control points | model | auto-twist must be *fitted* back to control points, not written per rib |
| Shape is a spline (front/back curves) | model | δx must be applied via control points → smoothing is structural, not optional |
| Line lengths derive from geometry (forward) | model | lines→twist (#18) needs the trimming solver, not v1 |
| Trimming tool in progress (`openglider/glider/trim/solver.py`) | project | share `rib_local_frame`, arc fitting; avoid a second solver |
| Existing AoA tool users | UX | keep AoA tool untouched; new tool is additive (#27) |
| FreeCAD/pivy UI | UX | ghost planform = second `Line_old` set in a separate `SoSeparator` |

### Phase 3 — Morphological Analysis

Axes and options:

- **A. Optimum criterion**: A1 constant α · A2 elliptic lift · A3 lift + tip washout margin · A4 resultant aligned with line cone · A5 min tip-line tension · A6 weighted mix of A2/A3/A5
- **B. Adjusted variable**: B1 α per rib (twist) · B2 δx per rib (sweep) · B3 α/δx mix (slider) · B4 attachment % chord · B5 arc
- **C. Physics source**: C1 2D polar kite model · C2 panel method · C3 sensitivity matrix · C4 measured line lengths
- **D. UI mode**: D1 one-shot button · D2 live slider + ghost planform · D3 guide curve overlaid on AoA tool · D4 bidirectional locked view

Combinations evaluated:

| Concept | Cells | Assessment |
|---|---|---|
| **K1 — "Diagnostic first"** | A4 · (none) · C1 · D3 | Only reads: error curve (#25) + radial/tangential decomposition (#14) in a new tool. No solver. Low risk, immediately useful, validates the physics with the user's real wings. |
| **K2 — "Align slider"** | A4 · B3 · C1 · D2 | Slider 0–100 % twist↔sweep to zero the misalignment; ghost planform; Apply writes to `aoa` control points and/or `shape` front/back curves. Core of the user's request. |
| **K3 — "Optimum by objective"** | A6 · B1 · C3 · D1+D2 | Sliders perf / safety / tension; needs 3D lift → sensitivity matrix from one panel run. Builds on K2. |
| **K4 — "Read twist from lines"** | A4 · B1 · C4 · D4 | Inverse direction; depends on trimming solver maturity. |
| K5 — Constant α button | A1 · B1 · – · D1 | Trivial (already possible by flattening the spline); rejected as a goal but useful as a **reference curve** in K1. |

### Phase 4 — Decision Tree Mapping

```
Twist tool
├─ v0  K1 Diagnostic  (no solver, 2D kite model, per-half-wing riser centre)
│     success: error curve matches user's field experience on 2 known wings
│     ├─ physics confirmed → v1
│     └─ physics off      → revisit #4/#13 assumptions before any solver
├─ v1  K2 Align slider (δα ⇄ δx, chord-fixed, smoothed δx spline, ghost planform, Apply)
│     success: slider at 100 % twist reproduces what user would hand-draw; at 100 % sweep gives a plausible planform
│     dependency: shape control-point fitting utility
├─ v2  K3 Objective sliders (elliptic / washout / tip tension) + Verify (panel method) + sensitivity cache
│     dependency: panel method callable headless; hinge rib display (#11)
└─ v3  K4 Lines → twist  (after trimming tool)  + attachment lever (#21) + turn simulation (#16)
```

**Deferred/parked:** #8 tip-as-fin regime, #10 yaw, #16 turn offset, #20 lock view, #21 attachments, #26 under-wing view.

## Idea Organization

### Themes

1. **Physics reframing** (#1–#14): two angles, radial/tangential decomposition, outward tip lift, hinge rib, arc-centre/pilot offset. → *Foundation; drives the diagnostic.*
2. **Model corrections** (#15, #22, #28, #31, #32): per-half-wing centre, per-rib glide, live 2D model, smoothed δx. → *Cheap, high value.*
3. **The tool** (#17, #23, #24, #25, #27): dedicated Twist tool, slider, exchange rate, ghost planform, error curve. → *The deliverable.*
4. **Solver directions** (#18, #19, #20, #29, #30): lines↔twist, verify, sensitivity cache. → *Later, shares code with trimming tool.*
5. **Behaviour criteria** (#7, #9, #10, #11, #12, #16): three-term compromise, tip tension threshold, yaw, turns. → *Objective functions for v2.*

### Prioritization

- **Top priority:** K1 diagnostic (#14, #25, #28, #15). Small, validates everything else.
- **Quick wins:** per-rib effective glide (#31) fixing the blue curve; ghost planform rendering (#24).
- **Strategic:** K2 slider (#17, #22, #23, #32) — the feature as asked.
- **Later:** K3, K4.

## Action Plan

1. **Spec v0 (diagnostic)** — `openglider/glider/twist/` module: `rib_alignment(rib, riser_pos, polar)` → radial/tangential components, misalignment angle, implied A/C split. Pure functions, unit-tested against a hand-built 2D case. Fallback polar when XFoil is absent.
2. **Twist tool skeleton** (`freecad/glider/tools/twist_tool.py`, additive): reuse `SpanMappingTool` plotting; curves: user AoA (red), misalignment (replaces blue), constant-α reference (grey). Riser centre from `lines` (chest strap).
3. **Validate on two real wings** (user) — does the error curve say what the field says about tip behaviour? Gate for v1.
4. **v1 slider** — solver: per rib, δα_i and δx_i = L_i·tan(δα_i) that zero misalignment; mix k∈[0,1]; fit δx(y) to a 3–5-point spline; fit resulting α to `aoa` control points. Ghost planform. Apply → `aoa` and `shape` control points.
5. **v2** — objective sliders + Verify (panel method) + sensitivity cache + hinge-rib marker.
6. **v3** — lines→twist via trimming solver.

**Success metrics:** v0 — misalignment curve agrees in sign/shape with user's field diagnosis on 2 wings; v1 — 100 % twist reproduces a hand-drawn washout within 0.5°, 100 % sweep gives a planform the user accepts; slider refresh < 100 ms.

**Open questions:** smoothing order of δx; whether "Apply" should also re-run the line computation; how to expose chest-strap spacing if not already stored per glider.

### Creative Facilitation Narrative

The session started from a genuine dilemma — the kite intuition ("70° line = nose-down tip") versus the chord/wind angle. Working from first principles revealed that these are two different angles with two different jobs, and that a single twist variable cannot serve both; the user's "move the ribs instead" idea turned out to be exactly the missing second degree of freedom. The user corrected the facilitator on lift direction (outward, not inward), which unlocked the arc-centre/pilot-offset insight. Phase 2 and later phases were run autonomously at the user's request.

### Session Highlights

**User creative strengths:** physical intuition, decisive choices, willingness to correct the facilitator.
**AI facilitation approach:** physics-first, one question at a time, pivoting to UX when physics saturated.
**Breakthrough moments:** two-angles/two-roles; outward tip lift keeps tip lines taut; δα ⇄ δx exchange rate via line length.
**Energy flow:** high through physics, tapering; user delegated convergence.

## Implementation note (same day)

v0 + v1 implemented on branch `feature/twist-tool` (`openglider/glider/twist/`, `freecad/glider/tools/twist_tool.py`, `tests/test_twist.py`).
Correction to the session's physics: twist is a *weak* lever on the per-rib moment arm (it acts only through x_cp(α) and the Cd/Cl tilt of the resultant); the "δx ≈ L·tan δα" exchange rate (#23) does not hold — the resultant direction is set by the wind, not by the rib angle. Sweep (δx) is the direct lever. The common offset of the arm is a pitch-trim quantity (riser position / glide), not twist: the tool targets the centre rib by default and reports the offset separately.
