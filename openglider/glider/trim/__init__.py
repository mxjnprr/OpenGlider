"""
Trimming: inverse solver mapping suspension-line length changes (measured on a
prototype during flight testing) back onto the parametric glider geometry.

The design flow in OpenGlider is one-directional: geometry (arch / AoA / profiles)
-> line lengths.  Attachment points are pinned to the wing, so changing a line
length does not natively move the wing.  This package builds the *inverse*
(form-finding) step on top of the existing force machinery:

    line-length deltas -> attachment-point displacements (along tension) ->
    per-rib rigid fit (dAoA + d-arc + residual profile deformation) ->
    proposed smoothed arch + AoA curves (+ baked profile overrides).

See :class:`openglider.glider.trim.solver.TrimModel`.
"""

from openglider.glider.trim.solver import TrimModel, TrimResult

__all__ = ["TrimModel", "TrimResult"]
