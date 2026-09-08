LINES
=====

Line model
----------

A glider is modelled as one half (y >= 0) and mirrored about the x-z plane.
The lineset of that half is a tree: every node has exactly one line below it
(`LineSet.get_lower_connected_lines`), lines are sorted from the lower
attachment points (risers, brake handles, `Node.type == 0`) up to the
attachment points on the wing (`Node.type == 2`). Tensions are computed from
the top down (`LineSet.calc_forces`): the force of an attachment point is
projected on its line, and each knot sums the forces of the lines above it.

Shared lines (Y bridle to both brake handles)
---------------------------------------------

`Line.shared = True` (checkbox *shared L/R* in the FreeCAD line tool, attribute
`shared` of `Line2D`) marks a line that is one leg of a symmetric pair: this
leg runs to a lower attachment point of this half (e.g. the right brake
handle) and its mirror image runs to the mirrored point (left handle). Both
legs, plus the upper lines coming from both halves, meet in a single knot on
the symmetry plane and carry the same tension. Typical use: the central brake
lines of the wing tied to both brake handles.

Only the half is modelled, so the lineset

* keeps the knot (upper node of the leg) on the symmetry plane: the
  y-component of the leg direction is imposed by the leg length, the in-plane
  direction follows the force balance as for any other line;
* drops the y-components of the forces at the knot (they cancel with the
  mirror image);
* computes the tension of the leg from the in-plane balance of the upper lines
  of this half only, the other half loading the other leg::

      F_leg * d_in_plane = (sum of upper line forces of this half)_in_plane

  so a splayed leg (angle to the symmetry plane) carries more than the in-plane
  resultant. The lines above the knot are computed as usual.

The leg must be longer than the distance of its lower attachment point to the
symmetry plane, otherwise a warning is logged and the constraint is ignored.
In the mirrored (complete) glider both legs appear, each with its own copy of
the knot at the same position; in the line tables the leg is flagged
*(partagée G/D)*.
