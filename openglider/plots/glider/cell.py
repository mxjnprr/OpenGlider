import logging

import numpy as np

from openglider.airfoil import get_x_value
from openglider.glider.cell import Panel
from openglider.plots.glider.config import PatternConfig
from openglider.vector import PolyLine2D, norm, normalize, vector_angle
from openglider.vector.drawing import PlotPart
from openglider.vector.text import Text


class PanelPlot:
    DefaultConf = PatternConfig

    def __init__(self, panel: Panel, cell, flattended_cell, config=None):
        self.panel = panel
        self.cell = cell
        self.config = self.DefaultConf(config)

        self._flattened_cell = flattended_cell

        # Use full lists for flattening
        self.inner = flattended_cell["inner"]
        self.ballooned = flattended_cell["ballooned"]
        self.outer = flattended_cell["outer"]
        self.outer_orig = flattended_cell["outer_orig"]
        
        # Store y_start/y_end for post-generation trimming
        self.y_start = getattr(panel, 'y_start', 0.0)
        self.y_end = getattr(panel, 'y_end', 1.0)

        self.x_values = self.cell.rib1.profile_2d.x_values

        self.logger = logging.getLogger(
            r"{self.__class__.__module__}.{self.__class__.__name__}"
        )

    def flatten(self, attachment_points):
        plotpart = PlotPart(
            material_code=self.panel.material_code, name=self.panel.name
        )

        cut_allowances = {
            "folded": self.config.allowance_entry_open,
            "parallel": self.config.allowance_trailing_edge,
            "orthogonal": self.config.allowance_design,
            "singleskin": self.config.allowance_entry_open,
            "cut_3d": self.config.allowance_design,
        }

        cut_types = {
            "folded": self.config.cut_entry,
            "parallel": self.config.cut_trailing_edge,
            "orthogonal": self.config.cut_design,
            "singleskin": self.config.cut_entry,
            "cut_3d": self.config.cut_3d,
        }

        ik_values = self.panel._get_ik_values(
            self.cell, self.config.midribs, exact=True
        )

        # allowance fallbacks
        allowance_front = cut_allowances[self.panel.cut_front["type"]]
        allowance_back = cut_allowances[self.panel.cut_back["type"]]

        # get allowance from self.panel
        amount_front = -self.panel.cut_front.get("amount", allowance_front)
        amount_back = self.panel.cut_back.get("amount", allowance_back)

        # cuts -> cut-line, index left, index right
        cut_front = cut_types[self.panel.cut_front["type"]](amount_front)
        cut_back = cut_types[self.panel.cut_back["type"]](amount_back)

        inner_front = [(line, ik[0]) for line, ik in zip(self.inner, ik_values)]
        inner_back = [(line, ik[1]) for line, ik in zip(self.inner, ik_values)]

        shape_3d_amount_front = [-x for x in self.panel.cut_front["amount_3d"]]
        shape_3d_amount_back = self.panel.cut_back["amount_3d"]
        
        # For split panels, the inner list may have different length than amount_3d
        # Interpolate amount_3d to match inner list length
        num_inner = len(inner_front)
        if len(shape_3d_amount_front) != num_inner:
            shape_3d_amount_front = list(np.linspace(
                shape_3d_amount_front[0],
                shape_3d_amount_front[-1],
                num_inner
            ))
        if len(shape_3d_amount_back) != num_inner:
            shape_3d_amount_back = list(np.linspace(
                shape_3d_amount_back[0],
                shape_3d_amount_back[-1],
                num_inner
            ))

        if self.panel.cut_front["type"] != "cut_3d":
            dist = np.linspace(
                shape_3d_amount_front[0],
                shape_3d_amount_front[-1],
                len(shape_3d_amount_front),
            )
            shape_3d_amount_front = list(dist)

        if self.panel.cut_back["type"] != "cut_3d":
            dist = np.linspace(
                shape_3d_amount_back[0],
                shape_3d_amount_back[-1],
                len(shape_3d_amount_back),
            )
            shape_3d_amount_back = list(dist)

        # For cut_3d type, use outer_orig to avoid geometry mismatch from .check()
        outer_left = self.outer_orig[0] if self.panel.cut_front["type"] == "cut_3d" or self.panel.cut_back["type"] == "cut_3d" else self.outer[0]
        outer_right = self.outer_orig[1] if self.panel.cut_front["type"] == "cut_3d" or self.panel.cut_back["type"] == "cut_3d" else self.outer[1]
        
        cut_front_result = cut_front.apply(
            inner_front, outer_left, outer_right, shape_3d_amount_front
        )
        cut_back_result = cut_back.apply(
            inner_back, outer_left, outer_right, shape_3d_amount_back
        )

        panel_left = self.outer[0][
            cut_front_result.index_left : cut_back_result.index_left
        ]
        panel_back = cut_back_result.curve.copy()
        panel_right = self.outer[1][
            cut_front_result.index_right : cut_back_result.index_right : -1
        ]
        panel_front = cut_front_result.curve.copy()

        # spitzer schnitt
        # rechts
        if cut_front_result.index_right >= cut_back_result.index_right:
            panel_right = PolyLine2D([])

            _cuts = panel_front.cut_with_polyline(
                panel_back, startpoint=len(panel_front) - 1
            )
            try:
                ik_front, ik_back = next(_cuts)
                panel_back = panel_back[:ik_back]
                panel_front = panel_front[:ik_front]
            except StopIteration:
                pass  # todo: fix!!

        # lechts
        if cut_front_result.index_left >= cut_back_result.index_left:
            panel_left = PolyLine2D([])

            _cuts = panel_front.cut_with_polyline(panel_back, startpoint=0)
            try:
                ik_front, ik_back = next(_cuts)
                panel_back = panel_back[ik_back:]
                panel_front = panel_front[ik_front:]
            except StopIteration:
                pass  # todo: fix as well!

        panel_back = panel_back[::-1]
        if panel_right:
            panel_right = panel_right[::-1]

        envelope = panel_right + panel_back
        if len(panel_left) > 0:
            envelope += panel_left[::-1]
        envelope += panel_front
        envelope += PolyLine2D([envelope[0]])

        plotpart.layers["envelope"].append(envelope)

        if self.config.debug:
            plotpart.layers["debug"].append(
                PolyLine2D([line[ik] for line, ik in inner_front])
            )
            plotpart.layers["debug"].append(
                PolyLine2D([line[ik] for line, ik in inner_back])
            )
            for front, back in zip(inner_front, inner_back):
                plotpart.layers["debug"].append(front[0][front[1] : back[1]])

        # sewings
        plotpart.layers["stitches"] += [
            self.inner[0][
                cut_front_result.inner_indices[0] : cut_back_result.inner_indices[0]
            ],
            self.inner[-1][
                cut_front_result.inner_indices[-1] : cut_back_result.inner_indices[-1]
            ],
        ]

        # folding line
        plotpart.layers["marks"] += [
            PolyLine2D(
                [line[x] for line, x in zip(self.inner, cut_front_result.inner_indices)]
            ),
            PolyLine2D(
                [line[x] for line, x in zip(self.inner, cut_back_result.inner_indices)]
            ),
        ]

        # TODO
        if False:
            if panel_right:
                right = (
                    PolyLine2D([panel_front.last()])
                    + panel_right
                    + PolyLine2D([panel_back[0]])
                )
                plotpart.layers["cuts"].append(right)

            plotpart.layers["cuts"].append(panel_back)

            if panel_left:
                left = (
                    PolyLine2D([panel_back.last()])
                    + panel_left
                    + PolyLine2D([panel_front[0]])
                )
                plotpart.layers["cuts"].append(left)

            plotpart.layers["cuts"].append(panel_front)
        else:
            plotpart.layers["cuts"].append(envelope.copy())

        self._insert_text(plotpart)
        self._insert_controlpoints(plotpart)
        self._insert_attachment_points(plotpart, attachment_points=attachment_points)
        self._insert_diagonals(plotpart)
        self._insert_rigidfoils(plotpart)
        self._insert_minirib_marks(plotpart)
        # self._insert_center_rods(plotpart)
        # TODO: add in parametric way

        self._align_upright(plotpart)

        return plotpart

    def get_point(self, x):
        ik = get_x_value(self.x_values, x)
        return [lst[ik] for lst in self.ballooned]

    def get_p1_p2(self, x, which):
        side = {"left": 0, "right": 1}[which]
        ik = get_x_value(self.x_values, x)

        return self.ballooned[side][ik], self.outer_orig[side][ik]

    def _align_upright(self, plotpart):
        def get_p1_p2(side):
            p1 = self.get_p1_p2(self.panel.cut_front[side], side)[0]
            p2 = self.get_p1_p2(self.panel.cut_back[side], side)[0]

            return p2 - p1

        vector = get_p1_p2("left")
        vector += get_p1_p2("right")
        angle = vector_angle(vector, [0, 1])

        plotpart.rotate(angle)
        return plotpart

    def _insert_text(self, plotpart):
        import numpy as np

        from openglider.vector.functions import norm

        if self.config.layout_seperate_panels and not self.panel.is_lower():
            left = get_x_value(self.x_values, self.panel.cut_back["left"])
            right = get_x_value(self.x_values, self.panel.cut_back["right"])
            p2 = self.ballooned[-1][right]  # Use last element instead of [1]
            p1 = self.ballooned[0][left]
            align = "left"
            allowance = self.config.allowance_design
        else:
            left = get_x_value(self.x_values, self.panel.cut_front["left"])
            right = get_x_value(self.x_values, self.panel.cut_front["right"])
            p1 = self.ballooned[-1][right]  # Use last element instead of [1]
            p2 = self.ballooned[0][left]
            align = "right"
            allowance = self.config.allowance_design

        # Offset p1/p2 into the seam margin using the perpendicular direction
        # For upper panels: text at back cut, perpendicular points UP (into margin)
        # For lower panels: p1/p2 reversed, perpendicular points DOWN (into margin)
        diff = np.array(p2) - np.array(p1)
        diff_len = norm(diff)
        if diff_len > 1e-10:
            perp = np.array([-diff[1], diff[0]]) / diff_len
        else:
            perp = np.array([0, 1])

        ratio = getattr(self.config, 'text_inset_ratio', 0.85)
        text = self.panel.name
        # Text size: 80% of allowance, but max 8mm to avoid huge text
        text_size = min(allowance * 0.8, 0.008)
        text_height = text_size * 0.8  # letter height in meters
        # Offset = position the TOP of letters at ratio*allowance from stitch line
        offset = perp * (allowance * ratio - text_height)
        p1 = np.array(p1) + offset
        p2 = np.array(p2) + offset
        use_dashed = getattr(self.config, 'laser_text_mode', False)
        part_text = Text(
            text,
            p1,
            p2,
            size=text_size,
            align=align,
            valign=0.5,
            height=0.8,
            dashed=use_dashed,
            dot_spacing=getattr(self.config, 'dot_spacing', 0.15),
        ).get_vectors()
        text_layer = "cuts" if use_dashed else "text"
        plotpart.layers[text_layer] += part_text

    def _insert_controlpoints(self, plotpart):
        for x in self.config.distribution_controlpoints:
            for side in ("left", "right"):
                if self.panel.cut_front[side] <= x <= self.panel.cut_back[side]:
                    p1, p2 = self.get_p1_p2(x, side)
                    plotpart.layers["L0"] += self.config.marks_laser_controlpoint(
                        p1, p2
                    )

    def _insert_diagonals(self, plotpart):
        def insert_diagonal(x, height, side, front):
            if height == 1:
                xval = -x
            elif height == -1:
                xval = x
            else:
                return

            if self.panel.cut_front[side] <= xval <= self.panel.cut_back[side]:
                p1, p2 = self.get_p1_p2(xval, side)
                plotpart.layers["L0"] += self.config.marks_laser_diagonal(p1, p2)
                if (front and height == -1) or (not front and height == 1):
                    plotpart.layers["marks"] += self.config.marks_diagonal_front(p1, p2)
                else:
                    plotpart.layers["marks"] += self.config.marks_diagonal_back(p1, p2)

        for strap in self.cell.straps + self.cell.diagonals:
            insert_diagonal(*strap.left_front, side="left", front=False)
            insert_diagonal(*strap.left_back, side="left", front=True)
            insert_diagonal(*strap.right_front, side="right", front=True)
            insert_diagonal(*strap.right_back, side="right", front=False)

        # band-split elements: where neighbouring bands meet on the rib
        for drib in self.cell.diagonals:
            if not getattr(drib, "band_split", None):
                continue
            for side in ("left", "right"):
                try:
                    bounds = drib.get_band_split_marks(self.cell, side == "right")
                except Exception:
                    bounds = []
                for x, height in bounds:
                    xval = -x if height > 0 else x
                    if self.panel.cut_front[side] <= xval <= self.panel.cut_back[side]:
                        p1, p2 = self.get_p1_p2(xval, side)
                        plotpart.layers["L0"] += self.config.marks_laser_diagonal(p1, p2)
                        plotpart.layers["marks"] += self.config.marks_band_split(p1, p2)

    def _insert_attachment_points(self, plotpart, attachment_points):
        for attachment_point in attachment_points:
            if hasattr(attachment_point, "cell"):
                if attachment_point.cell != self.cell:
                    continue

                cell_pos = attachment_point.cell_pos

            elif hasattr(attachment_point, "rib"):
                if attachment_point.rib not in self.cell.ribs:
                    continue

                if attachment_point.rib == self.cell.rib1:
                    cell_pos = 0
                elif attachment_point.rib == self.cell.rib2:
                    cell_pos = 1
                else:
                    raise AttributeError
            else:
                raise AttributeError

            cut_f_l = self.panel.cut_front["left"]
            cut_f_r = self.panel.cut_front["right"]
            cut_b_l = self.panel.cut_back["left"]
            cut_b_r = self.panel.cut_back["right"]
            cut_f = cut_f_l + cell_pos * (cut_f_r - cut_f_l)
            cut_b = cut_b_l + cell_pos * (cut_b_r - cut_b_l)

            if cut_f <= attachment_point.rib_pos <= cut_b:
                rib_pos = attachment_point.rib_pos
                left, right = self.get_point(rib_pos)

                p1 = left + cell_pos * (right - left)
                d = normalize(right - left) * 0.008  # 8mm
                if cell_pos == 1:
                    p2 = p1 + d
                else:
                    p2 = p1 - d

                if cell_pos in (1, 0):
                    which = ["left", "right"][cell_pos]
                    x1, x2 = self.get_p1_p2(rib_pos, which)
                    plotpart.layers["marks"] += self.config.marks_attachment_point(
                        x1, x2
                    )
                    plotpart.layers["L0"] += self.config.marks_laser_attachment_point(
                        x1, x2
                    )
                else:
                    plotpart.layers["marks"] += self.config.marks_attachment_point(
                        p1, p2
                    )
                    plotpart.layers["L0"] += self.config.marks_laser_attachment_point(
                        p1, p2
                    )

                # p1, p2 = self.get_p1_p2(attachment_point.rib_pos, which)

                if self.config.insert_attachment_point_text and not self.panel.is_lower():
                    text_align = "left" if cell_pos > 0.7 else "right"

                    if text_align == "right":
                        d1 = norm(self.get_point(cut_f_l)[0] - left)
                        d2 = norm(self.get_point(cut_b_l)[0] - left)
                    else:
                        d1 = norm(self.get_point(cut_f_r)[1] - right)
                        d2 = norm(self.get_point(cut_b_r)[1] - right)

                    bl = self.ballooned[0]
                    br = self.ballooned[1]

                    text_height = 0.01 * 0.8
                    dmin = text_height + 0.001

                    if d1 < dmin and d2 + d1 > 2 * dmin:
                        offset = dmin - d1
                        ik = get_x_value(self.x_values, rib_pos)
                        left = bl[bl.walk(ik, offset)]
                        right = br[br.walk(ik, offset)]
                    elif d2 < dmin and d1 + d2 > 2 * dmin:
                        offset = dmin - d2
                        ik = get_x_value(self.x_values, rib_pos)
                        left = bl[bl.walk(ik, -offset)]
                        right = br[br.walk(ik, -offset)]

                    if self.config.layout_seperate_panels and self.panel.is_lower():
                        # rotated later
                        p2 = left
                        p1 = right
                        # text_align = text_align
                    else:
                        p1 = left
                        p2 = right
                        # text_align = text_align
                    # Skip placeholder names (line_name_not_set, etc.)
                    ap_name = attachment_point.name
                    if 'not_set' in ap_name.lower():
                        continue
                    use_dashed = getattr(self.config, 'laser_text_mode', False)
                    text_layer = "cuts" if use_dashed else "text"
                    # Text size: fit in seam margin
                    ap_text_size = min(self.config.allowance_design * 0.8, 0.008)
                    plotpart.layers[text_layer] += Text(
                        f" {ap_name} ",
                        p1,
                        p2,
                        size=ap_text_size,
                        align=text_align,
                        valign=0,
                        height=0.8,
                        dashed=use_dashed,
                        dot_spacing=getattr(self.config, 'dot_spacing', 0.15),
                    ).get_vectors()

    def _insert_rigidfoils(self, plotpart):
        for rigidfoil in self.cell.rigidfoils:
            line = rigidfoil.draw_panel_marks(self.cell, self.panel)
            if line is not None:
                plotpart.layers["marks"].append(line)

                # laser dots
                plotpart.layers["L0"].append(PolyLine2D([line.data[0]]))
                plotpart.layers["L0"].append(PolyLine2D([line.data[-1]]))

    def _insert_minirib_marks(self, plotpart):
        """Insert marks showing where miniribs attach to this panel."""
        if not hasattr(self.cell, 'miniribs'):
            return
        
        for mr_idx, minirib in enumerate(self.cell.miniribs):
            y_value = minirib.y_value  # Position in cell (0-1)
            
            # Get the minirib attachment range (intrados and extrados start/end)
            intrados_start = minirib.intrados_start
            extrados_start = minirib.extrados_start
            
            # Calculate chord for end percentage
            chord = self.cell.rib1.chord * (1 - y_value) + self.cell.rib2.chord * y_value
            end_pct = minirib.get_end_percentage(chord)
            
            # Check if this panel covers the minirib range
            cut_front_left = self.panel.cut_front["left"]
            cut_back_left = self.panel.cut_back["left"]
            cut_front_right = self.panel.cut_front["right"]
            cut_back_right = self.panel.cut_back["right"]
            
            # Interpolate cut positions for the y_value
            cut_front = cut_front_left + y_value * (cut_front_right - cut_front_left)
            cut_back = cut_back_left + y_value * (cut_back_right - cut_back_left)
            
            # Determine which x positions to mark based on panel type (lower/upper)
            is_lower = self.panel.is_lower()
            
            if is_lower:
                # Lower panel (intrados): minirib starts at intrados_start
                x_start = intrados_start
                x_end = end_pct
            else:
                # Upper panel (extrados): minirib starts at extrados_start
                x_start = -extrados_start  # Negative for extrados
                x_end = -end_pct
            
            # Check if start position is within panel range
            if cut_front <= abs(x_start) <= cut_back:
                try:
                    # Get the interpolated position on the panel
                    ik_start = get_x_value(self.x_values, abs(x_start) if is_lower else x_start)
                    
                    # Interpolate between left and right ballooned curves
                    left_pt = np.array(self.ballooned[0][ik_start])
                    right_pt = np.array(self.ballooned[1][ik_start])
                    mark_pt = left_pt + y_value * (right_pt - left_pt)
                    
                    # Create a small cross mark
                    mark_size = 0.003  # 3mm
                    mark_line = PolyLine2D([
                        mark_pt + np.array([-mark_size, 0]),
                        mark_pt + np.array([mark_size, 0])
                    ])
                    mark_line_v = PolyLine2D([
                        mark_pt + np.array([0, -mark_size]),
                        mark_pt + np.array([0, mark_size])
                    ])
                    
                    plotpart.layers["marks"].append(mark_line)
                    plotpart.layers["marks"].append(mark_line_v)
                    
                    # Also add laser dot
                    plotpart.layers["L0"].append(PolyLine2D([mark_pt]))
                except Exception as e:
                    self.logger.debug(f"Failed to insert minirib mark: {e}")


def _line_intersection(p1, d1, p2, d2):
    """Intersection of two lines p + t*d, or None when parallel."""
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-12:
        return None
    dp = p2 - p1
    t = (dp[0] * d2[1] - dp[1] * d2[0]) / det
    return p1 + t * d1


def _offset_closed_polygon(points, amounts, miter_limit=2.0):
    """
    Offset a closed polygon outward.  ``amounts`` is either one distance or
    one distance per edge (edge i runs from point i to point i + 1), so seam
    allowances and hem allowances can differ.

    Corners are mitred (the two offset edges are intersected); a mitre longer
    than ``miter_limit`` times the allowance is bevelled by two points that
    stay on the offset edges.  Edges whose offset would run backwards (tight
    concave corners, rounded shoulders smaller than the allowance) are
    dropped so that the result never loops or crosses itself.
    Returns a list of 2D points (end point not repeated).
    """
    pts = [np.asarray(p, dtype=float) for p in points]
    n = len(pts)
    if n < 3:
        return [list(p) for p in pts]
    if np.isscalar(amounts):
        amounts = [float(amounts)] * n
    amounts = [float(a) for a in amounts]
    if all(abs(a) < 1e-12 for a in amounts):
        return [list(p) for p in pts]
    area = 0.0
    for i in range(n):
        p, q = pts[i], pts[(i + 1) % n]
        area += p[0] * q[1] - q[0] * p[1]
    ccw = area > 0

    # edges: start point, unit direction, outward normal, allowance
    edges = []
    for i in range(n):
        d = pts[(i + 1) % n] - pts[i]
        length = norm(d)
        if length < 1e-12:
            continue
        d = d / length
        nrm = np.array([d[1], -d[0]]) if ccw else np.array([-d[1], d[0]])
        edges.append((pts[i], d, nrm, amounts[i]))
    if len(edges) < 3:
        return [list(p) for p in pts]

    def corner(e_prev, e_next):
        """mitre point of two offset edges (lines), or None when parallel"""
        p1, d1, n1, a1 = e_prev
        p2, d2, n2, a2 = e_next
        return _line_intersection(p1 + n1 * a1, d1, p2 + n2 * a2, d2)

    # drop edges whose offset segment runs backwards, until stable
    active = list(range(len(edges)))
    for _ in range(len(edges)):
        m = len(active)
        if m < 3:
            break
        corners = []
        for k in range(m):
            e_prev = edges[active[k - 1]]
            e_cur = edges[active[k]]
            c = corner(e_prev, e_cur)
            if c is None:
                c = e_cur[0] + e_cur[2] * e_cur[3]
            corners.append(c)
        drop = []
        for k in range(m):
            e_cur = edges[active[k]]
            seg = corners[(k + 1) % m] - corners[k]
            if np.dot(seg, e_cur[1]) < -1e-9:
                drop.append(active[k])
        if not drop:
            break
        # drop the worst offender per pass to stay conservative
        worst = None
        for k in range(m):
            if active[k] in drop:
                e_cur = edges[active[k]]
                back = -np.dot(corners[(k + 1) % m] - corners[k], e_cur[1])
                if worst is None or back > worst[0]:
                    worst = (back, active[k])
        active.remove(worst[1])

    result = []
    m = len(active)
    for k in range(m):
        e_prev = edges[active[k - 1]]
        e_cur = edges[active[k]]
        p_prev, d_prev, n_prev, a_prev = e_prev
        p_cur, d_cur, n_cur, a_cur = e_cur
        mitre = corner(e_prev, e_cur)
        if mitre is None:
            result.append(list(p_cur + n_cur * (a_prev + a_cur) / 2.0))
            continue
        # reference corner: where the two original edge lines meet
        ref = _line_intersection(p_prev, d_prev, p_cur, d_cur)
        if ref is None:
            ref = p_cur
        limit = miter_limit * max(abs(a_prev), abs(a_cur), 1e-9)
        if norm(mitre - ref) <= limit:
            result.append(list(mitre))
            continue
        # bevel: cut the mitre with a line perpendicular to the bisector at
        # ``limit`` from the corner; keep the points on their offset lines
        bis = mitre - ref
        bis = bis / norm(bis)
        clip_point = ref + bis * limit
        clip_dir = np.array([-bis[1], bis[0]])
        b1 = _line_intersection(clip_point, clip_dir, p_prev + n_prev * a_prev, d_prev)
        b2 = _line_intersection(clip_point, clip_dir, p_cur + n_cur * a_cur, d_cur)
        if b1 is None or b2 is None:
            result.append(list(clip_point))
        else:
            result.append(list(b1))
            result.append(list(b2))
    return result


class DribPlot:
    DefaultConf = PatternConfig

    def __init__(self, drib, cell, config):
        self.drib = drib
        self.cell = cell
        self.config = self.DefaultConf(config)

        self.left, self.right = self.drib.get_flattened(self.cell)

        self.left_out = self.left.copy()
        self.right_out = self.right.copy()

        self.left_out.add_stuff(-self.config.allowance_general)
        self.right_out.add_stuff(self.config.allowance_general)

    def get_left(self, x):
        return self.get_p1_p2(x, side=0)

    def get_right(self, x):
        return self.get_p1_p2(x, side=1)

    def _is_valid(self, x, side=0):
        if side == 0:
            front = self.drib.left_front
            back = self.drib.left_back
        else:
            front = self.drib.right_front
            back = self.drib.right_back

        if (front[1], back[1]) not in ((-1, -1), (1, 1)):
            return False

        if front[1] > 0:
            # swapped sides
            boundary = [-front[0], -back[0]]
        else:
            boundary = [front[0], back[0]]
        boundary.sort()

        if not boundary[0] <= x <= boundary[1]:
            return False

        return True

    def get_p1_p2(self, x, side=0):
        assert self._is_valid(x, side=side)

        if side == 0:
            front = self.drib.left_front
            back = self.drib.left_back
            rib = self.cell.rib1
            inner = self.left
            outer = self.left_out
        else:
            front = self.drib.right_front
            back = self.drib.right_back
            rib = self.cell.rib2
            inner = self.right
            outer = self.right_out

        assert front[0] <= x <= back[0]

        foil = rib.profile_2d
        # -1 -> lower, 1 -> upper
        foil_side = 1 if front[1] == -1 else -1

        x1 = front[0] * foil_side
        x2 = x * foil_side

        ik_1 = foil(x1)
        ik_2 = foil(x2)
        length = foil[ik_1:ik_2].get_length() * rib.chord

        ik_new = inner.walk(0, length)
        return inner[ik_new], outer[ik_new]

    def _insert_attachment_points(self, plotpart, attachment_points=None):
        attachment_points = attachment_points or []

        for attachment_point in attachment_points:
            if not hasattr(attachment_point, "rib"):
                continue
            x = attachment_point.rib_pos
            if attachment_point.rib is self.cell.rib1:
                if not self._is_valid(x, side=0):
                    continue
                p1, p2 = self.get_left(attachment_point.rib_pos)
            elif attachment_point.rib is self.cell.rib2:
                if not self._is_valid(x, side=1):
                    continue

                p1, p2 = self.get_right(attachment_point.rib_pos)
            else:
                continue

            plotpart.layers["marks"] += self.config.marks_attachment_point(p1, p2)
            plotpart.layers["L0"] += self.config.marks_laser_attachment_point(p1, p2)

    def _insert_text(self, plotpart):
        # Place text in the front fold area (seam margin below the front stitch line)
        # Strategy: compute body direction (front→back), offset p1/p2 in the
        # opposite direction (outward) by the fold depth, so letters extend
        # upward from the bottom of the fold area toward the stitch line.
        import numpy as np

        from openglider.vector import norm

        mid = len(self.left) // 2
        body_dir = np.array(self.left[mid]) - np.array(self.left[0])
        body_len = norm(body_dir)
        if body_len > 1e-10:
            body_dir = body_dir / body_len
        else:
            body_dir = np.array([0, 1])

        # Outward at the front = opposite of body direction
        outward = -body_dir
        fold_depth = self.config.drib_allowance_folds * getattr(self.config, 'text_inset_ratio', 0.85)

        p1 = np.array(self.left[0]) + outward * fold_depth
        p2 = np.array(self.right[0]) + outward * fold_depth

        # Text size: 80% of allowance, but max 8mm to avoid huge text
        text_size = min(self.config.drib_allowance_folds * 0.8, 0.008)
        use_dashed = getattr(self.config, 'laser_text_mode', False)
        text_layer = "cuts" if use_dashed else "text"
        plotpart.layers[text_layer] += Text(
            f" {self.drib.name} ",
            p1,
            p2,
            size=text_size,
            height=0.8,
            valign=0.5,
            dashed=use_dashed,
            dot_spacing=getattr(self.config, 'dot_spacing', 0.15),
        ).get_vectors()

    def flatten(self, attachment_points=None):
        return self._flatten(attachment_points, self.config.drib_num_folds)

    def _band_split_pieces(self):
        """Independent pieces of a band-split element, frame of self.left/right."""
        config = getattr(self.drib, 'band_split', None)
        if not config:
            return None
        try:
            return self.drib.get_band_split_pieces_flat(self.left, self.right, config)
        except Exception:
            return None

    def flatten_pieces(self, attachment_points=None):
        """
        All plot parts of this element: one per band for a band-split
        element (independent pieces, own allowances, marks and name), else
        [flatten()].
        """
        pieces = self._band_split_pieces()
        if not pieces:
            return [self.flatten(attachment_points)]
        from openglider.glider.cell.elements import _edge_kinds

        seams = self.drib.get_band_split_seams(self.left, self.right)
        num_folds = self.config.drib_num_folds
        hem = self.config.drib_allowance_folds if num_folds > 0 else 0.0
        allowance = self.config.allowance_general
        parts = []
        for k, outline in enumerate(pieces):
            name = self.drib.name if len(pieces) == 1 else f"{self.drib.name}-{k + 1}"
            plotpart = PlotPart(material_code=self.drib.material_code, name=name)
            kinds = _edge_kinds(outline, seams)
            plotpart._band_outline = outline
            plotpart._band_kinds = kinds
            amounts = [allowance if kind == "seam" else hem for kind in kinds]
            cut = _offset_closed_polygon(outline, amounts)
            plotpart.layers["cuts"].append(PolyLine2D(cut + [cut[0]]))
            runs = self._seam_runs(outline, kinds)
            for run in runs:
                plotpart.layers["stitches"].append(PolyLine2D([list(map(float, p)) for p in run["points"]]))
            self._insert_piece_marks(plotpart, runs, allowance)
            self._insert_piece_text(plotpart, runs, allowance)
            try:
                self._insert_attachment_points(plotpart, attachment_points)
            except Exception:
                pass
            parts.append(plotpart)
        return parts

    def _seam_runs(self, outline, kinds):
        """
        Consecutive seam edges of a piece outline: list of dicts with the
        run points (front -> back along the rib), the rib curve they lie on
        and the outward normal of the piece along the run.
        """
        n = len(outline)
        pts = [np.asarray(p, dtype=float) for p in outline]
        area = 0.0
        for i in range(n):
            area += pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
        ccw = area > 0
        # start the scan on a free edge so that a run never wraps around
        start = next((i for i in range(n) if kinds[i] != "seam"), None)
        if start is None:
            return []
        runs, run = [], []
        for step in range(n):
            i = (start + step) % n
            if kinds[i] == "seam":
                if not run:
                    run.append(pts[i])
                run.append(pts[(i + 1) % n])
            elif run:
                runs.append(run)
                run = []
        if run:
            runs.append(run)

        result = []
        for run in runs:
            d = run[-1] - run[0]
            if norm(d) < 1e-9:
                continue
            d = d / norm(d)
            outward = np.array([d[1], -d[0]]) if ccw else np.array([-d[1], d[0]])
            # which rib curve, and which end is the front (start of the curve)
            best = None
            for curve in (self.left, self.right):
                data = np.asarray(curve.data, dtype=float)
                dist = np.linalg.norm(data - run[len(run) // 2], axis=1).min()
                if best is None or dist < best[0]:
                    best = (dist, curve, data)
            _, curve, data = best
            i_first = int(np.argmin(np.linalg.norm(data - run[0], axis=1)))
            i_last = int(np.argmin(np.linalg.norm(data - run[-1], axis=1)))
            points = run if i_first <= i_last else run[::-1]
            result.append({"points": points, "curve": curve, "outward": outward})
        return result

    def _insert_piece_marks(self, plotpart, runs, allowance):
        """Assembly marks on a piece: a tick across the allowance at both ends
        of every seam (front end: arrow, back end: double line) plus laser dots."""
        for run in runs:
            outward = run["outward"]
            front, back = run["points"][0], run["points"][-1]
            for point, mark in ((front, self.config.marks_diagonal_front),
                                (back, self.config.marks_diagonal_back)):
                p1 = np.asarray(point, dtype=float)
                p2 = p1 + outward * allowance
                plotpart.layers["marks"] += self.config.marks_band_split_seam(p1, p2)
                plotpart.layers["marks"] += mark(p1, p2)
                plotpart.layers["L0"] += self.config.marks_laser_diagonal(p1, p2)

    def _piece_label(self, name):
        """Piece name plus the wing side (G/D) on complete-glider exports."""
        side = getattr(self.cell, "wing_side", None)
        labels = getattr(self.config, "wing_side_labels", ("G", "D"))
        if side == "left":
            return f"{name} {labels[0]}"
        if side == "right":
            return f"{name} {labels[1]}"
        return name

    def _insert_piece_text(self, plotpart, runs, allowance):
        """
        Name written in the seam allowance of the longest seam, centred in
        the allowance and always upright on the sheet (never upside down),
        so that a readable name means "good side up".  The letters are
        shrunk to fit the seam length; when even 3 mm letters do not fit,
        the name goes to the hem allowance of the longest free edge instead.
        """
        if not runs:
            return
        text = f" {self._piece_label(plotpart.name)} "
        hem = self.config.drib_allowance_folds if self.config.drib_num_folds > 0 else 0.0
        min_size = 0.003
        candidates = []
        for run in runs:
            # follow the seam locally: baseline along the tangent at the middle
            # of the (possibly curved) seam, so the letters stay in the margin
            pts = [np.asarray(p, dtype=float) for p in run["points"]]
            seg = [norm(b - a) for a, b in zip(pts[:-1], pts[1:])]
            length = float(sum(seg))
            if length < 1e-6:
                continue
            half, acc, i = length / 2.0, 0.0, 0
            while i < len(seg) - 1 and acc + seg[i] < half:
                acc += seg[i]
                i += 1
            t = (half - acc) / seg[i] if seg[i] > 1e-12 else 0.0
            mid = pts[i] + (pts[i + 1] - pts[i]) * t
            tangent = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
            if norm(tangent) < 1e-9:
                tangent = pts[-1] - pts[0]
            tangent = tangent / norm(tangent)
            outward = np.array([-tangent[1], tangent[0]])
            if np.dot(outward, run["outward"]) < 0:
                outward = -outward
            candidates.append((mid, tangent, outward, length, allowance))
        free = self._longest_free_run(plotpart)
        if free is not None and hem > 0:
            p_a, p_b, outward = free
            length = norm(p_b - p_a)
            if length > 1e-6:
                tangent = (p_b - p_a) / length
                candidates.append(((p_a + p_b) / 2.0, tangent, outward, length, hem))
        if not candidates:
            return

        best = None
        for mid, tangent, outward, length, width in candidates:
            size = min(width * 0.7, 0.006, (min(length, 0.12) - 0.004) / len(text))
            if best is None or size > best[0]:
                best = (size, mid, tangent, outward, length, width)
        size, mid, tangent, outward, length, width = best
        if size < min_size:
            return
        # read left to right on the sheet (bottom to top when vertical)
        if tangent[0] < -1e-9 or (abs(tangent[0]) <= 1e-9 and tangent[1] < 0):
            tangent = -tangent
        half_len = min(length, 0.12) / 2.0
        p_a = mid - tangent * half_len
        p_b = mid + tangent * half_len
        shift = outward * width * 0.5
        use_dashed = getattr(self.config, 'laser_text_mode', False)
        text_layer = "cuts" if use_dashed else "text"
        plotpart.layers[text_layer] += Text(
            text,
            p_a + shift,
            p_b + shift,
            size=size,
            align="center",
            height=0.8,
            valign=0.0,
            dashed=use_dashed,
            dot_spacing=getattr(self.config, 'dot_spacing', 0.15),
        ).get_vectors()

    def _longest_free_run(self, plotpart):
        """(start, end, outward) of the longest straight free edge of the
        piece outline (the cut line minus the hem gives it back), or None."""
        outline = getattr(plotpart, "_band_outline", None)
        kinds = getattr(plotpart, "_band_kinds", None)
        if outline is None or kinds is None:
            return None
        n = len(outline)
        pts = [np.asarray(p, dtype=float) for p in outline]
        area = 0.0
        for i in range(n):
            area += pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
        ccw = area > 0
        best = None
        i = 0
        while i < n:
            if kinds[i] != "free":
                i += 1
                continue
            j = i
            direction = None
            while j < n and kinds[j] == "free":
                d = pts[(j + 1) % n] - pts[j]
                length = norm(d)
                if length < 1e-9:
                    j += 1
                    continue
                d = d / length
                if direction is not None and np.dot(direction, d) < np.cos(np.radians(5)):
                    break
                direction = d if direction is None else direction
                j += 1
            run_len = norm(pts[j % n] - pts[i])
            if best is None or run_len > best[0]:
                best = (run_len, i, j % n)
            i = max(j, i + 1)
        if best is None or best[0] < 1e-6:
            return None
        _, i, j = best
        d = pts[j] - pts[i]
        d = d / norm(d)
        outward = np.array([d[1], -d[0]]) if ccw else np.array([-d[1], d[0]])
        return pts[i], pts[j], outward

    def _flatten(self, attachment_points, num_folds):
        plotpart = PlotPart(material_code=self.drib.material_code, name=self.drib.name)

        pieces = self._band_split_pieces()
        if pieces:
            # Band-split element: the pieces are made by flatten_pieces();
            # keep a plain outline here for callers of flatten()
            first = pieces[0]
            cut = _offset_closed_polygon(first, self.config.allowance_general)
            plotpart.layers["cuts"].append(PolyLine2D(cut + [cut[0]]))

        elif num_folds > 0:
            alw2 = self.config.drib_allowance_folds
            cut_front = self.config.cut_diagonal_fold(-alw2, num_folds=num_folds)
            cut_back = self.config.cut_diagonal_fold(alw2, num_folds=num_folds)
            cut_front_result = cut_front.apply(
                [[self.left, 0], [self.right, 0]], self.left_out, self.right_out
            )
            cut_back_result = cut_back.apply(
                [[self.left, len(self.left) - 1], [self.right, len(self.right) - 1]],
                self.left_out,
                self.right_out,
            )

            plotpart.layers["cuts"] += [
                self.left_out[cut_front_result.index_left : cut_back_result.index_left]
                + cut_back_result.curve
                + self.right_out[
                    cut_front_result.index_right : cut_back_result.index_right : -1
                ]
                + cut_front_result.curve[::-1]
            ]

        else:
            p1 = next(
                self.left_out.cut(
                    self.left[0], self.right[0], startpoint=0, extrapolate=True
                )
            )[0]
            p2 = next(
                self.left_out.cut(
                    self.left[len(self.left) - 1],
                    self.right[len(self.right) - 1],
                    startpoint=len(self.left_out),
                    extrapolate=True,
                )
            )[0]
            p3 = next(
                self.right_out.cut(
                    self.left[0], self.right[0], startpoint=0, extrapolate=True
                )
            )[0]
            p4 = next(
                self.right_out.cut(
                    self.left[len(self.left) - 1],
                    self.right[len(self.right) - 1],
                    startpoint=len(self.right_out),
                    extrapolate=True,
                )
            )[0]

            outer = self.left_out[p1:p2]
            outer += self.right_out[p3:p4][::-1]
            outer += PolyLine2D([self.left_out[p1]])
            plotpart.layers["cuts"].append(outer)

        plotpart.layers["marks"].append(PolyLine2D([self.left[0], self.right[0]]))
        plotpart.layers["marks"].append(
            PolyLine2D([self.left[len(self.left) - 1], self.right[len(self.right) - 1]])
        )

        # Add front/back reference marks in the seam allowance
        # Front edge marks (at index 0)
        front_left = self.left_out[0]
        front_right = self.right_out[0]
        front_mid = (np.array(front_left) + np.array(front_right)) / 2
        
        # Back edge marks (at last index)
        back_left = self.left_out[len(self.left_out) - 1]
        back_right = self.right_out[len(self.right_out) - 1]
        back_mid = (np.array(back_left) + np.array(back_right)) / 2
        
        # Direction from front to back (for arrow orientation)
        direction = back_mid - front_mid
        dir_len = norm(direction)
        if dir_len > 1e-10:
            direction = direction / dir_len
        else:
            direction = np.array([0, 1])
        
        # Perpendicular direction
        perp = np.array([-direction[1], direction[0]])
        
        # Arrow size
        arrow_size = 0.005  # 5mm
        
        # Front mark: single arrow pointing forward (toward back)
        front_arrow_tip = front_mid + direction * arrow_size
        front_arrow_left = front_mid - direction * arrow_size * 0.5 + perp * arrow_size * 0.5
        front_arrow_right = front_mid - direction * arrow_size * 0.5 - perp * arrow_size * 0.5
        plotpart.layers["marks"].append(PolyLine2D([front_arrow_left, front_arrow_tip, front_arrow_right]))
        
        # Back mark: double line (two parallel lines)
        back_line1_start = back_mid - perp * arrow_size * 0.5
        back_line1_end = back_mid + perp * arrow_size * 0.5
        back_line2_start = back_mid - direction * arrow_size * 0.3 - perp * arrow_size * 0.5
        back_line2_end = back_mid - direction * arrow_size * 0.3 + perp * arrow_size * 0.5
        plotpart.layers["marks"].append(PolyLine2D([back_line1_start, back_line1_end]))
        plotpart.layers["marks"].append(PolyLine2D([back_line2_start, back_line2_end]))
        
        # Add dot marks at top of diagonal (in seam allowance on left and right sides)
        # These help identify front vs back when viewing the piece
        dot_size = 0.001  # 1mm dot represented as small cross
        
        # Find middle point along left edge (in seam allowance)
        left_mid_idx = len(self.left_out) // 2
        left_mid_inner = np.array(self.left[len(self.left) // 2])
        left_mid_outer = np.array(self.left_out[left_mid_idx])
        
        # Direction from inner to outer (into seam allowance)
        left_seam_dir = left_mid_outer - left_mid_inner
        left_seam_len = norm(left_seam_dir)
        if left_seam_len > 1e-10:
            left_seam_dir = left_seam_dir / left_seam_len
        
        # 2 dots toward front (left side = front in standard orientation)
        dot1_pos = left_mid_outer - left_seam_dir * 0.002  # 2mm from edge
        dot2_pos = left_mid_outer - left_seam_dir * 0.005  # 5mm from edge
        
        # Small cross for each dot
        for dot_pos in [dot1_pos, dot2_pos]:
            plotpart.layers["marks"].append(PolyLine2D([
                dot_pos + np.array([-dot_size, 0]),
                dot_pos + np.array([dot_size, 0])
            ]))
            plotpart.layers["marks"].append(PolyLine2D([
                dot_pos + np.array([0, -dot_size]),
                dot_pos + np.array([0, dot_size])
            ]))
        
        # Find middle point along right edge (in seam allowance)
        right_mid_idx = len(self.right_out) // 2
        right_mid_inner = np.array(self.right[len(self.right) // 2])
        right_mid_outer = np.array(self.right_out[right_mid_idx])
        
        # Direction from inner to outer (into seam allowance)
        right_seam_dir = right_mid_outer - right_mid_inner
        right_seam_len = norm(right_seam_dir)
        if right_seam_len > 1e-10:
            right_seam_dir = right_seam_dir / right_seam_len
        
        # 1 dot toward back (right side = back in standard orientation)
        dot3_pos = right_mid_outer - right_seam_dir * 0.003  # 3mm from edge
        plotpart.layers["marks"].append(PolyLine2D([
            dot3_pos + np.array([-dot_size, 0]),
            dot3_pos + np.array([dot_size, 0])
        ]))
        plotpart.layers["marks"].append(PolyLine2D([
            dot3_pos + np.array([0, -dot_size]),
            dot3_pos + np.array([0, dot_size])
        ]))

        plotpart.layers["stitches"] += [self.left, self.right]

        self._insert_attachment_points(plotpart, attachment_points)
        self._insert_text(plotpart)
        self._insert_diagonal_cone_holes(plotpart, attachment_points)
        self._insert_band_ellipse_holes(plotpart, attachment_points)
        self._insert_strap_holes(plotpart, attachment_points)

        return plotpart

    def _insert_strap_holes(self, plotpart, attachment_points=None):
        """Insert holes (ellipse or rounded rectangle) into an intrados tension strap.

        The strap is flattened into two edge curves ``self.left`` (on rib1) and
        ``self.right`` (on rib2). Holes are evenly distributed along the span
        (rib1 -> rib2) and centred across the strap width. Each hole is sized as
        a percentage of the per-hole span slot (width_pct) and of the strap
        width (height_pct). Matches ``_strap_holes_parametric`` used for the 3D mesh.
        """
        config = getattr(self.drib, 'strap_hole_config', None)
        if not config:
            return

        from numpy.linalg import norm

        num = int(config.get('num', 0))
        if num <= 0:
            return

        shape = config.get('shape', 0)          # 0 = ellipse, 1 = rounded rect
        width_pct = config.get('width_pct', 0.6)   # along span
        height_pct = config.get('height_pct', 0.6)  # across strap width
        corner_pct = config.get('corner_radius_pct', 0.25)

        inner = self.left    # curve on rib1 (front -> back), length = strap width
        outer = self.right   # curve on rib2

        inner_total = inner.get_length()
        outer_total = outer.get_length()
        if inner_total < 1e-9 or outer_total < 1e-9:
            return

        strap_width = (inner_total + outer_total) / 2.0

        # Mid-width points (t=0.5) on each edge define the span axis
        p_in_mid = np.array(inner[inner.walk(0, 0.5 * inner_total)])
        p_out_mid = np.array(outer[outer.walk(0, 0.5 * outer_total)])
        span_len = norm(p_out_mid - p_in_mid)
        if span_len < 1e-6:
            return

        slot = span_len / num
        hole_span_half = 0.5 * width_pct * slot        # along span
        hole_width_half = 0.5 * height_pct * strap_width  # across width
        if hole_span_half < 1e-4 or hole_width_half < 1e-4:
            return

        span_dir = (p_out_mid - p_in_mid) / span_len
        width_dir = np.array([-span_dir[1], span_dir[0]])

        for i in range(num):
            s = (i + 0.5) / num
            center = p_in_mid * (1.0 - s) + p_out_mid * s

            if shape == 1:
                pts = self._strap_rounded_rect(
                    center, span_dir, width_dir,
                    hole_span_half, hole_width_half, corner_pct)
            else:
                pts = []
                n = 32
                for j in range(n + 1):
                    a = 2 * np.pi * j / n
                    pt = (center
                          + hole_span_half * np.cos(a) * span_dir
                          + hole_width_half * np.sin(a) * width_dir)
                    pts.append(pt.tolist())

            plotpart.layers["cuts"].append(PolyLine2D(pts))

    @staticmethod
    def _strap_rounded_rect(center, u_dir, v_dir, half_u, half_v, corner_pct):
        """Rounded rectangle in the local (u_dir, v_dir) frame, returned as 2D points."""
        r = min(half_u, half_v) * max(0.0, min(corner_pct, 1.0))
        au = max(half_u - r, 0.0)
        av = max(half_v - r, 0.0)
        # corner centres in local coords (u, v) with sweep start angle
        corners = [
            (au, av, 0.0),
            (-au, av, np.pi / 2),
            (-au, -av, np.pi),
            (au, -av, 3 * np.pi / 2),
        ]
        per_corner = 8
        pts = []
        for ou, ov, base in corners:
            for k in range(per_corner):
                a = base + (np.pi / 2) * (k / (per_corner - 1))
                lu = ou + r * np.cos(a)
                lv = ov + r * np.sin(a)
                pt = center + lu * u_dir + lv * v_dir
                pts.append(pt.tolist())
        pts.append(pts[0])
        return pts

    def _insert_band_ellipse_holes(self, plotpart, attachment_points=None):
        """Insert elliptical holes into horizontal bands (both sides extrados).
        
        The band connects two diagonals. The diagonals have 2*num_zones holes
        (num_zones per side). The band gets the same number of ellipses,
        evenly distributed along its length. Ellipse width matches the max
        width of the neighboring diagonal holes at the extrados edge.
        """
        config = getattr(self.drib, 'band_hole_config', None)
        if not config:
            return
        
        from numpy.linalg import norm
        
        num_zones = config['num_zones']
        margin_side = config['margin_side_m']
        margin_top = config['margin_top_m']
        
        total_holes = 2 * num_zones  # num_zones per side of center axis
        
        # Flattened band curves
        inner = self.left
        outer = self.right
        
        inner_total = inner.get_length()
        outer_total = outer.get_length()
        if inner_total < 1e-9 or outer_total < 1e-9:
            return
        
        # Band width (distance between the two curves at midpoint)
        p_mid_inner = np.array(inner[inner.walk(0, 0.5 * inner_total)])
        p_mid_outer = np.array(outer[outer.walk(0, 0.5 * outer_total)])
        band_width = norm(p_mid_outer - p_mid_inner)
        
        if band_width < 2 * margin_top:
            return  # Band too narrow for holes
        
        # Ellipse semi-height = (band_width - 2*margin_top) / 2
        ellipse_h = (band_width - 2 * margin_top) / 2
        
        # Ellipse semi-width: match the max diagonal hole width at extrados
        # The band length roughly equals the extrados edge of the diagonal
        # Each zone gets an equal share of the band length
        usable_length = inner_total - 2 * margin_side
        if usable_length <= 0 or total_holes <= 0:
            return
        
        zone_width = usable_length / total_holes
        ellipse_w = max((zone_width - 2 * margin_side) / 2, 0.001)
        
        if ellipse_w < 0.001 or ellipse_h < 0.001:
            return
        
        for i in range(total_holes):
            # Center of this zone along the band (with margin at edges)
            t_center = (margin_side + zone_width * (i + 0.5)) / inner_total
            t_center = min(max(t_center, 0.01), 0.99)
            
            # Position on inner and outer curves
            ik_inner = inner.walk(0, t_center * inner_total)
            ik_outer = outer.walk(0, t_center * outer_total)
            p_inner = np.array(inner[ik_inner])
            p_outer = np.array(outer[ik_outer])
            
            center = (p_inner + p_outer) / 2
            
            # Tangent direction (along the band)
            dt = 0.01
            t_lo = max(0, t_center - dt)
            t_hi = min(1, t_center + dt)
            p_lo = np.array(inner[inner.walk(0, t_lo * inner_total)])
            p_hi = np.array(inner[inner.walk(0, t_hi * inner_total)])
            tangent = p_hi - p_lo
            tang_len = norm(tangent)
            if tang_len < 1e-9:
                continue
            tangent = tangent / tang_len
            
            # Normal (across the band width)
            normal = p_outer - p_inner
            norm_len = norm(normal)
            if norm_len < 1e-9:
                continue
            normal = normal / norm_len
            
            # Generate ellipse points
            n_pts = 30
            pts = []
            for j in range(n_pts + 1):
                angle = 2 * np.pi * j / n_pts
                pt = center + ellipse_w * np.cos(angle) * tangent + ellipse_h * np.sin(angle) * normal
                pts.append(pt.tolist() if isinstance(pt, np.ndarray) else list(pt))
            
            plotpart.layers["cuts"].append(PolyLine2D(pts))

    def _insert_diagonal_cone_holes(self, plotpart, attachment_points=None):
        """Insert cone-shaped holes into full (non-split) diagonals."""
        config = getattr(self.drib, 'cone_hole_config', None)
        if not config:
            return
        
        attachment_points = attachment_points or []
        
        num_zones = config['num_zones']
        margin_side = config['margin_side_m']
        margin_top = config['margin_top_m']
        margin_bottom = config['margin_bottom_m']
        corner_pct = config['corner_radius_pct']
        

        
        # Process each side: 0=left(rib1), 1=right(rib2)
        for side_idx in (0, 1):
            if side_idx == 0:
                front = self.drib.left_front
                back = self.drib.left_back
                rib = self.cell.rib1
                inner = self.left
                other_inner = self.right
            else:
                front = self.drib.right_front
                back = self.drib.right_back
                rib = self.cell.rib2
                inner = self.right
                other_inner = self.left
            
            if front[1] != -1:
                continue
            
            x_min = min(front[0], back[0])
            x_max = max(front[0], back[0])
            
            for ap in attachment_points:
                if not hasattr(ap, 'rib') or ap.rib is not rib:
                    continue
                if not hasattr(ap, 'rib_pos') or ap.rib_pos > 0.9:
                    continue
                
                if not (x_min <= ap.rib_pos <= x_max):
                    continue
                
                try:
                    self._create_diag_holes(
                        plotpart, ap, inner, other_inner,
                        front, back, rib, config
                    )
                except Exception:
                    pass

    def _create_diag_holes(self, plotpart, ap, inner, other_inner,
                            front, back, rib, config):
        """Create cone holes for one AP within the diagonal.
        
        Follows the SAME logic as rib cone holes:
        - Zone boundaries are linear interpolations between center and edges
        - Side edges of each hole are OFFSET from boundaries by margin_side
          (making adjacent hole edges parallel)
        - Inner boundary = circle at margin_bottom from AP
        - Outer boundary = outer curve offset by margin_top inward
        """
        num_zones = config['num_zones']
        margin_side = config['margin_side_m']
        margin_top = config['margin_top_m']
        margin_bottom = config['margin_bottom_m']
        corner_pct = config['corner_radius_pct']
        
        # AP fraction
        x_front, x_back = front[0], back[0]
        t_ap = 0.5 if abs(x_back - x_front) < 1e-9 else (ap.rib_pos - x_front) / (x_back - x_front)
        t_ap = min(max(t_ap, 0), 1)
        
        # Curve lengths
        inner_total = inner.get_length()
        other_total = other_inner.get_length()
        if inner_total < 1e-9 or other_total < 1e-9:
            return
        
        # AP position on inner curve
        ik_ap_inner = inner.walk(0, t_ap * inner_total)
        p_ap = np.array(inner[ik_ap_inner])
        
        # Center axis: AP → midpoint of outer curve at AP position
        ik_ap_outer = other_inner.walk(0, t_ap * other_total)
        p_center_outer = np.array(other_inner[ik_ap_outer])
        
        center_dir = p_center_outer - p_ap
        center_len = norm(center_dir)
        if center_len < 1e-9:
            return
        d_center = center_dir / center_len
        
        # Edge boundaries: front edge and back edge of the diagonal
        A = np.array(inner[0])                             # inner front
        B = np.array(other_inner[0])                       # outer front
        C = np.array(inner[len(inner) - 1])                # inner back
        D = np.array(other_inner[len(other_inner) - 1])    # outer back
        
        # Front edge direction (from AP toward front outer corner)
        front_dir = B - p_ap
        front_len = norm(front_dir)
        if front_len < 1e-9:
            return
        d_front = front_dir / front_len
        
        # Back edge direction (from AP toward back outer corner)
        back_dir = D - p_ap
        back_len = norm(back_dir)
        if back_len < 1e-9:
            return
        d_back = back_dir / back_len
        
        # Process each side (front and back of center axis)
        # Following rib cone hole convention: for each side,
        # "left" (t=0) is at center, "right" (t=1) is at edge.
        # But for back side, we swap to match the angular direction.
        for side_data in [
            (d_center, d_front),   # front side: center → front edge
            (d_center, d_back),    # back side: center → back edge
        ]:
            s_d_left_edge, s_d_right_edge = side_data
            
            for zone_i in range(num_zones):
                t0 = zone_i / num_zones
                t1 = (zone_i + 1) / num_zones
                
                # Zone boundary directions (interpolated between center and edge)
                d_left = (1 - t0) * s_d_left_edge + t0 * s_d_right_edge
                d_left = d_left / max(norm(d_left), 1e-9)
                d_right = (1 - t1) * s_d_left_edge + t1 * s_d_right_edge
                d_right = d_right / max(norm(d_right), 1e-9)
                
                # Use cross product to determine inward perpendicular direction
                cross = d_left[0] * d_right[1] - d_left[1] * d_right[0]
                
                if cross >= 0:
                    # d_left is CCW of d_right → zone interior is CCW
                    perp_left = np.array([-d_left[1], d_left[0]])
                    perp_right = np.array([d_right[1], -d_right[0]])
                else:
                    # d_left is CW of d_right → zone interior is CW
                    perp_left = np.array([d_left[1], -d_left[0]])
                    perp_right = np.array([-d_right[1], d_right[0]])
                
                # Offset the zone boundaries by margin_side perpendicular
                off_left_base = p_ap + perp_left * margin_side
                off_right_base = p_ap + perp_right * margin_side
                
                # Bottom corners: offset from AP by margin_bottom along each ray
                p_bl = off_left_base + d_left * margin_bottom
                p_br = off_right_base + d_right * margin_bottom
                
                # Check that bottom corners don't cross (V-shape detection)
                # Use the perpendicular of the average direction as reference
                d_avg = (d_left + d_right) / 2
                ref_vec = np.array([-d_avg[1], d_avg[0]]) if cross >= 0 else np.array([d_avg[1], -d_avg[0]])
                
                if np.dot(p_br - p_bl, ref_vec) <= 0:
                    # Crossed → compute single bottom point (V-shape)
                    dx = off_right_base - off_left_base
                    det_s = d_left[0]*(-d_right[1]) - d_left[1]*(-d_right[0])
                    if abs(det_s) < 1e-12:
                        continue
                    t_cross = (dx[0]*(-d_right[1]) - dx[1]*(-d_right[0])) / det_s
                    p_bottom = off_left_base + t_cross * d_left
                    p_bl = p_bottom
                    p_br = p_bottom
                
                # Top corners: find where offset rays hit the outer curve
                # Use the outer curve as a polyline and find intersection
                p_tl = self._ray_curve_intersect(
                    off_left_base, d_left, other_inner)
                p_tr = self._ray_curve_intersect(
                    off_right_base, d_right, other_inner)
                
                if p_tl is None or p_tr is None:
                    continue
                
                # Apply margin_top: pull inward along directon from AP
                d_tl_r = p_tl - p_ap
                d_tl_len = norm(d_tl_r)
                if d_tl_len > 1e-9:
                    p_tl = p_tl - (d_tl_r / d_tl_len) * margin_top
                
                d_tr_r = p_tr - p_ap
                d_tr_len = norm(d_tr_r)
                if d_tr_len > 1e-9:
                    p_tr = p_tr - (d_tr_r / d_tr_len) * margin_top
                
                # Validate positive area
                if np.dot(p_tr - p_tl, ref_vec) <= 0:
                    continue
                if np.dot(p_tl - p_bl, d_center) <= 0:
                    continue
                
                # Build hole with corner rounding
                is_v_shape = norm(p_bl - p_br) < 0.001
                
                if is_v_shape:
                    # V-shape: bottom apex + two sides + two top corners
                    p_bottom = p_bl
                    side_r = norm(p_tr - p_bottom)
                    side_l = norm(p_tl - p_bottom)
                    top_side = norm(p_tl - p_tr)
                    
                    if corner_pct > 1e-6 and side_r > 1e-6 and side_l > 1e-6:
                        dir_r = (p_tr - p_bottom) / max(side_r, 1e-9)
                        dir_l = (p_tl - p_bottom) / max(side_l, 1e-9)
                        
                        # Bottom apex rounding
                        cut_bot = min(side_r, side_l) * corner_pct * 0.5
                        bot_r = p_bottom + dir_r * cut_bot
                        bot_l = p_bottom + dir_l * cut_bot
                        
                        # Top-right corner rounding
                        cut_tr = min(side_r * corner_pct * 0.5, top_side * 0.4)
                        tr_from_bot = p_tr - dir_r * cut_tr
                        dir_top = (p_tl - p_tr) / max(top_side, 1e-9)
                        tr_to_top = p_tr + dir_top * cut_tr
                        
                        # Top-left corner rounding
                        cut_tl = min(side_l * corner_pct * 0.5, top_side * 0.4)
                        tl_from_top = p_tl - dir_top * cut_tl
                        tl_to_bot = p_tl - dir_l * cut_tl  # dir_l goes bottom→tl, so -dir_l goes tl→bottom
                        
                        hole_pts = []
                        # Bottom apex fillet
                        for fi in range(6):
                            t = fi / 5
                            hole_pts.append((1-t)**2 * bot_l + 2*(1-t)*t * p_bottom + t**2 * bot_r)
                        # Right side straight
                        hole_pts.append(tr_from_bot)
                        # Top-right corner fillet
                        for fi in range(6):
                            t = fi / 5
                            hole_pts.append((1-t)**2 * tr_from_bot + 2*(1-t)*t * p_tr + t**2 * tr_to_top)
                        # Top side straight
                        hole_pts.append(tl_from_top)
                        # Top-left corner fillet
                        for fi in range(6):
                            t = fi / 5
                            hole_pts.append((1-t)**2 * tl_from_top + 2*(1-t)*t * p_tl + t**2 * tl_to_bot)
                        # Left side straight back to start
                        hole_pts.append(hole_pts[0])
                    else:
                        hole_pts = [p_bottom, p_tr, p_tl, p_bottom]
                else:
                    # Normal quad with rounding on all 4 corners
                    hole_pts = self._round_quad_corners(
                        p_bl, p_br, p_tr, p_tl, corner_pct
                    )
                    hole_pts = hole_pts + [hole_pts[0]]
                
                if len(hole_pts) >= 3:
                    poly_pts = []
                    for p in hole_pts:
                        if isinstance(p, np.ndarray):
                            poly_pts.append(p.tolist())
                        else:
                            poly_pts.append(list(p))
                    plotpart.layers["cuts"].append(
                        PolyLine2D(poly_pts)
                    )
    
    def _ray_curve_intersect(self, ray_origin, ray_dir, curve):
        """Find where a ray (origin + t*dir, t>0) intersects a PolyLine2D."""
        normal = np.array([-ray_dir[1], ray_dir[0]])
        ref_val = np.dot(ray_origin, normal)
        
        best = None
        best_t = float('inf')
        
        for i in range(len(curve) - 1):
            p0 = np.array(curve[i])
            p1 = np.array(curve[i + 1])
            v0 = np.dot(p0, normal) - ref_val
            v1 = np.dot(p1, normal) - ref_val
            
            if v0 * v1 <= 0 and abs(v1 - v0) > 1e-12:
                s = v0 / (v0 - v1)
                if -1e-9 <= s <= 1 + 1e-9:
                    pt = p0 + s * (p1 - p0)
                    # Check that pt is in positive ray direction
                    t_ray = np.dot(pt - ray_origin, ray_dir)
                    if t_ray > -1e-9 and t_ray < best_t:
                        best = pt
                        best_t = t_ray
        
        return best
    


    
    def _round_corners_at_indices(self, pts, corner_indices, radius_pct):
        """Round specific corners of a polygon."""
        pts = [np.array(p) for p in pts]
        n = len(pts)
        result = []
        for i in range(n):
            if i not in corner_indices:
                result.append(pts[i])
                continue
            prev_pt = pts[(i - 1) % n]
            curr_pt = pts[i]
            next_pt = pts[(i + 1) % n]
            d_in = curr_pt - prev_pt
            d_out = next_pt - curr_pt
            len_in, len_out = norm(d_in), norm(d_out)
            if len_in < 1e-9 or len_out < 1e-9:
                result.append(curr_pt)
                continue
            offset = min(len_in * radius_pct, len_out * radius_pct,
                        len_in * 0.4, len_out * 0.4)
            if offset < 1e-6:
                result.append(curr_pt)
                continue
            p_start = curr_pt - d_in / len_in * offset
            p_end = curr_pt + d_out / len_out * offset
            for j in range(9):
                t = j / 8
                pt = (1-t)**2 * p_start + 2*(1-t)*t * curr_pt + t**2 * p_end
                result.append(pt)
        return result

    def _round_quad_corners(self, p1, p2, p3, p4, radius_pct, pts_per_corner=8):
        """Create a rounded quadrilateral from 4 corners.
        
        radius_pct: fraction applied to the AVERAGE of the two longest edges
        for the corner radius, ensuring visible rounding even when one edge
        is very short.
        """
        corners = [np.array(p1), np.array(p2), np.array(p3), np.array(p4)]
        n = len(corners)
        
        edge_lengths = sorted([norm(corners[(i+1) % n] - corners[i]) for i in range(n)])
        # Use average of two longest edges as reference (not min which may be tiny)
        ref_len = (edge_lengths[-1] + edge_lengths[-2]) / 2
        radius = ref_len * radius_pct
        
        if radius < 1e-6:
            return list(corners)
        
        points = []
        for i in range(n):
            prev_pt = corners[(i - 1) % n]
            curr_pt = corners[i]
            next_pt = corners[(i + 1) % n]
            
            d_in = curr_pt - prev_pt
            d_out = next_pt - curr_pt
            len_in, len_out = norm(d_in), norm(d_out)
            
            if len_in < 1e-9 or len_out < 1e-9:
                points.append(curr_pt)
                continue
            
            offset = min(radius, len_in * 0.4, len_out * 0.4)
            p_start = curr_pt - d_in / len_in * offset
            p_end = curr_pt + d_out / len_out * offset
            
            for j in range(pts_per_corner + 1):
                t = j / pts_per_corner
                pt = (1-t)**2 * p_start + 2*(1-t)*t * curr_pt + t**2 * p_end
                points.append(pt)
        
        return points


class StrapPlot(DribPlot):
    def flatten(self, attachment_points=None):
        return self._flatten(attachment_points, self.config.strap_num_folds)


class CellPlotMaker:
    run_check = True
    DefaultConf = PatternConfig
    DribPlot = DribPlot
    StrapPlot = StrapPlot
    PanelPlot = PanelPlot

    def __init__(self, cell, attachment_points, config=None):
        self.cell = cell
        self.attachment_points = attachment_points
        self.config = self.DefaultConf(config)

        self._flattened_cell = None

    def _get_flatten_cell(self):
        if self._flattened_cell is None:
            flattened_cell = self.cell.get_flattened_cell(self.config.midribs)

            left_bal, right_bal = flattened_cell["ballooned"]

            outer_left = left_bal.copy().add_stuff(-self.config.allowance_general)
            outer_right = right_bal.copy().add_stuff(self.config.allowance_general)

            outer_orig = [outer_left, outer_right]
            outer = [l.copy().check() for l in outer_orig]

            flattened_cell["outer"] = outer
            flattened_cell["outer_orig"] = outer_orig

            self._flattened_cell = flattened_cell

        return self._flattened_cell

    def get_panels(self, panels=None):
        cell_panels = []
        self.cell.calculate_3d_shaping(numribs=self.config.midribs)

        if panels is None:
            panels = self.cell.panels

        from openglider.glider.cell.polygon_panel import PolygonPanel

        for panel in panels:
            # Crossing-region panels flatten via the isometric developer (two
            # rails), not the strip PanelPlot which reads cut_front/cut_back.
            if isinstance(panel, PolygonPanel):
                cell_panels.append(self._flatten_polygon_panel(panel))
                continue

            # Check if this is a split panel with non-default y range
            y_start = getattr(panel, 'y_start', 0.0)
            y_end = getattr(panel, 'y_end', 1.0)
            
            if y_start == 0.0 and y_end == 1.0:
                # Full panel - use cached flattened cell
                flattened_cell = self._get_flatten_cell()
            else:
                # Split panel - generate specific flattened cell for this y range
                flattened_cell = self.cell.get_flattened_cell(
                    self.config.midribs, y_start=y_start, y_end=y_end
                )
                # Add outer with allowances
                left_bal, right_bal = flattened_cell["ballooned"]
                outer_left = left_bal.copy().add_stuff(-self.config.allowance_general)
                outer_right = right_bal.copy().add_stuff(self.config.allowance_general)
                outer_orig = [outer_left, outer_right]
                outer = [l.copy().check() for l in outer_orig]
                flattened_cell["outer"] = outer
                flattened_cell["outer_orig"] = outer_orig
            
            plot = self.PanelPlot(panel, self.cell, flattened_cell, self.config)
            dwg = plot.flatten(self.attachment_points)
            cell_panels.append(dwg)

        return cell_panels

    def _flatten_polygon_panel(self, panel):
        """Flatten a crossing-region PolygonPanel into a PlotPart, developed via
        the cell's isometric ``inner`` lines (profile-arc correct width).

        Seam allowance is applied per boundary, matching the strip-panel rules:
        each cut-side rail uses the allowance of ITS cut type (design / trailing
        edge / entry), and a spanwise end that reaches a rib uses the general
        (rib) allowance; an interior crossing (apex) end gets none.
        """
        import numpy as np

        from openglider.vector.polyline import PolyLine2D

        from openglider.glider.cell.polygon_panel import _dedup

        lo_rail, hi_rail, bottom_arc, top_arc, meta = panel._flatten_boundaries(
            self.cell, self.config.midribs + 2
        )

        # per-cut-type seam allowance (same table as the strip PanelPlot)
        cut_allowances = {
            "folded": self.config.allowance_entry_open,
            "parallel": self.config.allowance_trailing_edge,
            "orthogonal": self.config.allowance_design,
            "singleskin": self.config.allowance_entry_open,
            "cut_3d": self.config.allowance_design,
        }
        general = self.config.allowance_general

        # Panel centroid in the DEVELOPED plane. Every boundary point is offset
        # by its per-cut allowance AWAY from this centroid, so the seam
        # allowance always lands OUTSIDE the piece (never cuts into it),
        # regardless of each rail's point ordering / curvature.
        allpts = [np.asarray(p, float)
                  for p in list(lo_rail) + list(hi_rail)
                  + list(bottom_arc) + list(top_arc)]
        centroid = np.mean(allpts, axis=0)

        def _rep_amount(types):
            # representative allowance for a rail: the type is uniform per rail
            # in practice; use the mid-station type (fall back to general).
            if not types:
                return general
            return cut_allowances.get(types[len(types) // 2], general)

        def _offset_boundary(pts, amount):
            # Offset one boundary uniformly with the tested add_stuff (handles
            # miters/bevels), choosing the sign so the whole boundary moves
            # OUTWARD (its centroid ends up farther from the panel centroid).
            # A whole boundary lies on one side of the panel, so this per-
            # boundary test is unambiguous (unlike a per-vertex one).
            d = _dedup(pts)
            if len(d) < 2 or amount == 0:
                return [np.asarray(p, float) for p in d]
            pl = PolyLine2D([np.array(p) for p in d])
            plus = [np.asarray(p, float) for p in np.array(pl.copy().add_stuff(amount))]
            minus = [np.asarray(p, float) for p in np.array(pl.copy().add_stuff(-amount))]
            d_plus = np.linalg.norm(np.mean(plus, axis=0) - centroid)
            d_minus = np.linalg.norm(np.mean(minus, axis=0) - centroid)
            return plus if d_plus >= d_minus else minus

        # Offset each boundary outward by its allowance, then concatenate ONLY
        # the boundaries that carry a seam (allowance > 0 and not collapsed).
        # An interior crossing (apex) end has no seam: skipping its degenerate
        # point makes the two rails join directly, so the allowance never dips
        # back onto the sewing line there (no apex incursion) and no fragile
        # miter/clip is needed.
        raw = [
            (list(lo_rail), _rep_amount(meta["lo_types"])),
            (list(top_arc), general if meta["touches_rib2"] else 0.0),
            (list(hi_rail)[::-1], _rep_amount(meta["hi_types"])),
            (list(bottom_arc)[::-1], general if meta["touches_rib1"] else 0.0),
        ]
        cut = []
        for pts, amt in raw:
            if amt <= 0:
                continue
            off = _offset_boundary(pts, amt)
            if len(off) >= 2:
                cut.extend(off)
        cut = _dedup(cut)

        sewing = _dedup(list(lo_rail) + list(top_arc) + list(hi_rail[::-1]) + list(bottom_arc[::-1]))

        plotpart = PlotPart(material_code=panel.material_code, name=panel.name)
        plotpart.layers["cuts"] += [PolyLine2D(list(cut) + [cut[0]])]
        plotpart.layers["marks"] += [PolyLine2D(list(sewing) + [sewing[0]])]

        # ---- seam reference marks (repères de couture) ----------------------
        # Ticks where the cell's crossings cross each cut rail. Neighbouring
        # regions share the cut edge and develop it from the same inner grid,
        # so a tick at the same y lands at the same point on both pieces -> the
        # two panels align when sewn. Plus a connector line at each rib end.
        ys = meta.get("station_ys", [])

        def _rail_at(rail, y):
            for k in range(len(ys) - 1):
                if ys[k] <= y <= ys[k + 1]:
                    span = ys[k + 1] - ys[k]
                    t = 0.0 if span < 1e-12 else (y - ys[k]) / span
                    a = np.asarray(rail[k], float)
                    b = np.asarray(rail[k + 1], float)
                    return a + t * (b - a), (b - a)
            return None, None

        tick = 0.01  # 10 mm reference tick
        if ys:
            for y in getattr(panel, "crossings", []):
                if not (ys[0] < y < ys[-1]):
                    continue
                for rail in (lo_rail, hi_rail):
                    pt, tan = _rail_at(rail, y)
                    if pt is None:
                        continue
                    ln = float(np.hypot(tan[0], tan[1]))
                    if ln < 1e-9:
                        continue
                    perp = np.array([-tan[1], tan[0]]) / ln
                    plotpart.layers["marks"].append(
                        PolyLine2D([pt - perp * tick * 0.5, pt + perp * tick * 0.5]))
        return plotpart

    def get_panels_lower(self):
        panels = [p for p in self.cell.panels if p.is_lower()]
        return self.get_panels(panels)

    def get_panels_upper(self):
        panels = [p for p in self.cell.panels if not p.is_lower()]
        return self.get_panels(panels)

    def get_dribs(self):
        dribs = []
        for drib in self.cell.diagonals:
            drib_plot = self.DribPlot(drib, self.cell, self.config)
            # band-split elements give one independent piece per band
            dribs.extend(drib_plot.flatten_pieces(self.attachment_points))

        return dribs

    def get_straps(self):
        straps = []
        for strap in self.cell.straps:
            plot = self.StrapPlot(strap, self.cell, self.config)
            straps.append(plot.flatten(self.attachment_points))

        return straps

    def get_rigidfoils(self):
        rigidfoils = []
        for rigidfoil in self.cell.rigidfoils:
            rigidfoils.append(rigidfoil.get_flattened(self.cell))

        return rigidfoils

    def get_le_splits(self):
        """
        Get flattened patterns for LE panel spanwise splits.
        
        Checks if a panel touches the leading edge and if an LE split 
        is defined for this cell. Returns split patterns for both
        left and right half-panels.
        """
        
        le_splits = []
        
        # Check if cell has le_closures defined
        if not hasattr(self.cell, 'le_closures'):
            return le_splits
        
        for closure in self.cell.le_closures:
            # Generate left and right half-panel patterns
            for side in ["left", "right"]:
                plotpart = closure.get_flattened_plotpart(
                    self.cell, 
                    side, 
                    numribs=self.config.midribs,
                    seam_allowance=self.config.allowance_general
                )
                if plotpart is not None:
                    le_splits.append(plotpart)
        
        return le_splits
