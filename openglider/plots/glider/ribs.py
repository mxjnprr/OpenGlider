
import numpy as np

import openglider.glider
from openglider.airfoil import get_x_value
from openglider.plots import marks
from openglider.plots.glider.config import PatternConfig
from openglider.vector import PolyLine2D
from openglider.vector.drawing import PlotPart
from openglider.vector.functions import norm, rotation_2d
from openglider.vector.text import Text


class RibPlot:
    class DefaultConfig(PatternConfig):
        # allowance_general = 0.01
        # allowance_trailing_edge = 0.02

        marks_diagonal_front = marks.Inside(
            marks.Arrow(left=True, name="diagonal_front")
        )
        marks_diagonal_back = marks.Inside(
            marks.Arrow(left=False, name="diagonal_back")
        )
        marks_laser_diagonal = marks.Dot(0.8)
        marks_laser_attachment_point = marks.Dot(0.2, 0.8)

        marks_strap = marks.Inside(marks.Line(name="strap"))
        marks_attachment_point = marks.OnLine(
            marks.Rotate(marks.Cross(name="attachment_point"), np.pi / 4)
        )

        marks_controlpoint = marks.Dot(0.2)
        marks_panel_cut = marks.Line(name="panel_cut")
        rib_text_pos = -0.005

    def __init__(self, rib, config=None):
        self.rib = rib
        self.config = self.DefaultConfig(config)

        self.plotpart = self.x_values = self.inner = self.outer = None

    def flatten(self, glider):
        self.plotpart = PlotPart(
            name=self.rib.name, material_code=self.rib.material_code
        )
        prof2d = self.rib.get_hull(glider)
        self.x_values = prof2d.x_values
        self.inner = prof2d.copy().scale(self.rib.chord)
        self.outer = self.inner.copy().add_stuff(self.config.allowance_general)

        self._insert_attachment_points(glider.attachment_points)
        self.insert_holes()

        panel_cuts = set()
        for cell in glider.cells:
            if cell.rib1 == self.rib:
                # panel-cuts
                for panel in cell.panels:
                    if hasattr(panel, "cut_front"):
                        panel_cuts.add(panel.cut_front["left"])
                        panel_cuts.add(panel.cut_back["left"])
                    else:
                        # crossing-region PolygonPanel: chord bounds at rib1 (y=0)
                        try:
                            lo, hi = panel.region.chord_interval(0.0)
                            panel_cuts.add(lo)
                            panel_cuts.add(hi)
                        except Exception:
                            pass

                # diagonals
                for diagonal in cell.diagonals + cell.straps:
                    self.insert_drib_mark(diagonal, False)

            elif cell.rib2 == self.rib:
                for panel in cell.panels:
                    if hasattr(panel, "cut_front"):
                        panel_cuts.add(panel.cut_front["right"])
                        panel_cuts.add(panel.cut_back["right"])
                    else:
                        # crossing-region PolygonPanel: chord bounds at rib2 (y=1)
                        try:
                            lo, hi = panel.region.chord_interval(1.0)
                            panel_cuts.add(lo)
                            panel_cuts.add(hi)
                        except Exception:
                            pass

                for diagonal in cell.diagonals + cell.straps:
                    self.insert_drib_mark(diagonal, True)

        for cut in panel_cuts:
            # print(cut, self.marks_panel_cut)
            self.insert_mark(cut, self.config.marks_panel_cut)

        # rigidfoils
        for rigid in self.rib.rigidfoils:
            self.plotpart.layers["marks"].append(rigid.get_flattened(self.rib))

        for curve in self.rib.curves:
            self.plotpart.layers["marks"].append(curve.get_flattened(self.rib))

        # reinforcements (half-moon and rod sleeve)
        for reinforcement in self.rib.reinforcements:
            flat = reinforcement.get_flattened(self.rib, glider=glider)
            
            # Draw halfmoon outline
            if flat.get('halfmoon') and len(flat['halfmoon'].data) > 0:
                self.plotpart.layers["marks"].append(flat['halfmoon'])
                
                # Add text label if reinforcement has a name
                if reinforcement.name:
                    # Get center point of halfmoon for text placement
                    halfmoon_data = flat['halfmoon'].data
                    center_idx = len(halfmoon_data) // 4  # Top of the arc approx
                    if center_idx < len(halfmoon_data):
                        p1 = np.array(halfmoon_data[center_idx])
                        p2 = p1 + np.array([0.01, 0])  # Horizontal text
                        use_dashed = getattr(self.config, 'laser_text_mode', False)
                        _text = Text(reinforcement.name, p1, p2, size=0.008, valign=0,
                                     dashed=use_dashed,
                                     dot_spacing=getattr(self.config, 'dot_spacing', 0.15))
                        text_layer = "cuts" if use_dashed else "text"
                        self.plotpart.layers[text_layer] += _text.get_vectors()
            
            # Draw rod sleeve
            if flat.get('rod_sleeve') and len(flat['rod_sleeve'].data) > 0:
                self.plotpart.layers["marks"].append(flat['rod_sleeve'])
            
            # Add profile mark at reinforcement position
            self.insert_mark(reinforcement.position, self.config.marks_attachment_point)

        # Rod sleeves (profile extrados/intrados fourreaux) - add position marks
        if hasattr(self.rib, 'rod_sleeves') and self.rib.rod_sleeves:
            for sleeve in self.rib.rod_sleeves:
                try:
                    # Get start and end chord positions
                    start_chord = sleeve.start_chord
                    end_chord = sleeve.end_chord
                    
                    if sleeve.surface == 'extrados':
                        # Extrados uses negative x values in profile coordinate system
                        start_x = -start_chord
                        end_x = -end_chord
                    else:
                        # Intrados uses positive x values
                        start_x = start_chord
                        end_x = end_chord
                    
                    # Insert marks at start and end positions
                    # Use diagonal_front for start, diagonal_back for end
                    self.insert_mark(start_x, self.config.marks_diagonal_front)
                    self.insert_mark(end_x, self.config.marks_diagonal_back)

                    # Also add laser marks
                    self.insert_mark(start_x, self.config.marks_laser_diagonal, "L0")
                    self.insert_mark(end_x, self.config.marks_laser_diagonal, "L0")

                    # Draw the sleeve footprint on the rib so it can be sewn at
                    # the right place (consistent with reinforcement handling).
                    allowance = getattr(self.config, "allowance_rod_sleeve", 0.01)
                    footprint = sleeve.get_flattened(self.rib, glider=glider)
                    if footprint is not None and len(footprint.data) > 0:
                        self.plotpart.layers["marks"].append(footprint)

                    # Seam allowance (cut line) around the sleeve footprint
                    seam = sleeve.get_seam_allowance(self.rib, glider=glider, allowance=allowance)
                    if seam is not None and len(seam.data) > 0:
                        self.plotpart.layers["marks"].append(seam)

                    # Corner reference points on the seam-allowance (cut) line
                    # -- including the two at the sleeve end -- so the fabric
                    # can be positioned exactly before starting the seam.
                    for corner in sleeve.get_corner_points(self.rib, glider=glider, allowance=allowance):
                        self.plotpart.layers["marks"] += self._cross_marks(corner, size=0.005)
                        self.plotpart.layers["L0"] += self._cross_marks(corner, size=0.005)

                    # Mounting points spaced along the rod
                    for mp in sleeve.get_mounting_points(self.rib, glider=glider):
                        self.plotpart.layers["marks"] += self._cross_marks(mp, size=0.003)
                        self.plotpart.layers["L0"] += self._cross_marks(mp, size=0.003)
                except Exception as e:
                    print(f"Failed to add rod sleeve marks: {e}")

        self._insert_text(self.rib.name)
        self.insert_controlpoints()

        # insert cut
        self.draw_rib(glider)
        self.plotpart.layers["stitches"].append(self.inner)

        return self.plotpart


    def _get_inner_outer(self, x_value):
        ik = get_x_value(self.x_values, x_value)

        # ik = get_x_value(self.x_values, position)
        inner = self.inner[ik]
        outer = (
            inner
            + PolyLine2D(self.inner).get_normal(ik) * self.config.allowance_general
        )
        # inner = self.inner[ik]
        # outer = self.outer[ik]
        return inner, outer

    @staticmethod
    def _cross_marks(point, size=0.004):
        """Return a small '+' cross (two PolyLine2D segments) centered on point."""
        p = np.array(point, dtype=float)
        return [
            PolyLine2D([p + [-size, 0.0], p + [size, 0.0]]),
            PolyLine2D([p + [0.0, -size], p + [0.0, size]]),
        ]

    def insert_mark(self, position, mark_function, layer="marks"):
        if hasattr(mark_function, "__func__"):
            mark_function = mark_function.__func__

        if mark_function is None:
            return

        inner, outer = self._get_inner_outer(position)

        self.plotpart.layers[layer] += mark_function(inner, outer)

    def insert_controlpoints(self):
        for x in self.config.distribution_controlpoints:
            self.insert_mark(x, self.config.marks_controlpoint, layer="marks")
            self.insert_mark(x, self.config.marks_laser_controlpoint, layer="L0")

    def get_point(self, x, y=-1):
        assert x >= 0
        p = self.rib.profile_2d.profilepoint(x, y)
        return p * self.rib.chord

    def insert_drib_mark(self, drib, right=False):
        if right:
            p1 = drib.right_front
            p2 = drib.right_back
        else:
            p1 = drib.left_front
            p2 = drib.left_back

        if p1[1] == p2[1] == -1:
            # Intrados surface - just mark the positions
            self.insert_mark(p1[0], self.config.marks_diagonal_front)
            self.insert_mark(p2[0], self.config.marks_diagonal_back)
            self.insert_mark(p1[0], self.config.marks_laser_diagonal, "L0")
            self.insert_mark(p2[0], self.config.marks_laser_diagonal, "L0")
        elif p1[1] == p2[1] == 1:
            # Extrados surface - just mark the positions
            self.insert_mark(-p1[0], self.config.marks_diagonal_back)
            self.insert_mark(-p2[0], self.config.marks_diagonal_front)
            self.insert_mark(-p1[0], self.config.marks_laser_diagonal, "L0")
            self.insert_mark(-p2[0], self.config.marks_laser_diagonal, "L0")
        elif p1[1] == p2[1] and abs(p1[1]) > 0.5:
            # Offset case - draw a curved line with TRUE PERPENDICULAR offset from extrados
            height = p1[1]
            x_start = min(abs(p1[0]), abs(p2[0]))
            x_end = max(abs(p1[0]), abs(p2[0]))
            
            try:
                profile = self.rib.profile_2d
                chord = self.rib.chord
                
                # Calculate the offset distance in real units
                mid_x = (x_start + x_end) / 2
                try:
                    upper_pt = profile.profilepoint(mid_x, h=1.0)
                    lower_pt = profile.profilepoint(mid_x, h=-1.0)
                    actual_thickness = (upper_pt[1] - lower_pt[1]) * chord
                except:
                    actual_thickness = chord * 0.12
                
                offset_distance = (1.0 - abs(height)) * actual_thickness / 2
                
                # Generate extrados curve points and offset them perpendicular
                num_points = 30
                curve_points = []
                
                for i in range(num_points + 1):
                    t = i / num_points
                    x = x_start + (x_end - x_start) * t
                    
                    # Get extrados point at this x
                    ext_pt = np.array(profile.profilepoint(x, h=1.0)) * chord
                    
                    # Calculate normal by getting nearby points
                    eps = 0.001
                    if x - eps >= 0:
                        pt_prev = np.array(profile.profilepoint(x - eps, h=1.0)) * chord
                    else:
                        pt_prev = ext_pt
                    if x + eps <= 1:
                        pt_next = np.array(profile.profilepoint(x + eps, h=1.0)) * chord
                    else:
                        pt_next = ext_pt
                    
                    # Tangent vector
                    tangent = pt_next - pt_prev
                    tangent_len = norm(tangent)
                    if tangent_len > 1e-10:
                        tangent = tangent / tangent_len
                    else:
                        tangent = np.array([1, 0])
                    
                    # Normal perpendicular to tangent
                    # For extrados, we want to go INWARD (lower y = toward intrados)
                    # extrados has positive y, so normal pointing down is (-tangent_y, tangent_x) rotated
                    # Actually: rotate -90 degrees: (x,y) -> (y, -x)
                    normal = np.array([-tangent[1], tangent[0]])
                    
                    # Offset the point inward (toward intrados = negative y direction)
                    offset_pt = ext_pt - normal * offset_distance
                    curve_points.append(offset_pt)
                
                # Add the curved line to marks
                self.plotpart.layers["marks"].append(
                    PolyLine2D(curve_points, name=drib.name)
                )
                
                # Also add arrow marks at the start and end positions
                self.insert_mark(-x_start, self.config.marks_diagonal_back)
                self.insert_mark(-x_end, self.config.marks_diagonal_front)
                self.insert_mark(-x_start, self.config.marks_laser_diagonal, "L0")
                self.insert_mark(-x_end, self.config.marks_laser_diagonal, "L0")
                
            except Exception as e:
                # Fallback to just arrow marks if curve fails
                print(f"DEBUG: insert_drib_mark exception: {e}")
                import traceback
                traceback.print_exc()
                if height > 0:
                    self.insert_mark(-x_start, self.config.marks_diagonal_back)
                    self.insert_mark(-x_end, self.config.marks_diagonal_front)
                else:
                    self.insert_mark(x_start, self.config.marks_diagonal_front)
                    self.insert_mark(x_end, self.config.marks_diagonal_back)
        else:
            # Fallback - straight line between two points
            p1_pt = self.get_point(*p1)
            p2_pt = self.get_point(*p2)
            self.plotpart.layers["marks"].append(PolyLine2D([p1_pt, p2_pt], name=drib.name))

    def insert_holes(self):
        for hole in self.rib.holes:
            poly = hole.get_flattened(self.rib)
            self.plotpart.layers["cuts"].append(poly)
        # Cone holes are stored separately to avoid Triangle meshing issues
        cone_holes = getattr(self.rib, 'cone_holes', [])
        for hole in cone_holes:
            poly = hole.get_flattened(self.rib)
            self.plotpart.layers["cuts"].append(poly)

    def draw_rib(self, glider):
        """
        Cut trailing edge of outer rib
        """
        outer_rib = self.outer
        inner_rib = self.inner
        t_e_allowance = self.config.allowance_trailing_edge
        p1 = inner_rib[0] + [0, 1]
        p2 = inner_rib[0] + [0, -1]
        cuts = outer_rib.cut(p1, p2, extrapolate=True)

        start = next(cuts)[0]
        stop = next(cuts)[0]

        contour = PolyLine2D([])

        buerzl = PolyLine2D(
            [
                outer_rib[stop],
                outer_rib[stop] + [t_e_allowance, 0],
                outer_rib[start] + [t_e_allowance, 0],
                outer_rib[start],
            ]
        )

        contour += PolyLine2D(outer_rib[start:stop])
        contour += buerzl

        self.plotpart.layers["cuts"] += [contour]

    def _insert_attachment_points(self, points):
        for attachment_point in points:
            if hasattr(attachment_point, "rib") and attachment_point.rib == self.rib:
                self.insert_mark(
                    attachment_point.rib_pos, self.config.marks_attachment_point
                )
                self.insert_mark(
                    attachment_point.rib_pos,
                    self.config.marks_laser_attachment_point,
                    "L0",
                )

    def _insert_text(self, text):
        inner, outer = self._get_inner_outer(self.config.rib_text_pos)
        diff = outer - inner

        use_dashed = getattr(self.config, 'laser_text_mode', False)
        p1 = inner + diff / 2
        p2 = p1 + rotation_2d(np.pi / 2).dot(diff)

        # Text size: 50% of allowance width, but max 8mm
        text_size = min(norm(outer - inner) * 0.5, 0.008)
        _text = Text(text, p1, p2, size=text_size, valign=0, dashed=use_dashed,
                    dot_spacing=getattr(self.config, 'dot_spacing', 0.15))
        text_layer = "cuts" if use_dashed else "text"
        self.plotpart.layers[text_layer] += _text.get_vectors()


class SingleSkinRibPlot(RibPlot):
    skin_cut = None

    def _get_inner_outer(self, x_value):
        # TODO: shift when after the endpoint
        inner, outer = super()._get_inner_outer(x_value)

        if self.skin_cut is None or x_value < self.skin_cut:
            return inner, outer
        else:
            return inner, inner + (inner - outer)

    def _get_singleskin_cut(self, glider):
        if self.skin_cut is None:
            singleskin_cut = None

            for cell in glider.cells:
                # asserts first cut never is a singlesking cut!
                # asserts there is only one removed singleskin Panel!
                # maybe asserts no singleskin rib on stabilo
                if cell.rib1 == self.rib:
                    for panel in cell.panels:
                        if panel.cut_back["type"] == "singleskin":
                            singleskin_cut = panel.cut_back["left"]
                            break
                if cell.rib2 == self.rib:
                    for panel in cell.panels:
                        if panel.cut_back["type"] == "singleskin":
                            singleskin_cut = panel.cut_back["right"]
                            break

            self.skin_cut = singleskin_cut

        return self.skin_cut

    def insert_holes(self):
        """Insert holes using the hull profile so they fit within the hull outline.

        apply_holes() computes hole positions using rib.profile_2d (aero profile).
        The rib outline uses get_hull() which has bows pushing the intrados upward.
        Holes computed in aero space therefore extend below the hull intrados boundary
        and are invisible in the pattern.  Fix: temporarily swap profile_2d to the
        cached hull profile and clear stored available_height so get_points() measures
        the hole height against hull thickness rather than aero thickness.
        """
        hull = getattr(self.rib, '_hull_profile', None)
        if hull is None:
            # Hull not yet cached (shouldn't happen after flatten) — fall back.
            super().insert_holes()
            return

        orig_profile = self.rib.profile_2d
        self.rib.profile_2d = hull
        try:
            for hole in self.rib.holes:
                # Temporarily clear available_height so get_points() recomputes it
                # from hull local_thickness instead of the stored aero-based value.
                orig_ah = hole.available_height
                hole.available_height = None
                try:
                    poly = hole.get_flattened(self.rib)
                    self.plotpart.layers["cuts"].append(poly)
                finally:
                    hole.available_height = orig_ah
        finally:
            self.rib.profile_2d = orig_profile

        # Cone holes use custom_points in normalized coords — draw them as-is.
        for hole in getattr(self.rib, 'cone_holes', []):
            poly = hole.get_flattened(self.rib)
            self.plotpart.layers["cuts"].append(poly)

    def flatten(self, glider):
        self._get_singleskin_cut(glider)
        return super().flatten(glider)

    def draw_rib(self, glider):
        """
        Cut trailing edge of outer rib
        """
        outer_rib = self.outer
        inner_rib = self.inner
        t_e_allowance = self.config.allowance_trailing_edge
        p1 = inner_rib[0] + [0, 1]
        p2 = inner_rib[0] + [0, -1]
        cuts = outer_rib.cut(p1, p2, extrapolate=True)

        start = next(cuts)[0]
        stop = next(cuts)[0]

        contour = PolyLine2D([])

        if isinstance(self.rib, openglider.glider.rib.SingleSkinRib):
            # outer is going from the back back until the singleskin cut

            singleskin_cut_left = self._get_singleskin_cut(glider)

            if singleskin_cut_left is not None:
                single_skin_cut = self.rib.profile_2d(singleskin_cut_left)

                buerzl = PolyLine2D(
                    [
                        inner_rib[0],
                        inner_rib[0] + [t_e_allowance, 0],
                        outer_rib[start] + [t_e_allowance, 0],
                        outer_rib[start],
                    ]
                )
                contour += PolyLine2D(outer_rib[start:single_skin_cut])
                contour += PolyLine2D(inner_rib[single_skin_cut:stop])
                contour += buerzl
            else:
                # No singleskin cut found — treat like a regular rib
                buerzl = PolyLine2D(
                    [
                        outer_rib[stop],
                        outer_rib[stop] + [t_e_allowance, 0],
                        outer_rib[start] + [t_e_allowance, 0],
                        outer_rib[start],
                    ]
                )
                contour += PolyLine2D(outer_rib[start:stop])
                contour += buerzl


        else:
            buerzl = PolyLine2D(
                [
                    outer_rib[stop],
                    outer_rib[stop] + [t_e_allowance, 0],
                    outer_rib[start] + [t_e_allowance, 0],
                    outer_rib[start],
                ]
            )

            contour += PolyLine2D(outer_rib[start:stop])
            contour += buerzl

        self.plotpart.layers["cuts"] += [contour]
