import collections

from openglider.vector.drawing import Layout
from openglider.plots.glider.cell import CellPlotMaker
from openglider.plots.glider.ribs import RibPlot, SingleSkinRibPlot
from openglider.plots.glider.config import PatternConfig, OtherPatternConfig


class PlotMaker(object):
    CellPlotMaker = CellPlotMaker
    RibPlot = RibPlot
    DefaultConfig = OtherPatternConfig

    def __init__(self, glider_3d, config=None):
        self.glider_3d = glider_3d
        self.config = self.DefaultConfig(config)

        self.panels = Layout()
        self.dribs = collections.OrderedDict()
        self.straps = collections.OrderedDict()
        self.rigidfoils = collections.OrderedDict()
        self.le_splits = collections.OrderedDict()
        self.reinforcements = []  # Halfmoons and attachment rod sleeves
        self.rod_sleeves = []  # Profile rod sleeves (extrados/intrados)
        self.miniribs = []  # Mini ribs patterns
        self.ribs = []
        self._cellplotmakers = dict()

    def __json__(self):
        return {
            "glider3d": self.glider_3d,
            "config": self.config,
            "panels": self.panels,
            # "dribs": self.dribs,
            "ribs": self.ribs,
        }

    @classmethod
    def __from_json__(cls, dct):
        ding = cls(dct["glider3d"], dct["config"])
        ding.panels = dct["panels"]
        ding.ribs = dct["ribs"]
        # ding.dribs = dct["dribs"]

        return ding

    def _get_cellplotmaker(self, cell):
        if cell not in self._cellplotmakers:
            self._cellplotmakers[cell] = self.CellPlotMaker(
                cell, self.glider_3d.attachment_points, self.config
            )

        return self._cellplotmakers[cell]

    def _get_text_position_inside(self, points, offset_ratio=0.15):
        """
        Get a text position inside a closed polygon near the top edge.
        Returns (p1, p2) for text placement.
        """
        import numpy as np
        
        if len(points) < 3:
            center = np.mean(points, axis=0) if len(points) > 0 else np.array([0, 0])
            return center, center + np.array([0.01, 0])
        
        points = np.array(points)
        
        # Find bbox
        min_pt = np.min(points, axis=0)
        max_pt = np.max(points, axis=0)
        width = max_pt[0] - min_pt[0]
        height = max_pt[1] - min_pt[1]
        
        # Position text in upper-center area of the shape
        center_x = (min_pt[0] + max_pt[0]) / 2
        # Place text at offset_ratio from top
        text_y = max_pt[1] - height * offset_ratio
        
        p1 = np.array([center_x - width * 0.3, text_y])
        p2 = np.array([center_x + width * 0.3, text_y])
        
        return p1, p2

    def get_panels(self):
        self.panels.clear()
        panels_upper = []
        panels_lower = []

        for cell in self.glider_3d.cells:
            pm = self._get_cellplotmaker(cell)
            lower = pm.get_panels_lower()
            upper = pm.get_panels_upper()
            panels_lower.append(
                Layout.stack_column(lower, self.config.patterns_align_dist_y)
            )
            panels_upper.append(
                Layout.stack_column(upper, self.config.patterns_align_dist_y)
            )

        if self.config.layout_seperate_panels:
            layout_lower = Layout.stack_row(
                panels_lower, self.config.patterns_align_dist_x
            )
            layout_lower.rotate(180, radians=False)
            layout_upper = Layout.stack_row(
                panels_upper, self.config.patterns_align_dist_x
            )

            self.panels = Layout.stack_row(
                [layout_lower, layout_upper], 2 * self.config.patterns_align_dist_x
            )

        else:
            self.panels = Layout.stack_grid(
                [panels_upper, panels_lower],
                self.config.patterns_align_dist_x,
                self.config.patterns_align_dist_y,
            )

        return self.panels

    def get_ribs(self, rotate=False):
        from openglider.glider.rib.rib import SingleSkinRib

        self.ribs = []
        for rib in self.glider_3d.ribs:
            if isinstance(rib, SingleSkinRib):
                rib_plot = SingleSkinRibPlot(rib)
            else:
                rib_plot = self.RibPlot(rib, self.config)

            rib_plot.flatten(self.glider_3d)
            if rotate:
                rib_plot.plotpart.rotate(90, radians=False)
            self.ribs.append(rib_plot.plotpart)

    def get_dribs(self):
        self.dribs.clear()
        for cell in self.glider_3d.cells:
            # missing attachmentpoints []
            dribs = self._get_cellplotmaker(cell).get_dribs()
            self.dribs[cell] = dribs

        return self.dribs

    def get_straps(self):
        self.straps.clear()
        for cell in self.glider_3d.cells:
            # missing attachmentpoints []
            straps = self._get_cellplotmaker(cell).get_straps()
            self.straps[cell] = straps

        return self.straps

    def get_rigidfoils(self):
        # TODO: rib rigids
        self.rigidfoils.clear()

        for cell in self.glider_3d.cells:
            rigidfoils = self._get_cellplotmaker(cell).get_rigidfoils()
            self.rigidfoils[cell] = rigidfoils

        return self.rigidfoils

    def get_le_splits(self):
        """Get LE panel spanwise split patterns for 2D export."""
        self.le_splits.clear()

        for cell in self.glider_3d.cells:
            le_splits = self._get_cellplotmaker(cell).get_le_splits()
            if le_splits:
                self.le_splits[cell] = le_splits

        return self.le_splits

    def get_reinforcements(self):
        """Get attachment reinforcement parts (halfmoons and their rod sleeves) for 2D export."""
        from openglider.vector.drawing import PlotPart
        from openglider.vector.text import Text
        import numpy as np
        
        self.reinforcements = []
        
        for rib_idx, rib in enumerate(self.glider_3d.ribs):
            if hasattr(rib, "reinforcements") and rib.reinforcements:
                for reinf_idx, reinforcement in enumerate(rib.reinforcements):
                    try:
                        flat = reinforcement.get_flattened(rib, glider=self.glider_3d)
                        # Short name: rib number + row letter (A, B, C...)
                        row_letter = chr(ord('A') + reinf_idx)
                        unique_name = f"{rib_idx+1}{row_letter}"
                        
                        # Halfmoon part
                        if flat.get('halfmoon') and len(flat['halfmoon'].data) > 0:
                            halfmoon_part = PlotPart(
                                name=unique_name,
                                material_code="reinforcement"
                            )
                            halfmoon_part.layers["cuts"].append(flat['halfmoon'])
                            
                            # Text along the outer edge (bottom/intrados line)
                            # Halfmoon polygon = outer_points + reversed(inner_points) + [close]
                            # First half = outer_points = profile/intrados edge = BOTTOM
                            pts = np.array(flat['halfmoon'].data)
                            n = len(pts)
                            outer_end = n // 2 - 1  # last index of outer points
                            if outer_end > 3:
                                # Corner = last outer point (where profile meets arc)
                                corner = pts[outer_end]
                                centroid = np.mean(pts[:-1], axis=0)
                                
                                # 20% from corner toward centroid (scales with shape size)
                                p_base = corner + (centroid - corner) * 0.20
                                
                                # Tangent of outer edge (bottom curve) near the corner
                                prev_idx = max(0, outer_end - 3)
                                tangent = pts[outer_end] - pts[prev_idx]
                                tlen = np.linalg.norm(tangent)
                                if tlen > 1e-10:
                                    tangent = tangent / tlen
                                else:
                                    tangent = np.array([1, 0])
                                # Ensure text reads left-to-right (readable)
                                if tangent[0] < 0:
                                    tangent = -tangent
                                
                                # Text span proportional to shape (~10% of corner→centroid)
                                span = np.linalg.norm(centroid - corner) * 0.10
                                p1 = p_base - tangent * span
                                p2 = p_base + tangent * span
                            else:
                                # Fallback: centroid
                                centroid = np.mean(pts, axis=0)
                                p1 = centroid
                                p2 = centroid + np.array([0.02, 0])
                            
                            use_dashed = getattr(self.config, 'laser_text_mode', False)
                            text_layer = "cuts" if use_dashed else "text"
                            text_obj = Text(unique_name, p1, p2, size=0.005, valign=0.5,
                                           dashed=use_dashed,
                                           dot_spacing=getattr(self.config, 'dot_spacing', 0.15))
                            halfmoon_part.layers[text_layer] += text_obj.get_vectors()
                            
                            self.reinforcements.append(halfmoon_part)
                        
                        # Rod sleeve part (for attachments)
                        if flat.get('rod_sleeve') and len(flat['rod_sleeve'].data) > 0:
                            sleeve_part = PlotPart(
                                name=unique_name + "_sleeve",
                                material_code="rod_sleeve"
                            )
                            sleeve_part.layers["cuts"].append(flat['rod_sleeve'])
                            
                            # Text perpendicular to the left extremity
                            # Rod sleeve polygon = outer + reversed(inner) + [close]
                            # First point = outer[0] = left end of outer curve
                            pts = np.array(flat['rod_sleeve'].data)
                            n = len(pts)
                            
                            # Find narrowest point = extremity (leftmost or rightmost)
                            # Use first few points to get the tangent at the start
                            p_start = pts[0]
                            # Next point along the boundary
                            p_next = pts[min(2, n-1)]
                            tangent = p_next - p_start
                            tlen = np.linalg.norm(tangent)
                            if tlen > 1e-10:
                                tangent = tangent / tlen
                            else:
                                tangent = np.array([0, 1])
                            
                            # At the extremity, tangent crosses the crescent width
                            # = perpendicular to the crescent length = what user wants
                            # Ensure tangent points inward (toward centroid)
                            centroid = np.mean(pts[:-1], axis=0)
                            if np.dot(tangent, centroid - p_start) < 0:
                                tangent = -tangent
                            
                            # Center between inner and outer curves at the extremity
                            # pts[0] = outer[0], pts[-2] = inner[0] (before close point)
                            p_mid = (pts[0] + pts[-2]) / 2
                            
                            # Place text from midpoint, perpendicular to crescent
                            p1 = p_mid + tangent * 0.001
                            p2 = p1 + tangent * 0.02
                            
                            use_dashed = getattr(self.config, 'laser_text_mode', False)
                            text_layer = "cuts" if use_dashed else "text"
                            text_obj = Text(unique_name, p1, p2, size=0.003, valign=0.5,
                                           dashed=use_dashed,
                                           dot_spacing=getattr(self.config, 'dot_spacing', 0.15))
                            sleeve_part.layers[text_layer] += text_obj.get_vectors()
                            
                            self.reinforcements.append(sleeve_part)
                            
                    except Exception as e:
                        print(f"Failed to plot reinforcement: {e}")
        
        return self.reinforcements

    def get_miniribs(self):
        """Get miniribs patterns for 2D export."""
        from openglider.vector.drawing import PlotPart
        from openglider.vector.text import Text
        from openglider.vector.functions import rotation_2d, norm
        import numpy as np
        
        self.miniribs = []
        
        for cell_idx, cell in enumerate(self.glider_3d.cells):
            if hasattr(cell, "miniribs"):
                for mr_idx, mr in enumerate(cell.miniribs):
                    try:
                        # Get flattened shape with seam allowance
                        inner, outer = mr.get_flattened_with_allowance(
                            cell, 
                            allowance=self.config.allowance_general
                        )
                        if outer is None:
                            continue
                        
                        # Create unique name: MR_cell_index
                        unique_name = f"MR_{cell_idx+1}_{mr_idx+1}"
                        
                        # Create PlotPart with proper layers
                        part = PlotPart(
                            name=unique_name,
                            material_code="miniribs"
                        )
                        
                        # Outer = cut line, Inner = stitch line
                        part.layers["cuts"].append(outer)
                        # Strip name from inner line to avoid parasitic "line name" in exports
                        inner_copy = inner.copy()
                        inner_copy.name = None
                        part.layers["stitches"].append(inner_copy)
                        
                        # Add hole contours to cuts layer
                        hole_contours = mr.get_hole_contours_2d(cell)
                        for hole_contour in hole_contours:
                            part.layers["cuts"].append(hole_contour)
                        
                        # Add text label in the seam allowance area
                        inner_pts = list(inner.data)
                        outer_pts = list(outer.data)
                        if len(inner_pts) > 4 and len(outer_pts) > 4:
                            # Take points from the leading edge area
                            idx = len(inner_pts) // 8
                            p_inner = np.array(inner_pts[idx])
                            p_outer = np.array(outer_pts[idx])
                            
                            # Text position between inner and outer
                            text_center = (p_inner + p_outer) / 2
                            diff = p_outer - p_inner
                            
                            # Create perpendicular direction for text
                            p1 = text_center
                            p2 = text_center + rotation_2d(np.pi / 2).dot(diff)
                            
                            # Text size: 80% of allowance, max 8mm
                            text_size = min(norm(diff) * 0.8, 0.008)
                            use_dashed = getattr(self.config, 'laser_text_mode', False)
                            text_obj = Text(unique_name, p1, p2, size=text_size, valign=0,
                                           dashed=use_dashed,
                                           dot_spacing=getattr(self.config, 'dot_spacing', 0.15))
                            text_layer = "cuts" if use_dashed else "text"
                            part.layers[text_layer] += text_obj.get_vectors()
                        
                        self.miniribs.append(part)
                    except Exception as e:
                        print(f"Failed to plot minirib: {e}")
        
        return self.miniribs

    def get_rod_sleeves(self):
        """Get profile rod sleeves (extrados/intrados) for 2D export in separate frame."""
        from openglider.vector.drawing import PlotPart
        from openglider.vector.text import Text
        import numpy as np
        
        self.rod_sleeves = []
        
        for rib_idx, rib in enumerate(self.glider_3d.ribs):
            if hasattr(rib, "rod_sleeves") and rib.rod_sleeves:
                for sleeve_idx, sleeve in enumerate(rib.rod_sleeves):
                    try:
                        flat = sleeve.get_flattened(rib, glider=self.glider_3d)
                        surface_label = "E" if sleeve.surface == 'extrados' else "I"
                        unique_name = f"{rib.name}_{surface_label}{sleeve_idx+1}"
                        
                        if flat is not None and hasattr(flat, 'data') and len(flat.data) > 0:
                            sleeve_part = PlotPart(
                                name=unique_name,
                                material_code=f"{sleeve.surface}_sleeve"
                            )
                            sleeve_part.layers["cuts"].append(flat)
                            
                            # Text perpendicular to the left extremity
                            # RodSleeve polygon = inner + outer[-1] + reversed(outer) + inner[0]
                            pts = np.array(flat.data)
                            n = len(pts)
                            
                            p_start = pts[0]
                            p_next = pts[min(2, n-1)]
                            tangent = p_next - p_start
                            tlen = np.linalg.norm(tangent)
                            if tlen > 1e-10:
                                tangent = tangent / tlen
                            else:
                                tangent = np.array([0, 1])
                            
                            # At extremity, tangent crosses the crescent width
                            centroid = np.mean(pts[:-1], axis=0)
                            if np.dot(tangent, centroid - p_start) < 0:
                                tangent = -tangent
                            
                            # Center between the two curves at the extremity
                            p_mid = (pts[0] + pts[-2]) / 2
                            
                            p1 = p_mid + tangent * 0.001
                            p2 = p1 + tangent * 0.02
                            
                            use_dashed = getattr(self.config, 'laser_text_mode', False)
                            text_layer = "cuts" if use_dashed else "text"
                            text_obj = Text(unique_name, p1, p2, size=0.003, valign=0.5,
                                           dashed=use_dashed,
                                           dot_spacing=getattr(self.config, 'dot_spacing', 0.15))
                            sleeve_part.layers[text_layer] += text_obj.get_vectors()
                            
                            self.rod_sleeves.append(sleeve_part)
                            
                    except Exception as e:
                        print(f"Failed to plot rod sleeve {unique_name}: {e}")
                        import traceback
                        traceback.print_exc()
        
        return self.rod_sleeves

    def get_all_grouped(self) -> Layout:
        # create x-raster
        for rib in self.ribs:
            rib.rotate(90, radians=False)

        panels = self.panels
        ribs = Layout.stack_row(self.ribs, self.config.patterns_align_dist_x)

        def stack_grid(dct):
            layout_lst = [
                Layout.stack_column(p, self.config.patterns_align_dist_y)
                for p in dct.values()
            ]
            return Layout.stack_row(layout_lst, self.config.patterns_align_dist_x)

        dribs = stack_grid(self.dribs)
        straps = stack_grid(self.straps)
        rigidfoils = stack_grid(self.rigidfoils)

        def group(layout, prefix):
            grouped = layout.group_materials()
            border = layout.draw_border(append=False)

            for material_name, material_layout in grouped.items():
                material_layout.parts.append(border.copy())
                material_layout.add_text(f"{prefix}_{material_name}")

            return grouped.values()

        panels_grouped = group(panels.copy(), "panels")
        ribs_grouped = group(ribs, "ribs")
        dribs_grouped = group(dribs, "dribs")
        straps_grouped = group(straps, "straps")

        panels.add_text("panels_all")

        # Add reinforcements layout (halfmoons + attachment sleeves)
        reinforcements_layout = Layout()
        if self.reinforcements:
            reinforcements_layout = Layout.stack_row(
                self.reinforcements, self.config.patterns_align_dist_x
            )
            reinforcements_layout.draw_border(border=0.02)
            reinforcements_layout.add_text("reinforcements")

        # Add rod sleeves layout (profile rod sleeves - separate frame)
        rod_sleeves_layout = Layout()
        if self.rod_sleeves:
            rod_sleeves_layout = Layout.stack_row(
                self.rod_sleeves, self.config.patterns_align_dist_x
            )
            rod_sleeves_layout.draw_border(border=0.02)
            rod_sleeves_layout.add_text("rod_sleeves")

        # Add miniribs layout (between ribs and dribs)
        miniribs_layout = Layout()
        if self.miniribs:
            miniribs_layout = Layout.stack_row(
                self.miniribs, self.config.patterns_align_dist_x
            )
            miniribs_layout.draw_border(border=0.02)
            miniribs_layout.add_text("miniribs")

        all_layouts = [panels]
        all_layouts += panels_grouped
        if self.reinforcements:
            all_layouts += [reinforcements_layout]
        if self.rod_sleeves:
            all_layouts += [rod_sleeves_layout]
        all_layouts += ribs_grouped
        if self.miniribs:
            all_layouts += [miniribs_layout]
        all_layouts += dribs_grouped
        all_layouts += straps_grouped
        all_layouts += [rigidfoils]

        return Layout.stack_column(all_layouts, 0.01, center_x=False)

    def unwrap(self):
        self.get_panels()
        self.get_ribs()
        self.get_miniribs()
        self.get_dribs()
        self.get_straps()
        self.get_rigidfoils()
        self.get_le_splits()
        self.get_reinforcements()
        self.get_rod_sleeves()
        return self

    def get_all_parts(self):
        parts = []
        for cell in self.panels.values():
            parts += [p.copy() for p in cell]
        for rib in self.ribs:
            parts.append(rib.copy())
        for dribs in self.dribs.values():
            parts += [p.copy() for p in dribs]
        for rigidfoils in self.rigidfoils.values():
            parts += [p.copy() for p in rigidfoils]
        return Layout(parts)

    # def get_all_grouped(self):
    #    return self.get_all_parts().group_materials()
