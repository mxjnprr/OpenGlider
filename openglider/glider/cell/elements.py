#! /usr/bin/python2
# -*- coding: utf-8; -*-
#
# (c) 2013 booya (http://booya.at)
#
# This file is part of the OpenGlider project.
#
# OpenGlider is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# OpenGlider is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OpenGlider.  If not, see <http://www.gnu.org/licenses/>.
import copy
import logging
from typing import Tuple
import numpy as np
import math

import openglider.vector
from openglider.airfoil import get_x_value
from openglider.mesh import Mesh, triangulate
from openglider.utils.cache import cached_function, hash_list
from openglider.vector import norm, PolyLine
from openglider.vector.polyline import PolyLine2D
from openglider.vector.projection import flatten_list
from openglider.utils import Config


from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from openglider.glider.cell import Cell


class DiagonalRib(object):
    def __init__(
        self,
        left_front,
        left_back,
        right_front,
        right_back,
        material_code="",
        name="unnamed",
        edge_curve=0.0,
    ):
        """
        [left_front, left_back, right_front, right_back]
        -> Cut: (x_value, height)
        :param left_front as x-value
        :param left_back as x-value
        :param right_front as x-value
        :param right_back as x-value
        :param material_code: color/material (optional)
        :param name: optional name of DiagonalRib (optional)
        :param edge_curve: curvature of front/back edges (0=straight, 0.5=moderate ellipse)
        """
        # Attributes
        self.left_front = left_front
        self.left_back = left_back
        self.right_front = right_front
        self.right_back = right_back
        self.material_code = material_code
        self.name = name
        self.edge_curve = edge_curve

    def __json__(self):
        return {
            "left_front": self.left_front,
            "left_back": self.left_back,
            "right_front": self.right_front,
            "right_back": self.right_back,
            "material_code": self.material_code,
            "name": self.name,
            "edge_curve": self.edge_curve,
        }

    @property
    def width_left(self):
        return abs(self.left_front[0] - self.left_back[0])

    @property
    def width_right(self):
        return abs(self.right_front[0] - self.right_back[0])

    @property
    def center_left(self):
        return (self.left_front[0] + self.left_back[0]) / 2

    @property
    def center_right(self):
        return (self.right_front[0] + self.right_back[0]) / 2

    @width_left.setter
    def width_left(self, width):
        center = self.center_left
        self.left_front[0] = center - width / 2
        self.left_back[0] = center + width / 2

    @width_right.setter
    def width_right(self, width):
        center = self.center_right
        self.right_front[0] = center - width / 2
        self.right_back[0] = center + width / 2

    def copy(self):
        return copy.copy(self)

    def mirror(self):
        self.left_front, self.right_front = self.right_front, self.left_front
        self.left_back, self.right_back = self.right_back, self.left_back

    def get_center_length(self, cell):
        p1 = cell.rib1.point(self.center_left)
        p2 = cell.rib2.point(self.center_right)
        return norm(p2 - p1)

    def get_edge_curve_points(self, x_bottom, x_top, num_points=8):
        """
        Generate points for an edge with straight lower section + ellipse upper section.
        
        :param x_bottom: x position at bottom (intrados, height=-1)
        :param x_top: x position at top (extrados, height=1)
        :param num_points: number of points on the edge
        :return: list of (x, height) tuples for the edge
        """
        if self.edge_curve <= 0 or num_points < 3:
            # No curve - straight line
            return [(x_bottom, -1.0), (x_top, 1.0)]
        
        points = []
        transition_height = 0.0  # Height where we switch from straight to ellipse
        
        # Ellipse parameters
        # Semi-axis a = difference in x between bottom and top
        # Semi-axis b = height from transition to top (normalized)
        dx = x_top - x_bottom
        
        for i in range(num_points):
            t = i / (num_points - 1)  # 0 to 1
            height = -1.0 + 2.0 * t  # -1 to 1
            
            if height <= transition_height:
                # Straight section (lower half)
                # Linear interpolation from x_bottom to x_middle
                x_middle = x_bottom + (x_top - x_bottom) * 0.5 * (1 - self.edge_curve)
                local_t = (height + 1.0) / (transition_height + 1.0)  # 0 to 1 in lower section
                x = x_bottom + (x_middle - x_bottom) * local_t
            else:
                # Ellipse section (upper half)
                # Parametric ellipse: x = a * cos(theta), y = b * sin(theta)
                # Map height from transition_height to 1.0 -> theta from pi to pi/2
                local_t = (height - transition_height) / (1.0 - transition_height)  # 0 to 1
                theta = math.pi * (1 - local_t * 0.5)  # pi to pi/2
                
                # Ellipse center is at (x_top, transition_height)
                # Semi-axis a (horizontal) = edge_curve * |dx|
                # Semi-axis b (vertical) = 1.0 - transition_height
                a = self.edge_curve * abs(dx) * 0.5
                
                # Calculate x offset from center using ellipse
                x_offset = a * (1 + math.cos(theta))  # 0 at theta=pi, a at theta=pi/2
                
                # Base x position (straight continuation)
                x_base = x_bottom + (x_top - x_bottom) * (1 + height) / 2
                
                # Apply curve offset (inward for front edge, outward for back)
                if dx > 0:  # front edge (x increases from bottom to top)
                    x = x_base - x_offset
                else:  # back edge (x decreases from bottom to top)
                    x = x_base + x_offset
            
            points.append((x, height))
        
        return points

    def get_3d(self, cell):
        """
        Get 3d-Points of a diagonal rib
        :return: (left_list, right_list)
        """

        def get_list(rib, cut_front, cut_back):
            # Check if front and back are at the same height
            if cut_back[1] == cut_front[1]:
                height = cut_front[1]
                
                # Exact surface (height = 1 or -1): use profile slice directly
                if height in (-1, 1):
                    side = -height  # -1 -> lower, 1 -> upper
                    front = rib.profile_2d(cut_front[0] * side)
                    back = rib.profile_2d(cut_back[0] * side)
                    return rib.profile_3d[front:back]
                
                # Near surface with offset
                # Height encodes offset ratio from surface
                elif abs(height) > 0.5:
                    # Determine which surface we're near
                    if height > 0:
                        # Near extrados (upper surface)
                        side = -1  # upper surface
                        offset_direction = -1  # offset inward (toward intrados)
                    else:
                        # Near intrados (lower surface)
                        side = 1  # lower surface
                        offset_direction = 1  # offset inward (toward extrados)
                    
                    # Get profile indices for the surface curve
                    front_idx = rib.profile_2d(cut_front[0] * side)
                    back_idx = rib.profile_2d(cut_back[0] * side)
                    
                    # Get the surface curve points from profile_3d
                    surface_curve = rib.profile_3d[front_idx:back_idx]
                    
                    if len(surface_curve) < 2:
                        # Fallback to straight line
                        return PolyLine([
                            rib.align(rib.profile_2d.align(p) + [0])
                            for p in (cut_front, cut_back)
                        ])
                    
                    chord = rib.chord  # In meters
                    
                    # Apply offset to each point on the surface curve using local normals
                    offset_points = []
                    num_points = len(surface_curve)
                    
                    for i in range(num_points):
                        pt = surface_curve[i]
                        
                        # Calculate x position for this point
                        t = i / max(1, num_points - 1)
                        x_pos = cut_front[0] + (cut_back[0] - cut_front[0]) * t
                        
                        # Get REAL thickness at this x position from profile
                        try:
                            upper_pt = rib.profile_2d.profilepoint(abs(x_pos), h=1.0)
                            lower_pt = rib.profile_2d.profilepoint(abs(x_pos), h=-1.0)
                            local_thickness_norm = upper_pt[1] - lower_pt[1]  # Normalized 0-1
                            local_thickness = local_thickness_norm * chord  # In meters
                        except:
                            local_thickness = chord * 0.12  # Fallback to 12%
                        
                        # Calculate offset distance from height encoding
                        # height = 1.0 - (offset_mm / 10 / thickness_cm) * 2
                        # offset_mm = (1.0 - height) * thickness_cm * 10 / 2
                        # offset_m = (1.0 - height) * thickness_m / 2
                        height_offset_ratio = 1.0 - abs(height)
                        offset_distance = height_offset_ratio * local_thickness / 2
                        
                        # Calculate local normal (perpendicular to surface)
                        if i == 0:
                            tangent = surface_curve[1] - surface_curve[0]
                        elif i == num_points - 1:
                            tangent = surface_curve[-1] - surface_curve[-2]
                        else:
                            tangent = surface_curve[i + 1] - surface_curve[i - 1]
                        
                        # Normalize tangent
                        tangent_len = norm(tangent)
                        if tangent_len > 1e-10:
                            tangent = tangent / tangent_len
                        else:
                            tangent = np.array([1, 0, 0])
                        
                        # Normal is perpendicular to tangent in the rib plane
                        span_dir = np.array([0, 1, 0])  # Approximate span direction
                        normal = np.cross(tangent, span_dir)
                        normal_len = norm(normal)
                        if normal_len > 1e-10:
                            normal = normal / normal_len
                        else:
                            normal = np.array([0, 0, offset_direction])
                        
                        # Apply offset in normal direction (inward)
                        offset_pt = pt + normal * offset_distance * offset_direction
                        offset_points.append(offset_pt)
                    
                    return PolyLine(offset_points)
            
            # Fallback: straight line between two points (different heights on front/back)
            return PolyLine(
                [
                    rib.align(rib.profile_2d.align(p) + [0])
                    for p in (cut_front, cut_back)
                ]
            )

        left = get_list(cell.rib1, self.left_front, self.left_back)
        right = get_list(cell.rib2, self.right_front, self.right_back)

        return left, right

    def _get_hole_polygons_parametric(self, cell):
        """
        Generate hole contours in parametric (x_pos, y_pos) space.
        Returns list of (contour_2d_pts, center_2d) tuples.
        
        For cone_hole_config (full diagonals): trapezoidal holes near APs.
        For band_hole_config (horizontal bands): elliptical holes.
        """
        holes = []
        
        cone_config = getattr(self, 'cone_hole_config', None)
        band_config = getattr(self, 'band_hole_config', None)
        
        if cone_config:
            holes += self._cone_holes_parametric(cell, cone_config)
        
        if band_config:
            holes += self._band_holes_parametric(band_config)
        
        return holes
    
    def _cone_holes_parametric(self, cell, config):
        """
        Generate cone hole contours in parametric space using physical dimensions
        from get_flattened(cell), matching the 2D DXF export exactly.
        """
        holes = []
        num_zones = config.get('num_zones', 1)
        margin_side = config.get('margin_side_m', 0.003)
        margin_top = config.get('margin_top_m', 0.003)
        margin_bottom = config.get('margin_bottom_m', 0.003)
        corner_pct = config.get('corner_radius_pct', 0.25)
        
        # Which side is intrados (AP side)?
        left_h = (self.left_front[1], self.left_back[1])
        right_h = (self.right_front[1], self.right_back[1])
        left_is_intrados = (left_h[0] == -1.0 and left_h[1] == -1.0)
        right_is_intrados = (right_h[0] == -1.0 and right_h[1] == -1.0)
        if not (left_is_intrados or right_is_intrados):
            return holes
        
        if left_is_intrados:
            ap_y = 0.0; far_y = 1.0
        else:
            ap_y = 1.0; far_y = 0.0
        
        # Get physical dimensions from flattened diagonal
        try:
            flat_left, flat_right = self.get_flattened(cell)
            diag_height = flat_left.get_length()  # span direction
            # Width at AP side and far side
            from numpy.linalg import norm as _norm
            width_ap = _norm(np.array(flat_right.data[0]) - np.array(flat_left.data[0]))
            width_far = _norm(np.array(flat_right.data[-1]) - np.array(flat_left.data[-1]))
            diag_width = (width_ap + width_far) / 2
        except Exception:
            diag_height = 0.2
            diag_width = 0.3
        
        if diag_height < 0.01 or diag_width < 0.01:
            return holes
        
        # Convert physical margins to parametric fractions
        m_side = margin_side / diag_width   # fraction of x
        m_top = margin_top / diag_height    # fraction of y near far edge
        m_bot = margin_bottom / diag_height # fraction of y near AP
        
        t_ap = 0.5  # AP at center of chord range
        y_sign = 1.0 if far_y > ap_y else -1.0
        y_bot = ap_y + y_sign * m_bot
        y_top = far_y - y_sign * m_top
        
        if abs(y_top - y_bot) < 0.05:
            return holes
        
        # 2*num_zones holes total (num_zones per side of center)
        for side in range(2):
            x_edge = 0.0 if side == 0 else 1.0
            
            for zone_i in range(num_zones):
                t0 = zone_i / num_zones
                t1 = (zone_i + 1) / num_zones
                
                # Far side (wide end): zone boundaries
                x_far_l = t_ap + (x_edge - t_ap) * t0
                x_far_r = t_ap + (x_edge - t_ap) * t1
                
                # Margins at far end
                sgn = 1.0 if x_far_r > x_far_l else -1.0
                x_far_l += sgn * m_side
                x_far_r -= sgn * m_side
                if abs(x_far_r - x_far_l) < 0.01:
                    continue
                
                # AP side (narrow end): V-shape converges toward AP center
                # Each ray from AP has a perpendicular offset of margin_side
                # At distance margin_bottom from AP, the offset creates width
                convergence = m_bot / max(abs(y_top - y_bot), 0.01)
                x_bot_l = t_ap + (x_far_l - t_ap) * convergence
                x_bot_r = t_ap + (x_far_r - t_ap) * convergence
                
                # Build contour: simple triangle/trapezoid with rounded corners
                # p_bot_l, p_bot_r at AP side; p_far_l, p_far_r at far side
                corners = [
                    [x_bot_l, y_bot], [x_bot_r, y_bot],
                    [x_far_r, y_top], [x_far_l, y_top]
                ]
                
                # Check if bottom collapses to V-shape
                if abs(x_bot_r - x_bot_l) < 0.003:
                    # V-shape: single bottom point
                    p_bot = [(x_bot_l + x_bot_r) / 2, y_bot]
                    corners = [p_bot, [x_far_r, y_top], [x_far_l, y_top]]
                
                # Generate contour with rounded corners (Bézier)
                contour = []
                n_c = len(corners)
                for ci in range(n_c):
                    p_prev = corners[(ci - 1) % n_c]
                    p_curr = corners[ci]
                    p_next = corners[(ci + 1) % n_c]
                    d_prev = ((p_curr[0]-p_prev[0])**2 + (p_curr[1]-p_prev[1])**2)**0.5
                    d_next = ((p_next[0]-p_curr[0])**2 + (p_next[1]-p_curr[1])**2)**0.5
                    cut = min(d_prev, d_next) * corner_pct * 0.5
                    pa = [p_curr[0] + (p_prev[0]-p_curr[0]) / max(d_prev, 1e-9) * cut,
                          p_curr[1] + (p_prev[1]-p_curr[1]) / max(d_prev, 1e-9) * cut]
                    pb = [p_curr[0] + (p_next[0]-p_curr[0]) / max(d_next, 1e-9) * cut,
                          p_curr[1] + (p_next[1]-p_curr[1]) / max(d_next, 1e-9) * cut]
                    for fi in range(6):
                        t = fi / 5
                        px = (1-t)**2 * pa[0] + 2*(1-t)*t * p_curr[0] + t**2 * pb[0]
                        py = (1-t)**2 * pa[1] + 2*(1-t)*t * p_curr[1] + t**2 * pb[1]
                        contour.append([px, py])
                
                if len(contour) < 3:
                    continue
                cx = sum(p[0] for p in contour) / len(contour)
                cy = sum(p[1] for p in contour) / len(contour)
                holes.append((contour, [cx, cy]))
        
        return holes
    


    def _band_holes_parametric(self, config):
        """
        Generate uniform elliptical holes in parametric space for bands.
        Matches the 2D export: all ellipses have the SAME size,
        evenly distributed along the band length.
        
        In parametric space:
          x ∈ [0,1]: position along band (front→back on rib chord)
          y ∈ [0,1]: position across band width (rib1→rib2)
        """
        holes = []
        num_zones = config.get('num_zones', 1)
        
        total_holes = 2 * num_zones
        if total_holes <= 0:
            return holes
        
        # Margins in parametric space — use consistent fractions
        margin_x_edge = 0.03     # margin at front/back edges
        margin_y_edge = 0.12     # margin at rib1/rib2 edges
        margin_between = 0.03    # gap between adjacent ellipses
        
        # Ellipse sizing: all ellipses identical
        usable_x = 1.0 - 2 * margin_x_edge
        zone_width = usable_x / total_holes
        ellipse_w = max((zone_width - margin_between) / 2, 0.01)
        ellipse_h = max((1.0 - 2 * margin_y_edge) / 2, 0.01)
        
        for i in range(total_holes):
            cx = margin_x_edge + zone_width * (i + 0.5)
            cy = 0.5
            
            # Generate ellipse (24 points)
            n_pts = 24
            contour = []
            for j in range(n_pts):
                angle = 2 * np.pi * j / n_pts
                px = cx + ellipse_w * np.cos(angle)
                py = cy + ellipse_h * np.sin(angle)
                px = max(0.01, min(0.99, px))
                py = max(0.01, min(0.99, py))
                contour.append([px, py])
            
            holes.append((contour, [cx, cy]))
        
        return holes

    def get_mesh(self, cell, insert_points=4, project_3d=False):
        """
        get a mesh from a diagonal (2 poly lines)
        """
        # Increase grid resolution when holes are present for accurate filtering
        has_holes = getattr(self, 'cone_hole_config', None) or getattr(self, 'band_hole_config', None)
        if has_holes and insert_points < 10:
            insert_points = 10
        
        left, right = self.get_3d(cell)

        if insert_points:
            point_array = []
            points2d = []
            number_array = []
            # create array of points
            # the outermost points build the segments
            num_left = len(left)
            num_right = len(right)
            count = 0

            for y_pos in np.linspace(0.0, 1.0, insert_points + 2):
                # from left to right
                line_points = []
                line_points_2d = []  # TODO: mesh 2d (x, y) -> 3d nodes
                line_indices = []
                num_points = int(num_left * (1.0 - y_pos) + num_right * y_pos)

                for x_pos in np.linspace(0.0, 1.0, num_points):
                    line_points.append(
                        left[x_pos * (num_left - 1)] * (1.0 - y_pos)
                        + right[x_pos * (num_right - 1)] * y_pos
                    )
                    line_points_2d.append([x_pos, y_pos])
                    line_indices.append(count)
                    count += 1

                point_array += line_points
                points2d += line_points_2d
                number_array.append(line_indices)

            # outline
            edge = number_array[0]
            edge += [line[-1] for line in number_array[1:]]
            edge += number_array[-1][
                -2::-1
            ]  # last line reversed without the last element
            edge += [line[0] for line in number_array[1:-1]][::-1]

            segment = [[edge[i], edge[i + 1]] for i in range(len(edge) - 1)]
            segment.append([edge[-1], edge[0]])

            point_array = np.array(point_array)
            import openglider.mesh.mesh as _mesh

            if project_3d:
                points2d = _mesh.map_to_2d(point_array)

            # Generate hole contours in parametric space
            hole_polygons = self._get_hole_polygons_parametric(cell)
            
            if hole_polygons:
                # Build boundaries and hole data for PSLG triangulation
                edge_closed = edge + [edge[0]]
                boundaries = [edge_closed]
                hole_centers = []
                extra_points_2d = []
                extra_points_3d = []
                
                for contour_2d, center_2d in hole_polygons:
                    if len(contour_2d) < 3:
                        continue
                    
                    hole_start = len(points2d) + len(extra_points_2d)
                    hole_idx = []
                    
                    for pt_2d in contour_2d:
                        x_p = max(0.002, min(0.998, pt_2d[0]))
                        y_p = max(0.002, min(0.998, pt_2d[1]))
                        pt_3d = (
                            left[x_p * (num_left - 1)] * (1.0 - y_p)
                            + right[x_p * (num_right - 1)] * y_p
                        )
                        extra_points_2d.append([x_p, y_p])
                        extra_points_3d.append(pt_3d)
                        hole_idx.append(hole_start + len(hole_idx))
                    
                    hole_idx.append(hole_idx[0])  # close loop
                    boundaries.append(hole_idx)
                    hole_centers.append(center_2d)
                
                if extra_points_2d:
                    all_pts_2d = points2d + extra_points_2d
                    
                    # Sanitize: snap to grid + deduplicate to prevent PSLG segfault
                    GRID = 1e-6
                    snapped = [[round(p[0]/GRID)*GRID, round(p[1]/GRID)*GRID] for p in all_pts_2d]
                    
                    unique_map = {}
                    remap = {}
                    deduped_pts = []
                    for i, p in enumerate(snapped):
                        key = (p[0], p[1])
                        if key in unique_map:
                            remap[i] = unique_map[key]
                        else:
                            new_idx = len(deduped_pts)
                            unique_map[key] = new_idx
                            deduped_pts.append(p)
                            remap[i] = new_idx
                    
                    # Remap boundaries and remove consecutive duplicate indices
                    clean_bounds = []
                    for b in boundaries:
                        rb = [remap[i] for i in b]
                        cleaned = [rb[0]]
                        for j in range(1, len(rb)):
                            if rb[j] != cleaned[-1]:
                                cleaned.append(rb[j])
                        if len(cleaned) > 2:
                            clean_bounds.append(cleaned)
                    
                    try:
                        tri = triangulate.Triangulation(
                            deduped_pts, clean_bounds,
                            holes=hole_centers if hole_centers else None
                        )
                        mesh = tri.triangulate(options="Qzp")
                        
                        if len(mesh.elements) > 0:
                            # Compute 3D coordinates from all mesh.points (2D parametric)
                            # This handles any Steiner points that Triangle may have added
                            mesh_pts_3d = []
                            for pt2d in mesh.points:
                                x_p = max(0.0, min(1.0, pt2d[0]))
                                y_p = max(0.0, min(1.0, pt2d[1]))
                                pt_3d = (
                                    left[x_p * (num_left - 1)] * (1.0 - y_p)
                                    + right[x_p * (num_right - 1)] * y_p
                                )
                                mesh_pts_3d.append(pt_3d)
                            
                            return Mesh.from_indexed(
                                np.array(mesh_pts_3d),
                                {"diagonals": list(mesh.elements)},
                            )
                    except Exception:
                        pass  # Fall through to standard triangulation
            
            # Standard triangulation (no holes or PSLG fallback)
            tri = triangulate.Triangulation(points2d, [edge])
            mesh = tri.triangulate(options="Qz")

            return Mesh.from_indexed(
                point_array,
                {"diagonals": list(mesh.elements)},
                boundaries={"diagonals": edge},
            )

        else:
            vertices = np.array(list(left) + list(right)[::-1])
            polygon = [range(len(vertices))]
            return Mesh.from_indexed(vertices, {"diagonals": polygon})

    def get_flattened(self, cell, ribs_flattened=None) -> Tuple[PolyLine2D, PolyLine2D]:
        first, second = self.get_3d(cell)
        left, right = flatten_list(first, second)
        return left, right

    def get_average_x(self):
        """
        return average x value for sorting
        """
        return (
            self.left_front[0]
            + self.left_back[0]
            + self.right_back[0]
            + self.right_front[0]
        ) / 4


class DoubleDiagonalRib(object):
    pass  # TODO


class TensionStrap(DiagonalRib):
    def __init__(self, left, right, width, height=-1, material_code="", name=""):
        """
        Similar to a Diagonalrib but always connected to the bottom-sail.
        :param left: left center of TensionStrap as x-value
        :param right: right center of TesnionStrap as x-value
        :param width: width of TensionStrap
        :param material_code: color/material-name (optional)
        :param name: name of TensionStrap (optional)
        """
        width /= 2
        super(TensionStrap, self).__init__(
            (left - width / 2, height),
            (left + width / 2, height),
            (right - width / 2, height),
            (right + width / 2, height),
            material_code,
            name,
        )

    def __json__(self):
        return {
            "left": self.center_left,
            "right": self.center_right,
            "width": (self.width_left + self.width_right) / 2,
            "height": self.left_front[1],
        }


class TensionLine(TensionStrap):
    def __init__(self, left, right, material_code="", name=""):
        """
        Similar to a TensionStrap but with fixed width (0.01)
        :param left: left center of TensionStrap as x-value
        :param right: right center of TesnionStrap as x-value
        :param material_code: color/material-name
        :param name: optional argument names
        """
        super(TensionLine, self).__init__(
            left, right, 0.01, material_code=material_code, name=name
        )
        self.left = left
        self.right = right

    def __json__(self):
        return {
            "left": self.left,
            "right": self.right,
            "material_code": self.material_code,
            "name": self.name,
        }

    def get_length(self, cell):
        rib1 = cell.rib1
        rib2 = cell.rib2
        left = rib1.profile_3d[rib1.profile_2d(self.left)]
        right = rib2.profile_3d[rib2.profile_2d(self.right)]

        return norm(left - right)

    def get_center_length(self, cell):
        return self.get_length(cell)

    def mirror(self):
        self.left, self.right = self.right, self.left

    def get_mesh(self, cell):
        boundaries = {}
        rib1 = cell.rib1
        rib2 = cell.rib2
        p1 = rib1.profile_3d[rib1.profile_2d(self.left)]
        p2 = rib2.profile_3d[rib2.profile_2d(self.right)]
        boundaries[rib1.name] = [0]
        boundaries[rib2.name] = [1]
        return Mesh.from_indexed(
            [p1, p2], {"tension_lines": [[0, 1]]}, boundaries=boundaries
        )


class PanelCut(object):
    def __init__(self, left, right, style=0, is_3d=False):
        self.left = left
        self.right = right
        self.style = style
        self.is_3d = is_3d
        self.amount_3d = []

    def add_3d_amount(self, amount):
        self.amount_3d.append(amount)

    def get_3d_amount(self):
        if len(self.amount_3d) == 0:
            return 0

        return sum(self.amount_3d) / len(self.amount_3d)

    @property
    def mean_x(self):
        return (self.left + self.right) / 2


class Panel(object):
    """
    Glider cell-panel
    :param cut_front {'left': 0.06, 'right': 0.06, 'type': 'orthogonal'}
    """

    class CUT_TYPES(Config):
        """
        all available cut_types:
        - folded: start end of open panel (entry)
        - orthogonal: design cuts
        - singleskin-cut: start/end of a open singleskin-section (used for different rib-modifications)
        - 3d: 3d design cut
        """

        folded = "folded"
        orthogonal = "orthogonal"
        singleskin = "singleskin"
        cut_3d = "cut_3d"

    def __init__(self, cut_front, cut_back, material_code=None, name="unnamed", y_start=0.0, y_end=1.0):
        self.cut_front = cut_front  # (left, right, style(int))
        self.cut_back = cut_back
        self.material_code = material_code or ""
        self.name = name
        # y_start and y_end define the spanwise range (0=rib1, 1=rib2)
        # Default 0-1 covers the full cell, 0-0.5 and 0.5-1 for split panels
        self.y_start = y_start
        self.y_end = y_end

    def __json__(self):
        return {
            "cut_front": self.cut_front,
            "cut_back": self.cut_back,
            "material_code": self.material_code,
            "name": self.name,
            "y_start": self.y_start,
            "y_end": self.y_end,
        }

    def __hash__(self) -> int:
        return hash_list(*self.cut_front.values(), *self.cut_back.values())

    def mean_x(self) -> float:
        """
        :return: center point of the panel as x-values
        """
        total = self.cut_front["left"]
        total += self.cut_front["right"]
        total += self.cut_back["left"]
        total += self.cut_back["right"]

        return total / 4

    def __radd__(self, other):
        """needed for sum(panels)"""
        if not isinstance(other, Panel):
            return self

    def __add__(self, other):
        if self.cut_front == other.cut_back:
            return Panel(
                other.cut_front, self.cut_back, material_code=self.material_code
            )
        elif self.cut_back == other.cut_front:
            return Panel(
                self.cut_front, other.cut_back, material_code=self.material_code
            )
        else:
            return None

    def is_lower(self):
        return self.mean_x() > 0

    def get_3d(self, cell, numribs=0, midribs=None, with_numpy=False):
        """
        Get 3d-Panel
        :param glider: glider class
        :param numribs: number of miniribs to calculate
        :return: List of rib-pieces (Vectorlist)
        """
        xvalues = cell.rib1.base_profile_2d.x_values
        ribs = []
        for i in range(numribs + 1):
            # Map y from 0-1 to y_start-y_end range
            t = i / numribs
            y = self.y_start + t * (self.y_end - self.y_start)

            if midribs is None:
                midrib = cell.midrib(y, with_numpy)
            else:
                midrib = midribs[i]

            x1 = self.cut_front["left"] + y * (
                self.cut_front["right"] - self.cut_front["left"]
            )
            front = get_x_value(xvalues, x1)

            x2 = self.cut_back["left"] + y * (
                self.cut_back["right"] - self.cut_back["left"]
            )
            back = get_x_value(xvalues, x2)
            ribs.append(midrib.get(front, back))
            # todo: return polygon-data
        return ribs

    def get_mesh(self, cell, numribs=0, with_numpy=False):
        """
        Get Panel-mesh
        :param cell: the parent cell of the panel
        :param numribs: number of interpolation steps between ribs
        :param with_numpy: compute midribs with numpy (faster if available)
        :return: mesh objects consisting of triangles and quadrangles
        """
        numribs += 1
        # TODO: doesn't work for numribs=0?
        xvalues = cell.rib1.base_profile_2d.x_values
        ribs = []
        points = []
        nums = []
        count = 0
        for rib_no in range(numribs + 1):
            # Map y from 0-1 to y_start-y_end range
            t = rib_no / max(numribs, 1)
            y = self.y_start + t * (self.y_end - self.y_start)
            x1 = self.cut_front["left"] + y * (
                self.cut_front["right"] - self.cut_front["left"]
            )
            front = get_x_value(xvalues, x1)

            x2 = self.cut_back["left"] + y * (
                self.cut_back["right"] - self.cut_back["left"]
            )
            back = get_x_value(xvalues, x2)
            midrib = cell.midrib(y, with_numpy=with_numpy)
            ribs.append([x for x in midrib.get_positions(front, back)])
            points += list(midrib[front:back])
            nums.append([i + count for i, _ in enumerate(ribs[-1])])
            count += len(ribs[-1])

        triangles = []

        # helper functions
        def left_triangle(l_i, r_i):
            return [l_i + 1, l_i, r_i]

        def right_triangle(l_i, r_i):
            return [r_i + 1, l_i, r_i]

        def quad(l_i, r_i):
            return [l_i + 1, l_i, r_i, r_i + 1]

        for rib_no, _ in enumerate(ribs[:-1]):
            num_l = nums[rib_no]
            num_r = nums[rib_no + 1]
            pos_l = ribs[rib_no]
            pos_r = ribs[rib_no + 1]
            l_i = r_i = 0
            while l_i < len(num_l) - 1 or r_i < len(num_r) - 1:
                if l_i == len(num_l) - 1:
                    triangles.append(right_triangle(num_l[l_i], num_r[r_i]))
                    r_i += 1

                elif r_i == len(num_r) - 1:
                    triangles.append(left_triangle(num_l[l_i], num_r[r_i]))
                    l_i += 1

                elif abs(pos_l[l_i] - pos_r[r_i]) == 0:
                    triangles.append(quad(num_l[l_i], num_r[r_i]))
                    r_i += 1
                    l_i += 1

                elif pos_l[l_i] <= pos_r[r_i]:
                    triangles.append(left_triangle(num_l[l_i], num_r[r_i]))
                    l_i += 1

                elif pos_r[r_i] < pos_l[l_i]:
                    triangles.append(right_triangle(num_l[l_i], num_r[r_i]))
                    r_i += 1
        # connection_info = {cell.rib1: np.array(ribs[0], int),
        #                   cell.rib2: np.array(ribs[-1], int)}
        return Mesh.from_indexed(
            points, {"panel_" + self.material_code: triangles}, name=self.name
        )

    def mirror(self):
        """
        mirrors the cuts of the panel
        """
        front = self.cut_front
        self.cut_front = {"right": front["left"], "left": front["right"]}
        back = self.cut_back
        self.cut_back = {"right": back["left"], "left": back["right"]}

    def snap(self, cell):
        """
        replaces panel x_valus with x_values stored in profile-2d-x-values
        """
        p_l = cell.rib1.profile_2d
        p_r = cell.rib2.profile_2d
        self.cut_back["left"] = p_l.nearest_x_value(self.cut_back["left"])
        self.cut_back["right"] = p_r.nearest_x_value(self.cut_back["right"])
        self.cut_front["left"] = p_l.nearest_x_value(self.cut_front["left"])
        self.cut_front["right"] = p_r.nearest_x_value(self.cut_front["right"])

    @cached_function("self")
    def _get_ik_values(
        self, cell: "openglider.glider.cell.Cell", numribs=0, exact=True
    ):
        """
        :param cell: the parent cell of the panel
        :param numribs: number of interpolation steps between ribs
        :return: [[front_ik_0, back_ik_0], ...[front_ik_n, back_ik_n]] with n is numribs + 1
        """
        # TODO: move to cut!!
        x_values_left = cell.rib1.base_profile_2d.x_values

        ik_left_front = get_x_value(x_values_left, self.cut_front["left"])
        ik_left_back = get_x_value(x_values_left, self.cut_back["left"])

        x_values_right = cell.rib2.base_profile_2d.x_values
        ik_right_front = get_x_value(x_values_right, self.cut_front["right"])
        ik_right_back = get_x_value(x_values_right, self.cut_back["right"])

        ik_values = []
        
        # Use y_start and y_end for partial panels (default 0-1 for full panels)
        y_start = getattr(self, 'y_start', 0.0)
        y_end = getattr(self, 'y_end', 1.0)

        for i in range(numribs + 2):
            # Map i to y within [y_start, y_end] range
            t = float(i) / (numribs + 1)
            y = y_start + t * (y_end - y_start)

            front = ik_left_front + y * (ik_right_front - ik_left_front)
            back = ik_left_back + y * (ik_right_back - ik_left_back)

            ik_values.append([front, back])

        if exact:
            ik_values_new = []
            inner = cell.get_flattened_cell(numribs)["inner"]
            
            # For split panels, we need to get the inner lines for the y range
            # Calculate which inner lines correspond to our y range
            total_inners = len(inner)
            start_idx = int(y_start * (total_inners - 1))
            end_idx = int(y_end * (total_inners - 1))
            
            # Get the first and last points for cut lines
            p_front_left = inner[start_idx][ik_left_front + y_start * (ik_right_front - ik_left_front)]
            p_front_right = inner[end_idx][ik_left_front + y_end * (ik_right_front - ik_left_front)]
            p_back_left = inner[start_idx][ik_left_back + y_start * (ik_right_back - ik_left_back)]
            p_back_right = inner[end_idx][ik_left_back + y_end * (ik_right_back - ik_left_back)]

            # Map our ik_values indices to the appropriate inner lines
            for i, ik in enumerate(ik_values):
                ik_front, ik_back = ik
                # Calculate which inner line corresponds to this y value
                t = float(i) / (numribs + 1)
                y = y_start + t * (y_end - y_start)
                inner_idx = int(round(y * (total_inners - 1)))
                inner_idx = max(0, min(inner_idx, total_inners - 1))
                
                line: openglider.vector.PolyLine2D = inner[inner_idx]

                _cut_front = line.cut(p_front_left, p_front_right, ik_front, True)
                _cut_back = line.cut(p_back_left, p_back_right, ik_back, True)

                try:
                    _ik_front = next(_cut_front)[0]
                except StopIteration:
                    _ik_front = ik_front
                    logging.warning(f"panel_ik_failure: {ik_front} {cell}/{self}/back")
                try:
                    _ik_back = next(_cut_back)[0]
                except StopIteration:
                    _ik_back = ik_back
                    logging.warning(f"panel_ik_failure: {ik_front} {cell}/{self}/back")

                ik_values_new.append((_ik_front, _ik_back))

            return ik_values_new

        else:
            return ik_values

    @cached_function("self")
    def _get_ik_interpolation(self, cell: "Cell", numribs=0, exact=True):
        ik_values = self._get_ik_values(cell, numribs=5, exact=exact)
        numpoints = len(ik_values) - 1
        ik_interpolation_front = openglider.vector.Interpolation(
            [[i / numpoints, x[0]] for i, x in enumerate(ik_values)]
        )

        ik_interpolation_back = openglider.vector.Interpolation(
            [[i / numpoints, x[1]] for i, x in enumerate(ik_values)]
        )

        return ik_interpolation_front, ik_interpolation_back

    def integrate_3d_shaping(self, cell: "Cell", sigma, inner_2d, midribs=None):
        """
        :param cell: the parent cell of the panel
        :param sigma: std-deviation parameter of gaussian distribution used to weight the length differences.
        :param inner_2d: list of 2D polylines (flat representation of the cell)s
        :param midribs: precomputed midribs, None by default
        :return: front, back (lists of lengths) with length equal to number of midribs
        """
        numribs = len(inner_2d) - 2
        if midribs is None or len(midribs) != len(inner_2d):
            midribs = cell.get_midribs(numribs)

        ribs = [cell.prof1] + midribs + [cell.prof2]

        # ! vorn + hinten < gesamt !

        positions = self._get_ik_values(cell, numribs, exact=True)

        front = []
        back = []

        ff = math.sqrt(math.pi / 2) * sigma

        for rib_no in range(numribs + 2):
            x1, x2 = positions[rib_no]
            rib_2d = inner_2d[rib_no][x1:x2]
            rib_3d = ribs[rib_no][x1:x2]

            lengthes_2d = rib_2d.get_segment_lengthes()
            lengthes_3d = rib_3d.get_segment_lengthes()

            distance = 0
            amount_front = 0
            # influence factor: e^-(x^2/(2*sigma^2))
            # -> sigma = einflussfaktor [m]
            # integral = sqrt(pi/2)*sigma * [ erf(x / (sqrt(2)*sigma) ) ]

            def integrate(lengths_2d, lengths_3d):
                amount = 0
                distance = 0

                for l2d, l3d in zip(lengths_2d, lengths_3d):
                    if l3d > 0:
                        factor = (l3d - l2d) / l3d
                        x = math.erf(
                            (distance + l3d) / (sigma * math.sqrt(2))
                        ) - math.erf(distance / (sigma * math.sqrt(2)))

                        amount += factor * x
                    distance += l3d

                return amount

            amount_back = integrate(lengthes_2d, lengthes_3d)
            amount_front = integrate(lengthes_2d[::-1], lengthes_3d[::-1])

            for l2d, l3d in zip(lengthes_2d, lengthes_3d):
                if l3d > 0:
                    factor = (l3d - l2d) / l3d
                    x = math.erf((distance + l3d) / (sigma * math.sqrt(2))) - math.erf(
                        distance / (sigma * math.sqrt(2))
                    )

                    amount_front += factor * x
                distance += l3d

            distance = 0
            amount_back = 0

            for l2d, l3d in zip(lengthes_2d[::-1], lengthes_3d[::-1]):
                if l3d > 0:
                    factor = (l3d - l2d) / l3d
                    x = math.erf((distance + l3d) / (sigma * math.sqrt(2))) - math.erf(
                        distance / (sigma * math.sqrt(2))
                    )
                    amount_back += factor * x
                distance += l3d

            total = 0
            for l2d, l3d in zip(lengthes_2d, lengthes_3d):
                total += l3d - l2d

            amount_front *= ff
            amount_back *= ff

            # Use .get() with default to handle panels without 'type' key
            cut_front_type = self.cut_front.get("type", "orthogonal")
            cut_back_type = self.cut_back.get("type", "orthogonal")
            
            if cut_front_type != "cut_3d" and cut_back_type != "cut_3d":
                if abs(amount_front + amount_back) > abs(total):
                    normalization = abs(total / (amount_front + amount_back))
                    amount_front *= normalization
                    amount_back *= normalization

            if rib_no == 0 or rib_no == numribs + 1:
                amount_front = 0
                amount_back = 0

            front.append(amount_front)
            back.append(amount_back)

        return front, back


class PanelRigidFoil:
    channel_width = 0.01

    def __init__(self, x_start: float, x_end: float, y: float = 0.5):
        self.x_start = x_start
        self.x_end = x_end
        self.y = y

    def __json__(self):
        return {"x_start": self.x_start, "x_end": self.x_end, "y": self.y}

    def _get_flattened_line(self, cell):
        flattened_cell = cell.get_flattened_cell()
        left, right = flattened_cell["ballooned"]
        line = (left * (1 - self.y)).add(right * self.y)

        ik_front = (
            cell.rib1.profile_2d(self.x_start) + cell.rib2.profile_2d(self.x_start)
        ) / 2
        ik_back = (
            cell.rib1.profile_2d(self.x_end) + cell.rib2.profile_2d(self.x_end)
        ) / 2

        return line, ik_front, ik_back

    def draw_panel_marks(self, cell, panel):
        line, ik_front, ik_back = self._get_flattened_line(cell)

        ik_values = panel._get_ik_values(cell, numribs=5)
        numpoints = len(ik_values) - 1
        ik_interpolation_front, ik_interpolation_back = panel._get_ik_interpolation(
            cell, numribs=5
        )

        start = max(ik_front, ik_interpolation_front(self.y))
        stop = min(ik_back, ik_interpolation_back(self.y))

        if start < stop:
            return line[start:stop]

        return None

    def get_flattened(self, cell):
        line, ik_front, ik_back = self._get_flattened_line(cell)

        left = line.copy().add_stuff(-self.channel_width / 2)
        right = line.copy().add_stuff(self.channel_width / 2)

        contour = left[ik_front:ik_back] + right[ik_back:ik_front]
        contour.close()

        marks = []

        panel_iks = []
        for panel in cell.panels:
            interpolations = panel._get_ik_interpolation(cell, numribs=5)

            panel_iks.append(interpolations[0](self.y))
            panel_iks.append(interpolations[1](self.y))

        for ik in panel_iks:
            if ik_front < ik < ik_back:
                marks.append(openglider.vector.PolyLine2D([left[ik], right[ik]]))

        return openglider.vector.drawing.PlotPart(cuts=[contour], marks=marks)


class LeadingEdgeClosure:
    """
    Defines a virtual partition at mid-span for leading edge shaping.
    Creates two half-panels from LE to cut_back_x, with ballooned thickness.
    
    This creates a "virtual rib" in the middle of the cell, but only for a 
    limited portion of the chord (from leading edge to cut_back_x). The two
    resulting half-panels are flattened using mesh-based triangulation that
    preserves 3D arc lengths, following the "Sewing Dart Principle".
    
    Parameters:
    - cut_back_x: x position (0-1 normalized) where the partition ends
                  0 = leading edge, 0.1 = 10% chord from LE
    - y_position: spanwise position of the partition (0.5 = center)
    - material_code: optional material/color code for the partition
    - name: optional name for the closure element
    """
    
    def __init__(
        self, 
        cut_back_x: float = 0.1, 
        y_position: float = 0.5,
        material_code: str = "",
        name: str = "le_closure"
    ):
        self.cut_back_x = cut_back_x
        self.y_position = y_position
        self.material_code = material_code
        self.name = name
    
    def __json__(self):
        return {
            "cut_back_x": self.cut_back_x,
            "y_position": self.y_position,
            "material_code": self.material_code,
            "name": self.name
        }
    
    def mirror(self):
        """Mirror the closure (y_position stays the same for center)."""
        pass  # y_position = 0.5 is symmetric
    
    def get_midrib_profile_3d(self, cell: "Cell"):
        """
        Get the ballooned 3D profile at y_position.
        This is the virtual rib that forms the partition.
        """
        return cell.midrib(self.y_position, ballooning=True)
    
    def get_leading_edge_ik(self, cell: "Cell") -> float:
        """Get the profile index for the leading edge (x=0)."""
        x_values = cell.rib1.base_profile_2d.x_values
        return get_x_value(x_values, 0.0)
    
    def get_cut_back_ik(self, cell: "Cell") -> float:
        """Get the profile index for the cut_back_x position."""
        x_values = cell.rib1.base_profile_2d.x_values
        return get_x_value(x_values, self.cut_back_x)
    
    def get_3d_left(self, cell: "Cell", numribs: int = 8):
        """
        Get 3D geometry for left half-panel (y=0 to y_position).
        Returns list of PolyLine representing midribs from LE to cut_back_x.
        """
        ribs = []
        ik_le = self.get_leading_edge_ik(cell)
        ik_back = self.get_cut_back_ik(cell)
        
        for i in range(numribs + 1):
            y = i / numribs * self.y_position  # y from 0 to y_position
            midrib = cell.midrib(y, ballooning=True)
            ribs.append(midrib.get(ik_le, ik_back))
        
        return ribs
    
    def get_3d_right(self, cell: "Cell", numribs: int = 8):
        """
        Get 3D geometry for right half-panel (y_position to y=1).
        Returns list of PolyLine representing midribs from LE to cut_back_x.
        """
        ribs = []
        ik_le = self.get_leading_edge_ik(cell)
        ik_back = self.get_cut_back_ik(cell)
        
        for i in range(numribs + 1):
            y = self.y_position + i / numribs * (1.0 - self.y_position)  # y from y_position to 1
            midrib = cell.midrib(y, ballooning=True)
            ribs.append(midrib.get(ik_le, ik_back))
        
        return ribs
    
    def get_mesh(self, cell: "Cell", numribs: int = 8, with_numpy: bool = False):
        """
        Get combined mesh for 3D visualization of both half-panels.
        """
        mesh_left = self._get_panel_mesh(cell, "left", numribs, with_numpy)
        mesh_right = self._get_panel_mesh(cell, "right", numribs, with_numpy)
        
        # Combine meshes
        return mesh_left + mesh_right
    
    def _get_panel_mesh(
        self, 
        cell: "Cell", 
        side: str, 
        numribs: int = 8, 
        with_numpy: bool = False
    ) -> Mesh:
        """
        Generate mesh for one half-panel.
        
        :param side: "left" or "right"
        """
        if side == "left":
            ribs_3d = self.get_3d_left(cell, numribs)
        else:
            ribs_3d = self.get_3d_right(cell, numribs)
        
        points = []
        nums = []
        count = 0
        
        for rib in ribs_3d:
            rib_points = list(rib)
            points.extend(rib_points)
            nums.append([j + count for j in range(len(rib_points))])
            count += len(rib_points)
        
        if len(nums) < 2:
            return Mesh(polygons={})
        
        triangles = []
        
        def left_triangle(l_i, r_i):
            return [l_i + 1, l_i, r_i]
        
        def right_triangle(l_i, r_i):
            return [r_i + 1, l_i, r_i]
        
        def quad(l_i, r_i):
            return [l_i + 1, l_i, r_i, r_i + 1]
        
        for rib_no in range(len(nums) - 1):
            num_l = nums[rib_no]
            num_r = nums[rib_no + 1]
            len_l = len(num_l)
            len_r = len(num_r)
            l_i = r_i = 0
            
            while l_i < len_l - 1 or r_i < len_r - 1:
                if l_i >= len_l - 1:
                    if r_i < len_r - 1:
                        triangles.append(right_triangle(num_l[min(l_i, len_l - 1)], num_r[r_i]))
                    r_i += 1
                elif r_i >= len_r - 1:
                    if l_i < len_l - 1:
                        triangles.append(left_triangle(num_l[l_i], num_r[min(r_i, len_r - 1)]))
                    l_i += 1
                else:
                    # Use quad when possible
                    triangles.append(quad(num_l[l_i], num_r[r_i]))
                    l_i += 1
                    r_i += 1
        
        mesh_name = f"le_closure_{side}_{self.material_code}"
        return Mesh.from_indexed(
            points, 
            {mesh_name: triangles}, 
            name=f"{self.name}_{side}"
        )
    
    def get_flattened(self, cell: "Cell", side: str, numribs: int = 20):
        """
        Flatten one half-panel using mesh-based triangulation.
        Preserves 3D arc lengths for sewability (Sewing Dart Principle).
        
        :param side: "left" or "right"
        :return: tuple of (left_boundary, right_boundary) as PolyLine2D
        """
        if side == "left":
            ribs_3d = self.get_3d_left(cell, numribs)
        else:
            ribs_3d = self.get_3d_right(cell, numribs)
        
        if len(ribs_3d) < 2:
            return None, None
        
        numpoints = len(ribs_3d[0])
        
        # Calculate cross-span lengths
        def get_length(ik, rib_idx1, rib_idx2):
            """Get distance between same profile point on two ribs."""
            p1 = ribs_3d[rib_idx1][ik]
            p2 = ribs_3d[rib_idx2][ik]
            return norm(p1 - p2)
        
        # Initialize first two points
        l_0 = get_length(0, 0, len(ribs_3d) - 1)
        
        left_bal = [np.array([0, 0])]
        right_bal = [np.array([l_0, 0])]
        
        def get_point(p1, p2, l_base, l_l, l_r, left=True):
            """Calculate 2D point position preserving 3D distances."""
            if l_base < 1e-10:
                # Degenerate case - points are the same
                return p1 + np.array([l_l, 0]) if left else p2 + np.array([-l_r, 0])
            
            lx = (l_base**2 + l_l**2 - l_r**2) / (2 * l_base)
            ly_sq = l_l**2 - lx**2
            ly = math.sqrt(max(0, ly_sq))
            
            diff = p2 - p1
            diff_len = norm(diff)
            if diff_len > 1e-10:
                diff = diff / diff_len
            else:
                diff = np.array([1, 0])
            
            if left:
                diff_y = np.array([-diff[1], diff[0]])
            else:
                diff_y = np.array([diff[1], -diff[0]])
            
            return p1 + lx * diff + ly * diff_y
        
        # Triangulate along the profile
        for i in range(numpoints - 1):
            p1 = left_bal[-1]
            p2 = right_bal[-1]
            
            # Distances along first and last rib
            d_l = norm(ribs_3d[0][i] - ribs_3d[0][i + 1])
            d_r = norm(ribs_3d[-1][i] - ribs_3d[-1][i + 1])
            
            # Cross-span distances
            l_current = get_length(i, 0, len(ribs_3d) - 1)
            l_next = get_length(i + 1, 0, len(ribs_3d) - 1)
            l_diag = norm(ribs_3d[0][i + 1] - ribs_3d[-1][i])
            
            # Calculate next points
            pr_2 = get_point(p2, p1, l_current, d_r, l_diag, left=False)
            pl_2 = get_point(p1, pr_2, l_diag, d_l, l_next)
            
            left_bal.append(pl_2)
            right_bal.append(pr_2)
        
        return PolyLine2D(left_bal), PolyLine2D(right_bal)
    
    def get_flattened_plotpart(
        self, 
        cell: "Cell", 
        side: str, 
        numribs: int = 20,
        seam_allowance: float = 0.006
    ):
        """
        Get a PlotPart for 2D pattern export.
        
        :param side: "left" or "right"
        :param seam_allowance: seam allowance in meters (default 6mm)
        :return: PlotPart with cuts and marks
        """
        left, right = self.get_flattened(cell, side, numribs)
        
        if left is None or right is None:
            return None
        
        # Build envelope with seam allowance
        left_outer = left.copy()
        left_outer.add_stuff(-seam_allowance)
        
        right_outer = right.copy()
        right_outer.add_stuff(seam_allowance)
        
        # Create closed contour by combining boundaries
        # Go: left_outer forward -> connect to right_outer end -> right_outer backward -> connect to start
        contour_points = []
        
        # Add left outer boundary (forward)
        contour_points.extend(left_outer.data)
        
        # Add connection to right outer end
        contour_points.append(right_outer.data[-1])
        
        # Add right outer boundary (backward)
        contour_points.extend(right_outer.data[::-1])
        
        # Close back to start
        contour_points.append(left_outer.data[0])
        
        contour = PolyLine2D(contour_points)
        
        return openglider.vector.drawing.PlotPart(
            cuts=[contour],
            marks=[left, right],  # Inner edges as marks
            name=f"{self.name}_{side}"
        )

