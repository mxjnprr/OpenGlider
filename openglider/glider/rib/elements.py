#! /usr/bin/python2
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
import math

import numpy as np

from openglider.lines import Node
from openglider.vector import norm
from openglider.vector.functions import set_dimension
from openglider.vector.polygon import Circle
from openglider.vector.polyline import PolyLine2D
from openglider.vector.spline import Bezier


class RigidFoil:
    def __init__(self, start=-0.1, end=0.1, distance=0.005, circle_radius=0.03):
        self.start = start
        self.end = end
        self.distance = distance
        self.circle_radius = circle_radius
        # self.func = lambda x: distance

    def func(self, pos):
        dsq = None
        if -0.05 <= pos - self.start < self.circle_radius:
            dsq = self.circle_radius**2 - (self.circle_radius + self.start - pos) ** 2
        if -0.05 <= self.end - pos < self.circle_radius:
            dsq = self.circle_radius**2 - (self.circle_radius + pos - self.end) ** 2

        if dsq is not None:
            dsq = max(dsq, 0)
            return self.distance + (self.circle_radius - np.sqrt(dsq)) * 0.35
        return self.distance

    def __json__(self):
        return {"start": self.start, "end": self.end, "distance": self.distance}

    def get_3d(self, rib):
        return [rib.align(p, scale=False) for p in self.get_flattened(rib)]

    def get_length(self, rib):
        return self.get_flattened(rib).get_length()

    def get_flattened(self, rib):
        flat = PolyLine2D(self._get_flattened(rib))
        flat.check()
        return flat

    def _get_flattened(self, rib):
        max_segment = 0.005  # 5mm
        profile = rib.profile_2d
        profile_normvectors = PolyLine2D(profile.normvectors)

        start = profile(self.start)
        end = profile(self.end)

        point_range = []
        last_node = None
        for p in profile[start:end]:
            sign = -1 if p[1] > 0 else +1

            if last_node is not None:
                diff = norm(p - last_node) * rib.chord
                if diff > max_segment:
                    segments = int(math.ceil(diff / max_segment))
                    point_range += list(
                        np.linspace(point_range[-1], sign * p[0], segments)
                    )[1:]
                else:
                    point_range.append(sign * p[0])
            else:
                point_range.append(sign * p[0])

            last_node = p

        indices = [profile(x) for x in point_range]

        return [
            (profile[index] - profile_normvectors[index] * self.func(x)) * rib.chord
            for index, x in zip(indices, point_range)
        ]


class FoilCurve:
    def __init__(self, front=0, end=0.17):
        self.front = front
        self.end = end

    def get_flattened(self, rib, numpoints=30):
        curve = [
            [self.end, 0.75],
            [self.end - 0.05, 1],
            [self.front, 0],
            [self.end - 0.05, -1],
            [self.end, -0.75],
        ]
        profile = rib.profile_2d

        cp = [profile.align(point) * rib.chord for point in curve]

        return Bezier(cp).interpolation(numpoints)


class GibusArcs:
    """
    A Reinforcement, in the shape of an arc, to reinforce attachment points
    """

    def __init__(self, position, size=0.2, material_code=None):
        self.pos = position
        self.size = size
        self.size_abs = False
        self.material_code = material_code or ""

    def __json__(self):
        return {"position": self.pos, "size": self.size}

    def get_3d(self, rib, num_points=10):
        # create circle with center on the point
        gib_arc = self.get_flattened(rib, num_points=num_points)
        return [rib.align([p[0], p[1], 0], scale=False) for p in gib_arc]

    def get_flattened(self, rib, num_points=10):
        # get center point
        profile = rib.profile_2d
        start = profile(self.pos)
        point_1 = profile[start]

        if self.size_abs:
            # reverse scale now
            size = self.size / rib.chord
        else:
            size = self.size
        point_2 = profile.profilepoint(self.pos + size)

        gib_arc = [[], []]  # first, second
        circle = Circle(point_1, point_2).get_sequence()[1:]
        # circle = Polygon(edges=num_points)(point_1, point_2)[0][1:] # todo: is_center -> true
        is_second_run = False

        for i in range(len(circle)):
            if (
                profile.contains_point(circle[i])
                or (i < len(circle) - 1 and profile.contains_point(circle[i + 1]))
                or (i > 1 and profile.contains_point(circle[i - 1]))
            ):
                gib_arc[is_second_run].append(circle[i])
            else:
                is_second_run = True

        # Cut first and last
        gib_arc = gib_arc[1] + gib_arc[0]  # [secondlist] + [firstlist]
        start2 = profile.cut(gib_arc[0], gib_arc[1], start)
        stop = profile.cut(gib_arc[-2], gib_arc[-1], start)
        # Append Profile_List
        gib_arc += profile.get(start2.next()[0], stop.next()[0]).tolist()

        return np.array(gib_arc) * rib.chord


class CellAttachmentPoint(Node):
    def __init__(self, cell, name, cell_pos, rib_pos, force=None):
        super().__init__(node_type=2)
        self.cell = cell
        self.cell_pos = cell_pos
        self.rib_pos = rib_pos
        self.name = name
        self.force = force

    def __repr__(self):
        return f"<Attachment point '{self.name}' ({self.rib_pos})>"

    def __json__(self):
        return {
            "cell": self.cell,
            "cell_pos": self.cell_pos,
            "rib_pos": self.rib_pos,
            "name": self.name,
            "force": self.force,
        }

    def get_position(self):
        ik = self.cell.rib1.profile_2d(self.rib_pos)
        self.vec = self.cell.midrib(self.cell_pos)[ik]
        return self.vec


# Node from lines
class AttachmentPoint(Node):
    def __init__(self, rib, name, rib_pos, force=None):
        super().__init__(node_type=2)
        self.rib = rib
        self.rib_pos = rib_pos
        self.name = name
        self.force = force

    def __repr__(self):
        return f"<Attachment point '{self.name}' ({self.rib_pos})>"

    def __json__(self):
        return {
            "rib": self.rib,
            "name": self.name,
            "rib_pos": self.rib_pos,
            "force": self.force,
        }

    def get_position(self):
        self.vec = self.rib.profile_3d[self.rib.profile_2d(self.rib_pos)]
        return self.vec


class RibHole:
    def __init__(self, pos, size=0.5, vertical_shift=0.0, rotation=0.0, shape='ellipse', available_height=None, custom_points=None, corner_radius=0.25):
        self.pos = pos
        if isinstance(size, (list, tuple, np.ndarray)):
            self.size = np.array(list(size))
        else:  # float for uniform scaling
            self.size = np.array([size, size])
        self.vertical_shift = vertical_shift
        self.rotation = rotation  # rotation about p1
        self.shape = shape
        self.available_height = available_height
        self.custom_points = custom_points
        self.corner_radius = corner_radius  # Corner radius for rounded rectangles

    def get_3d(self, rib, num=20):
        hole = self.get_points(rib, num=num, available_height=self.available_height)
        return rib.align_all(set_dimension(hole, 3))

    def get_flattened(self, rib, num=80, scale=True):
        points = self.get_points(rib, num, available_height=self.available_height).data
        if scale:
            points *= rib.chord
        return PolyLine2D(points)

    def get_points(self, rib, num=80, available_height=None):
        if self.custom_points is not None:
             # If custom_points are provided, return them directly.
             # They are assumed to be in the rib's local coordinate system (normalized if scale=False context, but RibHole usually handles normalized)
             # Based on apply_holes logic, we will pass normalized coordinates.
             return PolyLine2D(self.custom_points, name=f"{rib.name}-hole")

        prof = rib.profile_2d
        p1 = prof[prof(self.pos)]  # Lower surface point
        p2 = prof[prof(-self.pos)] # Upper surface point

        local_thickness = np.linalg.norm(p2 - p1)

        # Avoid division by zero or errors for very thin profiles
        if local_thickness < 1e-9:
            return PolyLine2D([], name=f"{rib.name}-hole")

        # The caller may pass available_height directly.
        # Prioritize the passed argument, but fall back to the instance attribute.
        effective_ah = available_height if available_height is not None else getattr(self, 'available_height', None)
        height_base = effective_ah if effective_ah is not None else local_thickness

        # Calculate final hole dimensions
        final_width = self.size[0] * height_base
        final_height = self.size[1] * height_base

        # Generate shape centered at (0,0)
        if self.shape == 'ellipse':
            shape_points = []
            for angle in np.linspace(0, 2 * np.pi, num + 1):
                x = final_width / 2 * np.cos(angle)
                y = final_height / 2 * np.sin(angle)
                shape_points.append([x, y])
            shape_poly = np.array(shape_points)
        else:  # rounded rectangle
            shape_poly = self.create_rounded_rectangle(num, final_width, final_height, self.corner_radius)

        # Determine final center position
        # Note: self.vertical_shift is now an absolute shift in the local frame
        final_center = p1 + (p2 - p1) / 2
        final_center[1] += self.vertical_shift * local_thickness

        # Rotate and then translate the shape
        rotation_rad = np.deg2rad(self.rotation)
        rot_matrix = np.array([[np.cos(rotation_rad), -np.sin(rotation_rad)],
                               [np.sin(rotation_rad), np.cos(rotation_rad)]])

        # Apply rotation around shape's origin, then translate to final position
        shape_poly = shape_poly.dot(rot_matrix)
        shape_poly += final_center

        return PolyLine2D(shape_poly, name=f"{rib.name}-hole")

    def create_rounded_rectangle(self, num, width, height, corner_radius=0.005):
        # corner_radius is treated as a ratio of min(width, height)
        # Default 0.25 = 25% of smallest dimension for backward compatibility
        # If corner_radius > 1, assume it was passed as absolute and convert
        if corner_radius > 1.0:
            # Assume it was passed in same units as width/height, convert to ratio
            radius = min(corner_radius, min(width, height) / 2.0)
        else:
            # Treat as ratio
            radius = min(width, height) * corner_radius
        
        if radius < 0: radius = 0
        if radius > width / 2.0: radius = width / 2.0
        if radius > height / 2.0: radius = height / 2.0

        w = max(0.0, width / 2.0 - radius)
        h = max(0.0, height / 2.0 - radius)

        points = []
        num_corner = max(2, num // 4)

        # Top right
        center_x, center_y = w, h
        for angle in np.linspace(0, np.pi/2, num_corner):
            points.append((center_x + radius * np.cos(angle), center_y + radius * np.sin(angle)))

        # Top left
        center_x, center_y = -w, h
        for angle in np.linspace(np.pi/2, np.pi, num_corner):
            points.append((center_x + radius * np.cos(angle), center_y + radius * np.sin(angle)))

        # Bottom left
        center_x, center_y = -w, -h
        for angle in np.linspace(np.pi, 3*np.pi/2, num_corner):
            points.append((center_x + radius * np.cos(angle), center_y + radius * np.sin(angle)))

        # Bottom right
        center_x, center_y = w, -h
        for angle in np.linspace(3*np.pi/2, 2*np.pi, num_corner):
            points.append((center_x + radius * np.cos(angle), center_y + radius * np.sin(angle)))

        # Close the polygon (add first point at the end)
        if len(points) > 0:
            points.append(points[0])

        return np.array(points)

    def get_center(self, rib, scale=True):
        if self.custom_points is not None:
             # Calculate centroid of custom points
             # custom_points is list of [x, y] or numpy arrays
             # Exclude the last point if it closes the loop (same as first point)
             points = [np.array(p) for p in self.custom_points]
             if len(points) > 1:
                 # Check if last point is same as first (closed polygon)
                 if np.linalg.norm(points[0] - points[-1]) < 1e-9:
                     points = points[:-1]
             if len(points) == 0:
                 return np.array([0.0, 0.0])
             center = np.mean(points, axis=0)
             if scale:
                 center = center * rib.chord
             return center

        prof = rib.profile_2d
        p1 = prof[prof(self.pos)]
        p2 = prof[prof(-self.pos)]
        
        local_thickness = np.linalg.norm(p2 - p1)
        
        final_center = p1 + (p2 - p1) / 2
        final_center[1] += self.vertical_shift * local_thickness
        
        if scale:
            final_center *= rib.chord
            
        return final_center

    def __json__(self):
        return {
            "pos": self.pos,
            "size": self.size,
            "vertical_shift": self.vertical_shift,
            "rotation": self.rotation,
            "shape": self.shape,
            "corner_radius": self.corner_radius,
        }

class Mylar:
    pass


class RodSleeve:
    """
    Rod sleeve (fourreau de jonc) following the airfoil surface.
    
    Used to hold rigid rods that maintain the profile shape in the chord direction.
    Can be placed on extrados or intrados.
    
    Attributes:
        surface: 'extrados' or 'intrados'
        width: Sleeve width in meters (perpendicular to surface)
        offset: Offset from the surface in meters
        start_chord: Start position as chord percentage (0 = leading edge).
            May be NEGATIVE: the sleeve then crosses the leading edge and runs
            onto the opposite surface (e.g. an extrados sleeve descending onto
            the intrados), following the real profile contour. Useful for shark
            noses where the rod must wrap past the nose.
        end_chord: End position as chord percentage (1 = trailing edge).
        le_curl: Leading edge termination curl in degrees. The termination
            continues the sleeve tangent and rotates by this much *toward the
            profile*, tightening the curvature (radius-decreasing arc) instead
            of escaping straight. 0 = straight tangent continuation, no curl.
        te_curl: Trailing edge termination curl in degrees (same meaning).
        le_length: Leading edge termination length as a fraction of the rib chord.
        te_length: Trailing edge termination length as a fraction of the rib chord.
        le_angle / te_angle: DEPRECATED absolute escape angles, kept only so old
            saved gliders still load. No longer used by the geometry; ``le_curl``
            / ``te_curl`` replace them.

    Note:
        ``le_length`` / ``te_length`` are proportional (chord fractions) rather
        than absolute lengths, so the terminations scale with the profile size
        and stay reasonable on small tip ribs.
    """

    def __init__(
        self,
        surface='extrados',
        width=0.015,
        offset=0.005,
        start_chord=0.0,
        end_chord=0.85,
        le_curl=60.0,        # Leading edge curl (degrees), tightens toward profile
        te_curl=60.0,        # Trailing edge curl (degrees)
        le_length=0.08,      # Leading edge termination length (fraction of chord)
        te_length=0.06,      # Trailing edge termination length (fraction of chord)
        le_angle=None,       # DEPRECATED absolute escape angle (kept for old files)
        te_angle=None,       # DEPRECATED absolute escape angle (kept for old files)
        material_code=None,
    ):
        self.surface = surface
        self.width = width
        self.offset = offset
        self.start_chord = start_chord
        self.end_chord = end_chord
        self.le_curl = le_curl
        self.te_curl = te_curl
        self.le_length = le_length
        self.te_length = te_length
        # Deprecated, retained so round-tripping an old glider does not lose data.
        self.le_angle = le_angle
        self.te_angle = te_angle
        self.material_code = material_code or ""

    def __json__(self):
        return {
            "surface": self.surface,
            "width": self.width,
            "offset": self.offset,
            "start_chord": self.start_chord,
            "end_chord": self.end_chord,
            "le_curl": self.le_curl,
            "te_curl": self.te_curl,
            "le_length": self.le_length,
            "te_length": self.te_length,
            "material_code": self.material_code,
        }
    
    def get_profile_range(self, profile):
        """
        Get the x-value range for the sleeve based on chord percentages.
        Returns (start_x, end_x) in profile coordinates.
        """
        if self.surface == 'extrados':
            start_x = -self.start_chord
            end_x = -self.end_chord
        else:
            start_x = self.start_chord
            end_x = self.end_chord
        return (start_x, end_x)
    
    def _calculate_normal(self, profile_segment, i, surface):
        """Calculate the perpendicular normal vector at point i."""
        if len(profile_segment) < 2:
            return np.array([0.0, 1.0])
        
        if i == 0:
            tangent = profile_segment[1] - profile_segment[0]
        elif i == len(profile_segment) - 1:
            tangent = profile_segment[-1] - profile_segment[-2]
        else:
            tangent = profile_segment[i + 1] - profile_segment[i - 1]
        
        tangent_len = np.linalg.norm(tangent)
        if tangent_len < 1e-10:
            tangent = np.array([1.0, 0.0])
        else:
            tangent = tangent / tangent_len
        
        if surface == 'extrados':
            normal = np.array([tangent[1], -tangent[0]])
        else:
            normal = np.array([-tangent[1], tangent[0]])
        
        return normal
    
    def _create_smooth_termination_with_width(self, inner_start, outer_start, end_angle_deg, length, 
                                                start_tangent_vec=None, num_points=12):
        """
        Create a smooth termination curve with constant width.
        The curve starts tangent to the main sleeve direction and ends in the specified angle direction.
        
        Args:
            inner_start: Starting point of inner edge (numpy array)
            outer_start: Starting point of outer edge (numpy array)  
            end_angle_deg: FINAL direction angle in degrees (0=right, 90=up, 180=left, 270=down)
            length: Length of the termination
            start_tangent_vec: Optional starting tangent direction. If None, calculated from sleeve orientation.
            num_points: Number of points in the curve
        
        Returns:
            Tuple of (inner_curve, outer_curve) with constant width
        """
        # Calculate center line and width
        center_start = (inner_start + outer_start) / 2
        width = np.linalg.norm(outer_start - inner_start)
        
        # End direction
        end_angle_rad = np.deg2rad(end_angle_deg)
        end_direction = np.array([np.cos(end_angle_rad), np.sin(end_angle_rad)])
        
        # Calculate initial width direction (from inner to outer)
        width_dir = (outer_start - inner_start)
        width_len = np.linalg.norm(width_dir)
        if width_len > 1e-10:
            width_dir_norm = width_dir / width_len
        else:
            width_dir_norm = np.array([0, 1])
        
        # The ONLY valid tangent that preserves inner/outer alignment
        required_tangent = np.array([width_dir_norm[1], -width_dir_norm[0]])
        
        if start_tangent_vec is None:
            start_tangent = required_tangent
        else:
            start_tangent = start_tangent_vec / np.linalg.norm(start_tangent_vec)

        
        # End point: go in end_direction for 'length' distance
        center_end = center_start + end_direction * length
        
        # Control points for cubic Bezier (smooth curve with G1 continuity)
        # First control point: extend from start in start_tangent direction
        # This ensures the curve starts tangent to the main sleeve
        ctrl1 = center_start + start_tangent * length * 0.5
        
        # Second control point: approach end from the end_direction
        # This ensures the curve ends tangent to end_direction
        ctrl2 = center_end - end_direction * length * 0.5
        
        inner_curve = []
        outer_curve = []
        
        # Track previous normal for progressive orientation consistency
        prev_normal = width_dir_norm  # Start with initial width direction
        
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # Cubic Bezier for center line
            center_pt = (
                center_start * (1-t)**3 + 
                ctrl1 * 3 * (1-t)**2 * t + 
                ctrl2 * 3 * (1-t) * t**2 + 
                center_end * t**3
            )
            
            # Calculate tangent at this point (derivative of Bezier)
            tangent = (
                (ctrl1 - center_start) * 3 * (1-t)**2 +
                (ctrl2 - ctrl1) * 6 * (1-t) * t +
                (center_end - ctrl2) * 3 * t**2
            )
            
            tangent_len = np.linalg.norm(tangent)
            if tangent_len > 1e-10:
                tangent = tangent / tangent_len
            else:
                tangent = end_direction
            
            # Normal is perpendicular to tangent
            normal = np.array([-tangent[1], tangent[0]])
            
            # PROGRESSIVE FIX: Compare with PREVIOUS normal, not initial direction
            # This ensures smooth transitions even for large curve turns
            if np.dot(normal, prev_normal) < 0:
                normal = -normal
            
            prev_normal = normal  # Update for next iteration
            
            # Inner and outer points at constant width
            half_width = width / 2
            inner_pt = center_pt - normal * half_width
            outer_pt = center_pt + normal * half_width
            
            inner_curve.append(inner_pt)
            outer_curve.append(outer_pt)
        
        return inner_curve, outer_curve

    def _create_curl_termination(self, inner_start, outer_start, start_tangent,
                                 curl_deg, length, num_points=16):
        """
        Create a termination that *continues* the sleeve curvature and tightens
        it, keeping the rod pre-stressed in a curve (no S-shaped escape).

        The centreline leaves the sleeve extremity tangentially (``start_tangent``,
        pointing away from the sleeve body) and turns by ``curl_deg`` degrees
        *toward the profile* over an arc length of ``length``. A constant turn
        rate makes a circular arc of radius ``length / radians(curl_deg)``; the
        larger ``curl_deg``, the tighter it closes. ``curl_deg == 0`` gives a
        straight tangent continuation.

        Returns (inner_curve, outer_curve) at constant width, oriented like the
        old termination helper: index 0 sits at the sleeve junction.
        """
        inner_start = np.array(inner_start, dtype=float)
        outer_start = np.array(outer_start, dtype=float)
        center_start = (inner_start + outer_start) / 2
        width = float(np.linalg.norm(outer_start - inner_start))

        # width direction, inner -> outer (kept for consistent offset side)
        width_vec = outer_start - inner_start
        width_len = np.linalg.norm(width_vec)
        width_dir = width_vec / width_len if width_len > 1e-10 else np.array([0.0, 1.0])

        t0 = np.array(start_tangent, dtype=float)
        t0_len = np.linalg.norm(t0)
        if t0_len < 1e-10:
            # fall back to the tangent implied by the width direction
            t0 = np.array([width_dir[1], -width_dir[0]])
        else:
            t0 = t0 / t0_len

        # Turn the termination toward the profile body so the rod stays curved
        # (hook points inward, not flaring off the airfoil). Empirically the
        # surface side is +width_dir here, so bend the tangent toward it.
        toward_profile = width_dir
        cross = t0[0] * toward_profile[1] - t0[1] * toward_profile[0]
        turn_sign = 1.0 if cross >= 0 else -1.0
        curl_rad = np.deg2rad(curl_deg) * turn_sign

        n = max(2, num_points)
        step = length / (n - 1)

        # Arc-length integration of the centreline (midpoint rule on the angle).
        centers = [center_start.copy()]
        for i in range(1, n):
            ang = curl_rad * ((i - 0.5) / (n - 1))
            c, s = np.cos(ang), np.sin(ang)
            direction = np.array([c * t0[0] - s * t0[1], s * t0[0] + c * t0[1]])
            centers.append(centers[-1] + direction * step)

        inner_curve = []
        outer_curve = []
        prev_normal = width_dir
        half_width = width / 2
        for i in range(n):
            if i == 0:
                tangent = centers[1] - centers[0]
            elif i == n - 1:
                tangent = centers[-1] - centers[-2]
            else:
                tangent = centers[i + 1] - centers[i - 1]
            tl = np.linalg.norm(tangent)
            tangent = tangent / tl if tl > 1e-10 else t0

            normal = np.array([-tangent[1], tangent[0]])
            if np.dot(normal, prev_normal) < 0:
                normal = -normal
            prev_normal = normal

            inner_curve.append(centers[i] - normal * half_width)
            outer_curve.append(centers[i] + normal * half_width)

        return inner_curve, outer_curve

    def get_sleeve_points(self, rib, num_points=50, glider=None):
        """
        Get the sleeve outline points for visualization.
        Returns inner and outer polylines representing the sleeve pocket.
        Uses get_hull() for SingleSkinRib to follow the actual transformed profile.
        """
        from openglider.glider.rib.rib import SingleSkinRib
        chord = rib.chord

        # For SingleSkinRib, use the transformed profile (bows included)
        if isinstance(rib, SingleSkinRib) and glider is not None:
            try:
                profile = rib.get_hull(glider)
            except Exception:
                profile = rib.profile_2d
        else:
            profile = rib.profile_2d

        start_x, end_x = self.get_profile_range(profile)
        start_idx = profile(start_x)
        end_idx = profile(end_x)

        profile_segment = list(profile[start_idx:end_idx])

        if len(profile_segment) < 2:
            return [], []

        offset_norm = self.offset / chord
        width_norm = self.width / chord

        inner_points = []
        outer_points = []

        # Track the previous normal so the offset direction stays on the same
        # (outer) side of the contour even when the sleeve crosses the leading
        # edge onto the opposite surface -- there the raw per-point normal would
        # otherwise flip and pinch the pocket.
        prev_normal = None
        for i, point in enumerate(profile_segment):
            normal = self._calculate_normal(profile_segment, i, self.surface)

            if prev_normal is not None and np.dot(normal, prev_normal) < 0:
                normal = -normal
            prev_normal = normal

            inner_pt = point + normal * offset_norm
            outer_pt = point + normal * (offset_norm + width_norm)

            inner_points.append(inner_pt * chord)
            outer_points.append(outer_pt * chord)

        return inner_points, outer_points
    
    def get_seam_length(self, rib, glider=None):
        """
        Length along the seam contour, i.e. the inner edge sewn to the panel,
        measured from termination to termination (leading/trailing edge escape
        curves included). Returned in meters.
        """
        inner_points, _ = self.get_full_sleeve_points(rib, glider=glider)
        if len(inner_points) < 2:
            return 0.0
        return PolyLine2D(inner_points).get_length()

    def get_center_length(self, rib, glider=None):
        """
        Length along the centreline of the sleeve (the middle of the channel),
        measured from termination to termination. Returned in meters.
        """
        inner_points, outer_points = self.get_full_sleeve_points(rib, glider=glider)
        num = min(len(inner_points), len(outer_points))
        if num < 2:
            return 0.0
        center = [
            (np.array(inner_points[i]) + np.array(outer_points[i])) / 2
            for i in range(num)
        ]
        return PolyLine2D(center).get_length()

    def get_leading_edge_termination(self, rib, glider=None):
        """
        Get the leading edge termination curve.
        Uses the actual sleeve direction for smooth connection.
        """
        inner_main, outer_main = self.get_sleeve_points(rib, glider=glider)
        
        if not inner_main or len(inner_main) < 2:
            return [], []
        
        inner_start = np.array(inner_main[0])
        outer_start = np.array(outer_main[0])
        
        # Calculate actual sleeve direction from first two points
        sleeve_dir = np.array(inner_main[0]) - np.array(inner_main[1])
        dir_len = np.linalg.norm(sleeve_dir)
        if dir_len > 1e-10:
            start_tangent = sleeve_dir / dir_len
        else:
            start_tangent = None
        
        if start_tangent is None:
            return [], []
        return self._create_curl_termination(
            inner_start, outer_start, start_tangent,
            self.le_curl, self.le_length * rib.chord
        )

    def get_trailing_edge_termination(self, rib, glider=None):
        """
        Get the trailing edge termination curve.
        Uses the actual sleeve direction for smooth connection.
        """
        inner_main, outer_main = self.get_sleeve_points(rib, glider=glider)
        
        if not inner_main or len(inner_main) < 2:
            return [], []
        
        inner_start = np.array(inner_main[-1])
        outer_start = np.array(outer_main[-1])
        
        # Calculate actual sleeve direction from last two points
        sleeve_dir = np.array(inner_main[-1]) - np.array(inner_main[-2])
        dir_len = np.linalg.norm(sleeve_dir)
        if dir_len > 1e-10:
            start_tangent = sleeve_dir / dir_len
        else:
            start_tangent = None
        
        if start_tangent is None:
            return [], []
        return self._create_curl_termination(
            inner_start, outer_start, start_tangent,
            self.te_curl, self.te_length * rib.chord
        )

    def get_full_sleeve_points(self, rib, glider=None):
        """
        Get the complete sleeve with leading and trailing edge terminations.
        """
        inner_main, outer_main = self.get_sleeve_points(rib, glider=glider)
        inner_le, outer_le = self.get_leading_edge_termination(rib, glider=glider)
        inner_te, outer_te = self.get_trailing_edge_termination(rib, glider=glider)
        
        # Combine: LE termination (reversed) + main sleeve + TE termination
        # Reverse LE so it connects properly (curves outward from main sleeve)
        # Slice to avoid duplicate points at junctions
        if inner_le:
            inner_start = list(reversed(inner_le))[:-1] if len(inner_le) > 0 else []
        else:
            inner_start = []
            
        if outer_le:
            outer_start = list(reversed(outer_le))[:-1] if len(outer_le) > 0 else []
        else:
            outer_start = []
            
        inner_end = inner_te[1:] if len(inner_te) > 1 else []
        outer_end = outer_te[1:] if len(outer_te) > 1 else []
        
        inner_full = inner_start + inner_main + inner_end
        outer_full = outer_start + outer_main + outer_end
        
        return inner_full, outer_full
    
    def get_flattened(self, rib, num_points=50, glider=None):
        """
        Get the flattened 2D representation of the sleeve.
        Returns a closed polygon representing the sleeve pocket.
        """
        inner_points, outer_points = self.get_full_sleeve_points(rib, glider=glider)
        
        if not inner_points or not outer_points:
            return PolyLine2D([])
        
        polygon_points = []
        polygon_points.extend(inner_points)
        
        if len(inner_points) > 0 and len(outer_points) > 0:
            polygon_points.append(outer_points[-1])
        
        polygon_points.extend(reversed(outer_points))
        
        if len(inner_points) > 0:
            polygon_points.append(inner_points[0])
        
        return PolyLine2D(polygon_points)
    
    def get_3d(self, rib, num_points=50, glider=None):
        """Get 3D representation of the sleeve."""
        flat = self.get_flattened(rib, num_points, glider=glider)
        return [rib.align([p[0], p[1], 0], scale=False) for p in flat.data]

    def get_corner_points(self, rib, glider=None, allowance=0.0):
        """
        Return the 4 corner points at the two extremities of the sleeve, used as
        sewing reference marks on the rib and on the flattened sleeve piece.

        The corners are taken from the full sleeve outline (terminations
        included), so they sit on the drawn footprint extremities.

        Order: [inner_start, outer_start, inner_end, outer_end]
        (start = leading / start_chord extremity, end = trailing / end_chord
        extremity).  Coordinates are in profile space scaled by chord (meters),
        matching the rib plot and the flattened sleeve.

        If ``allowance`` > 0, each corner is pushed outward by that amount (along
        the sleeve width and along the sleeve length), so the marks land on the
        seam-allowance / cut line -- the reference used to position the fabric
        sleeve before starting the seam.
        """
        inner, outer = self.get_full_sleeve_points(rib, glider=glider)
        if not inner or not outer:
            return []

        inner = [np.array(p) for p in inner]
        outer = [np.array(p) for p in outer]

        if not allowance or len(inner) < 2:
            return [inner[0], outer[0], inner[-1], outer[-1]]

        def _unit(v):
            n = np.linalg.norm(v)
            return v / n if n > 1e-10 else np.zeros(2)

        def _expand(pt_inner, pt_outer, length_dir):
            w = _unit(pt_outer - pt_inner)
            corner_inner = pt_inner - w * allowance + length_dir * allowance
            corner_outer = pt_outer + w * allowance + length_dir * allowance
            return corner_inner, corner_outer

        # lengthwise directions pointing outward from the sleeve body
        l_start = _unit(inner[0] - inner[1])
        l_end = _unit(inner[-1] - inner[-2])

        ci_s, co_s = _expand(inner[0], outer[0], l_start)
        ci_e, co_e = _expand(inner[-1], outer[-1], l_end)
        return [ci_s, co_s, ci_e, co_e]

    def get_mounting_points(self, rib, glider=None, spacing=0.20):
        """
        Return evenly spaced points along the sleeve centerline, used as
        mounting / stitch reference marks along the rod.

        Points are placed every ``spacing`` meters (default 20 cm) measured
        along the centerline, starting one interval in from the leading
        extremity so the marks stay clear of the corner marks.  Returns an
        empty list if the sleeve is shorter than one interval.
        """
        inner, outer = self.get_sleeve_points(rib, glider=glider)
        if not inner or not outer:
            return []

        center = [(np.array(i) + np.array(o)) / 2 for i, o in zip(inner, outer)]

        # cumulative arc length along the centerline
        cum = [0.0]
        for k in range(1, len(center)):
            cum.append(cum[-1] + float(np.linalg.norm(center[k] - center[k - 1])))
        total = cum[-1]

        if spacing <= 0 or total < spacing:
            return []

        points = []
        seg = 1
        d = spacing
        while d < total - 1e-9:
            while seg < len(cum) and cum[seg] < d:
                seg += 1
            if seg >= len(cum):
                break
            seg_len = cum[seg] - cum[seg - 1]
            t = (d - cum[seg - 1]) / seg_len if seg_len > 1e-12 else 0.0
            points.append(center[seg - 1] + t * (center[seg] - center[seg - 1]))
            d += spacing
        return points

    def get_seam_allowance(self, rib, glider=None, allowance=0.01):
        """
        Return a closed polygon offset outward from the full sleeve outline by
        ``allowance`` meters, representing the cutting line (seam allowance)
        around the rod sleeve.  The sleeve outline itself stays as the stitch
        line.  Returns an empty PolyLine2D if the sleeve is empty.
        """
        inner, outer = self.get_full_sleeve_points(rib, glider=glider)
        if not inner or not outer or len(inner) != len(outer):
            return PolyLine2D([])

        inner = [np.array(p) for p in inner]
        outer = [np.array(p) for p in outer]
        n = len(inner)

        exp_inner = []
        exp_outer = []
        for i in range(n):
            # width direction (inner -> outer) at this station
            w = outer[i] - inner[i]
            w_len = np.linalg.norm(w)
            w = w / w_len if w_len > 1e-10 else np.array([0.0, 1.0])

            # lengthwise extension only at the two extremities
            lengthwise = np.array([0.0, 0.0])
            if n > 1 and i == 0:
                d = inner[0] - inner[1]
                d_len = np.linalg.norm(d)
                if d_len > 1e-10:
                    lengthwise = d / d_len
            elif n > 1 and i == n - 1:
                d = inner[-1] - inner[-2]
                d_len = np.linalg.norm(d)
                if d_len > 1e-10:
                    lengthwise = d / d_len

            exp_inner.append(inner[i] - w * allowance + lengthwise * allowance)
            exp_outer.append(outer[i] + w * allowance + lengthwise * allowance)

        polygon = exp_inner + list(reversed(exp_outer)) + [exp_inner[0]]
        return PolyLine2D(polygon)


class AttachmentReinforcement:
    """
    Reinforcement at attachment points with half-moon shape and optional rod sleeve.
    
    The half-moon has:
    - Outer edge following the intrados profile curve
    - Inner edge: circular arc centered on attachment point
    - Rod sleeve INSIDE the half-moon arc with end offset
    
    Attributes:
        position: Position on profile (chord %, e.g. 0.3 = 30%)
        surface_offset: Distance from profile surface to outer edge (m)
        halfmoon_radius: Radius of the circular arc (m)
        rod_enabled: Whether the rod sleeve is enabled
        rod_offset: Gap between half-moon arc and rod sleeve outer edge (m)
        rod_width: Thickness of the rod sleeve (m)
        rod_end_offset: Angular offset at ends so rod doesn't touch edges (degrees)
    """
    
    def __init__(
        self,
        position=0.0,
        surface_offset=0.003,
        halfmoon_radius=0.03,
        rod_enabled=True,
        rod_offset=0.005,
        rod_width=0.005,
        rod_end_offset=10.0,
        name="",
        material_code=None,
        relative=False,
        corner_radius=0.0,
        shark_nose=False,
        shark_start=0.0,
        shark_end=0.15,
        shark_depth=0.04,
        shark_start_angle=90.0,
        shark_end_angle=90.0,
        shark_corner_radius=0.5,
        shark_depth_relative=False,
    ):
        self.position = position
        self.surface_offset = surface_offset
        self.halfmoon_radius = halfmoon_radius
        self.rod_enabled = rod_enabled
        self.rod_offset = rod_offset
        self.rod_width = rod_width
        self.rod_end_offset = rod_end_offset
        self.name = name
        self.material_code = material_code or ""
        # Half-moon options:
        #   relative -> if True, the metric dimensions (surface_offset,
        #       halfmoon_radius, rod_offset, rod_width) are fractions of the rib
        #       chord and scale with the profile size. rod_end_offset stays a angle.
        #   corner_radius -> fillet radius (m) of the crescent tips where the
        #       half-moon meets the intrados; 0 = sharp.
        self.relative = relative
        self.corner_radius = corner_radius
        # Shark-nose mode: a rounded bounding box confined to the intrados. Its
        # bottom edge follows the intrados profile exactly from shark_start to
        # shark_end (chord fractions), its lid is a constant-thickness band offset
        # shark_depth (m) inward from the intrados, and the corners are rounded. It
        # englobes the attachment point, the air-intake and the intrados sleeve ends.
        #   shark_start  -> start chord fraction on the intrados
        #   shark_end    -> end chord fraction on the intrados
        #   shark_depth  -> band thickness inward from the intrados (m)
        #   shark_start_angle / shark_end_angle -> angle of each end cap vs the
        #       intrados tangent (degrees). 90 = perpendicular cut.
        #   shark_corner_radius -> corner rounding as a fraction (0..1) of the max
        #       rounding each cap can take (0 = sharp, 1 = fully rounded). Being
        #       relative to the local band depth, it scales with the profile and
        #       every value produces a visible change.
        #   shark_depth_relative -> if True, shark_depth is a fraction of the rib
        #       chord (scales with profile size) instead of an absolute length.
        self.shark_nose = shark_nose
        self.shark_start = shark_start
        self.shark_end = shark_end
        self.shark_depth = shark_depth
        self.shark_start_angle = shark_start_angle
        self.shark_end_angle = shark_end_angle
        self.shark_corner_radius = shark_corner_radius
        self.shark_depth_relative = shark_depth_relative

    def __json__(self):
        return {
            "position": self.position,
            "surface_offset": self.surface_offset,
            "halfmoon_radius": self.halfmoon_radius,
            "rod_enabled": self.rod_enabled,
            "rod_offset": self.rod_offset,
            "rod_width": self.rod_width,
            "rod_end_offset": self.rod_end_offset,
            "name": self.name,
            "material_code": self.material_code,
            "relative": self.relative,
            "corner_radius": self.corner_radius,
            "shark_nose": self.shark_nose,
            "shark_start": self.shark_start,
            "shark_end": self.shark_end,
            "shark_depth": self.shark_depth,
            "shark_start_angle": self.shark_start_angle,
            "shark_end_angle": self.shark_end_angle,
            "shark_corner_radius": self.shark_corner_radius,
            "shark_depth_relative": self.shark_depth_relative,
        }
    
    def _get_profile_section(self, rib, num_points=30, glider=None):
        """
        Retourne une section du profil délimitée par l'intersection du cercle
        de rayon halfmoon_radius centré sur le point d'accroche avec le profil hull.
        """
        from openglider.glider.rib.rib import SingleSkinRib

        if isinstance(rib, SingleSkinRib) and glider is not None:
            try:
                profile = rib.get_hull(glider)
            except Exception:
                profile = rib.profile_2d
        else:
            profile = rib.profile_2d

        chord = rib.chord
        
        # Point d'accroche = centre de l'arc
        center_idx = profile(self.position)
        arc_center = np.array(profile[center_idx]) * chord
        
        # --- Trouver les bornes par intersection cercle/profil ---
        all_pts = np.array(profile.data) * chord  # tous les points du profil en mètres

        def find_intersection_idx(pts, center, radius, from_idx, direction):
            """Parcourt pts depuis from_idx dans 'direction' (+1 ou -1)."""
            n = len(pts)
            i = from_idx
            while 0 < i < n - 1:
                i += direction
                d = np.linalg.norm(pts[i] - center)
                if d >= radius:
                    d_prev = np.linalg.norm(pts[i - direction] - center)
                    t = (radius - d_prev) / (d - d_prev) if abs(d - d_prev) > 1e-12 else 0.5
                    return (i - direction) + t * direction
            return float(i)  # bord du profil

        hr = self.halfmoon_radius * (chord if self.relative else 1.0)
        start_idx = find_intersection_idx(all_pts, arc_center, hr, int(center_idx), -1)
        end_idx   = find_intersection_idx(all_pts, arc_center, hr, int(center_idx), +1)
        
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx
        
        indices = np.linspace(start_idx, end_idx, num_points)
        
        points = []
        normals = []
        profile_normvectors = PolyLine2D(profile.normvectors)
        
        for idx in indices:
            pt = profile[idx] * chord
            points.append(np.array(pt))
            
            int_idx = int(min(idx, len(profile_normvectors.data) - 1))
            norm = np.array(profile_normvectors.data[int_idx])
            norm_len = np.linalg.norm(norm)
            if norm_len > 1e-10:
                norm = norm / norm_len
            normals.append(norm)
        
        return points, normals
    
    def get_halfmoon_points(self, rib, num_points=30, glider=None):
        """
        Get the half-moon (crescent) fabric reinforcement outline.
        
        - Outer edge: follows profile curve (with surface_offset)
        - Inner edge: circular ARC centered on attachment point
        """
        points, normals = self._get_profile_section(rib, num_points, glider=glider)
        
        if not points:
            return []
        
        # Get attachment point (center of the circular arc)
        from openglider.glider.rib.rib import SingleSkinRib
        if isinstance(rib, SingleSkinRib) and glider is not None:
            try:
                profile = rib.get_hull(glider)
            except Exception:
                profile = rib.profile_2d
        else:
            profile = rib.profile_2d

        center_idx = profile(self.position)
        arc_center = np.array(profile[center_idx]) * rib.chord

        # Metric dims scale with chord in relative mode.
        f = rib.chord if self.relative else 1.0
        surface_offset = self.surface_offset * f
        hr = self.halfmoon_radius * f

        # Outer edge: follows profile with surface offset
        outer_points = []
        for pt, normal in zip(points, normals):
            outer_pt = pt - normal * surface_offset
            outer_points.append(outer_pt)

        # Calculate angular range from arc_center to outer edge endpoints
        start_vec = outer_points[0] - arc_center
        end_vec = outer_points[-1] - arc_center

        start_angle = np.arctan2(start_vec[1], start_vec[0])
        end_angle = np.arctan2(end_vec[1], end_vec[0])

        # Ensure we go the right way (shorter arc)
        angle_diff = end_angle - start_angle
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi

        # Inner edge: circular arc centered at attachment point, radius = halfmoon_radius
        inner_points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            angle = start_angle + t * angle_diff

            inner_pt = arc_center + np.array([
                hr * np.cos(angle),
                hr * np.sin(angle)
            ])
            inner_points.append(inner_pt)

        # Combine: outer edge + reversed inner edge. Optionally round the crescent
        # tips (where the half-moon meets the intrados) before closing.
        loop = outer_points + list(reversed(inner_points))
        if self.corner_radius > 0 and len(loop) > 4:
            n = len(outer_points)
            r = min(self.corner_radius, 0.45 * hr)
            loop = self._fillet_corners_dist(loop, [n - 1, 0], r)
        return [np.asarray(p, dtype=float) for p in loop] + [np.asarray(loop[0], dtype=float)]
    
    def get_rod_sleeve_points(self, rib, num_points=30, glider=None):
        """
        Get the rod sleeve that sits INSIDE the half-moon arc.
        Uses relative offsets from the half-moon arc and end offset to avoid touching edges.
        """
        if not self.rod_enabled:
            return [], []
        
        points, normals = self._get_profile_section(rib, num_points, glider=glider)
        
        if not points:
            return [], []
        
        # Get attachment point (center of arcs)
        from openglider.glider.rib.rib import SingleSkinRib
        if isinstance(rib, SingleSkinRib) and glider is not None:
            try:
                profile = rib.get_hull(glider)
            except Exception:
                profile = rib.profile_2d
        else:
            profile = rib.profile_2d

        center_idx = profile(self.position)
        arc_center = np.array(profile[center_idx]) * rib.chord

        # Metric dims scale with chord in relative mode.
        f = rib.chord if self.relative else 1.0
        surface_offset = self.surface_offset * f
        hr = self.halfmoon_radius * f
        rod_offset = self.rod_offset * f
        rod_width = self.rod_width * f

        # Calculate angular range (same as half-moon)
        outer_points = []
        for pt, normal in zip(points, normals):
            outer_pt = pt - normal * surface_offset
            outer_points.append(outer_pt)
        
        start_vec = outer_points[0] - arc_center
        end_vec = outer_points[-1] - arc_center
        
        start_angle = np.arctan2(start_vec[1], start_vec[0])
        end_angle = np.arctan2(end_vec[1], end_vec[0])
        
        angle_diff = end_angle - start_angle
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        elif angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # Apply end offset (convert degrees to radians)
        end_offset_rad = np.deg2rad(self.rod_end_offset)
        rod_start_angle = start_angle + end_offset_rad * np.sign(angle_diff)
        rod_angle_diff = angle_diff - 2 * end_offset_rad * np.sign(angle_diff)
        
        # Rod sleeve: INSIDE the half-moon arc (closer to center)
        rod_outer_radius = hr - rod_offset
        rod_inner_radius = rod_outer_radius - rod_width
        
        # Ensure positive radii
        rod_outer_radius = max(0.001, rod_outer_radius)
        rod_inner_radius = max(0.001, rod_inner_radius)
        
        inner_curve = []
        outer_curve = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            angle = rod_start_angle + t * rod_angle_diff
            
            # Inner edge of rod sleeve (closer to center)
            inner_pt = arc_center + np.array([
                rod_inner_radius * np.cos(angle),
                rod_inner_radius * np.sin(angle)
            ])
            inner_curve.append(inner_pt)
            
            # Outer edge of rod sleeve (closer to half-moon arc)
            outer_pt = arc_center + np.array([
                rod_outer_radius * np.cos(angle),
                rod_outer_radius * np.sin(angle)
            ])
            outer_curve.append(outer_pt)
        
        return inner_curve, outer_curve
    
    def _shark_setup(self, rib, glider=None):
        """Profile + normals + resolved band depth shared by the shark helpers."""
        from openglider.glider.rib.rib import SingleSkinRib
        if isinstance(rib, SingleSkinRib) and glider is not None:
            try:
                profile = rib.get_hull(glider)
            except Exception:
                profile = rib.profile_2d
        else:
            profile = rib.profile_2d
        chord = rib.chord
        normvectors = PolyLine2D(profile.normvectors)
        n_norm = len(normvectors.data)
        # Band depth: absolute (m) or a fraction of the chord when relative.
        depth_abs = self.shark_depth * chord if self.shark_depth_relative else self.shark_depth
        return profile, chord, normvectors, n_norm, depth_abs

    def _shark_frame(self, ctx, x):
        """Intrados point, clamped band depth, inward normal and TE tangent at x."""
        profile, chord, normvectors, n_norm, depth_abs = ctx
        i_idx = profile(x)
        intr = np.array(profile[i_idx]) * chord
        extr = np.array(profile[profile(-x)]) * chord
        # Interpolate the normal at the fractional profile index. Flooring it
        # (data[int(i_idx)]) yields a staircase direction that, offset inward by
        # depth_abs, kicks the lid sideways at every integer boundary and saws
        # the curve into little triangles.
        nrm = np.array(normvectors[min(max(i_idx, 0.0), n_norm - 1.0)])
        nlen = np.linalg.norm(nrm)
        inward = -nrm / nlen if nlen > 1e-10 else np.array([0.0, 1.0])
        thickness = float(np.linalg.norm(extr - intr))
        # Band depth can never exceed the local profile thickness; at the nose
        # (thickness -> 0) it collapses to a point instead of jutting out full
        # depth in the degenerate tangential direction.
        d = min(depth_abs, 0.9 * thickness)
        tang = np.array([inward[1], -inward[0]])   # perpendicular to inward
        if tang[0] < 0:
            tang = -tang                            # point towards the trailing edge
        return intr, d, inward, tang

    def _shark_intrados_edge(self, ctx, x0, x1, n, allowance):
        """Sample the intrados edge from x0 to x1.

        Returns (net, out, a, b): ``net`` the net intrados points, ``out`` those
        offset outward by ``allowance`` (away from the lid), and [a, b] the index
        range of the straight part BEFORE the corner roundings (the fillet reach
        is trimmed off each end).
        """
        frames = [self._shark_frame(ctx, x) for x in np.linspace(x0, x1, n)]
        net = [f[0] for f in frames]
        out = [net[i] - frames[i][2] * allowance for i in range(n)]  # -inward = outward
        frac = float(np.clip(self.shark_corner_radius, 0.0, 1.0))
        r0 = frac * 0.45 * frames[0][1]
        r1 = frac * 0.45 * frames[-1][1]
        arc = [0.0]
        for i in range(1, n):
            arc.append(arc[-1] + float(np.linalg.norm(net[i] - net[i - 1])))
        lo, hi = r0, arc[-1] - r1
        keep = [i for i in range(n) if lo <= arc[i] <= hi]
        a = keep[0] if keep else 0
        b = keep[-1] if keep else n - 1
        return net, out, a, b

    def get_shark_notches(self, rib, glider=None, tick_len=0.01, num=3):
        """Assembly-alignment notches for the shark-nose reinforcement.

        Short ticks laid across the intrados seam, anchored on the NET intrados
        line and running outward by ``tick_len``, at ``num`` evenly spaced chord
        positions between shark_start and shark_end. The same chord positions are
        notched on the rib and on the cut piece (both in profile*chord coords), so
        the notches coincide when the piece is placed on the rib intrados.

        Returns a list of PolyLine2D ticks (empty if not a shark reinforcement).
        """
        if not self.shark_nose or num < 1:
            return []
        ctx = self._shark_setup(rib, glider)
        x0 = abs(self.shark_start)
        x1 = abs(self.shark_end)
        if x1 < x0:
            x0, x1 = x1, x0
        n = max(8, 140)
        net, out, a, b = self._shark_intrados_edge(ctx, x0, x1, n, tick_len)
        if b <= a:
            return []
        notches = []
        for k in range(num):
            i = a + int(round((k + 1) / (num + 1) * (b - a)))  # within the straight part
            notches.append(PolyLine2D([net[i], out[i]]))       # net edge -> allowance edge
        return notches

    def get_shark_seam_line(self, rib, glider=None, allowance=0.01, num_points=140):
        """Sewing (seam) line of the shark piece: the NET intrados edge over the
        straight part before the corner roundings -- i.e. the inner boundary of
        the seam allowance strip. Drawn green inside the red cut contour.

        Returns a PolyLine2D, or None if not a shark reinforcement / no allowance.
        """
        if not self.shark_nose or allowance <= 0:
            return None
        ctx = self._shark_setup(rib, glider)
        x0 = abs(self.shark_start)
        x1 = abs(self.shark_end)
        if x1 < x0:
            x0, x1 = x1, x0
        n = max(8, num_points)
        net, out, a, b = self._shark_intrados_edge(ctx, x0, x1, n, allowance)
        if b <= a:
            return None
        return PolyLine2D(net[a:b + 1])

    def get_shark_nose_points(self, rib, num_points=140, glider=None,
                              intrados_allowance=0.0):
        """
        Shark-nose reinforcement outline: a rounded bounding box confined to the
        intrados.

        - Bottom edge: follows the intrados profile exactly from ``shark_start``
          to ``shark_end`` (chord fractions), wrapping the nose.
        - Lid (top edge): a constant-thickness band, offset ``shark_depth`` inward
          from the intrados (perpendicular), so the reinforcement keeps a roughly
          constant thickness instead of bulging over the attachment point.
        - Both ends are closed with rounded corners.

        It englobes the attachment point, the air-intake and the intrados sleeve
        ends (all sitting below the lid).

        ``intrados_allowance`` (m) pushes only the intrados (bottom) edge outward
        by that much: a seam allowance on the side caught in the rib's intrados
        seam. The lid and end caps stay net (the piece is glued or sewn flat
        there). 0 = net contour (used for the on-rib placement mark).
        """
        ctx = self._shark_setup(rib, glider)
        profile, chord, normvectors, n_norm, depth_abs = ctx

        x0 = abs(self.shark_start)
        x1 = abs(self.shark_end)
        if x1 < x0:
            x0, x1 = x1, x0
        n = max(8, num_points)

        def frame(x):
            return self._shark_frame(ctx, x)

        def lid_point(x):
            """Lid point: intrados offset inward by the (clamped) band depth."""
            intr, d, inward, _ = frame(x)
            return intr + inward * d

        # End caps leave the intrados at the given angle to the tangent (90 =
        # perpendicular). The cap is the straight edge from the intrados corner to
        # the lid; to make it leave at the angle, the lid end is shifted along the
        # chord by depth/tan(angle) (0 at 90 deg -> a perpendicular cut).
        intr0, d0, m0, tau0 = frame(x0)
        intr1, d1, m1, tau1 = frame(x1)
        sh0 = d0 / np.tan(np.deg2rad(np.clip(self.shark_start_angle, 20.0, 160.0)))
        sh1 = d1 / np.tan(np.deg2rad(np.clip(self.shark_end_angle, 20.0, 160.0)))
        x0p = np.clip(x0 - tau0[0] * sh0 / chord, 0.0, 0.95) if chord else x0
        x1p = np.clip(x1 + tau1[0] * sh1 / chord, 0.0, 0.95) if chord else x1

        # Intrados (bottom) edge. With a seam allowance the edge bulges OUTWARD by
        # ``intrados_allowance`` on the straight part only (before the roundings),
        # closed at each end by a short segment back to the net edge; the corner
        # roundings stay net. 0 = the plain net intrados edge.
        net_b, out_b, a_idx, b_idx = self._shark_intrados_edge(ctx, x0, x1, n, intrados_allowance)
        if intrados_allowance > 0 and b_idx > a_idx:
            bottom = []
            for i in range(n):
                if i < a_idx or i > b_idx:
                    bottom.append(net_b[i])
                elif i == a_idx:
                    bottom.append(net_b[i])   # base of the start closer
                    bottom.append(out_b[i])   # step out to the allowance edge
                elif i == b_idx:
                    bottom.append(out_b[i])   # last allowance point
                    bottom.append(net_b[i])   # step back in (end closer)
                else:
                    bottom.append(out_b[i])   # allowance edge along the straight part
        else:
            bottom = net_b
        lid = [lid_point(x) for x in np.linspace(x0p, x1p, n)]

        if len(bottom) < 2 or len(lid) < 2:
            return []

        # Closed loop: intrados (x0->x1), end cap, lid (x1p->x0p), start cap. The
        # end/start caps are the single straight segments between the edges; the
        # four corners are then rounded with a fillet.
        loop = list(bottom) + list(reversed(lid))
        corner_idx = [len(bottom) - 1, len(bottom), len(loop) - 1, 0]
        # shark_corner_radius is a fraction (0..1) of the maximum rounding a cap
        # can take, so it scales with the reinforcement and the whole range maps
        # to a visible change (an absolute mm radius saturates at ~0.45*depth and
        # stops responding). Each corner is clamped by ITS OWN end depth, so a
        # degenerate (nose) end with d~0 rounds to nothing there WITHOUT zeroing
        # the far corners -- the old global min(d0, d1) let a nose end kill every
        # corner's radius.
        frac = float(np.clip(self.shark_corner_radius, 0.0, 1.0))
        r0 = frac * 0.45 * d0      # both corners of the start (x0) cap
        r1 = frac * 0.45 * d1      # both corners of the end (x1) cap
        # corner_idx order: [x1 intrados, x1 lid, x0 lid, x0 intrados]
        loop = self._fillet_corners_dist(loop, corner_idx, [r1, r1, r0, r0])

        contour = [np.asarray(p, dtype=float) for p in loop]
        contour.append(np.asarray(contour[0], dtype=float))
        return contour

    @staticmethod
    def _fillet_corners_dist(points, corner_indices, r, k=8):
        """Round the given corners of a closed loop with an arc of reach ``r``.

        For each corner, trim distance ``r`` along both adjacent edges (walking
        by arc length) and replace the trimmed span with a quadratic Bezier
        through the corner. Non-adjacent edge points are left untouched.

        ``r`` may be a scalar (same reach for every corner) or a sequence aligned
        with ``corner_indices`` (per-corner reach), so corners of unequal size can
        be rounded independently.
        """
        P = [np.asarray(p, dtype=float) for p in points]
        m = len(P)
        if m < 3:
            return P
        # Per-corner reach, keyed by loop index. A scalar applies to all corners.
        if np.ndim(r) == 0:
            reach = {i % m: float(r) for i in corner_indices}
        else:
            reach = {i % m: float(rr) for i, rr in zip(corner_indices, r)}
        corners = {i for i, rr in reach.items() if rr > 0}
        if not corners:
            return P

        def interp(i, direction, rr):
            rem = rr
            j = i
            for _ in range(m):
                nj = (j + direction) % m
                seg = float(np.linalg.norm(P[nj] - P[j]))
                if seg >= rem:
                    t = rem / seg if seg > 1e-12 else 0.0
                    return P[j] + t * (P[nj] - P[j])
                rem -= seg
                j = nj
            return P[j]

        # Points within a corner's reach are replaced by that corner's arc.
        trimmed = set()
        for i in corners:
            for direction in (-1, 1):
                rem = reach[i]
                j = i
                for _ in range(m):
                    nj = (j + direction) % m
                    seg = float(np.linalg.norm(P[nj] - P[j]))
                    if seg >= rem:
                        break
                    rem -= seg
                    trimmed.add(nj)
                    j = nj

        out = []
        for i in range(m):
            if i in corners:
                rr = reach[i]
                a = interp(i, -1, rr)
                c = interp(i, 1, rr)
                for t in np.linspace(0.0, 1.0, k):
                    out.append((1 - t) ** 2 * a + 2 * (1 - t) * t * P[i] + t ** 2 * c)
            elif i not in trimmed:
                out.append(P[i])
        return out

    def get_flattened(self, rib, num_points=30, glider=None, intrados_allowance=0.0):
        """Get the flattened 2D representation.

        ``intrados_allowance`` (m) adds a seam allowance on the intrados side of
        the shark-nose cut piece (0 = net outline, e.g. for the on-rib placement
        mark). It has no effect in half-moon mode.
        """
        if self.shark_nose:
            # Merged piece = half-moon prolonged into the nose band. Keep the
            # optional rod sleeve of the half-moon unchanged.
            shark_points = self.get_shark_nose_points(
                rib, glider=glider, intrados_allowance=intrados_allowance)
            inner_rod, outer_rod = self.get_rod_sleeve_points(rib, num_points, glider=glider)
            rod_points = []
            if inner_rod and outer_rod:
                rod_points = outer_rod + list(reversed(inner_rod)) + [outer_rod[0]]
            return {
                'halfmoon': PolyLine2D(shark_points),
                'rod_sleeve': PolyLine2D(rod_points),
            }

        halfmoon_points = self.get_halfmoon_points(rib, num_points, glider=glider)
        inner_rod, outer_rod = self.get_rod_sleeve_points(rib, num_points, glider=glider)
        
        # Create closed polygon for rod sleeve
        rod_points = []
        if inner_rod and outer_rod:
            rod_points = outer_rod + list(reversed(inner_rod)) + [outer_rod[0]]
        
        return {
            'halfmoon': PolyLine2D(halfmoon_points),
            'rod_sleeve': PolyLine2D(rod_points),
        }
    
    def get_3d(self, rib, num_points=30, glider=None):
        """Get 3D representation."""
        flat = self.get_flattened(rib, num_points, glider=glider)
        return {
            'halfmoon': [rib.align([p[0], p[1], 0], scale=False) for p in flat['halfmoon'].data],
            'rod_sleeve': [rib.align([p[0], p[1], 0], scale=False) for p in flat['rod_sleeve'].data],
        }

