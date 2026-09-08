import copy

import numpy as np
from numpy.linalg import norm

from openglider.airfoil import Profile3D
from openglider.mesh import Mesh, triangulate
from openglider.utils.cache import CachedObject, cached_property
from openglider.vector.functions import set_dimension
from openglider.vector.transformation import Rotation, Scale, Translation


def _point_in_polygon(point, polygon):
    """Ray-casting point-in-polygon test."""
    x, y = float(point[0]), float(point[1])
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(polygon[i][0]), float(polygon[i][1])
        xj, yj = float(polygon[j][0]), float(polygon[j][1])
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


class Rib(CachedObject):
    """
    Openglider Rib Class: contains a airfoil, needs a startpoint, angle (arcwide), angle of attack,
        glide-wide rotation and glider ratio.
        optional: name, absolute aoa (bool), startposition
    """

    hole_naming_scheme = "{rib.name}h{}"
    rigid_naming_scheme = "{rib.name}rigid{}"

    hashlist = (
        "aoa_absolute",
        "glide",
        "arcang",
        "zrot",
        "chord",
        "pos",
        "profile_2d",
    )  # pos

    def __init__(
        self,
        profile_2d=None,
        startpoint=None,
        chord=1.0,
        arcang=0,
        aoa_absolute=0,
        zrot=0,
        xrot=0,
        glide=1,
        name="unnamed rib",
        startpos=0.0,
        rigidfoils=None,
        holes=None,
        reinforcements=None,
        rod_sleeves=None,
        material_code=None,
    ):
        self.startpos = startpos
        # TODO: Startpos > Set Rotation Axis in Percent
        self.name = name
        self.profile_2d = profile_2d
        self.glide = glide
        self.aoa_absolute = aoa_absolute
        self.arcang = arcang
        self.zrot = zrot
        self.xrot = xrot
        self.pos = np.array(startpoint)  # or HashedList([0, 0, 0])
        self.chord = chord
        self.holes = holes or []
        self.rigidfoils = rigidfoils or []
        self.reinforcements = reinforcements or []
        self.rod_sleeves = rod_sleeves or []
        self.material_code = material_code or ""
        # self.curves = [FoilCurve()]
        # TODO: add in paramteric way
        self.curves = []

    def __json__(self):
        return {
            "profile_2d": self.profile_2d,
            "startpoint": self.pos.tolist(),
            "chord": self.chord,
            "arcang": self.arcang,
            "aoa_absolute": self.aoa_absolute,
            "zrot": self.zrot,
            "xrot": self.xrot,
            "glide": self.glide,
            "name": self.name,
            "rigidfoils": self.rigidfoils,
            "holes": self.holes,
            "reinforcements": self.reinforcements,
            "rod_sleeves": self.rod_sleeves,
            "material_code": self.material_code,
        }

    def align_all(self, data):
        """align 2d coordinates to the 3d pos of the rib"""
        return self.transformation.apply(set_dimension(data, 3))

    def align(self, point, scale=True):
        if len(point) == 2:
            return self.align([point[0], point[1], 0.0], scale=scale)
        elif len(point) == 3:
            if scale:
                return self.transformation(point)
            else:
                return self.pos + self.rotation_matrix(point)

        raise ValueError("Can only Align one single 2D or 3D-Point")

    def align_x(self, x_value):
        ik = self.profile_2d(x_value)
        return self.profile_3d[ik]

    def rename_parts(self):
        for hole_no, hole in enumerate(self.holes):
            hole.name = self.hole_naming_scheme.format(hole_no, rib=self)

        for rigid_no, rigid in enumerate(self.rigidfoils):
            rigid.name = self.rigid_naming_scheme.format(rigid_no, rib=self)

    @property
    def aoa_relative(self):
        return self.aoa_absolute + self._aoa_diff(self.arcang, self.glide)

    @aoa_relative.setter
    def aoa_relative(self, aoa):
        self.aoa_absolute = aoa - self._aoa_diff(self.arcang, self.glide)

    @cached_property("profile_3d")
    def normvectors(self):
        return map(
            lambda x: self.rotation_matrix([x[0], x[1], 0]), self.profile_2d.normvectors
        )

    @cached_property("arcang", "glide", "zrot", "xrot", "aoa_absolute")
    def rotation_matrix(self):
        zrot = np.arctan(self.arcang) / self.glide * self.zrot
        return rib_rotation(self.aoa_absolute, self.arcang, zrot, self.xrot)

    @cached_property("arcang", "glide", "zrot", "xrot", "aoa_absolute", "chord", "pos")
    def transformation(self):
        zrot = np.arctan(self.arcang) / self.glide * self.zrot
        return rib_transformation(
            self.aoa_absolute, self.arcang, zrot, self.xrot, self.chord, self.pos
        )

    def _te_cut_x(self):
        """Normalized trailing-edge cut position in (0, 1); 1.0 means no cut.

        Derived from ``trailing_edge_cut`` (an absolute length in model units)
        and the local chord, so the tip is cut by a fixed physical length.
        """
        te_cut = getattr(self, "trailing_edge_cut", 0.0)
        if te_cut and self.chord and self.chord > 0:
            cut_x = 1.0 - te_cut / self.chord
            if 0.0 < cut_x < 1.0:
                return cut_x
        return 1.0

    def _get_truncated_profile(self):
        """Return a copy of ``profile_2d`` with the trailing-edge tip truncated
        by ``trailing_edge_cut`` (absolute length in model units), if set.

        The truncation is applied here, at consumption time, rather than by
        editing ``profile_2d`` in place: any resampling of ``profile_2d`` (e.g.
        the 3d preview setting ``profile_numpoints``) would otherwise restore
        the full-chord tip. The x-distribution is simply compressed to
        ``[-cut_x, cut_x]`` so the point count stays constant. Returns an
        untouched copy when no truncation applies.

        Only the rib *piece* is truncated (``get_hull``: 2d template and 3d
        rib mesh).  The skins (``profile_3d``, hence cells, panels and their
        patterns) always run to the trailing edge -- otherwise the whole
        trailing edge would be open.
        """
        profile = copy.deepcopy(self.profile_2d)
        cut_x = self._te_cut_x()
        if cut_x < 1.0:
            profile.x_values = np.array(profile.x_values) * cut_x
        return profile

    @cached_property("self")
    def profile_3d(self):
        # full profile: the skins run to the trailing edge (the trailing-edge
        # truncation only shortens the rib piece, see _get_truncated_profile)
        profile = self.profile_2d
        if profile.data is not None:
            return Profile3D(self.align_all(profile.data))
        else:
            raise ValueError(
                f"no 2d-profile present for the rib at rib {self.name}"
            )

    def point(self, x_value):
        return self.align(self.profile_2d.profilepoint(x_value))

    @staticmethod
    def _aoa_diff(arc_angle, glide):
        ##Formula for aoa rel/abs: ArcTan[Cos[alpha]/gleitzahl]-aoa[rad];
        return np.arctan(np.cos(arc_angle) / glide)

    def mirror(self):
        self.arcang *= -1.0
        self.xrot *= -1.0
        # self.zrot = -self.zrot
        self.pos = np.multiply(self.pos, [1, -1.0, 1])

    def copy(self):
        new = copy.deepcopy(self)
        try:
            new.name += "_copy"
        except TypeError:
            new.name = str(new.name) + "_copy"
        return new

    def is_closed(self):
        return self.profile_2d.has_zero_thickness

    def get_hull(self, glider=None):
        """returns the outer contour of the normalized mesh in form
        of a Polyline (trailing-edge-truncated if ``trailing_edge_cut`` is set)"""
        return self._get_truncated_profile()

    @property
    def base_profile_2d(self):
        """Return the base (non-modified) 2D profile used for x_value indexing."""
        return self.profile_2d

    @property
    def normalized_normale(self):
        return self.rotation_matrix(np.array([0.0, 0.0, 1.0]))

    @property
    def in_plane_normale(self):
        return self.rotation_matrix(np.array([0.0, 1.0, 0.0]))

    def get_attachment_points(self, glider, brake=True):
        return glider.get_rib_attachment_points(self, brake=brake)

    def get_lines(self, glider, brake=False):
        att = self.get_attachment_points(glider, brake=brake)
        connected_lines = set()
        for line in glider.lineset.lines:
            if line.upper_node in att:
                connected_lines.add(line)
        return list(connected_lines)

    def get_mesh(self, hole_num=10, glider=None, filled=False, max_area=None):
        if self.is_closed():
            # stabi
            # TODO: return line
            return Mesh.from_indexed([], {}, {})

        hull_points = list(self.get_hull(glider))
        # A sharp trailing edge closes the contour by duplicating the first point
        # as the last one; drop that duplicate. A truncated (blunt) trailing edge
        # leaves the contour open, so keep every point and let the boundary close
        # the blunt edge — otherwise the last intrados point is dropped and the
        # intrados ends up truncated one point more than the extrados.
        if len(hull_points) > 1 and np.allclose(hull_points[0], hull_points[-1]):
            hull_points = hull_points[:-1]
        vertices = hull_points
        boundary = [list(range(len(vertices))) + [0]]
        hole_centers = []

        if len(self.holes) > 0 and hole_num > 3:
            for nr, hole in enumerate(self.holes):
                start_index = len(vertices)
                hole_vertices = list(
                    hole.get_flattened(self, num=hole_num, scale=False)
                )[:-1]
                hole_indices = list(range(len(hole_vertices))) + [0]
                vertices += hole_vertices
                boundary.append([start_index + i for i in hole_indices])
                hole_centers.append(hole.get_center(self, scale=False).tolist())

        # Cone holes (generated by apply_holes with cone_holes_enabled)
        for nr, hole in enumerate(getattr(self, 'cone_holes', [])):
            try:
                hole_vertices = list(
                    hole.get_flattened(self, num=hole_num, scale=False)
                )[:-1]
                if len(hole_vertices) < 3:
                    continue
                # Check for degenerate (near-zero area) polygons
                area = 0.0
                for i in range(len(hole_vertices)):
                    j = (i + 1) % len(hole_vertices)
                    area += hole_vertices[i][0] * hole_vertices[j][1]
                    area -= hole_vertices[j][0] * hole_vertices[i][1]
                if abs(area) < 1e-12:
                    continue
                start_index = len(vertices)
                hole_indices = list(range(len(hole_vertices))) + [0]
                vertices += hole_vertices
                boundary.append([start_index + i for i in hole_indices])
                hole_centers.append(hole.get_center(self, scale=False).tolist())
            except Exception:
                continue

        if not filled:
            segments = []
            for lst in boundary:
                segments += triangulate.Triangulation.get_segments(lst)
            return Mesh.from_indexed(self.align_all(vertices), {"rib": segments}, {})
        else:
            tri = triangulate.Triangulation(vertices, boundary, hole_centers)
            if max_area is not None:
                tri.meshpy_max_area = max_area
            mesh = tri.triangulate()

            triangles = list(mesh.elements)
            vertices = self.align_all(mesh.points)

            return Mesh.from_indexed(
                vertices,
                polygons={"ribs": triangles},
                boundaries={self.name: list(range(len(vertices)))},
            )


class SingleSkinRib(Rib):
    def __init__(
        self,
        profile_2d=None,
        startpoint=None,
        chord=1.0,
        arcang=0,
        aoa_absolute=0,
        zrot=0,
        xrot=0.0,
        glide=1,
        name="unnamed rib",
        startpos=0.0,
        rigidfoils=None,
        holes=None,
        reinforcements=None,
        rod_sleeves=None,
        material_code=None,
        single_skin_par=None,
    ):
        super().__init__(
            profile_2d=profile_2d,
            startpoint=startpoint,
            chord=chord,
            arcang=arcang,
            aoa_absolute=aoa_absolute,
            zrot=zrot,
            xrot=xrot,
            glide=glide,
            name=name,
            startpos=startpos,
            rigidfoils=rigidfoils,
            holes=holes,
            reinforcements=reinforcements,
            rod_sleeves=rod_sleeves,
            material_code=material_code,
        )
        self.single_skin_par = single_skin_par or {}
        self.shear_map = {}
        self.ss_x_start = None
        self._original_extrados = None
        # Save original aero profile so profile_3d is never polluted by hull bows.
        # (get_glider_3d calls get_hull() and used to overwrite profile_2d — don't.)
        self._aero_profile_2d = self.profile_2d
        self._hull_profile = None  # populated by get_hull() after warp is applied

        # we have to apply this function once for the profile2d
        # this will change the position of the attachmentpoints!
        # therefore it shouldn't be placed int the get_hull function
        if self.single_skin_par["continued_min"]:
            self.apply_continued_min()

    def apply_continued_min(self):
        self.profile_2d.move_nearest_point(self.single_skin_par["continued_min_end"])
        data = self.profile_2d.data
        x, y = data.T
        min_index = y.argmin()
        y_min = y[min_index]
        new_y = []
        for i, xy in enumerate(data):
            if (
                i > min_index
                and (self.single_skin_par["continued_min_end"] - xy[0]) > -1e-8
            ):
                new_y += [
                    y_min
                    + (xy[0] - data[min_index][0])
                    * np.tan(self.single_skin_par["continued_min_angle"])
                ]
            else:
                new_y += [xy[1]]
        self.profile_2d.data = np.array([x, new_y]).T

    @classmethod
    def from_rib(cls, rib, single_skin_par):
        import inspect
        json_dict = rib.__json__()
        json_dict["single_skin_par"] = single_skin_par
        # Filter to only keys accepted by SingleSkinRib.__init__
        valid_params = inspect.signature(cls.__init__).parameters
        json_dict = {k: v for k, v in json_dict.items() if k in valid_params}
        return cls(**json_dict)

    def __json__(self):
        json_dict = super().__json__()
        json_dict["single_skin_par"] = self.single_skin_par
        return json_dict

    @property
    def profile_3d(self):
        """Always built from the ORIGINAL aero profile with shear applied.

        Deliberately NOT cached: shear_map is set at runtime by apply_ss_rib_warp
        and is not part of __json__ / the cache hash key.  If this were cached,
        the version computed before the warp (shear_map={}) would be returned
        for all subsequent calls — including the final lineset.recalc() — causing
        suspension line endpoints to point at the unwarped rib positions.
        """
        aero = getattr(self, '_aero_profile_2d', None) or self.profile_2d
        if aero.data is not None:
            # full profile: the trailing-edge truncation only shortens the rib
            # piece (get_hull), the skins run to the trailing edge
            return Profile3D(self.align_all(aero.data))
        raise ValueError(f"no 2d-profile present for rib {self.name}")

    def get_hull(self, glider=None):
        """
        returns a modified profile2d (with parabolic bows for singleskin).
        Result is cached in self._hull_profile after first computation.
        """
        if self._hull_profile is not None:
            return self._hull_profile

        profile = copy.deepcopy(self.profile_2d)
        if self._original_extrados is None:
            self._original_extrados = profile.get_extrados_poly().data

        attach_pts = glider.get_rib_attachment_points(self)
        pos = list(set([att.rib_pos for att in attach_pts] + [1]))

        if len(pos) > 1:
            span_list = []
            pos.sort()
            # computing the bow start and end points (back to front)
            for i, p in enumerate(pos[:-1]):
                # the profile  has a normed chord of 1
                # so we have to normalize the "att_dist" which is the thickness of
                # rib between two bows. normally something like 2cm
                # length of the flat part at the attachment point
                le_gap = self.single_skin_par["att_dist"] / self.chord / 2
                te_gap = self.single_skin_par["att_dist"] / self.chord / 2

                # le_gap is the gap between the FIRST BOW start and the attachment point next
                # to this point. (before)
                if i == 0 and not self.single_skin_par["le_gap"]:
                    le_gap = 0

                # te_gap is the gap between the LAST BOW end and the trailing edge
                if i == (len(pos) - 2) and not self.single_skin_par["te_gap"]:
                    te_gap = 0

                span_list.append([p + le_gap, pos[i + 1] - te_gap])

            self.ss_x_start = span_list[0][0]

            for k, sp in enumerate(span_list):
                if self.single_skin_par["double_first"] and k == 0:
                    continue  # do not insert points between att for double-first ribs (a-b)

                # first we insert the start and end point of the bow
                profile.insert_point(sp[0])
                profile.insert_point(sp[1])

                # now we remove all points between these two points
                # we have to use a tolerance to not delete the start and end points of the bow.
                # problem: the x-value of a point inserted in a profile can have a slightly different
                # x-value
                profile.remove_points(sp[0], sp[1], tolerance=1e-5)

                # insert sequence of xvalues between start and end. endpoint=False is necessary because
                # start and end are already inserted.
                for i in np.linspace(
                    sp[0], sp[1], self.single_skin_par["num_points"], endpoint=False
                ):
                    profile.insert_point(i)

            # construct shifting function:
            foo_list = []
            for i, sp in enumerate(span_list):
                # parabola from 3 points
                if self.single_skin_par["double_first"] and i == 0:
                    continue
                x0 = np.array(profile.profilepoint(sp[0]))
                x1 = np.array(profile.profilepoint(sp[1]))
                x_mid = (x0 + x1)[0] / 2
                height = abs(
                    profile.profilepoint(-x_mid)[1] - profile.profilepoint(x_mid)[1]
                )
                height *= self.single_skin_par["height"]  # anything bewtween 0..1
                y_mid = profile.profilepoint(x_mid)[1] + height
                x_max = np.array([norm(x1 - x0) / 2, height])

                def foo(x, upper):
                    if not upper and x[0] > x0[0] and x[0] < x1[0]:
                        if (
                            self.single_skin_par["straight_te"]
                            and i == len(span_list) - 1
                        ):
                            return straight_line(x, x0, x1)
                        else:
                            return parabola(x, x0, x1, x_max)
                    else:
                        return x

                profile.apply_function(foo)
        
        # NOTE: The flat pattern of a SingleSkin rib is its 2D profile + parabolic bows.
        # The spanwise warp (shear_map) only affects the 3D positioning via align_all().
        # No coordinate stretching is applied here — that would distort the cut pattern.

        # Trailing-edge truncation (applied last, on the bowed contour).
        cut_x = self._te_cut_x()
        if cut_x < 1.0:
            profile.x_values = np.array(profile.x_values) * cut_x

        self._hull_profile = profile
        return profile

    @property
    def base_profile_2d(self):
        """Return the unmodified 2D profile (no bow shaping) for x_value indexing."""
        return self.profile_2d

    def align_all(self, data):
        """Override to apply progressive Y-shear for SingleSkin ribs."""
        if not self.shear_map or self._original_extrados is None or self.ss_x_start is None:
            return super().align_all(data)

        # Ensure x is increasing for np.interp (extrados points go from TE to LE, i.e. 1 to 0)
        ext_x = self._original_extrados[:, 0][::-1]
        ext_z = self._original_extrados[:, 1][::-1]

        data_3d = set_dimension(data, 3)
        
        # Extract shear_map points for interpolation
        sm_x = sorted(list(self.shear_map.keys()))
        sm_tan = [self.shear_map[x] for x in sm_x]
        
        # We need the first bow's length to transition the twist
        # If there are bows, ss_x_start is the start of the first bow.
        # We ramp from 0 to 1 over the first bow, or just over a fixed length.
        # Given "Option A" - transition over the first bow or at least 10% of chord
        twist_ramp_dist = max(0.1, self.ss_x_start * 0.5)

        for i, point in enumerate(data):
            x, z = point[0], point[1]
            z_extrados = np.interp(x, ext_x, ext_z)
            tan_theta_local = np.interp(x, sm_x, sm_tan) if sm_x else 0.0
            
            # Twist factor: 0 at LE tube, ramps up after ss_x_start
            if x <= self.ss_x_start:
                twist_factor = 0.0
            else:
                twist_factor = min(1.0, (x - self.ss_x_start) / twist_ramp_dist)

            # Use the side of the glider (sign of Y position) to correct the shear direction
            side_sign = np.sign(self.pos[1]) if abs(self.pos[1]) > 1e-4 else 1.0
            y_shift = side_sign * twist_factor * (z - z_extrados) * tan_theta_local
            data_3d[i][2] = y_shift

        return self.transformation.apply(data_3d)


def pseudo_2d_cross(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]


def straight_line(x, x0, x1):
    x_proj = (x - x0).dot(x1 - x0) / norm(x1 - x0) ** 2
    return x0 + (x1 - x0) * x_proj


def parabola(x, x0, x1, x_max):
    """parabola used for singleskin ribs
    x, x0, x1, x_max ... numpy 2d arrays
    xmax = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)"""
    x_proj = (x - x0).dot(x1 - x0) / norm(x1 - x0) ** 2
    x_proj = (x_proj - 0.5) * 2
    y_proj = -(x_proj**2)
    x = np.array([x_proj, y_proj]) * x_max
    c = (x1 - x0)[0] / norm(x1 - x0)
    s = (x1 - x0)[1] / norm(x1 - x0)
    rot = np.array([[c, -s], [s, c]])
    null = -x_max
    return rot.dot(x - null) + x0


# def rib_rotation(aoa, arc, zrot):
#     """
#     Rotation Matrix for Ribs, aoa, arcwide-angle and glidewise angle in radians
#     return -> np.array
#     """
#     # Rotate Arcangle, rotate from lying to standing (x-z)
#     rot = rotation_3d(-arc + math.pi / 2, [-1, 0, 0])
#     axis = rot.dot([0, 0, 1])
#     rot = rotation_3d(aoa, axis).dot(rot)
#     axis = rot.dot([0, 1, 0])
#     rot = rotation_3d(zrot, axis).dot(rot)
#     # rot = rotation_3d(-math.pi/2, [0, 0, 1]).dot(rot)

#     return rot


def rib_rotation(aoa, arc, zrot, xrot=0):
    rot0 = Rotation(np.pi / 2 - xrot, [1, 0, 0])
    rot1 = Rotation(aoa, [0, 1, 0])
    rot2 = Rotation(-arc, [1, 0, 0])
    axis = (rot1 * rot2)([0, 0, 1])
    rot3 = Rotation(-zrot, axis)
    return rot0 * rot1 * rot2 * rot3


def rib_transformation(aoa, arc, zrot, xrot, scale, pos):
    scale = Scale(scale)
    move = Translation(pos)
    rot = rib_rotation(aoa, arc, zrot, xrot)
    return scale * rot * move


class TipCurveRib(Rib):
    """Virtual last rib of a rounded, rib-less wingtip.

    The tip cell is closed by a *curve* instead of a rib: it starts at the
    nose of the previous rib, bulges spanwise until it touches the tip station
    (the straight chord line this rib would have) tangentially, and comes back
    to the previous rib's trailing edge.  The extrados skin is stretched
    between the previous rib's upper surface and that curve, the intrados skin
    between the lower surface and the same curve, so both skins are sewn
    together along it.  There is no rib piece: the 2d profile is a
    zero-thickness line, so the rib is "closed" (no rib mesh, no rib pattern).
    The cell's (symmetric) ballooning then inflates both skins.

    The curve is exposed as this rib's ``profile_3d``, so every consumer of the
    cell (midribs, panels, flattening, hull views) sees it as the outer
    boundary: point ``i`` of the curve pairs with point ``i`` of the previous
    rib, the upper and lower points of one chord position sharing one 3d point.

    ``curve_position``: chord fraction (0 = nose, 1 = trailing edge) where the
    curve touches the tip station.  ``curve_power`` shapes the outline:
    0.5 = elliptic (round), 1 = parabolic (pointier), smaller = squarer.
    Without ``previous_rib`` (e.g. a rib restored from json) the curve
    degenerates to the plain chord line.
    """

    def __init__(self, *args, previous_rib=None, curve_position=0.5,
                 curve_power=0.5, **kwargs):
        super().__init__(*args, **kwargs)
        self.previous_rib = previous_rib
        self.curve_position = float(curve_position)
        self.curve_power = float(curve_power)

    def __json__(self):
        data = super().__json__()
        data["curve_position"] = self.curve_position
        data["curve_power"] = self.curve_power
        return data

    def curve_fraction(self, chord_fraction):
        """Spanwise position of the tip curve (0 = previous rib, 1 = tip
        station) for chord fractions in [0, 1]: 0 at both ends, 1 with a
        horizontal tangent at ``curve_position``."""
        c = np.clip(np.asarray(chord_fraction, dtype=float), 0.0, 1.0)
        cs = min(max(self.curve_position, 0.01), 0.99)
        u = np.where(c <= cs, (cs - c) / cs, (c - cs) / (1.0 - cs))
        return np.clip(1.0 - u * u, 0.0, 1.0) ** max(self.curve_power, 1e-3)

    @property
    def profile_3d(self):
        """The tip curve, sampled like a profile (not cached: it depends on
        the previous rib, which is not part of this rib's hash)."""
        # straight chord line at the tip station (what Rib.profile_3d gives)
        line = Profile3D(self.align_all(self.profile_2d.data))
        prev = self.previous_rib
        if prev is None:
            return line
        p1 = np.array(prev.profile_3d.data, dtype=float)
        p2 = np.array(line.data, dtype=float)
        if len(p1) != len(p2) or len(p1) < 3:
            return line
        nose1 = p1[prev.profile_2d.noseindex]
        te1 = 0.5 * (p1[0] + p1[-1])
        nose2 = p2[self.profile_2d.noseindex]
        te2 = 0.5 * (p2[0] + p2[-1])
        # chord fraction of every point on the chord lines
        c = np.abs(np.array(self.profile_2d.x_values, dtype=float))
        f = self.curve_fraction(c)[:, None]
        on_prev = nose1 + c[:, None] * (te1 - nose1)
        on_tip = nose2 + c[:, None] * (te2 - nose2)
        return Profile3D((1.0 - f) * on_prev + f * on_tip)

    def seam_branches(self, t):
        """Chord fractions (front, back) where the seam crosses the spanwise
        parameter ``t`` (0 = previous rib, 1 = tip station): the two solutions
        of ``curve_fraction(c) == t``.  (0, 1) at t=0, both ``curve_position``
        at t=1."""
        t = min(max(float(t), 0.0), 1.0)
        cs = min(max(self.curve_position, 0.01), 0.99)
        power = max(self.curve_power, 1e-3)
        u = np.sqrt(max(0.0, 1.0 - t ** (1.0 / power)))
        return cs * (1.0 - u), cs + (1.0 - cs) * u

    def tip_line(self):
        """3d nose and trailing-edge (mid) points of the straight tip station."""
        line = np.array(self.align_all(self.profile_2d.data), dtype=float)
        return line[self.profile_2d.noseindex], 0.5 * (line[0] + line[-1])

    @property
    def apex(self):
        """3d point where the seam touches the tip station."""
        nose, te = self.tip_line()
        cs = min(max(self.curve_position, 0.01), 0.99)
        return nose + cs * (te - nose)
