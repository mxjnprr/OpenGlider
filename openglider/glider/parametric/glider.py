
import copy
import logging
import math

import numpy as np

try:
    import matplotlib.path as mpl_path
except ImportError:
    mpl_path = None

from openglider.airfoil import Profile2D
from openglider.glider.cell import Cell, DiagonalRib, Panel, TensionLine, TensionStrap
from openglider.glider.cell.elements import LeadingEdgeClosure, PanelRigidFoil
from openglider.glider.glider import Glider
from openglider.glider.parametric.export_ods import export_ods_2d
from openglider.glider.parametric.fitglider import fit_glider_3d
from openglider.glider.parametric.import_ods import import_ods_2d
from openglider.glider.parametric.lines import LineSet2D, UpperNode2D
from openglider.glider.parametric.shape import ParametricShape
from openglider.glider.rib import MiniRib, Rib, RibHole, RigidFoil
from openglider.utils import ZipCmp
from openglider.utils.distribution import Distribution
from openglider.utils.table import Table


class ParametricGlider:
    """
    A parametric (2D) Glider object used for gui input
    """

    num_arc_positions = 60
    num_shape = 30
    num_interpolate_ribs = 40
    num_cell_dist = 30
    num_depth_integral = 100
    num_interpolate = 30
    num_profile = None

    def __init__(
        self,
        shape,
        arc,
        aoa,
        profiles,
        profile_merge_curve,
        balloonings,
        ballooning_merge_curve,
        lineset,
        speed,
        glide,
        zrot,
        elements=None,
        **kwargs
    ):
        self.zrot = zrot or aoa
        self.shape: ParametricShape = shape
        self.arc = arc
        self.aoa = aoa
        self.profiles = profiles or []
        self.profile_merge_curve = profile_merge_curve
        self.balloonings = balloonings or []
        self.ballooning_merge_curve = ballooning_merge_curve
        self.lineset = lineset or LineSet2D([])
        self.speed = speed
        self.glide = glide
        self.elements = elements or {}

        # Hole properties
        self.holes = kwargs.get('holes', True)
        self.hole_shape_ns = kwargs.get('hole_shape_ns', 0)  # Default to Ellipse
        self.num_holes_ns = kwargs.get('num_holes_ns', 30)
        self.hole_width_ns = kwargs.get('hole_width_ns', 0.003)
        self.hole_height_ns = kwargs.get('hole_height_ns', 0.8)
        self.vertical_shift_ns = kwargs.get('vertical_shift_ns', 0.0)
        self.min_hole_pos_ns = kwargs.get('min_hole_pos_ns', 0.2)
        self.max_hole_pos_ns = kwargs.get('max_hole_pos_ns', 0.8)
        self.hole_height_mode_ns = kwargs.get('hole_height_mode_ns', 0)  # 0: Percent, 1: Margin
        self.hole_margin_ns = kwargs.get('hole_margin_ns', 0.02)    # 20mm
        self.hole_corner_radius_ns = kwargs.get('hole_corner_radius_ns', 0.25)  # Ratio of min dimension (0.25 = 25%)

        self.hole_shape_s = kwargs.get('hole_shape_s', 0)  # Default to Ellipse
        self.num_holes_s = kwargs.get('num_holes_s', 30)
        self.hole_width_s = kwargs.get('hole_width_s', 0.003)
        self.hole_height_s = kwargs.get('hole_height_s', 0.8)
        self.vertical_shift_s = kwargs.get('vertical_shift_s', 0.0)
        self.min_hole_pos_s = kwargs.get('min_hole_pos_s', 0.2)
        self.max_hole_pos_s = kwargs.get('max_hole_pos_s', 0.8)
        self.hole_height_mode_s = kwargs.get('hole_height_mode_s', 0)   # 0: Percent, 1: Margin
        self.hole_margin_s = kwargs.get('hole_margin_s', 0.02)     # 20mm
        self.hole_corner_radius_s = kwargs.get('hole_corner_radius_s', 0.25)  # Ratio of min dimension (0.25 = 25%)

        # No-hole zone parameters for suspended ribs
        self.hole_free_angle_s = kwargs.get('hole_free_angle_s', 30.0)  # degrees
        self.hole_arc_span_s = kwargs.get('hole_arc_span_s', 120.0)  # degrees (arc span on halfmoon)
        
        # Cone hole parameters (holes inside exclusion cones)
        self.cone_holes_enabled_s = kwargs.get('cone_holes_enabled_s', False)
        self.cone_hole_num_zones_s = kwargs.get('cone_hole_num_zones_s', 1)
        self.cone_hole_margin_top_s = kwargs.get('cone_hole_margin_top_s', 3.0)     # mm
        self.cone_hole_margin_side_s = kwargs.get('cone_hole_margin_side_s', 3.0)   # mm
        self.cone_hole_margin_bottom_s = kwargs.get('cone_hole_margin_bottom_s', 3.0)  # mm
        self.cone_hole_corner_radius_s = kwargs.get('cone_hole_corner_radius_s', 25.0)  # %

        # Airfoil Structure - Extrados Sleeve (suspended)
        self.extrados_sleeve_enabled_s = kwargs.get('extrados_sleeve_enabled_s', False)
        self.extrados_sleeve_width_s = kwargs.get('extrados_sleeve_width_s', 0.015)
        self.extrados_sleeve_offset_s = kwargs.get('extrados_sleeve_offset_s', 0.003)
        self.extrados_sleeve_start_s = kwargs.get('extrados_sleeve_start_s', 0.0)
        self.extrados_sleeve_end_s = kwargs.get('extrados_sleeve_end_s', 0.9)
        self.extrados_sleeve_le_angle_s = kwargs.get('extrados_sleeve_le_angle_s', 80.0)
        self.extrados_sleeve_le_length_s = kwargs.get('extrados_sleeve_le_length_s', 0.03)
        self.extrados_sleeve_te_angle_s = kwargs.get('extrados_sleeve_te_angle_s', 80.0)
        self.extrados_sleeve_te_length_s = kwargs.get('extrados_sleeve_te_length_s', 0.03)
        
        # Airfoil Structure - Intrados Sleeve (suspended)
        self.intrados_sleeve_enabled_s = kwargs.get('intrados_sleeve_enabled_s', False)
        self.intrados_sleeve_width_s = kwargs.get('intrados_sleeve_width_s', 0.015)
        self.intrados_sleeve_offset_s = kwargs.get('intrados_sleeve_offset_s', 0.003)
        self.intrados_sleeve_start_s = kwargs.get('intrados_sleeve_start_s', 0.0)
        self.intrados_sleeve_end_s = kwargs.get('intrados_sleeve_end_s', 0.9)
        self.intrados_sleeve_le_angle_s = kwargs.get('intrados_sleeve_le_angle_s', 100.0)
        self.intrados_sleeve_le_length_s = kwargs.get('intrados_sleeve_le_length_s', 0.03)
        self.intrados_sleeve_te_angle_s = kwargs.get('intrados_sleeve_te_angle_s', 100.0)
        self.intrados_sleeve_te_length_s = kwargs.get('intrados_sleeve_te_length_s', 0.03)
        
        # Airfoil Structure - Extrados Sleeve (non-suspended)
        self.extrados_sleeve_enabled_ns = kwargs.get('extrados_sleeve_enabled_ns', False)
        self.extrados_sleeve_width_ns = kwargs.get('extrados_sleeve_width_ns', 0.015)
        self.extrados_sleeve_offset_ns = kwargs.get('extrados_sleeve_offset_ns', 0.003)
        self.extrados_sleeve_start_ns = kwargs.get('extrados_sleeve_start_ns', 0.0)
        self.extrados_sleeve_end_ns = kwargs.get('extrados_sleeve_end_ns', 0.9)
        self.extrados_sleeve_le_angle_ns = kwargs.get('extrados_sleeve_le_angle_ns', 80.0)
        self.extrados_sleeve_le_length_ns = kwargs.get('extrados_sleeve_le_length_ns', 0.03)
        self.extrados_sleeve_te_angle_ns = kwargs.get('extrados_sleeve_te_angle_ns', 80.0)
        self.extrados_sleeve_te_length_ns = kwargs.get('extrados_sleeve_te_length_ns', 0.03)
        
        # Airfoil Structure - Intrados Sleeve (non-suspended)
        self.intrados_sleeve_enabled_ns = kwargs.get('intrados_sleeve_enabled_ns', False)
        self.intrados_sleeve_width_ns = kwargs.get('intrados_sleeve_width_ns', 0.015)
        self.intrados_sleeve_offset_ns = kwargs.get('intrados_sleeve_offset_ns', 0.003)
        self.intrados_sleeve_start_ns = kwargs.get('intrados_sleeve_start_ns', 0.0)
        self.intrados_sleeve_end_ns = kwargs.get('intrados_sleeve_end_ns', 0.9)
        self.intrados_sleeve_le_angle_ns = kwargs.get('intrados_sleeve_le_angle_ns', 100.0)
        self.intrados_sleeve_le_length_ns = kwargs.get('intrados_sleeve_le_length_ns', 0.03)
        self.intrados_sleeve_te_angle_ns = kwargs.get('intrados_sleeve_te_angle_ns', 100.0)
        self.intrados_sleeve_te_length_ns = kwargs.get('intrados_sleeve_te_length_ns', 0.03)
        
        # Airfoil Structure - Reinforcements (suspended only)
        self.reinforcement_enabled_s = kwargs.get('reinforcement_enabled_s', True)  # Enabled by default
        self.reinforcement_apply_all_s = kwargs.get('reinforcement_apply_all_s', False)  # Individual config by default
        self.reinforcement_master_s = kwargs.get('reinforcement_master_s', {
            'enabled': True,
            'surface_offset': 0.0005,  # 0.5mm
            'halfmoon_radius': 0.1,    # 100mm
            'rod_enabled': True,
            'rod_offset': 0.008,       # 8mm
            'rod_width': 0.009,        # 9mm
            'rod_end_offset': 1.0,     # 1°
        })
        self.reinforcement_configs_s = kwargs.get('reinforcement_configs_s', [])
        self.reinforcement_excluded_ribs_s = kwargs.get('reinforcement_excluded_ribs_s', [])

        # Shark-nose reinforcement (front-most attachment point, suspended only).
        # A rounded bounding box confined to the intrados: bottom edge follows the
        # intrados from start to end (chord fractions), the lid is a constant-
        # thickness band (depth, m) inward from the intrados, each end cap cut at
        # start_angle / end_angle degrees (90 = perpendicular), rounded corners.
        self.shark_nose_s = kwargs.get('shark_nose_s', {
            'enabled': False,
            'start': 0.03,          # box start on the intrados (chord fraction)
            'end': 0.35,            # box end on the intrados (chord fraction)
            'depth': 0.035,         # constant band thickness (35mm)
            'start_angle': 90.0,    # nose-side end cap angle (deg)
            'end_angle': 90.0,      # trailing-side end cap angle (deg)
            'corner_radius': 0.01,  # corner rounding radius (10mm); 0 = sharp
            'rod': True,            # include the attachment rod sleeve
            'depth_relative': False,  # thickness as a chord fraction (scales with size)
        })

        # Multi-rod sleeves (NEW FORMAT - list of configs)
        # Suspended ribs
        self.extrados_sleeves_enabled_s = kwargs.get('extrados_sleeves_enabled_s', True)
        self.extrados_sleeves_s = kwargs.get('extrados_sleeves_s', [])
        self.intrados_sleeves_enabled_s = kwargs.get('intrados_sleeves_enabled_s', True)
        self.intrados_sleeves_s = kwargs.get('intrados_sleeves_s', [])
        # Non-suspended ribs
        self.extrados_sleeves_enabled_ns = kwargs.get('extrados_sleeves_enabled_ns', True)
        self.extrados_sleeves_ns = kwargs.get('extrados_sleeves_ns', [])
        self.intrados_sleeves_enabled_ns = kwargs.get('intrados_sleeves_enabled_ns', True)
        self.intrados_sleeves_ns = kwargs.get('intrados_sleeves_ns', [])

        # Edit Cells - Diagonals Auto-fill configuration
        self.diagonal_autofill_params = kwargs.get('diagonal_autofill_params', {
            "A": (4.0, 5.0, 15.0, 1),   # (intrados_cm, extrados_start_%, extrados_end_%, num_bands)
            "B": (4.0, 15.0, 30.0, 1),
            "C": (4.0, 30.0, 50.0, 1),
            "D": (4.0, 50.0, 75.0, 1),
        })
        self.diagonal_autofill_offset = kwargs.get('diagonal_autofill_offset', 0)  # mm

        # Lines Auto-Placement configuration
        self.lines_placement_config = kwargs.get('lines_placement_config', {
            "demi_ecartement": 20,      # cm (half-span spread)
            "profondeur": 20.0,         # % of central chord (depth)
            "hauteur_cone": 7.0,
            "riser_length": 0.47,
            "basses_auto": True,
            "basses_length": 2.0,
            "inter_auto": True,
            "inter_length": 1.0,
            "include_stabilo": True,
            "stabilo_position": 50.0,
            "line_type_lowers": "default",
            "line_type_mid": "default",
            "line_type_uppers": "default",
            "fork_angle_auto": False,
            "fork_angle_max": 15.0,
            "line_types": {
                "A": {"enabled": True, "position": 8.5, "interval": 1, "start_cell": 0, "patterns": "3:1, 2:2:1"},
                "B": {"enabled": True, "position": 27.5, "interval": 2, "start_cell": 0, "patterns": "2:2:1, 2:1"},
                "C": {"enabled": True, "position": 53.0, "interval": 2, "start_cell": 0, "patterns": "2:1"},
                "D": {"enabled": True, "position": 77.0, "interval": 3, "start_cell": 0, "patterns": "2:1"},
                "F": {"enabled": True, "position": 100.0, "interval": 1, "start_cell": 0, "patterns": "1:1"},
            }
        })

        # =====================================================================
        # Profile Control - Unified airfoil management
        # =====================================================================
        
        # Thickness curve: controls relative thickness scaling along span
        # Values: 1.0 = original thickness, 0.8 = 80%, 1.2 = 120%
        self.thickness_curve = kwargs.get('thickness_curve', None)  # SymmetricBSpline or None
        self.thickness_curve_enabled = kwargs.get('thickness_curve_enabled', False)
        
        # Shark nose: intrados shelf + angled drop at leading edge
        self.sharknose_enabled = kwargs.get('sharknose_enabled', False)
        self.sharknose_start = kwargs.get('sharknose_start', 0.03)  # Shelf start on intrados (% chord, near nose)
        self.sharknose_end = kwargs.get('sharknose_end', 0.08)  # Drop position on intrados (% chord, further from nose)
        self.sharknose_angle = kwargs.get('sharknose_angle', 5.0)  # Tilt of drop from vertical (degrees)
        self.sharknose_curve = kwargs.get('sharknose_curve', None)  # SymmetricBSpline for spanwise variation
        self.sharknose_cells = kwargs.get('sharknose_cells', None)  # List of cell indices (None = all)
        
        # Profile overrides: rib-specific profile assignment
        # Format: {rib_index: profile_index} - overrides the distribution curve for specific ribs
        self.profile_overrides = kwargs.get('profile_overrides', {})
        self.profile_overrides_enabled = kwargs.get('profile_overrides_enabled', False)
        
        # Last rib profile (stabilo/wingtip) - applied to the very last rib
        self.last_profile_enabled = kwargs.get('last_profile_enabled', False)
        self.last_profile_type = kwargs.get('last_profile_type', 'line')  # 'line', 'thin', 'custom'
        self.last_profile_thickness = kwargs.get('last_profile_thickness', 0.3)  # Relative thickness (0.3 = 30% of original)
        self.last_profile_custom = kwargs.get('last_profile_custom', None)  # Custom Profile2D

        # Trailing-edge truncation: cut the profile tip at the trailing edge by
        # an absolute length (millimetres) so the rib does not reach all the way
        # to the tip, e.g. to let sand drain out of the ribs during build.
        self.te_cut_enabled = kwargs.get('te_cut_enabled', False)
        self.te_cut_mm = kwargs.get('te_cut_mm', 15.0)

        # Aerodynamic analysis results (stored for use by Lines Auto-placement tool)
        self.aerodynamics_results = kwargs.get('aerodynamics_results', None)

        # Single Skin configuration
        self.single_skin_config = kwargs.get('single_skin_config', None)

    def get_sleeve_exclusion_zones(self, rib, rib_idx, is_suspended):
        """
        Get chord ranges that should be excluded from hole placement due to rod sleeves.
        Returns list of (start_chord, end_chord) tuples.
        """
        suffix = '_s' if is_suspended else '_ns'
        exclusion_zones = []
        
        # Check extrados sleeves
        if getattr(self, f'extrados_sleeves_enabled{suffix}', True):
            for config in getattr(self, f'extrados_sleeves{suffix}', []):
                excluded_ribs = config.get('excluded_ribs', [])
                if rib_idx not in excluded_ribs:
                    start = config.get('start_chord', 0.0)
                    end = config.get('end_chord', 0.7)
                    if start < end:
                        exclusion_zones.append((start, end))
        
        # Check intrados sleeves
        if getattr(self, f'intrados_sleeves_enabled{suffix}', True):
            for config in getattr(self, f'intrados_sleeves{suffix}', []):
                excluded_ribs = config.get('excluded_ribs', [])
                if rib_idx not in excluded_ribs:
                    start = config.get('start_chord', 0.06)
                    end = config.get('end_chord', 0.5)
                    if start < end:
                        exclusion_zones.append((start, end))
        
        return exclusion_zones
    
    def get_reinforcement_exclusion_zones(self, rib, glider):
        """
        Get chord ranges that should be excluded from hole placement due to attachment reinforcements.
        Returns list of (start_chord, end_chord) tuples.
        """
        exclusion_zones = []
        
        if not getattr(self, 'reinforcement_enabled_s', False):
            return exclusion_zones
        
        apply_all = getattr(self, 'reinforcement_apply_all_s', False)
        master_config = getattr(self, 'reinforcement_master_s', {})
        configs = getattr(self, 'reinforcement_configs_s', [])
        
        attachment_points = glider.get_rib_attachment_points(rib)
        valid_aps = [ap for ap in attachment_points if ap.rib_pos <= 0.90]
        valid_aps.sort(key=lambda x: x.rib_pos)
        
        for i, ap in enumerate(valid_aps):
            if apply_all:
                config = master_config
            else:
                config = configs[i] if i < len(configs) else master_config
            
            if config.get('enabled', True):
                radius_normalized = config.get('halfmoon_radius', 0.03) / rib.chord
                start = max(0.0, ap.rib_pos - radius_normalized)
                end = min(1.0, ap.rib_pos + radius_normalized)
                exclusion_zones.append((start, end))
        
        return exclusion_zones
    
    def subtract_exclusion_zones(self, allowed_ranges, exclusion_zones):
        """
        Remove exclusion zones from allowed ranges.
        Both inputs are lists of (start, end) tuples.
        Returns a new list of allowed (start, end) tuples.
        """
        if not exclusion_zones:
            return allowed_ranges
        
        # Sort exclusion zones by start position
        exclusions = sorted(exclusion_zones, key=lambda x: x[0])
        
        result = []
        for start, end in allowed_ranges:
            current_start = start
            for ex_start, ex_end in exclusions:
                if ex_end <= current_start or ex_start >= end:
                    # No overlap with this exclusion
                    continue
                if ex_start > current_start:
                    # Add the gap before this exclusion
                    result.append((current_start, min(ex_start, end)))
                current_start = max(current_start, ex_end)
                if current_start >= end:
                    break
            if current_start < end:
                result.append((current_start, end))
        
        return result

    @staticmethod
    def _round_quad_corners_static(p0, p1, p2, p3, radius_fraction):
        """Generate a polygon with rounded corners for a quadrilateral.
        
        Args:
            p0, p1, p2, p3: Corner points (numpy arrays) in order.
            radius_fraction: How much to round (0.0 = sharp, 0.5 = max)
        
        Returns:
            List of points forming the rounded polygon (closed).
        """
        corners = [p0, p1, p2, p3]
        n = len(corners)
        pts = []
        num_arc_segments = 4

        for i in range(n):
            prev_corner = corners[(i - 1) % n]
            curr_corner = corners[i]
            next_corner = corners[(i + 1) % n]

            to_prev = prev_corner - curr_corner
            to_next = next_corner - curr_corner

            len_prev = np.linalg.norm(to_prev)
            len_next = np.linalg.norm(to_next)

            if len_prev < 1e-9 or len_next < 1e-9:
                pts.append(curr_corner)
                continue

            max_r = min(len_prev, len_next) * 0.5
            r = radius_fraction * max_r

            start_pt = curr_corner + (to_prev / len_prev) * r
            end_pt = curr_corner + (to_next / len_next) * r

            for j in range(num_arc_segments + 1):
                t = j / num_arc_segments
                pt = (1 - t)**2 * start_pt + 2 * (1 - t) * t * curr_corner + t**2 * end_pt
                pts.append(pt)

        pts.append(pts[0])  # Close the polygon
        return pts

    def apply_holes(self, glider):
        if not self.holes:
            return

        # Filter suspended ribs - exclude brake attachments (>90% chord position)
        # Only consider true suspension attachments (A/B/C lines), not brake tab attachments
        suspended_ribs = {
            att.rib for att in glider.lineset.attachment_points 
            if hasattr(att, 'rib') and hasattr(att, 'rib_pos') and att.rib_pos <= 0.9
        }

        NO_HOLE_ZONE_BASE_CHORD_FRACTION = 0.05
        
        # Minimum chord for hole generation - skip tiny ribs (stabilos) that cause mesh issues
        MIN_CHORD_FOR_HOLES = 0.15  # 15cm minimum chord (reduced from 30cm)

        for rib_idx, rib in enumerate(glider.ribs):

            # Skip sealed ribs (solid walls around SS zone — no holes)
            ss_sealed = getattr(glider, '_ss_sealed_rib_indices', set())
            if rib_idx in ss_sealed:
                continue

            # Skip the last rib if last_profile_enabled (should be solid, no holes)
            if getattr(self, 'last_profile_enabled', False) and rib_idx == len(glider.ribs) - 1:
                continue
            
            # Skip ribs with chord too small for reliable hole generation
            if rib.chord < MIN_CHORD_FOR_HOLES:
                continue
                
            is_suspended = rib in suspended_ribs

            if is_suspended:
                shape_idx, num_holes, w_factor, h_factor, v_shift_factor, start_pos, end_pos, hole_height_mode, hole_margin, corner_radius = (
                    getattr(self, 'hole_shape_s', 0), self.num_holes_s, self.hole_width_s,
                    self.hole_height_s, self.vertical_shift_s, self.min_hole_pos_s, self.max_hole_pos_s,
                    self.hole_height_mode_s, self.hole_margin_s, getattr(self, 'hole_corner_radius_s', 0.005)
                )
                no_hole_zones = []
                attachment_points = glider.get_rib_attachment_points(rib)
                halfmoon_circles = []  # For halfmoon exclusion (same as preview)
                
                # Get reinforcement config (needed for expanded exclusion zones)
                halfmoon_radius_norm_global = 0.0
                if getattr(self, 'reinforcement_enabled_s', False):
                    master_config = getattr(self, 'reinforcement_master_s', {})
                    halfmoon_radius_norm_global = master_config.get('halfmoon_radius', 0.03) / rib.chord
                
                # Get pilot point 2D coordinates (same method as compute_pull_axis_projection)
                pilot_2d = None
                if hasattr(glider, 'lineset') and glider.lineset:
                    try:
                        main_ap = glider.lineset.get_main_attachment_point()
                        if main_ap is not None and hasattr(main_ap, 'vec') and main_ap.vec is not None:
                            pilot_point_3d = np.array(main_ap.vec)
                            
                            # Project pilot to rib's 2D coordinate frame
                            le_3d = np.array(rib.profile_3d.data[rib.profile_2d.noseindex])
                            te_3d = (np.array(rib.profile_3d.data[0]) + np.array(rib.profile_3d.data[-1])) / 2
                            chord_3d = le_3d - te_3d
                            chord_length = np.linalg.norm(chord_3d)
                            if chord_length > 1e-9:
                                chord_dir = chord_3d / chord_length
                                upper_idx = len(rib.profile_3d.data) // 4
                                upper_3d = np.array(rib.profile_3d.data[upper_idx])
                                v1_3d = upper_3d - te_3d
                                normal = np.cross(chord_3d, v1_3d)
                                norm_len = np.linalg.norm(normal)
                                normal = normal / norm_len if norm_len > 0 else np.array([1, 0, 0])
                                up_dir = np.cross(normal, chord_dir)
                                up_norm = np.linalg.norm(up_dir)
                                up_dir = up_dir / up_norm if up_norm > 0 else np.array([0, 0, 1])
                                pilot_rel_3d = pilot_point_3d - te_3d
                                pilot_chord_pos = np.dot(pilot_rel_3d, chord_dir) / chord_length
                                pilot_up_pos = np.dot(pilot_rel_3d, up_dir) / chord_length
                                te_2d_x = (rib.profile_2d.data[0][0] + rib.profile_2d.data[-1][0]) / 2
                                le_2d_x = rib.profile_2d.data[rib.profile_2d.noseindex][0]
                                pilot_2d_x = te_2d_x + pilot_chord_pos * (le_2d_x - te_2d_x)
                                pilot_2d_y = pilot_up_pos
                                pilot_2d = np.array([pilot_2d_x, pilot_2d_y])
                    except Exception:
                        pass
                for ap in attachment_points:
                    v1 = rib.profile_2d.align([ap.rib_pos, -1.0]) # Apex on intrados

                    angle_rad = np.deg2rad(self.hole_free_angle_s)

                    # Calculate angle_offset from pilot direction (same as preview/compute_pull_axis_projection)
                    if pilot_2d is not None:
                        line_direction = v1 - pilot_2d
                        if np.linalg.norm(line_direction) > 1e-9:
                            line_direction = line_direction / np.linalg.norm(line_direction)
                        else:
                            line_direction = np.array([0, 1])
                        angle_offset = np.arctan2(line_direction[1], line_direction[0])
                    else:
                        # Fallback to local vertical
                        angle_offset = np.pi / 2

                    dir2 = np.array([np.cos(angle_offset - angle_rad), np.sin(angle_offset - angle_rad)])
                    dir3 = np.array([np.cos(angle_offset + angle_rad), np.sin(angle_offset + angle_rad)])

                    # Find intersection of these lines with the upper surface (extrados)
                    extrados_poly = rib.profile_2d.get_extrados_poly()

                    far_factor = 10.0  # Normalized coordinates
                    
                    if halfmoon_radius_norm_global > 1e-6:
                        # Build expanded exclusion polygon (same as preview)
                        halfmoon_circles.append((v1, halfmoon_radius_norm_global))
                        arc_span_rad = np.deg2rad(getattr(self, 'hole_arc_span_s', 120.0))
                        half_arc = arc_span_rad / 2.0
                        
                        arc_angle_left = angle_offset + half_arc
                        arc_angle_right = angle_offset - half_arc
                        
                        v1_left = v1 + halfmoon_radius_norm_global * np.array(
                            [np.cos(arc_angle_left), np.sin(arc_angle_left)])
                        v1_right = v1 + halfmoon_radius_norm_global * np.array(
                            [np.cos(arc_angle_right), np.sin(arc_angle_right)])
                        
                        v2_new = extrados_poly.line_intersection(v1_left, v1_left + dir3 * far_factor)
                        v3_new = extrados_poly.line_intersection(v1_right, v1_right + dir2 * far_factor)
                        
                        if v2_new is not None and v3_new is not None:
                            # Build full halfmoon arc points
                            full_halfmoon = []
                            num_arc_pts = 20
                            for i in range(num_arc_pts + 1):
                                t = i / num_arc_pts
                                arc_ang = np.pi + t * (-np.pi)
                                arc_pt = v1 + halfmoon_radius_norm_global * np.array(
                                    [np.cos(arc_ang), np.sin(arc_ang)])
                                full_halfmoon.append(arc_pt)
                            
                            # Trace extrados curve between intersection points
                            extrados_curve = []
                            v2_x, v3_x = v2_new[0], v3_new[0]
                            num_ext_pts = 15
                            for i in range(num_ext_pts + 1):
                                t = i / num_ext_pts
                                x = v2_x + t * (v3_x - v2_x)
                                ext_pts = [p for p in rib.profile_2d.data if p[1] > 0]
                                closest = min(ext_pts, key=lambda p: abs(p[0] - x), default=None)
                                if closest is not None:
                                    extrados_curve.append(np.array(closest))
                                else:
                                    y = v2_new[1] + t * (v3_new[1] - v2_new[1])
                                    extrados_curve.append(np.array([x, y]))
                            
                            # Full exclusion polygon: halfmoon + sides + extrados
                            exclusion_polygon = full_halfmoon + [v3_new] + list(reversed(extrados_curve)) + [v2_new, full_halfmoon[0]]
                            no_hole_zones.append(tuple(exclusion_polygon))
                        else:
                            # Fallback to simple triangle
                            v2 = extrados_poly.line_intersection(v1, v1 + dir2 * far_factor)
                            v3 = extrados_poly.line_intersection(v1, v1 + dir3 * far_factor)
                            if v2 is not None and v3 is not None:
                                no_hole_zones.append((v1, v2, v3))
                    else:
                        # No halfmoon - simple triangle
                        v2 = extrados_poly.line_intersection(v1, v1 + dir2 * far_factor)
                        v3 = extrados_poly.line_intersection(v1, v1 + dir3 * far_factor)
                        if v2 is not None and v3 is not None:
                            no_hole_zones.append((v1, v2, v3))
                
                # === CONE HOLE GENERATION ===
                if getattr(self, 'cone_holes_enabled_s', False):
                  try:
                    num_zones = getattr(self, 'cone_hole_num_zones_s', 1)
                    margin_top_m = getattr(self, 'cone_hole_margin_top_s', 3.0) / 1000.0
                    margin_side_m = getattr(self, 'cone_hole_margin_side_s', 3.0) / 1000.0
                    margin_bottom_m = getattr(self, 'cone_hole_margin_bottom_s', 3.0) / 1000.0
                    cone_corner_pct = getattr(self, 'cone_hole_corner_radius_s', 25.0) / 100.0
                    
                    margin_top = margin_top_m / rib.chord
                    margin_side = margin_side_m / rib.chord
                    margin_bottom = margin_bottom_m / rib.chord
                    
                    # Get reinforcement radius
                    halfmoon_radius_norm = 0.0
                    if getattr(self, 'reinforcement_enabled_s', False):
                        master_config = getattr(self, 'reinforcement_master_s', {})
                        halfmoon_radius_norm = master_config.get('halfmoon_radius', 0.03) / rib.chord
                    
                    for ap in attachment_points:
                        v1 = rib.profile_2d.align([ap.rib_pos, -1.0])
                        angle_rad_cone = np.deg2rad(self.hole_free_angle_s)
                        arc_span_rad = np.deg2rad(getattr(self, 'hole_arc_span_s', 120.0))
                        half_arc = arc_span_rad / 2.0
                        
                        if pilot_2d is not None:
                            line_dir_cone = v1 - pilot_2d
                            if np.linalg.norm(line_dir_cone) > 1e-9:
                                line_dir_cone = line_dir_cone / np.linalg.norm(line_dir_cone)
                            else:
                                line_dir_cone = np.array([0, 1])
                            angle_offset_cone = np.arctan2(line_dir_cone[1], line_dir_cone[0])
                        else:
                            angle_offset_cone = np.pi / 2
                        
                        extrados_poly = rib.profile_2d.get_extrados_poly()
                        far_factor = 10.0
                        inner_radius = halfmoon_radius_norm if halfmoon_radius_norm > 1e-6 else 0.005
                        
                        # Exclusion angle directions
                        dir2 = np.array([np.cos(angle_offset_cone - angle_rad_cone),
                                         np.sin(angle_offset_cone - angle_rad_cone)])
                        dir3 = np.array([np.cos(angle_offset_cone + angle_rad_cone),
                                         np.sin(angle_offset_cone + angle_rad_cone)])
                        
                        # Center axis
                        d_center = np.array([np.cos(angle_offset_cone), np.sin(angle_offset_cone)])
                        center_bottom = v1 + inner_radius * d_center
                        center_top = extrados_poly.line_intersection(
                            v1, v1 + d_center * far_factor)
                        if center_top is None:
                            continue
                        
                        # Zone edge boundaries (with arc span offset)
                        if halfmoon_radius_norm > 1e-6:
                            arc_angle_left = angle_offset_cone + half_arc
                            arc_angle_right = angle_offset_cone - half_arc
                            
                            left_edge_bottom = v1 + inner_radius * np.array(
                                [np.cos(arc_angle_left), np.sin(arc_angle_left)])
                            right_edge_bottom = v1 + inner_radius * np.array(
                                [np.cos(arc_angle_right), np.sin(arc_angle_right)])
                            
                            left_edge_top = extrados_poly.line_intersection(
                                left_edge_bottom, left_edge_bottom + dir3 * far_factor)
                            right_edge_top = extrados_poly.line_intersection(
                                right_edge_bottom, right_edge_bottom + dir2 * far_factor)
                        else:
                            left_edge_bottom = v1.copy()
                            right_edge_bottom = v1.copy()
                            left_edge_top = extrados_poly.line_intersection(
                                v1, v1 + dir3 * far_factor)
                            right_edge_top = extrados_poly.line_intersection(
                                v1, v1 + dir2 * far_factor)
                        
                        if left_edge_top is None or right_edge_top is None:
                            continue
                        
                        # Process each side
                        for side_data in [
                            (center_bottom, center_top, left_edge_bottom, left_edge_top),
                            (right_edge_bottom, right_edge_top, center_bottom, center_top),
                        ]:
                            s_bot_left, s_top_left, s_bot_right, s_top_right = side_data
                            
                            for zone_i in range(num_zones):
                                t0 = zone_i / num_zones
                                t1 = (zone_i + 1) / num_zones
                                
                                bl_bot = s_bot_left * (1 - t0) + s_bot_right * t0
                                bl_top = s_top_left * (1 - t0) + s_top_right * t0
                                br_bot = s_bot_left * (1 - t1) + s_bot_right * t1
                                br_top = s_top_left * (1 - t1) + s_top_right * t1
                                
                                d_left_line = bl_top - bl_bot
                                d_right_line = br_top - br_bot
                                len_left = np.linalg.norm(d_left_line)
                                len_right = np.linalg.norm(d_right_line)
                                
                                if len_left < 1e-9 or len_right < 1e-9:
                                    continue
                                
                                d_left_unit = d_left_line / len_left
                                d_right_unit = d_right_line / len_right
                                
                                perp_left = np.array([-d_left_unit[1], d_left_unit[0]])
                                perp_right = np.array([d_right_unit[1], -d_right_unit[0]])
                                
                                off_bl_bot = bl_bot + perp_left * margin_side
                                off_br_bot = br_bot + perp_right * margin_side
                                
                                # Bottom corners: intersect offset side lines with inner circle
                                R_arc = inner_radius + margin_bottom
                                single_bottom = False
                                
                                u_l = off_bl_bot - v1
                                dot_l = np.dot(u_l, d_left_unit)
                                disc_l = dot_l**2 - np.dot(u_l, u_l) + R_arc**2
                                
                                u_r = off_br_bot - v1
                                dot_r = np.dot(u_r, d_right_unit)
                                disc_r = dot_r**2 - np.dot(u_r, u_r) + R_arc**2
                                
                                if disc_l < 0 or disc_r < 0:
                                    single_bottom = True
                                else:
                                    p_bl = off_bl_bot + (-dot_l + np.sqrt(disc_l)) * d_left_unit
                                    p_br = off_br_bot + (-dot_r + np.sqrt(disc_r)) * d_right_unit
                                    ref_vec = br_bot - bl_bot
                                    if np.dot(p_br - p_bl, ref_vec) <= 0:
                                        single_bottom = True
                                
                                if single_bottom:
                                    dx = off_br_bot - off_bl_bot
                                    det_s = d_left_unit[0]*(-d_right_unit[1]) - d_left_unit[1]*(-d_right_unit[0])
                                    if abs(det_s) < 1e-12:
                                        continue
                                    t_cross = (dx[0]*(-d_right_unit[1]) - dx[1]*(-d_right_unit[0])) / det_s
                                    p_bottom = off_bl_bot + t_cross * d_left_unit
                                    p_bl = p_bottom
                                    p_br = p_bottom
                                
                                # Top corners
                                p_tl_ext = extrados_poly.line_intersection(
                                    off_bl_bot, off_bl_bot + d_left_unit * far_factor)
                                p_tr_ext = extrados_poly.line_intersection(
                                    off_br_bot, off_br_bot + d_right_unit * far_factor)
                                if p_tl_ext is None or p_tr_ext is None:
                                    continue
                                d_tl_r = p_tl_ext - v1
                                p_tl = p_tl_ext - (d_tl_r / np.linalg.norm(d_tl_r)) * margin_top
                                d_tr_r = p_tr_ext - v1
                                p_tr = p_tr_ext - (d_tr_r / np.linalg.norm(d_tr_r)) * margin_top
                                
                                ref_vec_t = br_bot - bl_bot
                                if np.dot(p_tr - p_tl, ref_vec_t) <= 0:
                                    continue
                                if np.dot(p_tl - p_bl, d_center) <= 0:
                                    continue
                                
                                num_curve_pts = 10
                                num_fillet_pts = 6
                                cr = cone_corner_pct
                                
                                angle_tr = np.arctan2(p_tr[1] - v1[1], p_tr[0] - v1[0])
                                angle_tl = np.arctan2(p_tl[1] - v1[1], p_tl[0] - v1[0])
                                ad_top = angle_tl - angle_tr
                                while ad_top > np.pi: ad_top -= 2 * np.pi
                                while ad_top < -np.pi: ad_top += 2 * np.pi
                                
                                if single_bottom:
                                    # === V-SHAPE ===
                                    side_r = np.linalg.norm(p_tr - p_bottom)
                                    side_l = np.linalg.norm(p_tl - p_bottom)
                                    if cr > 1e-6:
                                        dir_r = (p_tr - p_bottom) / max(side_r, 1e-9)
                                        dir_l = (p_tl - p_bottom) / max(side_l, 1e-9)
                                        cut_r = side_r * cr * 0.5
                                        cut_l = side_l * cr * 0.5
                                        bot_r = p_bottom + dir_r * cut_r
                                        bot_l = p_bottom + dir_l * cut_l
                                        tr_s = p_tr - dir_r * cut_r
                                        tl_s = p_tl - dir_l * cut_l
                                        st = np.sign(ad_top) if abs(ad_top) > 1e-9 else 1.0
                                        dist_tr = max(np.linalg.norm(p_tr - v1), 1e-9)
                                        dist_tl = max(np.linalg.norm(p_tl - v1), 1e-9)
                                        dtr = min(cut_r / dist_tr, abs(ad_top) * 0.45)
                                        dtl = min(cut_l / dist_tl, abs(ad_top) * 0.45)
                                        a_tr_f = angle_tr + st * dtr
                                        a_tl_f = angle_tl - st * dtl
                                        d_tr_f = np.array([np.cos(a_tr_f), np.sin(a_tr_f)])
                                        e_tr = extrados_poly.line_intersection(v1, v1 + d_tr_f * far_factor)
                                        tr_c = (e_tr - d_tr_f * margin_top) if e_tr is not None else p_tr
                                        d_tl_f = np.array([np.cos(a_tl_f), np.sin(a_tl_f)])
                                        e_tl = extrados_poly.line_intersection(v1, v1 + d_tl_f * far_factor)
                                        tl_c = (e_tl - d_tl_f * margin_top) if e_tl is not None else p_tl
                                        hole_pts = []
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * bot_l + 2*(1-t)*t * p_bottom + t**2 * bot_r)
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * tr_s + 2*(1-t)*t * p_tr + t**2 * tr_c)
                                        ad_t_s = a_tl_f - a_tr_f
                                        while ad_t_s > np.pi: ad_t_s -= 2*np.pi
                                        while ad_t_s < -np.pi: ad_t_s += 2*np.pi
                                        for ci in range(1, num_curve_pts):
                                            t = ci / num_curve_pts
                                            a = a_tr_f + t * ad_t_s
                                            d = np.array([np.cos(a), np.sin(a)])
                                            ext_pt = extrados_poly.line_intersection(v1, v1 + d * far_factor)
                                            if ext_pt is not None:
                                                hole_pts.append(ext_pt - d * margin_top)
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * tl_c + 2*(1-t)*t * p_tl + t**2 * tl_s)
                                        hole_pts.append(hole_pts[0])
                                    else:
                                        hole_pts = [p_bottom, p_tr]
                                        for ci in range(1, num_curve_pts):
                                            t = ci / num_curve_pts
                                            a = angle_tr + t * ad_top
                                            d = np.array([np.cos(a), np.sin(a)])
                                            ext_pt = extrados_poly.line_intersection(v1, v1 + d * far_factor)
                                            if ext_pt is not None:
                                                hole_pts.append(ext_pt - d * margin_top)
                                        hole_pts.append(p_tl)
                                        hole_pts.append(p_bottom)
                                else:
                                    # === NORMAL ARC ===
                                    side_right = np.linalg.norm(p_tr - p_br)
                                    side_left = np.linalg.norm(p_tl - p_bl)
                                    dir_left_up = (p_tl - p_bl) / max(side_left, 1e-9)
                                    dir_right_up = (p_tr - p_br) / max(side_right, 1e-9)
                                    cut_left = side_left * cr * 0.5
                                    cut_right = side_right * cr * 0.5
                                    angle_bl = np.arctan2(p_bl[1] - v1[1], p_bl[0] - v1[0])
                                    angle_br = np.arctan2(p_br[1] - v1[1], p_br[0] - v1[0])
                                    ad_bot = angle_br - angle_bl
                                    while ad_bot > np.pi: ad_bot -= 2 * np.pi
                                    while ad_bot < -np.pi: ad_bot += 2 * np.pi
                                    if cr > 1e-6:
                                        bl_s = p_bl + dir_left_up * cut_left
                                        br_s = p_br + dir_right_up * cut_right
                                        tr_s = p_tr - dir_right_up * cut_right
                                        tl_s = p_tl - dir_left_up * cut_left
                                        sb = np.sign(ad_bot) if abs(ad_bot) > 1e-9 else 1.0
                                        dbl = min(cut_left / R_arc, abs(ad_bot) * 0.45)
                                        dbr = min(cut_right / R_arc, abs(ad_bot) * 0.45)
                                        a_bl_f = angle_bl + sb * dbl
                                        a_br_f = angle_br - sb * dbr
                                        bl_a = v1 + R_arc * np.array([np.cos(a_bl_f), np.sin(a_bl_f)])
                                        br_a = v1 + R_arc * np.array([np.cos(a_br_f), np.sin(a_br_f)])
                                        st = np.sign(ad_top) if abs(ad_top) > 1e-9 else 1.0
                                        dist_tr = max(np.linalg.norm(p_tr - v1), 1e-9)
                                        dist_tl = max(np.linalg.norm(p_tl - v1), 1e-9)
                                        dtr = min(cut_right / dist_tr, abs(ad_top) * 0.45)
                                        dtl = min(cut_left / dist_tl, abs(ad_top) * 0.45)
                                        a_tr_f = angle_tr + st * dtr
                                        a_tl_f = angle_tl - st * dtl
                                        d_tr_f = np.array([np.cos(a_tr_f), np.sin(a_tr_f)])
                                        e_tr = extrados_poly.line_intersection(v1, v1 + d_tr_f * far_factor)
                                        tr_c = (e_tr - d_tr_f * margin_top) if e_tr is not None else p_tr
                                        d_tl_f = np.array([np.cos(a_tl_f), np.sin(a_tl_f)])
                                        e_tl = extrados_poly.line_intersection(v1, v1 + d_tl_f * far_factor)
                                        tl_c = (e_tl - d_tl_f * margin_top) if e_tl is not None else p_tl
                                        hole_pts = []
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * bl_s + 2*(1-t)*t * p_bl + t**2 * bl_a)
                                        ad_b_s = a_br_f - a_bl_f
                                        while ad_b_s > np.pi: ad_b_s -= 2*np.pi
                                        while ad_b_s < -np.pi: ad_b_s += 2*np.pi
                                        for ci in range(1, num_curve_pts):
                                            t = ci / num_curve_pts
                                            a = a_bl_f + t * ad_b_s
                                            hole_pts.append(v1 + R_arc * np.array([np.cos(a), np.sin(a)]))
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * br_a + 2*(1-t)*t * p_br + t**2 * br_s)
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * tr_s + 2*(1-t)*t * p_tr + t**2 * tr_c)
                                        ad_t_s = a_tl_f - a_tr_f
                                        while ad_t_s > np.pi: ad_t_s -= 2*np.pi
                                        while ad_t_s < -np.pi: ad_t_s += 2*np.pi
                                        for ci in range(1, num_curve_pts):
                                            t = ci / num_curve_pts
                                            a = a_tr_f + t * ad_t_s
                                            d = np.array([np.cos(a), np.sin(a)])
                                            ext_pt = extrados_poly.line_intersection(v1, v1 + d * far_factor)
                                            if ext_pt is not None:
                                                hole_pts.append(ext_pt - d * margin_top)
                                        for fi in range(num_fillet_pts):
                                            t = fi / (num_fillet_pts - 1)
                                            hole_pts.append((1-t)**2 * tl_c + 2*(1-t)*t * p_tl + t**2 * tl_s)
                                        hole_pts.append(hole_pts[0])
                                    else:
                                        hole_pts = [p_bl]
                                        for ci in range(1, num_curve_pts):
                                            t = ci / num_curve_pts
                                            a = angle_bl + t * ad_bot
                                            hole_pts.append(v1 + R_arc * np.array([np.cos(a), np.sin(a)]))
                                        hole_pts.append(p_br)
                                        hole_pts.append(p_tr)
                                        for ci in range(1, num_curve_pts):
                                            t = ci / num_curve_pts
                                            a = angle_tr + t * ad_top
                                            d = np.array([np.cos(a), np.sin(a)])
                                            ext_pt = extrados_poly.line_intersection(v1, v1 + d * far_factor)
                                            if ext_pt is not None:
                                                hole_pts.append(ext_pt - d * margin_top)
                                        hole_pts.append(p_tl)
                                        hole_pts.append(p_bl)
                                
                                custom_pts = [list(p) for p in hole_pts]
                                if not hasattr(rib, 'cone_holes'):
                                    rib.cone_holes = []
                                rib.cone_holes.append(
                                    RibHole(ap.rib_pos, custom_points=custom_pts)
                                )
                  except Exception:
                      pass

            else:
                shape_idx, num_holes, w_factor, h_factor, v_shift_factor, start_pos, end_pos, hole_height_mode, hole_margin, corner_radius = (
                    getattr(self, 'hole_shape_ns', 0), self.num_holes_ns, self.hole_width_ns,
                    self.hole_height_ns, self.vertical_shift_ns, self.min_hole_pos_ns, self.max_hole_pos_ns,
                    self.hole_height_mode_ns, self.hole_margin_ns, getattr(self, 'hole_corner_radius_ns', 0.005)
                )
                no_hole_zones = []
                halfmoon_circles = []

            hole_shape = 'ellipse' if shape_idx == 0 else 'rounded_rectangle'

            if num_holes == 0:
                continue

            allowed_ranges = [(start_pos, end_pos)]

            total_allowable_length = sum(end - start for start, end in allowed_ranges)
            if total_allowable_length <= 1e-6:
                continue

            holes_to_distribute = num_holes
            for i, (start, end) in enumerate(allowed_ranges):
                range_length = end - start
                if range_length <= 0: continue

                is_last_range = (i == len(allowed_ranges) - 1)
                if is_last_range:
                    num_holes_in_range = holes_to_distribute
                else:
                    num_holes_in_range = int(round(num_holes * (range_length / total_allowable_length)))

                if num_holes_in_range <= 0:
                    continue

                holes_to_distribute -= num_holes_in_range

                if num_holes_in_range == 1:
                    potential_positions = [start + range_length / 2] # Center the single hole
                else:
                    potential_positions = np.linspace(start, end, num_holes_in_range)

                for pos_x in potential_positions:
                    upper = rib.profile_2d.profilepoint(-pos_x)
                    lower = rib.profile_2d.profilepoint(pos_x)
                    local_thickness = upper[1] - lower[1]
                    if local_thickness < 1e-6:
                        continue

                    if is_suspended:
                        hole_center_x = (upper[0] + lower[0]) / 2.0
                        min_y_ceiling = upper[1]

                        # Check if this position is inside any halfmoon circle (same as preview)
                        inside_halfmoon = False
                        for hm_center, hm_radius in halfmoon_circles:
                            dist = np.sqrt((hole_center_x - hm_center[0])**2 + (lower[1] - hm_center[1])**2)
                            if dist < hm_radius:
                                inside_halfmoon = True
                                break
                        if inside_halfmoon:
                            continue

                        for zone in no_hole_zones:
                            # Get all x values from zone vertices
                            zone_x = [v[0] for v in zone]
                            if min(zone_x) <= hole_center_x <= max(zone_x):
                                # Iterate over edges of the polygon
                                n = len(zone)
                                for i in range(n):
                                    p1 = zone[i]
                                    p2 = zone[(i + 1) % n]
                                    if p1[0] != p2[0] and ((p1[0] <= hole_center_x <= p2[0]) or (p2[0] <= hole_center_x <= p1[0])):
                                        y_intersect = p1[1] + (p2[1] - p1[1]) * (hole_center_x - p1[0]) / (p2[0] - p1[0])
                                        if y_intersect < min_y_ceiling:
                                            min_y_ceiling = min(min_y_ceiling, y_intersect)

                        available_height = min_y_ceiling - lower[1]
                        hole_center_y = lower[1] + available_height / 2
                        new_lower_bound = np.array([hole_center_x, lower[1]]) # Base for vertical shift calculation logic
                        eff_upper_bound = np.array([hole_center_x, min_y_ceiling])
                    else:
                        available_height = upper[1] - lower[1]
                        new_lower_bound = lower
                        eff_upper_bound = upper

                    if available_height < 1e-4:
                        continue

                    # vertical shift is relative to LOCAL THICKNESS if we wanted to maintain consistent offset logic,
                    # but here we want to center in the AVAILABLE space usually.
                    # The original code used new_lower_bound + (available_height)/2 * (1+shift).
                    # Let's align with that.
                    
                    hole_center = np.array([hole_center_x if is_suspended else (upper[0]+lower[0])/2, lower[1] + available_height / 2])
                    
                    if not is_suspended: # Respect original shift logic for non-suspended
                         hole_center = lower + (upper - lower) / 2 * (1 + v_shift_factor)

                    # For suspended, we might want to respect vertical shift within the NEW available height?
                    if is_suspended:
                         hole_center[1] += available_height / 2 * v_shift_factor
                    
                    # Calculate final_vertical_shift relative to full local thickness
                    full_thickness = upper[1] - lower[1]
                    if full_thickness < 1e-6: continue
                    
                    original_center_y = (upper[1] + lower[1]) / 2.0
                    final_vertical_shift = (hole_center[1] - original_center_y) / full_thickness

                    width_param = (w_factor * rib.chord) / available_height if available_height > 1e-6 else 0
                    
                    # Calculate height parameter based on mode
                    # RibHole interprets size[1] as a factor of available_height (or local_thickness)
                    if hole_height_mode == 1: # Margin mode
                        target_height = available_height - 2 * hole_margin
                        if target_height <= 0:
                            continue
                        height_param = target_height / available_height
                    else: # Percent mode
                        height_param = h_factor

                    rib.holes.append(
                        RibHole(
                            pos_x,
                            size=np.array([width_param, height_param]),
                            vertical_shift=final_vertical_shift,
                            rotation=0.0,
                            shape=hole_shape,
                            available_height=available_height,
                            corner_radius=corner_radius
                        )
                    )

        # === DIAGONAL CONE HOLES ===
        # Store cone hole config on eligible (full) diagonals
        # Check new param first, fall back to old param for backward compatibility
        diag_enabled = getattr(self, 'diag_holes_enabled', None)
        if diag_enabled is None:
            diag_enabled = getattr(self, 'cone_holes_enabled_s', False)
        if diag_enabled:
            cone_config = {
                'num_zones': getattr(self, 'diag_hole_num_zones',
                             getattr(self, 'cone_hole_num_zones_s', 1)),
                'margin_top_m': getattr(self, 'diag_hole_margin_top',
                                getattr(self, 'cone_hole_margin_top_s', 3.0)) / 1000.0,
                'margin_side_m': getattr(self, 'diag_hole_margin_side',
                                 getattr(self, 'cone_hole_margin_side_s', 3.0)) / 1000.0,
                'margin_bottom_m': getattr(self, 'diag_hole_margin_bottom',
                                   getattr(self, 'cone_hole_margin_bottom_s', 3.0)) / 1000.0,
                'corner_radius_pct': getattr(self, 'diag_hole_corner_radius',
                                     getattr(self, 'cone_hole_corner_radius_s', 25.0)) / 100.0,
            }
            for cell in glider.cells:
                for drib in cell.diagonals:
                    left_h = (drib.left_front[1], drib.left_back[1])
                    right_h = (drib.right_front[1], drib.right_back[1])
                    
                    left_is_intrados = (left_h[0] == -1.0 and left_h[1] == -1.0)
                    right_is_intrados = (right_h[0] == -1.0 and right_h[1] == -1.0)
                    left_is_extrados = (left_h[0] > 0 and left_h[1] > 0)
                    right_is_extrados = (right_h[0] > 0 and right_h[1] > 0)
                    
                    is_full = (left_is_intrados and right_is_extrados) or \
                              (right_is_intrados and left_is_extrados)
                    
                    if is_full:
                        drib.cone_hole_config = cone_config
                    
                    # Detect horizontal bands (both sides extrados, same height)
                    is_band = (left_is_extrados and right_is_extrados and
                              abs(left_h[0] - right_h[0]) < 0.01 and
                              abs(left_h[1] - right_h[1]) < 0.01)
                    if is_band:
                        drib.band_hole_config = cone_config

    def apply_reinforcements(self, glider):
        """Apply reinforcement configurations to ribs for 2D export."""
        from openglider.glider.rib.elements import AttachmentReinforcement
        
        if not getattr(self, 'reinforcement_enabled_s', False):
            # Clear reinforcements from all ribs
            for rib in glider.ribs:
                rib.reinforcements = []
            return
        
        apply_all = getattr(self, 'reinforcement_apply_all_s', False)
        master_config = getattr(self, 'reinforcement_master_s', {})
        configs = getattr(self, 'reinforcement_configs_s', [])
        excluded_ribs = getattr(self, 'reinforcement_excluded_ribs_s', [])
        
        # Identify suspended ribs
        suspended_ribs = {att.rib for att in glider.lineset.attachment_points if hasattr(att, 'rib')}
        
        for rib_idx, rib in enumerate(glider.ribs):
            if rib in suspended_ribs:
                # Check if this rib is excluded from reinforcements
                if rib_idx in excluded_ribs:
                    rib.reinforcements = []
                    continue
                
                # Get attachment points for this rib
                attachment_points = glider.get_rib_attachment_points(rib)
                # Filter to valid attachment points (< 90% chord)
                valid_aps = [ap for ap in attachment_points if ap.rib_pos <= 0.90]
                valid_aps.sort(key=lambda x: x.rib_pos)
                
                reinforcements = []
                for i, ap in enumerate(valid_aps):
                    # Get config
                    if apply_all:
                        config = master_config
                    else:
                        config = configs[i] if i < len(configs) else master_config
                    
                    if config.get('enabled', True):
                        # Generate name: rib index + attachment point name
                        name = f"{rib_idx + 1}{ap.name}" if ap.name else f"{rib_idx + 1}_{i + 1}"
                        
                        reinforcement = AttachmentReinforcement(
                            position=ap.rib_pos,
                            surface_offset=config.get('surface_offset', 0.0005),  # 0.5mm
                            halfmoon_radius=config.get('halfmoon_radius', 0.1),   # 100mm
                            rod_enabled=config.get('rod_enabled', True),
                            rod_offset=config.get('rod_offset', 0.008),           # 8mm
                            rod_width=config.get('rod_width', 0.009),             # 9mm
                            rod_end_offset=config.get('rod_end_offset', 1.0),     # 1°
                            name=name,
                        )
                        reinforcements.append(reinforcement)

                
                rib.reinforcements = reinforcements
            else:
                # Non-suspended ribs don't get reinforcements
                rib.reinforcements = []

    def apply_rod_sleeves(self, glider):
        """Apply rod sleeve configurations to ribs for 2D export."""
        from openglider.glider.rib.elements import RodSleeve
        
        # Identify suspended ribs
        suspended_ribs = {att.rib for att in glider.lineset.attachment_points if hasattr(att, 'rib')}
        
        for rib_idx, rib in enumerate(glider.ribs):
            is_suspended = rib in suspended_ribs
            suffix = '_s' if is_suspended else '_ns'
            
            rod_sleeves = []
            
            # Get extrados sleeves
            extrados_enabled = getattr(self, f'extrados_sleeves_enabled{suffix}', True)
            extrados_configs = getattr(self, f'extrados_sleeves{suffix}', [])
            
            if extrados_enabled and extrados_configs:
                for i, config in enumerate(extrados_configs):
                    # Check if this rib is excluded for this config
                    excluded_ribs = config.get('excluded_ribs', [])
                    if rib_idx in excluded_ribs:
                        continue
                    
                    sleeve = RodSleeve(
                        surface='extrados',
                        width=config.get('width', 0.015),
                        offset=config.get('offset', 0.005),
                        start_chord=config.get('start_chord', 0.0),
                        end_chord=config.get('end_chord', 0.7),
                        le_curl=config.get('start_curl', 60.0),
                        te_curl=config.get('end_curl', 60.0),
                        le_length=config.get('start_length', 0.08),
                        te_length=config.get('end_length', 0.06),
                    )
                    rod_sleeves.append(sleeve)
            
            # Get intrados sleeves
            intrados_enabled = getattr(self, f'intrados_sleeves_enabled{suffix}', True)
            intrados_configs = getattr(self, f'intrados_sleeves{suffix}', [])
            
            if intrados_enabled and intrados_configs:
                for i, config in enumerate(intrados_configs):
                    # Check if this rib is excluded for this config
                    excluded_ribs = config.get('excluded_ribs', [])
                    if rib_idx in excluded_ribs:
                        continue
                    
                    sleeve = RodSleeve(
                        surface='intrados',
                        width=config.get('width', 0.015),
                        offset=config.get('offset', 0.005),
                        start_chord=config.get('start_chord', 0.06),
                        end_chord=config.get('end_chord', 0.5),
                        le_curl=config.get('start_curl', 60.0),
                        te_curl=config.get('end_curl', 60.0),
                        le_length=config.get('start_length', 0.08),
                        te_length=config.get('end_length', 0.06),
                    )
                    rod_sleeves.append(sleeve)
            
            rib.rod_sleeves = rod_sleeves

    def apply_ss_rib_warp(self, glider):
        """Align SingleSkinRib profiles with the suspension line pull direction.

        In a single-skin zone, ribs are free to warp laterally. The fabric
        aligns with the line pull axis (trame des suspentes). This method
        projects the 3D suspension lines onto the rib's local 2D plane and
        finds their intersection with the virtual intrados.
        
        This intersection defines the true anchor point on the rib (updated rib_pos).
        The tilt of the 3D line (spanwise shift / thickness depth) is recorded
        as the local shear angle at that chord position.
        
        If an attachment point is spanwise-constrained (by a diagonal or
        an adjacent intrados panel), its shear is fixed to 0.

        Must be called AFTER initial lineset.recalc().
        """
        from openglider.glider.rib.rib import SingleSkinRib
        from openglider.vector.polyline import PolyLine2D

        for rib in glider.ribs:
            if not isinstance(rib, SingleSkinRib):
                continue

            # Find adjacent cells (cells that have this rib as rib1 or rib2)
            adjacent_cells = [
                cell for cell in glider.cells
                if cell.rib1 is rib or cell.rib2 is rib
            ]

            # Collect all uppermost lines connected to this rib
            connected_lines = []
            for line in glider.lineset.uppermost_lines:
                if hasattr(line.upper_node, 'rib') and line.upper_node.rib is rib:
                    connected_lines.append(line)

            if not connected_lines:
                for line in glider.lineset.uppermost_lines:
                    if (hasattr(line.upper_node, 'rib') and
                            hasattr(line.upper_node.rib, 'name') and
                            line.upper_node.rib.name == rib.name):
                        connected_lines.append(line)

            if not connected_lines:
                continue

            # Check constraint function.
            # A rib is constrained at a given chord position only if there is
            # an INTRADOS PANEL at that position: intrados fabric physically
            # prevents the rib from leaning freely.
            # Diagonal ribs do NOT constrain the warp — they follow the new
            # shifted anchor point, since their 3D geometry is computed via
            # align_all() which already applies the shear map.
            def _is_constrained_at(rib_pos_val):
                abs_pos = abs(rib_pos_val)
                for cell in adjacent_cells:
                    is_rib1_side = cell.rib1 is rib
                    for panel in cell.panels:
                        if panel.is_lower():
                            if is_rib1_side:
                                p_min = min(panel.cut_front["left"], panel.cut_back["left"])
                                p_max = max(panel.cut_front["left"], panel.cut_back["left"])
                            else:
                                p_min = min(panel.cut_front["right"], panel.cut_back["right"])
                                p_max = max(panel.cut_front["right"], panel.cut_back["right"])
                            if p_min <= abs_pos <= p_max:
                                return True
                return False

            # Setup local inverse matrix for projection
            rot = rib.rotation_matrix
            chord_3d = np.array(rot([1, 0, 0]))
            thick_3d = np.array(rot([0, 1, 0]))
            span_3d  = np.array(rot([0, 0, 1]))
            matrix = np.column_stack((chord_3d, thick_3d, span_3d))
            try:
                inv_matrix = np.linalg.inv(matrix)
            except np.linalg.LinAlgError:
                continue

            intrados_poly = PolyLine2D(rib.profile_2d.data[rib.profile_2d.noseindex:])
            
            # Map of true anchor positions to local tan_theta (shear)
            shear_map = {}

            for line in connected_lines:
                if not hasattr(line.upper_node, 'rib_pos'):
                    continue
                
                # Check constraints at the extrados projection
                constrained = _is_constrained_at(line.upper_node.rib_pos)
                
                v_3d = line.lower_node.vec - line.upper_node.vec
                v_local = inv_matrix.dot(v_3d)
                
                # 2D local projection (chord, thickness)
                v_2d = np.array([v_local[0], v_local[1]])
                
                p_ext_2d = np.array(rib.profile_2d.profilepoint(line.upper_node.rib_pos))
                
                # Ray cast to intrados
                end_pt = p_ext_2d + 10 * v_2d
                intersect = intrados_poly.line_intersection(p_ext_2d + 0.001 * v_2d, end_pt)
                
                if intersect is not None:
                    # Update line's physical anchor point to intrados intersection!
                    true_rib_pos = intersect[0]  # Intrados is positive
                    line.upper_node.rib_pos = true_rib_pos
                    
                    if constrained:
                        shear_map[true_rib_pos] = 0.0
                    else:
                        # tan_theta = dz / dy
                        if abs(v_local[1]) > 1e-9:
                            tan_theta = v_local[2] / v_local[1]
                        else:
                            tan_theta = 0.0
                        shear_map[true_rib_pos] = tan_theta
                else:
                    # Fallback if no intersection
                    true_rib_pos = abs(line.upper_node.rib_pos)
                    line.upper_node.rib_pos = true_rib_pos
                    shear_map[true_rib_pos] = 0.0
                    
            rib.shear_map = shear_map
            rib.xrot = 0.0
            # Explicitly invalidate ALL cached properties on this rib.
            # shear_map is a runtime attribute not included in __json__, so
            # cached_property("self") would NOT detect the change.  Clearing the
            # _cache dict forces profile_3d (and anything else) to recompute
            # with the correct shear on the next access (e.g. lineset.recalc).
            if hasattr(rib, '_cache'):
                rib._cache.clear()

    def apply_single_skin(self, glider):
        """Apply single skin configuration to glider ribs.

        Logic:
        - All ribs adjacent to ANY selected SS cell are converted to SingleSkinRib
        - Intrados panels are removed from cells whose INDEX is in selected_cells
        - Transition ribs (adjacent to both SS and full cells) are kept full (no holes)
        """
        ss_config = getattr(self, 'single_skin_config', None)
        if not ss_config:
            return

        from openglider.glider.rib.rib import SingleSkinRib

        selected_cells = ss_config.get("cells", [])
        if not selected_cells:
            return

        cells_set = set(selected_cells)
        num_cells = self.shape.half_cell_num
        num_ribs = len(glider.ribs)

        # Determine which ribs to convert to SingleSkinRib:
        # Only ribs where ALL adjacent cells are SS get the bow-modified profile.
        # Transition ribs (touching both SS and full cells) stay as regular Rib.
        # Additionally, ALL ribs of boundary full cells (the full cell adjacent
        # to the SS zone) are sealed — no holes, no bows.
        rib_indices = set()           # ribs to convert to SingleSkinRib
        sealed_rib_indices = set()    # ribs that must be solid (no holes)
        boundary_full_cells = set()   # full cells adjacent to SS zone
        for rib_idx in range(num_ribs):
            adjacent_cells = []
            if rib_idx > 0:
                adjacent_cells.append(rib_idx - 1)
            if rib_idx < num_cells:
                adjacent_cells.append(rib_idx)
            ss_adjacent = [c for c in adjacent_cells if c in cells_set]
            non_ss_adjacent = [c for c in adjacent_cells if c not in cells_set]

            if ss_adjacent and not non_ss_adjacent:
                # All adjacent cells are SS → convert to SingleSkinRib
                rib_indices.add(rib_idx)
            elif ss_adjacent and non_ss_adjacent:
                # Transition rib: keep as regular Rib, sealed (no holes)
                sealed_rib_indices.add(rib_idx)
                # The full cells adjacent to this transition rib are boundary cells
                for c in non_ss_adjacent:
                    boundary_full_cells.add(c)

        # Seal BOTH ribs of each boundary full cell
        for cell_idx in boundary_full_cells:
            sealed_rib_indices.add(cell_idx)      # rib on left side of cell
            sealed_rib_indices.add(cell_idx + 1)  # rib on right side of cell

        single_skin_par = {
            "att_dist": ss_config.get("att_dist", 0.02),
            "height": ss_config.get("height", 0.5),
            "num_points": ss_config.get("num_points", 20),
            "le_gap": ss_config.get("le_gap", True),
            "te_gap": ss_config.get("te_gap", True),
            "double_first": ss_config.get("double_first", False),
            "straight_te": ss_config.get("straight_te", True),
            "continued_min": ss_config.get("continued_min", False),
            "continued_min_end": ss_config.get("continued_min_end", 0.9),
            "continued_min_angle": ss_config.get("continued_min_angle", 0.0),
            "continued_min_delta_y": ss_config.get("continued_min_delta_y", 0.0),
            "continued_min_x": ss_config.get("continued_min_x", 0.0),
        }

        # Replace ribs with SingleSkinRib (only fully-SS ribs, not transition)
        new_ribs = []
        for i, rib in enumerate(glider.ribs):
            if i in rib_indices:
                if not isinstance(rib, SingleSkinRib):
                    new_ribs.append(SingleSkinRib.from_rib(rib, single_skin_par))
                else:
                    rib.single_skin_par = single_skin_par
                    new_ribs.append(rib)
            else:
                new_ribs.append(rib)

        # Handle mirrored ribs
        for rib, ss_rib in zip(glider.ribs, new_ribs):
            if hasattr(rib, "mirrored_rib") and rib.mirrored_rib:
                nr = glider.ribs.index(rib.mirrored_rib)
                ss_rib.mirrored_rib = new_ribs[nr]

        glider.replace_ribs(new_ribs)

        # Clear holes from ALL SingleSkinRib (hole design overflow truncated profiles)
        # Also clear holes from sealed ribs (boundary full cell walls)
        for i, rib in enumerate(glider.ribs):
            if isinstance(rib, SingleSkinRib):
                rib.holes = []
            elif i in sealed_rib_indices:
                rib.holes = []

        # Store sealed rib indices on glider so apply_holes() can skip them
        glider._ss_sealed_rib_indices = sealed_rib_indices

        # Add SS-specific holes if configured
        if ss_config.get("holes", False):
            hole_size = np.array([
                ss_config.get("hole_width", 0.3),
                ss_config.get("hole_height", 0.7),
            ])
            min_pos = ss_config.get("min_hole_pos", 0.2)
            max_pos = ss_config.get("max_hole_pos", 1.0)
            v_shift = ss_config.get("vertical_shift", 0.2)

            for att_pnt in glider.lineset.attachment_points:
                if (isinstance(att_pnt.rib, SingleSkinRib)
                        and att_pnt.rib_pos > min_pos
                        and att_pnt.rib_pos < max_pos):
                    att_pnt.rib.holes.append(
                        RibHole(
                            att_pnt.rib_pos,
                            size=hole_size,
                            vertical_shift=v_shift,
                        )
                    )

        # (Hull profile generation is now deferred until AFTER apply_ss_rib_warp)

        # Remove intrados panels from single-skin cells (based on cell INDEX)
        double_first = ss_config.get("double_first", False)
        for cell_idx, cell in enumerate(glider.cells):
            if cell_idx in cells_set:
                if double_first:
                    extrados = [p for p in cell.panels if not p.is_lower()]
                    intrados = [p for p in cell.panels if p.is_lower()]
                    intrados.sort(key=lambda p: p.mean_x())
                    cell.panels = extrados + intrados[:1]
                else:
                    cell.panels = [p for p in cell.panels if not p.is_lower()]

    def remap_cell_indices(self, old_cell_num):
        """
        Remap all cell/rib indices proportionally after cell count change.
        Call this after changing shape.cell_num.
        """
        new_cell_num = self.shape.cell_num
        if old_cell_num == new_cell_num:
            return

        old_half = old_cell_num // 2 + (old_cell_num % 2)
        new_half = new_cell_num // 2 + (new_cell_num % 2)

        # number of ribs in the half-wing
        old_half_ribs = old_half + 1 - (old_cell_num % 2)
        new_half_ribs = new_half + 1 - (new_cell_num % 2)

        def remap_cell(idx, old_n, new_n):
            """Proportionally remap a cell index."""
            if old_n <= 0 or new_n <= 0:
                return 0
            return min(int(round(idx * new_n / old_n)), new_n - 1)

        def remap_cells_list(cells, old_n, new_n):
            """Remap a list of cell indices, removing duplicates."""
            remapped = []
            seen = set()
            for c in cells:
                new_c = remap_cell(c, old_n, new_n)
                if new_c not in seen:
                    remapped.append(new_c)
                    seen.add(new_c)
            return remapped

        count_lineset = 0
        count_elements = 0

        # 1. Remap lineset nodes (UpperNode2D.cell_no)
        from openglider.glider.parametric.lines import UpperNode2D
        for node in self.lineset.nodes:
            if isinstance(node, UpperNode2D):
                old_no = node.cell_no
                node.cell_no = remap_cell(old_no, old_half, new_half)
                if node.cell_no != old_no:
                    count_lineset += 1

        # 2. Remap elements with "cells" key
        cell_keys = [
            "cuts", "diagonals", "straps", "miniribs",
            "cell_rigidfoils", "le_panel_splits"
        ]
        for key in cell_keys:
            for item in self.elements.get(key, []):
                if "cells" in item:
                    old_cells = item["cells"]
                    item["cells"] = remap_cells_list(
                        old_cells, old_half, new_half
                    )
                    count_elements += 1

        # 3. Remap elements with "ribs" key (rib index = half_rib count)
        rib_keys = ["holes", "rigidfoils"]
        for key in rib_keys:
            for item in self.elements.get(key, []):
                if "ribs" in item:
                    old_ribs = item["ribs"]
                    item["ribs"] = remap_cells_list(
                        old_ribs, old_half_ribs, new_half_ribs
                    )
                    count_elements += 1

        # 4. Remap materials (list indexed by cell_no)
        if "materials" in self.elements:
            old_mats = self.elements["materials"]
            if isinstance(old_mats, list) and old_mats:
                new_mats = []
                for new_c in range(new_half):
                    old_c = remap_cell(new_c, new_half, old_half)
                    if old_c < len(old_mats):
                        new_mats.append(old_mats[old_c])
                    else:
                        new_mats.append(old_mats[-1] if old_mats else {})
                self.elements["materials"] = new_mats
                count_elements += 1

        logging.info(
            f"Remapped cell indices: {old_cell_num} -> {new_cell_num} cells "
            f"(half: {old_half} -> {new_half}, "
            f"{count_lineset} lineset nodes, {count_elements} element entries)"
        )

    def __json__(self):
        return {
            "shape": self.shape,
            "arc": self.arc,
            "aoa": self.aoa,
            "zrot": self.zrot,
            "profiles": self.profiles,
            "profile_merge_curve": self.profile_merge_curve,
            "balloonings": self.balloonings,
            "ballooning_merge_curve": self.ballooning_merge_curve,
            "lineset": self.lineset,
            "speed": self.speed,
            "glide": self.glide,
            "elements": self.elements,
            "hole_shape_ns": getattr(self, "hole_shape_ns", 0),
            "num_holes_ns": getattr(self, "num_holes_ns", 30),
            "hole_width_ns": getattr(self, "hole_width_ns", 0.003),
            "hole_height_ns": getattr(self, "hole_height_ns", 0.8),
            "vertical_shift_ns": getattr(self, "vertical_shift_ns", 0.0),
            "min_hole_pos_ns": getattr(self, "min_hole_pos_ns", 0.2),
            "max_hole_pos_ns": getattr(self, "max_hole_pos_ns", 0.8),
            "hole_height_mode_ns": getattr(self, "hole_height_mode_ns", 0),
            "hole_margin_ns": getattr(self, "hole_margin_ns", 0.02),
            "hole_corner_radius_ns": getattr(self, "hole_corner_radius_ns", 0.005),
            "hole_shape_s": getattr(self, "hole_shape_s", 0),
            "num_holes_s": getattr(self, "num_holes_s", 30),
            "hole_width_s": getattr(self, "hole_width_s", 0.003),
            "hole_height_s": getattr(self, "hole_height_s", 0.8),
            "vertical_shift_s": getattr(self, "vertical_shift_s", 0.0),
            "min_hole_pos_s": getattr(self, "min_hole_pos_s", 0.2),
            "max_hole_pos_s": getattr(self, "max_hole_pos_s", 0.8),
            "hole_height_mode_s": getattr(self, "hole_height_mode_s", 0),
            "hole_margin_s": getattr(self, "hole_margin_s", 0.02),
            "hole_corner_radius_s": getattr(self, "hole_corner_radius_s", 0.005),
            "hole_free_angle_s": getattr(self, "hole_free_angle_s", 30.0),
            "hole_arc_span_s": getattr(self, "hole_arc_span_s", 120.0),
            "cone_holes_enabled_s": getattr(self, "cone_holes_enabled_s", False),
            "cone_hole_num_zones_s": getattr(self, "cone_hole_num_zones_s", 1),
            "cone_hole_margin_top_s": getattr(self, "cone_hole_margin_top_s", 3.0),
            "cone_hole_margin_side_s": getattr(self, "cone_hole_margin_side_s", 3.0),
            "cone_hole_margin_bottom_s": getattr(self, "cone_hole_margin_bottom_s", 3.0),
            "cone_hole_corner_radius_s": getattr(self, "cone_hole_corner_radius_s", 25.0),
            # Airfoil Structure - Extrados Sleeve (suspended)
            "extrados_sleeve_enabled_s": getattr(self, "extrados_sleeve_enabled_s", False),
            "extrados_sleeve_width_s": getattr(self, "extrados_sleeve_width_s", 0.015),
            "extrados_sleeve_offset_s": getattr(self, "extrados_sleeve_offset_s", 0.003),
            "extrados_sleeve_start_s": getattr(self, "extrados_sleeve_start_s", 0.0),
            "extrados_sleeve_end_s": getattr(self, "extrados_sleeve_end_s", 0.9),
            "extrados_sleeve_le_angle_s": getattr(self, "extrados_sleeve_le_angle_s", 80.0),
            "extrados_sleeve_le_length_s": getattr(self, "extrados_sleeve_le_length_s", 0.03),
            "extrados_sleeve_te_angle_s": getattr(self, "extrados_sleeve_te_angle_s", 80.0),
            "extrados_sleeve_te_length_s": getattr(self, "extrados_sleeve_te_length_s", 0.03),
            # Airfoil Structure - Intrados Sleeve (suspended)
            "intrados_sleeve_enabled_s": getattr(self, "intrados_sleeve_enabled_s", False),
            "intrados_sleeve_width_s": getattr(self, "intrados_sleeve_width_s", 0.015),
            "intrados_sleeve_offset_s": getattr(self, "intrados_sleeve_offset_s", 0.003),
            "intrados_sleeve_start_s": getattr(self, "intrados_sleeve_start_s", 0.0),
            "intrados_sleeve_end_s": getattr(self, "intrados_sleeve_end_s", 0.9),
            "intrados_sleeve_le_angle_s": getattr(self, "intrados_sleeve_le_angle_s", 100.0),
            "intrados_sleeve_le_length_s": getattr(self, "intrados_sleeve_le_length_s", 0.03),
            "intrados_sleeve_te_angle_s": getattr(self, "intrados_sleeve_te_angle_s", 100.0),
            "intrados_sleeve_te_length_s": getattr(self, "intrados_sleeve_te_length_s", 0.03),
            # Airfoil Structure - Extrados Sleeve (non-suspended)
            "extrados_sleeve_enabled_ns": getattr(self, "extrados_sleeve_enabled_ns", False),
            "extrados_sleeve_width_ns": getattr(self, "extrados_sleeve_width_ns", 0.015),
            "extrados_sleeve_offset_ns": getattr(self, "extrados_sleeve_offset_ns", 0.003),
            "extrados_sleeve_start_ns": getattr(self, "extrados_sleeve_start_ns", 0.0),
            "extrados_sleeve_end_ns": getattr(self, "extrados_sleeve_end_ns", 0.9),
            "extrados_sleeve_le_angle_ns": getattr(self, "extrados_sleeve_le_angle_ns", 80.0),
            "extrados_sleeve_le_length_ns": getattr(self, "extrados_sleeve_le_length_ns", 0.03),
            "extrados_sleeve_te_angle_ns": getattr(self, "extrados_sleeve_te_angle_ns", 80.0),
            "extrados_sleeve_te_length_ns": getattr(self, "extrados_sleeve_te_length_ns", 0.03),
            # Airfoil Structure - Intrados Sleeve (non-suspended)
            "intrados_sleeve_enabled_ns": getattr(self, "intrados_sleeve_enabled_ns", False),
            "intrados_sleeve_width_ns": getattr(self, "intrados_sleeve_width_ns", 0.015),
            "intrados_sleeve_offset_ns": getattr(self, "intrados_sleeve_offset_ns", 0.003),
            "intrados_sleeve_start_ns": getattr(self, "intrados_sleeve_start_ns", 0.0),
            "intrados_sleeve_end_ns": getattr(self, "intrados_sleeve_end_ns", 0.9),
            "intrados_sleeve_le_angle_ns": getattr(self, "intrados_sleeve_le_angle_ns", 100.0),
            "intrados_sleeve_le_length_ns": getattr(self, "intrados_sleeve_le_length_ns", 0.03),
            "intrados_sleeve_te_angle_ns": getattr(self, "intrados_sleeve_te_length_ns", 100.0),
            "intrados_sleeve_te_length_ns": getattr(self, "intrados_sleeve_te_length_ns", 0.03),
            # Airfoil Structure - Reinforcements (suspended only)
            "reinforcement_enabled_s": getattr(self, "reinforcement_enabled_s", True),
            "reinforcement_apply_all_s": getattr(self, "reinforcement_apply_all_s", False),
            "reinforcement_master_s": getattr(self, "reinforcement_master_s", {
                'enabled': True, 'surface_offset': 0.0005, 'halfmoon_radius': 0.1,
                'rod_enabled': True, 'rod_offset': 0.008, 'rod_width': 0.009, 'rod_end_offset': 1.0
            }),
            "reinforcement_configs_s": getattr(self, "reinforcement_configs_s", []),
            "reinforcement_excluded_ribs_s": getattr(self, "reinforcement_excluded_ribs_s", []),
            # Multi-rod sleeves (NEW FORMAT)
            "extrados_sleeves_enabled_s": getattr(self, "extrados_sleeves_enabled_s", True),
            "extrados_sleeves_s": getattr(self, "extrados_sleeves_s", []),
            "intrados_sleeves_enabled_s": getattr(self, "intrados_sleeves_enabled_s", True),
            "intrados_sleeves_s": getattr(self, "intrados_sleeves_s", []),
            "extrados_sleeves_enabled_ns": getattr(self, "extrados_sleeves_enabled_ns", True),
            "extrados_sleeves_ns": getattr(self, "extrados_sleeves_ns", []),
            "intrados_sleeves_enabled_ns": getattr(self, "intrados_sleeves_enabled_ns", True),
            "intrados_sleeves_ns": getattr(self, "intrados_sleeves_ns", []),
            # Edit Cells - Diagonals Auto-fill configuration
            "diagonal_autofill_params": getattr(self, "diagonal_autofill_params", {
                "A": (4.0, 5.0, 15.0, 1),
                "B": (4.0, 15.0, 30.0, 1),
                "C": (4.0, 30.0, 50.0, 1),
                "D": (4.0, 50.0, 75.0, 1),
            }),
            "diagonal_autofill_offset": getattr(self, "diagonal_autofill_offset", 0),
            # Lines Auto-Placement configuration
            "lines_placement_config": getattr(self, "lines_placement_config", {
                "demi_ecartement": 0.2,
                "profondeur": 0.5,
                "hauteur_cone": 7.0,
                "riser_length": 0.47,
                "basses_auto": True,
                "basses_length": 2.0,
                "inter_auto": True,
                "inter_length": 1.0,
                "include_stabilo": True,
                "stabilo_position": 50.0,
                "line_type_lowers": "default",
                "line_type_mid": "default",
                "line_type_uppers": "default",
                "fork_angle_auto": False,
                "fork_angle_max": 15.0,
                "line_types": {
                    "A": {"enabled": True, "position": 8.5, "interval": 1, "start_cell": 0, "patterns": "3:1, 2:2:1"},
                    "B": {"enabled": True, "position": 27.5, "interval": 2, "start_cell": 0, "patterns": "2:2:1, 2:1"},
                    "C": {"enabled": True, "position": 53.0, "interval": 2, "start_cell": 0, "patterns": "2:1"},
                    "D": {"enabled": True, "position": 77.0, "interval": 3, "start_cell": 0, "patterns": "2:1"},
                    "F": {"enabled": True, "position": 100.0, "interval": 1, "start_cell": 0, "patterns": "1:1"},
                }
            }),
            # =====================================================================
            # Profile Control - Unified airfoil management
            # =====================================================================
            "thickness_curve": getattr(self, "thickness_curve", None),
            "thickness_curve_enabled": getattr(self, "thickness_curve_enabled", False),
            "sharknose_enabled": getattr(self, "sharknose_enabled", False),
            "sharknose_start": getattr(self, "sharknose_start", 0.03),
            "sharknose_end": getattr(self, "sharknose_end", 0.08),
            "sharknose_angle": getattr(self, "sharknose_angle", 5.0),
            "sharknose_curve": getattr(self, "sharknose_curve", None),
            "sharknose_cells": getattr(self, "sharknose_cells", None),
            "profile_overrides": getattr(self, "profile_overrides", {}),
            "profile_overrides_enabled": getattr(self, "profile_overrides_enabled", False),
            "last_profile_enabled": getattr(self, "last_profile_enabled", False),
            "last_profile_type": getattr(self, "last_profile_type", "line"),
            "last_profile_thickness": getattr(self, "last_profile_thickness", 0.3),
            "last_profile_custom": getattr(self, "last_profile_custom", None),
            # Trailing-edge truncation (sand-drain opening)
            "te_cut_enabled": getattr(self, "te_cut_enabled", False),
            "te_cut_mm": getattr(self, "te_cut_mm", 15.0),
            # Aerodynamic analysis results
            "aerodynamics_results": getattr(self, "aerodynamics_results", None),
            # Single Skin configuration
            "single_skin_config": getattr(self, "single_skin_config", None),
        }


    def __setstate__(self, state):
        """
        Restore state and ensure new attributes are initialized with defaults.
        This fixes persistence issues when loading old objects that lack new fields.
        """
        self.__dict__.update(state)
        # Initialize default values for missing attributes (e.g. from schema updates)
        # Using the same defaults as in __init__
        self.hole_free_angle_s = getattr(self, 'hole_free_angle_s', 30.0)
        self.single_skin_config = getattr(self, 'single_skin_config', None)


    @classmethod
    def import_ods(cls, path):
        return import_ods_2d(cls, path)

    export_ods = export_ods_2d

    def copy(self):
        return copy.deepcopy(self)

    def get_geomentry_table(self):
        table = Table()
        table.insert_row(
            [
                "",
                "Ribs",
                "Chord",
                "X",
                "Y",
                "%",
                "Arc",
                "Arc_diff",
                "AOA",
                "Z-rotation",
                "Y-rotation",
                "profile-merge",
                "ballooning-merge",
            ]
        )
        shape = self.shape.get_half_shape()
        for rib_no in range(self.shape.half_rib_num):
            table[1 + rib_no, 1] = rib_no + 1

        for rib_no, chord in enumerate(shape.chords):
            table[1 + rib_no, 2] = chord

        for rib_no, p in enumerate(self.shape.baseline):
            table[1 + rib_no, 3] = p[0]
            table[1 + rib_no, 4] = p[1]
            table[1 + rib_no, 5] = self.shape.baseline_pos

        last_angle = 0
        for cell_no, angle in enumerate(self.get_arc_angles()):
            angle = angle * 180 / math.pi
            table[1 + cell_no, 6] = angle
            table[1 + cell_no, 7] = angle - last_angle
            last_angle = angle

        for rib_no, aoa in enumerate(self.get_aoa()):
            table[1 + rib_no, 8] = aoa * 180 / math.pi
            table[1 + rib_no, 9] = 0
            table[1 + rib_no, 10] = 0

        return table

    @property
    def arc_positions(self):
        return self.arc.get_arc_positions(self.shape.rib_x_values)

    def get_arc_angles(self, arc_curve=None):
        """
        Get rib rotations
        :param arc_curve:
        :return: rotation angles
        """
        # arc_curve = ArcCurve(self.arc)
        arc_curve = self.arc

        return arc_curve.get_rib_angles(self.shape.rib_x_values)

    @property
    def attachment_points(self):
        """coordinates of the attachment_points"""
        return [
            a_p.get_2D(self.shape)
            for a_p in self.lineset.nodes
            if isinstance(a_p, UpperNode2D)
        ]

    def merge_ballooning(self, factor):
        factor = max(0, min(len(self.balloonings) - 1, factor))
        k = factor % 1
        i = int(factor // 1)
        first = self.balloonings[i]
        if k > 0:
            second = self.balloonings[i + 1]
            return first * (1 - k) + second * k
        else:
            return first.copy()

    def get_merge_profile(self, factor, pos_x=None, rib_index=None):
        """
        Get merged profile with optional profile control modifications.
        
        Args:
            factor: Interpolation factor from profile_merge_curve
            pos_x: Position along span (for thickness/sharknose curves)
            rib_index: Rib index (for profile overrides)
        
        Returns:
            Profile2D with all modifications applied
        """
        # 1. Check for rib-specific override
        if (getattr(self, 'profile_overrides_enabled', False) and 
            rib_index is not None and 
            str(rib_index) in getattr(self, 'profile_overrides', {})):
            override_idx = self.profile_overrides[str(rib_index)]
            if 0 <= override_idx < len(self.profiles):
                profile = self.profiles[override_idx].copy()
            else:
                profile = self._interpolate_profiles(factor)
        else:
            # 2. Standard interpolation
            profile = self._interpolate_profiles(factor)
        
        # 3. Apply thickness scaling
        if getattr(self, 'thickness_curve_enabled', False) and pos_x is not None:
            thickness_factor = self._get_thickness_factor(pos_x)
            if thickness_factor != 1.0:
                profile = self._apply_thickness_scaling(profile, thickness_factor)
        
        # 4. Apply shark nose if enabled AND rib belongs to a selected cell
        if getattr(self, 'sharknose_enabled', False):
            sharknose_cells = getattr(self, 'sharknose_cells', None)
            # Check if rib_index corresponds to a cell in sharknose_cells
            # A rib at index i borders cells i-1 and i (for i > 0)
            # We apply sharknose if either adjacent cell is selected
            apply_sharknose = True
            if sharknose_cells is not None and rib_index is not None:
                # Check if either adjacent cell is in the selected list
                cell_indices = []
                if rib_index > 0:
                    cell_indices.append(rib_index - 1)
                if rib_index < len(self.shape.rib_x_values) - 1:
                    cell_indices.append(rib_index)
                apply_sharknose = any(c in sharknose_cells for c in cell_indices)
            
            if apply_sharknose:
                sharknose_factor = self._get_sharknose_factor(pos_x)
                if sharknose_factor > 0:
                    profile = self._apply_sharknose(profile, sharknose_factor)
        
        # 5. Override last rib profile if enabled (stabilo/wingtip)
        last_enabled = getattr(self, 'last_profile_enabled', False)
        if rib_index is not None:
            last_rib_index = len(self.shape.rib_x_values) - 1
            is_last = rib_index == last_rib_index
            if is_last:
                pass
            if last_enabled and is_last:
                profile = self._get_last_profile(profile)
        
        return Profile2D(profile.data)
    
    def _interpolate_profiles(self, factor):
        """Standard profile interpolation between adjacent profiles."""
        factor = max(0, min(len(self.profiles) - 1, factor))
        k = factor % 1
        i = int(factor // 1)
        first = self.profiles[i].copy()
        if k > 0:
            second = self.profiles[i + 1]
            return first * (1 - k) + second * k
        return first
    
    def _get_thickness_factor(self, pos_x):
        """Get thickness scaling factor at given span position."""
        if not hasattr(self, 'thickness_curve') or self.thickness_curve is None:
            return 1.0
        try:
            interp = self.thickness_curve.interpolation(num=self.num_interpolate)
            return interp(abs(pos_x))
        except Exception:
            return 1.0
    
    def _apply_thickness_scaling(self, profile, factor):
        """Scale profile thickness by given factor."""
        if factor == 1.0:
            return profile
        new_profile = profile.copy()
        # Scale Y values relative to camber line
        data = np.array(new_profile.data)
        camber_line = dict(profile.camber_line)
        for i, (x, y) in enumerate(data):
            camber_y = camber_line.get(abs(x), 0)
            delta = y - camber_y
            data[i, 1] = camber_y + delta * factor
        new_profile.data = data
        return new_profile
    
    def _get_sharknose_factor(self, pos_x):
        """Get shark nose blending factor (0-1) at given span position."""
        if not getattr(self, 'sharknose_enabled', False):
            return 0.0
        if hasattr(self, 'sharknose_curve') and self.sharknose_curve is not None:
            try:
                interp = self.sharknose_curve.interpolation(num=self.num_interpolate)
                return max(0.0, min(1.0, interp(abs(pos_x))))
            except Exception:
                return 1.0
        return 1.0
    
    def _apply_sharknose(self, profile, factor=1.0):
        """Apply shark nose shelf + angled drop to profile intrados.
        
        Geometry (5 zones):
        A: x < start - r           → original (unchanged, near nose)
        B: start-r <= x <= start+r → smooth entry corner (original → shelf)
        C: start+r < x < end-xtilt → flat shelf at y_start
        D: end-xtilt <= x <= end   → tilted drop (straight line, sharp top corner)
        E: end < x <= end+r        → smooth exit corner (drop → original)
        F: x > end + r             → original (unchanged)
        
        Rounding at corners B (original→shelf) and E (drop→original).
        No rounding at shelf→drop junction (sharp angle).
        
        Args:
            profile: Profile2D to modify
            factor: Blending factor 0-1 (for spanwise variation)
        """
        if factor <= 0:
            return profile
        
        start_x = getattr(self, 'sharknose_start', 0.03)
        end_x = getattr(self, 'sharknose_end', 0.08)
        angle_deg = getattr(self, 'sharknose_angle', 5.0)
        
        if start_x >= end_x:
            return profile
        
        nose_idx = profile.noseindex
        
        # Extract intrados portion (after noseindex)
        intrados_x = profile.data[nose_idx:, 0]
        intrados_y = profile.data[nose_idx:, 1]
        
        # Interpolate original intrados y at start and end positions
        y_at_start = np.interp(start_x, intrados_x, intrados_y)  # less negative
        y_at_end = np.interp(end_x, intrados_x, intrados_y)      # more negative
        
        # Shelf level
        shelf_y = y_at_start
        
        # Drop geometry: height and horizontal tilt
        drop_height = abs(shelf_y - y_at_end)
        angle_rad = np.radians(angle_deg)
        x_tilt = drop_height * np.tan(angle_rad)  # horizontal shift due to tilt
        
        # Ensure x_tilt doesn't exceed available shelf space
        shelf_length = end_x - start_x
        x_tilt = min(x_tilt, shelf_length * 0.5)
        
        # Rounding radius (auto-computed, generous for visible effect)
        radius = min(0.008, shelf_length * 0.25)
        
        # Drop line: from (end_x - x_tilt, shelf_y) to (end_x, y_at_end)
        drop_top_x = end_x - x_tilt
        
        new_data = np.array(profile.data, dtype=float).copy()
        
        for i in range(nose_idx, len(new_data)):
            px = new_data[i, 0]
            orig_y = profile.data[i, 1]
            
            if px > end_x + radius:
                break  # Past the affected zone (Zone F)
            
            if px < start_x - radius:
                continue  # Before the affected zone (Zone A)
            
            # Zone B: Entry corner (original intrados → shelf)
            if px <= start_x + radius:
                t = (px - (start_x - radius)) / (2.0 * radius)
                t = max(0.0, min(1.0, t))
                blend = 0.5 * (1.0 - np.cos(np.pi * t))
                target_y = orig_y + (shelf_y - orig_y) * blend
            
            # Zone C: Flat shelf
            elif px < drop_top_x:
                target_y = shelf_y
            
            # Zone D: Tilted drop (straight line, no rounding)
            elif px <= end_x:
                if x_tilt > 0:
                    t = (px - drop_top_x) / (end_x - drop_top_x)
                else:
                    t = 1.0
                t = max(0.0, min(1.0, t))
                target_y = shelf_y + (y_at_end - shelf_y) * t
            
            # Zone E: Exit corner (drop → original intrados)
            else:
                t = (px - end_x) / radius
                t = max(0.0, min(1.0, t))
                blend = 0.5 * (1.0 - np.cos(np.pi * t))
                target_y = y_at_end + (orig_y - y_at_end) * blend
            
            # Apply spanwise blending factor
            new_data[i, 1] = orig_y + (target_y - orig_y) * factor
        
        new_profile = profile.copy()
        new_profile.data = new_data
        return new_profile
    
    def _get_last_profile(self, base_profile):
        """Get the profile to use for the last rib (stabilo/wingtip).
        
        Options:
        - 'line': Zero thickness (flat line)
        - 'thin': Scaled down version of base profile
        - 'custom': User-imported profile
        """
        last_profile_type = getattr(self, 'last_profile_type', 'line')
        
        if last_profile_type == 'line':
            return self._create_line_profile(base_profile)
        elif last_profile_type == 'thin':
            relative_thickness = getattr(self, 'last_profile_thickness', 0.3)  # 30% of original
            target = base_profile.thickness * relative_thickness
            result = self._create_thin_profile(base_profile, target)
            return result
        elif last_profile_type == 'custom':
            custom = getattr(self, 'last_profile_custom', None)
            if custom is not None:
                custom_copy = custom.copy()
                custom_copy.x_values = base_profile.x_values
                return custom_copy
        
        # Fallback to line
        return self._create_line_profile(base_profile)
    
    def _create_line_profile(self, base_profile):
        """Create a flat (zero thickness) profile based on given profile's x values."""
        data = [[x, 0.0] for x, _ in base_profile.data]
        return Profile2D(data, name="line_profile")
    
    def _create_thin_profile(self, base_profile, target_thickness):
        """Create a thin version of the profile with given thickness."""
        current_thickness = base_profile.thickness
        if current_thickness <= 0:
            return base_profile.copy()
        # Scale Y values to achieve target thickness
        factor = target_thickness / current_thickness
        new_data = [[x, y * factor] for x, y in base_profile.data]
        return Profile2D(new_data, name="thin_profile")

    @property
    def is_asymmetric(self):
        """True when any panel-cut or panel-colour data differs between the
        left and right wing.  When False, get_glider_3d()/copy_complete()
        behave exactly as before (a pure spanwise mirror of the half-glider)."""
        for cut in self.elements.get("cuts", []):
            if cut.get("side") in ("left", "right"):
                return True
        if self.elements.get("materials_left_by_name"):
            return True
        if self.elements.get("materials_right_by_name"):
            return True
        return False

    def _cuts_for_cell(self, cell_no, side):
        """Cut dicts that apply to one half-cell for a given wing.

        side:
          None    -> symmetric base: only untagged ("both") cuts
          "right" -> untagged cuts + cuts tagged "right"
          "left"  -> untagged cuts + cuts tagged "left"

        Returned dicts are copies with the "cells"/"side" keys removed, ready
        to feed _build_cell_panels().
        """
        out = []
        for cut in self.elements.get("cuts", []):
            if cell_no not in cut.get("cells", []):
                continue
            cut_side = cut.get("side")
            if cut_side in (None, "both") or (side is not None and cut_side == side):
                c = cut.copy()
                c.pop("cells", None)
                c.pop("side", None)
                out.append(c)
        return out

    def _materials_by_name_for_side(self, side):
        """Per-name colour lookup for a wing: base palette overlaid with the
        side-specific overrides used for asymmetric decoupe."""
        materials = dict(self.elements.get("materials_by_name", {}))
        if side == "left":
            materials.update(self.elements.get("materials_left_by_name", {}))
        else:  # "right" or None -> base, plus any right-only overrides
            materials.update(self.elements.get("materials_right_by_name", {}))
        return materials

    def _build_cell_panels(self, cell_no, cuts, materials_by_name, cell_materials):
        """Build the Panel/PolygonPanel objects for a single half-cell from an
        already-resolved list of cut dicts (each without the "cells" key).

        Factored out of get_panels() so the left and right wings can be built
        from different cut sets / colour maps for asymmetric decoupe while
        sharing one strip/crossing decomposition.
        """
        panel_lst = []
        cuts = [cut.copy() for cut in cuts]

        # add trailing edge (2x)
        # The implicit trailing-edge seam is only skipped when a cut already
        # lies *fully* on the trailing edge (left == right == ±1).  A cut
        # that merely touches ±1 at one end (e.g. a design curve snapped
        # onto the trailing edge at one rib) still needs the trailing edge
        # as the other boundary, otherwise the panel between them vanishes.
        if not any(c["left"] == -1 and c["right"] == -1 for c in cuts):
            cuts.append({"type": "parallel", "left": -1, "right": -1})
        if not any(c["left"] == 1 and c["right"] == 1 for c in cuts):
            cuts.append({"type": "parallel", "left": 1, "right": 1})

        # Crossing cuts: the strip decomposition below (ZipCmp of sorted
        # cuts) cannot tile a cell where two cuts cross at an interior point.
        # Detect it and build region PolygonPanels instead. No crossing -> the
        # ordinary strip path runs unchanged (R6). See
        # docs/crossing-cuts-design.md.
        from openglider.glider.cell.cut_arrangement import (
            Cut,
            build_regions,
            find_crossings,
        )

        _cut_objs = [
            Cut(c["left"], c["right"], c.get("type", "orthogonal"), i)
            for i, c in enumerate(cuts)
        ]
        _crossings = find_crossings(_cut_objs)
        if _crossings:
            from openglider.glider.cell.polygon_panel import PolygonPanel

            regions = build_regions(cuts)  # sorted deterministically by centroid
            # every region samples through the cell crossings so shared cut
            # edges coincide (watertight mesh + matching flattened seams)
            crossings = list(_crossings)
            part_no = 0
            for region in regions:
                if region.is_entry():
                    continue
                name = f"c{cell_no + 1}p{part_no + 1}"
                # Region colouring: prefer an explicit per-name colour (set by
                # a region-aware colour tool); otherwise inherit the colour of
                # whatever panel sat at this region's chord position, so the
                # user's existing palette shows through instead of grey.
                material_code = materials_by_name.get(name)
                if material_code is None and cell_materials:
                    chord_centroid = region.centroid()[1]  # in [-1, +1]
                    idx = int((chord_centroid + 1.0) / 2.0 * len(cell_materials))
                    idx = max(0, min(idx, len(cell_materials) - 1))
                    material_code = cell_materials[idx]
                if material_code is None:
                    material_code = "unknown"
                panel_lst.append(
                    PolygonPanel(
                        region,
                        material_code=material_code,
                        name=name,
                        crossings=crossings,
                    )
                )
                part_no += 1
            return panel_lst

        # Sort cuts by their average chord position (left+right)/2
        # This handles polylines that fold back and might have
        # cut1.left < cut2.left but cut1.right > cut2.right
        cuts.sort(key=lambda cut: (cut["left"] + cut["right"]) / 2.0)

        for cut1, cut2 in ZipCmp(cuts):
            part_no = len(panel_lst)

            if cut1["right"] > cut2["right"] and cut1["left"] > cut2["left"]:
                error_str = "Invalid cut: C{} {:.02f}/{:.02f}/{} + {:.02f}/{:.02f}/{}".format(
                    cell_no + 1,
                    cut1["left"],
                    cut1["right"],
                    cut1["type"],
                    cut2["left"],
                    cut2["right"],
                    cut2["type"],
                )
                raise ValueError(error_str)

            # If cuts cross (left smaller but right larger), swap them
            # so the panel can be rendered without errors, or at least try
            # to not crash the entire application
            if cut1["right"] > cut2["right"]:
                # We don't raise ValueError anymore for intersecting lines
                # as complex polylines might cross each other intentionally
                pass

            if (
                cut1["type"] == cut2["type"] == "folded"
                or cut1["type"] == cut2["type"] == "singleskin"
            ):
                # entry
                continue

            name = f"c{cell_no + 1}p{part_no + 1}"
            try:
                material_code = cell_materials[part_no]
            except (KeyError, IndexError):
                material_code = "unknown"
            # An explicit per-name colour (from the colour tool, incl. per-side
            # overrides) wins over the positional palette.
            material_code = materials_by_name.get(name, material_code)

            panel = Panel(
                cut1,
                cut2,
                name=name,
                material_code=material_code,
            )

            # Check if this is an LE panel that should be split chordwise
            le_splits = self.elements.get("le_panel_splits", [])
            should_split = False

            for le_split in le_splits:
                if cell_no in le_split.get("cells", []):
                    cut_limit = le_split.get("cut_limit", 0.1)
                    # The LE panel is the one where:
                    # - cut_front (cut1) is at -cut_limit (our cut_3d on extrados)
                    # - cut_back (cut2) is at the LE entry (close to 0 or slightly positive)
                    front_left = cut1.get("left", 0)
                    back_left = cut2.get("left", 0)

                    # Only match extrados (front_left negative) panels
                    # Match if:
                    # 1. front is near -cut_limit
                    # 2. back is at LE entry (close to 0 or positive, i.e. > -0.02)
                    is_extrados = front_left < 0
                    front_matches = abs(abs(front_left) - cut_limit) < 0.02
                    back_at_le_entry = back_left > -0.02  # At or past the leading edge

                    if is_extrados and front_matches and back_at_le_entry:
                        should_split = True
                        break

            if should_split:
                # Split into 2 panels at y=0.5 using y_start/y_end
                # Lookup colors by panel name if available
                name_L = f"c{cell_no + 1}p{part_no + 1}_L"
                name_R = f"c{cell_no + 1}p{part_no + 1}_R"

                # Panel L: from rib1 (y=0) to mid (y=0.5)
                panel_left = Panel(
                    cut1,
                    cut2,
                    name=name_L,
                    material_code=materials_by_name.get(name_L, material_code),
                    y_start=0.0,
                    y_end=0.5,
                )

                # Panel R: from mid (y=0.5) to rib2 (y=1)
                panel_right = Panel(
                    cut1,
                    cut2,
                    name=name_R,
                    material_code=materials_by_name.get(name_R, material_code),
                    y_start=0.5,
                    y_end=1.0,
                )

                panel_lst.append(panel_left)
                panel_lst.append(panel_right)
            else:
                panel_lst.append(panel)

        return panel_lst

    def get_panels(self, glider_3d=None, side=None):
        """
        Create Panels Objects and apply on gliders cells if provided, otherwise create a list of panels
        :param glider_3d: (optional)
        :param side: None (symmetric base), "left" or "right" -- selects the
            side-specific cuts/colours used for asymmetric decoupe.
        :return: list of "cells"
        """
        if glider_3d is None:
            cells = [[] for _ in range(self.shape.half_cell_num)]
        else:
            cells = [cell.panels for cell in glider_3d.cells]
            for cell in cells:
                cell.clear()

        materials_by_name = self._materials_by_name_for_side(side)

        for cell_no, panel_lst in enumerate(cells):
            cuts = self._cuts_for_cell(cell_no, side)
            try:
                cell_materials = list(self.elements["materials"][cell_no])
            except (KeyError, IndexError, TypeError):
                cell_materials = []
            panel_lst.extend(
                self._build_cell_panels(
                    cell_no, cuts, materials_by_name, cell_materials
                )
            )

        return cells

    def _get_cell_straps(self, name, _cls):
        elements = []
        for cell_no in range(self.shape.half_cell_num):
            cell_elements = []
            for strap in self.elements.get(name, []):
                if cell_no in strap["cells"]:
                    dct = strap.copy()
                    dct.pop("cells")
                    cell_elements.append(_cls(**dct))

            cell_elements.sort(key=lambda strap: strap.get_average_x())

            for strap_no, strap in enumerate(cell_elements):
                strap.name = f"c{cell_no + 1}{name[0]}{strap_no}"

            elements.append(cell_elements)

        return elements

    def get_cell_diagonals(self):
        return self._get_cell_straps("diagonals", DiagonalRib)

    def get_cell_straps(self):
        return self._get_cell_straps("straps", TensionStrap)

    def get_cell_tension_lines(self):
        return self._get_cell_straps("tension_lines", TensionLine)

    def apply_diagonals(self, glider):
        cell_straps = self.get_cell_straps()
        cell_diagonals = self.get_cell_diagonals()
        cell_tensionlines = self.get_cell_tension_lines()

        for cell_no, cell in enumerate(glider.cells):
            cell.diagonals = cell_diagonals[cell_no]
            cell.straps = cell_straps[cell_no]
            cell.straps += cell_tensionlines[cell_no]

    @classmethod
    def fit_glider_3d(cls, glider, numpoints=3):
        return fit_glider_3d(cls, glider, numpoints)

    def get_front_line(self):
        """
        Get Nose Positions for cells
        :return:
        """

    def get_aoa(self, interpolation_num=None):
        aoa_interpolation = self.aoa.interpolation(
            num=interpolation_num or self.num_interpolate
        )

        return [aoa_interpolation(x) for x in self.shape.rib_x_values]

    def apply_aoa(self, glider, interpolation_num=50):
        aoa_interpolation = self.aoa.interpolation(num=interpolation_num)
        aoa_values = [aoa_interpolation(x) for x in self.shape.rib_x_values]

        if self.shape.has_center_cell:
            aoa_values.insert(0, aoa_values[0])

        for rib, aoa in zip(glider.ribs, aoa_values):
            rib.aoa_relative = aoa

    def get_profile_merge(self):
        profile_merge_curve = self.profile_merge_curve.interpolation(
            num=self.num_interpolate
        )
        return [profile_merge_curve(abs(x)) for x in self.shape.rib_x_values]

    def get_ballooning_merge(self):
        ballooning_merge_curve = self.ballooning_merge_curve.interpolation(
            num=self.num_interpolate
        )
        return [ballooning_merge_curve(abs(x) for x in self.shape.cell_x_values)]

    def apply_shape_and_arc(self, glider):
        x_values = self.shape.rib_x_values
        shape_ribs = self.shape.ribs
        arc_pos = list(self.arc.get_arc_positions(x_values))
        offset_x = shape_ribs[0][0][1]

        line = []
        chords = []

        for rib_no, x in enumerate(x_values):
            front, back = shape_ribs[rib_no]
            arc = arc_pos[rib_no]
            startpoint = np.array([-front[1] + offset_x, arc[0], arc[1]])

            line.append(startpoint)
            chords.append(abs(front[1] - back[1]))

        if self.shape.has_center_cell:
            line.insert(0, line[0] * [1, -1, 1])
            chords.insert(0, chords[0])

        for rib_no, p in enumerate(line):
            glider.ribs[rib_no].pos = p
            glider.ribs[rib_no].chord = chords[rib_no]

    def get_glider_3d(self, glider=None, num=50, num_profile=None):
        """returns a new glider from parametric values"""
        glider = glider or Glider()
        ribs = []

        self.rescale_curves()

        x_values = self.shape.rib_x_values
        shape_ribs = self.shape.ribs

        profile_merge_curve = self.profile_merge_curve.interpolation(num=num)
        ballooning_merge_curve = self.ballooning_merge_curve.interpolation(num=num)
        aoa_int = self.aoa.interpolation(num=num)
        zrot_int = self.zrot.interpolation(num=num)

        arc_pos = list(self.arc.get_arc_positions(x_values))
        rib_angles = self.arc.get_rib_angles(x_values)

        if self.num_profile is not None:
            num_profile = self.num_profile

        if num_profile is not None:
            profile_x_values = Distribution.from_cos_distribution(num_profile)
        else:
            profile_x_values = self.profiles[0].x_values

        rib_holes = self.elements.get("holes", [])
        rigids = self.elements.get("rigidfoils", [])

        cell_centers = [(p1 + p2) / 2 for p1, p2 in zip(x_values[:-1], x_values[1:])]
        offset_x = shape_ribs[0][0][1]

        rib_material = None
        if "rib_material" in self.elements:
            rib_material = self.elements["rib_material"]

        # Track previous chord for stabilo handling
        prev_chord = None
        last_rib_index = len(x_values) - 1
        
        
        for rib_no, pos in enumerate(x_values):
            front, back = shape_ribs[rib_no]
            arc = arc_pos[rib_no]
            startpoint = np.array([-front[1] + offset_x, arc[0], arc[1]])

            chord = abs(front[1] - back[1])
            original_chord = chord
            
            # Debug for last few ribs
            if rib_no >= last_rib_index - 2:
                pass
            
            # For last rib with last_profile_enabled: use previous rib's chord if current is 0
            # This ensures the thin/custom profile is visible instead of collapsed to a line
            if rib_no == last_rib_index and getattr(self, 'last_profile_enabled', False):
                if chord < 0.01 and prev_chord is not None:  # Chord is essentially zero
                    chord = prev_chord * getattr(self, 'last_profile_thickness', 0.3)
                elif chord >= 0.01:
                    pass
                else:
                    pass
            
            factor = profile_merge_curve(abs(pos))
            profile = self.get_merge_profile(factor, pos_x=pos, rib_index=rib_no)
            profile.name = f"Profile{rib_no}"
            profile.x_values = profile_x_values
            
            # Debug profile thickness for last rib
            if rib_no == last_rib_index:
                pass
            
            prev_chord = chord if chord > 0.01 else prev_chord

            this_rib_holes = []
            this_rigid_foils = [
                RigidFoil(rigid["start"], rigid["end"], rigid["distance"])
                for rigid in rigids
                if rib_no in rigid["ribs"]
            ]

            ribs.append(
                Rib(
                    profile_2d=profile,
                    startpoint=startpoint,
                    chord=chord,
                    arcang=rib_angles[rib_no],
                    glide=self.glide,
                    aoa_absolute=aoa_int(pos),
                    zrot=zrot_int(pos),
                    holes=this_rib_holes,
                    rigidfoils=this_rigid_foils,
                    name=f"rib{rib_no}",
                    material_code=rib_material,
                )
            )
            ribs[-1].aoa_relative = aoa_int(pos)

            # Trailing-edge truncation length (model units = metres). Stored on the
            # rib; the actual cut is applied lazily in Rib.get_hull / Rib.profile_3d
            # (see Rib._get_truncated_profile) so it survives any later resampling
            # of profile_2d and propagates to both the 3d hull and 2d templates.
            if getattr(self, "te_cut_enabled", False) and chord > 0:
                ribs[-1].trailing_edge_cut = getattr(self, "te_cut_mm", 15.0) / 1000.0
            else:
                ribs[-1].trailing_edge_cut = 0.0

        if self.shape.has_center_cell:
            new_rib = ribs[0].copy()
            new_rib.name = "rib0"
            new_rib.mirror()
            new_rib.mirrored_rib = ribs[0]
            ribs.insert(0, new_rib)
            cell_centers.insert(0, 0.0)

        glider.cells = []
        for cell_no, (rib1, rib2) in enumerate(zip(ribs[:-1], ribs[1:])):
            ballooning_factor = ballooning_merge_curve(cell_centers[cell_no])
            ballooning = self.merge_ballooning(ballooning_factor)

            cell = Cell(rib1, rib2, ballooning, name=f"c{cell_no + 1}")

            glider.cells.append(cell)

        # Only close the last rib (collapse to line) if NOT using custom last profile
        if not getattr(self, 'last_profile_enabled', False):
            glider.close_rib()
        else:
            pass

        # CELL-ELEMENTS
        # Panels are stored on the half-glider (which becomes the *right* wing
        # after copy_complete()).  For asymmetric decoupe we also pre-build the
        # *left* wing's panels here and stash them on the glider; copy_complete()
        # swaps them onto the mirrored half.  Symmetric gliders are unaffected.
        asymmetric = self.is_asymmetric
        self.get_panels(glider, side="right" if asymmetric else None)
        if asymmetric:
            left_materials = self._materials_by_name_for_side("left")
            left_panels = {}
            for cell_no in range(self.shape.half_cell_num):
                cuts = self._cuts_for_cell(cell_no, "left")
                try:
                    cell_materials = list(self.elements["materials"][cell_no])
                except (KeyError, IndexError, TypeError):
                    cell_materials = []
                left_panels[cell_no] = self._build_cell_panels(
                    cell_no, cuts, left_materials, cell_materials
                )
            glider._asym_left_panels = left_panels
        self.apply_diagonals(glider)

        for minirib in self.elements.get("miniribs", []):
            data = minirib.copy()
            cells = data.pop("cells")
            count = data.pop("count", 1)
            base_y = data.get("yvalue", 0.5)
            
            # Generate y_values for multiple mini ribs
            if count > 1:
                # Distribute evenly across the cell span
                # For count=3: y_values = [0.25, 0.5, 0.75]
                y_values = [(i + 1) / (count + 1) for i in range(count)]
            else:
                y_values = [base_y]
            
            # Apply global minirib hole settings if enabled
            if getattr(self, 'minirib_holes', False):
                data['num_holes'] = getattr(self, 'minirib_num_holes', 1)
                data['hole_width'] = getattr(self, 'minirib_hole_width', 0.5)
                data['hole_height'] = getattr(self, 'minirib_hole_height', 0.7)
                data['hole_shape'] = getattr(self, 'minirib_hole_shape', 0)
                data['hole_corner_radius'] = getattr(self, 'minirib_hole_corner_radius', 0.25)
                data['hole_max_pos'] = getattr(self, 'minirib_hole_max_pos', 0.9)
            else:
                data['num_holes'] = 0  # Disable holes
            
            for cell_no in cells:
                for idx, y_val in enumerate(y_values):
                    mr_data = data.copy()
                    mr_data["yvalue"] = y_val
                    base_name = data.get('name', 'minirib')
                    if count > 1:
                        mr_data["name"] = f"{base_name}_{idx + 1}"
                    glider.cells[cell_no].miniribs.append(MiniRib(**mr_data))

        for rigidfoil in self.elements.get("cell_rigidfoils", []):
            data = rigidfoil.copy()
            for cell_no in data.pop("cells"):
                glider.cells[cell_no].rigidfoils.append(PanelRigidFoil(**data))

        # LE Panel Splits - create LeadingEdgeClosure for spanwise split
        for le_split in self.elements.get("le_panel_splits", []):
            cut_limit = le_split.get("cut_limit", 0.1)
            material = le_split.get("material_code", "")
            cells = le_split.get("cells", [])
            
            for cell_no in cells:
                if 0 <= cell_no < len(glider.cells):
                    # Initialize le_closures list if needed
                    if not hasattr(glider.cells[cell_no], 'le_closures'):
                        glider.cells[cell_no].le_closures = []
                    
                    closure = LeadingEdgeClosure(
                        cut_back_x=cut_limit,
                        y_position=0.5,  # Center split
                        material_code=material,
                        name=f"le_split_c{cell_no+1}"
                    )
                    glider.cells[cell_no].le_closures.append(closure)

        # RIB-ELEMENTS

        glider.rename_parts()

        glider.lineset = self.lineset.return_lineset(glider, self.v_inf)
        glider.lineset.glider = glider

        # ---- SINGLESKIN: PHASE 1 ----
        # Remove intrados panels and convert to SingleSkinRib.
        self.apply_single_skin(glider)

        # Force all SingleSkin APs to extrados temporarily for ray casting.
        # Save _orig_rib_pos BEFORE modification so tools (e.g. auto-fill diagonals)
        # can match projections against the original parametric chord position.
        from openglider.glider.rib.rib import SingleSkinRib
        for att in glider.lineset.attachment_points:
            if hasattr(att, 'rib') and isinstance(att.rib, SingleSkinRib):
                att._orig_rib_pos = abs(att.rib_pos)   # parametric chord pos (e.g. 0.30)
                att.rib_pos = -abs(att.rib_pos)

        glider.lineset.calculate_sag = False
        
        # Build lines mesh and let it settle (trame des suspentes)
        for i in range(3):
            glider.lineset.recalc()

        # ---- SINGLESKIN: PHASE 2 ----
        # Cast rays from extrados along lines to intersect virtual intrados.
        # This updates att.rib_pos to the true intrados points and computes shear map.
        self.apply_ss_rib_warp(glider)

        # Build SingleSkin hull profiles (parabola cutouts) using true intrados APs.
        # We call get_hull() to cache the result in rib._hull_profile.
        # We do NOT overwrite profile_2d — that keeps profile_3d based on the clean
        # aero profile, which is essential for correct get_flattened_cell() results.
        for rib in glider.ribs:
            if isinstance(rib, SingleSkinRib):
                rib.get_hull(glider)

        # apply holes, reinforcements, etc.
        self.apply_holes(glider)
        self.apply_reinforcements(glider)
        self.apply_rod_sleeves(glider)

        # Final line recalc (now attaching to physical intrados points on sheared ribs)
        glider.lineset.recalc()

        return glider

    def apply_ballooning(self, glider3d):
        for ballooning in self.balloonings:
            ballooning.apply_splines()
        cell_centers = self.shape.cell_x_values
        ballooning_merge_curve = self.ballooning_merge_curve.interpolation(
            num=self.num_interpolate
        )
        for cell_no, cell in enumerate(glider3d.cells):
            ballooning_factor = ballooning_merge_curve(cell_centers[cell_no])
            ballooning = self.merge_ballooning(ballooning_factor)
            cell.ballooning = ballooning

        return glider3d

    @property
    def v_inf(self):
        angle = np.arctan(1 / self.glide)
        return np.array([np.cos(angle), 0, np.sin(angle)]) * self.speed

    def set_area(self, area):
        factor = math.sqrt(area / self.shape.area)
        self.shape.scale(factor)
        self.lineset.scale(factor, scale_lower_floor=False)
        self.rescale_curves()

    def set_aspect_ratio(self, aspect_ratio, remain_area=True):
        ar0 = self.shape.aspect_ratio
        area0 = self.shape.area

        self.shape.scale(y=ar0 / aspect_ratio)

        for p in self.lineset.get_lower_attachment_points():
            p.pos_2D[1] *= ar0 / aspect_ratio

        if remain_area:
            self.set_area(area0)

        return self.shape.aspect_ratio

    ##############################################################
    # is this used?
    def scale(self, x=1, y=1):
        self.shape.scale(x, y)
        if x != 1:
            self.rescale_curves()

    ##############################################################

    def rescale_curves(self):
        span = self.shape.span

        def rescale(curve):
            span_orig = curve.controlpoints[-1][0]
            factor = span / span_orig
            curve._data[:, 0] *= factor

        rescale(self.ballooning_merge_curve)
        rescale(self.profile_merge_curve)
        rescale(self.aoa)
        rescale(self.zrot)
        self.arc.rescale(self.shape.rib_x_values)

    def get_line_bbox(self):
        points = []
        for point in self.lineset.nodes:
            points.append(point.get_2D(self.shape))

        return [
            [min([p[0] for p in points]), min([p[1] for p in points])],
            [max([p[0] for p in points]), max([p[1] for p in points])],
        ]

    def export_lines2D_as_svg(self, file_name=None):
        # sollte im lineset2D sein, aber des lineset hat keine moeglichkeit auf diese Klasse
        # zuzugreifen...
        border = 0.1
        bbox = self.get_line_bbox()
        width = bbox[1][0] - bbox[0][0]
        height = bbox[1][1] - bbox[0][1]

        import svgwrite
        import svgwrite.container

        drawing = svgwrite.Drawing(size=[800, 800 * height / width])

        drawing.viewbox(
            bbox[0][0] - border * width,
            -bbox[1][1] - border * height,
            width * (1 + 2 * border),
            height * (1 + 2 * border),
        )
        lines = svgwrite.container.Group()
        lines.scale(1, -1)
        for line in self.lineset.lines:
            p1 = line.lower_node.get_2D(self.shape)
            p2 = line.upper_node.get_2D(self.shape)
            drawing_line = drawing.polyline(
                [p1, p2],
                style="stroke:black; vector-effect: fill: none; stroke-width:0.01px",
            )
            lines.add(drawing_line)
        drawing.add(lines)

        ribs = svgwrite.container.Group()

        ribs.scale(1, -1)
        p1_old, p2_old = None, None
        for rib in self.shape.ribs:
            p1 = rib[0]
            p2 = rib[1]
            ribs.add(
                drawing.polyline(
                    [p1, p2],
                    style="stroke:black; vector-effect: fill: none; stroke-width:0.01px",
                )
            )
            if p1_old and p2_old:
                ribs.add(
                    drawing.polyline(
                        [p1_old, p1],
                        style="stroke:black; vector-effect: fill: none; stroke-width:0.01px",
                    )
                )
                ribs.add(
                    drawing.polyline(
                        [p2_old, p2],
                        style="stroke:black; vector-effect: fill: none; stroke-width:0.01px",
                    )
                )
            p1_old, p2_old = p1, p2

        drawing.add(ribs)
        if file_name:
            drawing.saveas(file_name)
        return drawing.tostring()
