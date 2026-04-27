from __future__ import division

from pivy import coin

from PySide import QtGui

from .glider import draw_glider, draw_lines
from .tools import BaseTool, input_field
from .table import base_table_widget
from .pull_axis_utils import compute_pull_axis_projection


def refresh():
    pass


class CellTool(BaseTool):
    hide = True
    turn = False

    # Default parameters for auto-generation
    DIAGONAL_HALF_WIDTH = 0.02  # 2cm half-width (4cm total)
    DIAGONAL_EXTRADOS_OFFSET = 0.0  # No offset from top of extrados
    VECTOR_STRAP_WIDTH = 0.04  # 4cm width
    
    # Persistent storage for auto-fill dialog values (class-level = shared across instances)
    _last_diagonal_params = None  # Will store {"A": (intrados, start, end, bands), ...}
    _last_diagonal_offset = 0

    def __init__(self, obj):
        super(CellTool, self).__init__(obj)
        self.diagonals_table = diagonals_table()
        self.diagonals_table.get_from_ParametricGlider(self.parametric_glider)
        self.diagonals_button = QtGui.QPushButton("diagonals")
        self.diagonals_button.clicked.connect(self.diagonals_table.show)
        self.layout.setWidget(0, input_field, self.diagonals_button)

        self.vector_table = vector_table()
        self.vector_table.get_from_ParametricGlider(self.parametric_glider)
        self.vector_button = QtGui.QPushButton("vector strap")
        self.vector_button.clicked.connect(self.vector_table.show)
        self.layout.setWidget(1, input_field, self.vector_button)

        self.update_button = QtGui.QPushButton("update glider")
        self.update_button.clicked.connect(self.update_glider)
        self.layout.setWidget(2, input_field, self.update_button)

        # Auto-fill buttons
        self.auto_diagonals_button = QtGui.QPushButton("Auto-fill Diagonals")
        self.auto_diagonals_button.clicked.connect(self.auto_fill_diagonals)
        self.layout.setWidget(3, input_field, self.auto_diagonals_button)

        self.auto_straps_button = QtGui.QPushButton("Auto-fill Vector Straps")
        self.auto_straps_button.clicked.connect(self.auto_fill_vector_straps)
        self.layout.setWidget(4, input_field, self.auto_straps_button)
        
        self.custom_strap_button = QtGui.QPushButton("Add Custom Strap")
        self.custom_strap_button.clicked.connect(self.add_custom_vector_strap)
        self.layout.setWidget(5, input_field, self.custom_strap_button)

        self.draw_glider()

    def draw_glider(self):
        _rot = coin.SbRotation()
        _rot.setValue(coin.SbVec3f(0, 1, 0), coin.SbVec3f(1, 0, 0))
        rot = coin.SoRotation()
        rot.rotation.setValue(_rot)
        self.task_separator += rot
        draw_glider(
            self.parametric_glider.get_glider_3d(),
            self.task_separator,
            hull=None,
            ribs=True,
            fill_ribs=False,
        )
        draw_lines(
            self.parametric_glider.get_glider_3d(),
            vis_lines=self.task_separator,
            line_num=1,
        )

    def update_glider(self):
        self.task_separator.removeAllChildren()
        self.apply_elements()
        self.draw_glider()

    def apply_elements(self):
        self.diagonals_table.apply_to_glider(self.parametric_glider)
        self.vector_table.apply_to_glider(self.parametric_glider)

    def accept(self):
        super(CellTool, self).accept()
        self.diagonals_table.hide()
        self.vector_table.hide()
        del self.diagonals_table
        del self.vector_table
        self.update_view_glider()

    def reject(self):
        super(CellTool, self).reject()
        self.diagonals_table.hide()
        self.vector_table.hide()
        del self.diagonals_table
        del self.vector_table

    def _get_suspended_ribs(self, exclude_brake=True):
        """
        Get set of rib indices that have suspension attachment points.
        Returns dict mapping rib_no -> list of attachment point positions.
        """
        lineset = self.parametric_glider.lineset
        upper_nodes = lineset.get_upper_nodes()
        
        # Map rib_no to attachment positions
        rib_attachments = {}
        for node in upper_nodes:
            if exclude_brake:
                name = (node.name or "").upper()
                layer_attr = (node.layer or "").upper()
                if any(kw in name for kw in ("BRAKE", "STABILO", "FREIN")):
                    continue
                if layer_attr in ("BRAKE", "STABILO", "S", "FREIN", "F"):
                    continue
            
            rib_no = node.cell_no + int(node.cell_pos) + self.parametric_glider.shape.has_center_cell
            
            if rib_no not in rib_attachments:
                rib_attachments[rib_no] = []
            rib_attachments[rib_no].append(node.rib_pos)
        
        return rib_attachments

    def _get_cell_count(self):
        """Get total number of cells (half-span)."""
        return self.parametric_glider.shape.half_cell_num

    def _get_suspended_ribs_with_layer(self):
        """
        Get attachment points with their line layer (A, B, C, D, etc.).
        Returns dict mapping rib_no -> list of (rib_pos, layer) tuples.
        
        Layer is determined by position on chord:
        1. All unique rib_pos values are grouped with tolerance
        2. Only position groups appearing on ≥40% of ribs are kept (filters stabilo/brake)
        3. Remaining groups are sorted front-to-back and assigned A, B, C, D labels
        """
        lineset = self.parametric_glider.lineset
        upper_nodes = lineset.get_upper_nodes()
        
        layer_letters = ["A", "B", "C", "D", "E", "F", "G", "H"]
        
        # First pass: collect all attachment points with their rib_no
        raw_attachments = {}  # rib_no -> list of rib_pos
        for node in upper_nodes:
            # Skip obvious brake/stabilo from name or layer attributes
            name = (node.name or "").upper()
            layer_attr = (node.layer or "").upper()
            if any(kw in name for kw in ("BRAKE", "STABILO", "FREIN")):
                continue
            if layer_attr in ("BRAKE", "STABILO", "S", "FREIN", "F"):
                continue
            
            rib_no = node.cell_no + int(node.cell_pos) + self.parametric_glider.shape.has_center_cell
            
            if rib_no not in raw_attachments:
                raw_attachments[rib_no] = []
            raw_attachments[rib_no].append(node.rib_pos)
        
        if not raw_attachments:
            return {}
        
        total_ribs = len(raw_attachments)
        
        # Second pass: group positions with tolerance, tracking which ribs each group appears on
        tolerance = 0.03
        all_positions_with_rib = []  # (pos, rib_no) pairs
        for rib_no, positions in raw_attachments.items():
            for pos in positions:
                all_positions_with_rib.append((pos, rib_no))
        
        all_positions_with_rib.sort(key=lambda x: x[0])
        
        # Group: (center_pos, set of rib_nos)
        position_groups_raw = []
        for pos, rib_no in all_positions_with_rib:
            merged = False
            for i, (center, ribs_set) in enumerate(position_groups_raw):
                if abs(pos - center) < tolerance:
                    ribs_set.add(rib_no)
                    merged = True
                    break
            if not merged:
                position_groups_raw.append((pos, {rib_no}))

        
        # Filter: keep only groups appearing on >= 40% of ribs (main line families)
        min_ribs = max(2, int(total_ribs * 0.4))
        main_groups = [(c, r) for c, r in position_groups_raw if len(r) >= min_ribs]
        main_groups.sort(key=lambda x: x[0])  # Sort front-to-back
        
        # Assign A, B, C, D labels to main groups
        position_groups = []  # (center, label)
        for i, (center, ribs_set) in enumerate(main_groups):
            label = layer_letters[i] if i < len(layer_letters) else f"L{i}"
            position_groups.append((center, label))
        

        
        # Third pass: assign layer to each point based on closest MAIN group
        # Skip points that don't match any main group (tolerance * 3)
        max_dist = tolerance * 5
        rib_attachments = {}
        for rib_no, positions in raw_attachments.items():
            rib_attachments[rib_no] = []
            for rib_pos in positions:
                best_layer = None
                best_dist = float('inf')
                for center, label in position_groups:
                    dist = abs(rib_pos - center)
                    if dist < best_dist:
                        best_dist = dist
                        best_layer = label
                # Only include if close enough to a main group
                if best_layer and best_dist < max_dist:
                    rib_attachments[rib_no].append((rib_pos, best_layer))
        
        return rib_attachments


    def auto_fill_diagonals(self):
        """
        Auto-generate diagonal ribs from attachment points.
        Shows configuration dialog with per-line-type parameters.
        Supports two modes: Percentage (% avant/arrière axe) and Angle (opening angle from pull axis).
        """
        import math
        
        # Configuration dialog
        dialog = QtGui.QDialog()
        dialog.setWindowTitle("Diagonal Auto-fill Configuration")
        dialog.setMinimumWidth(600)
        layout = QtGui.QVBoxLayout(dialog)
        
        # Mode selection
        mode_layout = QtGui.QHBoxLayout()
        mode_layout.addWidget(QtGui.QLabel("Mode:"))
        mode_combo = QtGui.QComboBox()
        mode_combo.addItems(["Percentage", "Angle"])
        saved_mode = getattr(self.parametric_glider, 'diagonal_autofill_mode', 'percent')
        mode_combo.setCurrentIndex(0 if saved_mode == 'percent' else 1)
        mode_layout.addWidget(mode_combo)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)
        
        # Help text (updates with mode)
        help_label = QtGui.QLabel()
        help_label.setWordWrap(True)
        help_label.setStyleSheet("color: #555; font-size: 11px; padding: 4px; background: #f8f8f8; border-radius: 3px;")
        layout.addWidget(help_label)
        
        HELP_PERCENT = (
            "<b>Percentage mode:</b> For each attachment point, the pull axis is projected "
            "onto the extrados. The diagonal extends a fixed <i>% of chord</i> forward and backward "
            "from this projection point. Adjust <i>% Fwd</i> and <i>% Aft</i> per line layer."
        )
        HELP_ANGLE = (
            "<b>Angle mode:</b> For each attachment point, the pull axis is used as bisector. "
            "Two rays are traced at ±<i>angle/2</i> from the pull axis, starting from the base edges "
            "(intrados), until they meet the extrados of the adjacent cell. This produces diagonals "
            "whose opening adapts naturally to the local profile geometry."
        )
        
        # Header
        header = QtGui.QLabel("Parameters per line layer")
        header.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(header)
        
        # Create table for line parameters - 5 columns, show/hide based on mode
        line_types = ["A", "B", "C", "D"]
        param_table = QtGui.QTableWidget(len(line_types), 5)
        param_table.setHorizontalHeaderLabels([
            "Intrados width (cm)",    # col 0 - always visible
            "% Fwd of axis",          # col 1 - percent mode only
            "% Aft of axis",          # col 2 - percent mode only
            "Angle (°)",              # col 3 - angle mode only
            "Num. bands"              # col 4 - always visible
        ])
        param_table.setVerticalHeaderLabels(line_types)
        param_table.horizontalHeader().setStretchLastSection(True)
        
        # Default values - read from ParametricGlider for persistence
        saved_params = getattr(self.parametric_glider, 'diagonal_autofill_params', None)
        saved_angles = getattr(self.parametric_glider, 'diagonal_autofill_angles', None)
        if saved_params:
            defaults = dict(saved_params)
        else:
            defaults = {
                "A": (4.0, 5.0, 5.0, 1),
                "B": (4.0, 7.0, 8.0, 1),
                "C": (4.0, 8.0, 12.0, 1),
                "D": (4.0, 10.0, 15.0, 1),
            }
        if saved_angles:
            angle_defaults = dict(saved_angles)
        else:
            angle_defaults = {"A": 45.0, "B": 45.0, "C": 45.0, "D": 45.0}
        
        line_spinboxes = {}
        angle_spinboxes = {}
        for row, line_type in enumerate(line_types):
            intrados_spin = QtGui.QDoubleSpinBox()
            intrados_spin.setRange(1, 20)
            intrados_spin.setValue(defaults[line_type][0])
            intrados_spin.setSuffix(" cm")
            
            before_axis_spin = QtGui.QDoubleSpinBox()
            before_axis_spin.setRange(0, 50)
            before_axis_spin.setValue(defaults[line_type][1])
            before_axis_spin.setSuffix(" %")
            
            after_axis_spin = QtGui.QDoubleSpinBox()
            after_axis_spin.setRange(0, 50)
            after_axis_spin.setValue(defaults[line_type][2])
            after_axis_spin.setSuffix(" %")
            
            angle_spin = QtGui.QDoubleSpinBox()
            angle_spin.setRange(5, 120)
            angle_spin.setValue(angle_defaults.get(line_type, 45.0))
            angle_spin.setSuffix(" °")
            angle_spin.setSingleStep(5.0)
            
            num_bands_spin = QtGui.QSpinBox()
            num_bands_spin.setRange(1, 10)
            num_bands_spin.setValue(defaults[line_type][3])
            
            param_table.setCellWidget(row, 0, intrados_spin)
            param_table.setCellWidget(row, 1, before_axis_spin)
            param_table.setCellWidget(row, 2, after_axis_spin)
            param_table.setCellWidget(row, 3, angle_spin)
            param_table.setCellWidget(row, 4, num_bands_spin)
            
            line_spinboxes[line_type] = (intrados_spin, before_axis_spin, after_axis_spin, num_bands_spin)
            angle_spinboxes[line_type] = angle_spin
        
        # Show/hide columns and help text based on mode
        def update_columns():
            is_angle = mode_combo.currentIndex() == 1
            param_table.setColumnHidden(1, is_angle)   # % Fwd
            param_table.setColumnHidden(2, is_angle)   # % Aft
            param_table.setColumnHidden(3, not is_angle)  # Angle
            help_label.setText(HELP_ANGLE if is_angle else HELP_PERCENT)
        
        mode_combo.currentIndexChanged.connect(update_columns)
        update_columns()  # Apply initial state
        
        layout.addWidget(param_table)
        
        # Vertical offset for all diagonals (in mm)
        layout.addWidget(QtGui.QLabel(""))
        offset_layout = QtGui.QHBoxLayout()
        offset_layout.addWidget(QtGui.QLabel("Vertical offset (from extrados):"))
        offset_spin = QtGui.QSpinBox()
        offset_spin.setRange(0, 100)
        offset_spin.setValue(getattr(self.parametric_glider, 'diagonal_autofill_offset', 0))
        offset_spin.setSuffix(" mm")
        offset_layout.addWidget(offset_spin)
        offset_layout.addStretch()
        layout.addLayout(offset_layout)
        
        # Button box
        buttons = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        
        if dialog.exec_() != QtGui.QDialog.Accepted:
            return
        
        # Determine selected mode
        use_angle_mode = mode_combo.currentIndex() == 1
        
        # Save entered values to ParametricGlider for persistence
        self.parametric_glider.diagonal_autofill_mode = 'angle' if use_angle_mode else 'percent'
        self.parametric_glider.diagonal_autofill_params = {}
        self.parametric_glider.diagonal_autofill_angles = {}
        for line_type, (intrados_spin, before_spin, after_spin, num_bands_spin) in line_spinboxes.items():
            self.parametric_glider.diagonal_autofill_params[line_type] = (
                intrados_spin.value(),
                before_spin.value(),
                after_spin.value(),
                num_bands_spin.value()
            )
            self.parametric_glider.diagonal_autofill_angles[line_type] = angle_spinboxes[line_type].value()
        self.parametric_glider.diagonal_autofill_offset = offset_spin.value()
        
        # Get vertical offset in mm - will be converted per position using real profile thickness
        offset_mm = offset_spin.value()
        
        # Get glider 3D data for profile information
        glider_3d = self.parametric_glider.get_glider_3d()
        
        # Get a reference rib for profile thickness calculation
        # Use middle rib as reference
        ref_rib_index = len(glider_3d.ribs) // 2
        ref_rib = glider_3d.ribs[ref_rib_index]
        ref_chord = ref_rib.chord
        

        
        # Function to calculate height value for a given x position with offset
        def get_extrados_height_with_offset(x_pos, offset_mm):
            """
            Calculate the height value (0-1 range for extrados) considering the offset.
            Uses real profile thickness at the given x position.
            """
            if offset_mm == 0:
                return 1.0
            
            profile = ref_rib.profile_2d
            try:
                # Get upper and lower points at this x position
                upper_point = profile.profilepoint(x_pos, h=1.0)
                lower_point = profile.profilepoint(x_pos, h=-1.0)
                
                # Local thickness in profile units (0-1 normalized)
                local_thickness_normalized = upper_point[1] - lower_point[1]
                
                # Convert to real thickness in cm
                local_thickness_cm = local_thickness_normalized * ref_chord * 100
                
                if local_thickness_cm < 0.1:
                    return 1.0  # Very thin, no offset
                
                # Height reduction: offset_mm / (local_thickness_cm * 10) gives fraction,
                # multiply by 2 for height range (-1 to 1)
                offset_cm = offset_mm / 10.0
                height_reduction = (offset_cm / local_thickness_cm) * 2.0
                
                extrados_height = 1.0 - height_reduction
                return max(0.0, min(1.0, extrados_height))
            except:
                # Fallback to simple calculation
                return 1.0 - offset_mm * 0.005
        
        # Calculate extrados_height at a reference position (middle of chord)
        ref_extrados_height = get_extrados_height_with_offset(0.3, offset_mm)

        
        # Function to split extrados range into multiple bands with gaps
        def split_range_into_bands(start, end, num_bands):
            """
            Divise une plage [start, end] en num_bands bandes avec num_bands-1 espaces.
            Retourne liste de tuples (band_start, band_end).
            
            Ex: split_range_into_bands(0.02, 0.28, 3) 
            → [(0.02, 0.072), (0.124, 0.176), (0.228, 0.28)]
            """
            if num_bands <= 1:
                return [(start, end)]
            
            total_width = end - start
            num_segments = 2 * num_bands - 1  # bandes + espaces
            segment_width = total_width / num_segments
            
            bands = []
            for i in range(num_bands):
                band_start = start + (2 * i) * segment_width
                band_end = band_start + segment_width
                bands.append((band_start, band_end))
            return bands
        
        # Get per-line-type parameters
        chord_cm = ref_chord * 100  # Convert to cm
        line_params = {}
        for line_type, (intrados_spin, before_spin, after_spin, num_bands_spin) in line_spinboxes.items():
            # Convert cm to fraction of chord, % to fraction
            half_width_intrados = (intrados_spin.value() / 2) / chord_cm
            before_axis = before_spin.value() / 100.0
            after_axis = after_spin.value() / 100.0
            num_bands = num_bands_spin.value()
            angle_deg = angle_spinboxes[line_type].value()
            
            line_params[line_type] = {
                "half_intrados": half_width_intrados,
                "before_axis": before_axis,
                "after_axis": after_axis,
                "num_bands": num_bands,
                "angle_deg": angle_deg,
            }
        
        # Default params for unknown line types
        default_params = line_params.get("A", {
            "half_intrados": 0.02,
            "before_axis": 0.05,
            "after_axis": 0.05,
            "num_bands": 1,
            "angle_deg": 45.0,
        })
        
        # Pre-compute pull axis projections for all ribs (keyed by rib index)
        # Store full projection data for both % mode (extrados_intersection_x) and angle mode
        rib_projections = {}  # rib_index -> {ap_rib_pos -> projection_dict}
        rib_extrados_polys = {}  # rib_index -> extrados PolyLine2D
        for rib_idx, rib in enumerate(glider_3d.ribs):
            projections = compute_pull_axis_projection(rib, glider_3d)
            if projections:
                proj_map = {}
                for proj in projections:
                    # Key by the ORIGINAL parametric rib_pos (before apply_ss_rib_warp moved it
                    # to the intrados intersection), so _get_axis_x can match it against
                    # the values returned by _get_suspended_ribs_with_layer (parametric lineset).
                    key_pos = getattr(proj['ap'], '_orig_rib_pos', abs(proj['ap'].rib_pos))
                    proj_map[round(key_pos, 4)] = proj
                rib_projections[rib_idx] = proj_map
            # Cache extrados poly for angle intersection
            try:
                rib_extrados_polys[rib_idx] = rib.profile_2d.get_extrados_poly()
            except Exception:
                pass
        
        def _get_axis_x(projections, rib_idx, rib_pos, before, after):
            """Look up pull axis extrados intersection for a given rib+AP."""
            if rib_idx in projections:
                proj_map = projections[rib_idx]
                best_key = min(proj_map.keys(), key=lambda k: abs(k - round(rib_pos, 4)), default=None)
                if best_key is not None and abs(best_key - round(rib_pos, 4)) < 0.05:
                    return proj_map[best_key]['extrados_intersection_x']
            return rib_pos
        
        def _get_full_proj(projections, rib_idx, rib_pos):
            """Get full projection dict for a given rib+AP (for angle mode)."""
            if rib_idx in projections:
                proj_map = projections[rib_idx]
                best_key = min(proj_map.keys(), key=lambda k: abs(k - round(rib_pos, 4)), default=None)
                if best_key is not None and abs(best_key - round(rib_pos, 4)) < 0.05:
                    return proj_map[best_key]
            return None
        
        def _compute_angle_extrados(rib_idx, rib_pos, half_intrados, angle_deg):
            """Compute extrados x-values using angle mode (trapezoid shape).
            
            The diagonal is a truncated triangle (trapezoid):
            - Base: intrados width centered on AP (2 * half_intrados)
            - Sides: at ±angle/2 from pull axis (bisector)
            - Top: intersection with extrados
            
            Rays start from each base EDGE and go in the direction
            at ±angle/2 from the pull axis until they hit the extrados.
            """
            proj = _get_full_proj(rib_projections, rib_idx, rib_pos)
            if proj is None:
                return None
            
            extrados_poly = rib_extrados_polys.get(rib_idx)
            if extrados_poly is None:
                return None
            
            import numpy as np
            
            # Pull axis direction (from pilot to AP, normalized 2D profile coords)
            line_dir = proj['line_direction_2d']
            
            # Base edge positions (intrados, in profile coords)
            rib = glider_3d.ribs[rib_idx]
            base_front_x = rib_pos - half_intrados
            base_back_x = rib_pos + half_intrados
            base_front_pt = rib.profile_2d.align([base_front_x, -1.0])
            base_back_pt = rib.profile_2d.align([base_back_x, -1.0])
            
            # Rotate pull axis by ±angle/2
            half_angle = math.radians(angle_deg / 2.0)
            cos_a = math.cos(half_angle)
            sin_a = math.sin(half_angle)
            
            # Front side: rotate pull axis by +angle/2 (toward front/LE)
            ray_front = np.array([
                line_dir[0] * cos_a - line_dir[1] * sin_a,
                line_dir[0] * sin_a + line_dir[1] * cos_a
            ])
            # Back side: rotate pull axis by -angle/2 (toward back/TE)
            ray_back = np.array([
                line_dir[0] * cos_a + line_dir[1] * sin_a,
                -line_dir[0] * sin_a + line_dir[1] * cos_a
            ])
            
            # Intersect: front ray from front base edge, back ray from back base edge
            far_factor = rib.chord * 100
            
            ext_front_pt = extrados_poly.line_intersection(
                base_front_pt, base_front_pt + ray_front * far_factor
            )
            ext_back_pt = extrados_poly.line_intersection(
                base_back_pt, base_back_pt + ray_back * far_factor
            )
            
            if ext_front_pt is not None and ext_back_pt is not None:
                x1 = ext_front_pt[0]
                x2 = ext_back_pt[0]
                return (min(x1, x2), max(x1, x2))
            
            return None
        
        rib_attachments = self._get_suspended_ribs_with_layer()
        cell_count = self._get_cell_count()
        
        
        # Track which cells have diagonals from which direction
        # cell_diagonals[cell_no] = {"from_left": [...], "from_right": [...]}
        # Each entry is (rib_pos, layer, params)
        cell_diagonals = {i: {"from_left": [], "from_right": []} for i in range(cell_count)}
        
        # Generate diagonals from each attachment point
        for rib_no, attachments in sorted(rib_attachments.items()):
            for rib_pos, layer in attachments:
                # Get params for this line type
                params = line_params.get(layer, default_params)
                
                # For a rib at index N:
                # - Cell N-1 has this rib as its RIGHT side (rib2)
                # - Cell N has this rib as its LEFT side (rib1)
                
                # Create diagonal to the left cell (where this rib is the right side)
                left_cell = rib_no - 1
                if left_cell >= 0 and left_cell < cell_count:
                    cell_diagonals[left_cell]["from_right"].append((rib_pos, layer, params))
                
                # Create diagonal to the right cell (where this rib is the left side)
                right_cell = rib_no
                if right_cell >= 0 and right_cell < cell_count:
                    cell_diagonals[right_cell]["from_left"].append((rib_pos, layer, params))
        
        # Group diagonals by parameters (separate lists for diagonals and bands)
        diagonals_grouped = {}
        bands_grouped = {}
        
        for cell_no in range(cell_count):
            cell_data = cell_diagonals[cell_no]
            
            # Diagonals from left side of cell (attachment on left rib -> goes to right rib extrados)
            for rib_pos, layer, params in cell_data["from_left"]:
                half_bottom = params["half_intrados"]
                before_axis = params["before_axis"]
                after_axis = params["after_axis"]
                num_bands = params.get("num_bands", 1)
                
                # Get the axis x for this AP on its rib (left rib of this cell = rib cell_no)
                left_rib_idx = cell_no
                
                if use_angle_mode:
                    angle_result = _compute_angle_extrados(
                        left_rib_idx, rib_pos, half_bottom, params.get('angle_deg', 45))
                    if angle_result:
                        ext_start, ext_end = angle_result
                    else:
                        axis_x = _get_axis_x(rib_projections, left_rib_idx, rib_pos, before_axis, after_axis)
                        ext_start = max(0, axis_x - before_axis)
                        ext_end = min(1, axis_x + after_axis)
                else:
                    axis_x = _get_axis_x(rib_projections, left_rib_idx, rib_pos, before_axis, after_axis)
                    ext_start = max(0, axis_x - before_axis)
                    ext_end = min(1, axis_x + after_axis)
                
                # Calculate extrados height at the middle of the range
                mid_x = (ext_start + ext_end) / 2
                ext_height = get_extrados_height_with_offset(mid_x, offset_mm)
                
                # Split extrados range into bands if num_bands > 1
                bands = split_range_into_bands(ext_start, ext_end, num_bands)
                
                for band_start, band_end in bands:
                    # Intrados: centered on rib_pos
                    # Extrados: from band_start to band_end (axis-relative positions)
                    key = (
                        round(band_start, 4), ext_height,          # right_front (extrados start)
                        round(band_end, 4), ext_height,            # right_back (extrados end)
                        round(rib_pos + half_bottom, 4), -1.0,     # left_back (intrados)
                        round(rib_pos - half_bottom, 4), -1.0,     # left_front (intrados)
                    )
                    if key not in diagonals_grouped:
                        diagonals_grouped[key] = []
                    diagonals_grouped[key].append(cell_no)
            
            # Diagonals from right side of cell (attachment on right rib -> goes to left rib extrados)
            for rib_pos, layer, params in cell_data["from_right"]:
                half_bottom = params["half_intrados"]
                before_axis = params["before_axis"]
                after_axis = params["after_axis"]
                num_bands = params.get("num_bands", 1)
                
                # Get the axis x for this AP on its rib (right rib of this cell = rib cell_no + 1)
                right_rib_idx = cell_no + 1
                
                if use_angle_mode:
                    angle_result = _compute_angle_extrados(
                        right_rib_idx, rib_pos, half_bottom, params.get('angle_deg', 45))
                    if angle_result:
                        ext_start, ext_end = angle_result
                    else:
                        axis_x = _get_axis_x(rib_projections, right_rib_idx, rib_pos, before_axis, after_axis)
                        ext_start = max(0, axis_x - before_axis)
                        ext_end = min(1, axis_x + after_axis)
                else:
                    axis_x = _get_axis_x(rib_projections, right_rib_idx, rib_pos, before_axis, after_axis)
                    ext_start = max(0, axis_x - before_axis)
                    ext_end = min(1, axis_x + after_axis)
                
                # Calculate extrados height at the middle of the range
                mid_x = (ext_start + ext_end) / 2
                ext_height = get_extrados_height_with_offset(mid_x, offset_mm)
                
                # Split extrados range into bands if num_bands > 1
                bands = split_range_into_bands(ext_start, ext_end, num_bands)
                
                for band_start, band_end in bands:
                    # Intrados: centered on rib_pos
                    # Extrados: from band_start to band_end (axis-relative positions)
                    key = (
                        round(rib_pos - half_bottom, 4), -1.0,     # right_front (intrados)
                        round(rib_pos + half_bottom, 4), -1.0,     # right_back (intrados)
                        round(band_end, 4), ext_height,            # left_back (extrados end)
                        round(band_start, 4), ext_height,          # left_front (extrados start)
                    )
                    if key not in diagonals_grouped:
                        diagonals_grouped[key] = []
                    diagonals_grouped[key].append(cell_no)
        
        # Create horizontal bands on cells that need them, PER LINE TYPE
        # Each line type (A, B, C, D) gets its own bands on cells that don't have diagonals of that type
        # Bands connect extrados to extrados (never to intrados)
        
        # Track which cells have diagonals of each line type AND their extrados ranges
        # cells_by_layer[layer] = set of cell numbers with diagonals of that layer
        cells_by_layer = {}
        layer_params = {}  # Store params for each layer
        # layer_ext_ranges[(layer, cell_no)] = (ext_start, ext_end, ext_height)
        layer_ext_ranges = {}
        
        for cell_no in range(cell_count):
            cell_data = cell_diagonals[cell_no]
            for direction in ["from_left", "from_right"]:
                for rib_pos, layer, params in cell_data[direction]:
                    if layer not in cells_by_layer:
                        cells_by_layer[layer] = set()
                        layer_params[layer] = params
                    cells_by_layer[layer].add(cell_no)
                    
                    # Compute the extrados range for this diagonal (same as above)
                    before_axis = params["before_axis"]
                    after_axis = params["after_axis"]
                    half_intrados = params["half_intrados"]
                    if direction == "from_left":
                        rib_idx = cell_no
                    else:
                        rib_idx = cell_no + 1
                    
                    if use_angle_mode:
                        angle_result = _compute_angle_extrados(
                            rib_idx, rib_pos, half_intrados, params.get('angle_deg', 45))
                        if angle_result:
                            ext_start, ext_end = angle_result
                        else:
                            axis_x = _get_axis_x(rib_projections, rib_idx, rib_pos, before_axis, after_axis)
                            ext_start = max(0, axis_x - before_axis)
                            ext_end = min(1, axis_x + after_axis)
                    else:
                        axis_x = _get_axis_x(rib_projections, rib_idx, rib_pos, before_axis, after_axis)
                        ext_start = max(0, axis_x - before_axis)
                        ext_end = min(1, axis_x + after_axis)
                    mid_x = (ext_start + ext_end) / 2
                    ext_height = get_extrados_height_with_offset(mid_x, offset_mm)
                    
                    # Store (or update) the extrados range for this layer+cell
                    layer_ext_ranges[(layer, cell_no)] = (ext_start, ext_end, ext_height)
        
        # For each line type, find gaps and create bands using neighbor ranges
        for layer, cells_with_diag in cells_by_layer.items():
            params = layer_params[layer]
            num_bands = params.get("num_bands", 1)
            
            # Find cells that DON'T have diagonals of this layer
            for cell_no in range(cell_count):
                if cell_no in cells_with_diag:
                    continue  # This cell has a diagonal of this layer, skip
                
                # Check if there are neighboring cells with this layer's diagonals
                has_left_neighbor = (cell_no - 1) in cells_with_diag if cell_no > 0 else False
                has_right_neighbor = (cell_no + 1) in cells_with_diag if cell_no < cell_count - 1 else False
                
                # Only create band if there are diagonals on at least one side
                if has_left_neighbor or has_right_neighbor:
                    # Look up the extrados range from the nearest neighbor diagonal
                    ext_start = ext_end = ext_height = None
                    
                    if has_left_neighbor and (layer, cell_no - 1) in layer_ext_ranges:
                        left_range = layer_ext_ranges[(layer, cell_no - 1)]
                        ext_start, ext_end, ext_height = left_range
                    
                    if has_right_neighbor and (layer, cell_no + 1) in layer_ext_ranges:
                        right_range = layer_ext_ranges[(layer, cell_no + 1)]
                        if ext_start is None:
                            ext_start, ext_end, ext_height = right_range
                        else:
                            # Average between left and right neighbor ranges
                            ext_start = (ext_start + right_range[0]) / 2
                            ext_end = (ext_end + right_range[1]) / 2
                            ext_height = (ext_height + right_range[2]) / 2
                    
                    if ext_start is None:
                        continue  # No valid range found
                    
                    # Split the horizontal bands using the same logic as diagonals
                    bands = split_range_into_bands(ext_start, ext_end, num_bands)
                    
                    # Create a mini-band for each segment
                    for band_start, band_end in bands:
                        # Band is horizontal - all 4 corners at same height
                        band_key = (
                            round(band_start, 4), ext_height,  # right_front
                            round(band_end, 4), ext_height,    # right_back
                            round(band_end, 4), ext_height,    # left_back
                            round(band_start, 4), ext_height,  # left_front
                        )
                        if band_key not in bands_grouped:
                            bands_grouped[band_key] = []
                        if cell_no not in bands_grouped[band_key]:
                            bands_grouped[band_key].append(cell_no)
        

        
        # Convert grouped diagonals to list format
        diagonals = []
        for key, cells in diagonals_grouped.items():
            diag = {
                "right_front": (key[0], key[1]),
                "right_back": (key[2], key[3]),
                "left_back": (key[4], key[5]),
                "left_front": (key[6], key[7]),
                "cells": sorted(set(cells)),
            }
            diagonals.append(diag)
        
        # Add bands as SEPARATE entries (all 4 heights same)
        for key, cells in bands_grouped.items():
            band = {
                "right_front": (key[0], key[1]),
                "right_back": (key[2], key[3]),
                "left_back": (key[4], key[5]),
                "left_front": (key[6], key[7]),
                "cells": sorted(set(cells)),
            }
            diagonals.append(band)
        

        
        # Sort by position
        diagonals.sort(key=lambda d: (d["cells"][0] if d["cells"] else 0, d["right_front"][0]))
        
        # Populate the diagonals table
        self.diagonals_table.set_diagonals(diagonals)
        self.diagonals_table.show()

    def auto_fill_vector_straps(self):
        """
        Auto-generate vector straps connecting intrados attachment points.
        Shows configuration dialog first to get parameters.
        """
        # Configuration dialog
        dialog = QtGui.QDialog()
        dialog.setWindowTitle("Vector Straps Configuration")
        layout = QtGui.QFormLayout(dialog)
        
        # Width in mm
        width_spin = QtGui.QSpinBox()
        width_spin.setRange(10, 1000)
        width_spin.setValue(40)
        width_spin.setSuffix(" mm")
        layout.addRow("Strap width:", width_spin)
        
        # Button box
        buttons = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
        
        if dialog.exec_() != QtGui.QDialog.Accepted:
            return
        
        # Get width value (convert mm to fraction)
        width_mm = width_spin.value()
        half_width = (width_mm / 2) / 1000.0
        
        # Get attachment points (excluding brake lines)
        rib_attachments = self._get_suspended_ribs(exclude_brake=True)
        cell_count = self._get_cell_count()
        
        # Collect all unique positions and their rib locations
        position_tolerance = 0.02  # 2% tolerance for grouping
        
        all_positions = []
        for rib_no, positions in rib_attachments.items():
            for pos in positions:
                all_positions.append((rib_no, pos))
        
        # Sort by position
        all_positions.sort(key=lambda x: x[1])
        
        # Group similar positions
        position_groups = []
        used = set()
        for rib_no, pos in all_positions:
            if (rib_no, pos) in used:
                continue
            
            # Find all positions close to this one
            group = [(rib_no, pos)]
            used.add((rib_no, pos))
            
            for other_rib, other_pos in all_positions:
                if (other_rib, other_pos) not in used:
                    if abs(other_pos - pos) < position_tolerance:
                        group.append((other_rib, other_pos))
                        used.add((other_rib, other_pos))
            
            position_groups.append(group)
        
        # Use dict to group straps by center position
        straps_grouped = {}
        width = half_width * 2  # Full width for storage
        
        # Create straps for each position group
        for group in position_groups:
            if len(group) < 2:
                continue  # Need at least 2 points to create a strap
            
            # Get average position (rounded) - this is the center
            avg_pos = round(sum(p[1] for p in group) / len(group), 4)
            
            # Get max rib (straps go from center/cell 0 to this rib)
            rib_nos = [p[0] for p in group]
            max_rib = max(rib_nos)
            
            # Key is center position
            key = avg_pos
            if key not in straps_grouped:
                straps_grouped[key] = []
            
            # Cells go from 0 (center) to max_rib
            for cell_no in range(0, min(max_rib, cell_count)):
                if cell_no not in straps_grouped[key]:
                    straps_grouped[key].append(cell_no)
        
        # Convert to list format with width
        straps = []
        for center_pos, cells in straps_grouped.items():
            if cells:
                strap = {
                    "left": center_pos,  # Center position
                    "right": center_pos, # Same position on both sides
                    "width": width,      # Width in profile fraction
                    "cells": sorted(set(cells)),
                }
                straps.append(strap)
        
        # Sort by position
        straps.sort(key=lambda s: s["left"])
        
        # Populate the vector straps table
        self.vector_table.set_straps(straps)
        self.vector_table.show()

    def add_custom_vector_strap(self):
        """
        Add a custom vector strap at any chord position.
        Shows dialog to configure position, width, and cell range.
        """
        dialog = QtGui.QDialog()
        dialog.setWindowTitle("Ajouter une bande de tension personnalisée")
        layout = QtGui.QFormLayout(dialog)
        
        cell_count = self._get_cell_count()
        
        # Position on chord (%)
        position_spin = QtGui.QDoubleSpinBox()
        position_spin.setRange(0, 100)
        position_spin.setValue(50.0)
        position_spin.setSuffix(" %")
        position_spin.setDecimals(1)
        layout.addRow("Position sur corde:", position_spin)
        
        # Width in mm
        width_spin = QtGui.QSpinBox()
        width_spin.setRange(10, 1000)
        width_spin.setValue(40)
        width_spin.setSuffix(" mm")
        layout.addRow("Largeur:", width_spin)
        
        # Cell range
        cell_start_spin = QtGui.QSpinBox()
        cell_start_spin.setRange(0, cell_count - 1)
        cell_start_spin.setValue(0)
        layout.addRow("Cellule début:", cell_start_spin)
        
        cell_end_spin = QtGui.QSpinBox()
        cell_end_spin.setRange(0, cell_count - 1)
        cell_end_spin.setValue(cell_count - 1)
        layout.addRow("Cellule fin:", cell_end_spin)
        
        # Height on profile (-1 = intrados, 0 = middle, 1 = extrados)
        height_spin = QtGui.QDoubleSpinBox()
        height_spin.setRange(-1.0, 1.0)
        height_spin.setValue(0.0)  # Default to middle
        height_spin.setSingleStep(0.1)
        height_spin.setDecimals(2)
        layout.addRow("Hauteur (-1 int, 0 mid, 1 ext):", height_spin)
        
        # Button box
        buttons = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
        
        if dialog.exec_() != QtGui.QDialog.Accepted:
            return
        
        # Get values
        position = position_spin.value() / 100.0  # Convert % to fraction
        width_mm = width_spin.value()
        width = width_mm / 1000.0  # Convert mm to meters (fraction)
        cell_start = cell_start_spin.value()
        cell_end = cell_end_spin.value()
        height = height_spin.value()
        
        # Ensure cell_start <= cell_end
        if cell_start > cell_end:
            cell_start, cell_end = cell_end, cell_start
        
        # Create cells list
        cells = list(range(cell_start, cell_end + 1))
        
        # Find next empty row in vector table
        table = self.vector_table.table
        insert_row = 0
        for row in range(table.rowCount()):
            item = table.item(row, 0)
            if item is None or not item.text().strip():
                insert_row = row
                break
            insert_row = row + 1
        
        # Add to table - now includes height column
        cells_str = ",".join(map(str, cells))
        table.setItem(insert_row, 0, round(position, 4))
        table.setItem(insert_row, 1, round(position, 4))
        table.setItem(insert_row, 2, round(width, 4))
        table.setItem(insert_row, 3, round(height, 2))
        table.setItem(insert_row, 4, cells_str)
        
        # Show table
        self.vector_table.show()


def number_input(number):
    return QtGui.QTableWidgetItem(str(number))


class diagonals_table(base_table_widget):
    name = "diagonals"
    keyword = "diagonals"

    def __init__(self):
        super(diagonals_table, self).__init__(name="diagonals")
        self.table.setRowCount(200)
        self.table.setColumnCount(9)
        self.table.setHorizontalHeaderLabels(
            [
                "right\nfront",
                "rf\nheight",
                "right\nback",
                "rb\nheight",
                "left\nback",
                "lb\nheight",
                "left\nfront",
                "lf\nheight",
                "cells",
            ]
        )

    def get_from_ParametricGlider(self, ParametricGlider):
        if "diagonals" in ParametricGlider.elements:
            diags = ParametricGlider.elements["diagonals"]
            for row, element in enumerate(diags):
                entries = list(
                    element["right_front"]
                    + element["right_back"]
                    + element["left_back"]
                    + element["left_front"]
                )
                entries.append(element["cells"])
                self.table.setRow(row, entries)

    def apply_to_glider(self, ParametricGlider):
        num_rows = self.table.rowCount()
        # remove all diagonals from the glide_2d
        ParametricGlider.elements[self.keyword] = []
        for n_row in range(num_rows):
            row = self.get_row(n_row)
            if row:
                diagonal = {}
                diagonal["right_front"] = (row[0], row[1])
                diagonal["right_back"] = (row[2], row[3])
                diagonal["left_back"] = (row[4], row[5])
                diagonal["left_front"] = (row[6], row[7])
                diagonal["cells"] = row[-1]
                ParametricGlider.elements["diagonals"].append(diagonal)

    def set_diagonals(self, diagonals_list):
        """Populate table with auto-generated diagonals."""
        # Clear existing rows
        for row in range(self.table.rowCount()):
            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                if item:
                    item.setText("")

        # Ensure the table has enough rows for all entries.
        # With num_bands > 1 the generated list can easily exceed the 200-row default,
        # causing Qt to silently drop setItem() calls beyond rowCount().
        needed = max(200, len(diagonals_list) + 20)
        if self.table.rowCount() < needed:
            self.table.setRowCount(needed)

        # Fill with new data
        for row, element in enumerate(diagonals_list):
            # Format: rf_x, rf_h, rb_x, rb_h, lb_x, lb_h, lf_x, lf_h, cells
            rf = element["right_front"]
            rb = element["right_back"]
            lb = element["left_back"]
            lf = element["left_front"]
            cells_str = ",".join(map(str, element["cells"]))

            entries = [rf[0], rf[1], rb[0], rb[1], lb[0], lb[1], lf[0], lf[1], cells_str]
            for col, value in enumerate(entries):
                self.table.setItem(row, col, value)

    def get_row(self, n_row):
        str_row = []
        for i in range(9):
            item = self.table.item(n_row, i)
            if item:
                text = item.text()
                if text and text.strip():
                    # Replace comma with dot for decimal separator
                    str_row.append(text.strip().replace(",", "."))
        
        if len(str_row) != 9:
            return None
        try:
            return list(map(float, str_row[:-1])) + [
                list(map(int, str_row[-1].replace(".", ",").split(",")))
            ]
        except (TypeError, ValueError) as e:
            print(e)
            print("something wrong with row " + str(n_row))
            return None


class vector_table(base_table_widget):
    def __init__(self):
        super(vector_table, self).__init__(name="vector straps")
        self.table.setRowCount(200)
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels(["left", "right", "width", "height", "cells"])

    def get_from_ParametricGlider(self, ParametricGlider):
        # Try straps first, then tension_lines for backwards compatibility
        straps = ParametricGlider.elements.get("straps", [])
        if not straps:
            straps = ParametricGlider.elements.get("tension_lines", [])
        
        for row, element in enumerate(straps):
            left = element.get("left", element.get("center_left", 0))
            right = element.get("right", element.get("center_right", 0))
            width = element.get("width", 0.04)  # Default 40mm
            height = element.get("height", -1)  # Default to intrados
            entries = [left, right, width, height, element["cells"]]
            self.table.setRow(row, entries)

    def apply_to_glider(self, ParametricGlider):
        num_rows = self.table.rowCount()
        # Use 'straps' keyword for TensionStrap objects with width
        ParametricGlider.elements["straps"] = []
        for n_row in range(num_rows):
            row = self.get_row(n_row)
            if row:
                strap = {
                    "left": row[0],
                    "right": row[1],
                    "width": row[2],
                    "height": row[3],
                    "cells": row[4],
                }
                ParametricGlider.elements["straps"].append(strap)

    def set_straps(self, straps_list):
        """Populate table with auto-generated vector straps."""
        # Clear existing rows
        for row in range(self.table.rowCount()):
            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                if item:
                    item.setText("")
        
        # Fill with new data
        for row, element in enumerate(straps_list):
            cells_str = ",".join(map(str, element["cells"]))
            height = element.get("height", -1)  # Default height for auto-fill
            entries = [element["left"], element["right"], element["width"], height, cells_str]
            for col, value in enumerate(entries):
                self.table.setItem(row, col, value)

    def get_row(self, n_row):
        str_row = []
        for i in range(5):
            item = self.table.item(n_row, i)
            if item:
                text = item.text()
                if text and text.strip():
                    # Replace comma with dot for decimal separator
                    str_row.append(text.strip().replace(",", "."))
        
        if len(str_row) != 5:
            return None
        try:
            return list(map(float, str_row[:-1])) + [
                list(map(int, str_row[-1].replace(".", ",").split(",")))
            ]
        except (TypeError, ValueError) as e:
            print("something wrong with row " + str(n_row))
            return None
