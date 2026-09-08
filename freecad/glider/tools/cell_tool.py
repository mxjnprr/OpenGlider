
from pivy import coin
from PySide import QtCore, QtGui

from .glider import draw_glider, draw_lines
from .pull_axis_utils import compute_pull_axis_projection
from .table import base_table_widget
from .tools import BaseTool, input_field


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
        super().__init__(obj)
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
        super().accept()
        self.diagonals_table.hide()
        self.vector_table.hide()
        del self.diagonals_table
        del self.vector_table
        self.update_view_glider()

    def reject(self):
        super().reject()
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
        Supports two modes: Percentage (absolute extrados range, start/end in
        % of chord) and Angle (opening angle around the pull axis).

        A single diagonal is generated per attachment point and cell.  With
        ``Num. bands`` > 1 it is cut into bands as wide as the intrados base
        that flare out at the "flare angle" to meet on the extrados (see
        ``DiagonalRib.band_split``).  The connecting bands get one strip per
        band, with shoes on the ribs at the "shoe angle".
        """
        import math

        from openglider.glider.parametric.glider import DIAGONAL_AUTOFILL_DEFAULT_PARAMS

        pg = self.parametric_glider

        # ------------------------------------------------------------------
        # Geometry data (needed by the dialog preview and by the generation)
        # ------------------------------------------------------------------
        glider_3d = pg.get_glider_3d()
        ref_rib_index = len(glider_3d.ribs) // 2
        ref_rib = glider_3d.ribs[ref_rib_index]
        ref_chord = ref_rib.chord
        chord_cm = ref_chord * 100  # Convert to cm

        # Pre-compute pull axis projections for all ribs (keyed by rib index)
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
            try:
                rib_extrados_polys[rib_idx] = rib.profile_2d.get_extrados_poly()
            except Exception:
                pass

        rib_attachments = self._get_suspended_ribs_with_layer()
        cell_count = self._get_cell_count()

        # Reference attachment point per layer (closest to mid span) for the preview
        layer_ref = {}
        mid_rib = len(glider_3d.ribs) / 2.0
        for rib_no in sorted(rib_attachments, key=lambda r: abs(r - mid_rib)):
            for rib_pos, layer in rib_attachments[rib_no]:
                layer_ref.setdefault(layer, (rib_no, rib_pos))

        def _get_full_proj(rib_idx, rib_pos):
            """Get full projection dict for a given rib+AP."""
            proj_map = rib_projections.get(rib_idx)
            if not proj_map:
                return None
            best_key = min(proj_map.keys(), key=lambda k: abs(k - round(rib_pos, 4)))
            if abs(best_key - round(rib_pos, 4)) < 0.05:
                return proj_map[best_key]
            return None

        def _get_axis_x(rib_idx, rib_pos):
            """Look up pull axis extrados intersection for a given rib+AP."""
            proj = _get_full_proj(rib_idx, rib_pos)
            if proj is not None:
                return proj['extrados_intersection_x']
            return rib_pos

        def _compute_angle_extrados(rib_idx, rib_pos, half_intrados, angle_deg):
            """Compute extrados x-values using angle mode (trapezoid shape).

            The diagonal is a truncated triangle (trapezoid):
            - Base: intrados width centered on AP (2 * half_intrados)
            - Sides: at ±angle/2 from pull axis (bisector)
            - Top: intersection with extrados
            """
            proj = _get_full_proj(rib_idx, rib_pos)
            if proj is None:
                return None
            extrados_poly = rib_extrados_polys.get(rib_idx)
            if extrados_poly is None:
                return None

            import numpy as np

            line_dir = proj['line_direction_2d']
            rib = glider_3d.ribs[rib_idx]
            base_front_pt = rib.profile_2d.align([rib_pos - half_intrados, -1.0])
            base_back_pt = rib.profile_2d.align([rib_pos + half_intrados, -1.0])

            half_angle = math.radians(angle_deg / 2.0)
            cos_a = math.cos(half_angle)
            sin_a = math.sin(half_angle)
            ray_front = np.array([
                line_dir[0] * cos_a - line_dir[1] * sin_a,
                line_dir[0] * sin_a + line_dir[1] * cos_a
            ])
            ray_back = np.array([
                line_dir[0] * cos_a + line_dir[1] * sin_a,
                -line_dir[0] * sin_a + line_dir[1] * cos_a
            ])
            far_factor = rib.chord * 100
            ext_front_pt = extrados_poly.line_intersection(
                base_front_pt, base_front_pt + ray_front * far_factor
            )
            ext_back_pt = extrados_poly.line_intersection(
                base_back_pt, base_back_pt + ray_back * far_factor
            )
            if ext_front_pt is not None and ext_back_pt is not None:
                x1, x2 = ext_front_pt[0], ext_back_pt[0]
                return (min(x1, x2), max(x1, x2))
            return None

        def _ext_range(rib_idx, rib_pos, params, angle_mode):
            """Extrados range (x_start, x_end) of the diagonal for one AP."""
            if angle_mode:
                result = _compute_angle_extrados(
                    rib_idx, rib_pos, params["half_intrados"], params["angle_deg"])
                if result:
                    return result
            # Percentage mode: the values entered by the user ARE the extrados
            # range (absolute % of chord), independent of the pull axis.
            lo = min(max(params["ext_start"], 0.0), 1.0)
            hi = min(max(params["ext_end"], 0.0), 1.0)
            return (min(lo, hi), max(lo, hi))

        def get_extrados_height_with_offset(x_pos, offset_mm):
            """
            Calculate the height value (0-1 range for extrados) considering the offset.
            Uses real profile thickness at the given x position.
            """
            if offset_mm == 0:
                return 1.0
            profile = ref_rib.profile_2d
            try:
                upper_point = profile.profilepoint(x_pos, h=1.0)
                lower_point = profile.profilepoint(x_pos, h=-1.0)
                local_thickness_cm = (upper_point[1] - lower_point[1]) * ref_chord * 100
                if local_thickness_cm < 0.1:
                    return 1.0
                height_reduction = ((offset_mm / 10.0) / local_thickness_cm) * 2.0
                return max(0.0, min(1.0, 1.0 - height_reduction))
            except Exception:
                return 1.0 - offset_mm * 0.005

        # ------------------------------------------------------------------
        # Configuration dialog
        # ------------------------------------------------------------------
        dialog = QtGui.QDialog()
        dialog.setWindowTitle("Diagonal Auto-fill Configuration")
        dialog.setMinimumWidth(720)
        layout = QtGui.QVBoxLayout(dialog)

        mode_layout = QtGui.QHBoxLayout()
        mode_layout.addWidget(QtGui.QLabel("Mode:"))
        mode_combo = QtGui.QComboBox()
        mode_combo.addItems(["Percentage", "Angle"])
        saved_mode = getattr(pg, 'diagonal_autofill_mode', 'percent')
        mode_combo.setCurrentIndex(0 if saved_mode == 'percent' else 1)
        mode_layout.addWidget(mode_combo)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        help_label = QtGui.QLabel()
        help_label.setWordWrap(True)
        help_label.setStyleSheet("color: #555; font-size: 11px; padding: 4px; background: #f8f8f8; border-radius: 3px;")
        layout.addWidget(help_label)

        HELP_BANDS = (
            " <b>Num. bands</b> &gt; 1 keeps ONE diagonal per attachment point, cut into "
            "bands as wide as the intrados base: each band rises straight, then flares out "
            "at the <i>flare angle</i> to meet its neighbours on the extrados. The "
            "connecting bands (extrados to extrados) follow the same drawing: a shoe on each "
            "rib (<i>shoe angle</i>) joined by one thin strip of the intrados width per band, "
            "in their continuity. "
            "The last column previews the extrados range on a mid-span rib; it turns red "
            "when two layers overlap."
        )
        HELP_PERCENT = (
            "<b>Percentage mode:</b> For each attachment point the diagonal goes from the "
            "attachment point (intrados) to the extrados of the adjacent rib, where it covers "
            "<i>Extrados start</i> to <i>Extrados end</i>: absolute positions in % of chord, "
            "used exactly as entered."
            + HELP_BANDS
        )
        HELP_ANGLE = (
            "<b>Angle mode:</b> For each attachment point, the pull axis is used as bisector. "
            "Two rays are traced at ±<i>angle/2</i> from the pull axis, starting from the base edges "
            "(intrados), until they meet the extrados of the adjacent cell."
            + HELP_BANDS
        )

        header = QtGui.QLabel("Parameters per line layer")
        header.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(header)

        line_types = ["A", "B", "C", "D"]
        param_table = QtGui.QTableWidget(len(line_types), 6)
        param_table.setHorizontalHeaderLabels([
            "Intrados width (mm)",     # col 0 - always visible
            "Extrados start\n(% chord)",   # col 1 - percent mode only
            "Extrados end\n(% chord)",     # col 2 - percent mode only
            "Angle (°)",               # col 3 - angle mode only
            "Num. bands",              # col 4 - always visible
            "Extrados range\n(% chord, preview)",  # col 5 - read only
        ])
        param_table.setVerticalHeaderLabels(line_types)
        param_table.horizontalHeader().setStretchLastSection(True)

        # Saved values: (intrados cm, extrados start %, extrados end %, bands);
        # the intrados width is shown in mm.
        saved_params = getattr(pg, 'diagonal_autofill_params', None)
        defaults = dict(saved_params) if saved_params else dict(DIAGONAL_AUTOFILL_DEFAULT_PARAMS)
        for line_type in line_types:
            defaults.setdefault(line_type, DIAGONAL_AUTOFILL_DEFAULT_PARAMS[line_type])
        saved_angles = getattr(pg, 'diagonal_autofill_angles', None)
        angle_defaults = dict(saved_angles) if saved_angles else {}

        line_spinboxes = {}
        angle_spinboxes = {}
        for row, line_type in enumerate(line_types):
            intrados_spin = QtGui.QDoubleSpinBox()
            intrados_spin.setRange(10, 200)
            intrados_spin.setDecimals(0)
            intrados_spin.setSingleStep(5)
            intrados_spin.setValue(defaults[line_type][0] * 10.0)
            intrados_spin.setSuffix(" mm")
            intrados_spin.setToolTip("Width of the diagonal on the intrados, also the width of each band")

            ext_start_spin = QtGui.QDoubleSpinBox()
            ext_start_spin.setRange(0, 100)
            ext_start_spin.setValue(defaults[line_type][1])
            ext_start_spin.setSuffix(" %")
            ext_start_spin.setToolTip("Front limit of the diagonal on the extrados (% of chord from the LE)")

            ext_end_spin = QtGui.QDoubleSpinBox()
            ext_end_spin.setRange(0, 100)
            ext_end_spin.setValue(defaults[line_type][2])
            ext_end_spin.setSuffix(" %")
            ext_end_spin.setToolTip("Rear limit of the diagonal on the extrados (% of chord from the LE)")

            angle_spin = QtGui.QDoubleSpinBox()
            angle_spin.setRange(5, 120)
            angle_spin.setValue(angle_defaults.get(line_type, 45.0))
            angle_spin.setSuffix(" °")
            angle_spin.setSingleStep(5.0)

            num_bands_spin = QtGui.QSpinBox()
            num_bands_spin.setRange(1, 10)
            num_bands_spin.setValue(int(defaults[line_type][3]))

            param_table.setCellWidget(row, 0, intrados_spin)
            param_table.setCellWidget(row, 1, ext_start_spin)
            param_table.setCellWidget(row, 2, ext_end_spin)
            param_table.setCellWidget(row, 3, angle_spin)
            param_table.setCellWidget(row, 4, num_bands_spin)

            line_spinboxes[line_type] = (intrados_spin, ext_start_spin, ext_end_spin, num_bands_spin)
            angle_spinboxes[line_type] = angle_spin

        layout.addWidget(param_table)

        # Vertical offset for all diagonals (in mm)
        offset_layout = QtGui.QHBoxLayout()
        offset_layout.addWidget(QtGui.QLabel("Vertical offset (from extrados):"))
        offset_spin = QtGui.QSpinBox()
        offset_spin.setRange(0, 100)
        offset_spin.setValue(getattr(pg, 'diagonal_autofill_offset', 0))
        offset_spin.setSuffix(" mm")
        offset_layout.addWidget(offset_spin)
        offset_layout.addStretch()
        layout.addLayout(offset_layout)

        # Band split: flare angles (fabric bias = 45 deg), 0 = plain shapes
        angles_layout = QtGui.QHBoxLayout()
        angles_layout.addWidget(QtGui.QLabel("Flare angle (diagonal bands):"))
        flare_spin = QtGui.QDoubleSpinBox()
        flare_spin.setRange(0, 90)
        flare_spin.setDecimals(0)
        flare_spin.setSingleStep(5.0)
        flare_spin.setValue(getattr(pg, 'diagonal_autofill_flare_angle', 40.0))
        flare_spin.setSuffix(" °")
        flare_spin.setToolTip(
            "Angle between the flare edge and the band. The flare is anchored where "
            "neighbouring bands meet on the extrados, so the angle sets how far down "
            "the band it starts (45° = fabric bias). 0° gives a plain trapezoid."
        )
        angles_layout.addWidget(flare_spin)
        angles_layout.addSpacing(20)
        angles_layout.addWidget(QtGui.QLabel("Shoe angle (connecting bands):"))
        shoe_spin = QtGui.QDoubleSpinBox()
        shoe_spin.setRange(0, 90)
        shoe_spin.setDecimals(0)
        shoe_spin.setSingleStep(5.0)
        shoe_spin.setValue(getattr(pg, 'diagonal_autofill_shoe_angle', 40.0))
        shoe_spin.setSuffix(" °")
        shoe_spin.setToolTip(
            "Angle between the shoe edge and the strip on the connecting bands. The shoe "
            "covers the full range on the rib; the angle sets its length. When both shoes "
            "would meet, the strip keeps 10 % of the cell width. 0° gives a plain rectangle."
        )
        angles_layout.addWidget(shoe_spin)
        angles_layout.addStretch()
        layout.addLayout(angles_layout)

        def _params_from_widgets(line_type):
            intrados_spin, start_spin, end_spin, num_bands_spin = line_spinboxes[line_type]
            return {
                "half_intrados": (intrados_spin.value() / 10.0 / 2) / chord_cm,
                "intrados_m": intrados_spin.value() / 1000.0,
                "ext_start": start_spin.value() / 100.0,
                "ext_end": end_spin.value() / 100.0,
                "num_bands": num_bands_spin.value(),
                "angle_deg": angle_spinboxes[line_type].value(),
            }

        def update_preview():
            angle_mode = mode_combo.currentIndex() == 1
            ranges = {}
            for line_type in line_types:
                ref = layer_ref.get(line_type)
                if ref is None:
                    continue
                try:
                    ranges[line_type] = _ext_range(
                        ref[0], ref[1], _params_from_widgets(line_type), angle_mode)
                except Exception:
                    continue
            for row, line_type in enumerate(line_types):
                item = QtGui.QTableWidgetItem()
                item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                if line_type not in ranges:
                    item.setText("—")
                    item.setToolTip("No attachment point of this layer")
                else:
                    lo, hi = ranges[line_type]
                    item.setText(f"{lo * 100:.0f} – {hi * 100:.0f} %")
                    overlaps = [
                        other for other, (o_lo, o_hi) in ranges.items()
                        if other != line_type and o_lo < hi and lo < o_hi
                    ]
                    if overlaps:
                        item.setBackground(QtGui.QBrush(QtGui.QColor(255, 190, 190)))
                        item.setToolTip("Overlaps layer(s) " + ", ".join(sorted(overlaps)))
                    else:
                        item.setToolTip("")
                param_table.setItem(row, 5, item)

        def update_columns():
            is_angle = mode_combo.currentIndex() == 1
            param_table.setColumnHidden(1, is_angle)   # % Fwd
            param_table.setColumnHidden(2, is_angle)   # % Aft
            param_table.setColumnHidden(3, not is_angle)  # Angle
            help_label.setText(HELP_ANGLE if is_angle else HELP_PERCENT)
            update_preview()

        mode_combo.currentIndexChanged.connect(update_columns)
        for line_type in line_types:
            for spin in line_spinboxes[line_type][:3] + (angle_spinboxes[line_type],):
                spin.valueChanged.connect(lambda *_: update_preview())
        update_columns()  # Apply initial state

        buttons = QtGui.QDialogButtonBox(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec_() != QtGui.QDialog.Accepted:
            return

        use_angle_mode = mode_combo.currentIndex() == 1

        # Save entered values to ParametricGlider for persistence
        pg.diagonal_autofill_mode = 'angle' if use_angle_mode else 'percent'
        pg.diagonal_autofill_params = {}
        pg.diagonal_autofill_angles = {}
        for line_type, (intrados_spin, start_spin, end_spin, num_bands_spin) in line_spinboxes.items():
            pg.diagonal_autofill_params[line_type] = (
                intrados_spin.value() / 10.0,  # stored in cm
                start_spin.value(),
                end_spin.value(),
                num_bands_spin.value()
            )
            pg.diagonal_autofill_angles[line_type] = angle_spinboxes[line_type].value()
        pg.diagonal_autofill_offset = offset_spin.value()
        pg.diagonal_autofill_flare_angle = flare_spin.value()
        pg.diagonal_autofill_shoe_angle = shoe_spin.value()

        offset_mm = offset_spin.value()
        band_split_base = make_band_split_config(pg)
        band_split_connecting = make_band_split_config(pg, connecting=True)

        # ------------------------------------------------------------------
        # Generation
        # ------------------------------------------------------------------
        line_params = {lt: _params_from_widgets(lt) for lt in line_types}
        default_params = line_params["A"]

        # Track which cells have diagonals from which direction
        # cell_diagonals[cell_no] = {"from_left": [...], "from_right": [...]}
        # Each entry is (rib_pos, layer, params)
        cell_diagonals = {i: {"from_left": [], "from_right": []} for i in range(cell_count)}
        for rib_no, attachments in sorted(rib_attachments.items()):
            for rib_pos, layer in attachments:
                params = line_params.get(layer, default_params)
                # For a rib at index N:
                # - Cell N-1 has this rib as its RIGHT side (rib2)
                # - Cell N has this rib as its LEFT side (rib1)
                left_cell = rib_no - 1
                if 0 <= left_cell < cell_count:
                    cell_diagonals[left_cell]["from_right"].append((rib_pos, layer, params))
                right_cell = rib_no
                if 0 <= right_cell < cell_count:
                    cell_diagonals[right_cell]["from_left"].append((rib_pos, layer, params))

        # Group identical diagonals; key = 4 corners + num_bands
        diagonals_grouped = {}
        # Per layer: cells with diagonals + their extrados ranges (for the connecting bands)
        cells_by_layer = {}
        layer_ext_ranges = {}  # (layer, cell_no) -> (ext_start, ext_end, ext_height)

        for cell_no in range(cell_count):
            for direction in ("from_left", "from_right"):
                for rib_pos, layer, params in cell_diagonals[cell_no][direction]:
                    half_bottom = params["half_intrados"]
                    num_bands = int(params.get("num_bands", 1))
                    # left rib of cell N is rib N, right rib is rib N+1
                    rib_idx = cell_no if direction == "from_left" else cell_no + 1
                    ext_start, ext_end = _ext_range(rib_idx, rib_pos, params, use_angle_mode)
                    ext_height = get_extrados_height_with_offset((ext_start + ext_end) / 2, offset_mm)

                    if direction == "from_left":
                        # attachment on the left rib (intrados) -> right rib extrados
                        key = (
                            round(ext_start, 4), ext_height,          # right_front (extrados start)
                            round(ext_end, 4), ext_height,            # right_back (extrados end)
                            round(rib_pos + half_bottom, 4), -1.0,    # left_back (intrados)
                            round(rib_pos - half_bottom, 4), -1.0,    # left_front (intrados)
                            num_bands,
                        )
                    else:
                        key = (
                            round(rib_pos - half_bottom, 4), -1.0,    # right_front (intrados)
                            round(rib_pos + half_bottom, 4), -1.0,    # right_back (intrados)
                            round(ext_end, 4), ext_height,            # left_back (extrados end)
                            round(ext_start, 4), ext_height,          # left_front (extrados start)
                            num_bands,
                        )
                    diagonals_grouped.setdefault(key, []).append(cell_no)

                    cells_by_layer.setdefault(layer, set()).add(cell_no)
                    layer_ext_ranges[(layer, cell_no)] = (ext_start, ext_end, ext_height)

        # Horizontal bands (extrados to extrados) on cells without a diagonal of a
        # layer but next to cells that have one: one full-range band per layer,
        # continuing the T bar of the neighbouring diagonals.
        bands_grouped = {}  # key -> {"cells": [...], "strip_width": m, "num_bands": n}
        for layer, cells_with_diag in cells_by_layer.items():
            layer_p = line_params.get(layer, default_params)
            strip_width = layer_p["intrados_m"]
            band_num = int(layer_p.get("num_bands", 1))
            for cell_no in range(cell_count):
                if cell_no in cells_with_diag:
                    continue
                left_range = layer_ext_ranges.get((layer, cell_no - 1)) if cell_no > 0 else None
                right_range = layer_ext_ranges.get((layer, cell_no + 1)) if cell_no < cell_count - 1 else None
                if left_range is None and right_range is None:
                    continue
                if left_range is not None and right_range is not None:
                    ext_start = (left_range[0] + right_range[0]) / 2
                    ext_end = (left_range[1] + right_range[1]) / 2
                    ext_height = (left_range[2] + right_range[2]) / 2
                else:
                    ext_start, ext_end, ext_height = left_range or right_range
                band_key = (
                    round(ext_start, 4), ext_height,  # right_front
                    round(ext_end, 4), ext_height,    # right_back
                    round(ext_end, 4), ext_height,    # left_back
                    round(ext_start, 4), ext_height,  # left_front
                    round(strip_width, 4), band_num,
                )
                group = bands_grouped.setdefault(
                    band_key, {"cells": [], "strip_width": strip_width, "num_bands": band_num}
                )
                if cell_no not in group["cells"]:
                    group["cells"].append(cell_no)

        diagonals = []
        for key, cells in diagonals_grouped.items():
            diag = {
                "right_front": (key[0], key[1]),
                "right_back": (key[2], key[3]),
                "left_back": (key[4], key[5]),
                "left_front": (key[6], key[7]),
                "cells": sorted(set(cells)),
            }
            if key[8] > 1 or band_split_base["flare_angle"] > 0:
                diag["band_split"] = dict(band_split_base, num=int(key[8]))
            diagonals.append(diag)

        for key, group in bands_grouped.items():
            band = {
                "right_front": (key[0], key[1]),
                "right_back": (key[2], key[3]),
                "left_back": (key[4], key[5]),
                "left_front": (key[6], key[7]),
                "cells": sorted(set(group["cells"])),
            }
            if group["num_bands"] > 1 or band_split_connecting["flare_angle"] > 0:
                # same drawing as the neighbouring diagonal: shoes on the ribs
                # joined by one strip per diagonal band, in their continuity
                band["band_split"] = dict(
                    band_split_connecting, num=group["num_bands"], strip_width=group["strip_width"]
                )
            diagonals.append(band)

        diagonals.sort(key=lambda d: (d["cells"][0] if d["cells"] else 0, d["right_front"][0]))

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


# Constants of the band split ("T" diagonals) not exposed in the auto-fill
# dialog: fabric kept along the extrados at the hole apex (m), corner rounding.
BAND_SPLIT_MARGIN_TOP = 0.01
BAND_SPLIT_CORNER_RADIUS = 0.25


def make_band_split_config(parametric_glider, num=None, connecting=False):
    """
    ``band_split`` dict for DiagonalRib from the auto-fill settings stored on
    the ParametricGlider: the flare angle of the diagonal bands, or the shoe
    angle when ``connecting`` (extrados to extrados band).
    """
    if connecting:
        angle = getattr(parametric_glider, 'diagonal_autofill_shoe_angle', 40.0)
    else:
        angle = getattr(parametric_glider, 'diagonal_autofill_flare_angle', 40.0)
    cfg = {
        "flare_angle": float(angle),
        "margin_top": BAND_SPLIT_MARGIN_TOP,
        "corner_radius": BAND_SPLIT_CORNER_RADIUS,
    }
    if num is not None:
        cfg["num"] = int(num)
    return cfg


def default_strip_width(parametric_glider):
    """Intrados width of layer A (m), used for hand-typed connecting bands."""
    params = getattr(parametric_glider, 'diagonal_autofill_params', None) or {}
    try:
        return float(params.get("A", (4.0,))[0]) / 100.0
    except (TypeError, ValueError, IndexError):
        return 0.04


def number_input(number):
    return QtGui.QTableWidgetItem(str(number))


class diagonals_table(base_table_widget):
    name = "diagonals"
    keyword = "diagonals"
    # Attributes without a dedicated geometry column, cached per row so an
    # edit round-trip does not silently wipe them from an existing glider.
    EXTRA_KEYS = ("material_code", "name", "edge_curve", "band_split")
    NUM_GEOMETRY_COLUMNS = 9  # 4 corners x (x, height) + cells
    BANDS_COLUMN = 9

    def __init__(self):
        super().__init__(name="diagonals")
        self._extra_by_row = {}
        self.table.setRowCount(200)
        self.table.setColumnCount(10)
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
                "bands\n(T split)",
            ]
        )

    @staticmethod
    def _bands_text(element):
        band_split = element.get("band_split")
        if not band_split:
            return ""
        return str(int(band_split.get("num", 1) or 1))

    def _fill_row(self, row, element):
        entries = list(
            list(element["right_front"])
            + list(element["right_back"])
            + list(element["left_back"])
            + list(element["left_front"])
        )
        entries.append(element["cells"])
        entries.append(self._bands_text(element))
        self.table.setRow(row, entries)
        self._extra_by_row[row] = {
            k: element[k] for k in self.EXTRA_KEYS if k in element
        }

    def get_from_ParametricGlider(self, ParametricGlider):
        self._extra_by_row = {}
        if "diagonals" in ParametricGlider.elements:
            diags = ParametricGlider.elements["diagonals"]
            self._ensure_rows(len(diags))
            for row, element in enumerate(diags):
                self._fill_row(row, element)

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
                # Restore preserved non-geometry attributes (material_code /
                # edge_curve / name / band_split); new rows have no cache entry.
                diagonal.update(self._extra_by_row.get(n_row, {}))
                bands = self.get_row_bands(n_row)
                if bands is not None:
                    connecting = row[1] == row[7]  # both sides on the same surface
                    band_split = dict(
                        diagonal.get("band_split")
                        or make_band_split_config(ParametricGlider, connecting=connecting)
                    )
                    band_split["num"] = bands
                    if connecting and not band_split.get("strip_width"):
                        band_split["strip_width"] = default_strip_width(ParametricGlider)
                    if bands > 1 or band_split.get("flare_angle", 0) > 0:
                        diagonal["band_split"] = band_split
                    else:
                        diagonal.pop("band_split", None)
                ParametricGlider.elements["diagonals"].append(diagonal)

    def _ensure_rows(self, count):
        # With many diagonals the generated list can exceed the 200-row default,
        # causing Qt to silently drop setItem() calls beyond rowCount().
        needed = max(200, count + 20)
        if self.table.rowCount() < needed:
            self.table.setRowCount(needed)

    def set_diagonals(self, diagonals_list):
        """Populate table with auto-generated diagonals."""
        # Fresh geometry -> drop any cached attributes of a previously loaded
        # glider so they aren't misapplied to these rows.
        self._extra_by_row = {}
        for row in range(self.table.rowCount()):
            for col in range(self.table.columnCount()):
                item = self.table.item(row, col)
                if item:
                    item.setText("")
        self._ensure_rows(len(diagonals_list))
        for row, element in enumerate(diagonals_list):
            self._fill_row(row, element)

    def _cell_text(self, n_row, col):
        item = self.table.item(n_row, col)
        if item is None:
            return ""
        return (item.text() or "").strip()

    def get_row(self, n_row):
        str_row = []
        for i in range(self.NUM_GEOMETRY_COLUMNS):
            text = self._cell_text(n_row, i)
            if text:
                # Replace comma with dot for decimal separator
                str_row.append(text.replace(",", "."))

        if len(str_row) != self.NUM_GEOMETRY_COLUMNS:
            return None
        try:
            return list(map(float, str_row[:-1])) + [
                list(map(int, str_row[-1].replace(".", ",").split(",")))
            ]
        except (TypeError, ValueError) as e:
            print(e)
            print("something wrong with row " + str(n_row))
            return None

    def get_row_bands(self, n_row):
        """Number of bands of the T split (column 9); None when left empty."""
        text = self._cell_text(n_row, self.BANDS_COLUMN)
        if not text:
            return None
        try:
            return max(1, int(float(text.replace(",", "."))))
        except (TypeError, ValueError):
            print("something wrong with bands of row " + str(n_row))
            return None


class vector_table(base_table_widget):
    def __init__(self):
        super().__init__(name="vector straps")
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
        except (TypeError, ValueError):
            print("something wrong with row " + str(n_row))
            return None
