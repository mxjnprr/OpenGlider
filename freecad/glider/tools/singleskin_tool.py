from __future__ import division

import copy
import numpy as np
from pivy import coin
from PySide import QtGui, QtCore

from openglider.glider.rib.rib import SingleSkinRib
from openglider.glider.rib import RibHole

from .glider import draw_glider, draw_lines
from .tools import BaseTool, input_field, text_field


def refresh():
    pass


class SingleSkinTool(BaseTool):
    hide = True
    turn = False
    widget_name = "Single Skin"

    def __init__(self, obj):
        super(SingleSkinTool, self).__init__(obj)

        # Get glider info
        glider = self.parametric_glider.get_glider_3d()
        self.num_cells = self.parametric_glider.shape.half_cell_num
        self.num_ribs = len(glider.ribs)

        # Load existing config from ParametricGlider if available
        self._load_existing_config()

        # Build UI
        self._build_ui()

        # Initial draw
        self.draw_glider()

    def _load_existing_config(self):
        """Load existing single skin configuration or set defaults."""
        pg = self.parametric_glider
        ss = getattr(pg, 'single_skin_config', None) or {}

        self.config = {
            "cells": ss.get("cells", []),
            "height": ss.get("height", 0.5),
            "att_dist": ss.get("att_dist", 0.02),
            "num_points": ss.get("num_points", 20),
            "le_gap": ss.get("le_gap", True),
            "te_gap": ss.get("te_gap", True),
            "double_first": ss.get("double_first", False),
            "straight_te": ss.get("straight_te", True),
            "holes": ss.get("holes", False),
            "hole_width": ss.get("hole_width", 0.3),
            "hole_height": ss.get("hole_height", 0.7),
            "min_hole_pos": ss.get("min_hole_pos", 0.2),
            "max_hole_pos": ss.get("max_hole_pos", 1.0),
            "vertical_shift": ss.get("vertical_shift", 0.2),
            "continued_min": ss.get("continued_min", False),
            "continued_min_end": ss.get("continued_min_end", 0.9),
            "continued_min_angle": ss.get("continued_min_angle", 0.0),
            "continued_min_delta_y": ss.get("continued_min_delta_y", 0.0),
            "continued_min_x": ss.get("continued_min_x", 0.0),
            "xrot": ss.get("xrot", [0.0] * self.num_ribs),
        }

    def _build_ui(self):
        """Build the complete Qt interface."""
        row = 0

        # ── Section: Cell Selection ──────────────────────────────
        header_cells = QtGui.QLabel("Cell Selection")
        header_cells.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 4px;")
        self.layout.setWidget(row, text_field, header_cells)
        row += 1

        # Quick action buttons
        btn_layout = QtGui.QHBoxLayout()
        self.btn_all = QtGui.QPushButton("All")
        self.btn_none = QtGui.QPushButton("None")
        self.btn_tips = QtGui.QPushButton("Tips")
        self.tip_count_spin = QtGui.QSpinBox()
        self.tip_count_spin.setRange(1, self.num_cells)
        self.tip_count_spin.setValue(min(3, self.num_cells))
        self.tip_count_spin.setPrefix("last ")
        self.tip_count_spin.setSuffix(" cells")

        self.btn_all.clicked.connect(self._select_all_cells)
        self.btn_none.clicked.connect(self._select_no_cells)
        self.btn_tips.clicked.connect(self._select_tip_cells)

        btn_layout.addWidget(self.btn_all)
        btn_layout.addWidget(self.btn_none)
        btn_layout.addWidget(self.btn_tips)
        btn_layout.addWidget(self.tip_count_spin)

        btn_widget = QtGui.QWidget()
        btn_widget.setLayout(btn_layout)
        self.layout.setWidget(row, input_field, btn_widget)
        row += 1

        # Cell checkboxes in a grid
        self.cell_checks = []
        cell_grid = QtGui.QGridLayout()
        cols = min(6, self.num_cells)
        for i in range(self.num_cells):
            cb = QtGui.QCheckBox("C{}".format(i + 1))
            cb.setChecked(i in self.config["cells"])
            cb.stateChanged.connect(self._on_param_changed)
            self.cell_checks.append(cb)
            cell_grid.addWidget(cb, i // cols, i % cols)

        cell_grid_widget = QtGui.QWidget()
        cell_grid_widget.setLayout(cell_grid)
        self.layout.setWidget(row, input_field, cell_grid_widget)
        row += 1

        # ── Section: Leading Edge ─────────────────────────────────
        header_le = QtGui.QLabel("Leading Edge")
        header_le.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 8px;")
        self.layout.setWidget(row, text_field, header_le)
        row += 1

        self.double_first_cb = QtGui.QCheckBox("Keep first panel double-surface (A-B)")
        self.double_first_cb.setChecked(self.config["double_first"])
        self.double_first_cb.stateChanged.connect(self._on_param_changed)
        self.layout.setWidget(row, input_field, self.double_first_cb)
        row += 1

        self.le_gap_cb = QtGui.QCheckBox("Gap at leading edge")
        self.le_gap_cb.setChecked(self.config["le_gap"])
        self.le_gap_cb.stateChanged.connect(self._on_param_changed)
        self.layout.setWidget(row, input_field, self.le_gap_cb)
        row += 1

        # ── Section: Bow Shape ────────────────────────────────────
        header_bow = QtGui.QLabel("Bow Shape")
        header_bow.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 8px;")
        self.layout.setWidget(row, text_field, header_bow)
        row += 1

        # Height slider (0 to 1)
        self.layout.setWidget(row, text_field, QtGui.QLabel("Height (0-1):"))
        height_layout = QtGui.QHBoxLayout()
        self.height_slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.height_slider.setRange(0, 100)
        self.height_slider.setValue(int(self.config["height"] * 100))
        self.height_label = QtGui.QLabel("{:.2f}".format(self.config["height"]))
        self.height_slider.valueChanged.connect(self._on_height_slider)
        height_layout.addWidget(self.height_slider)
        height_layout.addWidget(self.height_label)
        height_widget = QtGui.QWidget()
        height_widget.setLayout(height_layout)
        self.layout.setWidget(row, input_field, height_widget)
        row += 1

        # att_dist
        self.layout.setWidget(row, text_field, QtGui.QLabel("Attachment gap (m):"))
        self.att_dist_spin = QtGui.QDoubleSpinBox()
        self.att_dist_spin.setRange(0.0, 0.5)
        self.att_dist_spin.setDecimals(3)
        self.att_dist_spin.setSingleStep(0.005)
        self.att_dist_spin.setValue(self.config["att_dist"])
        self.att_dist_spin.valueChanged.connect(self._on_param_changed)
        self.layout.setWidget(row, input_field, self.att_dist_spin)
        row += 1

        # num_points
        self.layout.setWidget(row, text_field, QtGui.QLabel("Points per bow:"))
        self.num_points_spin = QtGui.QSpinBox()
        self.num_points_spin.setRange(5, 50)
        self.num_points_spin.setValue(self.config["num_points"])
        self.num_points_spin.valueChanged.connect(self._on_param_changed)
        self.layout.setWidget(row, input_field, self.num_points_spin)
        row += 1

        # ── Section: Trailing Edge ────────────────────────────────
        header_te = QtGui.QLabel("Trailing Edge")
        header_te.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 8px;")
        self.layout.setWidget(row, text_field, header_te)
        row += 1

        self.te_gap_cb = QtGui.QCheckBox("Gap at trailing edge")
        self.te_gap_cb.setChecked(self.config["te_gap"])
        self.te_gap_cb.stateChanged.connect(self._on_param_changed)
        self.layout.setWidget(row, input_field, self.te_gap_cb)
        row += 1

        self.straight_te_cb = QtGui.QCheckBox("Straight last bow")
        self.straight_te_cb.setChecked(self.config["straight_te"])
        self.straight_te_cb.stateChanged.connect(self._on_param_changed)
        self.layout.setWidget(row, input_field, self.straight_te_cb)
        row += 1

        # ── Section: Holes ────────────────────────────────────────
        header_holes = QtGui.QLabel("Holes")
        header_holes.setStyleSheet("font-weight: bold; font-size: 12px; margin-top: 8px;")
        self.layout.setWidget(row, text_field, header_holes)
        row += 1

        self.holes_cb = QtGui.QCheckBox("Enable rib holes")
        self.holes_cb.setChecked(self.config["holes"])
        self.holes_cb.stateChanged.connect(self._on_holes_toggled)
        self.layout.setWidget(row, input_field, self.holes_cb)
        row += 1

        # Holes parameters (in a collapsible group)
        self.holes_widget = QtGui.QWidget()
        holes_layout = QtGui.QFormLayout(self.holes_widget)

        self.hole_width_spin = QtGui.QDoubleSpinBox()
        self.hole_width_spin.setRange(0.05, 1.0)
        self.hole_width_spin.setDecimals(2)
        self.hole_width_spin.setValue(self.config["hole_width"])
        self.hole_width_spin.valueChanged.connect(self._on_param_changed)
        holes_layout.addRow("Width:", self.hole_width_spin)

        self.hole_height_spin = QtGui.QDoubleSpinBox()
        self.hole_height_spin.setRange(0.1, 1.0)
        self.hole_height_spin.setDecimals(2)
        self.hole_height_spin.setValue(self.config["hole_height"])
        self.hole_height_spin.valueChanged.connect(self._on_param_changed)
        holes_layout.addRow("Height:", self.hole_height_spin)

        self.min_hole_pos_spin = QtGui.QDoubleSpinBox()
        self.min_hole_pos_spin.setRange(0.0, 1.0)
        self.min_hole_pos_spin.setDecimals(2)
        self.min_hole_pos_spin.setValue(self.config["min_hole_pos"])
        self.min_hole_pos_spin.valueChanged.connect(self._on_param_changed)
        holes_layout.addRow("Min chord pos:", self.min_hole_pos_spin)

        self.max_hole_pos_spin = QtGui.QDoubleSpinBox()
        self.max_hole_pos_spin.setRange(0.0, 1.0)
        self.max_hole_pos_spin.setDecimals(2)
        self.max_hole_pos_spin.setValue(self.config["max_hole_pos"])
        self.max_hole_pos_spin.valueChanged.connect(self._on_param_changed)
        holes_layout.addRow("Max chord pos:", self.max_hole_pos_spin)

        self.vert_shift_spin = QtGui.QDoubleSpinBox()
        self.vert_shift_spin.setRange(-0.5, 0.5)
        self.vert_shift_spin.setDecimals(2)
        self.vert_shift_spin.setValue(self.config["vertical_shift"])
        self.vert_shift_spin.valueChanged.connect(self._on_param_changed)
        holes_layout.addRow("Vertical shift:", self.vert_shift_spin)

        self.holes_widget.setVisible(self.config["holes"])
        self.layout.setWidget(row, input_field, self.holes_widget)
        row += 1

        # ── Section: Advanced ─────────────────────────────────────
        self.advanced_toggle = QtGui.QPushButton("▶ Advanced (continued_min)")
        self.advanced_toggle.setFlat(True)
        self.advanced_toggle.setStyleSheet("text-align: left; color: #666; font-size: 11px; margin-top: 8px;")
        self.advanced_toggle.clicked.connect(self._toggle_advanced)
        self.layout.setWidget(row, input_field, self.advanced_toggle)
        row += 1

        self.advanced_widget = QtGui.QWidget()
        adv_layout = QtGui.QFormLayout(self.advanced_widget)

        self.continued_min_cb = QtGui.QCheckBox("Enable continued_min")
        self.continued_min_cb.setChecked(self.config["continued_min"])
        self.continued_min_cb.stateChanged.connect(self._on_param_changed)
        adv_layout.addRow(self.continued_min_cb)

        self.cont_min_end_spin = QtGui.QDoubleSpinBox()
        self.cont_min_end_spin.setRange(0.0, 1.0)
        self.cont_min_end_spin.setDecimals(2)
        self.cont_min_end_spin.setValue(self.config["continued_min_end"])
        self.cont_min_end_spin.valueChanged.connect(self._on_param_changed)
        adv_layout.addRow("End position:", self.cont_min_end_spin)

        self.cont_min_angle_spin = QtGui.QDoubleSpinBox()
        self.cont_min_angle_spin.setRange(-1.0, 1.0)
        self.cont_min_angle_spin.setDecimals(3)
        self.cont_min_angle_spin.setValue(self.config["continued_min_angle"])
        self.cont_min_angle_spin.valueChanged.connect(self._on_param_changed)
        adv_layout.addRow("Angle:", self.cont_min_angle_spin)

        self.advanced_widget.setVisible(False)
        self.layout.setWidget(row, input_field, self.advanced_widget)
        row += 1

        # ── Update Button ─────────────────────────────────────────
        self.update_button = QtGui.QPushButton("Update Preview")
        self.update_button.setStyleSheet("font-weight: bold; margin-top: 10px;")
        self.update_button.clicked.connect(self.update_preview)
        self.layout.setWidget(row, input_field, self.update_button)
        row += 1

    # ── UI Callbacks ──────────────────────────────────────────────

    def _on_height_slider(self, value):
        self.height_label.setText("{:.2f}".format(value / 100.0))
        self._on_param_changed()

    def _on_holes_toggled(self, state):
        self.holes_widget.setVisible(bool(state))
        self._on_param_changed()

    def _toggle_advanced(self):
        vis = not self.advanced_widget.isVisible()
        self.advanced_widget.setVisible(vis)
        self.advanced_toggle.setText(
            ("▼ " if vis else "▶ ") + "Advanced (continued_min)"
        )

    def _select_all_cells(self):
        for cb in self.cell_checks:
            cb.setChecked(True)

    def _select_no_cells(self):
        for cb in self.cell_checks:
            cb.setChecked(False)

    def _select_tip_cells(self):
        n = self.tip_count_spin.value()
        for i, cb in enumerate(self.cell_checks):
            cb.setChecked(i >= self.num_cells - n)

    def _on_param_changed(self, *args):
        """Called when any parameter changes. Could auto-update or just flag dirty."""
        pass  # Preview is triggered by the "Update Preview" button

    # ── Config Gathering ──────────────────────────────────────────

    def _get_selected_cells(self):
        """Return list of selected cell indices."""
        return [i for i, cb in enumerate(self.cell_checks) if cb.isChecked()]

    def _get_rib_indices_from_cells(self, cells):
        """Convert cell indices to rib indices.
        
        Only ribs where ALL adjacent cells are SS become SingleSkinRib.
        Sealed ribs include transition ribs AND both ribs of boundary
        full cells (the full cell adjacent to SS zone).
        
        Returns (rib_indices, sealed_rib_indices).
        """
        cells_set = set(cells)
        total_ribs = self.num_ribs
        total_cells = self.num_cells
        ribs = []
        sealed_ribs = set()
        boundary_full_cells = set()
        for rib_idx in range(total_ribs):
            adjacent_cells = []
            if rib_idx > 0:
                adjacent_cells.append(rib_idx - 1)
            if rib_idx < total_cells:
                adjacent_cells.append(rib_idx)
            ss_adjacent = [c for c in adjacent_cells if c in cells_set]
            non_ss_adjacent = [c for c in adjacent_cells if c not in cells_set]
            if ss_adjacent and not non_ss_adjacent:
                ribs.append(rib_idx)
            elif ss_adjacent and non_ss_adjacent:
                sealed_ribs.add(rib_idx)
                for c in non_ss_adjacent:
                    boundary_full_cells.add(c)
        # Seal BOTH ribs of each boundary full cell
        for cell_idx in boundary_full_cells:
            sealed_ribs.add(cell_idx)
            sealed_ribs.add(cell_idx + 1)
        return ribs, sealed_ribs

    def _get_single_skin_par(self):
        """Gather all single_skin_par from the UI widgets."""
        return {
            "att_dist": self.att_dist_spin.value(),
            "height": self.height_slider.value() / 100.0,
            "num_points": self.num_points_spin.value(),
            "le_gap": self.le_gap_cb.isChecked(),
            "te_gap": self.te_gap_cb.isChecked(),
            "double_first": self.double_first_cb.isChecked(),
            "straight_te": self.straight_te_cb.isChecked(),
            "continued_min": self.continued_min_cb.isChecked(),
            "continued_min_end": self.cont_min_end_spin.value(),
            "continued_min_angle": self.cont_min_angle_spin.value(),
            "continued_min_delta_y": 0.0,
            "continued_min_x": 0.0,
        }

    def _get_full_config(self):
        """Return the complete config dict for persistence."""
        config = self._get_single_skin_par()
        config["cells"] = self._get_selected_cells()
        config["holes"] = self.holes_cb.isChecked()
        config["hole_width"] = self.hole_width_spin.value()
        config["hole_height"] = self.hole_height_spin.value()
        config["min_hole_pos"] = self.min_hole_pos_spin.value()
        config["max_hole_pos"] = self.max_hole_pos_spin.value()
        config["vertical_shift"] = self.vert_shift_spin.value()
        config["xrot"] = [0.0] * self.num_ribs
        return config

    # ── Preview / Drawing ─────────────────────────────────────────

    def _apply_singleskin(self, glider):
        """Apply single skin modifications to a glider instance."""
        selected_cells = self._get_selected_cells()
        if not selected_cells:
            return glider

        cells_set = set(selected_cells)
        rib_indices, sealed_rib_indices = self._get_rib_indices_from_cells(selected_cells)
        rib_indices_set = set(rib_indices)
        single_skin_par = self._get_single_skin_par()

        # Replace ribs with SingleSkinRib
        new_ribs = []
        for i, rib in enumerate(glider.ribs):
            if i in rib_indices_set:
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

        # Clear holes from SS ribs AND sealed ribs (boundary full cell walls)
        for i, rib in enumerate(glider.ribs):
            if isinstance(rib, SingleSkinRib):
                rib.holes = []
            elif i in sealed_rib_indices:
                rib.holes = []

        # Add SS-specific holes if enabled
        if self.holes_cb.isChecked():
            hole_size = np.array([self.hole_width_spin.value(),
                                  self.hole_height_spin.value()])
            min_pos = self.min_hole_pos_spin.value()
            max_pos = self.max_hole_pos_spin.value()
            v_shift = self.vert_shift_spin.value()

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

        # Apply hull modification: replace profile_2d with the bow-modified version
        for rib in glider.ribs:
            if isinstance(rib, SingleSkinRib):
                hull_profile = rib.get_hull(glider)
                rib.profile_2d = hull_profile

        # Remove intrados panels from single-skin cells (by cell index)
        double_first = self.double_first_cb.isChecked()
        for cell_idx, cell in enumerate(glider.cells):
            if cell_idx in cells_set:
                if double_first:
                    extrados = [p for p in cell.panels if not p.is_lower()]
                    intrados = [p for p in cell.panels if p.is_lower()]
                    intrados.sort(key=lambda p: p.mean_x())
                    cell.panels = extrados + intrados[:1]
                else:
                    cell.panels = [p for p in cell.panels if not p.is_lower()]

        # Align SS rib profiles with line pull direction
        for rib in glider.ribs:
            if not isinstance(rib, SingleSkinRib):
                continue
            connected_lines = []
            for line in glider.lineset.uppermost_lines:
                if hasattr(line.upper_node, 'rib') and (
                        line.upper_node.rib is rib or
                        (hasattr(line.upper_node.rib, 'name') and
                         line.upper_node.rib.name == rib.name)):
                    connected_lines.append(line)
            if not connected_lines:
                continue
            resultant = np.zeros(3)
            for line in connected_lines:
                if line.force is not None and line.force > 0:
                    pull_dir = line.lower_node.vec - line.upper_node.vec
                    pull_dir_norm = pull_dir / np.linalg.norm(pull_dir)
                    resultant += line.force * pull_dir_norm
            res_norm = np.linalg.norm(resultant)
            if res_norm < 1e-9:
                continue
            resultant /= res_norm
            rot = rib.rotation_matrix
            chord_3d = np.array(rot([1, 0, 0]))
            current_normal = np.array(rot([0, 0, 1]))
            desired_normal = np.cross(chord_3d, resultant)
            dn_norm = np.linalg.norm(desired_normal)
            if dn_norm < 1e-9:
                continue
            desired_normal /= dn_norm
            if np.dot(desired_normal, current_normal) < 0:
                desired_normal = -desired_normal
            cos_angle = np.clip(np.dot(current_normal, desired_normal), -1, 1)
            sin_angle = np.dot(np.cross(desired_normal, current_normal), chord_3d)
            rib.xrot = np.arctan2(sin_angle, cos_angle)

        return glider

    def draw_glider(self):
        """Draw the glider with current single skin settings."""
        _rot = coin.SbRotation()
        _rot.setValue(coin.SbVec3f(0, 1, 0), coin.SbVec3f(1, 0, 0))
        rot = coin.SoRotation()
        rot.rotation.setValue(_rot)
        self.task_separator += rot

        glider = self.parametric_glider.get_glider_3d()
        glider = self._apply_singleskin(glider)

        draw_glider(
            glider,
            self.task_separator,
            hull="panels",
            ribs=True,
            fill_ribs=False,
        )
        draw_lines(
            glider,
            vis_lines=self.task_separator,
            line_num=1,
        )

    def update_preview(self):
        """Rebuild the 3D preview with current parameters."""
        self.task_separator.removeAllChildren()
        self.draw_glider()

    # ── Accept / Reject ───────────────────────────────────────────

    def accept(self):
        """Save configuration and update the glider."""
        # Store config in ParametricGlider for persistence
        self.parametric_glider.single_skin_config = self._get_full_config()
        super(SingleSkinTool, self).accept()
        self.update_view_glider()

    def reject(self):
        """Discard changes."""
        super(SingleSkinTool, self).reject()
