from .tools import BaseTool, Line_old
import FreeCADGui as Gui
from PySide import QtCore, QtGui
from openglider.glider.rib import RibHole
from openglider.utils.geometry import is_inside_triangle
import numpy as np
from pivy import coin
import os
from .pull_axis_utils import compute_pull_axis_projection

class HoleDesignTool(BaseTool):
    widget_name = "Hole Design"

    def __init__(self, obj):
        super(HoleDesignTool, self).__init__(obj)

        # UI Elements with parent widget specified
        self.ribTypeComboBox = QtGui.QComboBox(self.base_widget)
        self.holeShapeComboBox = QtGui.QComboBox(self.base_widget)
        self.numHolesSpinBox = QtGui.QSpinBox(self.base_widget)
        self.holeWidthSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.holeHeightSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.holeHeightSpinBox.hide()  # Not in layout, hide to prevent floating widget

        self.holeMarginSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.verticalShiftSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.minPosSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.maxPosSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.holeCornerRadiusSpinBox = QtGui.QDoubleSpinBox(self.base_widget)

        # Controls for no-hole zones on suspended ribs
        self.noHoleZoneLabel = QtGui.QLabel("<b>No-Hole Zone Geometry</b>", self.base_widget)
        self.noHoleArcAngleSpinBox = QtGui.QDoubleSpinBox(self.base_widget)  # Arc span angle
        self.noHoleAngleSpinBox = QtGui.QDoubleSpinBox(self.base_widget)  # Direction to extrados

        # Cone hole controls
        self.coneHoleLabel = QtGui.QLabel("<b>Cone Exclusion Holes</b>", self.base_widget)
        self.coneHolesEnabledCheckBox = QtGui.QCheckBox("Enable", self.base_widget)
        self.coneHoleNumZonesSpinBox = QtGui.QSpinBox(self.base_widget)
        self.coneHoleMarginTopSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.coneHoleMarginSideSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.coneHoleMarginBottomSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.coneHoleCornerRadiusSpinBox_cone = QtGui.QDoubleSpinBox(self.base_widget)

        # Diagonal hole controls
        self.diagHoleLabel = QtGui.QLabel("<b>Diagonal Holes</b> <i>(3D view incorrect, but 2D export correct)</i>", self.base_widget)
        self.diagHolesEnabledCheckBox = QtGui.QCheckBox("Enable", self.base_widget)
        self.diagHoleNumZonesSpinBox = QtGui.QSpinBox(self.base_widget)
        self.diagHoleMarginTopSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.diagHoleMarginSideSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.diagHoleMarginBottomSpinBox = QtGui.QDoubleSpinBox(self.base_widget)
        self.diagHoleCornerRadiusSpinBox = QtGui.QDoubleSpinBox(self.base_widget)

        # Preview rib selector - ComboBox to show only relevant ribs with real names
        self.previewRibComboBox = QtGui.QComboBox(self.base_widget)
        self._rib_indices = []  # Mapping from combo index to actual rib index

        self.applyButton = QtGui.QPushButton("Apply", self.base_widget)

        self.preview_root = coin.SoSeparator()
        self.setup_widget()
        self.setup_pivy()

    def setup_widget(self):
        # Add items to combo box
        self.ribTypeComboBox.addItems(["Non-Suspended", "Suspended"])
        self.holeShapeComboBox.addItems(["Ellipse", "Rounded Rectangle"])


        # Add widgets to the QFormLayout provided by BaseTool
        self.layout.addRow("Rib Type", self.ribTypeComboBox)
        self.layout.addRow("Hole Shape", self.holeShapeComboBox)
        self.layout.addRow("Number of Holes", self.numHolesSpinBox)
        self.layout.addRow("Hole Width (%)", self.holeWidthSpinBox)
        self.layout.addRow("Height Margin (mm)", self.holeMarginSpinBox)
        self.layout.addRow("Vertical Shift (%)", self.verticalShiftSpinBox)
        self.layout.addRow("Min Position (%)", self.minPosSpinBox)
        self.layout.addRow("Max Position (%)", self.maxPosSpinBox)
        self.layout.addRow("Corner Radius (mm)", self.holeCornerRadiusSpinBox)

        self.layout.addRow(self.noHoleZoneLabel)
        self.layout.addRow("Arc Span (deg)", self.noHoleArcAngleSpinBox)
        self.layout.addRow("Exclusion Angle (deg)", self.noHoleAngleSpinBox)

        # Cone hole controls
        self.layout.addRow(self.coneHoleLabel)
        self.layout.addRow("", self.coneHolesEnabledCheckBox)
        self.layout.addRow("Zones per Side", self.coneHoleNumZonesSpinBox)
        self.layout.addRow("Top Margin (mm)", self.coneHoleMarginTopSpinBox)
        self.layout.addRow("Side Margin (mm)", self.coneHoleMarginSideSpinBox)
        self.layout.addRow("Bottom Margin (mm)", self.coneHoleMarginBottomSpinBox)
        self.layout.addRow("Corner Radius (%)", self.coneHoleCornerRadiusSpinBox_cone)

        # Diagonal hole controls (suspended only)
        self.layout.addRow(self.diagHoleLabel)
        self.layout.addRow("", self.diagHolesEnabledCheckBox)
        self.layout.addRow("Diag Zones", self.diagHoleNumZonesSpinBox)
        self.layout.addRow("Diag Top Margin (mm)", self.diagHoleMarginTopSpinBox)
        self.layout.addRow("Diag Side Margin (mm)", self.diagHoleMarginSideSpinBox)
        self.layout.addRow("Diag Bottom Margin (mm)", self.diagHoleMarginBottomSpinBox)
        self.layout.addRow("Diag Corner Radius (%)", self.diagHoleCornerRadiusSpinBox)

        self.layout.addRow("Preview Rib", self.previewRibComboBox)

        # Right-align the apply button
        button_layout = QtGui.QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(self.applyButton)
        self.layout.addRow(button_layout)

        # Configure spinboxes
        for spinbox in [self.holeWidthSpinBox, self.holeHeightSpinBox, self.verticalShiftSpinBox]:
            spinbox.setSingleStep(0.01)
            spinbox.setDecimals(3)
            spinbox.setMinimum(0.0)
            spinbox.setMaximum(1.0) # Relative to chord

        # Margin spinbox (in mm)
        self.holeMarginSpinBox.setSingleStep(1.0) # 1mm steps
        self.holeMarginSpinBox.setDecimals(1)
        self.holeMarginSpinBox.setSuffix(" mm")
        self.holeMarginSpinBox.setRange(0.0, 1000.0) # Reasonable max margin

        # Corner radius spinbox (as percentage 0-50%)
        self.holeCornerRadiusSpinBox.setSingleStep(1.0) # 1% steps
        self.holeCornerRadiusSpinBox.setDecimals(0)
        self.holeCornerRadiusSpinBox.setSuffix(" %")
        self.holeCornerRadiusSpinBox.setRange(0.0, 50.0) # Max 50% = half the smallest dimension

        for spinbox in [self.minPosSpinBox, self.maxPosSpinBox]:
            spinbox.setSingleStep(0.01)
            spinbox.setDecimals(3)
            spinbox.setMinimum(0.0)
            spinbox.setMaximum(1.0) # Relative to chord

        self.noHoleArcAngleSpinBox.setSingleStep(5.0)
        self.noHoleArcAngleSpinBox.setMinimum(0)
        self.noHoleArcAngleSpinBox.setMaximum(180)
        self.noHoleArcAngleSpinBox.setValue(120)  # Default: 120° arc span
        
        self.noHoleAngleSpinBox.setSingleStep(1.0)
        self.noHoleAngleSpinBox.setMinimum(0)
        self.noHoleAngleSpinBox.setMaximum(90)

        # Cone hole spinbox configuration
        self.coneHoleNumZonesSpinBox.setRange(1, 3)
        self.coneHoleNumZonesSpinBox.setValue(1)

        for spinbox in [self.coneHoleMarginTopSpinBox, self.coneHoleMarginSideSpinBox, self.coneHoleMarginBottomSpinBox]:
            spinbox.setSingleStep(0.5)
            spinbox.setDecimals(1)
            spinbox.setSuffix(" mm")
            spinbox.setRange(0.0, 50.0)
            spinbox.setValue(3.0)

        self.coneHoleCornerRadiusSpinBox_cone.setSingleStep(5.0)
        self.coneHoleCornerRadiusSpinBox_cone.setDecimals(0)
        self.coneHoleCornerRadiusSpinBox_cone.setSuffix(" %")
        self.coneHoleCornerRadiusSpinBox_cone.setRange(0.0, 50.0)
        self.coneHoleCornerRadiusSpinBox_cone.setValue(25.0)

        # Diagonal hole spinbox configuration
        self.diagHoleNumZonesSpinBox.setRange(1, 5)
        self.diagHoleNumZonesSpinBox.setValue(1)

        for spinbox in [self.diagHoleMarginTopSpinBox, self.diagHoleMarginSideSpinBox, self.diagHoleMarginBottomSpinBox]:
            spinbox.setSingleStep(0.5)
            spinbox.setDecimals(1)
            spinbox.setSuffix(" mm")
            spinbox.setRange(0.0, 50.0)
            spinbox.setValue(3.0)

        self.diagHoleCornerRadiusSpinBox.setSingleStep(5.0)
        self.diagHoleCornerRadiusSpinBox.setDecimals(0)
        self.diagHoleCornerRadiusSpinBox.setSuffix(" %")
        self.diagHoleCornerRadiusSpinBox.setRange(0.0, 50.0)
        self.diagHoleCornerRadiusSpinBox.setValue(25.0)

        # Load initial values
        self.update_form_from_glider_data()

        # Connections
        self.ribTypeComboBox.currentIndexChanged.connect(self.on_rib_type_change)
        self.holeShapeComboBox.currentIndexChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.numHolesSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.holeWidthSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))

        self.holeHeightSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.holeMarginSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.verticalShiftSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.minPosSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.maxPosSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.holeCornerRadiusSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.noHoleArcAngleSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.noHoleAngleSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.coneHolesEnabledCheckBox.stateChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.coneHoleNumZonesSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.coneHoleMarginTopSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.coneHoleMarginSideSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.coneHoleMarginBottomSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.coneHoleCornerRadiusSpinBox_cone.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        # Diagonal hole signal connections
        self.diagHolesEnabledCheckBox.stateChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.diagHoleNumZonesSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.diagHoleMarginTopSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.diagHoleMarginSideSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.diagHoleMarginBottomSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.diagHoleCornerRadiusSpinBox.valueChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.previewRibComboBox.currentIndexChanged.connect(lambda: self.update_glider_data_and_preview(switch=False))
        self.applyButton.clicked.connect(self.accept)

        # Set initial visibility of no-hole zone controls (and cone holes)
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        self.noHoleZoneLabel.setVisible(is_suspended)
        self.noHoleArcAngleSpinBox.setVisible(is_suspended)
        self.layout.labelForField(self.noHoleArcAngleSpinBox).setVisible(is_suspended)
        self.noHoleAngleSpinBox.setVisible(is_suspended)
        self.layout.labelForField(self.noHoleAngleSpinBox).setVisible(is_suspended)
        # Cone hole controls
        for w in [self.coneHoleLabel, self.coneHolesEnabledCheckBox,
                   self.coneHoleNumZonesSpinBox, self.coneHoleMarginTopSpinBox,
                   self.coneHoleMarginSideSpinBox, self.coneHoleMarginBottomSpinBox,
                   self.coneHoleCornerRadiusSpinBox_cone]:
            w.setVisible(is_suspended)
            label = self.layout.labelForField(w)
            if label:
                label.setVisible(is_suspended)
        # Diagonal hole controls
        for w in [self.diagHoleLabel, self.diagHolesEnabledCheckBox,
                   self.diagHoleNumZonesSpinBox, self.diagHoleMarginTopSpinBox,
                   self.diagHoleMarginSideSpinBox, self.diagHoleMarginBottomSpinBox,
                   self.diagHoleCornerRadiusSpinBox]:
            w.setVisible(is_suspended)
            label = self.layout.labelForField(w)
            if label:
                label.setVisible(is_suspended)

    def setup_pivy(self):
        self.task_separator.addChild(self.preview_root)
        # Set spinbox max to actual rib count
        glider_instance = self.obj.Proxy.getGliderInstance()
        num_ribs = len(glider_instance.ribs) if glider_instance.ribs else 1
        self._populate_rib_combo()  # Populate with filtered ribs based on type
        self.update_preview()
        Gui.SendMsgToActiveView("ViewFit")

    def _is_truly_suspended(self, rib, glider_instance):
        """Check if rib has true suspension attachments (not just brake tabs)."""
        # Filter attachment points - exclude brake tabs (>90% chord)
        attachment_points = glider_instance.get_rib_attachment_points(rib)
        true_suspension_aps = [ap for ap in attachment_points if hasattr(ap, 'rib_pos') and ap.rib_pos <= 0.9]
        return len(true_suspension_aps) > 0
    
    def _populate_rib_combo(self):
        """Populate rib combo box with filtered ribs based on suspended/non-suspended type."""
        glider_instance = self.obj.Proxy.getGliderInstance()
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        
        # Block signals while updating
        self.previewRibComboBox.blockSignals(True)
        
        # Remember current selection if possible
        old_rib_idx = None
        if self._rib_indices and self.previewRibComboBox.currentIndex() >= 0:
            old_combo_idx = self.previewRibComboBox.currentIndex()
            if old_combo_idx < len(self._rib_indices):
                old_rib_idx = self._rib_indices[old_combo_idx]
        
        self.previewRibComboBox.clear()
        self._rib_indices = []
        
        for i, rib in enumerate(glider_instance.ribs):
            truly_suspended = self._is_truly_suspended(rib, glider_instance)
            
            # Check thickness to exclude flat ribs (like wingtips)
            try:
                # Check thickness at 30% chord (typical max thickness location)
                p_up = rib.profile_2d.profilepoint(-0.3)
                p_down = rib.profile_2d.profilepoint(0.3)
                thickness = p_up[1] - p_down[1]
                if thickness < 1e-3: # Less than 0.1% thickness -> Flat/Line
                    continue
            except:
                pass # If calculation fails, include it (safer)

            # Filter based on type
            if is_suspended and truly_suspended:
                self._rib_indices.append(i)
                self.previewRibComboBox.addItem(f"{i}: {rib.name}")
            elif not is_suspended and not truly_suspended:
                self._rib_indices.append(i)
                self.previewRibComboBox.addItem(f"{i}: {rib.name}")
        
        # Restore selection if possible
        if old_rib_idx is not None and old_rib_idx in self._rib_indices:
            new_combo_idx = self._rib_indices.index(old_rib_idx)
            self.previewRibComboBox.setCurrentIndex(new_combo_idx)
        elif self._rib_indices:
            self.previewRibComboBox.setCurrentIndex(0)
        
        self.previewRibComboBox.blockSignals(False)
    
    def get_representative_rib(self, suspended=False):
        """Get a representative rib for preview.
        
        Get rib for preview based on spinner selection (like airfoil structure).
        """
        glider_instance = self.obj.Proxy.getGliderInstance()
        combo_idx = self.previewRibComboBox.currentIndex()
        rib_idx = self._rib_indices[combo_idx] if 0 <= combo_idx < len(self._rib_indices) else 0
        
        if rib_idx < len(glider_instance.ribs):
            return glider_instance.ribs[rib_idx]
        return glider_instance.ribs[0] if glider_instance.ribs else None

    def on_height_mode_change(self, index):
        # Simplified - always use margin mode
        self.update_glider_data_and_preview(switch=False)

    def on_rib_type_change(self, new_index):
        # Save the data of the previous tab before switching
        previous_index = 1 - new_index
        self.update_glider_data(is_suspended=previous_index == 1)

        is_suspended = new_index == 1

        # Show/hide no-hole zone controls
        self.noHoleZoneLabel.setVisible(is_suspended)
        self.noHoleArcAngleSpinBox.setVisible(is_suspended)
        self.layout.labelForField(self.noHoleArcAngleSpinBox).setVisible(is_suspended)
        self.noHoleAngleSpinBox.setVisible(is_suspended)
        # Also hide the labels associated with the spinboxes
        self.layout.labelForField(self.noHoleAngleSpinBox).setVisible(is_suspended)
        # Cone hole controls
        for w in [self.coneHoleLabel, self.coneHolesEnabledCheckBox,
                   self.coneHoleNumZonesSpinBox, self.coneHoleMarginTopSpinBox,
                   self.coneHoleMarginSideSpinBox, self.coneHoleMarginBottomSpinBox,
                   self.coneHoleCornerRadiusSpinBox_cone]:
            w.setVisible(is_suspended)
            label = self.layout.labelForField(w)
            if label:
                label.setVisible(is_suspended)
        # Diagonal hole controls
        for w in [self.diagHoleLabel, self.diagHolesEnabledCheckBox,
                   self.diagHoleNumZonesSpinBox, self.diagHoleMarginTopSpinBox,
                   self.diagHoleMarginSideSpinBox, self.diagHoleMarginBottomSpinBox,
                   self.diagHoleCornerRadiusSpinBox]:
            w.setVisible(is_suspended)
            label = self.layout.labelForField(w)
            if label:
                label.setVisible(is_suspended)

        # Then, load the values for the newly selected rib type
        self.update_form_from_glider_data()
        # Repopulate rib combo with filtered ribs for the new type
        self._populate_rib_combo()
        self.update_preview()

    def update_preview(self, *args):
        self.preview_root.removeAllChildren()
        self._cached_projections = {}  # Clear projection cache

        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        rib = self.get_representative_rib(suspended=is_suspended)
        if not rib: return

        glider_instance = self.obj.Proxy.getGliderInstance()

        # Draw profile outline - scaled by chord (like airfoil structure)
        scale = rib.chord  # All preview elements should be scaled by this
        
        if hasattr(rib, 'get_hull') and glider_instance is not None:
            try:
                hull_profile = rib.get_hull(glider_instance)
            except Exception:
                hull_profile = rib.profile_2d
            profile_points = [p * scale for p in hull_profile.data]
        else:
            profile_points = [p * scale for p in rib.profile_2d.data]
            
        profile_3d = [[p[0], p[1], 0] for p in profile_points]
        self.preview_root.addChild(Line_old(profile_3d + [profile_3d[0]], width=2).object)

        no_hole_zones = []
        halfmoon_circles = []  # List of (center, radius) for each halfmoon - used for exclusion
        if is_suspended:
            all_attachment_points = glider_instance.get_rib_attachment_points(rib)
            # Filter to exclude brake tabs (>90% chord) - only true suspension attachments
            attachment_points = [ap for ap in all_attachment_points if hasattr(ap, 'rib_pos') and ap.rib_pos <= 0.9]
            for ap in attachment_points:
                # Visualize attachment point - scale by chord
                ap_pos_norm = rib.profile_2d.align([ap.rib_pos, -1.0])
                ap_pos_scaled = ap_pos_norm * scale
                marker = coin.SoSeparator()
                trans = coin.SoTranslation()
                trans.translation.setValue(ap_pos_scaled[0], ap_pos_scaled[1], 0)
                mat = coin.SoMaterial()
                mat.diffuseColor.setValue(1, 0, 0) # Red
                sphere = coin.SoSphere()
                sphere.radius = 0.005 * scale  # Scale marker size too
                marker.addChild(trans)
                marker.addChild(mat)
                marker.addChild(sphere)
                self.preview_root.addChild(marker)

                # Define and draw no-hole zones
                angle = self.noHoleAngleSpinBox.value()
                # Keep v1 normalized for geometric calculations (extrados_poly uses normalized coords)
                v1 = ap_pos_norm  # Apex on intrados (normalized)
                v1_scaled = ap_pos_scaled  # For drawing only
                
                angle_rad = np.deg2rad(angle)

                # Use shared projection utility for this rib
                if not hasattr(self, '_cached_projections'):
                    self._cached_projections = {}
                cache_key = id(rib)
                if cache_key not in self._cached_projections:
                    self._cached_projections[cache_key] = compute_pull_axis_projection(rib, glider_instance)
                projections = self._cached_projections[cache_key]
                
                # Find this AP's projection result
                proj_data = None
                for proj in projections:
                    if proj['ap'] is ap or (hasattr(proj['ap'], 'rib_pos') and abs(proj['ap'].rib_pos - ap.rib_pos) < 0.001):
                        proj_data = proj
                        break
                
                if proj_data is not None:
                    line_direction = proj_data['line_direction_2d']
                    angle_offset = proj_data['angle_offset']
                    pilot_2d_norm = proj_data['pilot_2d_norm']
                    pilot_2d = pilot_2d_norm * scale
                    
                    # Draw pilot point marker (blue) — only for first AP to avoid clutter
                    if ap is attachment_points[0]:
                        pilot_marker = coin.SoSeparator()
                        pilot_trans = coin.SoTransform()
                        pilot_trans.translation.setValue(pilot_2d[0], pilot_2d[1], 0)
                        pilot_mat = coin.SoMaterial()
                        pilot_mat.diffuseColor.setValue(0, 0, 1)  # Blue
                        pilot_sphere = coin.SoSphere()
                        pilot_sphere.radius = 0.05 * scale
                        pilot_marker.addChild(pilot_trans)
                        pilot_marker.addChild(pilot_mat)
                        pilot_marker.addChild(pilot_sphere)
                        self.preview_root.addChild(pilot_marker)
                    
                    # Draw line from pilot to AP
                    self.preview_root.addChild(Line_old([pilot_2d, v1_scaled], color='green', width=1).object)
                else:
                    # Fallback to vertical if no projection data
                    line_direction = np.array([0, 1])
                    angle_offset = np.arctan2(1, 0)
                
                # The angle_offset is now based on the suspension line direction
                # This makes the exclusion angle centered on the line axis
                angle_offset = np.arctan2(line_direction[1], line_direction[0])
                
                # Draw the suspension line axis for visualization (scaled)
                axis_end_scaled = v1_scaled + line_direction * 0.1 * scale
                self.preview_root.addChild(Line_old([v1_scaled, axis_end_scaled], color='yellow', width=2).object)

                dir2 = np.array([np.cos(angle_offset - angle_rad), np.sin(angle_offset - angle_rad)])
                dir3 = np.array([np.cos(angle_offset + angle_rad), np.sin(angle_offset + angle_rad)])

                extrados_poly = rib.profile_2d.get_extrados_poly()

                # Use a very large number to ensure the line cuts through the extrados
                far_factor = rib.chord * 100

                v2 = extrados_poly.line_intersection(v1, v1 + dir2 * far_factor)
                v3 = extrados_poly.line_intersection(v1, v1 + dir3 * far_factor)

                if v2 is not None and v3 is not None:
                    # Get reinforcement config for this attachment point
                    pg = self.parametric_glider
                    halfmoon_radius_norm = 0.0
                    
                    if getattr(pg, 'reinforcement_enabled_s', False):
                        apply_all = getattr(pg, 'reinforcement_apply_all_s', False)
                        master_config = getattr(pg, 'reinforcement_master_s', {})
                        configs = getattr(pg, 'reinforcement_configs_s', [])
                        
                        # Find this AP's index among valid attachment points
                        all_aps = glider_instance.get_rib_attachment_points(rib)
                        valid_aps = [a for a in all_aps if a.rib_pos <= 0.90]
                        valid_aps.sort(key=lambda x: x.rib_pos)
                        
                        try:
                            ap_index = valid_aps.index(ap)
                            if apply_all:
                                config = master_config
                            else:
                                config = configs[ap_index] if ap_index < len(configs) else master_config
                            
                            if config.get('enabled', True):
                                halfmoon_radius = config.get('halfmoon_radius', 0.03)  # in meters
                                halfmoon_radius_norm = halfmoon_radius / rib.chord
                                # Store center (v1) and radius for hole exclusion check
                                halfmoon_circles.append((v1, halfmoon_radius_norm))
                        except (ValueError, IndexError):
                            pass
                    
                    if halfmoon_radius_norm > 1e-6:
                        # TWO-ANGLE SYSTEM:
                        # 1. Arc Span angle defines where on the halfmoon the exclusion sides START
                        # 2. Exclusion Angle defines the direction of sides going to extrados
                        # BOTH are now centered on the suspension line axis (angle_offset)
                        
                        arc_span_deg = self.noHoleArcAngleSpinBox.value()  # e.g., 120 degrees
                        arc_span_rad = np.deg2rad(arc_span_deg)
                        half_arc = arc_span_rad / 2.0
                        
                        # Arc start points at ±(arc_span/2) from the SUSPENSION LINE AXIS
                        # angle_offset = direction toward pilot point
                        arc_angle_left = angle_offset + half_arc   # Left side of arc
                        arc_angle_right = angle_offset - half_arc  # Right side of arc
                        
                        # Calculate arc start points (relative to v1 as center)
                        v1_left = v1 + halfmoon_radius_norm * np.array([np.cos(arc_angle_left), np.sin(arc_angle_left)])
                        v1_right = v1 + halfmoon_radius_norm * np.array([np.cos(arc_angle_right), np.sin(arc_angle_right)])
                        
                        # Now trace lines from arc points at the Side Angle to extrados
                        v2_new = extrados_poly.line_intersection(v1_left, v1_left + dir3 * far_factor)
                        v3_new = extrados_poly.line_intersection(v1_right, v1_right + dir2 * far_factor)
                        
                        if v2_new is not None and v3_new is not None:
                            # Generate the FULL HALFMOON arc (from 0° to 180°)
                            # This ensures holes are excluded from the entire reinforcement area
                            full_halfmoon = []
                            num_arc_pts = 20
                            for i in range(num_arc_pts + 1):
                                t = i / num_arc_pts
                                arc_ang = np.pi + t * (-np.pi)  # 180° to 0° (left to right through top)
                                arc_pt = v1 + halfmoon_radius_norm * np.array([np.cos(arc_ang), np.sin(arc_ang)])
                                full_halfmoon.append(arc_pt)
                            
                            # Generate the upper arc portion (from arc_angle_left to arc_angle_right)
                            upper_arc = []
                            for i in range(num_arc_pts + 1):
                                t = i / num_arc_pts
                                arc_ang = arc_angle_left - t * (arc_angle_left - arc_angle_right)
                                arc_pt = v1 + halfmoon_radius_norm * np.array([np.cos(arc_ang), np.sin(arc_ang)])
                                upper_arc.append(arc_pt)
                            
                            # Trace the EXTRADOS curve between v2_new and v3_new
                            # This makes the top of the exclusion zone follow the airfoil shape
                            extrados_curve = []
                            # Find the positions on extrados
                            v2_x, v3_x = v2_new[0], v3_new[0]
                            x_min, x_max = min(v2_x, v3_x), max(v2_x, v3_x)
                            num_ext_pts = 15
                            for i in range(num_ext_pts + 1):
                                t = i / num_ext_pts
                                x = v2_x + t * (v3_x - v2_x)
                                # Find corresponding y on extrados at this x
                                # Use profile_2d to get extrados point
                                ext_pts = [p for p in rib.profile_2d.data if p[1] > 0]  # extrados points
                                closest = min(ext_pts, key=lambda p: abs(p[0] - x), default=None)
                                if closest is not None:
                                    extrados_curve.append(np.array(closest))
                                else:
                                    # Fallback: interpolate
                                    y = v2_new[1] + t * (v3_new[1] - v2_new[1])
                                    extrados_curve.append(np.array([x, y]))
                            
                            # Exclusion zone: full halfmoon + extrados area
                            # The zone polygon goes: full_halfmoon + sides to extrados + extrados curve back
                            exclusion_polygon = full_halfmoon + [v3_new] + list(reversed(extrados_curve)) + [v2_new, full_halfmoon[0]]
                            no_hole_zones.append(tuple(exclusion_polygon))
                            
                            # For visual, draw: upper arc -> right side -> extrados curve -> left side -> close
                            zone_points = upper_arc + [v3_new] + list(reversed(extrados_curve)) + [v2_new, upper_arc[0]]
                            # Scale for drawing
                            zone_points_scaled = [p * scale for p in zone_points]
                            self.preview_root.addChild(Line_old(zone_points_scaled, color='red', width=1).object)
                        else:
                            # Fallback to original triangle
                            no_hole_zones.append((v1, v2, v3))
                            zone_points = [v1, v2, v3, v1]
                            zone_points_scaled = [p * scale for p in zone_points]
                            self.preview_root.addChild(Line_old(zone_points_scaled, color='red', width=1).object)
                    else:
                        # No reinforcement - use simple triangle
                        no_hole_zones.append((v1, v2, v3))
                        zone_points = [v1, v2, v3, v1]
                        zone_points_scaled = [p * scale for p in zone_points]
                        self.preview_root.addChild(Line_old(zone_points_scaled, color='red', width=1).object)
                    
                    # === CONE HOLE SUBDIVISION ===
                    # If cone holes are enabled, inscribe holes within the exclusion zone
                    # Using the actual zone boundary lines (with arc span offset)
                    if self.coneHolesEnabledCheckBox.isChecked():
                      try:
                        num_zones = self.coneHoleNumZonesSpinBox.value()
                        margin_top_m = self.coneHoleMarginTopSpinBox.value() / 1000.0
                        margin_side_m = self.coneHoleMarginSideSpinBox.value() / 1000.0
                        margin_bottom_m = self.coneHoleMarginBottomSpinBox.value() / 1000.0
                        cone_corner_radius = self.coneHoleCornerRadiusSpinBox_cone.value() / 100.0
                        
                        # Convert margins to normalized coords
                        margin_top = margin_top_m / rib.chord
                        margin_side = margin_side_m / rib.chord
                        margin_bottom = margin_bottom_m / rib.chord
                        
                        # Determine the actual zone boundary lines
                        # Each side has a bottom point (on halfmoon/AP) and top point (on extrados)
                        # The center axis goes from v1 toward angle_offset
                        d_center = np.array([np.cos(angle_offset), np.sin(angle_offset)])
                        
                        if halfmoon_radius_norm > 1e-6:
                            inner_radius = halfmoon_radius_norm
                            # Center axis bottom point = point on halfmoon at angle_offset
                            center_bottom = v1 + inner_radius * d_center
                        else:
                            inner_radius = 0.005
                            center_bottom = v1 + inner_radius * d_center
                        
                        # Center axis top = intersection with extrados
                        center_top = extrados_poly.line_intersection(
                            v1, v1 + d_center * far_factor)
                        if center_top is None:
                            raise ValueError("No center-extrados intersection")
                        
                        # Build the two sides' boundary data:
                        # Each side = [(bottom_left, top_left), (bottom_right, top_right)]
                        # "left" side of the cone = between center axis and left edge
                        # "right" side of the cone = between right edge and center axis
                        
                        if halfmoon_radius_norm > 1e-6:
                            arc_span_deg = self.noHoleArcAngleSpinBox.value()
                            arc_span_rad = np.deg2rad(arc_span_deg)
                            half_arc = arc_span_rad / 2.0
                            
                            # Arc points (where the zone sides start on the halfmoon)
                            arc_angle_left = angle_offset + half_arc
                            arc_angle_right = angle_offset - half_arc
                            
                            left_edge_bottom = v1 + inner_radius * np.array(
                                [np.cos(arc_angle_left), np.sin(arc_angle_left)])
                            right_edge_bottom = v1 + inner_radius * np.array(
                                [np.cos(arc_angle_right), np.sin(arc_angle_right)])
                            
                            # Edge top points: from arc points in dir3/dir2 direction to extrados
                            left_edge_top = extrados_poly.line_intersection(
                                left_edge_bottom, left_edge_bottom + dir3 * far_factor)
                            right_edge_top = extrados_poly.line_intersection(
                                right_edge_bottom, right_edge_bottom + dir2 * far_factor)
                        else:
                            # No reinforcement: sides from v1 to v2/v3
                            left_edge_bottom = v1.copy()
                            right_edge_bottom = v1.copy()
                            left_edge_top = v3  # dir3 → left edge
                            right_edge_top = v2  # dir2 → right edge
                        
                        if left_edge_top is None or right_edge_top is None:
                            raise ValueError("No edge-extrados intersection")
                        
                        # Process each side (left side and right side of the cone)
                        for side_data in [
                            # Left side: from center axis to left edge
                            (center_bottom, center_top, left_edge_bottom, left_edge_top),
                            # Right side: from right edge to center axis
                            (right_edge_bottom, right_edge_top, center_bottom, center_top),
                        ]:
                            s_bot_left, s_top_left, s_bot_right, s_top_right = side_data
                            
                            # For N zones, create N+1 boundary lines by interpolation
                            for zone_i in range(num_zones):
                                t0 = zone_i / num_zones
                                t1 = (zone_i + 1) / num_zones
                                
                                # Interpolate boundary line endpoints
                                bl_bot = s_bot_left * (1 - t0) + s_bot_right * t0
                                bl_top = s_top_left * (1 - t0) + s_top_right * t0
                                br_bot = s_bot_left * (1 - t1) + s_bot_right * t1
                                br_top = s_top_left * (1 - t1) + s_top_right * t1
                                
                                # Direction vectors of each boundary line
                                d_left_line = bl_top - bl_bot
                                d_right_line = br_top - br_bot
                                len_left = np.linalg.norm(d_left_line)
                                len_right = np.linalg.norm(d_right_line)
                                
                                if len_left < 1e-9 or len_right < 1e-9:
                                    continue
                                
                                d_left_unit = d_left_line / len_left
                                d_right_unit = d_right_line / len_right
                                
                                # Perpendiculars pointing INWARD (toward zone center)
                                # Left boundary: rotate +90° → points toward right
                                perp_left = np.array([-d_left_unit[1], d_left_unit[0]])
                                # Right boundary: rotate -90° → points toward left
                                perp_right = np.array([d_right_unit[1], -d_right_unit[0]])
                                
                                # Offset side lines by margin_side
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
                                    # Sides cross: find intersection → V-shaped bottom
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
                                
                                # Validate top
                                ref_vec_t = br_bot - bl_bot
                                if np.dot(p_tr - p_tl, ref_vec_t) <= 0:
                                    continue
                                if np.dot(p_tl - p_bl, d_center) <= 0:
                                    continue
                                
                                num_curve_pts = 10
                                num_fillet_pts = 6
                                cr = cone_corner_radius
                                
                                angle_tr = np.arctan2(p_tr[1] - v1[1], p_tr[0] - v1[0])
                                angle_tl = np.arctan2(p_tl[1] - v1[1], p_tl[0] - v1[0])
                                ad_top = angle_tl - angle_tr
                                while ad_top > np.pi: ad_top -= 2 * np.pi
                                while ad_top < -np.pi: ad_top += 2 * np.pi
                                
                                if single_bottom:
                                    # === V-SHAPE POLYGON ===
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
                                    # === NORMAL ARC POLYGON ===
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
                                
                                # Draw cone hole
                                hole_pts_scaled = [p * scale for p in hole_pts]
                                hole_line = Line_old(hole_pts_scaled, color='blue', width=2)
                                self.preview_root.addChild(hole_line.object)
                      except Exception as e:
                        import traceback
                        print(f"[ConeHole] Error in cone hole subdivision: {e}")
                        traceback.print_exc()
                    
                    

        # Get current parameters from the UI
        num_holes = self.numHolesSpinBox.value()
        hole_width_perc = self.holeWidthSpinBox.value()
        hole_height_perc = self.holeHeightSpinBox.value()
        hole_height_mode = 0  # Default: margin mode
        hole_margin_m = self.holeMarginSpinBox.value() / 1000.0 # Convert mm to m for usage
        vertical_shift_perc = self.verticalShiftSpinBox.value()
        hole_shape_index = self.holeShapeComboBox.currentIndex()
        min_pos = self.minPosSpinBox.value()
        max_pos = self.maxPosSpinBox.value()

        if num_holes == 0:
            return

        allowed_ranges = [(min_pos, max_pos)]

        # Visualize Airfoil Structure elements (rod sleeves and reinforcements)
        pg = self.parametric_glider
        glider_instance = self.obj.Proxy.getGliderInstance()
        rib_idx = glider_instance.ribs.index(rib) if rib in glider_instance.ribs else 0
        
        # Draw rod sleeves from configuration
        from openglider.glider.rib.elements import RodSleeve
        suffix = '_s' if is_suspended else '_ns'
        
        # Draw extrados sleeves
        extrados_enabled = getattr(pg, f'extrados_sleeves_enabled{suffix}', True)
        extrados_configs = getattr(pg, f'extrados_sleeves{suffix}', [])
        print(f"DEBUG: extrados_enabled={extrados_enabled}, configs={len(extrados_configs)}, suffix={suffix}")
        if extrados_enabled and extrados_configs:
            for config in extrados_configs:
                excluded_ribs = config.get('excluded_ribs', [])
                if rib_idx not in excluded_ribs:
                    try:
                        sleeve = RodSleeve(
                            surface='extrados',
                            width=config.get('width', 0.015),
                            offset=config.get('offset', 0.005),
                            start_chord=config.get('start_chord', 0.0),
                            end_chord=config.get('end_chord', 0.7),
                        )
                        inner_pts, outer_pts = sleeve.get_sleeve_points(rib)
                        if inner_pts and outer_pts:
                            # Draw at full scale (already scaled by rib.chord in get_sleeve_points)
                            sleeve_poly = list(inner_pts) + list(reversed(outer_pts)) + [inner_pts[0]]
                            self.preview_root.addChild(Line_old(sleeve_poly, color='green', width=2).object)
                    except Exception as e:
                        import traceback
                        print(f"Error drawing extrados sleeve: {e}")
                        traceback.print_exc()
        
        # Draw intrados sleeves
        intrados_enabled = getattr(pg, f'intrados_sleeves_enabled{suffix}', True)
        intrados_configs = getattr(pg, f'intrados_sleeves{suffix}', [])
        print(f"DEBUG: intrados_enabled={intrados_enabled}, configs={len(intrados_configs)}, suffix={suffix}")
        if intrados_enabled and intrados_configs:
            for config in intrados_configs:
                excluded_ribs = config.get('excluded_ribs', [])
                if rib_idx not in excluded_ribs:
                    try:
                        sleeve = RodSleeve(
                            surface='intrados',
                            width=config.get('width', 0.015),
                            offset=config.get('offset', 0.005),
                            start_chord=config.get('start_chord', 0.06),
                            end_chord=config.get('end_chord', 0.5),
                        )
                        inner_pts, outer_pts = sleeve.get_sleeve_points(rib)
                        if inner_pts and outer_pts:
                            # Draw at full scale (already scaled by rib.chord in get_sleeve_points)
                            sleeve_poly = list(inner_pts) + list(reversed(outer_pts)) + [inner_pts[0]]
                            self.preview_root.addChild(Line_old(sleeve_poly, color='white', width=2).object)
                    except Exception as e:
                        import traceback
                        print(f"Error drawing intrados sleeve: {e}")
                        traceback.print_exc()
        
        # Draw reinforcements if they exist for this rib (suspended only)
        if is_suspended and hasattr(rib, 'reinforcements') and rib.reinforcements:
            for reinf in rib.reinforcements:
                try:
                    halfmoon_pts = reinf.get_halfmoon_points(rib)
                    if halfmoon_pts:
                        # Draw at full scale (already scaled in get_halfmoon_points)
                        self.preview_root.addChild(Line_old(list(halfmoon_pts), color='yellow', width=2).object)
                except Exception as e:
                    print(f"Error drawing reinforcement: {e}")

        # Distribute holes across the allowed ranges
        total_allowable_length = sum(end - start for start, end in allowed_ranges)
        if total_allowable_length <= 1e-6:
            return

        # A more robust way to distribute N holes across M ranges
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
                upper_point = rib.profile_2d.profilepoint(-pos_x)
                lower_point = rib.profile_2d.profilepoint(pos_x)
                local_thickness = upper_point[1] - lower_point[1]
                if local_thickness < 1e-6:
                    continue

                new_lower_bound = lower_point
                if is_suspended:
                    hole_center_x = (upper_point[0] + lower_point[0]) / 2.0
                    min_y_ceiling = upper_point[1]
                    
                    # Check if this position is inside any halfmoon circle
                    inside_halfmoon = False
                    for hm_center, hm_radius in halfmoon_circles:
                        dist = np.sqrt((hole_center_x - hm_center[0])**2 + (lower_point[1] - hm_center[1])**2)
                        if dist < hm_radius:
                            inside_halfmoon = True
                            break
                    
                    if inside_halfmoon:
                        continue  # Skip hole positions inside halfmoon

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

                    available_height = min_y_ceiling - lower_point[1]
                    hole_center = np.array([hole_center_x, lower_point[1] + available_height / 2])
                    
                    # Apply vertical shift within available height
                    hole_center[1] += available_height / 2 * vertical_shift_perc
                    
                else:
                    available_height = upper_point[1] - lower_point[1]
                    hole_center = lower_point + (upper_point - lower_point) / 2 * (1 + vertical_shift_perc)

                if available_height < 1e-4:
                    continue
                
                # Always use margin mode for hole height
                hole_height = available_height - 2 * hole_margin_m
                
                hole_width = hole_width_perc * rib.chord

                if hole_height <= 0 or hole_width <= 0:
                    continue

                if hole_shape_index == 0: # Ellipse
                    shape_points = []
                    for angle in np.linspace(0, 2 * np.pi, 50):
                        x = hole_width / 2 * np.cos(angle)
                        y = hole_height / 2 * np.sin(angle)
                        shape_points.append([x, y])
                else: # Rounded Rectangle
                    corner_radius_ratio = self.holeCornerRadiusSpinBox.value() / 100.0 # Convert % to ratio
                    shape_points = self.create_rounded_rectangle(hole_width, hole_height, corner_radius_ratio)

                shape_poly = np.array(shape_points)
                shape_poly += hole_center

                shape_points_closed = list(shape_poly)
                # Scale for drawing (points are in normalized coords)
                shape_points_scaled = [p * scale for p in shape_points_closed]
                self.preview_root.addChild(Line_old(shape_points_scaled + [shape_points_scaled[0]], color='blue').object)

        # === DIAGONAL 2D PREVIEW ===
        if self.diagHolesEnabledCheckBox.isChecked():
            try:
                self._draw_diagonal_preview(rib, scale)
            except Exception:
                pass

    def _draw_diagonal_preview(self, rib, scale):
        """Draw 2D flattened diagonals adjacent to the selected rib."""
        from numpy.linalg import norm
        glider_instance = self.obj.Proxy.getGliderInstance()
        
        # Find cells adjacent to this rib
        diag_items = []  # (drib, cell, side_idx)
        for cell in glider_instance.cells:
            if cell.rib1 is rib or cell.rib2 is rib:
                for drib in cell.diagonals:
                    left_h = (drib.left_front[1], drib.left_back[1])
                    right_h = (drib.right_front[1], drib.right_back[1])
                    left_is_int = (left_h[0] == -1.0 and left_h[1] == -1.0)
                    right_is_int = (right_h[0] == -1.0 and right_h[1] == -1.0)
                    left_is_ext = (left_h[0] > 0 and left_h[1] > 0)
                    right_is_ext = (right_h[0] > 0 and right_h[1] > 0)
                    is_full = (left_is_int and right_is_ext) or (right_is_int and left_is_ext)
                    if is_full:
                        side = 0 if cell.rib1 is rib else 1
                        diag_items.append((drib, cell, side))
        
        if not diag_items:
            return
        
        # Get diagonal hole config
        pg = self.parametric_glider
        config = {
            'num_zones': self.diagHoleNumZonesSpinBox.value(),
            'margin_top_m': self.diagHoleMarginTopSpinBox.value() / 1000.0,
            'margin_side_m': self.diagHoleMarginSideSpinBox.value() / 1000.0,
            'margin_bottom_m': self.diagHoleMarginBottomSpinBox.value() / 1000.0,
            'corner_radius_pct': self.diagHoleCornerRadiusSpinBox.value() / 100.0,
        }
        
        # Position diagonals ABOVE the rib profile, aligned on AP axis
        y_base = 0.18 * scale  # Base offset above the rib
        
        for diag_idx, (drib, cell, side_idx) in enumerate(diag_items):
            try:
                left, right = drib.get_flattened(cell)
            except Exception:
                continue
            
            if side_idx == 0:
                front = drib.left_front
                back = drib.left_back
                inner = left
                other_inner = right
                target_rib = cell.rib1
            else:
                front = drib.right_front
                back = drib.right_back
                inner = right
                other_inner = left
                target_rib = cell.rib2
            
            if front[1] != -1:
                continue
            
            # Find the AP position to align the diagonal on the suspension axis
            x_front, x_back = front[0], back[0]
            t_ap_range = (min(x_front, x_back), max(x_front, x_back))
            all_aps = glider_instance.get_rib_attachment_points(target_rib)
            aps_in_range = [ap for ap in all_aps 
                          if hasattr(ap, 'rib_pos') and ap.rib_pos <= 0.9
                          and t_ap_range[0] <= ap.rib_pos <= t_ap_range[1]]
            
            if not aps_in_range:
                continue
            
            # Use first AP for alignment
            ap = aps_in_range[0]
            t_ap = 0.5 if abs(x_back - x_front) < 1e-9 else (ap.rib_pos - x_front) / (x_back - x_front)
            t_ap = min(max(t_ap, 0), 1)
            
            inner_total = inner.get_length()
            other_total = other_inner.get_length()
            if inner_total < 1e-9 or other_total < 1e-9:
                continue
            ik_ap = inner.walk(0, t_ap * inner_total)
            p_ap_diag = np.array(inner[ik_ap])
            
            # Center of outer curve (extrados side)
            ik_center_outer = other_inner.walk(0, t_ap * other_total)
            p_center_outer = np.array(other_inner[ik_center_outer])
            
            # Compute rotation: rotate AP→center_outer to point upward (0, 1)
            pull_dir = p_center_outer - p_ap_diag
            pull_len = norm(pull_dir)
            if pull_len < 1e-9:
                continue
            pull_dir = pull_dir / pull_len
            
            # Rotation angle: from pull_dir to (0, 1)
            # To rotate (ux, uy) to (0, 1): cos(a) = uy, sin(a) = ux
            cos_a = pull_dir[1]
            sin_a = pull_dir[0]
            
            # AP position on the rib profile (scaled)
            ap_rib_pos = rib.profile_2d.align([ap.rib_pos, -1.0])
            ap_rib_scaled = ap_rib_pos * scale
            
            # Y base above rib
            y_offset = y_base + diag_idx * 0.08 * scale
            
            def transform_pt(p):
                """Rotate point around AP then translate to rib position."""
                rel = np.array(p) - p_ap_diag
                rx = cos_a * rel[0] - sin_a * rel[1]
                ry = sin_a * rel[0] + cos_a * rel[1]
                return [ap_rib_scaled[0] + rx, ap_rib_scaled[1] + y_offset + ry, 0]
            
            # Draw outline
            outline_pts = []
            for i in range(len(inner)):
                outline_pts.append(transform_pt(inner[i]))
            outline_pts.append(transform_pt(other_inner[len(other_inner)-1]))
            for i in range(len(other_inner) - 1, -1, -1):
                outline_pts.append(transform_pt(other_inner[i]))
            outline_pts.append(transform_pt(inner[0]))
            
            self.preview_root.addChild(Line_old(outline_pts, color='green', width=2).object)
            
            # Draw front and back edges
            front_edge = [transform_pt(inner[0]), transform_pt(other_inner[0])]
            back_edge = [transform_pt(inner[len(inner)-1]), transform_pt(other_inner[len(other_inner)-1])]
            self.preview_root.addChild(Line_old(front_edge, color='green').object)
            self.preview_root.addChild(Line_old(back_edge, color='green').object)
            
            # Draw holes with corner rounding
            for ap_hole in aps_in_range:
                hole_polys = self._compute_diag_holes(
                    ap_hole, inner, other_inner, front, back, config)
                for poly in hole_polys:
                    pts_3d = [transform_pt(p) for p in poly]
                    self.preview_root.addChild(Line_old(pts_3d, color='blue', width=1).object)
    
    def _compute_diag_holes(self, ap, inner, other_inner, front, back, config):
        """Compute diagonal hole polygons (same logic as cell.py _create_diag_holes).
        Returns list of polygon point lists."""
        from numpy.linalg import norm
        
        num_zones = config['num_zones']
        margin_side = config['margin_side_m']
        margin_top = config['margin_top_m']
        margin_bottom = config['margin_bottom_m']
        corner_pct = config['corner_radius_pct']
        
        x_front, x_back = front[0], back[0]
        t_ap = 0.5 if abs(x_back - x_front) < 1e-9 else (ap.rib_pos - x_front) / (x_back - x_front)
        t_ap = min(max(t_ap, 0), 1)
        
        inner_total = inner.get_length()
        other_total = other_inner.get_length()
        if inner_total < 1e-9 or other_total < 1e-9:
            return []
        
        ik_ap_inner = inner.walk(0, t_ap * inner_total)
        p_ap = np.array(inner[ik_ap_inner])
        
        ik_ap_outer = other_inner.walk(0, t_ap * other_total)
        p_center_outer = np.array(other_inner[ik_ap_outer])
        
        center_dir = p_center_outer - p_ap
        center_len = norm(center_dir)
        if center_len < 1e-9:
            return []
        d_center = center_dir / center_len
        
        A = np.array(inner[0])
        B = np.array(other_inner[0])
        C = np.array(inner[len(inner) - 1])
        D = np.array(other_inner[len(other_inner) - 1])
        
        front_dir = B - p_ap
        front_len = norm(front_dir)
        back_dir = D - p_ap
        back_len = norm(back_dir)
        if front_len < 1e-9 or back_len < 1e-9:
            return []
        d_front = front_dir / front_len
        d_back = back_dir / back_len
        
        results = []
        
        for side_data in [(d_center, d_front), (d_center, d_back)]:
            s_d_left_edge, s_d_right_edge = side_data
            
            for zone_i in range(num_zones):
                t0 = zone_i / num_zones
                t1 = (zone_i + 1) / num_zones
                
                d_left = (1 - t0) * s_d_left_edge + t0 * s_d_right_edge
                d_left = d_left / max(norm(d_left), 1e-9)
                d_right = (1 - t1) * s_d_left_edge + t1 * s_d_right_edge
                d_right = d_right / max(norm(d_right), 1e-9)
                
                cross = d_left[0] * d_right[1] - d_left[1] * d_right[0]
                if cross >= 0:
                    perp_left = np.array([-d_left[1], d_left[0]])
                    perp_right = np.array([d_right[1], -d_right[0]])
                else:
                    perp_left = np.array([d_left[1], -d_left[0]])
                    perp_right = np.array([-d_right[1], d_right[0]])
                
                off_left_base = p_ap + perp_left * margin_side
                off_right_base = p_ap + perp_right * margin_side
                
                p_bl = off_left_base + d_left * margin_bottom
                p_br = off_right_base + d_right * margin_bottom
                
                d_avg = (d_left + d_right) / 2
                ref_vec = np.array([-d_avg[1], d_avg[0]]) if cross >= 0 else np.array([d_avg[1], -d_avg[0]])
                
                is_v = False
                if np.dot(p_br - p_bl, ref_vec) <= 0:
                    dx = off_right_base - off_left_base
                    det_s = d_left[0]*(-d_right[1]) - d_left[1]*(-d_right[0])
                    if abs(det_s) < 1e-12:
                        continue
                    t_cross = (dx[0]*(-d_right[1]) - dx[1]*(-d_right[0])) / det_s
                    p_bottom = off_left_base + t_cross * d_left
                    p_bl = p_bottom
                    p_br = p_bottom
                    is_v = True
                
                # Ray-curve intersection for top
                p_tl = self._ray_curve_intersect_preview(off_left_base, d_left, other_inner)
                p_tr = self._ray_curve_intersect_preview(off_right_base, d_right, other_inner)
                if p_tl is None or p_tr is None:
                    continue
                
                d_tl_r = p_tl - p_ap
                if norm(d_tl_r) > 1e-9:
                    p_tl = p_tl - (d_tl_r / norm(d_tl_r)) * margin_top
                d_tr_r = p_tr - p_ap
                if norm(d_tr_r) > 1e-9:
                    p_tr = p_tr - (d_tr_r / norm(d_tr_r)) * margin_top
                
                if np.dot(p_tr - p_tl, ref_vec) <= 0:
                    continue
                if np.dot(p_tl - p_bl, d_center) <= 0:
                    continue
                
                # Build polygon with corner rounding
                if is_v:
                    p_bottom = p_bl
                    if corner_pct > 1e-6:
                        side_r = norm(p_tr - p_bottom)
                        side_l = norm(p_tl - p_bottom)
                        top_side = norm(p_tl - p_tr)
                        dir_r = (p_tr - p_bottom) / max(side_r, 1e-9)
                        dir_l = (p_tl - p_bottom) / max(side_l, 1e-9)
                        dir_top = (p_tl - p_tr) / max(top_side, 1e-9)
                        cut_bot = min(side_r, side_l) * corner_pct * 0.5
                        cut_tr = min(side_r * corner_pct * 0.5, top_side * 0.4)
                        cut_tl = min(side_l * corner_pct * 0.5, top_side * 0.4)
                        
                        poly = []
                        # Bottom fillet
                        bot_l = p_bottom + dir_l * cut_bot
                        bot_r = p_bottom + dir_r * cut_bot
                        for fi in range(6):
                            t = fi / 5
                            poly.append(((1-t)**2 * bot_l + 2*(1-t)*t * p_bottom + t**2 * bot_r).tolist())
                        # Right side → top-right fillet
                        tr_from = p_tr - dir_r * cut_tr
                        tr_to = p_tr + dir_top * cut_tr
                        poly.append(tr_from.tolist())
                        for fi in range(6):
                            t = fi / 5
                            poly.append(((1-t)**2 * tr_from + 2*(1-t)*t * p_tr + t**2 * tr_to).tolist())
                        # Top side → top-left fillet
                        tl_from = p_tl - dir_top * cut_tl
                        tl_to = p_tl - dir_l * cut_tl
                        poly.append(tl_from.tolist())
                        for fi in range(6):
                            t = fi / 5
                            poly.append(((1-t)**2 * tl_from + 2*(1-t)*t * p_tl + t**2 * tl_to).tolist())
                        poly.append(poly[0])
                    else:
                        poly = [p_bl.tolist(), p_tr.tolist(), p_tl.tolist(), p_bl.tolist()]
                else:
                    # Normal quad with rounded corners
                    poly = self._round_quad_preview(p_bl, p_br, p_tr, p_tl, corner_pct)
                results.append(poly)
        
        return results
    
    def _round_quad_preview(self, p1, p2, p3, p4, radius_pct):
        """Create a rounded quadrilateral from 4 corners for preview."""
        from numpy.linalg import norm
        corners = [np.array(p1), np.array(p2), np.array(p3), np.array(p4)]
        n = len(corners)
        edge_lengths = sorted([norm(corners[(i+1) % n] - corners[i]) for i in range(n)])
        ref_len = (edge_lengths[-1] + edge_lengths[-2]) / 2
        radius = ref_len * radius_pct
        if radius < 1e-6:
            return [c.tolist() for c in corners] + [corners[0].tolist()]
        pts = []
        for i in range(n):
            prev_pt = corners[(i - 1) % n]
            curr_pt = corners[i]
            next_pt = corners[(i + 1) % n]
            d_in = curr_pt - prev_pt
            d_out = next_pt - curr_pt
            len_in, len_out = norm(d_in), norm(d_out)
            if len_in < 1e-9 or len_out < 1e-9:
                pts.append(curr_pt.tolist())
                continue
            offset = min(radius, len_in * 0.4, len_out * 0.4)
            p_start = curr_pt - d_in / len_in * offset
            p_end = curr_pt + d_out / len_out * offset
            for j in range(9):
                t = j / 8
                pt = (1-t)**2 * p_start + 2*(1-t)*t * curr_pt + t**2 * p_end
                pts.append(pt.tolist())
        pts.append(pts[0])
        return pts
    
    def _ray_curve_intersect_preview(self, ray_origin, ray_dir, curve):
        """Find where a ray intersects a PolyLine2D (for preview)."""
        from numpy.linalg import norm
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
                    t_ray = np.dot(pt - ray_origin, ray_dir)
                    if t_ray > -1e-9 and t_ray < best_t:
                        best = pt
                        best_t = t_ray
        return best

    def create_rounded_rectangle(self, width, height, corner_radius_ratio=0.25):
        # corner_radius_ratio is a ratio of min(width, height)
        radius = min(width, height) * corner_radius_ratio
        if radius > width / 2.0: radius = width / 2.0
        if radius > height / 2.0: radius = height / 2.0
        
        w = width / 2 - radius
        h = height / 2 - radius

        points = []
        # Top right corner
        for angle in np.linspace(0, np.pi/2, 10):
            points.append((w + radius * np.cos(angle), h + radius * np.sin(angle)))
        # Top left corner
        for angle in np.linspace(np.pi/2, np.pi, 10):
            points.append((-w + radius * np.cos(angle), h + radius * np.sin(angle)))
        # Bottom left corner
        for angle in np.linspace(np.pi, 3*np.pi/2, 10):
            points.append((-w + radius * np.cos(angle), -h + radius * np.sin(angle)))
        # Bottom right corner
        for angle in np.linspace(3*np.pi/2, 2*np.pi, 10):
            points.append((w + radius * np.cos(angle), -h + radius * np.sin(angle)))
            
        points.append(points[0]) # Close the loop

        return points

    @staticmethod
    def _round_quad_corners(p0, p1, p2, p3, radius_fraction):
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
        num_arc_segments = 4  # Points per rounded corner
        
        for i in range(n):
            prev_corner = corners[(i - 1) % n]
            curr_corner = corners[i]
            next_corner = corners[(i + 1) % n]
            
            # Vectors to neighboring corners
            to_prev = prev_corner - curr_corner
            to_next = next_corner - curr_corner
            
            len_prev = np.linalg.norm(to_prev)
            len_next = np.linalg.norm(to_next)
            
            if len_prev < 1e-9 or len_next < 1e-9:
                pts.append(curr_corner)
                continue
            
            # Clamp radius to half the shortest edge
            max_r = min(len_prev, len_next) * 0.5
            r = radius_fraction * max_r
            
            # Start and end of the rounded corner
            start_pt = curr_corner + (to_prev / len_prev) * r
            end_pt = curr_corner + (to_next / len_next) * r
            
            # Interpolate the arc
            for j in range(num_arc_segments + 1):
                t = j / num_arc_segments
                # Quadratic Bezier through start, corner, end
                pt = (1 - t)**2 * start_pt + 2 * (1 - t) * t * curr_corner + t**2 * end_pt
                pts.append(pt)
        
        pts.append(pts[0])  # Close the polygon
        return pts

    def update_form_from_glider_data(self):
        pg = self.parametric_glider
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        print(f"DEBUG: update_form - Current Tab: {'Suspended' if is_suspended else 'Non-Suspended'}")
        
        suffix = '_s' if is_suspended else '_ns'

        widgets_to_block = [self.holeShapeComboBox, self.numHolesSpinBox, self.holeWidthSpinBox,
                            self.holeHeightSpinBox, self.verticalShiftSpinBox,
                            self.minPosSpinBox, self.maxPosSpinBox,
                            self.holeMarginSpinBox, self.holeCornerRadiusSpinBox,
                            # Diagonal hole widgets (always loaded)
                            self.diagHolesEnabledCheckBox, self.diagHoleNumZonesSpinBox,
                            self.diagHoleMarginTopSpinBox, self.diagHoleMarginSideSpinBox,
                            self.diagHoleMarginBottomSpinBox, self.diagHoleCornerRadiusSpinBox]
        if is_suspended:
            widgets_to_block.extend([self.noHoleAngleSpinBox,
                                     self.coneHolesEnabledCheckBox,
                                     self.coneHoleNumZonesSpinBox,
                                     self.coneHoleMarginTopSpinBox,
                                     self.coneHoleMarginSideSpinBox,
                                     self.coneHoleMarginBottomSpinBox,
                                     self.coneHoleCornerRadiusSpinBox_cone])

        # Block signals to prevent feedback loops
        for widget in widgets_to_block:
            widget.blockSignals(True)

        self.holeShapeComboBox.setCurrentIndex(getattr(pg, f'hole_shape{suffix}', 0))
        self.numHolesSpinBox.setValue(getattr(pg, f'num_holes{suffix}', 30))
        self.holeWidthSpinBox.setValue(getattr(pg, f'hole_width{suffix}', 0.003))
        self.holeHeightSpinBox.setValue(getattr(pg, f'hole_height{suffix}', 0.8))
        self.verticalShiftSpinBox.setValue(getattr(pg, f'vertical_shift{suffix}', 0.0))
        self.minPosSpinBox.setValue(getattr(pg, f'min_hole_pos{suffix}', 0.2))
        self.maxPosSpinBox.setValue(getattr(pg, f'max_hole_pos{suffix}', 0.8))

        self.holeMarginSpinBox.setValue(getattr(pg, f'hole_margin{suffix}', 0.02) * 1000.0) # Convert m to mm for UI
        self.holeCornerRadiusSpinBox.setValue(getattr(pg, f'hole_corner_radius{suffix}', 0.25) * 100.0) # Convert ratio to % for UI

        if is_suspended:
            val = getattr(pg, 'susp_hole_radius_top_s', 'MISSING')
            print(f"DEBUG: update_form - Reading susp_hole_radius_top_s: {val} (Type: {type(val)})")
            
            self.noHoleAngleSpinBox.setValue(getattr(pg, 'hole_free_angle_s', 30.0))
            self.noHoleArcAngleSpinBox.setValue(getattr(pg, 'hole_arc_span_s', 120.0))
            
            # Cone hole parameters
            self.coneHolesEnabledCheckBox.setChecked(getattr(pg, 'cone_holes_enabled_s', False))
            self.coneHoleNumZonesSpinBox.setValue(getattr(pg, 'cone_hole_num_zones_s', 1))
            self.coneHoleMarginTopSpinBox.setValue(getattr(pg, 'cone_hole_margin_top_s', 3.0))
            self.coneHoleMarginSideSpinBox.setValue(getattr(pg, 'cone_hole_margin_side_s', 3.0))
            self.coneHoleMarginBottomSpinBox.setValue(getattr(pg, 'cone_hole_margin_bottom_s', 3.0))
            self.coneHoleCornerRadiusSpinBox_cone.setValue(getattr(pg, 'cone_hole_corner_radius_s', 25.0))

        # Diagonal hole parameters (always loaded, with backward compat)
        diag_enabled = getattr(pg, 'diag_holes_enabled', None)
        if diag_enabled is None:
            diag_enabled = getattr(pg, 'cone_holes_enabled_s', False)
        self.diagHolesEnabledCheckBox.setChecked(diag_enabled)
        self.diagHoleNumZonesSpinBox.setValue(getattr(pg, 'diag_hole_num_zones',
                                             getattr(pg, 'cone_hole_num_zones_s', 1)))
        self.diagHoleMarginTopSpinBox.setValue(getattr(pg, 'diag_hole_margin_top',
                                              getattr(pg, 'cone_hole_margin_top_s', 3.0)))
        self.diagHoleMarginSideSpinBox.setValue(getattr(pg, 'diag_hole_margin_side',
                                               getattr(pg, 'cone_hole_margin_side_s', 3.0)))
        self.diagHoleMarginBottomSpinBox.setValue(getattr(pg, 'diag_hole_margin_bottom',
                                                 getattr(pg, 'cone_hole_margin_bottom_s', 3.0)))
        self.diagHoleCornerRadiusSpinBox.setValue(getattr(pg, 'diag_hole_corner_radius',
                                                 getattr(pg, 'cone_hole_corner_radius_s', 25.0)))

        # Initial visibility update
        self.on_height_mode_change(0)  # Default mode

        # Unblock signals
        for widget in widgets_to_block:
            widget.blockSignals(False)

    def update_glider_data(self, is_suspended):
        pg = self.parametric_glider
        if is_suspended:
            pg.hole_shape_s = self.holeShapeComboBox.currentIndex()
            pg.num_holes_s = self.numHolesSpinBox.value()
            pg.hole_width_s = self.holeWidthSpinBox.value()
            pg.hole_height_s = self.holeHeightSpinBox.value()
            pg.vertical_shift_s = self.verticalShiftSpinBox.value()
            pg.min_hole_pos_s = self.minPosSpinBox.value()
            pg.max_hole_pos_s = self.maxPosSpinBox.value()
            pg.hole_free_angle_s = self.noHoleAngleSpinBox.value()
            pg.hole_arc_span_s = self.noHoleArcAngleSpinBox.value()
            pg.hole_height_mode_s = 1  # Margin mode (fixed mm margin instead of percent)
            pg.hole_margin_s = self.holeMarginSpinBox.value() / 1000.0 # Convert mm to m for storage
            pg.hole_corner_radius_s = self.holeCornerRadiusSpinBox.value() / 100.0 # Convert % to ratio
            # Cone hole params
            pg.cone_holes_enabled_s = self.coneHolesEnabledCheckBox.isChecked()
            pg.cone_hole_num_zones_s = self.coneHoleNumZonesSpinBox.value()
            pg.cone_hole_margin_top_s = self.coneHoleMarginTopSpinBox.value()
            pg.cone_hole_margin_side_s = self.coneHoleMarginSideSpinBox.value()
            pg.cone_hole_margin_bottom_s = self.coneHoleMarginBottomSpinBox.value()
            pg.cone_hole_corner_radius_s = self.coneHoleCornerRadiusSpinBox_cone.value()

        # Diagonal hole params (always saved)
        pg.diag_holes_enabled = self.diagHolesEnabledCheckBox.isChecked()
        pg.diag_hole_num_zones = self.diagHoleNumZonesSpinBox.value()
        pg.diag_hole_margin_top = self.diagHoleMarginTopSpinBox.value()
        pg.diag_hole_margin_side = self.diagHoleMarginSideSpinBox.value()
        pg.diag_hole_margin_bottom = self.diagHoleMarginBottomSpinBox.value()
        pg.diag_hole_corner_radius = self.diagHoleCornerRadiusSpinBox.value()

        if not is_suspended:
            pg.hole_shape_ns = self.holeShapeComboBox.currentIndex()
            pg.num_holes_ns = self.numHolesSpinBox.value()
            pg.hole_width_ns = self.holeWidthSpinBox.value()
            pg.hole_height_ns = self.holeHeightSpinBox.value()
            pg.vertical_shift_ns = self.verticalShiftSpinBox.value()
            pg.min_hole_pos_ns = self.minPosSpinBox.value()
            pg.max_hole_pos_ns = self.maxPosSpinBox.value()
            pg.hole_height_mode_ns = 1  # Margin mode (fixed mm margin instead of percent)
            pg.hole_margin_ns = self.holeMarginSpinBox.value() / 1000.0 # Convert mm to m for storage
            pg.hole_corner_radius_ns = self.holeCornerRadiusSpinBox.value() / 100.0 # Convert % to ratio

    def update_glider_data_and_preview(self, *args, switch=False):
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        self.update_glider_data(is_suspended)
        self.update_preview()

    def accept(self):
        # When accepting, save the data from the currently visible tab.
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        self.update_glider_data(is_suspended)
        self.update_view_glider()
        super(HoleDesignTool, self).accept()
