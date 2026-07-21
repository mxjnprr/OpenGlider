"""
Airfoil Structure Tool for OpenGlider FreeCAD workbench.

This tool allows configuration of structural elements on rib profiles:
- Rod sleeves (fourreaux de joncs) for extrados and intrados (multiple per surface)
- Attachment reinforcements with half-moon load distribution rods (suspended ribs only)
"""


import FreeCADGui as Gui
import numpy as np
from pivy import coin
from PySide import QtCore, QtGui

from openglider.glider.rib import AttachmentReinforcement, RodSleeve

from .tools import BaseTool, Line_old


class RodSleeveConfigWidget(QtGui.QWidget):
    """Widget for configuring a single rod sleeve's parameters."""
    changed = QtCore.Signal()
    
    def __init__(self, surface='extrados', parent=None):
        super().__init__(parent)
        self.surface = surface
        self.layout = QtGui.QFormLayout(self)
        self.layout.setContentsMargins(5, 5, 5, 5)
        
        # Start/End position
        self.startSpinBox = QtGui.QDoubleSpinBox()
        self.startSpinBox.setSingleStep(1.0)
        self.startSpinBox.setDecimals(1)
        self.startSpinBox.setSuffix(" %")
        # Negative = the sleeve crosses the leading edge and runs onto the
        # opposite surface (e.g. extrados rod descending onto the intrados).
        self.startSpinBox.setRange(-40.0, 100.0)
        self.startSpinBox.setValue(0.0)
        self.startSpinBox.setToolTip(
            "Start position along the chord.\n"
            "Negative values cross the leading edge onto the opposite surface "
            "(rod wraps past the nose, follows the real profile contour)."
        )
        self.layout.addRow("Start (% chord)", self.startSpinBox)
        
        self.endSpinBox = QtGui.QDoubleSpinBox()
        self.endSpinBox.setSingleStep(1.0)
        self.endSpinBox.setDecimals(1)
        self.endSpinBox.setSuffix(" %")
        self.endSpinBox.setRange(0.0, 100.0)
        self.endSpinBox.setValue(70.0)
        self.layout.addRow("End (% chord)", self.endSpinBox)
        
        # Width and offset
        self.widthSpinBox = QtGui.QDoubleSpinBox()
        self.widthSpinBox.setSingleStep(1.0)
        self.widthSpinBox.setDecimals(1)
        self.widthSpinBox.setSuffix(" mm")
        self.widthSpinBox.setRange(1.0, 50.0)
        self.widthSpinBox.setValue(15.0)
        self.layout.addRow("Width", self.widthSpinBox)
        
        self.offsetSpinBox = QtGui.QDoubleSpinBox()
        self.offsetSpinBox.setSingleStep(0.5)
        self.offsetSpinBox.setDecimals(1)
        self.offsetSpinBox.setSuffix(" mm")
        self.offsetSpinBox.setRange(0.0, 20.0)
        self.offsetSpinBox.setValue(5.0)
        self.layout.addRow("Offset", self.offsetSpinBox)
        
        # Start termination (formerly "Leading Edge")
        self.layout.addRow(QtGui.QLabel("<b>Start Termination</b>"))
        self.startCurlSpinBox = QtGui.QDoubleSpinBox()
        self.startCurlSpinBox.setSingleStep(5.0)
        self.startCurlSpinBox.setDecimals(0)
        self.startCurlSpinBox.setSuffix(" °")
        self.startCurlSpinBox.setRange(0.0, 180.0)
        self.startCurlSpinBox.setValue(60.0)
        self.startCurlSpinBox.setToolTip(
            "Curl: how much the termination tightens the curvature toward the "
            "profile (keeps the rod pre-stressed). 0 = straight continuation."
        )
        self.layout.addRow("Curl", self.startCurlSpinBox)

        self.startLengthSpinBox = QtGui.QDoubleSpinBox()
        self.startLengthSpinBox.setSingleStep(1.0)
        self.startLengthSpinBox.setDecimals(1)
        self.startLengthSpinBox.setSuffix(" %")
        self.startLengthSpinBox.setRange(1.0, 50.0)
        self.startLengthSpinBox.setValue(8.0)
        self.layout.addRow("Length (% chord)", self.startLengthSpinBox)
        
        # End termination (formerly "Trailing Edge")
        self.layout.addRow(QtGui.QLabel("<b>End Termination</b>"))
        self.endCurlSpinBox = QtGui.QDoubleSpinBox()
        self.endCurlSpinBox.setSingleStep(5.0)
        self.endCurlSpinBox.setDecimals(0)
        self.endCurlSpinBox.setSuffix(" °")
        self.endCurlSpinBox.setRange(0.0, 180.0)
        self.endCurlSpinBox.setValue(60.0)
        self.endCurlSpinBox.setToolTip(
            "Curl: how much the termination tightens the curvature toward the "
            "profile (keeps the rod pre-stressed). 0 = straight continuation."
        )
        self.layout.addRow("Curl", self.endCurlSpinBox)

        self.endLengthSpinBox = QtGui.QDoubleSpinBox()
        self.endLengthSpinBox.setSingleStep(1.0)
        self.endLengthSpinBox.setDecimals(1)
        self.endLengthSpinBox.setSuffix(" %")
        self.endLengthSpinBox.setRange(1.0, 50.0)
        self.endLengthSpinBox.setValue(6.0)
        self.layout.addRow("Length (% chord)", self.endLengthSpinBox)
        
        # Excluded ribs field
        self.layout.addRow(QtGui.QLabel("<b>Exclusions</b>"))
        self.excludedRibsEdit = QtGui.QLineEdit()
        self.excludedRibsEdit.setPlaceholderText("e.g. 1, 3, 5 (rib numbers to exclude)")
        self.layout.addRow("Excluded Ribs", self.excludedRibsEdit)
        
        # Connect signals
        self.startSpinBox.valueChanged.connect(self.emit_changed)
        self.endSpinBox.valueChanged.connect(self.emit_changed)
        self.widthSpinBox.valueChanged.connect(self.emit_changed)
        self.offsetSpinBox.valueChanged.connect(self.emit_changed)
        self.startCurlSpinBox.valueChanged.connect(self.emit_changed)
        self.startLengthSpinBox.valueChanged.connect(self.emit_changed)
        self.endCurlSpinBox.valueChanged.connect(self.emit_changed)
        self.endLengthSpinBox.valueChanged.connect(self.emit_changed)
        self.excludedRibsEdit.textChanged.connect(self.emit_changed)
    
    def emit_changed(self):
        self.changed.emit()
    
    def get_excluded_ribs(self):
        """Parse excluded ribs from text field. Returns list of 0-based indices."""
        text = self.excludedRibsEdit.text().strip()
        if not text:
            return []
        try:
            # Parse comma-separated rib numbers (1-based from user, convert to 0-based)
            return [int(x.strip()) - 1 for x in text.split(',') if x.strip().isdigit()]
        except:
            return []
    
    def get_values(self):
        return {
            'start_chord': self.startSpinBox.value() / 100.0,
            'end_chord': self.endSpinBox.value() / 100.0,
            'width': self.widthSpinBox.value() / 1000.0,
            'offset': self.offsetSpinBox.value() / 1000.0,
            'start_curl': self.startCurlSpinBox.value(),
            'start_length': self.startLengthSpinBox.value() / 100.0,
            'end_curl': self.endCurlSpinBox.value(),
            'end_length': self.endLengthSpinBox.value() / 100.0,
            'excluded_ribs': self.get_excluded_ribs(),
        }
    
    def set_values(self, config):
        if not config:
            return
        self.startSpinBox.setValue(config.get('start_chord', 0.0) * 100.0)
        self.endSpinBox.setValue(config.get('end_chord', 0.7) * 100.0)
        self.widthSpinBox.setValue(config.get('width', 0.015) * 1000.0)
        self.offsetSpinBox.setValue(config.get('offset', 0.005) * 1000.0)
        self.startCurlSpinBox.setValue(config.get('start_curl', 60.0))
        self.startLengthSpinBox.setValue(config.get('start_length', 0.08) * 100.0)
        self.endCurlSpinBox.setValue(config.get('end_curl', 60.0))
        self.endLengthSpinBox.setValue(config.get('end_length', 0.06) * 100.0)
        # Load excluded ribs (convert 0-based to 1-based for display)
        excluded = config.get('excluded_ribs', [])
        if excluded:
            self.excludedRibsEdit.setText(', '.join(str(x + 1) for x in excluded))
        else:
            self.excludedRibsEdit.clear()
    
    def create_rod_sleeve(self):
        """Create a RodSleeve object from this widget's values."""
        values = self.get_values()
        return RodSleeve(
            surface=self.surface,
            width=values['width'],
            offset=values['offset'],
            start_chord=values['start_chord'],
            end_chord=values['end_chord'],
            le_curl=values['start_curl'],
            te_curl=values['end_curl'],
            le_length=values['start_length'],
            te_length=values['end_length'],
        )


class SurfaceRodSleeveGroup(QtGui.QGroupBox):
    """Group box for managing multiple rod sleeves on one surface."""
    changed = QtCore.Signal()
    
    def __init__(self, surface='extrados', title="Extrados Rod Sleeves", parent=None):
        super().__init__(title, parent)
        self.surface = surface
        self.rod_widgets = []
        
        main_layout = QtGui.QVBoxLayout(self)
        
        # Enable checkbox
        self.enabledCheckBox = QtGui.QCheckBox("Enable")
        self.enabledCheckBox.setChecked(True)
        main_layout.addWidget(self.enabledCheckBox)
        
        # Tabs for multiple rods
        self.tabWidget = QtGui.QTabWidget()
        self.tabWidget.setTabsClosable(True)
        self.tabWidget.tabCloseRequested.connect(self.remove_rod)
        main_layout.addWidget(self.tabWidget)
        
        # Add button
        button_layout = QtGui.QHBoxLayout()
        self.addButton = QtGui.QPushButton("+ Add Rod")
        self.addButton.clicked.connect(self.add_rod)
        button_layout.addWidget(self.addButton)
        button_layout.addStretch()
        main_layout.addLayout(button_layout)
        
        # Connect enable checkbox
        self.enabledCheckBox.stateChanged.connect(self.emit_changed)
        
        # Add one rod by default
        self.add_rod()
    
    def emit_changed(self):
        self.changed.emit()
    
    def add_rod(self):
        """Add a new rod sleeve tab."""
        widget = RodSleeveConfigWidget(surface=self.surface)
        widget.changed.connect(self.emit_changed)
        
        rod_num = len(self.rod_widgets) + 1
        self.rod_widgets.append(widget)
        self.tabWidget.addTab(widget, f"Rod {rod_num}")
        self.emit_changed()
    
    def remove_rod(self, index):
        """Remove a rod sleeve tab."""
        if len(self.rod_widgets) <= 1:
            return  # Keep at least one rod
        
        widget = self.rod_widgets.pop(index)
        self.tabWidget.removeTab(index)
        widget.deleteLater()
        
        # Renumber tabs
        for i, w in enumerate(self.rod_widgets):
            self.tabWidget.setTabText(i, f"Rod {i + 1}")
        
        self.emit_changed()
    
    def is_enabled(self):
        return self.enabledCheckBox.isChecked()
    
    def get_rod_sleeves(self):
        """Get list of RodSleeve objects for all enabled rods."""
        if not self.is_enabled():
            return []
        return [w.create_rod_sleeve() for w in self.rod_widgets]
    
    def get_configs(self):
        """Get list of config dicts for all rods."""
        return [w.get_values() for w in self.rod_widgets]
    
    def set_configs(self, enabled, configs):
        """Set the widget state from a list of configs."""
        self.enabledCheckBox.setChecked(enabled)
        
        # Clear existing tabs (except first)
        while len(self.rod_widgets) > 1:
            self.remove_rod(len(self.rod_widgets) - 1)
        
        if not configs:
            return
        
        # Set first rod's values
        if len(configs) > 0 and len(self.rod_widgets) > 0:
            self.rod_widgets[0].set_values(configs[0])
        
        # Add additional rods
        for i, config in enumerate(configs[1:], start=1):
            self.add_rod()
            self.rod_widgets[i].set_values(config)


class AirfoilStructureTool(BaseTool):
    widget_name = "Airfoil Structure"

    def __init__(self, obj):
        super().__init__(obj)

        # Profile type selector
        self.ribTypeComboBox = QtGui.QComboBox(self.base_widget)
        
        # === Extrados Sleeve Group (multiple rods) ===
        self.extradosGroup = SurfaceRodSleeveGroup(
            surface='extrados', 
            title="Extrados Rod Sleeves",
            parent=self.base_widget
        )
        
        # === Intrados Sleeve Group (multiple rods) ===
        self.intradosGroup = SurfaceRodSleeveGroup(
            surface='intrados',
            title="Intrados Rod Sleeves", 
            parent=self.base_widget
        )
        
        # === Attachment Reinforcement Group (only for suspended) ===
        self.reinforcementGroupBox = QtGui.QGroupBox("Attachment Reinforcements", self.base_widget)
        self.reinforcementLayout = QtGui.QFormLayout(self.reinforcementGroupBox)
        
        self.reinforcementEnabledCheckBox = QtGui.QCheckBox("Enable Reinforcements", self.reinforcementGroupBox)
        self.reinforcementEnabledCheckBox.setChecked(True)  # Enabled by default
        self.reinforcementApplyAllCheckBox = QtGui.QCheckBox("Apply same parameters to all", self.reinforcementGroupBox)
        self.reinforcementApplyAllCheckBox.setChecked(False)  # Individual config by default
        
        self.reinforcementStack = QtGui.QStackedWidget(self.reinforcementGroupBox)
        
        # Excluded ribs for reinforcements
        self.reinforcementExcludedRibsEdit = QtGui.QLineEdit(self.reinforcementGroupBox)
        self.reinforcementExcludedRibsEdit.setPlaceholderText("e.g. 1, 3, 5 (rib numbers to exclude from reinforcements)")
        
        # Master Config
        self.masterConfig = ReinforcementConfigWidget(self.reinforcementGroupBox)
        self.reinforcementStack.addWidget(self.masterConfig)
        
        # Tabbed Config
        self.reinforcementTabs = QtGui.QTabWidget(self.reinforcementGroupBox)
        self.reinforcementStack.addWidget(self.reinforcementTabs)
        self.reinforcement_widgets = []

        # Shark-nose config (applies to the front-most attachment point only)
        self.sharkNoseLabel = QtGui.QLabel("— Shark nose —", self.reinforcementGroupBox)
        self.sharkConfig = SharkNoseConfigWidget(self.reinforcementGroupBox)

        self.applyButton = QtGui.QPushButton("Apply", self.base_widget)

        # Preview rib selector - use ComboBox to show actual rib names
        self.previewRibLabel = QtGui.QLabel("Preview Rib:")
        self.previewRibComboBox = QtGui.QComboBox(self.base_widget)
        self._populate_rib_combo()

        self.preview_root = coin.SoSeparator()
        self.setup_widget()
        self.setup_pivy()

    def setup_widget(self):
        self.ribTypeComboBox.addItems(["Non-Suspended", "Suspended"])
        self.layout.addRow("Profile Type", self.ribTypeComboBox)
        
        # Add rod sleeve groups
        self.layout.addRow(self.extradosGroup)
        self.layout.addRow(self.intradosGroup)
        
        # --- Configure Attachment Reinforcements ---
        self.reinforcementLayout.addRow(self.reinforcementEnabledCheckBox)
        self.reinforcementLayout.addRow(self.reinforcementApplyAllCheckBox)
        self.reinforcementLayout.addRow("Excluded Ribs", self.reinforcementExcludedRibsEdit)
        self.reinforcementLayout.addRow(self.reinforcementStack)
        self.reinforcementLayout.addRow(self.sharkNoseLabel)
        self.reinforcementLayout.addRow(self.sharkConfig)

        self.layout.addRow(self.reinforcementGroupBox)
        
        # Preview selector
        preview_layout = QtGui.QHBoxLayout()
        preview_layout.addWidget(self.previewRibLabel)
        preview_layout.addWidget(self.previewRibComboBox)
        preview_layout.addStretch()
        self.layout.addRow(preview_layout)
        
        # Apply button
        button_layout = QtGui.QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(self.applyButton)
        self.layout.addRow(button_layout)

        # Load initial values
        self.update_form_from_glider_data()

        # Connections - Rod sleeves
        self.extradosGroup.changed.connect(self.update_preview)
        self.intradosGroup.changed.connect(self.update_preview)
        
        # Connections - Reinforcements
        self.reinforcementEnabledCheckBox.stateChanged.connect(self.update_preview)
        self.reinforcementApplyAllCheckBox.stateChanged.connect(self.on_reinforcement_mode_change)
        self.reinforcementApplyAllCheckBox.stateChanged.connect(self.update_preview)
        # Master config signals are connected in ReinforcementConfigWidget
        self.masterConfig.changed.connect(self.update_preview)
        self.sharkConfig.changed.connect(self.update_preview)
        
        self.ribTypeComboBox.currentIndexChanged.connect(self.on_rib_type_change)
        self.previewRibComboBox.currentIndexChanged.connect(self.update_preview)
        self.applyButton.clicked.connect(self.accept)

        # Set initial visibility of reinforcement group (only for suspended)
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        self.reinforcementGroupBox.setVisible(is_suspended)
        
        # Always initialize reinforcement tabs (even if Non-Suspended is selected)
        # so they're ready when switching to Suspended
        rib = self.get_first_suspended_rib()
        if rib:
            valid_aps = self.get_valid_attachment_points(rib)
            self.update_reinforcement_tabs(valid_aps)
        
        # Ensure correct stack widget is shown
        self.on_reinforcement_mode_change(None)

    def setup_pivy(self):
        self.task_separator.addChild(self.preview_root)
        self.update_preview()
        Gui.SendMsgToActiveView("ViewFit")

    def get_num_ribs(self):
        """Get total number of ribs."""
        try:
            glider_instance = self.obj.Proxy.getGliderInstance()
            return len(glider_instance.ribs)
        except:
            return 1
    
    def _populate_rib_combo(self):
        """Populate the rib combo box with actual rib names."""
        self.previewRibComboBox.clear()
        try:
            glider_instance = self.obj.Proxy.getGliderInstance()
            for rib in glider_instance.ribs:
                # Use rib.name if available, otherwise use index+1
                name = rib.name if hasattr(rib, 'name') and rib.name else f"r{glider_instance.ribs.index(rib) + 1}"
                self.previewRibComboBox.addItem(name)
        except Exception:
            self.previewRibComboBox.addItem("r1")

    def get_representative_rib(self, suspended=False):
        """Get rib for preview based on spinner selection."""
        glider_instance = self.obj.Proxy.getGliderInstance()
        rib_idx = self.previewRibComboBox.currentIndex()
        
        if rib_idx < len(glider_instance.ribs):
            return glider_instance.ribs[rib_idx]
        return glider_instance.ribs[0] if glider_instance.ribs else None

    def get_first_suspended_rib(self):
        """Get the first suspended rib that has valid attachment points (< 90% chord)."""
        glider_instance = self.obj.Proxy.getGliderInstance()

        for rib in glider_instance.ribs:
            # Robust matcher (see apply_rod_sleeves_to_ribs): an identity-only
            # set over attachment-point .rib objects can miss every rib.
            all_aps = glider_instance.get_rib_attachment_points(rib)
            valid_aps = [ap for ap in all_aps if ap.rib_pos <= 0.90]
            if valid_aps:
                return rib

        return glider_instance.ribs[0] if glider_instance.ribs else None
    
    def get_valid_attachment_points(self, rib):
        """Get attachment points < 90% chord (excluding brake attachments)."""
        glider_instance = self.obj.Proxy.getGliderInstance()
        all_aps = glider_instance.get_rib_attachment_points(rib)
        # Filter out brake attachments (> 90% chord)
        valid_aps = [ap for ap in all_aps if ap.rib_pos <= 0.90]
        # Sort by position
        valid_aps.sort(key=lambda x: x.rib_pos)
        return valid_aps

    def get_first_suspended_rib_index(self):
        """Get the index of the first suspended rib with valid attachment points."""
        glider_instance = self.obj.Proxy.getGliderInstance()

        for idx, rib in enumerate(glider_instance.ribs):
            # Robust matcher (see apply_rod_sleeves_to_ribs): an identity-only
            # set over attachment-point .rib objects can miss every rib.
            all_aps = glider_instance.get_rib_attachment_points(rib)
            valid_aps = [ap for ap in all_aps if ap.rib_pos <= 0.90]
            if valid_aps:
                return idx
        return 0

    def on_rib_type_change(self, new_index):
        previous_index = 1 - new_index
        # Save data for previous state
        self.update_glider_data(is_suspended=previous_index == 1)
        
        is_suspended = new_index == 1
        self.reinforcementGroupBox.setVisible(is_suspended)
        
        # Auto-switch preview rib to first suspended rib when switching to Suspended mode
        if is_suspended:
            first_suspended_idx = self.get_first_suspended_rib_index()
            self.previewRibComboBox.setCurrentIndex(first_suspended_idx)
        
        # Reload data for new state
        self.update_form_from_glider_data()
        self.update_preview(force=True)

    def on_reinforcement_mode_change(self, state):
        """Switch between Master and Tabbed view."""
        if self.reinforcementApplyAllCheckBox.isChecked():
            self.reinforcementStack.setCurrentWidget(self.masterConfig)
        else:
            self.reinforcementStack.setCurrentWidget(self.reinforcementTabs)

    def update_reinforcement_tabs(self, valid_aps):
        """Rebuild tabs based on current valid attachment points."""
        self.reinforcementTabs.clear()
        self.reinforcement_widgets = []
        
        for i, ap in enumerate(valid_aps):
            label = f"AP {i + 1} ({ap.rib_pos * 100:.1f}%)"
            widget = ReinforcementConfigWidget()
            widget.changed.connect(self.update_preview)
            self.reinforcementTabs.addTab(widget, label)
            self.reinforcement_widgets.append(widget)

    def update_preview(self, *args, **kwargs):
        self.preview_root.removeAllChildren()

        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        rib = self.get_representative_rib(suspended=is_suspended)
        # Reference chord for the mm<->% (proportional) conversions.
        if rib is not None:
            ref_c = getattr(rib, 'chord', 2.5)
            self.sharkConfig.ref_chord = ref_c
            self.masterConfig.ref_chord = ref_c
            for w in self.reinforcement_widgets:
                w.ref_chord = ref_c
        if not rib:
            return

        glider_instance = self.obj.Proxy.getGliderInstance()

        # Draw profile outline - scaled by chord
        if hasattr(rib, 'get_hull') and glider_instance is not None:
            try:
                hull_profile = rib.get_hull(glider_instance)
            except Exception:
                hull_profile = rib.profile_2d
            profile_points = [p * rib.chord for p in hull_profile.data]
        else:
            profile_points = [p * rib.chord for p in rib.profile_2d.data]
            
        profile_3d = [[p[0], p[1], 0] for p in profile_points]
        self.preview_root.addChild(Line_old(profile_3d + [profile_3d[0]], width=2).object)

        # Air intake location: two ticks marking the folded intrados cuts
        self._draw_air_intake_marks(rib, glider_instance)

        # Visualizing attachment points (red dots) if suspended
        valid_aps = []
        if is_suspended:
            valid_aps = self.get_valid_attachment_points(rib)
            for ap in valid_aps:
                self._draw_attachment_point_marker(rib, ap.rib_pos)

        # Draw extrados sleeves
        for sleeve in self.extradosGroup.get_rod_sleeves():
            self._draw_sleeve(sleeve, rib, color='blue', glider=glider_instance)

        # Draw intrados sleeves
        for sleeve in self.intradosGroup.get_rod_sleeves():
            self._draw_sleeve(sleeve, rib, color='green', glider=glider_instance)

        # Draw attachment reinforcements if enabled and suspended
        if is_suspended and self.reinforcementEnabledCheckBox.isChecked():
            apply_all = self.reinforcementApplyAllCheckBox.isChecked()
            shark_cfg = self.sharkConfig.get_values()

            for i, ap in enumerate(valid_aps):
                # Get config
                if apply_all:
                    config = self.masterConfig.get_values()
                else:
                    # Individual config
                    if i < len(self.reinforcement_widgets):
                        config = self.reinforcement_widgets[i].get_values()
                    else:
                        continue # Should not happen if tabs aligned with valid_aps

                # Front-most attachment point: the half-moon is prolonged into a
                # merged shark-nose piece when enabled (keeps the half-moon).
                if i == 0 and shark_cfg.get('enabled'):
                    reinforcement, _, _ = self._create_shark_reinforcement(
                        rib, glider_instance, shark_cfg, ap.rib_pos,
                        self.extradosGroup.get_configs(),
                        self.intradosGroup.get_configs(),
                        config)
                    self._draw_reinforcement(reinforcement, rib, glider=glider_instance)
                    continue

                if config['enabled']:
                    reinforcement = self._create_reinforcement(ap.rib_pos, config)
                    self._draw_reinforcement(reinforcement, rib, glider=glider_instance)

    def _create_reinforcement(self, position, config, name=""):
        """Create an AttachmentReinforcement from config values."""
        return AttachmentReinforcement(
            position=position,
            surface_offset=config.get('surface_offset', 0.003),
            halfmoon_radius=config['halfmoon_radius'],
            rod_enabled=config.get('rod_enabled', True),
            rod_offset=config['rod_offset'],
            rod_width=config['rod_width'],
            rod_end_offset=config.get('rod_end_offset', 10.0),
            name=name,
            relative=config.get('relative', False),
            corner_radius=config.get('corner_radius', 0.0),
        )


    def _draw_sleeve(self, sleeve, rib, color='blue', glider=None):
        """Draw a rod sleeve preview with terminations."""
        try:
            # Get full sleeve with terminations
            inner_points, outer_points = sleeve.get_full_sleeve_points(rib, glider=glider)
            
            if inner_points and outer_points:
                # Draw inner edge
                inner_3d = [[p[0], p[1], 0] for p in inner_points]
                self.preview_root.addChild(Line_old(inner_3d, color=color, width=2).object)
                
                # Draw outer edge
                outer_3d = [[p[0], p[1], 0] for p in outer_points]
                self.preview_root.addChild(Line_old(outer_3d, color=color, width=2).object)
                
                # Draw end caps connecting inner and outer
                if len(inner_points) > 0 and len(outer_points) > 0:
                    # Start cap
                    start_cap = [[inner_points[0][0], inner_points[0][1], 0],
                                 [outer_points[0][0], outer_points[0][1], 0]]
                    self.preview_root.addChild(Line_old(start_cap, color=color, width=1).object)
                    
                    # End cap
                    end_cap = [[inner_points[-1][0], inner_points[-1][1], 0],
                               [outer_points[-1][0], outer_points[-1][1], 0]]
                    self.preview_root.addChild(Line_old(end_cap, color=color, width=1).object)
        except Exception as e:
            print(f"Error drawing sleeve: {e}")

    def _draw_attachment_point_marker(self, rib, position):
        """Draw a red marker at the attachment point position."""
        profile = rib.profile_2d
        # Get point from profile coordinate system
        idx = profile(position)
        center_point = profile[idx] * rib.chord
        
        # Create a small diamond marker
        size = 0.005  # 5mm visual size
        center_3d = np.array([center_point[0], center_point[1], 0])
        
        marker_points = [
            center_3d + np.array([size, 0, 0]),
            center_3d + np.array([0, size, 0]),
            center_3d + np.array([-size, 0, 0]),
            center_3d + np.array([0, -size, 0]),
            center_3d + np.array([size, 0, 0])
        ]
        
        self.preview_root.addChild(Line_old(marker_points, color='red', width=3).object)

    def _draw_air_intake_marks(self, rib, glider):
        """Mark the air intake with two ticks at the folded intrados cuts.

        The air-intake mouth is defined by the cell panels whose cut type is
        'folded' (see Panel.CUT_TYPES). Their chord positions are drawn as short
        lines crossing the profile so rod sleeves can be placed relative to the
        opening.
        """
        if glider is None:
            return
        try:
            for x in self._get_air_intake_positions(rib, glider):
                self._draw_intake_tick(rib, x)
        except Exception as e:
            print(f"Error drawing air intake marks: {e}")

    def _get_air_intake_positions(self, rib, glider):
        """Return the chord positions of the air-intake mouth (folded cuts).

        A rib borders up to two cells; the intake edges may come from either.
        For each, use the side value that belongs to this rib:
          - right cell (this rib is rib1) -> 'left'
          - left cell  (this rib is rib2) -> 'right'
        """
        positions = []
        if glider is None:
            return positions
        rib_idx = next((i for i, r in enumerate(glider.ribs) if r is rib), None)
        if rib_idx is None:
            return positions

        cells = glider.cells
        neighbours = []
        if rib_idx < len(cells):
            neighbours.append((cells[rib_idx], 'left'))
        if 0 <= rib_idx - 1 < len(cells):
            neighbours.append((cells[rib_idx - 1], 'right'))

        for cell, side in neighbours:
            for panel in cell.panels:
                for cut in (panel.cut_front, panel.cut_back):
                    if cut.get("type") == "folded":
                        x = cut[side]
                        if not any(abs(x - u) < 1e-4 for u in positions):
                            positions.append(x)
        return positions

    def _create_shark_reinforcement(self, rib, glider, shark_cfg, ap_pos,
                                    extrados_configs, intrados_configs, base_config):
        """Build the shark-nose AttachmentReinforcement (intrados rounded box).

        Inherits the half-moon parameters (radius, surface offset, rod sleeve)
        from ``base_config`` (that AP's normal reinforcement config) so the rod
        sleeve is kept inside the box.
        """
        base_config = base_config or {}
        start = shark_cfg.get('start', 0.03)
        end = shark_cfg.get('end', 0.15)

        return AttachmentReinforcement(
            position=ap_pos,
            surface_offset=base_config.get('surface_offset', 0.0005),
            halfmoon_radius=base_config.get('halfmoon_radius', 0.1),
            rod_enabled=shark_cfg.get('rod', True),
            rod_offset=base_config.get('rod_offset', 0.008),
            rod_width=base_config.get('rod_width', 0.009),
            rod_end_offset=base_config.get('rod_end_offset', 1.0),
            name="",
            shark_nose=True,
            shark_start=start,
            shark_end=end,
            shark_depth=shark_cfg.get('depth', 0.035),
            shark_start_angle=shark_cfg.get('start_angle', 90.0),
            shark_end_angle=shark_cfg.get('end_angle', 90.0),
            shark_corner_radius=shark_cfg.get('corner_radius', 0.5),
            shark_depth_relative=shark_cfg.get('depth_relative', False),
        ), start, end

    def _draw_intake_tick(self, rib, x):
        """Draw one air-intake tick perpendicular to the profile at chord pos x."""
        profile = rib.profile_2d
        ik = profile(x)
        pt = np.array(profile[ik]) * rib.chord

        # Local tangent from the two bounding profile points, then rotate 90 deg.
        i0 = int(np.floor(ik))
        i1 = min(i0 + 1, len(profile) - 1)
        tangent = (np.array(profile[i1]) - np.array(profile[i0])) * rib.chord
        norm = np.linalg.norm(tangent)
        if norm == 0:
            return
        tangent = tangent / norm
        normal = np.array([-tangent[1], tangent[0]])

        half = 0.025 * rib.chord  # tick half-length, ~2.5% chord each side
        a = pt + normal * half
        b = pt - normal * half
        tick = Line_old([[a[0], a[1], 0], [b[0], b[1], 0]], width=3)
        tick.object.color.diffuseColor = (1.0, 0.4, 0.0)  # orange
        self.preview_root.addChild(tick.object)

    def _draw_reinforcement(self, reinforcement, rib, glider=None):
        """Draw an attachment reinforcement preview."""
        try:
            flat = reinforcement.get_flattened(rib, glider=glider)
            
            # Draw half-moon fabric reinforcement in yellow
            halfmoon_points = [[p[0], p[1], 0] for p in flat['halfmoon'].data]
            if halfmoon_points:
                self.preview_root.addChild(Line_old(halfmoon_points, color='yellow', width=2).object)
            
            # Draw rod sleeve in red
            rod_points = [[p[0], p[1], 0] for p in flat['rod_sleeve'].data]
            if rod_points:
                self.preview_root.addChild(Line_old(rod_points, color='red', width=2).object)
        except Exception as e:
            print(f"Error drawing reinforcement: {e}")

    def update_form_from_glider_data(self):
        """Load values from parametric glider into UI."""
        pg = self.parametric_glider
        is_suspended = self.ribTypeComboBox.currentIndex() == 1
        suffix = "_s" if is_suspended else "_ns"

        # Load extrados rod sleeve configs (new multi-rod format)
        extrados_enabled = getattr(pg, f'extrados_sleeves_enabled{suffix}', True)
        extrados_configs = getattr(pg, f'extrados_sleeves{suffix}', None)
        
        # Backward compatibility: migrate old single-rod format
        if extrados_configs is None:
            old_enabled = getattr(pg, f'extrados_sleeve_enabled{suffix}', True)
            old_config = {
                'start_chord': getattr(pg, f'extrados_sleeve_start{suffix}', 0.0),
                'end_chord': getattr(pg, f'extrados_sleeve_end{suffix}', 0.71),
                'width': getattr(pg, f'extrados_sleeve_width{suffix}', 0.015),
                'offset': getattr(pg, f'extrados_sleeve_offset{suffix}', 0.005),
                'start_curl': getattr(pg, f'extrados_sleeve_le_curl{suffix}', 60.0),
                'start_length': getattr(pg, f'extrados_sleeve_le_length{suffix}', 0.125),
                'end_curl': getattr(pg, f'extrados_sleeve_te_curl{suffix}', 60.0),
                'end_length': getattr(pg, f'extrados_sleeve_te_length{suffix}', 0.075),
            }
            extrados_configs = [old_config]
            extrados_enabled = old_enabled
        
        self.extradosGroup.set_configs(extrados_enabled, extrados_configs)

        # Load intrados rod sleeve configs (new multi-rod format)
        intrados_enabled = getattr(pg, f'intrados_sleeves_enabled{suffix}', True)
        intrados_configs = getattr(pg, f'intrados_sleeves{suffix}', None)
        
        # Backward compatibility: migrate old single-rod format
        if intrados_configs is None:
            old_enabled = getattr(pg, f'intrados_sleeve_enabled{suffix}', True)
            old_config = {
                'start_chord': getattr(pg, f'intrados_sleeve_start{suffix}', 0.06),
                'end_chord': getattr(pg, f'intrados_sleeve_end{suffix}', 0.50),
                'width': getattr(pg, f'intrados_sleeve_width{suffix}', 0.015),
                'offset': getattr(pg, f'intrados_sleeve_offset{suffix}', 0.005),
                'start_curl': getattr(pg, f'intrados_sleeve_le_curl{suffix}', 60.0),
                'start_length': getattr(pg, f'intrados_sleeve_le_length{suffix}', 0.100),
                'end_curl': getattr(pg, f'intrados_sleeve_te_curl{suffix}', 60.0),
                'end_length': getattr(pg, f'intrados_sleeve_te_length{suffix}', 0.090),
            }
            intrados_configs = [old_config]
            intrados_enabled = old_enabled
            
        self.intradosGroup.set_configs(intrados_enabled, intrados_configs)

        # Load reinforcement values (only for suspended)
        if is_suspended:
            # Rebuild tabs first - use first suspended rib to get attachment points
            rib = self.get_first_suspended_rib()
            if rib:
                valid_aps = self.get_valid_attachment_points(rib)
                self.update_reinforcement_tabs(valid_aps)
            
            # Global Enable/ApplyAll
            self.reinforcementEnabledCheckBox.setChecked(getattr(pg, 'reinforcement_enabled_s', True))
            apply_all = getattr(pg, 'reinforcement_apply_all_s', False)
            self.reinforcementApplyAllCheckBox.setChecked(apply_all)
            self.on_reinforcement_mode_change(None) # Update stack
            
            # Load excluded ribs for reinforcements
            excluded = getattr(pg, 'reinforcement_excluded_ribs_s', [])
            if excluded:
                self.reinforcementExcludedRibsEdit.setText(', '.join(str(x + 1) for x in excluded))
            else:
                self.reinforcementExcludedRibsEdit.clear()

            # Load Master Config
            master_config = getattr(pg, 'reinforcement_master_s', {})
            self.masterConfig.set_values(master_config)
            
            # Load Individual Configs
            configs = getattr(pg, 'reinforcement_configs_s', [])
            for i, widget in enumerate(self.reinforcement_widgets):
                if i < len(configs):
                    widget.set_values(configs[i])
                else:
                    pass

            # Load shark-nose config
            self.sharkConfig.set_values(getattr(pg, 'shark_nose_s', {}))

    def update_glider_data(self, is_suspended):
        """Save UI values to parametric glider."""
        pg = self.parametric_glider
        suffix = "_s" if is_suspended else "_ns"

        # Save extrados rod sleeve configs (new multi-rod format)
        setattr(pg, f'extrados_sleeves_enabled{suffix}', self.extradosGroup.is_enabled())
        setattr(pg, f'extrados_sleeves{suffix}', self.extradosGroup.get_configs())

        # Save intrados rod sleeve configs (new multi-rod format)
        setattr(pg, f'intrados_sleeves_enabled{suffix}', self.intradosGroup.is_enabled())
        setattr(pg, f'intrados_sleeves{suffix}', self.intradosGroup.get_configs())

        # Save reinforcement values (only for suspended)
        if is_suspended:
            pg.reinforcement_enabled_s = self.reinforcementEnabledCheckBox.isChecked()
            pg.reinforcement_apply_all_s = self.reinforcementApplyAllCheckBox.isChecked()
            pg.reinforcement_master_s = self.masterConfig.get_values()
            
            # Save excluded ribs for reinforcements
            pg.reinforcement_excluded_ribs_s = self.get_reinforcement_excluded_ribs()
            
            # Save list of configs
            configs = [w.get_values() for w in self.reinforcement_widgets]
            pg.reinforcement_configs_s = configs

            # Save shark-nose config
            pg.shark_nose_s = self.sharkConfig.get_values()
    
    def get_reinforcement_excluded_ribs(self):
        """Parse excluded ribs for reinforcements. Returns list of 0-based indices."""
        text = self.reinforcementExcludedRibsEdit.text().strip()
        if not text:
            return []
        try:
            # Parse comma-separated rib numbers (1-based from user, convert to 0-based)
            return [int(x.strip()) - 1 for x in text.split(',') if x.strip().isdigit()]
        except:
            return []

    def apply_reinforcements_to_ribs(self):
        """Apply reinforcement configurations to the actual rib objects for 2D export."""
        pg = self.parametric_glider
        glider_instance = self.obj.Proxy.getGliderInstance()
        
        # Only apply if reinforcements are enabled for suspended ribs
        if not getattr(pg, 'reinforcement_enabled_s', False):
            # Clear reinforcements from all ribs
            for rib in glider_instance.ribs:
                rib.reinforcements = []
            return
        
        apply_all = getattr(pg, 'reinforcement_apply_all_s', False)
        master_config = getattr(pg, 'reinforcement_master_s', {})
        configs = getattr(pg, 'reinforcement_configs_s', [])
        excluded_ribs = getattr(pg, 'reinforcement_excluded_ribs_s', [])

        # Shark-nose config + the rod-sleeve configs it englobes (parametric,
        # suspended-side). Only the front-most attachment point uses it.
        shark_cfg = getattr(pg, 'shark_nose_s', {}) or {}
        shark_extrados_configs = (getattr(pg, 'extrados_sleeves_s', [])
                                  if getattr(pg, 'extrados_sleeves_enabled_s', True) else [])
        shark_intrados_configs = (getattr(pg, 'intrados_sleeves_s', [])
                                  if getattr(pg, 'intrados_sleeves_enabled_s', True) else [])

        for rib_idx, rib in enumerate(glider_instance.ribs):
            # Robust suspended-rib test (see apply_rod_sleeves_to_ribs): match
            # attachment points by name/identity via get_rib_attachment_points,
            # not an identity-only set that can misclassify every rib.
            if glider_instance.get_rib_attachment_points(rib):
                # Check if this rib is excluded from reinforcements
                if rib_idx in excluded_ribs:
                    rib.reinforcements = []
                    continue

                # Get valid attachment points for this rib
                valid_aps = self.get_valid_attachment_points(rib)

                reinforcements = []
                for i, ap in enumerate(valid_aps):
                    # Generate name: rib index + attachment point name (contains
                    # the line letter A, B, C, D...).
                    name = f"{rib_idx + 1}{ap.name}" if ap.name else f"{rib_idx + 1}_{i + 1}"

                    # Get config
                    if apply_all:
                        config = master_config
                    else:
                        config = configs[i] if i < len(configs) else master_config

                    # Front-most attachment point: the half-moon is prolonged into
                    # a merged shark-nose piece when enabled (keeps the half-moon).
                    if i == 0 and shark_cfg.get('enabled'):
                        reinforcement, _, _ = self._create_shark_reinforcement(
                            rib, glider_instance, shark_cfg, ap.rib_pos,
                            shark_extrados_configs, shark_intrados_configs, config)
                        reinforcement.name = name
                        reinforcements.append(reinforcement)
                        continue

                    if config.get('enabled', True):
                        reinforcement = self._create_reinforcement(ap.rib_pos, config, name)
                        reinforcements.append(reinforcement)

                rib.reinforcements = reinforcements
            else:
                # Non-suspended ribs don't get reinforcements
                rib.reinforcements = []

    def apply_rod_sleeves_to_ribs(self):
        """Apply rod sleeve configurations to the actual rib objects for 2D export."""
        from openglider.glider.rib.elements import RodSleeve
        
        pg = self.parametric_glider
        glider_instance = self.obj.Proxy.getGliderInstance()

        for rib_idx, rib in enumerate(glider_instance.ribs):
            # A rib is "suspended" when lines attach to it. Classify with the
            # robust name/identity matcher (get_rib_attachment_points) rather
            # than an identity-only set over attachment-point .rib objects:
            # those objects are not always the same instances as the glider's
            # ribs (SingleSkinRib replacements, mirrored ribs, rebuilds), so an
            # identity set silently classifies EVERY rib as non-suspended, and
            # every rib then reads the _ns config — coupling the two profile
            # types (removing non-suspended joncs makes them vanish everywhere).
            is_suspended = len(glider_instance.get_rib_attachment_points(rib)) > 0
            suffix = '_s' if is_suspended else '_ns'
            
            rod_sleeves = []
            
            # Get extrados sleeves
            extrados_enabled = getattr(pg, f'extrados_sleeves_enabled{suffix}', True)
            extrados_configs = getattr(pg, f'extrados_sleeves{suffix}', [])
            
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
            intrados_enabled = getattr(pg, f'intrados_sleeves_enabled{suffix}', True)
            intrados_configs = getattr(pg, f'intrados_sleeves{suffix}', [])
            
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

    def accept(self):
        is_suspended = self.ribTypeComboBox.currentIndex() == 1

        # Save only the currently displayed mode's data. The other mode's config
        # was already persisted to the parametric glider when switching profile
        # type (see on_rib_type_change), so the suspended (_s) and non-suspended
        # (_ns) configs stay independent. Writing to both suffixes here would
        # clobber the other mode with the currently shown config.
        self.update_glider_data(is_suspended)

        # Apply reinforcements and rod sleeves to ribs for 2D export
        self.apply_reinforcements_to_ribs()
        self.apply_rod_sleeves_to_ribs()
        self.update_view_glider()
        super().accept()


class ReinforcementConfigWidget(QtGui.QWidget):
    """Widget for configuring a single reinforcement's parameters."""
    changed = QtCore.Signal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QtGui.QFormLayout(self)
        self.layout.setContentsMargins(0, 5, 0, 5)
        
        self.enableCheckBox = QtGui.QCheckBox("Enable")
        self.enableCheckBox.setChecked(True)
        self.layout.addRow(self.enableCheckBox)
        
        self.surfaceOffsetSpinBox = QtGui.QDoubleSpinBox()
        self.surfaceOffsetSpinBox.setSingleStep(0.5)
        self.surfaceOffsetSpinBox.setDecimals(1)
        self.surfaceOffsetSpinBox.setSuffix(" mm")
        self.surfaceOffsetSpinBox.setRange(0.0, 1000.0)
        self.surfaceOffsetSpinBox.setValue(0.5)  # 0.5mm default
        self.layout.addRow("Surface offset", self.surfaceOffsetSpinBox)
        
        self.halfmoonRadiusSpinBox = QtGui.QDoubleSpinBox()
        self.halfmoonRadiusSpinBox.setSingleStep(1.0)
        self.halfmoonRadiusSpinBox.setDecimals(1)
        self.halfmoonRadiusSpinBox.setSuffix(" mm")
        self.halfmoonRadiusSpinBox.setRange(0.0, 1000.0)
        self.halfmoonRadiusSpinBox.setValue(100.0)  # 100mm default
        self.layout.addRow("Half-moon radius", self.halfmoonRadiusSpinBox)
        
        self.rodEnabledCheckBox = QtGui.QCheckBox("Enable Rod Sleeve")
        self.rodEnabledCheckBox.setChecked(True)
        self.layout.addRow(self.rodEnabledCheckBox)
        
        self.rodOffsetSpinBox = QtGui.QDoubleSpinBox()
        self.rodOffsetSpinBox.setSingleStep(1.0)
        self.rodOffsetSpinBox.setDecimals(1)
        self.rodOffsetSpinBox.setSuffix(" mm")
        self.rodOffsetSpinBox.setRange(0.0, 1000.0)
        self.rodOffsetSpinBox.setValue(8.0)  # 8mm default
        self.layout.addRow("Rod offset", self.rodOffsetSpinBox)
        
        self.rodWidthSpinBox = QtGui.QDoubleSpinBox()
        self.rodWidthSpinBox.setSingleStep(0.5)
        self.rodWidthSpinBox.setDecimals(1)
        self.rodWidthSpinBox.setSuffix(" mm")
        self.rodWidthSpinBox.setRange(0.0, 1000.0)
        self.rodWidthSpinBox.setValue(9.0)  # 9mm default
        self.layout.addRow("Rod width", self.rodWidthSpinBox)
        
        self.rodEndOffsetSpinBox = QtGui.QDoubleSpinBox()
        self.rodEndOffsetSpinBox.setSingleStep(1.0)
        self.rodEndOffsetSpinBox.setDecimals(1)
        self.rodEndOffsetSpinBox.setSuffix(" °")
        self.rodEndOffsetSpinBox.setRange(0.0, 90.0)
        self.rodEndOffsetSpinBox.setValue(1.0)  # 1° default
        self.layout.addRow("Rod end offset", self.rodEndOffsetSpinBox)

        self.cornerRadiusSpinBox = QtGui.QDoubleSpinBox()
        self.cornerRadiusSpinBox.setSingleStep(1.0)
        self.cornerRadiusSpinBox.setDecimals(1)
        self.cornerRadiusSpinBox.setSuffix(" mm")
        self.cornerRadiusSpinBox.setRange(0.0, 200.0)
        self.cornerRadiusSpinBox.setValue(0.0)
        self.cornerRadiusSpinBox.setToolTip("Rounding of the crescent tips near the intrados. 0 = sharp.")
        self.layout.addRow("Corner radius", self.cornerRadiusSpinBox)

        self.relativeCheckBox = QtGui.QCheckBox("Proportional (% chord)")
        self.relativeCheckBox.setChecked(False)
        self.relativeCheckBox.setToolTip(
            "Scale the half-moon dimensions (offset, radius, rod) with the profile "
            "size: interpret them as percentages of the chord.")
        self.layout.addRow(self.relativeCheckBox)

        # Reference chord (of the preview rib) for the mm<->% conversion.
        self.ref_chord = 2.5
        # Metric fields that switch between mm and % chord in proportional mode.
        self._metric_spins = [self.surfaceOffsetSpinBox, self.halfmoonRadiusSpinBox,
                              self.rodOffsetSpinBox, self.rodWidthSpinBox]

        self.enableCheckBox.stateChanged.connect(self.emit_changed)
        self.surfaceOffsetSpinBox.valueChanged.connect(self.emit_changed)
        self.halfmoonRadiusSpinBox.valueChanged.connect(self.emit_changed)
        self.rodEnabledCheckBox.stateChanged.connect(self.emit_changed)
        self.rodOffsetSpinBox.valueChanged.connect(self.emit_changed)
        self.rodWidthSpinBox.valueChanged.connect(self.emit_changed)
        self.rodEndOffsetSpinBox.valueChanged.connect(self.emit_changed)
        self.cornerRadiusSpinBox.valueChanged.connect(self.emit_changed)
        self.relativeCheckBox.stateChanged.connect(self._on_relative_toggle)

    def emit_changed(self):
        self.changed.emit()

    def _apply_relative_units(self, relative):
        """Set the metric spinboxes' unit for %-chord vs mm (no conversion)."""
        for spin in self._metric_spins:
            blocked = spin.blockSignals(True)
            if relative:
                spin.setSuffix(" %")
                spin.setDecimals(2)
                spin.setRange(0.0, 100.0)
                spin.setSingleStep(0.1)
            else:
                spin.setSuffix(" mm")
                spin.setDecimals(1)
                spin.setRange(0.0, 1000.0)
                spin.setSingleStep(1.0)
            spin.blockSignals(blocked)

    def _on_relative_toggle(self):
        """Convert the metric fields between mm and % chord, then re-unit."""
        relative = self.relativeCheckBox.isChecked()
        ref = self.ref_chord or 2.5
        vals = [s.value() for s in self._metric_spins]
        self._apply_relative_units(relative)
        for spin, v in zip(self._metric_spins, vals):
            new = (v / 1000.0) / ref * 100.0 if relative else (v / 100.0) * ref * 1000.0
            blocked = spin.blockSignals(True)
            spin.setValue(new)
            spin.blockSignals(blocked)
        self.emit_changed()

    def get_values(self):
        relative = self.relativeCheckBox.isChecked()
        div = 100.0 if relative else 1000.0   # % chord (fraction) vs mm (meters)
        return {
            'enabled': self.enableCheckBox.isChecked(),
            'surface_offset': self.surfaceOffsetSpinBox.value() / div,
            'halfmoon_radius': self.halfmoonRadiusSpinBox.value() / div,
            'rod_enabled': self.rodEnabledCheckBox.isChecked(),
            'rod_offset': self.rodOffsetSpinBox.value() / div,
            'rod_width': self.rodWidthSpinBox.value() / div,
            'rod_end_offset': self.rodEndOffsetSpinBox.value(),
            'corner_radius': self.cornerRadiusSpinBox.value() / 1000.0,
            'relative': relative,
        }

    def set_values(self, config):
        if not config:
            return
        self.enableCheckBox.setChecked(config.get('enabled', True))
        relative = config.get('relative', False)
        blocked = self.relativeCheckBox.blockSignals(True)
        self.relativeCheckBox.setChecked(relative)
        self.relativeCheckBox.blockSignals(blocked)
        self._apply_relative_units(relative)
        mul = 100.0 if relative else 1000.0
        self.surfaceOffsetSpinBox.setValue(config.get('surface_offset', 0.003) * mul)
        self.halfmoonRadiusSpinBox.setValue(config.get('halfmoon_radius', 0.03) * mul)
        self.rodEnabledCheckBox.setChecked(config.get('rod_enabled', True))
        self.rodOffsetSpinBox.setValue(config.get('rod_offset', 0.005) * mul)
        self.rodWidthSpinBox.setValue(config.get('rod_width', 0.005) * mul)
        self.rodEndOffsetSpinBox.setValue(config.get('rod_end_offset', 10.0))
        self.cornerRadiusSpinBox.setValue(config.get('corner_radius', 0.0) * 1000.0)


class SharkNoseConfigWidget(QtGui.QWidget):
    """Config for the shark-nose reinforcement on the front attachment point.

    A rounded bounding box confined to the intrados: its bottom edge follows the
    intrados profile (and the nose) from 'Start' to 'End' (chord %), its lid is a
    constant-thickness band 'Thickness' inward from the intrados, and each end cap
    is cut at its 'Start angle' / 'End angle' (90 = perpendicular) with rounded
    corners.
    """
    changed = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QtGui.QFormLayout(self)
        self.layout.setContentsMargins(0, 5, 0, 5)

        # Reference chord (of the preview rib) used to convert the Thickness field
        # between mm and % chord; updated by the tool.
        self.ref_chord = 2.5

        self.enableCheckBox = QtGui.QCheckBox("Shark nose (front attachment)")
        self.enableCheckBox.setChecked(False)
        self.layout.addRow(self.enableCheckBox)

        self.startSpinBox = QtGui.QDoubleSpinBox()
        self.startSpinBox.setSingleStep(1.0)
        self.startSpinBox.setDecimals(1)
        self.startSpinBox.setSuffix(" %")
        self.startSpinBox.setRange(0.0, 100.0)
        self.startSpinBox.setValue(3.0)
        self.startSpinBox.setToolTip("Start of the box on the intrados (chord %). 0 = leading edge.")
        self.layout.addRow("Start (intrados)", self.startSpinBox)

        self.endSpinBox = QtGui.QDoubleSpinBox()
        self.endSpinBox.setSingleStep(1.0)
        self.endSpinBox.setDecimals(1)
        self.endSpinBox.setSuffix(" %")
        self.endSpinBox.setRange(0.0, 100.0)
        self.endSpinBox.setValue(15.0)
        self.endSpinBox.setToolTip("End of the box on the intrados (chord %).")
        self.layout.addRow("End (intrados)", self.endSpinBox)

        self.thicknessSpinBox = QtGui.QDoubleSpinBox()
        self.thicknessSpinBox.setSingleStep(1.0)
        self.thicknessSpinBox.setDecimals(1)
        self.thicknessSpinBox.setSuffix(" mm")
        self.thicknessSpinBox.setRange(1.0, 500.0)
        self.thicknessSpinBox.setValue(35.0)
        self.thicknessSpinBox.setToolTip("Constant band thickness, inward from the intrados.")
        self.layout.addRow("Thickness", self.thicknessSpinBox)

        self.depthRelativeCheckBox = QtGui.QCheckBox("Proportional (% chord)")
        self.depthRelativeCheckBox.setChecked(False)
        self.depthRelativeCheckBox.setToolTip(
            "Scale the thickness with the profile size: interpret Thickness as a "
            "percentage of the chord instead of an absolute length.")
        self.layout.addRow(self.depthRelativeCheckBox)

        self.startAngleSpinBox = QtGui.QDoubleSpinBox()
        self.startAngleSpinBox.setSingleStep(5.0)
        self.startAngleSpinBox.setDecimals(0)
        self.startAngleSpinBox.setSuffix(" °")
        self.startAngleSpinBox.setRange(20.0, 160.0)
        self.startAngleSpinBox.setValue(90.0)
        self.startAngleSpinBox.setToolTip("Angle of the start (nose-side) end cap. 90 = perpendicular.")
        self.layout.addRow("Start angle", self.startAngleSpinBox)

        self.endAngleSpinBox = QtGui.QDoubleSpinBox()
        self.endAngleSpinBox.setSingleStep(5.0)
        self.endAngleSpinBox.setDecimals(0)
        self.endAngleSpinBox.setSuffix(" °")
        self.endAngleSpinBox.setRange(20.0, 160.0)
        self.endAngleSpinBox.setValue(90.0)
        self.endAngleSpinBox.setToolTip("Angle of the end (trailing-side) end cap. 90 = perpendicular.")
        self.layout.addRow("End angle", self.endAngleSpinBox)

        self.cornerRadiusSpinBox = QtGui.QDoubleSpinBox()
        self.cornerRadiusSpinBox.setSingleStep(5.0)
        self.cornerRadiusSpinBox.setDecimals(0)
        self.cornerRadiusSpinBox.setSuffix(" %")
        self.cornerRadiusSpinBox.setRange(0.0, 100.0)
        self.cornerRadiusSpinBox.setValue(50.0)
        self.cornerRadiusSpinBox.setToolTip(
            "Corner rounding as a fraction of the band depth: 0 % = sharp, "
            "100 % = fully rounded. Scales with the reinforcement size.")
        self.layout.addRow("Corner rounding", self.cornerRadiusSpinBox)

        self.rodCheckBox = QtGui.QCheckBox("Rod sleeve")
        self.rodCheckBox.setChecked(True)
        self.rodCheckBox.setToolTip("Include the attachment-point rod sleeve inside the box.")
        self.layout.addRow(self.rodCheckBox)

        self.enableCheckBox.stateChanged.connect(self._update_enabled_state)
        self.enableCheckBox.stateChanged.connect(self.emit_changed)
        self.startSpinBox.valueChanged.connect(self.emit_changed)
        self.endSpinBox.valueChanged.connect(self.emit_changed)
        self.thicknessSpinBox.valueChanged.connect(self.emit_changed)
        self.depthRelativeCheckBox.stateChanged.connect(self._on_depth_mode_toggle)
        self.startAngleSpinBox.valueChanged.connect(self.emit_changed)
        self.endAngleSpinBox.valueChanged.connect(self.emit_changed)
        self.cornerRadiusSpinBox.valueChanged.connect(self.emit_changed)
        self.rodCheckBox.stateChanged.connect(self.emit_changed)

        self._update_enabled_state()

    def emit_changed(self):
        self.changed.emit()

    def _apply_depth_units(self, relative):
        """Set the Thickness spinbox unit/range for %-chord vs mm (no convert)."""
        blocked = self.thicknessSpinBox.blockSignals(True)
        if relative:
            self.thicknessSpinBox.setSuffix(" %")
            self.thicknessSpinBox.setDecimals(2)
            self.thicknessSpinBox.setRange(0.1, 50.0)
            self.thicknessSpinBox.setSingleStep(0.1)
        else:
            self.thicknessSpinBox.setSuffix(" mm")
            self.thicknessSpinBox.setDecimals(1)
            self.thicknessSpinBox.setRange(1.0, 500.0)
            self.thicknessSpinBox.setSingleStep(1.0)
        self.thicknessSpinBox.blockSignals(blocked)

    def _on_depth_mode_toggle(self):
        """Convert the Thickness value between mm and % chord, then re-unit."""
        relative = self.depthRelativeCheckBox.isChecked()
        ref = self.ref_chord or 2.5
        cur = self.thicknessSpinBox.value()
        new = (cur / 1000.0) / ref * 100.0 if relative else (cur / 100.0) * ref * 1000.0
        self._apply_depth_units(relative)
        blocked = self.thicknessSpinBox.blockSignals(True)
        self.thicknessSpinBox.setValue(new)
        self.thicknessSpinBox.blockSignals(blocked)
        self.emit_changed()

    def _update_enabled_state(self):
        on = self.enableCheckBox.isChecked()
        for w in (self.startSpinBox, self.endSpinBox, self.thicknessSpinBox,
                  self.depthRelativeCheckBox, self.startAngleSpinBox, self.endAngleSpinBox,
                  self.cornerRadiusSpinBox, self.rodCheckBox):
            w.setEnabled(on)

    def get_values(self):
        relative = self.depthRelativeCheckBox.isChecked()
        # relative -> depth is a chord fraction (value %); else meters (value mm)
        depth = self.thicknessSpinBox.value() / (100.0 if relative else 1000.0)
        return {
            'enabled': self.enableCheckBox.isChecked(),
            'start': self.startSpinBox.value() / 100.0,
            'end': self.endSpinBox.value() / 100.0,
            'depth': depth,
            'depth_relative': relative,
            'start_angle': self.startAngleSpinBox.value(),
            'end_angle': self.endAngleSpinBox.value(),
            'corner_radius': self.cornerRadiusSpinBox.value() / 100.0,
            'rod': self.rodCheckBox.isChecked(),
        }

    def set_values(self, config):
        if not config:
            return
        self.enableCheckBox.setChecked(config.get('enabled', False))
        self.startSpinBox.setValue(config.get('start', 0.03) * 100.0)
        self.endSpinBox.setValue(config.get('end', 0.15) * 100.0)
        relative = config.get('depth_relative', False)
        blocked = self.depthRelativeCheckBox.blockSignals(True)
        self.depthRelativeCheckBox.setChecked(relative)
        self.depthRelativeCheckBox.blockSignals(blocked)
        self._apply_depth_units(relative)
        blocked = self.thicknessSpinBox.blockSignals(True)
        self.thicknessSpinBox.setValue(config.get('depth', 0.035) * (100.0 if relative else 1000.0))
        self.thicknessSpinBox.blockSignals(blocked)
        self.startAngleSpinBox.setValue(config.get('start_angle', 90.0))
        self.endAngleSpinBox.setValue(config.get('end_angle', 90.0))
        self.cornerRadiusSpinBox.setValue(config.get('corner_radius', 0.5) * 100.0)
        self.rodCheckBox.setChecked(config.get('rod', True))
        self._update_enabled_state()

