"""
Lines Auto-Placement Dialog for OpenGlider

This dialog allows users to automatically generate suspension line attachment points
and line architecture based on configurable parameters.

Architecture concept:
- Lines are organized by TYPE (A, B, C, D, F) along the chord
- Within each type, lines are organized by GROUPS along the span
- Each GROUP corresponds to ONE "basse" (lower line connecting to riser)
- Each group can have its own pattern (3:1, 2:2:1, etc.)

Example for A lines with 6 attachment points:
- Group 1 (A1): pattern 3:1 → 3 hautes connect to basse A1
- Group 2 (A2): pattern 2:2:1 → 2+2 hautes connect via inters to basse A2
"""


from PySide import QtCore, QtGui

from openglider.glider.parametric.lines import (
    BatchNode2D,
    Line2D,
    LineSet2D,
    LowerNode2D,
    UpperNode2D,
)
from openglider.lines.line_types import LineType


class GroupPatternWidget(QtGui.QWidget):
    """Widget for defining pattern per group."""
    
    PATTERNS = ["1:1", "2:1", "3:1", "2:2:1", "3:2:1", "4:2:1"]
    
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtGui.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Pattern input as text
        self.pattern_edit = QtGui.QLineEdit()
        self.pattern_edit.setPlaceholderText("e.g.: 3:1, 2:2:1, 2:1, 2:1")
        self.pattern_edit.setText("2:1")
        layout.addWidget(self.pattern_edit)
        
        # Help label
        help_label = QtGui.QLabel("Format: pattern1, pattern2, ... (repeated as needed)")
        help_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(help_label)
    
    def get_patterns(self):
        """Parse pattern string and return list of patterns."""
        text = self.pattern_edit.text().strip()
        if not text:
            return [[2]]  # Default 2:1
        
        patterns = []
        for part in text.split(","):
            part = part.strip()
            if ":" in part:
                try:
                    pattern = [int(x) for x in part.split(":") if x.strip() and x.strip() != "1"]
                    if not pattern:
                        pattern = [1]
                    patterns.append(pattern)
                except ValueError:
                    patterns.append([2])  # Default
            else:
                try:
                    patterns.append([int(part)])
                except ValueError:
                    patterns.append([2])
        
        return patterns if patterns else [[2]]


class LineTypeConfigRow(QtGui.QWidget):
    """Widget for configuring a single line type (A, B, C, D, F)."""
    
    configChanged = QtCore.Signal()
    
    def __init__(self, line_type_name, default_position, default_pattern="2:1", parent=None):
        super().__init__(parent)
        self.line_type_name = line_type_name
        
        layout = QtGui.QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        
        # Enable checkbox
        self.enable_checkbox = QtGui.QCheckBox(line_type_name)
        self.enable_checkbox.setChecked(True)
        self.enable_checkbox.setFixedWidth(40)
        layout.addWidget(self.enable_checkbox)
        
        # Position (%)
        self.position_spinbox = QtGui.QDoubleSpinBox()
        self.position_spinbox.setRange(0, 100)
        self.position_spinbox.setValue(default_position)
        self.position_spinbox.setDecimals(1)
        self.position_spinbox.setSuffix("%")
        self.position_spinbox.setFixedWidth(65)
        layout.addWidget(self.position_spinbox)
        
        # Interval
        self.interval_spinbox = QtGui.QSpinBox()
        self.interval_spinbox.setRange(1, 10)
        self.interval_spinbox.setValue(1)
        self.interval_spinbox.setPrefix("/")
        self.interval_spinbox.setFixedWidth(45)
        layout.addWidget(self.interval_spinbox)
        
        # Start cell
        self.start_spinbox = QtGui.QSpinBox()
        self.start_spinbox.setRange(0, 50)
        self.start_spinbox.setValue(0)
        self.start_spinbox.setPrefix("@")
        self.start_spinbox.setFixedWidth(45)
        layout.addWidget(self.start_spinbox)
        
        # Group patterns (text input)
        self.patterns_edit = QtGui.QLineEdit()
        self.patterns_edit.setPlaceholderText("2:1, 3:1, ...")
        self.patterns_edit.setText(default_pattern)
        self.patterns_edit.setFixedWidth(120)
        layout.addWidget(self.patterns_edit)
        
        # Connect signals
        self.enable_checkbox.toggled.connect(self._update_enabled_state)
        self.enable_checkbox.toggled.connect(lambda: self.configChanged.emit())
        self.position_spinbox.valueChanged.connect(lambda: self.configChanged.emit())
        self.interval_spinbox.valueChanged.connect(lambda: self.configChanged.emit())
        self.start_spinbox.valueChanged.connect(lambda: self.configChanged.emit())
        self.patterns_edit.textChanged.connect(lambda: self.configChanged.emit())
        
    def _update_enabled_state(self, enabled):
        self.position_spinbox.setEnabled(enabled)
        self.interval_spinbox.setEnabled(enabled)
        self.start_spinbox.setEnabled(enabled)
        self.patterns_edit.setEnabled(enabled)
    
    def is_enabled(self):
        return self.enable_checkbox.isChecked()
    
    def get_group_patterns(self):
        """Parse group patterns from text input."""
        text = self.patterns_edit.text().strip()
        if not text:
            return [[2]]  # Default 2:1
        
        patterns = []
        for part in text.split(","):
            part = part.strip()
            if ":" in part:
                try:
                    # Parse "2:2:1" -> [2, 2] (remove final :1)
                    nums = [int(x) for x in part.split(":") if x.strip()]
                    # Remove trailing 1s (they're implicit)
                    while len(nums) > 1 and nums[-1] == 1:
                        nums.pop()
                    patterns.append(nums if nums else [1])
                except ValueError:
                    patterns.append([2])
            else:
                try:
                    patterns.append([int(part)])
                except ValueError:
                    patterns.append([2])
        
        return patterns if patterns else [[2]]
    
    def get_config(self):
        return {
            "enabled": self.is_enabled(),
            "position": self.position_spinbox.value(),  # Keep as % for save/load roundtrip
            "interval": self.interval_spinbox.value(),
            "start_cell": self.start_spinbox.value(),
            "group_patterns": self.get_group_patterns(),
            "patterns": self.patterns_edit.text(),  # Save raw text too
        }


class LinesAutoPlacementDialog(QtGui.QDialog):
    """Dialog for automatic placement of suspension lines."""
    
    # Default configurations
    LINE_DEFAULTS = {
        "A": {"position": 8.5, "interval": 1, "pattern": "3:1, 2:2:1"},
        "B": {"position": 27.5, "interval": 2, "pattern": "2:2:1, 2:1"},
        "C": {"position": 53.0, "interval": 2, "pattern": "2:1"},
        "D": {"position": 77.0, "interval": 3, "pattern": "2:1"},
        "F": {"position": 100.0, "interval": 1, "pattern": "1:1"},
    }
    
    def __init__(self, parametric_glider, parent=None):
        super().__init__(parent)
        self.parametric_glider = parametric_glider
        self.half_cell_num = parametric_glider.shape.half_cell_num
        self.half_rib_num = parametric_glider.shape.half_rib_num
        
        # Calculate span - shape.span gives half-span (from center to tip)
        try:
            self.half_span = self.parametric_glider.shape.span
        except:
            self.half_span = 6.0
        
        # Calculate central chord for % depth conversion
        try:
            ribs = self.parametric_glider.shape.ribs
            fr, ba = ribs[0]  # Central rib
            self.central_chord = abs(ba[1] - fr[1])  # Always positive chord length
        except:
            self.central_chord = 2.5  # Fallback
        
        self.setWindowTitle("Suspension Lines Auto-Placement")
        self.setMinimumWidth(520)
        
        self.setup_ui()
        self.load_from_glider()  # Restore saved dialog state (fork angle, etc.)
        self._extract_from_existing_lineset()  # Override with real lineset data
        
    def setup_ui(self):
        outer_layout = QtGui.QVBoxLayout(self)
        
        # Scroll area for all content
        scroll = QtGui.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QtGui.QWidget()
        main_layout = QtGui.QVBoxLayout(scroll_widget)
        scroll.setWidget(scroll_widget)
        outer_layout.addWidget(scroll)
        
        # === Main Point ===
        lower_group = QtGui.QGroupBox("Main Point (X=span, Y=chord, Z=height)")
        lower_layout = QtGui.QFormLayout(lower_group)
        
        # Half-span spread (X - span direction) in cm
        self.demi_ecartement = QtGui.QDoubleSpinBox()
        self.demi_ecartement.setRange(0, 200)
        self.demi_ecartement.setValue(20)
        self.demi_ecartement.setSingleStep(1)
        self.demi_ecartement.setSuffix(" cm")
        lower_layout.addRow("Half-span spread:", self.demi_ecartement)
        
        # Depth (Y - chord direction, as % of central chord)
        self.profondeur = QtGui.QDoubleSpinBox()
        self.profondeur.setRange(-100, 200)
        self.profondeur.setValue(20.0)
        self.profondeur.setSingleStep(1.0)
        self.profondeur.setDecimals(1)
        self.profondeur.setSuffix(" %")
        lower_layout.addRow("Depth (% central chord):", self.profondeur)
        
        # Central chord info label
        chord_info = QtGui.QLabel(f"Central chord: {self.central_chord * 100:.0f} cm")
        chord_info.setStyleSheet("color: gray; font-size: 10px;")
        lower_layout.addRow("", chord_info)
        
        # === Aerodynamic Cp Reference ===
        aero_results = getattr(self.parametric_glider, 'aerodynamics_results', None)
        if aero_results:
            cop_central = aero_results.get('cop_central_pct', None)
            cop_global = aero_results.get('cop_global_pct', None)
            best_alpha = aero_results.get('best_ld_alpha', None)
            
            aero_label_text = "<b>Aero Cp (Best L/D"
            if best_alpha is not None:
                aero_label_text += f", α={best_alpha:.1f}°"
            aero_label_text += "):</b>"
            
            if cop_central is not None:
                aero_label_text += f"  Central: <b>{cop_central:.1f}%</b>"
            if cop_global is not None:
                aero_label_text += f"  |  Average: <b>{cop_global:.1f}%</b>"
            
            aero_label = QtGui.QLabel(aero_label_text)
            aero_label.setStyleSheet("color: #2196F3; font-size: 11px; padding: 2px;")
            lower_layout.addRow("", aero_label)
        else:
            aero_hint = QtGui.QLabel("<i>Run Aerodynamic Analysis to get Cp guidance</i>")
            aero_hint.setStyleSheet("color: gray; font-size: 10px;")
            lower_layout.addRow("", aero_hint)
        
        # Cone height (Z - height below wing)
        hauteur_widget = QtGui.QWidget()
        hauteur_layout = QtGui.QHBoxLayout(hauteur_widget)
        hauteur_layout.setContentsMargins(0, 0, 0, 0)
        
        self.hauteur_cone = QtGui.QDoubleSpinBox()
        self.hauteur_cone.setRange(1, 15)
        # Auto value: ~75% of full wingspan (typical ratio for paragliders)
        auto_hauteur = round(self.half_span * 2 * 0.75, 1)
        self.hauteur_cone.setValue(auto_hauteur)
        self.hauteur_cone.setSingleStep(0.5)
        self.hauteur_cone.setSuffix(" m")
        self.hauteur_cone.setEnabled(False)
        hauteur_layout.addWidget(self.hauteur_cone)
        
        self.hauteur_auto = QtGui.QCheckBox("Auto (75% wingspan)")
        self.hauteur_auto.setChecked(True)
        self.hauteur_auto.toggled.connect(self._update_hauteur_auto)
        hauteur_layout.addWidget(self.hauteur_auto)
        
        lower_layout.addRow("Cone height:", hauteur_widget)
        
        main_layout.addWidget(lower_group)
        
        # === Brake Point (offset from main) ===
        brake_group = QtGui.QGroupBox("Brake Point (offset from Main Point)")
        brake_layout = QtGui.QFormLayout(brake_group)
        
        # Enable separate brake point
        self.brake_separate = QtGui.QCheckBox("Separate brake point")
        self.brake_separate.setChecked(True)
        self.brake_separate.toggled.connect(self._update_brake_enabled)
        brake_layout.addRow("", self.brake_separate)
        
        # Offset height (Z - higher than main, positive = higher)
        self.brake_offset_z = QtGui.QDoubleSpinBox()
        self.brake_offset_z.setRange(-1.0, 2.0)
        self.brake_offset_z.setValue(0.40)  # 40cm higher
        self.brake_offset_z.setSingleStep(0.05)
        self.brake_offset_z.setSuffix(" m")
        brake_layout.addRow("Height offset:", self.brake_offset_z)
        
        # Offset spread (Y - more outward, positive = more spread)
        self.brake_offset_y = QtGui.QDoubleSpinBox()
        self.brake_offset_y.setRange(-0.5, 1.0)
        self.brake_offset_y.setValue(0.15)  # 15cm more outward
        self.brake_offset_y.setSingleStep(0.05)
        self.brake_offset_y.setSuffix(" m")
        brake_layout.addRow("Spread offset:", self.brake_offset_y)
        
        # Offset depth (X - chord direction, positive = more backward)
        self.brake_offset_x = QtGui.QDoubleSpinBox()
        self.brake_offset_x.setRange(-1.0, 1.0)
        self.brake_offset_x.setValue(0.0)  # 0cm by default
        self.brake_offset_x.setSingleStep(0.05)
        self.brake_offset_x.setSuffix(" m")
        brake_layout.addRow("Depth offset:", self.brake_offset_x)
        
        main_layout.addWidget(brake_group)

        
        # === Lengths ===
        lengths_group = QtGui.QGroupBox("Lengths")
        lengths_layout = QtGui.QFormLayout(lengths_group)
        
        self.riser_length = QtGui.QDoubleSpinBox()
        self.riser_length.setRange(0.1, 2.0)
        self.riser_length.setValue(0.47)
        self.riser_length.setSingleStep(0.01)
        self.riser_length.setSuffix(" m")
        lengths_layout.addRow("Risers:", self.riser_length)
        
        # Lowers (longest)
        basses_widget = QtGui.QWidget()
        basses_layout = QtGui.QHBoxLayout(basses_widget)
        basses_layout.setContentsMargins(0, 0, 0, 0)
        self.basses_length = QtGui.QDoubleSpinBox()
        self.basses_length.setRange(0.5, 10.0)
        self.basses_length.setValue(round(self.hauteur_cone.value() * 0.45, 2))
        self.basses_length.setSuffix(" m")
        self.basses_length.setEnabled(False)
        basses_layout.addWidget(self.basses_length)
        self.basses_auto = QtGui.QCheckBox("Auto")
        self.basses_auto.setChecked(True)
        self.basses_auto.toggled.connect(lambda c: self.basses_length.setEnabled(not c))
        basses_layout.addWidget(self.basses_auto)
        lengths_layout.addRow("Lowers:", basses_widget)
        
        # Mid (medium)
        inter_widget = QtGui.QWidget()
        inter_layout = QtGui.QHBoxLayout(inter_widget)
        inter_layout.setContentsMargins(0, 0, 0, 0)
        self.inter_length = QtGui.QDoubleSpinBox()
        self.inter_length.setRange(0.3, 8.0)
        self.inter_length.setValue(round(self.hauteur_cone.value() * 0.30, 2))
        self.inter_length.setSuffix(" m")
        self.inter_length.setEnabled(False)
        inter_layout.addWidget(self.inter_length)
        self.inter_auto = QtGui.QCheckBox("Auto")
        self.inter_auto.setChecked(True)
        self.inter_auto.toggled.connect(lambda c: self.inter_length.setEnabled(not c))
        inter_layout.addWidget(self.inter_auto)
        lengths_layout.addRow("Mid:", inter_widget)
        
        # Uppers (shortest)
        hautes_widget = QtGui.QWidget()
        hautes_layout = QtGui.QHBoxLayout(hautes_widget)
        hautes_layout.setContentsMargins(0, 0, 0, 0)
        self.hautes_length = QtGui.QDoubleSpinBox()
        self.hautes_length.setRange(0.2, 5.0)
        self.hautes_length.setValue(round(self.hauteur_cone.value() * 0.20, 2))
        self.hautes_length.setSuffix(" m")
        self.hautes_length.setEnabled(False)
        hautes_layout.addWidget(self.hautes_length)
        self.hautes_auto = QtGui.QCheckBox("Auto")
        self.hautes_auto.setChecked(True)
        self.hautes_auto.toggled.connect(lambda c: self.hautes_length.setEnabled(not c))
        hautes_layout.addWidget(self.hautes_auto)
        lengths_layout.addRow("Uppers:", hautes_widget)
        
        # Help text for auto calculation
        self.lengths_help = QtGui.QLabel("Auto: Lowers=45%, Mid=30%, Uppers=20% of cone")
        self.lengths_help.setStyleSheet("color: gray; font-size: 10px;")
        lengths_layout.addRow("", self.lengths_help)
        
        # Fork angle auto mode
        fork_widget = QtGui.QWidget()
        fork_layout = QtGui.QHBoxLayout(fork_widget)
        fork_layout.setContentsMargins(0, 0, 0, 0)
        
        self.fork_angle_auto = QtGui.QCheckBox("By max fork angle")
        self.fork_angle_auto.setChecked(False)
        self.fork_angle_auto.toggled.connect(self._update_fork_angle_mode)
        fork_layout.addWidget(self.fork_angle_auto)
        
        self.fork_angle_max = QtGui.QDoubleSpinBox()
        self.fork_angle_max.setRange(5, 45)
        self.fork_angle_max.setValue(15.0)
        self.fork_angle_max.setSingleStep(1.0)
        self.fork_angle_max.setSuffix("°")
        self.fork_angle_max.setEnabled(False)
        fork_layout.addWidget(self.fork_angle_max)
        
        lengths_layout.addRow("", fork_widget)
        
        fork_help = QtGui.QLabel("Maximizes lowers to reduce drag while keeping fork angles within limit")
        fork_help.setStyleSheet("color: gray; font-size: 10px;")
        lengths_layout.addRow("", fork_help)
        
        main_layout.addWidget(lengths_group)
        
        # === Line Types ===
        lines_group = QtGui.QGroupBox("Lines (Type | Pos | /Cell | @Start | Patterns)")
        lines_layout = QtGui.QVBoxLayout(lines_group)
        
        self.line_type_rows = {}
        for lt in ["A", "B", "C", "D", "F"]:
            defaults = self.LINE_DEFAULTS[lt]
            row = LineTypeConfigRow(lt, defaults["position"], defaults["pattern"])
            row.interval_spinbox.setValue(defaults["interval"])
            self.line_type_rows[lt] = row
            lines_layout.addWidget(row)
            row.configChanged.connect(self._update_info)
        
        # Help text
        help_text = QtGui.QLabel("Patterns: 2:1 = 2 uppers→1 lower | 2:2:1 = 2+2 uppers→2 mid→1 lower")
        help_text.setStyleSheet("color: gray; font-size: 10px;")
        lines_layout.addWidget(help_text)
        
        main_layout.addWidget(lines_group)
        
        # === Mini-Pyramidales ===
        pyra_group = QtGui.QGroupBox("Mini-Pyramidales")
        pyra_layout = QtGui.QFormLayout(pyra_group)
        
        self.pyra_enable = QtGui.QCheckBox("Enable")
        self.pyra_enable.setChecked(False)
        pyra_layout.addRow("", self.pyra_enable)
        
        self.pyra_layer = QtGui.QComboBox()
        self.pyra_layer.addItems(["A", "B", "C", "D", "F"])
        self.pyra_layer.setCurrentText("A")
        pyra_layout.addRow("Layer:", self.pyra_layer)
        
        self.pyra_separation = QtGui.QDoubleSpinBox()
        self.pyra_separation.setRange(1.0, 15.0)
        self.pyra_separation.setValue(4.0)
        self.pyra_separation.setSingleStep(0.5)
        self.pyra_separation.setSuffix(" %")
        pyra_layout.addRow("Separation:", self.pyra_separation)
        
        self.pyra_length = QtGui.QDoubleSpinBox()
        self.pyra_length.setRange(0.05, 1.0)
        self.pyra_length.setValue(0.20)
        self.pyra_length.setSingleStep(0.05)
        self.pyra_length.setSuffix(" m")
        pyra_layout.addRow("Mini length:", self.pyra_length)
        
        self.pyra_groups = QtGui.QSpinBox()
        self.pyra_groups.setRange(1, 10)
        self.pyra_groups.setValue(2)
        pyra_layout.addRow("Nb groups:", self.pyra_groups)
        
        pyra_help = QtGui.QLabel("Splits each attachment point into 2 points on chord, connected by short lines")
        pyra_help.setStyleSheet("color: gray; font-size: 10px;")
        pyra_layout.addRow("", pyra_help)
        
        main_layout.addWidget(pyra_group)
        
        # === Stabilo ===
        stabilo_group = QtGui.QGroupBox("Stabilo")
        stabilo_layout = QtGui.QFormLayout(stabilo_group)
        
        self.stabilo_checkbox = QtGui.QCheckBox("Include")
        self.stabilo_checkbox.setChecked(True)
        self.stabilo_checkbox.toggled.connect(self._update_stabilo_enabled)
        stabilo_layout.addRow("", self.stabilo_checkbox)
        
        # Number of stabilo attachment points
        self.stabilo_count = QtGui.QSpinBox()
        self.stabilo_count.setRange(1, 5)
        self.stabilo_count.setValue(2)
        stabilo_layout.addRow("Nb points:", self.stabilo_count)
        
        self.stabilo_position = QtGui.QDoubleSpinBox()
        self.stabilo_position.setRange(0, 100)
        self.stabilo_position.setValue(50)
        self.stabilo_position.setSuffix(" %")
        stabilo_layout.addRow("Position:", self.stabilo_position)
        
        # Which riser to connect stabilo to
        self.stabilo_riser = QtGui.QComboBox()
        self.stabilo_riser.addItems(["A", "B", "C", "D", "F"])
        self.stabilo_riser.setCurrentText("A")  # Default to A riser
        stabilo_layout.addRow("Connect to:", self.stabilo_riser)
        
        main_layout.addWidget(stabilo_group)
        
        # === Material (per-level) ===
        material_group = QtGui.QGroupBox("Material")
        material_layout = QtGui.QFormLayout(material_group)
        
        line_type_names = sorted(LineType.types.keys())
        
        self.line_type_lowers = QtGui.QComboBox()
        for lt in line_type_names:
            self.line_type_lowers.addItem(lt)
        idx = self.line_type_lowers.findText("default")
        if idx >= 0:
            self.line_type_lowers.setCurrentIndex(idx)
        material_layout.addRow("Lowers:", self.line_type_lowers)
        
        self.line_type_mid = QtGui.QComboBox()
        for lt in line_type_names:
            self.line_type_mid.addItem(lt)
        idx = self.line_type_mid.findText("default")
        if idx >= 0:
            self.line_type_mid.setCurrentIndex(idx)
        material_layout.addRow("Mid:", self.line_type_mid)
        
        self.line_type_uppers = QtGui.QComboBox()
        for lt in line_type_names:
            self.line_type_uppers.addItem(lt)
        idx = self.line_type_uppers.findText("default")
        if idx >= 0:
            self.line_type_uppers.setCurrentIndex(idx)
        material_layout.addRow("Uppers:", self.line_type_uppers)
        
        main_layout.addWidget(material_group)
        
        # === Total Line Length (shown after generation) ===
        self.total_length_label = QtGui.QLabel("")
        self.total_length_label.setStyleSheet("font-size: 11px; color: #4CAF50; padding: 4px;")
        main_layout.addWidget(self.total_length_label)
        
        # === Buttons (outside scroll) ===
        button_layout = QtGui.QHBoxLayout()
        button_layout.addStretch()
        
        cancel_btn = QtGui.QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        
        apply_btn = QtGui.QPushButton("Apply")
        apply_btn.clicked.connect(self.accept)
        apply_btn.setDefault(True)
        button_layout.addWidget(apply_btn)
        
        outer_layout.addLayout(button_layout)
        
        # Info
        self.info_label = QtGui.QLabel("")
        self.info_label.setStyleSheet("color: gray;")
        outer_layout.addWidget(self.info_label)
        
        self._update_info()
    
    def _update_info(self):
        if not hasattr(self, 'info_label'):
            return
        count = 0
        groups = 0
        for lt, row in self.line_type_rows.items():
            if row.is_enabled():
                cfg = row.get_config()
                n = len(list(range(cfg["start_cell"], self.half_cell_num, cfg["interval"])))
                count += n
                groups += 1
        if hasattr(self, 'stabilo_checkbox') and self.stabilo_checkbox.isChecked():
            count += self.stabilo_count.value() if hasattr(self, 'stabilo_count') else 1
        self.info_label.setText(f"Points: {count} | Risers: {groups}")
    
    def _update_stabilo_enabled(self, enabled):
        """Update stabilo controls enabled state."""
        self.stabilo_count.setEnabled(enabled)
        self.stabilo_position.setEnabled(enabled)
        self.stabilo_riser.setEnabled(enabled)
    
    def _update_brake_enabled(self, enabled):
        """Update brake offset controls enabled state."""
        self.brake_offset_z.setEnabled(enabled)
        self.brake_offset_y.setEnabled(enabled)
        self.brake_offset_x.setEnabled(enabled)
    
    def _update_fork_angle_mode(self, checked):
        """Toggle fork angle auto mode."""
        self.fork_angle_max.setEnabled(checked)
        if checked:
            # Disable individual auto checkboxes
            self.basses_auto.setChecked(True)
            self.basses_auto.setEnabled(False)
            self.inter_auto.setChecked(True)
            self.inter_auto.setEnabled(False)
            self.hautes_auto.setChecked(True)
            self.hautes_auto.setEnabled(False)
            self.lengths_help.setText("Fork angle mode: maximizes lowers, minimizes uppers")
        else:
            self.basses_auto.setEnabled(True)
            self.inter_auto.setEnabled(True)
            self.hautes_auto.setEnabled(True)
            self.lengths_help.setText("Auto: Lowers=45%, Mid=30%, Uppers=20% of cone")
    
    def _update_hauteur_auto(self, auto):
        """Update hauteur cone when auto is toggled."""
        self.hauteur_cone.setEnabled(not auto)
        if auto:
            # Recalculate from full span (75% of wingspan)
            self.hauteur_cone.setValue(round(self.half_span * 2 * 0.75, 1))
    
    def _extract_from_existing_lineset(self):
        """Extract configuration from the existing lineset."""
        lineset = getattr(self.parametric_glider, 'lineset', None)
        if lineset is None:
            print("[AutoPlace] No lineset attribute on parametric_glider")
            return
        if not hasattr(lineset, 'lines') or not lineset.lines:
            print(f"[AutoPlace] Lineset has no lines (type={type(lineset).__name__})")
            return
        
        print(f"[AutoPlace] Found lineset with {len(lineset.lines)} lines, {len(lineset.nodes)} nodes")
        
        try:
            # Sort lines first (ensures lower_node/upper_node direction)
            for node in lineset.get_lower_attachment_points():
                lineset.sort_lines(node)
            
            # === Extract Lower Node (pilot point) ===
            lower_nodes = lineset.get_lower_attachment_points()
            print(f"[AutoPlace] Found {len(lower_nodes)} lower nodes")
            
            if lower_nodes:
                main_lower = lower_nodes[0]
                if hasattr(main_lower, 'pos_3D') and main_lower.pos_3D is not None:
                    px, py, pz = main_lower.pos_3D
                    print(f"[AutoPlace] Main lower pos_3D = [{px:.3f}, {py:.3f}, {pz:.3f}]")
                    self.demi_ecartement.setValue(abs(py) * 100)
                    self.profondeur.setValue((px / self.central_chord) * 100 if self.central_chord > 0.01 else 20.0)
                    self.hauteur_cone.setValue(abs(pz))
                    self.hauteur_auto.setChecked(False)
            
            # === Collect ALL upper nodes directly (most reliable) ===
            all_upper_nodes = lineset.get_upper_nodes()
            print(f"[AutoPlace] Found {len(all_upper_nodes)} upper nodes")
            
            # Group by layer
            layer_upper_nodes = {}
            for node in all_upper_nodes:
                layer = getattr(node, 'layer', '') or ''
                layer_upper_nodes.setdefault(layer, []).append(node)
            
            print(f"[AutoPlace] Layers found: {list(layer_upper_nodes.keys())}")
            
            # === Set upper node config per layer ===
            for layer, nodes in layer_upper_nodes.items():
                if layer not in self.line_type_rows:
                    print(f"[AutoPlace] Layer '{layer}' not in rows, skipping")
                    continue
                if not nodes:
                    continue
                
                row = self.line_type_rows[layer]
                row.enable_checkbox.setChecked(True)
                
                # Position: average rib_pos → convert to %
                avg_pos = sum(n.rib_pos for n in nodes) / len(nodes)
                if avg_pos <= 1.0:
                    avg_pos *= 100.0
                row.position_spinbox.setValue(round(avg_pos, 1))
                print(f"[AutoPlace] Layer {layer}: pos={avg_pos:.1f}%, {len(nodes)} nodes")
                
                # Interval from cell_no differences
                cell_nos = sorted(set(n.cell_no for n in nodes))
                if len(cell_nos) >= 2:
                    intervals = [cell_nos[i+1] - cell_nos[i] for i in range(len(cell_nos)-1)]
                    from collections import Counter
                    interval = Counter(intervals).most_common(1)[0][0]
                    row.interval_spinbox.setValue(max(interval, 1))
                    print(f"[AutoPlace] Layer {layer}: interval={interval}, cells={cell_nos}")
                
                if cell_nos:
                    row.start_spinbox.setValue(cell_nos[0])
            
            # === Extract patterns per layer ===
            # Build adjacency for tree walk
            upper_lines_adj = {}
            lower_lines_adj = {}
            for line in lineset.lines:
                upper_lines_adj.setdefault(id(line.lower_node), []).append(line)
                lower_lines_adj.setdefault(id(line.upper_node), []).append(line)
            
            for layer, nodes in layer_upper_nodes.items():
                if layer not in self.line_type_rows:
                    continue
                row = self.line_type_rows[layer]
                
                # For each upper node of this layer, walk DOWN to find the
                # basse node (first BatchNode2D directly below a LowerNode2D/riser)
                # then count how many upper nodes connect through each basse
                basse_groups = {}  # basse_node_id -> list of upper node counts per sub-fork
                
                def count_upper_children(node):
                    """Count UpperNode2D leaves reachable from this node."""
                    if isinstance(node, UpperNode2D):
                        return 1
                    children = upper_lines_adj.get(id(node), [])
                    return sum(count_upper_children(c.upper_node) for c in children)
                
                def get_fork_pattern(node, depth=0):
                    """Get the branching pattern from a basse node to upper nodes."""
                    children = upper_lines_adj.get(id(node), [])
                    if not children:
                        return ""
                    
                    # Filter to only children in this layer
                    layer_children = []
                    for c in children:
                        child_layer = getattr(c.upper_node, 'layer', getattr(c, 'layer', ''))
                        if child_layer == layer or isinstance(c.upper_node, BatchNode2D):
                            layer_children.append(c)
                    
                    if not layer_children:
                        return ""
                    
                    # If all children are UpperNode2D, this is a simple fork
                    if all(isinstance(c.upper_node, UpperNode2D) for c in layer_children):
                        return str(len(layer_children))
                    
                    # Mix of BatchNode2D and UpperNode2D — recurse
                    sub_counts = []
                    for c in layer_children:
                        if isinstance(c.upper_node, UpperNode2D):
                            sub_counts.append("1")
                        else:
                            sub = get_fork_pattern(c.upper_node, depth + 1)
                            if sub:
                                # Count children of this sub-fork
                                n_children = count_upper_children(c.upper_node)
                                sub_counts.append(str(n_children))
                    
                    if sub_counts:
                        return str(len(layer_children))
                    return ""
                
                # Find basse nodes for this layer (BatchNode2D whose parent is a riser/lower)
                group_patterns = []
                for lower in lineset.get_lower_attachment_points():
                    for riser_line in upper_lines_adj.get(id(lower), []):
                        riser = riser_line.upper_node
                        if not isinstance(riser, BatchNode2D):
                            continue
                        # Walk from riser to find basses for this layer
                        for basse_line in upper_lines_adj.get(id(riser), []):
                            basse = basse_line.upper_node
                            basse_layer = getattr(basse_line, 'layer', '')
                            if basse_layer != layer:
                                continue
                            # Count upper nodes from this basse
                            n_upper = count_upper_children(basse)
                            if n_upper > 0:
                                # Get the fork pattern
                                children = upper_lines_adj.get(id(basse), [])
                                n_children = len(children)
                                if n_children > 0 and n_upper > n_children:
                                    # Multi-level: e.g. 2:2:1 = 4 uppers via 2 inters of 2
                                    group_patterns.append(f"{n_children}:{n_upper // n_children}:1")
                                else:
                                    group_patterns.append(f"{n_upper}:1")
                
                if group_patterns:
                    pattern_str = ", ".join(group_patterns)
                    row.patterns_edit.setText(pattern_str)
                    print(f"[AutoPlace] Layer {layer}: pattern='{pattern_str}'")
            
            # Disable layers with no upper nodes
            for layer, row_widget in self.line_type_rows.items():
                if layer not in layer_upper_nodes:
                    row_widget.enable_checkbox.setChecked(False)
            
            # === Extract lengths by tree walk ===
            upper_lines = {}
            for line in lineset.lines:
                upper_lines.setdefault(id(line.lower_node), []).append(line)
            
            riser_lengths, basses_lengths, inter_lengths, hautes_lengths = [], [], [], []
            riser_types, basses_types, inter_types, hautes_types = [], [], [], []
            
            def classify_line(line, depth):
                tl = line.target_length
                lt_name = line.line_type.name if hasattr(line.line_type, 'name') else str(line.line_type)
                
                if isinstance(line.upper_node, UpperNode2D):
                    hautes_lengths.append(tl)
                    hautes_types.append(lt_name)
                elif isinstance(line.upper_node, BatchNode2D):
                    if depth == 0:
                        riser_lengths.append(tl)
                        riser_types.append(lt_name)
                    elif depth == 1:
                        basses_lengths.append(tl)
                        basses_types.append(lt_name)
                    else:
                        inter_lengths.append(tl)
                        inter_types.append(lt_name)
                    
                    for child in upper_lines.get(id(line.upper_node), []):
                        classify_line(child, depth + 1)
            
            for lower in lower_nodes:
                for line in upper_lines.get(id(lower), []):
                    classify_line(line, 0)
            
            print(f"[AutoPlace] Lengths: riser={len(riser_lengths)}, basses={len(basses_lengths)}, "
                  f"inter={len(inter_lengths)}, hautes={len(hautes_lengths)}")
            
            def median(lst):
                s = sorted([x for x in lst if x is not None])
                return s[len(s) // 2] if s else None
            
            def mode_str(lst):
                if not lst:
                    return None
                from collections import Counter
                return Counter(lst).most_common(1)[0][0]
            
            rl = median(riser_lengths)
            if rl is not None:
                self.riser_length.setValue(rl)
            bl = median(basses_lengths)
            if bl is not None:
                self.basses_length.setValue(bl)
                self.basses_auto.setChecked(False)
            il = median(inter_lengths)
            if il is not None:
                self.inter_length.setValue(il)
                self.inter_auto.setChecked(False)
            hl = median(hautes_lengths)
            if hl is not None:
                self.hautes_length.setValue(hl)
                self.hautes_auto.setChecked(False)
            
            def set_combo(combo, type_name):
                if type_name:
                    idx = combo.findText(type_name)
                    if idx >= 0:
                        combo.setCurrentIndex(idx)
            
            set_combo(self.line_type_lowers, mode_str(basses_types + riser_types))
            set_combo(self.line_type_mid, mode_str(inter_types))
            set_combo(self.line_type_uppers, mode_str(hautes_types))
            
            print("[AutoPlace] Extraction complete")
                    
        except Exception as e:
            import traceback
            print(f"[AutoPlace] ERROR: {e}")
            traceback.print_exc()
    
    def load_from_glider(self):
        """Load configuration from ParametricGlider if available."""
        config = getattr(self.parametric_glider, 'lines_placement_config', None)
        if not config:
            return
        
        # Main Point
        if 'demi_ecartement' in config:
            val = config['demi_ecartement']
            # Backward compat: if value is small (< 5), it was in meters -> convert to cm
            if val < 5:
                val = val * 100
            self.demi_ecartement.setValue(val)
        if 'profondeur' in config:
            val = config['profondeur']
            # Backward compat: if value is small (< 5), it was in meters -> convert to % of central chord
            if abs(val) < 5:
                val = (val / self.central_chord) * 100
            self.profondeur.setValue(val)
        if 'hauteur_auto' in config:
            self.hauteur_auto.setChecked(config['hauteur_auto'])
        if 'hauteur_cone' in config:
            self.hauteur_cone.setValue(config['hauteur_cone'])
        
        # Brake Point
        if 'brake_separate' in config:
            self.brake_separate.setChecked(config['brake_separate'])
        if 'brake_offset_z' in config:
            self.brake_offset_z.setValue(config['brake_offset_z'])
        if 'brake_offset_y' in config:
            self.brake_offset_y.setValue(config['brake_offset_y'])
        if 'brake_offset_x' in config:
            self.brake_offset_x.setValue(config['brake_offset_x'])
        
        # Lengths
        if 'riser_length' in config:
            self.riser_length.setValue(config['riser_length'])
        if 'basses_auto' in config:
            self.basses_auto.setChecked(config['basses_auto'])
        if 'basses_length' in config:
            self.basses_length.setValue(config['basses_length'])
        if 'inter_auto' in config:
            self.inter_auto.setChecked(config['inter_auto'])
        if 'inter_length' in config:
            self.inter_length.setValue(config['inter_length'])
        if 'hautes_auto' in config:
            self.hautes_auto.setChecked(config['hautes_auto'])
        if 'hautes_length' in config:
            self.hautes_length.setValue(config['hautes_length'])
        
        # Line types
        if 'line_types' in config:
            for lt, lt_config in config['line_types'].items():
                if lt in self.line_type_rows:
                    row = self.line_type_rows[lt]
                    if 'enabled' in lt_config:
                        row.enable_checkbox.setChecked(lt_config['enabled'])
                    if 'position' in lt_config:
                        pos_val = lt_config['position']
                        # Backward compat: if stored as fraction (<= 1.0), convert to %
                        if pos_val <= 1.0 and pos_val > 0:
                            pos_val *= 100.0
                        row.position_spinbox.setValue(pos_val)
                    if 'interval' in lt_config:
                        row.interval_spinbox.setValue(lt_config['interval'])
                    if 'start_cell' in lt_config:
                        row.start_spinbox.setValue(lt_config['start_cell'])
                    if 'patterns' in lt_config:
                        row.patterns_edit.setText(lt_config['patterns'])
        
        # Stabilo
        if 'include_stabilo' in config:
            self.stabilo_checkbox.setChecked(config['include_stabilo'])
        if 'stabilo_count' in config:
            self.stabilo_count.setValue(config['stabilo_count'])
        if 'stabilo_position' in config:
            self.stabilo_position.setValue(config['stabilo_position'])
        if 'stabilo_riser' in config:
            idx = self.stabilo_riser.findText(config['stabilo_riser'])
            if idx >= 0:
                self.stabilo_riser.setCurrentIndex(idx)
        
        # Material (per-level)
        for key, combo in [('line_type_lowers', self.line_type_lowers),
                           ('line_type_mid', self.line_type_mid),
                           ('line_type_uppers', self.line_type_uppers)]:
            if key in config:
                idx = combo.findText(config[key])
                if idx >= 0:
                    combo.setCurrentIndex(idx)
        # Backward compat: single line_type_name -> apply to all levels
        if 'line_type_name' in config and 'line_type_lowers' not in config:
            for combo in [self.line_type_lowers, self.line_type_mid, self.line_type_uppers]:
                idx = combo.findText(config['line_type_name'])
                if idx >= 0:
                    combo.setCurrentIndex(idx)
        
        # Fork angle
        if 'fork_angle_auto' in config:
            self.fork_angle_auto.setChecked(config['fork_angle_auto'])
        if 'fork_angle_max' in config:
            self.fork_angle_max.setValue(config['fork_angle_max'])
        
        # Mini-pyramidales
        if 'pyra_enable' in config:
            self.pyra_enable.setChecked(config['pyra_enable'])
        if 'pyra_layer' in config:
            self.pyra_layer.setCurrentText(config['pyra_layer'])
        if 'pyra_separation' in config:
            self.pyra_separation.setValue(config['pyra_separation'])
        if 'pyra_length' in config:
            self.pyra_length.setValue(config['pyra_length'])
        if 'pyra_groups' in config:
            self.pyra_groups.setValue(config['pyra_groups'])
        
        self._update_info()
    
    def save_to_glider(self):
        """Save configuration to ParametricGlider for persistence."""
        config = {
            "demi_ecartement": self.demi_ecartement.value(),  # Now in cm
            "profondeur": self.profondeur.value(),  # Now in % of central chord
            "hauteur_auto": self.hauteur_auto.isChecked(),
            "hauteur_cone": self.hauteur_cone.value(),
            "brake_separate": self.brake_separate.isChecked(),
            "brake_offset_z": self.brake_offset_z.value(),
            "brake_offset_y": self.brake_offset_y.value(),
            "brake_offset_x": self.brake_offset_x.value(),
            "riser_length": self.riser_length.value(),
            "basses_auto": self.basses_auto.isChecked(),
            "basses_length": self.basses_length.value(),
            "inter_auto": self.inter_auto.isChecked(),
            "inter_length": self.inter_length.value(),
            "hautes_auto": self.hautes_auto.isChecked(),
            "hautes_length": self.hautes_length.value(),
            "include_stabilo": self.stabilo_checkbox.isChecked(),
            "stabilo_count": self.stabilo_count.value(),
            "stabilo_position": self.stabilo_position.value(),
            "stabilo_riser": self.stabilo_riser.currentText(),
            "line_type_lowers": self.line_type_lowers.currentText(),
            "line_type_mid": self.line_type_mid.currentText(),
            "line_type_uppers": self.line_type_uppers.currentText(),
            "fork_angle_auto": self.fork_angle_auto.isChecked(),
            "fork_angle_max": self.fork_angle_max.value(),
            "pyra_enable": self.pyra_enable.isChecked(),
            "pyra_layer": self.pyra_layer.currentText(),
            "pyra_separation": self.pyra_separation.value(),
            "pyra_length": self.pyra_length.value(),
            "pyra_groups": self.pyra_groups.value(),
            "line_types": {}
        }
        
        for lt, row in self.line_type_rows.items():
            config["line_types"][lt] = {
                "enabled": row.enable_checkbox.isChecked(),
                "position": row.position_spinbox.value(),
                "interval": row.interval_spinbox.value(),
                "start_cell": row.start_spinbox.value(),
                "patterns": row.patterns_edit.text(),
            }
        
        self.parametric_glider.lines_placement_config = config
    
    def get_configuration(self):
        import math
        
        # Calculate cone height - auto uses full span
        if self.hauteur_auto.isChecked():
            # ~75% of full wingspan (2 * half_span)
            hauteur = round(self.half_span * 2 * 0.75, 2)
        else:
            hauteur = self.hauteur_cone.value()
        
        if self.fork_angle_auto.isChecked():
            # Fork angle mode: minimize total line length
            # For each fork, the angle between two child lines must be <= max_angle
            # L_min = spacing / (2 * tan(max_angle/2))
            # Spacings computed from TRUE 3D positions (arc + shape)
            max_angle_rad = math.radians(self.fork_angle_max.value())
            half_tan = math.tan(max_angle_rad / 2)
            
            # Get 3D arc positions for each rib: [y_projected, z_arc]
            try:
                x_values = self.parametric_glider.shape.rib_x_values
                arc_positions = list(self.parametric_glider.arc.get_arc_positions(x_values))
                has_3d = True
                print(f"[AutoPlace] Using 3D positions ({len(arc_positions)} ribs, "
                      f"span={arc_positions[-1][0]:.2f}m)")
            except Exception as e:
                print(f"[AutoPlace] Could not get arc positions: {e}, falling back to 2D")
                arc_positions = None
                has_3d = False
            
            def get_node_3d(node):
                """Get approximate 3D position of an upper node: [chord_x, span_y, height_z]"""
                cell = min(node.cell_no, self.half_cell_num - 1)
                shape_pos = self.parametric_glider.shape[cell, node.rib_pos]  # [span_x, chord_y]
                
                if has_3d and cell < len(arc_positions):
                    # Arc position for this rib: [y_projected, z_arc]
                    arc_y = arc_positions[cell][0]
                    arc_z = arc_positions[cell][1]
                    # 3D position: chord along wing, span with arc, height from arc
                    return [shape_pos[1], arc_y, arc_z]  # [chord, y, z]
                else:
                    return [shape_pos[1], shape_pos[0], 0]  # Fallback 2D
            
            def dist_3d(p1, p2):
                """3D distance between two points."""
                return math.sqrt(sum((a - b)**2 for a, b in zip(p1, p2)))
            
            # Compute spacings at each fork level using 3D distances
            max_hautes_spacing = 0.0
            max_inter_spacing = 0.0
            max_basses_spacing = 0.0
            
            enabled_types = [lt for lt in ["A", "B", "C", "D", "F"]
                           if self.line_type_rows[lt].enable_checkbox.isChecked()]
            
            for lt in enabled_types:
                row = self.line_type_rows[lt]
                lt_config = row.get_config()
                if not lt_config["enabled"]:
                    continue
                
                upper_nodes = self._generate_upper_nodes(lt, lt_config)
                if len(upper_nodes) < 2:
                    continue
                
                # Get 3D positions for each upper node
                positions_3d = [get_node_3d(n) for n in upper_nodes]
                
                group_patterns = lt_config["group_patterns"]
                
                node_idx = 0
                group_idx = 0
                group_center_positions = []
                
                while node_idx < len(positions_3d):
                    pattern = group_patterns[group_idx % len(group_patterns)]
                    nodes_per = self._calc_nodes_for_pattern(pattern)
                    group_pos = positions_3d[node_idx:node_idx + nodes_per]
                    
                    if not group_pos:
                        break
                    
                    # Hautes: 3D spacing within sub-groups
                    if len(pattern) > 1:
                        sub_size = pattern[0]
                        sub_centers_3d = []
                        for j in range(0, len(group_pos), sub_size):
                            sub = group_pos[j:j + sub_size]
                            if len(sub) >= 2:
                                for k in range(len(sub) - 1):
                                    s = dist_3d(sub[k], sub[k + 1])
                                    max_hautes_spacing = max(max_hautes_spacing, s)
                            if sub:
                                center = [sum(p[d] for p in sub) / len(sub) for d in range(3)]
                                sub_centers_3d.append(center)
                        # Inter: 3D distance between sub-group centers
                        for j in range(len(sub_centers_3d) - 1):
                            s = dist_3d(sub_centers_3d[j], sub_centers_3d[j + 1])
                            max_inter_spacing = max(max_inter_spacing, s)
                    else:
                        # Simple pattern: spacing between consecutive nodes
                        for i in range(len(group_pos) - 1):
                            s = dist_3d(group_pos[i], group_pos[i + 1])
                            max_hautes_spacing = max(max_hautes_spacing, s)
                    
                    # Group center for basses level
                    center = [sum(p[d] for p in group_pos) / len(group_pos) for d in range(3)]
                    group_center_positions.append(center)
                    
                    node_idx += len(group_pos)
                    group_idx += 1
                
                # Basses: 3D distance between group centers
                for i in range(len(group_center_positions) - 1):
                    s = dist_3d(group_center_positions[i], group_center_positions[i + 1])
                    max_basses_spacing = max(max_basses_spacing, s)
            
            # L_min = spacing / (2 * tan(max_angle/2))
            hautes = max(round(max_hautes_spacing / (2 * half_tan), 2), 0.20) if max_hautes_spacing > 0 else 0.30
            inter_min = max(round(max_inter_spacing / (2 * half_tan), 2), 0.20) if max_inter_spacing > 0 else 0.30
            basses_min = max(round(max_basses_spacing / (2 * half_tan), 2), 0.30) if max_basses_spacing > 0 else 0.30
            
            # Fixed cone height: hautes has priority, inter/basses absorb the rest
            riser_len = self.riser_length.value()
            remaining = hauteur - riser_len - hautes  # What's left for inter + basses
            
            if remaining >= inter_min + basses_min:
                # Everything fits: use ideal values, basses takes the leftover
                inter = inter_min
                basses = remaining - inter
            elif remaining > 0:
                # Not enough for both at ideal — distribute proportionally
                total_min = inter_min + basses_min
                inter = round(remaining * (inter_min / total_min), 2)
                basses = round(remaining - inter, 2)
                actual_inter_angle = math.degrees(2 * math.atan(max_inter_spacing / (2 * inter))) if inter > 0.01 else 90
                actual_basses_angle = math.degrees(2 * math.atan(max_basses_spacing / (2 * basses))) if basses > 0.01 else 90
                print(f"[AutoPlace] WARNING: Cone too short for {self.fork_angle_max.value()}° everywhere. "
                      f"Hautes={self.fork_angle_max.value()}°, Inter≈{actual_inter_angle:.0f}°, Basses≈{actual_basses_angle:.0f}°")
            else:
                # Even hautes alone exceeds available — shouldn't happen often
                inter = 0.20
                basses = 0.30
                print("[AutoPlace] WARNING: Cone much too short for fork angle constraint!")
            
            print(f"[AutoPlace] Fork angle: spacings h={max_hautes_spacing:.3f} i={max_inter_spacing:.3f} b={max_basses_spacing:.3f}")
            print(f"[AutoPlace] Fork angle: lengths h={hautes:.2f} i={inter:.2f} b={basses:.2f} (cone={hauteur:.1f}, riser={riser_len:.2f})")
        else:
            # Standard percentage mode
            # Lowers (longest): 45% of cone height
            basses = self.basses_length.value()
            if self.basses_auto.isChecked():
                basses = round(hauteur * 0.45, 2)
            
            # Mid (medium): 30% of cone height
            inter = self.inter_length.value()
            if self.inter_auto.isChecked():
                inter = round(hauteur * 0.30, 2)
            
            # Uppers (shortest): 20% of cone height  
            hautes = self.hautes_length.value()
            if self.hautes_auto.isChecked():
                hautes = round(hauteur * 0.20, 2)
        
        # Convert depth from % of central chord to meters
        depth_pct = self.profondeur.value()
        depth_m = (depth_pct / 100.0) * self.central_chord
        
        # Convert half-span spread from cm to meters
        half_spread_m = self.demi_ecartement.value() / 100.0
        
        return {
            "demi_ecartement": half_spread_m,             # X - span (in m)
            "profondeur": depth_m,                        # Y - chord (in m, converted from %)
            "hauteur_cone": hauteur,                      # Z - height
            "brake_separate": self.brake_separate.isChecked(),
            "brake_offset_z": self.brake_offset_z.value(),    # Higher than main
            "brake_offset_y": self.brake_offset_y.value(),    # More outward than main
            "brake_offset_x": self.brake_offset_x.value(),    # Depth offset from main
            "riser_length": self.riser_length.value(),
            "basses_length": basses,
            "inter_length": inter,
            "hautes_length": hautes,
            "line_types": {lt: row.get_config() for lt, row in self.line_type_rows.items()},
            "line_type_lowers": self.line_type_lowers.currentText(),
            "line_type_mid": self.line_type_mid.currentText(),
            "line_type_uppers": self.line_type_uppers.currentText(),
            "include_stabilo": self.stabilo_checkbox.isChecked(),
            "stabilo_count": self.stabilo_count.value(),
            "stabilo_position": self.stabilo_position.value() / 100.0,
            "stabilo_riser": self.stabilo_riser.currentText(),
        }
    
    def generate_lineset(self):
        """Generate LineSet2D."""
        self.save_to_glider()  # Persist configuration for next time
        config = self.get_configuration()
        lines = []
        
        # Main Point coordinates (values already converted to meters)
        lower_x = config["profondeur"]
        lower_y = config["demi_ecartement"]
        lower_z = -config["hauteur_cone"]
        
        # Single main lower node
        main_lower = LowerNode2D(
            pos_2D=[lower_y, lower_z],  # [span, height] for 2D view
            pos_3D=[lower_x, lower_y, lower_z],
            name="main",
            layer="",
        )
        
        # Brake lower node - separate if enabled
        brake_lower = main_lower
        if config["brake_separate"]:
            brake_z = lower_z + config["brake_offset_z"]
            brake_y = lower_y + config["brake_offset_y"]
            brake_x = lower_x + config["brake_offset_x"]
            brake_lower = LowerNode2D(
                pos_2D=[brake_y, brake_z],  # [span, height]
                pos_3D=[brake_x, brake_y, brake_z],
                name="brake",
                layer="F",
            )
        
        # Get enabled types
        enabled = [lt for lt in ["A", "B", "C", "D", "F"] 
                  if config["line_types"][lt]["enabled"]]
        
        # Calculate layout parameters
        try:
            shape = self.parametric_glider.shape.get_half_shape()
            x_extent = abs(shape.front[-1][0] - shape.front[0][0])
        except:
            x_extent = 6.0
        
        riser_base_z = lower_z + config["riser_length"]
        riser_nodes = {}
        
        for i, lt in enumerate(enabled):
            # Compute average span position of this line type's upper nodes
            lt_config_temp = config["line_types"][lt]
            temp_nodes = self._generate_upper_nodes(lt, lt_config_temp)
            if temp_nodes:
                riser_span = self._calc_span_position(temp_nodes)
            else:
                riser_span = lower_y
            
            lower_node_for_riser = brake_lower if lt == "F" else main_lower
            actual_lower_z = lower_node_for_riser.pos_3D[2] if hasattr(lower_node_for_riser, 'pos_3D') else lower_z
            
            riser = BatchNode2D(
                pos_2D=[riser_span, actual_lower_z + config["riser_length"]],  # [avg span, riser height]
                name=f"riser_{lt}",
                layer=lt,
            )
            riser_nodes[lt] = riser
            lines.append(Line2D(
                lower_node=lower_node_for_riser,
                upper_node=riser,
                target_length=config["riser_length"],
                line_type=config["line_type_lowers"],  # Risers use lowers material
                layer=lt,
                name=f"riser_{lt}",
            ))
        
        # Generate lines for each type with per-level materials
        for lt in enabled:
            lt_config = config["line_types"][lt]
            upper_nodes = self._generate_upper_nodes(lt, lt_config)
            
            if upper_nodes:
                lt_lines = self._generate_grouped_architecture(
                    upper_nodes=upper_nodes,
                    riser_node=riser_nodes[lt],
                    group_patterns=lt_config["group_patterns"],
                    basses_length=config["basses_length"],
                    inter_length=config["inter_length"],
                    hautes_length=config["hautes_length"],
                    line_type_lowers=config["line_type_lowers"],
                    line_type_mid=config["line_type_mid"],
                    line_type_uppers=config["line_type_uppers"],
                    layer=lt,
                )
                lines.extend(lt_lines)
        
        # Stabilo - connects to an existing riser, not to pilot point directly
        if config["include_stabilo"]:
            stabilo_riser_name = config["stabilo_riser"]
            stabilo_count = config["stabilo_count"]
            
            # Find the riser to connect to
            if stabilo_riser_name in riser_nodes:
                stabilo_riser = riser_nodes[stabilo_riser_name]
            else:
                # Fallback: use first available riser or create one
                stabilo_riser = list(riser_nodes.values())[0] if riser_nodes else main_lower # Fallback to main_lower if no risers
            
            if stabilo_riser:
                for i in range(stabilo_count):
                    # Distribute points on the outermost cells
                    cell_no = self.half_cell_num - 1 - i
                    if cell_no < 0:
                        cell_no = 0
                    
                    stabilo = UpperNode2D(
                        cell_no=cell_no,
                        rib_pos=config["stabilo_position"],
                        cell_pos=0,
                        force=1.0,
                        name=f"S{i + 1}",
                        layer="S",
                    )
                    lines.append(Line2D(
                        lower_node=stabilo_riser,
                        upper_node=stabilo,
                        target_length=config["hautes_length"],
                        line_type=config["line_type_uppers"],
                        layer="S",
                        name=f"S{i + 1}",
                    ))
        
        lineset = LineSet2D(lines)
        
        # Calculate and display total line length
        total_length = sum(getattr(line, 'target_length', 0) for line in lines)
        self.total_length_label.setText(
            f"<b>Total line length: {total_length:.1f} m</b>  "
            f"(Lowers: {config['basses_length']:.2f} m, Mid: {config['inter_length']:.2f} m, Uppers: {config['hautes_length']:.2f} m)"
        )
        
        return lineset
    
    def _generate_upper_nodes(self, line_type, config):
        """Generate upper attachment points."""
        nodes = []
        start = config["start_cell"]
        interval = config["interval"]
        position = config["position"] / 100.0  # Convert % to fraction (0-1)
        
        idx = 1
        for cell_no in range(start, self.half_cell_num, interval):
            # Ensure cell_no is valid
            if cell_no >= self.half_cell_num:
                break
            node = UpperNode2D(
                cell_no=cell_no,
                rib_pos=position,
                cell_pos=0,
                force=1.0,
                name=f"{line_type}{idx}",
                layer=line_type,
            )
            nodes.append(node)
            idx += 1
        
        return nodes
    
    def _generate_grouped_architecture(self, upper_nodes, riser_node, group_patterns,
                                       basses_length, inter_length, hautes_length,
                                       line_type_lowers, line_type_mid, line_type_uppers, layer):
        """
        Generate architecture with groups.
        Each group = 1 basse connected to riser.
        Pattern defines how hautes connect to basse (via inter if needed).
        """
        lines = []
        
        if not upper_nodes:
            return lines
        
        # Get riser position for reference
        riser_pos = riser_node.pos_2D if hasattr(riser_node, 'pos_2D') else [0, 0]
        
        # Mini-pyramidal config
        pyra_enabled = self.pyra_enable.isChecked() and self.pyra_layer.currentText() == layer
        pyra_separation = self.pyra_separation.value() / 100.0  # % to fraction
        pyra_length = self.pyra_length.value()
        pyra_max_groups = self.pyra_groups.value()
        print(f"[AutoPlace] Pyramidal: enabled={pyra_enabled} (checked={self.pyra_enable.isChecked()}, layer_ui={self.pyra_layer.currentText()}, layer_arg={layer}), sep={pyra_separation:.3f}, len={pyra_length}, groups={pyra_max_groups}")
        
        # Total number of groups for spacing calculation
        total_groups = 0
        temp_idx = 0
        for g in range(len(upper_nodes)):
            pattern = group_patterns[total_groups % len(group_patterns)]
            nodes_per = self._calc_nodes_for_pattern(pattern)
            if temp_idx >= len(upper_nodes):
                break
            temp_idx += nodes_per
            total_groups += 1
        
        node_idx = 0
        group_idx = 0
        
        while node_idx < len(upper_nodes):
            # Get pattern for this group (cycle through patterns)
            pattern = group_patterns[group_idx % len(group_patterns)]
            
            # Calculate how many nodes this pattern consumes
            nodes_per_group = self._calc_nodes_for_pattern(pattern)
            
            # Get nodes for this group
            group_nodes = upper_nodes[node_idx:node_idx + nodes_per_group]
            if not group_nodes:
                break
            
            # Calculate basse position: X = avg span, Y = interpolated from riser to wing chord
            upper_avg_pos = self._calc_batch_position(group_nodes, 0)
            upper_span = upper_avg_pos[0]  # span from shape
            
            # Y: interpolate from riser toward wing chord position
            total_line_height = basses_length + inter_length + hautes_length
            basse_y_ratio = basses_length / total_line_height if total_line_height > 0 else 0.33
            basse_y = riser_pos[1] + (upper_avg_pos[1] - riser_pos[1]) * basse_y_ratio
            
            basse_node = BatchNode2D(
                pos_2D=[upper_span, basse_y],
                name=f"{layer}{group_idx + 1}_basse",
                layer=layer,
            )
            
            # Connect basse to riser (lowers material)
            lines.append(Line2D(
                lower_node=riser_node,
                upper_node=basse_node,
                target_length=basses_length,
                line_type=line_type_lowers,
                layer=layer,
                name=f"{layer}{group_idx + 1}",
            ))
            
            # Generate architecture within group
            group_lines = self._generate_group_architecture(
                group_nodes, basse_node, pattern, inter_length, hautes_length,
                line_type_mid, line_type_uppers, layer, group_idx
            )
            
            # === Mini-pyramidales: insert BatchNode2D level between haute and wing ===
            if pyra_enabled and group_idx < pyra_max_groups:
                print(f"[AutoPlace] Applying pyramidal to group {group_idx} ({len(group_lines)} lines, {sum(1 for l in group_lines if isinstance(l.upper_node, UpperNode2D))} to wing)")
                new_lines = []
                for line in group_lines:
                    if isinstance(line.upper_node, UpperNode2D):
                        # This haute line goes to the wing → insert pyramidal level
                        orig_node = line.upper_node
                        front_pos = max(orig_node.rib_pos - pyra_separation / 2, 0.0)
                        back_pos = min(orig_node.rib_pos + pyra_separation / 2, 1.0)
                        
                        front_node = UpperNode2D(
                            cell_no=orig_node.cell_no, rib_pos=front_pos,
                            cell_pos=orig_node.cell_pos, force=orig_node.force,
                            name=f"{orig_node.name}_pf", layer=orig_node.layer,
                        )
                        back_node = UpperNode2D(
                            cell_no=orig_node.cell_no, rib_pos=back_pos,
                            cell_pos=orig_node.cell_pos, force=orig_node.force,
                            name=f"{orig_node.name}_pb", layer=orig_node.layer,
                        )
                        
                        # Pyramidal batch node between haute and wing
                        shape_pos = list(orig_node.get_2D(self.parametric_glider.shape))
                        shape_pos[1] -= pyra_length * 0.5  # Offset below wing in 2D
                        pyra_batch = BatchNode2D(
                            pos_2D=shape_pos,
                            name=f"{orig_node.name}_pyra", layer=orig_node.layer,
                        )
                        
                        # 1. Shortened haute: lower→pyra_batch (keep original length minus pyra)
                        shortened = max(line.target_length - pyra_length, 0.10)
                        lt_name = line.line_type.name if hasattr(line.line_type, 'name') else line.line_type
                        new_lines.append(Line2D(
                            lower_node=line.lower_node,
                            upper_node=pyra_batch,
                            target_length=shortened,
                            line_type=lt_name, layer=layer,
                            name=line.name,
                        ))
                        # 2. Pyra→front
                        new_lines.append(Line2D(
                            lower_node=pyra_batch,
                            upper_node=front_node,
                            target_length=pyra_length,
                            line_type=line_type_uppers, layer=layer,
                            name=f"{orig_node.name}_pf",
                        ))
                        # 3. Pyra→back
                        new_lines.append(Line2D(
                            lower_node=pyra_batch,
                            upper_node=back_node,
                            target_length=pyra_length,
                            line_type=line_type_uppers, layer=layer,
                            name=f"{orig_node.name}_pb",
                        ))
                    else:
                        new_lines.append(line)
                group_lines = new_lines
            
            lines.extend(group_lines)
            
            node_idx += len(upper_nodes[node_idx:node_idx + nodes_per_group])
            group_idx += 1
        
        return lines
    
    def _calc_nodes_for_pattern(self, pattern):
        """Calculate how many upper nodes a pattern consumes."""
        if pattern == [1]:
            return 1
        
        # Pattern [2] = 2 nodes
        # Pattern [2, 2] = 2*2 = 4 nodes
        # Pattern [3] = 3 nodes
        # Pattern [3, 2] = 3*2 = 6 nodes
        result = 1
        for p in pattern:
            result *= p
        return result
    
    def _generate_group_architecture(self, nodes, basse_node, pattern, 
                                    inter_length, hautes_length,
                                    line_type_mid, line_type_uppers, layer, group_idx):
        """
        Generate lines within a group.
        - hautes_length: length of lines connecting to wing (uppermost) → uppers material
        - inter_length: length of intermediate lines → mid material
        """
        lines = []
        
        # Get basse position for reference
        basse_pos = basse_node.pos_2D if hasattr(basse_node, 'pos_2D') else [0, 0]
        
        if pattern == [1] or len(nodes) == 1:
            # Direct connection: use uppers material (basse to wing)
            for i, node in enumerate(nodes):
                lines.append(Line2D(
                    lower_node=basse_node,
                    upper_node=node,
                    target_length=hautes_length,
                    line_type=line_type_uppers,
                    layer=layer,
                    name=node.name,
                ))
            return lines
        
        if len(pattern) == 1:
            # Simple pattern like 2:1 or 3:1: uppers go directly from basse to wing
            merge = pattern[0]
            for i, node in enumerate(nodes):
                lines.append(Line2D(
                    lower_node=basse_node,
                    upper_node=node,
                    target_length=hautes_length,
                    line_type=line_type_uppers,
                    layer=layer,
                    name=node.name,
                ))
            return lines
        
        # Multi-level pattern like 2:2:1
        current_nodes = list(nodes)
        total_levels = len(pattern) - 1
        
        for level, merge in enumerate(pattern[:-1]):
            next_nodes = []
            
            for i in range(0, len(current_nodes), merge):
                group = current_nodes[i:i + merge]
                if len(group) == 1:
                    next_nodes.append(group[0])
                    continue
                
                upper_avg_pos = self._calc_batch_position(group, level)
                upper_span = upper_avg_pos[0]  # span from shape
                
                # Y: interpolate from basse toward chord position of this sub-group
                level_ratio = (level + 1) / (total_levels + 1)
                inter_y = basse_pos[1] + (upper_avg_pos[1] - basse_pos[1]) * level_ratio
                
                inter_node = BatchNode2D(
                    pos_2D=[upper_span, inter_y],
                    name=f"{layer}{group_idx + 1}_i{level}_{i // merge}",
                    layer=layer,
                )
                
                # First level (level 0) connects to wing = uppers material
                # Other levels use mid material
                line_length = hautes_length if level == 0 else inter_length
                line_mat = line_type_uppers if level == 0 else line_type_mid
                
                for node in group:
                    lines.append(Line2D(
                        lower_node=inter_node,
                        upper_node=node,
                        target_length=line_length,
                        line_type=line_mat,
                        layer=layer,
                        name=f"{node.name}_h",
                    ))
                
                next_nodes.append(inter_node)
            
            current_nodes = next_nodes
        
        # Connect remaining inter nodes to basse with mid material
        for node in current_nodes:
            lines.append(Line2D(
                lower_node=basse_node,
                upper_node=node,
                target_length=inter_length,
                line_type=line_type_mid,
                layer=layer,
                name=f"{layer}{group_idx + 1}_to_basse",
            ))
        
        return lines
    
    def _calc_batch_position(self, nodes, level):
        """Calculate average 2D position for batch node."""
        positions = []
        
        for n in nodes:
            if isinstance(n, UpperNode2D):
                try:
                    cell = min(n.cell_no, self.half_cell_num - 1)
                    pos = list(self.parametric_glider.shape[cell, n.rib_pos])
                    positions.append(pos)
                except:
                    positions.append([n.cell_no * 0.5, 0])
            elif hasattr(n, 'pos_2D'):
                positions.append(list(n.pos_2D))
        
        if positions:
            return [
                sum(p[0] for p in positions) / len(positions),
                sum(p[1] for p in positions) / len(positions),
            ]
        return [0, 0]
    
    def _calc_span_position(self, nodes):
        """Calculate average span (X) position for a set of nodes.
        
        Uses shape[cell, rib_pos][0] for UpperNode2D (the span coordinate).
        For BatchNode2D, uses pos_2D[0].
        """
        spans = []
        for n in nodes:
            if isinstance(n, UpperNode2D):
                try:
                    cell = min(n.cell_no, self.half_cell_num - 1)
                    pos = self.parametric_glider.shape[cell, n.rib_pos]
                    spans.append(pos[0])
                except:
                    spans.append(n.cell_no * 0.5)
            elif hasattr(n, 'pos_2D'):
                spans.append(n.pos_2D[0])
        
        if spans:
            return sum(spans) / len(spans)
        return 0.0

