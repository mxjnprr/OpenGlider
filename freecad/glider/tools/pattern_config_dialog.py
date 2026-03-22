# -*- coding: utf-8 -*-
"""
Pattern Configuration Dialog for Unwrap Glider tool.
Allows users to modify PatternConfig parameters before export.
"""

from PySide import QtGui, QtCore


# Parameter definitions: (attribute_name, default_value, description, value_type, unit)
# unit: 'mm' for millimeters (stored as meters internally), None for no unit
# IMPORTANT: Defaults match OtherPatternConfig (used by PlotMaker.DefaultConfig)
PATTERN_PARAMETERS = [
    # Section: General
    ("complete_glider", False, "Exporter le planeur complet. ATTENTION: True peut causer des erreurs", bool, None),
    ("debug", False, "Mode debug - affiche des lignes de construction supplémentaires", bool, None),
    ("profile_numpoints", 250, "Nombre de points pour discrétiser le profil", int, None),
    
    # Section: Layout
    ("patterns_scale", 1000, "Échelle de sortie (1000 = mètres vers millimètres)", float, None),
    ("patterns_align_dist_x", 100, "Espacement horizontal entre les pièces", float, "mm"),
    ("patterns_align_dist_y", 100, "Espacement vertical entre les pièces", float, "mm"),
    ("layout_seperate_panels", True, "Séparer les panneaux extrados/intrados", bool, None),
    
    # Section: Seam Allowances (displayed in mm, stored in meters)
    ("allowance_general", 10, "Marge de couture générale", float, "mm"),
    ("allowance_trailing_edge", 10, "Marge bord de fuite", float, "mm"),
    ("allowance_entry_open", 21, "Marge entrée d'air ouverte", float, "mm"),
    ("allowance_design", 10, "Marge coupes design", float, "mm"),
    ("allowance_diagonals", 10, "Marge diagonales", float, "mm"),
    ("allowance_parallel", 10, "Marge coupes parallèles", float, "mm"),
    ("allowance_orthogonal", 10, "Marge coupes orthogonales", float, "mm"),
    
    # Section: Diagonals and Straps
    ("drib_allowance_folds", 10, "Marge pour plis des diagonales", float, "mm"),
    ("drib_num_folds", 1, "Nombre de plis pour les diagonales", int, None),
    ("strap_num_folds", 1, "Nombre de plis pour les straps", int, None),
    
    # Section: Labels and Marks
    ("insert_attachment_point_text", True, "Afficher le nom des points d'attache", bool, None),
    ("laser_text_mode", False, "Mode laser: texte en pointillés sur calque de découpe (rouge)", bool, None),
    ("dot_spacing", 0.15, "Espacement entre points laser (relatif à la taille de lettre)", float, None),
    ("midribs", 50, "Nombre de nervures intermédiaires pour le ballooning", int, None),
]


class PatternConfigDialog(QtGui.QDialog):
    """Dialog for configuring pattern export parameters."""
    
    def __init__(self, parent=None, config=None):
        super(PatternConfigDialog, self).__init__(parent)
        self.setWindowTitle("Configuration Export Patterns")
        self.setMinimumSize(750, 550)
        
        # Store config for loading values
        self.input_config = config or {}
        
        self._setup_ui()
        self._load_values()
    
    def _setup_ui(self):
        """Create the dialog UI."""
        layout = QtGui.QVBoxLayout(self)
        
        # Description label
        desc_label = QtGui.QLabel(
            "Modifiez les paramètres ci-dessous avant l'export. "
            "Les longueurs sont en millimètres."
        )
        desc_label.setWordWrap(True)
        layout.addWidget(desc_label)
        
        # Create table
        self.table = QtGui.QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Paramètre", "Valeur", "Unité", "Description"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setColumnWidth(0, 180)
        self.table.setColumnWidth(1, 80)
        self.table.setColumnWidth(2, 40)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QtGui.QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        
        # Populate table
        self.table.setRowCount(len(PATTERN_PARAMETERS))
        self.widgets = {}  # Store widgets for value retrieval
        
        for row, (name, default, desc, vtype, unit) in enumerate(PATTERN_PARAMETERS):
            # Parameter name
            name_item = QtGui.QTableWidgetItem(name)
            name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(row, 0, name_item)
            
            # Value widget - depends on type
            if vtype == bool:
                widget = QtGui.QCheckBox()
                widget.setChecked(default)
                self.table.setCellWidget(row, 1, widget)
            elif vtype == int:
                widget = QtGui.QSpinBox()
                widget.setRange(0, 10000)
                widget.setValue(default)
                self.table.setCellWidget(row, 1, widget)
            elif vtype == float:
                widget = QtGui.QDoubleSpinBox()
                widget.setRange(0, 10000)
                widget.setDecimals(1)
                widget.setSingleStep(1)
                widget.setValue(default)
                self.table.setCellWidget(row, 1, widget)
            else:
                widget = QtGui.QLineEdit(str(default))
                self.table.setCellWidget(row, 1, widget)
            
            self.widgets[name] = (widget, vtype, default, unit)
            
            # Unit column
            unit_item = QtGui.QTableWidgetItem(unit or "")
            unit_item.setFlags(unit_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(row, 2, unit_item)
            
            # Description
            desc_item = QtGui.QTableWidgetItem(desc)
            desc_item.setFlags(desc_item.flags() & ~QtCore.Qt.ItemIsEditable)
            desc_item.setToolTip(desc)
            self.table.setItem(row, 3, desc_item)
        
        layout.addWidget(self.table)
        
        # Buttons
        button_layout = QtGui.QHBoxLayout()
        
        reset_btn = QtGui.QPushButton("Réinitialiser les valeurs par défaut")
        reset_btn.clicked.connect(self._reset_defaults)
        button_layout.addWidget(reset_btn)
        
        button_layout.addStretch()
        
        cancel_btn = QtGui.QPushButton("Annuler")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        
        ok_btn = QtGui.QPushButton("Exporter")
        ok_btn.setDefault(True)
        ok_btn.clicked.connect(self.accept)
        button_layout.addWidget(ok_btn)
        
        layout.addLayout(button_layout)
    
    def _load_values(self):
        """Load values from input_config into widgets (converting m to mm for display)."""
        for name, (widget, vtype, default, unit) in self.widgets.items():
            # Get value from config, or use default
            if name in self.input_config:
                value = self.input_config[name]
                # Convert meters to mm for display if unit is mm
                if unit == "mm" and isinstance(value, (int, float)):
                    value = value * 1000
            else:
                value = default
            
            if vtype == bool:
                widget.setChecked(bool(value))
            elif vtype in (int, float):
                widget.setValue(value)
            else:
                widget.setText(str(value))
    
    def _reset_defaults(self):
        """Reset all values to defaults."""
        for name, (widget, vtype, default, unit) in self.widgets.items():
            if vtype == bool:
                widget.setChecked(default)
            elif vtype in (int, float):
                widget.setValue(default)
            else:
                widget.setText(str(default))
    
    def get_config_dict(self):
        """Return a dictionary of all parameter values (converting mm back to meters)."""
        result = {}
        for name, (widget, vtype, default, unit) in self.widgets.items():
            if vtype == bool:
                result[name] = widget.isChecked()
            elif vtype == int:
                result[name] = widget.value()
            elif vtype == float:
                value = widget.value()
                # Convert mm back to meters if unit is mm
                if unit == "mm":
                    value = value / 1000.0
                result[name] = value
            else:
                result[name] = widget.text()
        return result


def show_pattern_config_dialog(parent=None, current_config=None):
    """
    Show the pattern configuration dialog and return the config dict.
    
    Args:
        parent: Parent widget
        current_config: Dict of current config values (in meters for lengths)
    
    Returns:
        dict or None: Configuration dict if accepted (lengths in meters), None if cancelled.
    """
    dialog = PatternConfigDialog(parent, current_config)
    if dialog.exec_() == QtGui.QDialog.Accepted:
        return dialog.get_config_dict()
    return None
