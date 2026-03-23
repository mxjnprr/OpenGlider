# -*- coding: utf-8 -*-
"""
Pattern Configuration Dialog for Unwrap Glider tool.
Allows users to modify PatternConfig parameters before export.
Bilingual FR/EN with toggle.
"""

from PySide import QtGui, QtCore


# Parameter definitions: (attribute_name, default_value, (desc_fr, desc_en), value_type, unit)
# unit: 'mm' for millimeters (stored as meters internally), None for no unit
# IMPORTANT: Defaults match OtherPatternConfig (used by PlotMaker.DefaultConfig)
PATTERN_PARAMETERS = [
    # Section: General
    ("complete_glider", False,
     ("Exporter le planeur complet. ATTENTION: True peut causer des erreurs",
      "Export complete glider. WARNING: True may cause errors"),
     bool, None),
    ("debug", False,
     ("Mode debug - affiche des lignes de construction supplémentaires",
      "Debug mode - shows extra construction lines"),
     bool, None),
    ("profile_numpoints", 250,
     ("Nombre de points pour discrétiser le profil",
      "Number of points for profile discretization"),
     int, None),
    ("midribs", 50,
     ("Nombre de nervures intermédiaires pour le ballooning",
      "Number of intermediate ribs for ballooning"),
     int, None),

    # Section: Layout
    ("patterns_scale", 1000,
     ("Échelle de sortie (1000 = mètres vers millimètres)",
      "Output scale (1000 = meters to millimeters)"),
     float, None),
    ("patterns_align_dist_x", 100,
     ("Espacement horizontal entre les pièces",
      "Horizontal spacing between parts"),
     float, "mm"),
    ("patterns_align_dist_y", 100,
     ("Espacement vertical entre les pièces",
      "Vertical spacing between parts"),
     float, "mm"),
    ("layout_seperate_panels", True,
     ("Séparer les panneaux extrados/intrados",
      "Separate upper/lower panels"),
     bool, None),

    # Section: Seam Allowances (displayed in mm, stored in meters)
    ("allowance_general", 10,
     ("Marge de couture générale",
      "General seam allowance"),
     float, "mm"),
    ("allowance_trailing_edge", 10,
     ("Marge bord de fuite",
      "Trailing edge allowance"),
     float, "mm"),
    ("allowance_entry_open", 21,
     ("Marge entrée d'air ouverte",
      "Open air entry allowance"),
     float, "mm"),
    ("allowance_design", 10,
     ("Marge coupes design",
      "Design cut allowance"),
     float, "mm"),
    ("allowance_diagonals", 10,
     ("Marge diagonales",
      "Diagonal allowance"),
     float, "mm"),
    ("allowance_parallel", 10,
     ("Marge coupes parallèles",
      "Parallel cut allowance"),
     float, "mm"),
    ("allowance_orthogonal", 10,
     ("Marge coupes orthogonales",
      "Orthogonal cut allowance"),
     float, "mm"),

    # Section: Diagonals and Straps
    ("drib_allowance_folds", 10,
     ("Marge pour plis des diagonales",
      "Diagonal fold allowance"),
     float, "mm"),
    ("drib_num_folds", 1,
     ("Nombre de plis pour les diagonales",
      "Number of folds for diagonals"),
     int, None),
    ("strap_num_folds", 1,
     ("Nombre de plis pour les straps",
      "Number of folds for straps"),
     int, None),

    # Section: Labels and Marks
    ("insert_attachment_point_text", True,
     ("Afficher le nom des points d'attache",
      "Show attachment point names"),
     bool, None),
    ("laser_text_mode", True,
     ("Mode laser: texte en pointillés sur calque de découpe (rouge)",
      "Laser mode: dotted text on cut layer (red)"),
     bool, None),
    ("dot_spacing", 0.15,
     ("Espacement entre points laser (relatif à la taille de lettre)",
      "Laser dot spacing (relative to letter size)"),
     float, None),
    ("text_inset_ratio", 0.85,
     ("Position du texte dans la marge (0=couture, 1=bord découpe)",
      "Text position in margin (0=stitch line, 1=cut edge)"),
     float, None),
]

# UI strings
_STRINGS = {
    "window_title":   ("Configuration Export Patterns", "Pattern Export Configuration"),
    "desc_label":     ("Modifiez les paramètres ci-dessous avant l'export. "
                       "Les longueurs sont en millimètres.",
                       "Edit parameters below before export. "
                       "Lengths are in millimeters."),
    "col_param":      ("Paramètre", "Parameter"),
    "col_value":      ("Valeur", "Value"),
    "col_unit":       ("Unité", "Unit"),
    "col_desc":       ("Description", "Description"),
    "reset_btn":      ("Réinitialiser les valeurs par défaut", "Reset to defaults"),
    "cancel_btn":     ("Annuler", "Cancel"),
    "export_btn":     ("Exporter", "Export"),
}


class PatternConfigDialog(QtGui.QDialog):
    """Dialog for configuring pattern export parameters."""

    def __init__(self, parent=None, config=None):
        super(PatternConfigDialog, self).__init__(parent)
        self.setMinimumSize(750, 550)

        # 0 = FR, 1 = EN
        self._lang = 0

        # Store config for loading values
        self.input_config = config or {}

        self._setup_ui()
        self._load_values()
        self._apply_language()

    # ------------------------------------------------------------------ UI
    def _setup_ui(self):
        """Create the dialog UI."""
        layout = QtGui.QVBoxLayout(self)

        # Top row: description + language toggle
        top_row = QtGui.QHBoxLayout()
        self.desc_label = QtGui.QLabel()
        self.desc_label.setWordWrap(True)
        top_row.addWidget(self.desc_label, 1)

        self.lang_btn = QtGui.QPushButton("EN")
        self.lang_btn.setFixedSize(40, 26)
        self.lang_btn.setToolTip("Switch language / Changer de langue")
        self.lang_btn.clicked.connect(self._toggle_language)
        top_row.addWidget(self.lang_btn)
        layout.addLayout(top_row)

        # Create table
        self.table = QtGui.QTableWidget()
        self.table.setColumnCount(4)
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

        for row, (name, default, descs, vtype, unit) in enumerate(PATTERN_PARAMETERS):
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
                widget.setDecimals(2)
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

            # Description (set later by _apply_language)
            desc_item = QtGui.QTableWidgetItem("")
            desc_item.setFlags(desc_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.table.setItem(row, 3, desc_item)

        layout.addWidget(self.table)

        # Buttons
        button_layout = QtGui.QHBoxLayout()

        self.reset_btn = QtGui.QPushButton()
        self.reset_btn.clicked.connect(self._reset_defaults)
        button_layout.addWidget(self.reset_btn)

        button_layout.addStretch()

        self.cancel_btn = QtGui.QPushButton()
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.cancel_btn)

        self.ok_btn = QtGui.QPushButton()
        self.ok_btn.setDefault(True)
        self.ok_btn.clicked.connect(self.accept)
        button_layout.addWidget(self.ok_btn)

        layout.addLayout(button_layout)

    # ----------------------------------------------------------- Language
    def _toggle_language(self):
        self._lang = 1 - self._lang
        self._apply_language()

    def _apply_language(self):
        lang = self._lang
        self.setWindowTitle(_STRINGS["window_title"][lang])
        self.desc_label.setText(_STRINGS["desc_label"][lang])
        self.lang_btn.setText("FR" if lang == 1 else "EN")
        self.table.setHorizontalHeaderLabels([
            _STRINGS["col_param"][lang],
            _STRINGS["col_value"][lang],
            _STRINGS["col_unit"][lang],
            _STRINGS["col_desc"][lang],
        ])
        self.reset_btn.setText(_STRINGS["reset_btn"][lang])
        self.cancel_btn.setText(_STRINGS["cancel_btn"][lang])
        self.ok_btn.setText(_STRINGS["export_btn"][lang])

        # Update description column
        for row, (name, default, descs, vtype, unit) in enumerate(PATTERN_PARAMETERS):
            desc_text = descs[lang]
            item = self.table.item(row, 3)
            if item:
                item.setText(desc_text)
                item.setToolTip(desc_text)

    # --------------------------------------------------------- Values
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
